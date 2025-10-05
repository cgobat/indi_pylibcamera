#!/usr/bin/env python3
import sys
import os
import os.path
from pathlib import Path
import subprocess
import signal
import traceback
from collections import OrderedDict
import json

from picamera2 import Picamera2

from configparser import ConfigParser

from . import __version__
from pyindi.device import (ISwitchVector, INumberVector, ITextVector, IBLOBVector,
                           ISRule, ISwitch, IPState, ISState, IBLOB, device as Device)
from .CameraControl import CameraControl


IniPath = Path(Path.home(), ".indi_pylibcamera")
IniPath.mkdir(parents=True, exist_ok=True)


def read_config():
    # iterative list of INI files to load
    configfiles = [Path(__file__, "indi_pylibcamera.ini")]
    if "INDI_PYLIBCAMERA_CONFIG_PATH" in os.environ:
        configfiles += [Path(os.environ["INDI_PYLIBCAMERA_CONFIG_PATH"], "indi_pylibcamera.ini")]
    configfiles += [IniPath / Path("indi_pylibcamera.ini")]
    configfiles += [Path(Path.cwd(), "indi_pylibcamera.ini")]
    # create config parser instance
    config = ConfigParser()
    config.read(configfiles)
    logger.debug(f"ConfigParser: {config}")
    return config


# INDI vectors with immediate actions

class LoggingVector(ISwitchVector):
    """INDI Switch vector with logging configuration

    Logging verbosity gets changed when client writes this vector.
    """

    def __init__(self, parent):
        self.parent = parent
        LoggingLevel = self.parent.config.get("driver", "LoggingLevel", fallback="Info")
        if LoggingLevel not in ["Debug", "Info", "Warning", "Error"]:
            logger.error('Parameter "LoggingLevel" in INI file has an unsupported value!')
            LoggingLevel = "Info"
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="LOGGING_LEVEL",
            sp=[
                ISwitch(name="LOGGING_DEBUG", label="Debug", state=ISState.ON if LoggingLevel == "Debug" else ISState.OFF),
                ISwitch(name="LOGGING_INFO", label="Info", state=ISState.ON if LoggingLevel == "Info" else ISState.OFF),
                ISwitch(name="LOGGING_WARN", label="Warning", value=ISState.ON if LoggingLevel == "Warning" else ISState.OFF),
                ISwitch(name="LOGGING_ERROR", label="Error", state=ISState.ON if LoggingLevel == "Error" else ISState.OFF),
            ],
            label="Logging", group="Options",
            rule=ISRule.ONEOFMANY,
        )
        self.configure_logger()

    def configure_logger(self):
        selectedLogLevel = self.get_OnSwitches()[0]
        logger.info(f'selected logging level: {selectedLogLevel}')
        if selectedLogLevel == "LOGGING_DEBUG":
            logger.setLevel(logging.DEBUG)
        elif selectedLogLevel == "LOGGING_INFO":
            logger.setLevel(logging.INFO)
        elif selectedLogLevel == "LOGGING_WARN":
            logger.setLevel(logging.WARN)
        else:
            logger.setLevel(logging.ERROR)

    def set_byClient(self, values: dict):
        """called when vector gets set by client
        special version for changing logging level

        Args:
            values: dict(propertyName: value) of values to set
        """
        logger.debug(f"logging level action: {values}")
        for k, v in values.items():
            for ele in self.elements:
                if ele.name == k:
                    ele.value = v
                    break
            else:
                raise KeyError(f"ISwitch '{k}' not valid.")
        self.configure_logger()


class ConnectionVector(ISwitchVector):
    """INDI Switch vector with "Connect" and "Disconnect" buttons

    Camera gets connected or disconnected when client writes this vector.
    """

    def __init__(self, parent):
        self.parent=parent
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="CONNECTION",
            sp=[
                ISwitch(name="CONNECT", label="Connect", state=ISState.OFF),
                ISwitch(name="DISCONNECT", label="Disconnect", state=ISState.ON),
            ],
            label="Connection", group="Main Control",
            rule=ISRule.ONEOFMANY, is_savable=False,
        )

    def set_byClient(self, values: dict):
        """called when vector gets set by client
        special version for connect/disconnect actions

        Args:
            values: dict(propertyName: value) of values to set
        """
        logger.debug(f"connect/disconnect action: {values}")
        self.message = self.update_SwitchStates(values=values)
        # send updated property values
        if len(self.message) > 0:
            self.state = IPState.ALERT
            self.send_setVector()
            self.message = ""
            return
        self.state = IPState.BUSY
        self.send_setVector()
        if self.get_OnSwitches()[0] == "CONNECT":
            if self.parent.openCamera():
                self.state = IPState.OK
            else:
                self.state = IPState.ALERT
        else:
            self.parent.closeCamera()
            self.state = IPState.OK
        self.send_setVector()


class ExposureVector(INumberVector):
    """INDI Number vector for exposure time

    Exposure gets started when client writes this vector.
    """
    def __init__(self, parent, min_exp, max_exp):
        self.parent = parent
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="CCD_EXPOSURE",
            np=[
                INumber(name="CCD_EXPOSURE_VALUE", label="Duration (s)", min=min_exp / 1e6, max=max_exp / 1e6,
                        step=0.001, value=1.0, format="%.3f"),
            ],
            label="Expose", group="Main Control", is_savable=False,
        )

    def set_byClient(self, values: dict):
        """called when vector gets set by client
        special version for exposure actions

        Args:
            values: dict(propertyName: value) of values to set
        """
        errmsgs = []
        for propName, value in values.items():
            errmsg = self[propName].set_byClient(value)
            if len(errmsg) > 0:
                errmsgs.append(errmsg)
        # send updated property values
        if len(errmsgs) > 0:
            self.state = IPState.ALERT
            self.message = "; ".join(errmsgs)
            self.send_setVector()
            self.message = ""
            return
        else:
            self.state = IPState.OK
        self.state = IPState.BUSY
        self.send_setVector()
        self.parent.startExposure(exposuretime=self["CCD_EXPOSURE_VALUE"].value)


class RawFormatVector(ISwitchVector):
    """INDI Switch vector to select raw format

    For some cameras the raw format changes binning.
    """

    def __init__(self, parent: Device, CameraThread: CameraControl, do_CameraAdjustments: bool):
        self.parent = parent
        self.CameraThread = CameraThread
        self.do_CameraAdjustments = do_CameraAdjustments
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="RAW_FORMAT",
            sp=[
                ISwitch(name=f'RAWFORMAT{i}', label=rm["label"], state=ISState.ON if i == 0 else ISState.OFF)
                for i, rm in enumerate(self.CameraThread.RawModes)
            ],
            label="Raw format", group="Image Settings",
            rule=ISRule.ONEOFMANY,
        )

    def get_SelectedRawMode(self):
        return self.CameraThread.RawModes[self.get_OnSwitchesIdxs()[0]]

    def update_Binning(self):
        if self.do_CameraAdjustments:
            if self.parent["CCD_CAPTURE_FORMAT"]["INDI_RAW"].value == ISState.ON:
                # set binning according to raw format
                selectedRawMode = self.CameraThread.RawModes[self.get_OnSwitchesIdxs()[0]]
                binning = selectedRawMode["binning"]
            else:
                # processed frames are all with 1x1 binning
                binning = (1, 1)
            self.parent.IUUpdate(self.parent.device, "CCD_BINNING", values=binning, names=["HOR_BIN", "VER_BIN"], Set=True)

    def set_byClient(self, values: dict):
        """called when vector gets set by client
        special version for changing raw mode depending binning

        Args:
            values: dict(propertyName: value) of values to set
        """
        super().set_byClient(values=values)
        self.update_Binning()


class RawProcessedVector(ISwitchVector):
    """INDI Switch vector to select raw or processed format

    Processed formats have allways binning = (1,1).
    """

    def __init__(self, parent, CameraThread):
        self.parent=parent
        if len(CameraThread.RawModes) > 0:
            elements = [
                ISwitch(name="INDI_RAW", label="RAW", state=ISState.ON),
                ISwitch(name="INDI_RGB", label="RGB", state=ISState.OFF),
            ]
        else:
            elements = [
                ISwitch(name="INDI_RGB", label="RGB", state=ISState.ON),
            ]
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="CCD_CAPTURE_FORMAT",
            sp=elements,
            label="Format", group="Image Settings",
            rule=ISRule.ONEOFMANY,
        )

    def set_byClient(self, values: dict):
        """called when vector gets set by client
        special version for changing frame type depending binning

        Args:
            values: dict(propertyName: value) of values to set
        """
        super().set_byClient(values=values)
        self.parent.knownVectors["RAW_FORMAT"].update_Binning()


class BinningVector(INumberVector):
    """INDI Number vector for binning setting

    Binning is related to raw modes: when changing binning the raw mode must also be changed.
    """
    def __init__(self, parent, CameraThread, do_CameraAdjustments):
        self.parent = parent
        self.CameraThread = CameraThread
        self.do_CameraAdjustments = do_CameraAdjustments
        # make dict: binning-->index in CameraThread.RawModes
        self.RawBinningModes = dict()
        for i, rm in enumerate(self.CameraThread.RawModes):
            if not rm["binning"] in self.RawBinningModes:
                self.RawBinningModes[rm["binning"]] = i
        # determine max binning values
        if self.do_CameraAdjustments:
            max_HOR_BIN = 1
            max_VER_BIN = 1
            for binning in self.RawBinningModes.keys():
                max_HOR_BIN = max(max_HOR_BIN, binning[0])
                max_VER_BIN = max(max_VER_BIN, binning[1])
        else:
            max_HOR_BIN = 10
            max_VER_BIN = 10
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="CCD_BINNING",
            np=[
                INumber(name="HOR_BIN", label="X", min=1, max=max_HOR_BIN, step=1, value=1, format="%2.0f"),
                INumber(name="VER_BIN", label="Y", min=1, max=max_VER_BIN, step=1, value=1, format="%2.0f"),
            ],
            label="Binning", group="Image Settings",
            state=IPState.IDLE, perm=IPerm.RW,
        )

    def set_byClient(self, values: dict):
        """called when vector gets set by client
        special version for binning

        Args:
            values: dict(propertyName: value) of values to set
        """
        if self.do_CameraAdjustments:
            # allowed binning depends on CCD_CAPTURE_FORMAT (raw or processed) and raw mode
            bestRawIdx = 1
            if self.parent["CCD_CAPTURE_FORMAT"]["INDI_RAW"].value == ISState.ON:
                # select best matching frame type
                bestError = 1000000
                for binning, RawIdx in self.RawBinningModes.items():
                    err = abs(float(values["HOR_BIN"]) - binning[0]) + abs(float(values["VER_BIN"]) - binning[1])
                    if err < bestError:
                        bestError = err
                        bestRawIdx = RawIdx
            # set fitting raw mode and matching binning
            self.parent["RAW_FORMAT"].set_byClient({f'RAWFORMAT{bestRawIdx}': ISState.ON})
        else:
            super().set_byClient(values=values)


class SnoopingVector(ITextVector):
    """INDI Text vector with other devices to snoop
    """

    def __init__(self, parent):
        self.parent = parent
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="ACTIVE_DEVICES",
            # empty values mean do not snoop
            tp=[
                IText(name="ACTIVE_TELESCOPE", label="Telescope", value=""),
                #IText(name="ACTIVE_ROTATOR", label="Rotator", value=""),
                #IText(name="ACTIVE_FOCUSER", label="Focuser", value=""),
                #IText(name="ACTIVE_FILTER", label="Filter", value=""),
                #IText(name="ACTIVE_SKYQUALITY", label="Sky Quality", value=""),
            ],
            label="Snoop devices", group="Snooping",
        )

    def set_byClient(self, values: dict):
        """called when vector gets set by client
        special version for activating snooping

        Args:
            values: dict(propertyName: value) of values to set
        """
        super().set_byClient(values=values)
        if self.parent.config.getboolean("driver", "DoSnooping", fallback=True):
            for k, v in values.items():
                if k == "ACTIVE_TELESCOPE":
                    self.parent.stop_Snooping(kind="ACTIVE_TELESCOPE")
                    if v != "":
                        self.parent.start_Snooping(
                            kind="ACTIVE_TELESCOPE",
                            device=v,
                            names=[
                                "GEOGRAPHIC_COORD",  # observer site coordinates
                                "EQUATORIAL_EOD_COORD",
                                "EQUATORIAL_COORD",
                                "TELESCOPE_PIER_SIDE",
                                "TELESCOPE_INFO",
                            ]
                        )


class FitsHeaderVector(ITextVector):
    """INDI Text vector with other devices to snoop
    """

    def __init__(self, parent):
        self.parent = parent
        self.FitsHeader = OrderedDict()
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="FITS_HEADER",
            # empty values mean do not snoop
            tp=[
                IText(name="KEYWORD_NAME", label="Name", value=""),
                IText(name="KEYWORD_VALUE", label="Value", value=""),
                IText(name="KEYWORD_COMMENT", label="Comment", value=""),
            ],
            label="FITS Header", group="General Info", perm=IPerm.WO, is_savable=False,
        )

    def set_byClient(self, values: dict):
        """called when vector gets set by client
        special version for activating snooping

        Args:
            values: dict(propertyName: value) of values to set
        """
        super().set_byClient(values=values)
        self.FitsHeader[values["KEYWORD_NAME"]] = (values["KEYWORD_VALUE"], values["KEYWORD_COMMENT"])


class DoSnoopingVector(ISwitchVector):
    """INDI SwitchVector to enable/disable snooping; gets initialized from config file
    """

    def __init__(self, parent):
        self.parent = parent
        config_DoSnooping = self.parent.config.getboolean("driver", "DoSnooping", fallback=True)
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="DO_SNOOPING",
            sp=[
                ISwitch(name="SNOOP", label="Yes", state=ISState.ON if config_DoSnooping else ISState.OFF),
                ISwitch(name="NO_SNOOP", label="No", state=ISState.OFF if config_DoSnooping else ISState.ON),
            ],
            label="Do snooping", group="Snooping",
            rule=ISRule.ONEOFMANY,
        )


class AbortVector(ISwitchVector):
    """INDI SwitchVector to abort exposure
    """

    def __init__(self, parent):
        self.parent = parent
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="CCD_ABORT_EXPOSURE",
            sp=[
                ISwitch(name="ABORT", label="Abort", value=ISState.OFF),
            ],
            label="Abort", group="Main Control",
            rule=ISRule.ATMOST1, is_savable=False,
        )

    def set_byClient(self, values: dict):
        super().set_byClient(values = values)
        if self.get_OnSwitches()[0] == "ABORT":
            self.parent.IUUpdate()
            self.parent.setVector("CCD_FAST_COUNT", "FRAMES", value=0, state=IPState.OK)
            self.parent.setVector("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", value=0, state=IPState.OK)
            self.parent.abortExposure()
            self.parent.setVector("CCD_ABORT_EXPOSURE", "ABORT", value=ISState.OFF, state=IPState.OK)

class PrintSnoopedValuesVector(ISwitchVector):
    """Button that prints all snooped values as INFO in log
    """

    def __init__(self, parent):
        self.parent = parent
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="PRINT_SNOOPED_VALUES",
            sp=[
                ISwitch(name="PRINT_SNOOPED", label="Print", state=ISState.OFF),
            ],
            label="Print snooped values", group="Snooping",
            rule=ISRule.ATMOST1, is_savable=False,
        )

    def set_byClient(self, values: dict):
        """called when vector gets set by client
        special version to print snooped values

        Args:
            values: dict(propertyName: value) of values to set
        """
        logger.info(f'Snooped values: {str(self.parent.SnoopingManager)}')
        self.state = IPState.OK
        self.send_setVector()


class ConfigProcessVector(ISwitchVector):
    """INDI Switch vector to save and load configurations
    """

    def __init__(self, parent):
        self.parent=parent
        super().__init__(
            device=self.parent.device, timestamp=self.parent.timestamp, name="CONFIG_PROCESS",
            sp=[
                ISwitch(name="CONFIG_LOAD", label="Load", state=ISState.OFF),
                ISwitch(name="CONFIG_SAVE", label="Save", state=ISState.OFF),
                ISwitch(name="CONFIG_DEFAULT", label="Default", state=ISState.OFF),
                ISwitch(name="CONFIG_PURGE", label="Purge", state=ISState.OFF),
            ],
            label="Configuration", group="Options",
            rule=ISRule.ATMOST1, is_savable=False,
        )

    def set_byClient(self, values: dict):
        """called when vector gets set by client
        special version for saving and loading configurations

        Args:
            values: dict(propertyName: value) of values to set
        """
        super().set_byClient(values=values)
        config_filename = IniPath / f'{self.parent.knownVectors["APPLY_CONFIG"].get_OnSwitches()[0]}.json'
        actions = self.get_OnSwitches()
        if len(actions) > 0:
            action = actions[0]
            if action == "CONFIG_LOAD":
                if config_filename.exists():
                    logger.info(f'loading configuration from {config_filename}')
                    with open(config_filename, "r") as fh:
                        states = json.load(fh)
                    for vector in states:
                        if vector["name"] in self.parent.knownVectors:
                            self.parent.knownVectors[vector["name"]].set_byClient(vector["values"])
                        else:
                            logger.warning(f'Ignoring unknown vector {vector["name"]}!')
                else:
                    logger.info(f'configuration {config_filename} does not exist')
            elif action == "CONFIG_SAVE":
                logger.info(f'saving configuration in {config_filename}')
                states = list()
                for vector in self.parent.knownVectors:
                    state = vector.save()
                    if state is not None:
                        states.append(state)
                with open(config_filename, "w") as fh:
                    json.dump(states, fh, indent=4)
            elif action == "CONFIG_DEFAULT":
                logger.info(f'restoring driver defaults')
                for vector in self.parent.knownVectors:
                    vector.restore_DriverDefault()
            else:  # action == "CONFIG_PURGE"
                logger.info(f'deleting configuration {config_filename}')
                config_filename.unlink(missing_ok=True)
        # set all buttons Off again
        super().set_byClient(values={element.name: ISState.OFF for element in self.elements})



def kill_oldDriver():
    """test if another instance of driver is already running and kill it

    This relies on the output of "ps ax" system command.
    Alternative would be 3rd party library psutil which may need to be installed.
    """
    my_PID = os.getpid()
    logger.info(f'my PID: {my_PID}')
    my_fileName = os.path.basename(__file__)[:-3]
    logger.info(f'my file name: {my_fileName}')
    ps_ax = subprocess.check_output(["ps", "ax"]).decode(sys.stdout.encoding)
    ps_ax = ps_ax.split("\n")
    pids_oldDriver = []
    for processInfo in ps_ax:
        if ("python" in processInfo) and (my_fileName in processInfo):
            PID = int(processInfo.strip().split(" ", maxsplit=1)[0])
            if PID != my_PID:
                logger.info(f'found old driver with PID {PID} ({processInfo})')
                pids_oldDriver.append(PID)
    for pid_oldDriver in pids_oldDriver:
        try:
            os.kill(pid_oldDriver, signal.SIGKILL)
        except ProcessLookupError:
            # process does not exist anymore
            pass
        except PermissionError:
            # not allowed to kill
            logger.error(f'Do not have permission to kill old driver with PID {pid_oldDriver}.')


# the device driver

class indi_pylibcamera(Device):
    """camera driver using libcamera
    """

    def __init__(self, config=None):
        """constructor

        Args:
            config: driver configuration
        """
        kill_oldDriver()
        super().__init__(name=config.get("driver", "DeviceName", fallback="indi_pylibcamera"), config=config)
        self.timestamp = self.config.getboolean("driver", "SendTimeStamps", fallback=False)
        # send logging messages to client
        enable_Logging(device=self.device, timestamp=self.timestamp)
        # camera
        self.CameraThread = CameraControl(
            parent=self,
            config=config,
        )
        # handle SIGINT and SIGTERM gracefully
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        # get connected cameras
        cameras = Picamera2.global_camera_info()
        logger.info(f'found cameras: {cameras}')
        # use Id as unique camera identifier
        self.Cameras = [c["Id"] for c in cameras]
        # INDI vector names only available with connected camera
        self.CameraVectorNames = []
        # INDI general vectors
        self.IDDef(
            ISwitchVector(
                device=self.device, timestamp=self.timestamp, name="CAMERA_SELECTION",
                sp=[
                    ISwitch(
                        name=f'CAM{i}',
                        state=ISState.ON if i == 0 else ISState.OFF,
                        label=self.Cameras[i]
                    ) for i in range(len(self.Cameras))
                ],
                label="Camera", group="Main Control",
                rule=ISRule.ONEOFMANY, is_savable=False,
            )
        )
        self.IDDef(
            ConnectionVector(parent=self),
        )
        self.IDDef(
            ITextVector(
                device=self.device, timestamp=self.timestamp, name="DRIVER_INFO",
                tp=[
                    IText(name="DRIVER_NAME", label="Name", text=self.device),
                    IText(name="DRIVER_EXEC", label="Exec", text=sys.argv[0]),
                    IText(name="DRIVER_VERSION", label="Version", text=__version__),
                    IText(name="DRIVER_INTERFACE", label="Interface", text="2"),  # This is a CCD!
                ],
                label="Driver Info", group="General Info",
                perm=IPerm.RO, is_savable=False,
            )
        )
        self.IDDef(
            LoggingVector(parent=self),
        )
        self.IDDef(
            INumberVector(
                device=self.device, timestamp=self.timestamp, name="POLLING_PERIOD",
                np=[
                    INumber(name="PERIOD_MS", label="Period (ms)", min=10, max=600000,
                            step=1000, value=1000, format="%.f"),
                ],
                label="Polling", group="Options",
                perm=IPerm.RW,
            ),
        )
        # snooping
        self.IDDef(
            INumberVector(
                device=self.device, timestamp=self.timestamp, name="GEOGRAPHIC_COORD",
                np=[
                    INumber(name="LAT", label="Lat (dd:mm:ss.s)", min=-90, max=90, step=0, value=0, format="%012.8m"),
                    INumber(name="LONG", label="Lon (dd:mm:ss.s)", min=0, max=360, step=0, value=0, format="%012.8m"),
                    INumber(name="ELEV", label="Elevation (m)", min=-200, max=10000, step=0, value=0, format="%g"),
                ],
                label="Scope Location", group="Snooping",
                perm=IPerm.RW, is_savable=False,
            ),
        )
        self.IDDef(
            INumberVector(
                device=self.device, timestamp=self.timestamp, name="EQUATORIAL_EOD_COORD",
                np=[
                    INumber(name="RA", label="RA (hh:mm:ss)", min=0, max=24, step=0, value=0, format="%010.6m"),
                    INumber(name="DEC", label="DEC (dd:mm:ss)", min=-90, max=90, step=0, value=0, format="%010.6m"),
                ],
                label="Eq. Coordinates", group="Snooping",
                perm=IPerm.RW, is_savable=False,
            ),
        )
        # TODO: "EQUATORIAL_COORD" (J2000 coordinates from mount) are not used!
        if False:
            self.IDDef(
                INumberVector(
                    device=self.device, timestamp=self.timestamp, name="EQUATORIAL_COORD",
                    np=[
                        INumber(name="RA", label="RA (hh:mm:ss)", min=0, max=24, step=0, value=0, format="%010.6m"),
                        INumber(name="DEC", label="DEC (dd:mm:ss)", min=-90, max=90, step=0, value=0, format="%010.6m"),
                    ],
                    label="Eq. J2000 Coordinates", group="Snooping",
                    perm=IPerm.RW, is_savable=False,
                ),
            )
        self.IDDef(
            ISwitchVector(
                device=self.device, timestamp=self.timestamp, name="TELESCOPE_PIER_SIDE",
                sp=[
                    ISwitch(name="PIER_WEST", state=ISState.ON, label="West (pointing east)"),
                    ISwitch(name="PIER_EAST", state=ISState.OFF, label="East (pointing west)"),
                ],
                label="Pier Side", group="Snooping",
                rule=ISRule.ONEOFMANY, is_savable=False,
            )
        )
        self.IDDef(
            INumberVector(
                device=self.device, timestamp=self.timestamp, name="TELESCOPE_INFO",
                np=[
                    INumber(name="TELESCOPE_APERTURE", label="Aperture (mm)", min=10, max=5000, step=0, value=0, format="%g"),
                    INumber(name="TELESCOPE_FOCAL_LENGTH", label="Focal Length (mm)", min=10, max=10000, step=0, value=0, format="%g"),
                    INumber(name="GUIDER_APERTURE", label="Guider Aperture (mm)", min=10, max=5000, step=0, value=0, format="%g"),
                    INumber(name="GUIDER_FOCAL_LENGTH", label="Guider Focal Length (mm)", min=10, max=10000, step=0, value=0, format="%g"),
                ],
                label="Scope Properties", group="Snooping",
                perm=IPerm.RW,
            ),
        )
        self.IDDef(
            ISwitchVector(
                device=self.device, timestamp=self.timestamp, name="CAMERA_LENS",
                sp=[
                    ISwitch(name="PRIMARY_LENS", state=ISState.ON, label="Primary"),
                    ISwitch(name="GUIDER_LENS", state=ISState.OFF, label="Guide"),
                ],
                label="Camera lens", group="Snooping",
                rule=ISRule.ONEOFMANY,
            )
        )
        self.IDDef(
            DoSnoopingVector(parent=self, ),
        )
        self.IDDef(
            SnoopingVector(parent=self,),
        )
        if self.config.getboolean("driver", "PrintSnoopedValuesButton", fallback=False):
            self.IDDef(
                PrintSnoopedValuesVector(parent=self, ),
            )

    def ISNewNumber(self, device, name, values, names):
        ...

    def ISNewText(self, device, name, values, names):
        ...

    def ISNewSwitch(self, device, name, values, names):
        ...

    def ISNewBLOB(self, device, name, values, names):
        ...

    def exit_gracefully(self, sig, frame):
        """exit driver on system signals
        """
        logger.info("Exit triggered by SIGINT or SIGTERM")
        self.CameraThread.closeCamera()
        traceback.print_stack(frame)
        sys.exit(0)

    def closeCamera(self):
        """close camera and tell client to remove camera vectors from GUI
        """
        self.CameraThread.closeCamera()
        for n in self.CameraVectorNames:
            self.checkout(n)
        self.CameraVectorNames = []

    def openCamera(self):
        """ opens camera, reads camera properties and still configurations, updates INDI properties
        """
        #
        CameraSel = self.knownVectors["CAMERA_SELECTION"].get_OnSwitchesIdxs()
        if len(CameraSel) < 1:
            return False
        CameraIdx = CameraSel[0]
        logger.info(f'connecting to camera {self.Cameras[CameraIdx]}')
        self.closeCamera()
        self.CameraThread.openCamera(CameraIdx)
        # update INDI properties
        self.IDDef(
            ITextVector(
                device=self.device, timestamp=self.timestamp, name="CAMERA_INFO",
                tp=[
                    IText(name="CAMERA_MODEL", label="Model", text=self.CameraThread.getProp("Model")),
                    IText(name="CAMERA_PIXELARRAYSIZE", label="Pixel array size", text=str(self.CameraThread.getProp("PixelArraySize"))),
                    IText(name="CAMERA_PIXELARRAYACTIVEAREA", label="Pixel array active area", text=str(self.CameraThread.getProp("PixelArrayActiveAreas"))),
                    IText(name="CAMERA_UNITCELLSIZE", label="Pixel size", text=str(self.CameraThread.getProp("UnitCellSize"))),
                ],
                label="Camera Info", group="General Info",
                state=IPState.OK, perm=IPerm.RO, is_savable=False,
            ),
        )
        self.CameraVectorNames.append("CAMERA_INFO")
        # allow to select raw or processed frame
        self.IDDef(
            RawProcessedVector(parent=self, CameraThread=self.CameraThread),
        )
        self.CameraVectorNames.append("CCD_CAPTURE_FORMAT")
        # raw frame types
        self.IDDef(
            RawFormatVector(
                parent=self,
                CameraThread=self.CameraThread,
                do_CameraAdjustments=self.config.getboolean("driver", "CameraAdjustments", fallback=True),
            ),
        )
        self.CameraVectorNames.append("RAW_FORMAT")
        #
        self.IDDef(
            INumberVector(
                device=self.device, timestamp=self.timestamp, name="CCD_PROCFRAME",
                np=[
                    INumber(name="WIDTH", label="Width", min=1, max=self.CameraThread.getProp("PixelArraySize")[0],
                            step=0, value=self.CameraThread.getProp("PixelArraySize")[0], format="%4.0f"),
                    INumber(name="HEIGHT", label="Height", min=1, max=self.CameraThread.getProp("PixelArraySize")[1],
                            step=0, value=self.CameraThread.getProp("PixelArraySize")[1], format="%4.0f"),
                ],
                label="RGB format", group="Image Settings",
                perm=IPerm.RW,
            ),
        )
        self.CameraVectorNames.append("CCD_PROCFRAME")
        # camera controls
        self.addCameraControls()
        #
        self.IDDef(
            ExposureVector(parent=self, min_exp=self.CameraThread.min_ExposureTime, max_exp=self.CameraThread.max_ExposureTime),
        )
        self.CameraVectorNames.append("CCD_EXPOSURE")
        #
        self.IDDef(
            AbortVector(parent=self),
        )
        self.CameraVectorNames.append("CCD_ABORT_EXPOSURE")
        # CCD_FRAME defines a cropping area in the frame.
        self.IDDef(
            INumberVector(
                device=self.device, timestamp=self.timestamp, name="CCD_FRAME",
                np=[
                    # ATTENTION: max must be >0
                    INumber(name="X", label="Left", min=0, max=self.CameraThread.getProp("PixelArraySize")[0], step=0, value=0, format="%4.0f"),
                    INumber(name="Y", label="Top", min=0, max=self.CameraThread.getProp("PixelArraySize")[1], step=0, value=0, format="%4.0f"),
                    INumber(name="WIDTH", label="Width", min=1, max=self.CameraThread.getProp("PixelArraySize")[0],
                            step=0, value=self.CameraThread.getProp("PixelArraySize")[0], format="%4.0f"),
                    INumber(name="HEIGHT", label="Height", min=1, max=self.CameraThread.getProp("PixelArraySize")[1],
                            step=0, value=self.CameraThread.getProp("PixelArraySize")[1], format="%4.0f"),
                ],
                label="Frame", group="Image Info",
                perm=IPerm.RO, is_savable=False,  # TODO: make it savable after implementing frame cropping
            ),
        )
        self.CameraVectorNames.append("CCD_FRAME")
        # TODO: implement functionality
        self.IDDef(
            ISwitchVector(
                device=self.device, timestamp=self.timestamp, name="CCD_FRAME_RESET",
                sp=[
                    ISwitch(name="RESET", label="Reset", state=ISState.OFF),
                ],
                label="Frame Values", group="Image Settings",
                rule=ISRule.ONEOFMANY, perm=IPerm.WO, is_savable=False,
            ),
        )
        self.CameraVectorNames.append("CCD_FRAME_RESET")
        #
        self.IDDef(
            BinningVector(
                parent=self,
                CameraThread=self.CameraThread,
                do_CameraAdjustments=self.config.getboolean("driver", "CameraAdjustments", fallback=True),
            ),
        )
        self.CameraVectorNames.append("CCD_BINNING")
        #
        self.IDDef(
            FitsHeaderVector(parent=self,),
        )
        self.CameraVectorNames.append("FITS_HEADER")
        #
        self.IDDef(
            INumberVector(
                device=self.device, timestamp=self.timestamp, name="CCD_TEMPERATURE",
                np=[
                    INumber(name="CCD_TEMPERATURE_VALUE", label="Temperature (C)", min=-50, max=50, step=0, value=0, format="%5.2f"),
                ],
                label="Temperature", group="Main Control",
                state=IPState.IDLE, perm=IPerm.RO, is_savable=False,
            ),
        )
        self.CameraVectorNames.append("CCD_TEMPERATURE")
        #
        self.IDDef(
            INumberVector(
                device=self.device, timestamp=self.timestamp, name="CCD_INFO",
                np=[
                    INumber(name="CCD_MAX_X", label="Max. Width", min=1, max=1000000, step=0,
                            value=self.CameraThread.getProp("PixelArraySize")[0], format="%.f"),
                    INumber(name="CCD_MAX_Y", label="Max. Height", min=1, max=1000000, step=0,
                            value=self.CameraThread.getProp("PixelArraySize")[1], format="%.f"),
                    INumber(name="CCD_PIXEL_SIZE", label="Pixel size (um)", min=0, max=1000, step=0,
                            value=max(self.CameraThread.getProp("UnitCellSize")) / 1e3, format="%.2f"),
                    INumber(name="CCD_PIXEL_SIZE_X", label="Pixel size X", min=0, max=1000, step=0,
                            value=self.CameraThread.getProp("UnitCellSize")[0] / 1e3, format="%.2f"),
                    INumber(name="CCD_PIXEL_SIZE_Y", label="Pixel size Y", min=0, max=1000, step=0,
                            value=self.CameraThread.getProp("UnitCellSize")[1] / 1e3, format="%.2f"),
                    INumber(name="CCD_BITSPERPIXEL", label="Bits per pixel", min=0, max=1000, step=0,
                            # using value of first raw mode or 8 if no raw mode available, TODO: is that right?
                            value=8 if len(self.CameraThread.RawModes) < 1 else self.CameraThread.RawModes[0]["bit_depth"], format="%.f"),
                ],
                label="CCD Information", group="Image Info",
                state=IPState.IDLE, perm=IPerm.RO, is_savable=False,
            ),
        )
        self.CameraVectorNames.append("CCD_INFO")
        #
        self.IDDef(
            ISwitchVector(
                device=self.device, timestamp=self.timestamp, name="CCD_COMPRESSION",
                sp=[
                    # The CCD Simulator has here other names which are not conform to protocol specification:
                    # INDI_ENABLED and INDI_DISABLED
                    #ISwitch(name="INDI_ENABLED", label="Compressed", state=ISState.OFF),
                    #ISwitch(name="INDI_DISABLED", label="Uncompressed", state=ISState.ON),
                    # Specification conform names are: CCD_COMPRESS and CCD_RAW
                    ISwitch(name="CCD_COMPRESS", label="Compressed", state=ISState.OFF),
                    ISwitch(name="CCD_RAW", label="Uncompressed", state=ISState.ON),
                ],
                label="Image compression", group="Image Settings",
                rule=ISRule.ONEOFMANY,
            ),
        )
        self.CameraVectorNames.append("CCD_COMPRESSION")
        # the image BLOB
        self.IDDef(
            IBlobVector(
                device=self.device, timestamp=self.timestamp, name="CCD1",
                bp=[
                    IBlob(name="CCD1", label="Image"),
                ],
                label="Image Data", group="Image Info",
                state=IPState.OK, perm=IPerm.RO, is_savable=False,
            ),
        )
        self.CameraVectorNames.append("CCD1")
        #
        self.IDDef(
            ISwitchVector(
                device=self.device, timestamp=self.timestamp, name="CCD_FRAME_TYPE",
                sp=[
                    ISwitch(name="FRAME_LIGHT", label="Light", state=ISState.ON),
                    ISwitch(name="FRAME_BIAS", label="Bias", state=ISState.OFF),
                    ISwitch(name="FRAME_DARK", label="Dark", state=ISState.OFF),
                    ISwitch(name="FRAME_FLAT", label="Flat", state=ISState.OFF),
                ],
                label="Frame Type", group="Image Settings",
                rule=ISRule.ONEOFMANY,
            ),
        )
        self.CameraVectorNames.append("CCD_FRAME_TYPE")
        #
        self.IDDef(
            ISwitchVector(
                device=self.device, timestamp=self.timestamp, name="UPLOAD_MODE",
                sp=[
                    ISwitch(name="UPLOAD_CLIENT", label="Client", state=ISState.ON),
                    ISwitch(name="UPLOAD_LOCAL", label="Local", state=ISState.OFF),
                    ISwitch(name="UPLOAD_BOTH", label="Both", state=ISState.OFF),
                ],
                label="Upload", group="Options",
                rule=ISRule.ONEOFMANY,
            ),
        )
        self.CameraVectorNames.append("UPLOAD_MODE")
        #
        self.IDDef(
            ITextVector(
                device=self.device, timestamp=self.timestamp, name="UPLOAD_SETTINGS",
                tp=[
                    IText(name="UPLOAD_DIR", label="Dir", text=str(Path.home())),
                    IText(name="UPLOAD_PREFIX", label="Prefix", text="IMAGE_XXX"),
                ],
                label="Upload Settings", group="Options",
            ),
        )
        self.CameraVectorNames.append("UPLOAD_SETTINGS")
        #
        self.IDDef(
            ISwitchVector(
                device=self.device, timestamp=self.timestamp, name="CCD_FAST_TOGGLE",
                sp=[
                    ISwitch(name="INDI_ENABLED", label="Enabled", state=ISState.OFF),
                    ISwitch(name="INDI_DISABLED", label="Disabled", state=ISState.ON),
                ],
                label="Fast Exposure", group="Main Control",
                rule=ISRule.ONEOFMANY,
            ),
        )
        self.CameraVectorNames.append("CCD_FAST_TOGGLE")
        # need also CCD_FAST_COUNT for fast exposure
        self.IDDef(
            INumberVector(
                device=self.device, timestamp=self.timestamp, name="CCD_FAST_COUNT",
                np=[
                    INumber(name="FRAMES", label="Frames", min=0, max=100000, step=1, value=1, format="%.f"),
                ],
                label="Fast Count", group="Main Control", is_savable=False,
            ),
        )
        self.CameraVectorNames.append("CCD_FAST_COUNT")
        #
        self.IDDef(
            INumberVector(
                device=self.device, timestamp=self.timestamp, name="CCD_GAIN",
                np=[
                    INumber(name="GAIN", label="Analog Gain", min=self.CameraThread.min_AnalogueGain,
                            max=self.CameraThread.max_AnalogueGain, step=0.1,
                            value=self.CameraThread.max_AnalogueGain, format="%.1f"),
                ],
                label="Gain", group="Main Control",
            ),
        )
        self.CameraVectorNames.append("CCD_GAIN")
        #
        # configuration save and load
        self.IDDef(
            ISwitchVector(
                device=self.device, timestamp=self.timestamp, name="APPLY_CONFIG",
                sp=[
                    ISwitch(name=f"CONFIG{i}", label=f"Config #{i}", state=ISState.ON if i == 1 else ISState.OFF)
                    for i in range(1, 7)
                ],
                label="Configs", group="Options",
                rule=ISRule.ONEOFMANY,
            ),
        )
        self.CameraVectorNames.append("APPLY_CONFIG")
        #
        self.IDDef(
            ITextVector(
                device=self.device, timestamp=self.timestamp, name="CONFIG_NAME",
                tp=[
                    IText(name="CONFIG_NAME", label="Config Name", text=""),
                ],
                label="Configuration Name", group="Options",
            ),
        )
        self.CameraVectorNames.append("CONFIG_NAME")
        #
        self.IDDef(
            ConfigProcessVector(parent=self,),
        )
        self.CameraVectorNames.append("CONFIG_PROCESS")
        #
        # Maybe needed: CCD_CFA
        # self.IDDef(
        #     ITextVector(
        #         device=self.device, timestamp=self.timestamp, name="CCD_CFA",
        #         tp=[
        #             IText(name="CFA_OFFSET_X", label="Offset X", text="0"),
        #             IText(name="CFA_OFFSET_Y", label="Offset Y", text="0"),
        #             IText(name="CFA_TYPE", label="Type", text=self.raw_mode["format"][1:].rstrip("0123456789")),
        #         ],
        #         label="Color filter array", group="Image Info",
        #         state=IPState.IDLE, perm=IPerm.RO,
        #     ),
        # )
        # self.CameraVectorNames.append("CCD_CFA")
        #
        # Maybe needed: CCD_COOLER
        #
        # needed for field solver?
        # self.IDDef(
        #     ISwitchVector(
        #         device=self.device, timestamp=self.timestamp, name="TELESCOPE_TYPE",
        #         sp=[
        #             ISwitch(name="TELESCOPE_PRIMARY", label="Primary", value=ISState.ON),
        #             ISwitch(name="TELESCOPE_GUIDE", label="Guide", value=ISState.OFF),
        #         ],
        #         label="Telescope", group="Options",
        #         rule=ISRule.ONEOFMANY,
        #     ),
        # )
        # self.CameraVectorNames.append("TELESCOPE_TYPE")
        #
        # delayed updates
        self.knownVectors["RAW_FORMAT"].update_Binning()  # set binning according to frame type and raw format
        # finish
        return True

    def addCameraControls(self, group="Camera controls"):
        """add vectors for camera controls

        See picamera2 manual for details. Default values are set for manual exposure control.
        """
        # automatic exposure control
        if "AeEnable" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AEENABLE", label="AeEnable", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="INDI_ENABLED", label="Enabled", state=ISState.OFF),
                        ISwitch(name="INDI_DISABLED", label="Disabled", state=ISState.ON),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AEENABLE")
        #
        if "AeConstraintMode" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AECONSTRAINTMODE", label="AeConstraintMode", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="NORMAL", label="Normal", state=ISState.ON),
                        ISwitch(name="HIGHLIGHT", label="Highlight", state=ISState.OFF),
                        ISwitch(name="SHADOWS", label="Shadows", state=ISState.OFF),
                        ISwitch(name="CUSTOM", label="Custom", state=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AECONSTRAINTMODE")
        #
        if "AeExposureMode" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AEEXPOSUREMODE", label="AeExposureMode", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="NORMAL", label="Normal", value=ISState.ON),
                        ISwitch(name="SHORT", label="Short", value=ISState.OFF),
                        ISwitch(name="LONG", label="Long", value=ISState.OFF),
                        ISwitch(name="CUSTOM", label="Custom", value=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AEEXPOSUREMODE")
        #
        if "AeMeteringMode" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AEMETERINGMODE", label="AeMeteringMode", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="CENTREWEIGHTED", label="CentreWeighted", value=ISState.ON),
                        ISwitch(name="SPOT", label="Spot", value=ISState.OFF),
                        ISwitch(name="MATRIX", label="Matrix", value=ISState.OFF),
                        ISwitch(name="CUSTOM", label="Custom", value=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AEMETERINGMODE")
        # automatic focus control
        if "AfMode" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AFMODE", label="AfMode", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="MANUAL", label="Manual", value=ISState.ON),
                        ISwitch(name="AUTO", label="Auto", value=ISState.OFF),
                        ISwitch(name="CONTINUOUS", label="Continuous", value=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AFMODE")
        #
        if "AfMetering" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AFMETERING", label="AfMetering", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="AUTO", label="Auto", value=ISState.ON),
                        ISwitch(name="WINDOWS", label="Windows", value=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AFMETERING")
        #
        if "AfPause" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AFPAUSE", label="AfPause", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="DEFERRED", label="Deferred", value=ISState.ON),
                        ISwitch(name="IMMEDIATE", label="Immediate", value=ISState.OFF),
                        ISwitch(name="RESUME", label="Resume", value=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AFPAUSE")
        #
        if "AfRange" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AFRANGE", label="AfRange", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="NORMAL", label="Normal", value=ISState.ON),
                        ISwitch(name="MACRO", label="Macro", value=ISState.OFF),
                        ISwitch(name="FULL", label="Full", value=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AFRANGE")
        #
        if "AfSpeed" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AFSPEED", label="AfSpeed", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="NORMAL", label="Normal", value=ISState.ON),
                        ISwitch(name="FAST", label="Fast", value=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AFSPEED")
        #
        if "AfTrigger" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AFTRIGGER", label="AfTrigger", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="START", label="Start", value=ISState.ON),
                        ISwitch(name="CANCEL", label="Cancel", value=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AFTRIGGER")
        # automatic white balance
        if "AwbEnable" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AWBENABLE", label="AwbEnable", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="INDI_ENABLED", label="Enabled", value=ISState.OFF),
                        ISwitch(name="INDI_DISABLED", label="Disabled", value=ISState.ON),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AWBENABLE")
        #
        if "AwbMode" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_AWBMODE", label="AwbMode", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="AUTO", label="Auto", value=ISState.ON),
                        ISwitch(name="TUNGSTEN", label="Tungsten", value=ISState.OFF),
                        ISwitch(name="FLUORESCENT", label="Fluorescent", value=ISState.OFF),
                        ISwitch(name="INDOOR", label="Indoor", value=ISState.OFF),
                        ISwitch(name="DAYLIGHT", label="Daylight", value=ISState.OFF),
                        ISwitch(name="CLOUDY", label="Cloudy", value=ISState.OFF),
                        ISwitch(name="CUSTOM", label="Custom", value=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_AWBMODE")
        # brightness, contrast and color adjustments
        if "Brightness" in self.CameraThread.camera_controls:
            self.IDDef(
                INumberVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_BRIGHTNESS", label="Brightness",
                    np=[
                        INumber(name="BRIGHTNESS", label="Brightness", min=-1.0, max=1.0, step=0.1, value=0.0, format="%.1f"),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_BRIGHTNESS")
        #
        if "ColourGains" in self.CameraThread.camera_controls:
            self.IDDef(
                INumberVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_COLOURGAINS", label="ColourGains",  # only used when CAMCTRL_AWBENABLE disabled
                    np=[
                        INumber(name="REDGAIN", label="Red gain", min=0.0, max=32.0, step=0.1, value=2.0, format="%.2f"),
                        INumber(name="BLUEGAIN", label="Blue gain", min=0.0, max=32.0, step=0.1, value=2.0, format="%.2f"),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_COLOURGAINS")
        #
        if "Contrast" in self.CameraThread.camera_controls:
            self.IDDef(
                INumberVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_CONTRAST", label="Contrast",
                    np=[
                        INumber(name="CONTRAST", label="Contrast", min=0.0, max=32.0, step=0.1, value=1.0, format="%.2f"),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_CONTRAST")
        #
        if "ExposureValue" in self.CameraThread.camera_controls:
            self.IDDef(
                INumberVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_EXPOSUREVALUE", label="ExposureValue",
                    np=[
                        INumber(name="EXPOSUREVALUE", label="ExposureValue", min=-8.0, max=8.0, step=0.1, value=0.0, format="%.1f"),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_EXPOSUREVALUE")
        # misc
        if "NoiseReductionMode" in self.CameraThread.camera_controls:
            self.IDDef(
                ISwitchVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_NOISEREDUCTIONMODE", label="NoiseReductionMode", rule=ISRule.ONEOFMANY,
                    sp=[
                        ISwitch(name="OFF", label="Off", value=ISState.ON),
                        ISwitch(name="FAST", label="Fast", value=ISState.OFF),
                        ISwitch(name="HIGHQUALITY", label="HighQuality", value=ISState.OFF),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_NOISEREDUCTIONMODE")
        #
        if "Saturation" in self.CameraThread.camera_controls:
            self.IDDef(
                INumberVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_SATURATION", label="Saturation",
                    np=[
                        INumber(name="SATURATION", label="Saturation", min=0.0, max=32.0, step=0.1, value=1.0, format="%.2f"),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_SATURATION")
        #
        if "Sharpness" in self.CameraThread.camera_controls:
            self.IDDef(
                INumberVector(
                    device=self.device, timestamp=self.timestamp, group=group,
                    name="CAMCTRL_SHARPNESS", label="Sharpness",
                    np=[
                        INumber(name="SHARPNESS", label="Sharpness", min=0.0, max=16.0, step=0.1, value=0.0, format="%.2f"),
                    ],
                ),
            )
            self.CameraVectorNames.append("CAMCTRL_SHARPNESS")


    def startExposure(self, exposuretime):
        """start single or fast exposure

        Args:
            exposuretime: exposure time (seconds)
        """
        self.CameraThread.startExposure(exposuretime)


    def abortExposure(self):
        """abort a running exposure
        """
        self.CameraThread.abortExposure()


# main entry point
def main():
    device = indi_pylibcamera(config=read_config())
    device.start()
    return 0


if __name__ == "__main__":
    main()
