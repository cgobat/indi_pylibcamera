#!/usr/bin/env python3
"""
Post-installation script for indi_pylibcamera.

This script creates a symbolic link to indi_pylibcamera.xml in /usr/share/indi
Run it with root privileges.
"""

import sys
from pathlib import Path


DEFAULT_INDI_PATH = "/usr/share/indi"
SRC_FILE_PATH = Path(__file__)
XML_FILE_NAME = "indi_pylibcamera.xml"


def create_Link(indi_path, overwrite=True, mkdir=False):
    src = SRC_FILE_PATH.parent/XML_FILE_NAME
    dest = Path(indi_path)/XML_FILE_NAME
    if overwrite:
        try:
            dest.unlink(missing_ok=True)
        except PermissionError:
            print(f'ERROR: You need to run this with root permissions (sudo).')
            return -3
    if mkdir:
        try:
            dest.parent.mkdir(parents=True, exist_ok=True)
        except PermissionError:
            print(f'ERROR: Insufficient permissions to create directory {indi_path}.')
            return -3
    try:
        dest.symlink_to(src)
    except FileExistsError:
        print(f'ERROR: File {dest} exists. Please remove it before running this script.')
        return -1
    except FileNotFoundError:
        print(f'ERROR: File {dest} could not be created. Is the INDI path wrong?')
        return -2
    except PermissionError:
        print(f'ERROR: You need to run this with root permissions (sudo).')
        return -3
    return 0


def create_LinkInteractive(interactive, indi_path):
    if interactive:
        print("""
This script tells INDI about the installation of the indi_pylibcamera driver. It is only needed to run this
script once after installing INDI (KStars) and indi_pylibcamera.

Please run this script with root privileges (sudo).

        """)
        while True:
            inp_cont = input("Do you want to continue? (Y/n): ").lower()
            if inp_cont in ["", "y", "yes"]:
                break
            elif inp_cont in ["n", "no"]:
                return
        inp_indi_path = input(
            f'Path to INDI driver XMLs (must contain "driver.xml") (press ENTER to leave default {indi_path}): '
        )
        if len(inp_indi_path) > 0:
            indi_path = inp_indi_path
        print(f'Creating symbolic link in {indi_path}...')
    ret = create_Link(indi_path=indi_path, overwrite=True)
    if interactive:
        if ret == 0:
            print("Done.")
        else:
            print(f'Exit with error {ret}.')
    return ret


def main():
    import argparse

    parser = argparse.ArgumentParser(
        prog="indi_pylibcamera_postinstall",
        description="Make settings in INDI to use indi_pylibcamera.",
    )
    parser.add_argument("-s", "--silent", action="store_true", help="run silently")
    parser.add_argument("-p", "--path", type=str, default=DEFAULT_INDI_PATH,
                        help=f'path to INDI driver XMLs, default: {DEFAULT_INDI_PATH}')
    args = parser.parse_args()
    #
    create_LinkInteractive(interactive=not args.silent, indi_path=args.path)


if __name__ == "__main__":
    sys.exit(main())
