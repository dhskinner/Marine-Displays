# PlatformIO extra script (post): write boot_app0.bin to this project's otadata
# partition on USB upload, not inside NVS.
#
# The Arduino framework always adds boot_app0.bin at 0xe000, the otadata
# offset of the default partition table. Every display here uses its own
# partitions.csv with NVS at 0x9000-0x29000 and otadata at 0x29000, so 0xe000
# is in the middle of NVS: each full `pio run -t upload` overwrote 8 KB of it
# and NVS dropped the settings stored there.
#
# This script:
#   - moves that boot_app0 write to the otadata offset read from partitions.csv
#     (the same offset tools/merge_firmware.py and the web installer use);
#   - adds the boot_app0 write when it's missing (`-t nobuild` uploads only
#     the app), so the bootloader always starts app0 after a USB upload rather
#     than a stale app1 left by a previous OTA update.
# Everything goes in the one esptool run. It replaces each project's old
# fix_ota_boot.py, which ran esptool a second time to write 0x29000 but left
# the 0xe000 write in place.
#
# Use in platformio.ini:   extra_scripts = ... ../tools/fix_flash_images.py

Import("env")  # noqa: F821  (provided by PlatformIO)

import csv
import os

from SCons.Script import COMMAND_LINE_TARGETS

DEFAULT_OTADATA = 0xE000


def _otadata_offset(env):
    """otadata offset from the project's partitions.csv, or None."""
    csv_name = env.GetProjectOption("board_build.partitions", "")
    if not csv_name:
        return None
    path = os.path.join(env.subst("$PROJECT_DIR"), csv_name)
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        rows = (line for line in f if line.strip() and not line.lstrip().startswith("#"))
        for row in csv.reader(rows):
            row = [c.strip() for c in row]
            if len(row) >= 4 and row[1] == "data" and row[2] == "ota":
                return int(row[3], 0)
    return None


def _boot_app0_path(env):
    framework = env.PioPlatform().get_package_dir("framework-arduinoespressif32") or ""
    path = os.path.join(framework, "tools", "partitions", "boot_app0.bin")
    return path if os.path.isfile(path) else None


def _fix_upload_flags(env):
    if env.subst("$UPLOAD_PROTOCOL") != "esptool":
        return  # e.g. espota: nothing is written outside the app
    targets = [str(t) for t in COMMAND_LINE_TARGETS]
    if "upload" not in targets or "uploadfs" in targets:
        return

    otadata = _otadata_offset(env)
    if otadata is None:
        print("fix_flash_images: no otadata partition in partitions.csv; leaving upload unchanged")
        return

    flags = list(env.get("UPLOADERFLAGS", []))
    found = False
    for i in range(len(flags) - 1):
        offset, image = str(flags[i]), str(flags[i + 1])
        if "boot_app0" not in os.path.basename(image):
            continue
        found = True
        if int(offset, 0) != otadata:
            print("fix_flash_images: boot_app0.bin %s -> %s (otadata; %s is inside NVS)"
                  % (offset, hex(otadata), offset))
            flags[i] = hex(otadata)

    if not found:
        boot_app0 = _boot_app0_path(env)
        if not boot_app0:
            print("fix_flash_images: boot_app0.bin not found; otadata not written")
            return
        print("fix_flash_images: adding boot_app0.bin at %s (app-only upload)" % hex(otadata))
        flags += [hex(otadata), boot_app0]

    env.Replace(UPLOADERFLAGS=flags)

    # Keep FLASH_EXTRA_IMAGES consistent for anything else that reads it.
    extra = []
    for offset, image in env.get("FLASH_EXTRA_IMAGES", []):
        if "boot_app0" in os.path.basename(env.subst(image)) and int(offset, 0) == DEFAULT_OTADATA:
            offset = hex(otadata)
        extra.append((offset, image))
    env.Replace(FLASH_EXTRA_IMAGES=extra)


_fix_upload_flags(env)  # noqa: F821
