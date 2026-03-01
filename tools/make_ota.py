#!/usr/bin/env python3
"""
Create Zigbee OTA upgrade file (.zigbee) from ESP32 firmware binary.

Usage: python3 tools/make_ota.py [firmware.bin] [output.zigbee]
Defaults: build/szambo_tof_native.bin -> build/szambo_tof_native.zigbee
"""

import struct
import sys
import os

# Must match zigbee_device.h
MANUFACTURER_CODE = 0x1001
IMAGE_TYPE = 0x1011
FILE_VERSION = 0x01000009  # update this when bumping firmware version
ZIGBEE_STACK_VERSION = 0x0002

OTA_FILE_IDENTIFIER = 0x0BEEF11E
HEADER_VERSION = 0x0100
HEADER_LENGTH = 56
FIELD_CONTROL = 0x0000
OTA_HEADER_STRING = b'ESP32-C6 Szambo Sensor'

TAG_UPGRADE_IMAGE = 0x0000


def make_ota(input_bin, output_ota):
    with open(input_bin, 'rb') as f:
        firmware = f.read()

    # Sub-element: Upgrade Image tag (2B) + length (4B) + data
    sub_element = struct.pack('<HI', TAG_UPGRADE_IMAGE, len(firmware)) + firmware

    total_image_size = HEADER_LENGTH + len(sub_element)

    header_string_padded = OTA_HEADER_STRING[:32].ljust(32, b'\x00')

    # Header format (56 bytes):
    # I  - OTA file identifier
    # H  - header version
    # H  - header length
    # H  - field control
    # H  - manufacturer code
    # H  - image type
    # I  - file version
    # H  - zigbee stack version
    # 32s - header string
    # I  - total image size
    header = struct.pack('<IHHHHHIH32sI',
        OTA_FILE_IDENTIFIER,
        HEADER_VERSION,
        HEADER_LENGTH,
        FIELD_CONTROL,
        MANUFACTURER_CODE,
        IMAGE_TYPE,
        FILE_VERSION,
        ZIGBEE_STACK_VERSION,
        header_string_padded,
        total_image_size,
    )
    assert len(header) == HEADER_LENGTH, f"Header size {len(header)} != {HEADER_LENGTH}"

    with open(output_ota, 'wb') as f:
        f.write(header + sub_element)

    print(f"OTA file: {output_ota}")
    print(f"  Firmware:     {len(firmware):,} bytes")
    print(f"  OTA total:    {total_image_size:,} bytes")
    print(f"  Manufacturer: 0x{MANUFACTURER_CODE:04X}")
    print(f"  Image type:   0x{IMAGE_TYPE:04X}")
    print(f"  Version:      0x{FILE_VERSION:08X}")


if __name__ == '__main__':
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    input_bin = sys.argv[1] if len(sys.argv) > 1 else os.path.join(base, 'build', 'szambo_tof_native.bin')
    output_ota = sys.argv[2] if len(sys.argv) > 2 else os.path.join(base, 'build', 'szambo_tof_native.zigbee')
    make_ota(input_bin, output_ota)
