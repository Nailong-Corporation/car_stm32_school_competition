"""Decode the 2026-11-05 SRAM ring buffers using the archived read-time ELF layout.

Addresses/sizes were read with arm-none-eabi-gdb ptype /o against read-time.elf.
This script only reads the saved SRAM image; it does not access the target.
"""
import csv
import struct
from pathlib import Path

root = Path(__file__).resolve().parent
ram = (root / "ram.bin").read_bytes()
assert len(ram) == 0x4000


def export(name, base, capacity, stride, next_at, count_at, fmt, fields):
    if stride == 36:  # TurnTrace ring indices are uint8_t.
        next_index, count = ram[next_at], ram[count_at]
    else:  # NavigationTrace ring indices are uint16_t.
        next_index = struct.unpack_from("<H", ram, next_at)[0]
        count = struct.unpack_from("<H", ram, count_at)[0]
    assert 0 <= next_index < capacity and 0 <= count <= capacity
    assert struct.calcsize(fmt) == stride
    with (root / name).open("w", newline="") as out:
        writer = csv.writer(out)
        writer.writerow(["ring_index", *fields])
        for n in range(count):
            index = (next_index - count + n) % capacity
            writer.writerow([index, *struct.unpack_from(fmt, ram, base + index * stride)])
    print(name, "next", next_index, "count", count)


# TurnTrace at 0x200005b8: 48 x 36 B, followed by last_ms / next / count.
export("turn_trace.csv", 0x5b8, 48, 36, 0x5b8 + 1732, 0x5b8 + 1733,
       "<I7f4B", ["ms", "distance_cm", "turn_rad", "position", "left_target",
                    "right_target", "left_speed", "right_speed", "mask", "kind",
                    "route_state", "capturing"])

# NavigationTrace at 0x20000c88: 128 x 48 B, next/count are uint16 after samples.
# Pad 2 bytes after the 14 byte tail; not part of the data fields.
export("navigation_trace.csv", 0xc88, 128, 48, 0xc88 + 6144, 0xc88 + 6146,
       "<I5fIH2H12B2x", ["ms", "dt_s", "heading_rad", "distance_cm",
                           "left_speed", "right_speed", "route_index", "junctions_seen",
                           "left_count", "right_count", "raw_mask", "line_mask",
                           "prefilter_mask", "observation", "route_before", "route_after",
                           "evidence", "sides", "rejection", "heading_source",
                           "candidate_samples", "stop_reason"])
