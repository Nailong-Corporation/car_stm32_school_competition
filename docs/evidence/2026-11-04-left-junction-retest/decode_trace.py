"""Offline decoder; offsets from read-time.elf (see layout.txt).
No target access or firmware comparison. ram.core is an SRAM-only ELF container.
"""
import csv
import struct
from pathlib import Path

root = Path(__file__).resolve().parent
ram = (root / "ram.bin").read_bytes()
assert len(ram) == 16384
assert ram[0x3c4:0x3c4 + 160] == ram[0x2a40:0x2a40 + 160]
base, stride, capacity = 0xca4, 56, 128
next_index, count = struct.unpack_from("<HH", ram, base + stride * capacity)
assert next_index < capacity and count <= capacity
fields = ["ms", "dt_s", "heading_rad", "distance_cm", "left_speed", "right_speed",
          "route_index", "junctions_seen", "left_count", "right_count", "raw_mask",
          "line_mask", "prefilter_mask", "observation", "route_before", "route_after",
          "evidence", "sides", "rejection", "heading_source", "candidate_samples",
          "stop_reason", "node_kind", "component_pass_active", "left_target", "right_target"]
fmt = "<I5fIH2H14B2f"
assert struct.calcsize(fmt) == stride
with (root / "navigation_trace.csv").open("w", newline="") as out:
    writer = csv.writer(out)
    writer.writerow(fields)
    for n in range(count):
        i = (next_index - count + n) % capacity
        writer.writerow(struct.unpack_from(fmt, ram, base + i * stride))
print(f"Snapshot copies match (160 bytes); trace next={next_index}, count={count}")
