"""Decode archived SRAM only, using read-time.elf offsets in layout.txt."""
import csv
import struct
from pathlib import Path
p=Path(__file__).resolve().parent
ram=(p/'ram.bin').read_bytes()
assert len(ram)==16384
assert ram[0x3c4:0x3c4+176]==ram[0x2e60:0x2e60+176]
base,stride,capacity=0xcc4,64,128
next_index,count=struct.unpack_from('<HH',ram,base+stride*capacity)
assert next_index<capacity and count<=capacity
fields=['ms','dt_s','heading_rad','distance_cm','left_speed','right_speed','route_index',
        'junctions_seen','left_count','right_count','raw_mask','line_mask','prefilter_mask',
        'observation','route_before','route_after','evidence','sides','rejection','heading_source',
        'candidate_samples','stop_reason','node_kind','component_pass_active',
        'left_target','right_target','left_pwm','right_pwm']
fmt='<I5fIH2H14B4f'
assert struct.calcsize(fmt)==stride
with (p/'navigation_trace.csv').open('w',newline='') as out:
 w=csv.writer(out);w.writerow(fields)
 for n in range(count):
  i=(next_index-count+n)%capacity
  w.writerow(struct.unpack_from(fmt,ram,base+i*stride))
print(f'Snapshot copies match: 176 bytes; trace next={next_index}, count={count}')
