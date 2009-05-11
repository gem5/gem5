#!/s/std/bin/python
import sys, os, string, re, math

rows = 0
cols =0

if len(sys.argv) == 3:
    rows = int(sys.argv[1])
    cols = int(sys.argv[2])
else:
    sys.stderr.write("usage : NetworkFileMaker.py <rows> <cols> \n\n")

banks = rows*cols
bank = 0
while bank < banks:
    sys.stdout.write("ext_node:L2Cache:0:bank:%d int_node:%d link_latency:1 bw_multiplier:16\n" % (bank, bank))
    bank += 1

sys.stdout.write("\n")

col = 0
while col < cols:
    row = 1
    bank = col*rows
    while row < rows:
        sys.stdout.write("int_node:%d int_node:%d link_latency:1 bw_multiplier:16\n" % (bank, bank+1))
        bank += 1
        row += 1
    sys.stdout.write("\n")
    col += 1

sys.stdout.write("\n")

row = 0
while row < rows:
    col = 1
    bank = row
    while col < cols:
        sys.stdout.write("int_node:%d int_node:%d link_latency:1 bw_multiplier:16\n" % (bank, rows+bank))
        bank += rows
        col += 1
    sys.stdout.write("\n")
    row += 1

