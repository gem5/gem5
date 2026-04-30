"""
run_benchmark.py — LVP Performance Benchmark Runner

Runs the benchmark.c loop (5 million loads) to measure IPC and ticks.
"""

import m5
from m5.objects import *
import os
import sys

# ── System ──────────────────────────────────────────────────────────────────
system = System()
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "3GHz"
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = "timing"
system.mem_ranges = [AddrRange("512MB")]

# ── O3 CPU ──────────────────────────────────────────────────────────────────
system.cpu = ArmO3CPU()
system.cpu.numROBEntries = 256
system.cpu.LQEntries     = 128
system.cpu.SQEntries     = 128

system.cpu.createInterruptController()

# ── Memory bus + DDR3 ───────────────────────────────────────────────────────
system.membus = SystemXBar()
system.cpu.icache_port = system.membus.cpu_side_ports
system.cpu.dcache_port = system.membus.cpu_side_ports
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports
system.system_port = system.membus.cpu_side_ports

# ── Workload ─────────────────────────────────────────────────────────────────
thispath = os.path.dirname(os.path.realpath(__file__))
binary = os.path.join(thispath, "benchmark")

system.workload = SEWorkload.init_compatible(binary)
process = Process()
process.cmd = [binary]
system.cpu.workload = process
system.cpu.createThreads()

# ── Run ──────────────────────────────────────────────────────────────────────
root = Root(full_system=False, system=system)
m5.instantiate()

print("=" * 60)
print("Starting Scientific Benchmark (5 million loads)")
print("Check GEM5_DISABLE_LVP status to determine A/B test.")
print("=" * 60)

exit_event = m5.simulate()

# Print the final cycle count explicitly to make it easy to read
print("=" * 60)
print(f"Benchmark Complete! Total Ticks: {m5.curTick()}")
print("Check m5out/stats.txt for system.cpu.ipc and system.cpu.cpi")
print("=" * 60)
