"""
run_simple_loop.py — MLP Benchmark Runner

Runs the simple_loop_bench.c to demonstrate Out-of-Order Memory-Level Parallelism.
"""

import m5
from m5.objects import *
import os

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
binary = os.path.join(thispath, "simple_loop_bench")

system.workload = SEWorkload.init_compatible(binary)
process = Process()
process.cmd = [binary]
system.cpu.workload = process
system.cpu.createThreads()

# ── Run ──────────────────────────────────────────────────────────────────────
root = Root(full_system=False, system=system)
m5.instantiate()

print("=" * 60)
print("Starting Simple Loop Benchmark (MLP Demonstration)")
print("=" * 60)

exit_event = m5.simulate()

print("=" * 60)
print(f"Benchmark Complete! Total Ticks: {m5.curTick()}")
print("=" * 60)
