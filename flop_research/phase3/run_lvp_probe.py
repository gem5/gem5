"""
run_lvp_probe.py — gem5 script for Phase 3 LVP Probe

Runs lvp_probe (300-iteration load loop) on the O3 ARM CPU.
With --debug-flags=LVP the trace should show:
  - Iterations 0-249:   'no confident prediction (counter=N/250)'
  - Iterations 250-299: 'confident prediction 0x2a (counter=250/250)'
  - Final load (value=99): LVP predicts 42, real value 99 → SQUASH
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

# ── O3 CPU (same config as Phase 1 — only valid params) ─────────────────────
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
binary = os.path.join(thispath, "lvp_probe")

system.workload = SEWorkload.init_compatible(binary)
process = Process()
process.cmd = [binary]
system.cpu.workload = process
system.cpu.createThreads()

# ── Run ──────────────────────────────────────────────────────────────────────
root = Root(full_system=False, system=system)
m5.instantiate()

print("=" * 60)
print("Phase 3 LVP Probe: 300 loads of sensor_value=42")
print("Confident predictions expected after iteration 250.")
print("Final load (value=99) should trigger a SQUASH.")
print("=" * 60)

exit_event = m5.simulate()
print(f"\nExiting @ tick {m5.curTick()} — {exit_event.getCause()}")
