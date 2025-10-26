# Copyright (c) 2015 Jason Power
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
This is the RISCV equivalent to `simple.py` (which is designed to run using the
X86 ISA). More detailed documentation can be found in `simple.py`.
"""

import m5
from m5.objects import *
from gem5.components.memory import HBM2Stack

system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1.5GHz"
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = "timing"

system.mem_ranges = [
        AddrRange("32GiB")]

# system.cpu = [RiscvTimingSimpleCPU(), RiscvTimingSimpleCPU(),RiscvTimingSimpleCPU(),RiscvTimingSimpleCPU()]
# system.cpu = [
    # ArmO3CPU(cpu_id=i, numThreads=1) for i in range(8)]

system.cpu = [
    ArmO3CPU(cpu_id=i, numThreads=1) for i in range(16)]
    
# system.cpu = [ArmTimingSimpleCPU(), ArmTimingSimpleCPU(), ArmTimingSimpleCPU(), ArmTimingSimpleCPU()]
# system.cpu = [ArmO3CPU(),ArmO3CPU()]

# system.cpu = [ArmTimingSimpleCPU()]


system.membus = NoncoherentXBar(
        frontend_latency=0,
        forward_latency=0,
        response_latency=0,
        header_latency=0,
        width=64,
        p_size_p=1,
        p_addr_strat=0x0
    )
for cpu in system.cpu:
    cpu.icache_port = system.membus.cpu_side_ports
    cpu.dcache_port = system.membus.cpu_side_ports
    # cpu.createInterruptController()

    cpu.createInterruptController()
    # cpu.interrupts[0].pio = system.membus.mem_side_ports
    # cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
    # cpu.interrupts[0].int_responder = system.membus.mem_side_ports
  
start = 0
end = 32*1024
mem_range = AddrRange(str(start) + 'MiB', str(end) + 'MiB')
system.mem_ranges.append(mem_range)

# Instantiate HBM2Stack and complete the correct configuration
hbm_memory = HBM2Stack(size="32GiB",num_channels=64)
system.hbm_memory = hbm_memory
system.hbm_memory._create_mem_interfaces_controller()
system.hbm_memory.set_memory_range([system.mem_ranges[0]])

system.hbm_memory._interleave_addresses()
i = 0

for ctrl in system.hbm_memory.mem_ctrl:
    print(f"Controller {i}:")
    print(f"  Start: {ctrl.dram.range.start}")
    print(f"  End: {ctrl.dram.range.end}")
    print(f"  Masks: {ctrl.dram.range.masks}")
    print(f"  IntlvMatch: {ctrl.dram.range.intlvMatch}")

    ctrl.port = system.membus.mem_side_ports


system.membus.badaddr_responder = BadAddr()
system.membus.default = system.membus.badaddr_responder.pio

thispath = os.path.dirname(os.path.realpath(__file__))
binary = os.path.join(
    thispath,
    # "../../../",
    # "arm_memory_demo_aarch64"
    # "arm_allreduce_aarch64"
    "arm_allgather_aarch64"
    # "arm_con_pro_aarch64"
)
# arm_con_pro_aarch64
# arm_allreduce_aarch64
# aarch64-linux-gnu-gcc -o arm_allgather_aarch64 -static ../part5_o/a

system.workload = SEWorkload.init_compatible(binary)

    # cwd = os.getcwd()
multiprocesses = []
idx = 0x80
process = Process()
process.cmd = [binary,idx]
for cpu in system.cpu:
    # process = Process(pid=10 + idx)
    # process.cmd = [binary, idx]  # 添加参数0xFE
    # process.gid = os.getgid()
    cpu.workload = process
    cpu.createThreads()
    print(f"Process {idx}:")
    idx += 1


# cpu = system.cpu[0]
# process = Process(pid=100)
# process.cmd = [binary]
# process.gid = os.getgid()
# cpu.workload = process
# cpu.createThreads()
# idx += 1




root = Root(full_system=False, system=system)
m5.instantiate()

print(f"Beginning simulation!")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")















