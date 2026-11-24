# Copyright (c) 2026 Nathanon Chirattithiphan
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

"""Run a bare-metal x86 MONITOR/MWAIT regression."""

import resource
import subprocess
import sys
from pathlib import Path

import m5
from m5.objects import (
    X86O3CPU,
    AddrRange,
    Cache,
    PyTrafficGen,
    Root,
    SimpleMemory,
    SrcClockDomain,
    System,
    SystemXBar,
    VoltageDomain,
    X86AtomicSimpleCPU,
    X86FsWorkload,
)
from m5.stats.gem5stats import get_simstat


def l1():
    return Cache(
        size="16KiB",
        assoc=2,
        tag_latency=1,
        data_latency=1,
        response_latency=1,
        mshrs=4,
        tgts_per_mshr=4,
    )


def check_exit(event, expected, phase):
    if event.getCause() != expected or event.getCode():
        print(
            f"{phase}: expected {expected!r}, got {event.getCause()!r} "
            f"(code {event.getCode()}) at tick {m5.curTick()}",
            file=sys.stderr,
        )
        sys.exit(1)


def write_line(writer, addr):
    # End on the byte limit after one write and leave the generator idle
    # until the next phase starts.
    writer.start(
        [writer.createLinear(0, addr, addr + 64, 64, 1000, 1000, 0, 64)]
    )


def main():
    cpu, case = sys.argv[1:]
    model, mem_mode = {
        "atomic": (X86AtomicSimpleCPU, "atomic"),
        "o3": (X86O3CPU, "timing"),
    }[cpu]
    # Build in TestLib's per-run output directory. The suites use the
    # native toolchain on x86_64 hosts.
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    source = Path(__file__).resolve().with_suffix(".S")
    guest = Path(m5.options.outdir) / "mwait.elf"
    include = source.parents[4] / "include"
    monitored_line = 0x210000
    case_define = case.upper().replace("-", "_")
    subprocess.run(
        [
            "cc",
            "-m64",
            "-nostdlib",
            "-static",
            "-Wl,--build-id=none,-Ttext=0x200000",
            f"-Wl,--section-start=.data={monitored_line:#x}",
            f"-I{include}",
            f"-D{case_define}",
            str(source),
            "-o",
            str(guest),
        ],
        check=True,
    )

    system = System(
        mem_mode=mem_mode,
        mem_ranges=[AddrRange("64MiB")],
        clk_domain=SrcClockDomain(
            clock="1GHz", voltage_domain=VoltageDomain()
        ),
        cpu=model(),
        bus=SystemXBar(),
    )
    system.system_port = system.bus.cpu_side_ports
    system.memory = SimpleMemory(range=system.mem_ranges[0])
    system.memory.port = system.bus.mem_side_ports
    system.cpu.createThreads()
    system.cpu.createInterruptController()
    system.cpu.addPrivateSplitL1Caches(l1(), l1())
    system.cpu.connectBus(system.bus)
    if case == "armed":
        system.writer = PyTrafficGen()
        system.writer_cache = l1()
        system.writer.port = system.writer_cache.cpu_side
        system.writer_cache.mem_side = system.bus.cpu_side_ports

    system.workload = X86FsWorkload(
        object_file=str(guest),
        exit_on_kernel_panic=False,
        exit_on_kernel_oops=False,
    )
    root = Root(full_system=True, system=system)
    m5.instantiate()

    budget = 10_000_000
    if case == "armed":
        limit = "simulate() limit reached"
        check_exit(m5.simulate(budget), limit, "before writes")
        write_line(system.writer, monitored_line + 64)
        check_exit(m5.simulate(budget), limit, "after different-line write")
        if get_simstat(system.writer).totalWrites.value != 1:
            print("Different-line write did not complete", file=sys.stderr)
            sys.exit(1)
        write_line(system.writer, monitored_line)
    check_exit(m5.simulate(budget), "m5_exit instruction encountered", case)


if __name__ == "__m5_main__":
    main()
