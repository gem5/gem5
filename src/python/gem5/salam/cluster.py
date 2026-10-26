# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

# Accelerator cluster construction shared by configs/SALAM and the gem5
# standard library. It is embedded in gem5 builds with SALAM.

import os
from types import SimpleNamespace

from m5.objects import *
from m5.params import AddrRange

from . import (
    config_parser,
    power_model,
    workload_config,
)


class AcceleratorCache(Cache):
    # An accelerator's private cache (configs/common's L1Cache values).
    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20


def build_clusters(
    system,
    working_dir,
    config_name="config.yml",
    acc_cache=False,
    l2cache=False,
    membus=None,
):
    # Add the accelerator clusters a workload configuration describes to
    # an ARM system with an iobus, a membus and a realview GIC. acc_cache
    # and l2cache choose where a cluster's coherency bus connects; membus
    # is the system's unless given.
    options = SimpleNamespace(acc_cache=acc_cache, l2cache=l2cache)
    clusters = config_parser.load_clusters(working_dir, config_name)
    for spec in clusters:
        if (
            spec.base_address < workload_config.WORKLOAD_ADDR_START
            or spec.top_address > workload_config.WORKLOAD_ADDR_END
        ):
            raise workload_config.WorkloadConfigError(
                spec.config_path,
                f"cluster {spec.name}",
                "Size",
                spec.top_address - spec.base_address,
                "allocated devices exceed the supported ARM workload "
                "address range [0x2f000000, 0x30000000)",
            )
        if hasattr(system, spec.name.lower()):
            raise workload_config.WorkloadConfigError(
                spec.config_path,
                "cluster",
                "Name",
                spec.name,
                f"Name {spec.name!r} collides with a system attribute",
            )
    for spec in clusters:
        setattr(system, spec.name.lower(), AccCluster())
        clstr = getattr(system, spec.name.lower())
        _attach_cluster(options, system, clstr, spec, membus)
    return clusters


def _attach_cluster(options, system, clstr, spec, membus=None):
    # The accelerators' compute clock (the cluster's Clocks Compute).
    # Their CommInterfaces and, through them, their LLVMInterfaces run on
    # it.
    clstr.compute_voltage_domain = VoltageDomain()
    clstr.compute_clk_domain = SrcClockDomain(
        clock=spec.clocks["Compute"],
        voltage_domain=clstr.compute_voltage_domain,
    )
    # The clock of the cluster's local bus, DMAs, streams and memories
    # (Clocks Fabric).
    clstr.fabric_voltage_domain = VoltageDomain()
    clstr.fabric_clk_domain = SrcClockDomain(
        clock=spec.clocks["Fabric"],
        voltage_domain=clstr.fabric_voltage_domain,
    )
    system.iobus.mem_side_ports = clstr.local_bus.cpu_side_ports
    clstr._connect_caches(system, options, l2coherent=False, membus=membus)
    gic = system.realview.gic

    children = [(obj.name, obj.name) for obj in spec.dmas + spec.accs]
    children.extend(
        (var.name if var.type == "Cache" else var.name.lower(), var.name)
        for acc in spec.accs
        for var in acc.variables
    )
    for child, name in children:
        if hasattr(clstr, child):
            raise workload_config.WorkloadConfigError(
                spec.config_path,
                f"cluster {spec.name}",
                "Name",
                name,
                f"Name {name!r} collides with a cluster attribute",
            )
    for dma in spec.dmas:
        if dma.dmaType == "Stream":
            _attach_stream_dma(clstr, dma, gic)
        else:
            _attach_noncoherent_dma(clstr, dma, gic)

    # Create every CommInterface before wiring.
    for acc in spec.accs:
        _attach_accelerator(clstr, acc, gic)
    for acc in spec.accs:
        _connect_accelerator(clstr, acc)

    clstr.local_bus.clk_domain = clstr.fabric_clk_domain
    fabric = (
        NoncoherentDma,
        StreamDma,
        StreamBuffer,
        ScratchpadMemory,
        RegisterBank,
    )
    for child in clstr._children.values():
        if isinstance(child, fabric):
            child.clk_domain = clstr.fabric_clk_domain


def _attach_noncoherent_dma(clstr, dma, gic):
    params = {
        "pio_addr": dma.address,
        "pio_size": dma.pio,
        "gic": gic,
    }
    if dma.int_num is not None:
        params["int_num"] = dma.int_num
    setattr(clstr, dma.name, NoncoherentDma(**params))
    obj = getattr(clstr, dma.name)
    obj.cluster_dma = clstr.local_bus.cpu_side_ports
    if dma.maxReq is not None:
        obj.max_req_size = dma.maxReq
    if dma.size is not None:
        obj.buffer_size = dma.size
    obj.dma = clstr.coherency_bus.cpu_side_ports
    if dma.pio_masters is not None:
        for master in dma.pio_masters:
            getattr(clstr, master.lower()).mem_side_ports = obj.pio


def _attach_stream_dma(clstr, dma, gic):
    setattr(
        clstr,
        dma.name,
        StreamDma(
            pio_addr=dma.address,
            status_addr=dma.statusAddress,
            pio_size=dma.pio,
            gic=gic,
        ),
    )
    obj = getattr(clstr, dma.name)
    obj.stream_addr = dma.address + dma.pio
    if dma.size is not None:
        obj.read_buffer_size = dma.size
        obj.write_buffer_size = dma.size
    if dma.maxReq is not None:
        obj.max_req_size = dma.maxReq
    obj.pio_delay = "1ns"
    if dma.rd_int != None:
        obj.rd_int = dma.rd_int
    if dma.wr_int != None:
        obj.wr_int = dma.wr_int
    obj.dma = clstr.coherency_bus.cpu_side_ports
    if dma.pio_masters is not None:
        for master in dma.pio_masters:
            getattr(clstr, master.lower()).mem_side_ports = obj.pio


def _attach_accelerator(clstr, acc, gic):
    ir = os.path.normpath(os.path.join(acc.working_dir, acc.ir_path))
    cycle_counts = config_parser.resolve_cycle_counts(
        acc.hw_config_document,
        acc.hw_config_path,
        acc.profile_name,
        acc.hw_config_source,
    )
    if acc.int_num is not None:
        setattr(
            clstr,
            acc.name,
            CommInterface(
                devicename=acc.name,
                gic=gic,
                pio_addr=acc.address,
                pio_size=acc.size,
                int_num=acc.int_num,
            ),
        )
    else:
        setattr(
            clstr,
            acc.name,
            CommInterface(
                devicename=acc.name,
                gic=gic,
                pio_addr=acc.address,
                pio_size=acc.size,
            ),
        )
    getattr(clstr, acc.name).clk_domain = clstr.compute_clk_domain
    configure_accelerator(getattr(clstr, acc.name), ir, cycle_counts)


def _connect_accelerator(clstr, acc):
    obj = getattr(clstr, acc.name)
    for connection in acc.local_connections:
        if "LocalBus" in connection:
            obj.local = clstr.local_bus.cpu_side_ports
        else:
            obj.local = getattr(clstr, connection.lower()).pio
    for master in acc.pio_masters:
        if "LocalBus" in master:
            obj.pio = clstr.local_bus.mem_side_ports
        else:
            assert False, "Shouldn't be here?"
    for inCon in acc.stream_in:
        obj.stream = getattr(clstr, inCon.lower()).stream_in
    for outCon in acc.stream_out:
        obj.stream = getattr(clstr, outCon.lower()).stream_out
    obj.enable_debug_msgs = acc.debug
    for var in acc.variables:
        _attach_variable(clstr, var)


def _attach_variable(clstr, var):
    if var.type == "Stream":
        params = dict(
            stream_address=var.address,
            status_address=var.statusAddress,
            stream_size=var.streamSize,
        )
        if var.bufferSize is not None:
            params["buffer_size"] = var.bufferSize
        setattr(clstr, var.name.lower(), StreamBuffer(**params))
        buf = getattr(clstr, var.name.lower())
        getattr(clstr, var.inCon).stream = buf.stream_in
        getattr(clstr, var.outCon).stream = buf.stream_out
    elif var.type == "SPM":
        spmRange = AddrRange(var.address, var.address + var.size)
        setattr(clstr, var.name.lower(), ScratchpadMemory(range=spmRange))
        spm = getattr(clstr, var.name.lower())
        if var.characterization:
            spm.ppa = SALAMPowerModel(**power_model.load(var.characterization))
        spm.conf_table_reported = False
        spm.ready_mode = var.readyMode
        spm.reset_on_scratchpad_read = var.resetOnRead
        spm.read_on_invalid = var.readOnInvalid
        spm.write_on_valid = var.writeOnValid
        spm.port = clstr.local_bus.mem_side_ports
        for con in var.connections:
            for i in range(con.numPorts):
                getattr(clstr, con.conName.lower()).spm = spm.spm_ports
    elif var.type == "RegisterBank":
        regRange = AddrRange(var.address, var.address + var.size)
        setattr(clstr, var.name.lower(), RegisterBank(range=regRange))
        reg = getattr(clstr, var.name.lower())
        if var.characterization:
            reg.ppa = SALAMPowerModel(**power_model.load(var.characterization))
        reg.load_port = clstr.local_bus.mem_side_ports
        for con in var.connections:
            getattr(clstr, con.conName.lower()).reg = reg.reg_port
    elif var.type == "Cache":
        # Keep normalized integer bytes in the workload model; MemorySize
        # expects a unit-bearing string at the SimObject boundary.
        setattr(clstr, var.name, AcceleratorCache(size=f"{var.size}B"))
        cache = getattr(clstr, var.name)
        cache.mem_side = clstr.coherency_bus.cpu_side_ports
        cache.cpu_side = getattr(clstr, var.accName.lower()).local
    else:
        raise Exception(
            "The variable: "
            + var.name
            + " has an invalid type named: "
            + var.type
        )


def configure_accelerator(acc, bench_file, cycle_counts):
    # Give a CommInterface its LLVM and hardware interfaces.
    # Initialize LLVMInterface Objects
    acc.llvm_interface = LLVMInterface()

    # The IR path is only the resource location. Profile identity is
    # resolved before this function runs.
    acc.llvm_interface.in_file = bench_file

    # Initialize HWInterface Objects
    acc.hw_interface = HWInterface()
    # Define HW Counts
    acc.hw_interface.cycle_counts = CycleCounts()

    if cycle_counts is not None:
        for instruction, cycles in cycle_counts.items():
            setattr(acc.hw_interface.cycle_counts, instruction, cycles)

    # Functional units and instructions of the build's hardware profile.
    acc.hw_interface.functional_units = FunctionalUnits.from_profile()
    acc.hw_interface.inst_config = InstConfig.from_profile()

    acc.hw_interface.salam_power_model = SALAMPowerModel(
        **FunctionalUnits.characterization()
    )
    acc.hw_interface.simulator_config = SimulatorConfig()
    acc.hw_interface.opcodes = InstOpCodes()
