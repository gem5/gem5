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

import os

from common.Caches import L1Cache
from HWAccConfig import AccConfig

from m5.objects import *
from m5.util import addToPath

_repo_root = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)
addToPath(
    os.path.join(_repo_root, "util", "SALAM-tools", "SALAM-Configurator")
)

import config_parser  # isort: skip


def makeHWAcc(options, system):
    working_dir = options.accpath
    config_name = getattr(options, "acccfg", "config.yml")
    clusters = config_parser.load_clusters(working_dir, config_name)
    for spec in clusters:
        setattr(system, spec.name.lower(), AccCluster())
        clstr = getattr(system, spec.name.lower())
        _attach_cluster(options, system, clstr, spec)


def _attach_cluster(options, system, clstr, spec):
    system.iobus.mem_side_ports = clstr.local_bus.cpu_side_ports
    clstr._connect_caches(system, options, l2coherent=False)
    gic = system.realview.gic

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
    obj.max_req_size = dma.maxReq
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
            max_pending=dma.pio,
        ),
    )
    obj = getattr(clstr, dma.name)
    obj.stream_addr = dma.address + dma.pio
    obj.stream_size = dma.size
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
    ir = acc.working_dir + "/" + acc.ir_path
    hw_config = acc.hw_config_path
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
    AccConfig(getattr(clstr, acc.name), ir, hw_config)


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
        setattr(
            clstr,
            var.name.lower(),
            StreamBuffer(
                stream_address=var.address,
                status_address=var.statusAddress,
                stream_size=var.streamSize,
                buffer_size=var.bufferSize,
            ),
        )
        buf = getattr(clstr, var.name.lower())
        getattr(clstr, var.inCon).stream = buf.stream_in
        getattr(clstr, var.outCon).stream = buf.stream_out
    elif var.type == "SPM":
        spmRange = AddrRange(var.address, var.address + var.size)
        setattr(clstr, var.name.lower(), ScratchpadMemory(range=spmRange))
        spm = getattr(clstr, var.name.lower())
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
        reg.load_port = clstr.local_bus.mem_side_ports
        for con in var.connections:
            getattr(clstr, con.conName.lower()).reg = reg.reg_port
    elif var.type == "Cache":
        setattr(clstr, var.name, L1Cache(size=str(var.size) + "B"))
        cache = getattr(clstr, var.name)
        cache.mem_side = clstr.coherency_bus.cpu_side_ports
        cache.cpu_side = getattr(clstr, var.accName).local
    else:
        raise Exception(
            "The variable: "
            + var.name
            + " has an invalid type named: "
            + var.type
        )
