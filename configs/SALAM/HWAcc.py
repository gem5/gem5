# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import ConfigParser
from HWAccConfig import *

import m5
from m5.objects import *
from m5.util import *


def makeHWAcc(options, system):
    # Specify the path to the benchmark file for an accelerator
    # acc_bench = <Absolute path to benchmark LLVM file>
    acc_bench = (
        options.accpath
        + "/"
        + options.accbench
        + "/bench/"
        + options.accbench
        + ".ll"
    )

    # Specify the path to the config file for an accelerator
    # acc_config = <Absolute path to the config file>
    # acc_config = options.accpath + "/" + options.accbench + "/config.ini"

    ################### Creating the Accelerator Cluster #####################
    # Create a new Accelerator Cluster
    system.acctest = AccCluster()
    local_low = 0x2F000000
    local_high = 0x2FFFFFFF
    local_range = AddrRange(local_low, local_high)
    external_range = [
        AddrRange(0x00000000, local_low - 1),
        AddrRange(local_high + 1, 0xFFFFFFFF),
    ]
    system.acctest._attach_bridges(system, local_range, external_range)
    system.acctest._connect_caches(system, options, l2coherent=True)

    ################### Adding Accelerators to Cluster #######################
    # Add an accelerator to the cluster
    system.acctest.acc = CommInterface(devicename=options.accbench)
    AccConfig(system.acctest.acc, acc_config, acc_bench)

    # Add an SPM for the accelerator
    system.acctest.acc_spm = ScratchpadMemory()
    system.acctest._connect_spm(system.acctest.acc_spm)
    system.acctest.acc_spm.reset_on_scratchpad_read = False

    # Connect the accelerator to the system's interrupt controller
    system.acctest.acc.gic = system.realview.gic

    # Connect HWAcc to cluster buses
    system.acctest._connect_hwacc(system.acctest.acc)
    system.acctest.acc.local = system.acctest.local_bus.cpu_side_ports
    system.acctest.acc.acp = system.acctest.coherency_bus.cpu_side_ports

    # Enable display of debug messages for the accelerator
    system.acctest.acc.enable_debug_msgs = False

    #################### Adding DMAs to Cluster ##############################
    # Add DMA devices to the cluster and connect them
    system.acctest.dma = NoncoherentDma(
        pio_addr=0x2FF00000,
        pio_size=24,
        gic=system.realview.gic,
        max_pending=32,
        int_num=95,
    )
    system.acctest._connect_cluster_dma(system, system.acctest.dma)

    system.acctest.stream_dma_0 = StreamDma(
        pio_addr=0x2FF10000,
        pio_size=32,
        gic=system.realview.gic,
        max_pending=32,
    )
    system.acctest.stream_dma_0.stream_in = system.acctest.acc.stream
    system.acctest.stream_dma_0.stream_out = system.acctest.acc.stream
    system.acctest.stream_dma_0.stream_addr = 0x2FF10020
    system.acctest.stream_dma_0.stream_size = 8
    system.acctest.stream_dma_0.pio_delay = "1ns"
    system.acctest.stream_dma_0.rd_int = 210
    system.acctest.stream_dma_0.wr_int = 211
    system.acctest._connect_dma(system, system.acctest.stream_dma_0)

    system.acctest.stream_dma_1 = StreamDma(
        pio_addr=0x2FF20000,
        pio_size=32,
        gic=system.realview.gic,
        max_pending=32,
    )
    system.acctest.stream_dma_1.stream_in = system.acctest.acc.stream
    system.acctest.stream_dma_1.stream_out = system.acctest.acc.stream
    system.acctest.stream_dma_1.stream_addr = 0x2FF20020
    system.acctest.stream_dma_1.stream_size = 8
    system.acctest.stream_dma_1.pio_delay = "1ns"
    system.acctest.stream_dma_1.rd_int = 212
    system.acctest.stream_dma_1.wr_int = 213
    system.acctest._connect_dma(system, system.acctest.stream_dma_1)
