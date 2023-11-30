# Copyright 2020 Google, Inc.
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

from m5.objects.FastModel import (
    AmbaInitiatorSocket,
    AmbaTargetSocket,
)
from m5.objects.IntPin import IntSourcePin
from m5.objects.ResetPort import ResetResponsePort
from m5.objects.SystemC import SystemC_ScModule
from m5.params import *


class FastModelPL330(SystemC_ScModule):
    type = "FastModelPL330"
    cxx_class = "gem5::fastmodel::PL330"
    cxx_header = "arch/arm/fastmodel/PL330_DMAC/pl330.hh"

    clock = Param.Frequency("Clock frequency")

    irq_0 = IntSourcePin("Sets when DMASEV 0")
    irq_1 = IntSourcePin("Sets when DMASEV 1")
    irq_2 = IntSourcePin("Sets when DMASEV 2")
    irq_3 = IntSourcePin("Sets when DMASEV 3")
    irq_4 = IntSourcePin("Sets when DMASEV 4")
    irq_5 = IntSourcePin("Sets when DMASEV 5")
    irq_6 = IntSourcePin("Sets when DMASEV 6")
    irq_7 = IntSourcePin("Sets when DMASEV 7")
    irq_8 = IntSourcePin("Sets when DMASEV 8")
    irq_9 = IntSourcePin("Sets when DMASEV 9")
    irq_10 = IntSourcePin("Sets when DMASEV 10")
    irq_11 = IntSourcePin("Sets when DMASEV 11")
    irq_12 = IntSourcePin("Sets when DMASEV 12")
    irq_13 = IntSourcePin("Sets when DMASEV 13")
    irq_14 = IntSourcePin("Sets when DMASEV 14")
    irq_15 = IntSourcePin("Sets when DMASEV 15")
    irq_16 = IntSourcePin("Sets when DMASEV 16")
    irq_17 = IntSourcePin("Sets when DMASEV 17")
    irq_18 = IntSourcePin("Sets when DMASEV 18")
    irq_19 = IntSourcePin("Sets when DMASEV 19")
    irq_20 = IntSourcePin("Sets when DMASEV 20")
    irq_21 = IntSourcePin("Sets when DMASEV 21")
    irq_22 = IntSourcePin("Sets when DMASEV 22")
    irq_23 = IntSourcePin("Sets when DMASEV 23")
    irq_24 = IntSourcePin("Sets when DMASEV 24")
    irq_25 = IntSourcePin("Sets when DMASEV 25")
    irq_26 = IntSourcePin("Sets when DMASEV 26")
    irq_27 = IntSourcePin("Sets when DMASEV 27")
    irq_28 = IntSourcePin("Sets when DMASEV 28")
    irq_29 = IntSourcePin("Sets when DMASEV 29")
    irq_30 = IntSourcePin("Sets when DMASEV 30")
    irq_31 = IntSourcePin("Sets when DMASEV 31")
    irq_abort = IntSourcePin("Undefined instruction or instruction error")

    fifo_size = Param.UInt32(16, "Channel FIFO size in bytes")
    max_transfer = Param.UInt32(256, "Largest atomic transfer")
    generate_clear = Param.Bool(False, "Generate clear response")
    activate_delay = Param.UInt32(0, "request delay")
    revision = Param.String("r0p0", "revision ID")

    max_irqs = Param.UInt32(32, "number of interrupts")
    buffer_depth = Param.UInt32(16, "buffer depth")
    lsq_read_size = Param.UInt32(4, "LSQ read buffer depth")
    lsq_write_size = Param.UInt32(4, "LSQ write buffer depth")
    read_issuing_capability = Param.UInt32(1, "AXI read issuing capability")
    write_issuing_capability = Param.UInt32(1, "AXI write issuing capability")
    axi_bus_width = Param.UInt32(32, "AXI bus width")
    cache_line_words = Param.UInt32(1, "number of words in a cache line")
    cache_lines = Param.UInt32(1, "number of cache lines")
    max_channels = Param.UInt32(8, "virtual channels")
    controller_nsecure = Param.Bool(
        False, "Controller non-secure at reset (boot_manager_ns)"
    )
    irq_nsecure = Param.UInt32(0, "Interrupts non-secure at reset")
    periph_nsecure = Param.Bool(False, "Peripherals non-secure at reset")
    controller_boots = Param.Bool(True, "DMA boots from reset")
    reset_pc = Param.UInt32(0x60000000, "DMA PC at reset")
    max_periph = Param.UInt32(32, "number of peripheral interfaces")
    perip_request_acceptance_0 = Param.UInt32(
        2, "Peripheral 0 request acceptance"
    )
    perip_request_acceptance_1 = Param.UInt32(
        2, "Peripheral 1 request acceptance"
    )
    perip_request_acceptance_2 = Param.UInt32(
        2, "Peripheral 2 request acceptance"
    )
    perip_request_acceptance_3 = Param.UInt32(
        2, "Peripheral 3 request acceptance"
    )
    perip_request_acceptance_4 = Param.UInt32(
        2, "Peripheral 4 request acceptance"
    )
    perip_request_acceptance_5 = Param.UInt32(
        2, "Peripheral 5 request acceptance"
    )
    perip_request_acceptance_6 = Param.UInt32(
        2, "Peripheral 6 request acceptance"
    )
    perip_request_acceptance_7 = Param.UInt32(
        2, "Peripheral 7 request acceptance"
    )
    perip_request_acceptance_8 = Param.UInt32(
        2, "Peripheral 8 request acceptance"
    )
    perip_request_acceptance_9 = Param.UInt32(
        2, "Peripheral 9 request acceptance"
    )
    perip_request_acceptance_10 = Param.UInt32(
        2, "Peripheral 10 request acceptance"
    )
    perip_request_acceptance_11 = Param.UInt32(
        2, "Peripheral 11 request acceptance"
    )
    perip_request_acceptance_12 = Param.UInt32(
        2, "Peripheral 12 request acceptance"
    )
    perip_request_acceptance_13 = Param.UInt32(
        2, "Peripheral 13 request acceptance"
    )
    perip_request_acceptance_14 = Param.UInt32(
        2, "Peripheral 14 request acceptance"
    )
    perip_request_acceptance_15 = Param.UInt32(
        2, "Peripheral 15 request acceptance"
    )
    perip_request_acceptance_16 = Param.UInt32(
        2, "Peripheral 16 request acceptance"
    )
    perip_request_acceptance_17 = Param.UInt32(
        2, "Peripheral 17 request acceptance"
    )
    perip_request_acceptance_18 = Param.UInt32(
        2, "Peripheral 18 request acceptance"
    )
    perip_request_acceptance_19 = Param.UInt32(
        2, "Peripheral 19 request acceptance"
    )
    perip_request_acceptance_20 = Param.UInt32(
        2, "Peripheral 20 request acceptance"
    )
    perip_request_acceptance_21 = Param.UInt32(
        2, "Peripheral 21 request acceptance"
    )
    perip_request_acceptance_22 = Param.UInt32(
        2, "Peripheral 22 request acceptance"
    )
    perip_request_acceptance_23 = Param.UInt32(
        2, "Peripheral 23 request acceptance"
    )
    perip_request_acceptance_24 = Param.UInt32(
        2, "Peripheral 24 request acceptance"
    )
    perip_request_acceptance_25 = Param.UInt32(
        2, "Peripheral 25 request acceptance"
    )
    perip_request_acceptance_26 = Param.UInt32(
        2, "Peripheral 26 request acceptance"
    )
    perip_request_acceptance_27 = Param.UInt32(
        2, "Peripheral 27 request acceptance"
    )
    perip_request_acceptance_28 = Param.UInt32(
        2, "Peripheral 28 request acceptance"
    )
    perip_request_acceptance_29 = Param.UInt32(
        2, "Peripheral 29 request acceptance"
    )
    perip_request_acceptance_30 = Param.UInt32(
        2, "Peripheral 30 request acceptance"
    )
    perip_request_acceptance_31 = Param.UInt32(
        2, "Peripheral 31 request acceptance"
    )

    # Singleton IRQ abort signal port
    # 32 bit wide IRQ master DMASEV port
    dma = AmbaInitiatorSocket(64, "Memory accesses")
    pio_s = AmbaTargetSocket(64, "Register accesses (secure)")
    pio_ns = AmbaTargetSocket(64, "Register accesses (non-secure)")

    reset_in = ResetResponsePort("System reset")

    # irq_abort_master_port
    # irq_master_port
    # pvbus_m
    # pvbus_s
    # pvbus_s_ns
