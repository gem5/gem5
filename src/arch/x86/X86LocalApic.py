# Copyright (c) 2012 ARM Limited
# All rights reserved.
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
# Copyright (c) 2008 The Regents of The University of Michigan
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

from m5.defines import buildEnv
from m5.objects.BaseInterrupts import BaseInterrupts
from m5.objects.ClockDomain import DerivedClockDomain
from m5.objects.IntPin import IntSinkPin
from m5.params import *
from m5.proxy import *


class X86LocalApic(BaseInterrupts):
    type = "X86LocalApic"
    cxx_class = "gem5::X86ISA::Interrupts"
    cxx_header = "arch/x86/interrupts.hh"

    int_requestor = RequestPort("Port for sending interrupt messages")
    int_master = DeprecatedParam(
        int_requestor, "`int_master` is now called `int_requestor`"
    )

    int_responder = ResponsePort("Port for receiving interrupt messages")
    int_slave = DeprecatedParam(
        int_responder, "`int_slave` is now called `int_responder`"
    )

    lint0 = IntSinkPin("Local interrupt pin 0")
    lint1 = IntSinkPin("Local interrupt pin 1")

    int_latency = Param.Latency(
        "1ns", "Latency for an interrupt to propagate through this device."
    )
    pio = ResponsePort("Programmed I/O port")
    system = Param.System(Parent.any, "System this device is part of")

    pio_latency = Param.Latency("100ns", "Programmed IO latency")

    # The clock rate for the local APIC timer is supposed to be the "bus clock"
    # which we assume is 1/16th the rate of the CPU clock. I don't think this
    # is a hard rule, but seems to be true in practice. This can be overriden
    # in configs that use it.
    clk_domain = Param.DerivedClockDomain(
        DerivedClockDomain(clk_domain=Parent.clk_domain, clk_divider=16),
        "The clock for the local APIC. Should not be modified.",
    )
