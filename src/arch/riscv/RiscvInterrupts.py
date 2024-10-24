# Copyright (c) 2008 The Regents of The University of Michigan
# Copyright (c) 2014 Sven Karlsson
# Copyright (c) 2016 RISC-V Foundation
# Copyright (c) 2016 The University of Virginia
# Copyright (c) 2024 University of Rostock
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

from m5.citations import add_citation
from m5.objects.BaseInterrupts import BaseInterrupts
from m5.objects.IntPin import VectorIntSinkPin
from m5.params import (
    Param,
    VectorParam,
)


class RiscvInterrupts(BaseInterrupts):
    type = "RiscvInterrupts"
    cxx_class = "gem5::RiscvISA::Interrupts"
    cxx_header = "arch/riscv/interrupts.hh"

    local_interrupt_pins = VectorIntSinkPin("Pins for local interrupts")
    local_interrupt_ids = VectorParam.Unsigned(
        [], "list of local interrupt ids"
    )
    nmi_cause = Param.Int(0, "Non-maskable interrupt(NMI) cause")


add_citation(
    RiscvInterrupts,
    r"""@INPROCEEDINGS{Hauser:2024:LocalRiscvInterrupts,
  author    = {Hauser, Robert and
               Steffen, Lukas and
               Gr{\"u}tzmacher, Florian and
               Haubelt, Christian},
  booktitle = {MBMV 2024; 27. Workshop},
  title     = {Analyzing Local RISC-V Interrupt Latencies
               with Virtual Prototyping},
  year      = {2024},
  pages     = {7-14},
  address   = {Kaiserslautern, Germany},
  }
    """,
)
