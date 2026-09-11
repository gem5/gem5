# Copyright (c) 2026 The Board of Trustees of the Leland Stanford
# Junior University
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

import os

from m5.defines import buildEnv
from m5.objects.BaseCPU import BaseCPU
from m5.objects.X86CPU import X86CPU
from m5.objects.X86MMU import X86MMU
from m5.params import *
from m5.SimObject import *


class X86PinCPU(BaseCPU, X86CPU):
    type = "X86PinCPU"
    cxx_header = "arch/x86/pin/cpu.hh"
    cxx_class = "gem5::X86ISA::PinCPU"

    mmu = X86MMU()

    @classmethod
    def memory_mode(cls):
        return "atomic"

    @classmethod
    def support_take_over(cls):
        return False

    @cxxMethod
    def executePinCommand(command):
        pass

    pinExe = Param.String(
        os.path.join(buildEnv["PIN_DIR"], "pin"),
        "Path to Intel Pin executable",
    )
    pinGuest = Param.String(
        "",
        "Path to the guest program. If empty, it is looked up next to the "
        "gem5 binary, where the build places it.",
    )
    pinTool = Param.String(
        "",
        "Path to the host PinTool. If empty, it is looked up next to the "
        "gem5 binary, where the build places it.",
    )
    pinToolArgs = Param.String("", "Arguments to pass to PinTool")
    pinArgs = Param.String("", "Arguments to pass to Pin")

    countInsts = Param.Bool(
        False, "Enable instruction counting (moderate performance penalty)"
    )
