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

# AUTO-GENERATED FILE (See util/SALAM-docs/README_SALAM.md for details)

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


# AUTO-GENERATED CLASSES (See util/SALAM-docs/README_SALAM.md for details)
class Add(SimObject):
    # SimObject type
    type = "Add"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/add.hh"
    # Instruction params
    functional_unit = Param.UInt32(1, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        13, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Addrspacecast(SimObject):
    # SimObject type
    type = "Addrspacecast"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/addrspacecast.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        50, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Alloca(SimObject):
    # SimObject type
    type = "Alloca"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/alloca.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        31, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class AndInst(SimObject):
    # SimObject type
    type = "AndInst"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/and_inst.hh"
    # Instruction params
    functional_unit = Param.UInt32(4, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        28, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Ashr(SimObject):
    # SimObject type
    type = "Ashr"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/ashr.hh"
    # Instruction params
    functional_unit = Param.UInt32(3, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        27, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Bitcast(SimObject):
    # SimObject type
    type = "Bitcast"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/bitcast.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        49, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Br(SimObject):
    # SimObject type
    type = "Br"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/br.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(2, "Default instruction llvm enum opcode value.")
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Call(SimObject):
    # SimObject type
    type = "Call"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/call.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        56, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Fadd(SimObject):
    # SimObject type
    type = "Fadd"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fadd.hh"
    # Instruction params
    functional_unit = Param.UInt32(6, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        14, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(5, "Default instruction runtime cycles.")


class Fcmp(SimObject):
    # SimObject type
    type = "Fcmp"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fcmp.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        54, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Fdiv(SimObject):
    # SimObject type
    type = "Fdiv"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fdiv.hh"
    # Instruction params
    functional_unit = Param.UInt32(10, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        21, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(16, "Default instruction runtime cycles.")


class Fence(SimObject):
    # SimObject type
    type = "Fence"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fence.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        35, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Fmul(SimObject):
    # SimObject type
    type = "Fmul"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fmul.hh"
    # Instruction params
    functional_unit = Param.UInt32(7, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        18, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(4, "Default instruction runtime cycles.")


class Fmuladd(SimObject):
    # SimObject type
    type = "Fmuladd"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fmuladd.hh"
    # Instruction params
    functional_unit = Param.UInt32(9, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        99, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(3, "Default instruction runtime cycles.")


class Fneg(SimObject):
    # SimObject type
    type = "Fneg"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fneg.hh"
    # Instruction params
    functional_unit = Param.UInt32(6, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        62, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Fpext(SimObject):
    # SimObject type
    type = "Fpext"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fpext.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        46, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Fptosi(SimObject):
    # SimObject type
    type = "Fptosi"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fptosi.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        42, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Fptoui(SimObject):
    # SimObject type
    type = "Fptoui"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fptoui.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        41, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Fptrunc(SimObject):
    # SimObject type
    type = "Fptrunc"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fptrunc.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        45, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Frem(SimObject):
    # SimObject type
    type = "Frem"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/frem.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        24, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(16, "Default instruction runtime cycles.")


class Fsub(SimObject):
    # SimObject type
    type = "Fsub"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/fsub.hh"
    # Instruction params
    functional_unit = Param.UInt32(6, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        16, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(5, "Default instruction runtime cycles.")


class Gep(SimObject):
    # SimObject type
    type = "Gep"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/gep.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        34, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Icmp(SimObject):
    # SimObject type
    type = "Icmp"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/icmp.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        53, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Indirectbr(SimObject):
    # SimObject type
    type = "Indirectbr"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/indirectbr.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(4, "Default instruction llvm enum opcode value.")
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Inttoptr(SimObject):
    # SimObject type
    type = "Inttoptr"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/inttoptr.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        48, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Invoke(SimObject):
    # SimObject type
    type = "Invoke"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/invoke.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(5, "Default instruction llvm enum opcode value.")
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Landingpad(SimObject):
    # SimObject type
    type = "Landingpad"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/landingpad.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        66, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Load(SimObject):
    # SimObject type
    type = "Load"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/load.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        32, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Lshr(SimObject):
    # SimObject type
    type = "Lshr"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/lshr.hh"
    # Instruction params
    functional_unit = Param.UInt32(3, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        26, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Mul(SimObject):
    # SimObject type
    type = "Mul"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/mul.hh"
    # Instruction params
    functional_unit = Param.UInt32(2, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        17, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class OrInst(SimObject):
    # SimObject type
    type = "OrInst"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/or_inst.hh"
    # Instruction params
    functional_unit = Param.UInt32(4, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        29, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Phi(SimObject):
    # SimObject type
    type = "Phi"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/phi.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        55, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Ptrtoint(SimObject):
    # SimObject type
    type = "Ptrtoint"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/ptrtoint.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        47, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Resume(SimObject):
    # SimObject type
    type = "Resume"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/resume.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(6, "Default instruction llvm enum opcode value.")
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Ret(SimObject):
    # SimObject type
    type = "Ret"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/ret.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(1, "Default instruction llvm enum opcode value.")
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Sdiv(SimObject):
    # SimObject type
    type = "Sdiv"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/sdiv.hh"
    # Instruction params
    functional_unit = Param.UInt32(2, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        20, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Select(SimObject):
    # SimObject type
    type = "Select"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/select.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        57, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Sext(SimObject):
    # SimObject type
    type = "Sext"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/sext.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        40, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Shl(SimObject):
    # SimObject type
    type = "Shl"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/shl.hh"
    # Instruction params
    functional_unit = Param.UInt32(3, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        25, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Srem(SimObject):
    # SimObject type
    type = "Srem"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/srem.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        23, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Store(SimObject):
    # SimObject type
    type = "Store"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/store.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        33, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Sub(SimObject):
    # SimObject type
    type = "Sub"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/sub.hh"
    # Instruction params
    functional_unit = Param.UInt32(1, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        15, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class SwitchInst(SimObject):
    # SimObject type
    type = "SwitchInst"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/switch_inst.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(3, "Default instruction llvm enum opcode value.")
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Trunc(SimObject):
    # SimObject type
    type = "Trunc"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/trunc.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        38, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Udiv(SimObject):
    # SimObject type
    type = "Udiv"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/udiv.hh"
    # Instruction params
    functional_unit = Param.UInt32(2, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        19, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Uitofp(SimObject):
    # SimObject type
    type = "Uitofp"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/uitofp.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        43, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Unreachable(SimObject):
    # SimObject type
    type = "Unreachable"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/unreachable.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(7, "Default instruction llvm enum opcode value.")
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class Urem(SimObject):
    # SimObject type
    type = "Urem"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/urem.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        22, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Vaarg(SimObject):
    # SimObject type
    type = "Vaarg"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/vaarg.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        60, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(0, "Default instruction runtime cycles.")


class XorInst(SimObject):
    # SimObject type
    type = "XorInst"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/xor_inst.hh"
    # Instruction params
    functional_unit = Param.UInt32(4, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        30, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class Zext(SimObject):
    # SimObject type
    type = "Zext"  # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instructions/zext.hh"
    # Instruction params
    functional_unit = Param.UInt32(0, "Default functional unit assignment.")
    functional_unit_limit = Param.UInt32(0, "Default functional unit limit.")
    opcode_num = Param.UInt32(
        39, "Default instruction llvm enum opcode value."
    )
    runtime_cycles = Param.UInt32(1, "Default instruction runtime cycles.")


class InstConfig(SimObject):
    # SimObject type
    type = "InstConfig"
    # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/instruction_config.hh"

    add = Param.Add(Parent.any, "add instruction SimObject")
    addrspacecast = Param.Addrspacecast(
        Parent.any, "addrspacecast instruction SimObject"
    )
    alloca = Param.Alloca(Parent.any, "alloca instruction SimObject")
    and_inst = Param.AndInst(Parent.any, "and_inst instruction SimObject")
    ashr = Param.Ashr(Parent.any, "ashr instruction SimObject")
    bitcast = Param.Bitcast(Parent.any, "bitcast instruction SimObject")
    br = Param.Br(Parent.any, "br instruction SimObject")
    call = Param.Call(Parent.any, "call instruction SimObject")
    fadd = Param.Fadd(Parent.any, "fadd instruction SimObject")
    fcmp = Param.Fcmp(Parent.any, "fcmp instruction SimObject")
    fdiv = Param.Fdiv(Parent.any, "fdiv instruction SimObject")
    fence = Param.Fence(Parent.any, "fence instruction SimObject")
    fmul = Param.Fmul(Parent.any, "fmul instruction SimObject")
    fmuladd = Param.Fmuladd(Parent.any, "fmuladd instruction SimObject")
    fneg = Param.Fneg(Parent.any, "fneg instruction SimObject")
    fpext = Param.Fpext(Parent.any, "fpext instruction SimObject")
    fptosi = Param.Fptosi(Parent.any, "fptosi instruction SimObject")
    fptoui = Param.Fptoui(Parent.any, "fptoui instruction SimObject")
    fptrunc = Param.Fptrunc(Parent.any, "fptrunc instruction SimObject")
    frem = Param.Frem(Parent.any, "frem instruction SimObject")
    fsub = Param.Fsub(Parent.any, "fsub instruction SimObject")
    gep = Param.Gep(Parent.any, "gep instruction SimObject")
    icmp = Param.Icmp(Parent.any, "icmp instruction SimObject")
    indirectbr = Param.Indirectbr(
        Parent.any, "indirectbr instruction SimObject"
    )
    inttoptr = Param.Inttoptr(Parent.any, "inttoptr instruction SimObject")
    invoke = Param.Invoke(Parent.any, "invoke instruction SimObject")
    landingpad = Param.Landingpad(
        Parent.any, "landingpad instruction SimObject"
    )
    load = Param.Load(Parent.any, "load instruction SimObject")
    lshr = Param.Lshr(Parent.any, "lshr instruction SimObject")
    mul = Param.Mul(Parent.any, "mul instruction SimObject")
    or_inst = Param.OrInst(Parent.any, "or_inst instruction SimObject")
    phi = Param.Phi(Parent.any, "phi instruction SimObject")
    ptrtoint = Param.Ptrtoint(Parent.any, "ptrtoint instruction SimObject")
    resume = Param.Resume(Parent.any, "resume instruction SimObject")
    ret = Param.Ret(Parent.any, "ret instruction SimObject")
    sdiv = Param.Sdiv(Parent.any, "sdiv instruction SimObject")
    select = Param.Select(Parent.any, "select instruction SimObject")
    sext = Param.Sext(Parent.any, "sext instruction SimObject")
    shl = Param.Shl(Parent.any, "shl instruction SimObject")
    srem = Param.Srem(Parent.any, "srem instruction SimObject")
    store = Param.Store(Parent.any, "store instruction SimObject")
    sub = Param.Sub(Parent.any, "sub instruction SimObject")
    switch_inst = Param.SwitchInst(
        Parent.any, "switch_inst instruction SimObject"
    )
    trunc = Param.Trunc(Parent.any, "trunc instruction SimObject")
    udiv = Param.Udiv(Parent.any, "udiv instruction SimObject")
    uitofp = Param.Uitofp(Parent.any, "uitofp instruction SimObject")
    unreachable = Param.Unreachable(
        Parent.any, "unreachable instruction SimObject"
    )
    urem = Param.Urem(Parent.any, "urem instruction SimObject")
    vaarg = Param.Vaarg(Parent.any, "vaarg instruction SimObject")
    xor_inst = Param.XorInst(Parent.any, "xor_inst instruction SimObject")
    zext = Param.Zext(Parent.any, "zext instruction SimObject")
