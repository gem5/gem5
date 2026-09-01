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

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class InstOpCodes(SimObject):
    # SimObject type
    type = "InstOpCodes"
    # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/opcodes.hh"
    # Default Values Below are LLVM Instruction Enumerations from
    # llvm/IR/Instructions.def
    gep = Param.UInt32(34, "GetElementPtr operation.")
    phi = Param.UInt32(55, "Phi operation.")
    select = Param.UInt32(57, "Select operation.")
    ret = Param.UInt32(1, "Return operation.")
    br = Param.UInt32(2, "Branch operation.")
    switch_inst = Param.UInt32(3, "Switch operation.")
    indirectbr = Param.UInt32(4, "Indirect Branch operation.")
    invoke = Param.UInt32(5, "Invoke operation.")
    resume = Param.UInt32(6, "Resume operation.")
    unreachable = Param.UInt32(7, "Unreachable operation.")
    icmp = Param.UInt32(53, "Integer compare operation.")
    fcmp = Param.UInt32(54, "Floating point compare operation.")
    trunc = Param.UInt32(38, "Truncate operation.")
    zext = Param.UInt32(39, "Zero extend operation.")
    sext = Param.UInt32(40, "Sign extend operation.")
    fptrunc = Param.UInt32(45, "Floating point truncate operation.")
    fpext = Param.UInt32(46, "Floating point extend operation.")
    fptoui = Param.UInt32(41, "Floating point to unsigned integer operation.")
    fptosi = Param.UInt32(42, "Floating point to signed integer operation.")
    uitofp = Param.UInt32(43, "Unsigned integer to floating point operation.")
    ptrtoint = Param.UInt32(47, "Pointer to integer operation.")
    inttoptr = Param.UInt32(48, "Integer to pointer operation.")
    bitcast = Param.UInt32(49, "Bitcast operation.")
    addrspacecast = Param.UInt32(50, "Address space cast operation.")
    call = Param.UInt32(56, "Call operation.")
    vaarg = Param.UInt32(60, "Vaarg operation.")
    landingpad = Param.UInt32(66, "Landing pad operation.")
    alloca = Param.UInt32(31, "Allocate operation.")
    load = Param.UInt32(32, "Must be 0, handled by memory controller")
    store = Param.UInt32(33, "Must be 0, handled by memory controller")
    fence = Param.UInt32(35, "Fence operation.")
    shl = Param.UInt32(25, "Shift left operation.")
    lshr = Param.UInt32(26, "Logical shift right operation.")
    ashr = Param.UInt32(27, "Arithmetic shift right operation.")
    and_inst = Param.UInt32(28, "And operation.")
    or_inst = Param.UInt32(29, "Or operation.")
    xor_inst = Param.UInt32(30, "Xor operation.")
    add = Param.UInt32(13, "Integer add operation.")
    sub = Param.UInt32(15, "Integer subtract operation.")
    mul = Param.UInt32(17, "Integer multiply operation.")
    udiv = Param.UInt32(19, "Unsigned integer division operation.")
    sdiv = Param.UInt32(20, "Signed integer division operation.")
    urem = Param.UInt32(22, "Unsigned remainder operation.")
    srem = Param.UInt32(23, "Signed remainder operation.")
    fadd = Param.UInt32(14, "Floating point addition operation.")
    fsub = Param.UInt32(16, "Floating point subtraction operation.")
    fmul = Param.UInt32(18, "Floating point multiplication operation.")
    fdiv = Param.UInt32(21, "Floating point division operation.")
    frem = Param.UInt32(24, "Floating point remainder operation.")
