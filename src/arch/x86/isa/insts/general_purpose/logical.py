# Copyright (c) 2007 The Hewlett-Packard Development Company
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

microcode = """
def macroop OR_R_R
{
    or reg, reg, regm, flags=(OF,SF,ZF,PF,CF)
};

def macroop OR_M_I
{
    limm t2, imm
    ldst t1, seg, sib, disp
    or t1, t1, t2, flags=(OF,SF,ZF,PF,CF)
    st t1, seg, sib, disp
};

def macroop OR_P_I
{
    limm t2, imm
    rdip t7
    ldst t1, seg, riprel, disp
    or t1, t1, t2, flags=(OF,SF,ZF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop OR_LOCKED_M_I
{
    limm t2, imm
    mfence
    ldstl t1, seg, sib, disp
    or t1, t1, t2, flags=(OF,SF,ZF,PF,CF)
    stul t1, seg, sib, disp
    mfence
};

def macroop OR_LOCKED_P_I
{
    limm t2, imm
    rdip t7
    mfence
    ldstl t1, seg, riprel, disp
    or t1, t1, t2, flags=(OF,SF,ZF,PF,CF)
    stul t1, seg, riprel, disp
    mfence
};

def macroop OR_M_R
{
    ldst t1, seg, sib, disp
    or t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    st t1, seg, sib, disp
};

def macroop OR_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    or t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop OR_LOCKED_M_R
{
    mfence
    ldstl t1, seg, sib, disp
    or t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    stul t1, seg, sib, disp
    mfence
};

def macroop OR_LOCKED_P_R
{
    rdip t7
    mfence
    ldstl t1, seg, riprel, disp
    or t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    stul t1, seg, riprel, disp
    mfence
};

def macroop OR_R_M
{
    ld t1, seg, sib, disp
    or reg, reg, t1, flags=(OF,SF,ZF,PF,CF)
};

def macroop OR_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    or reg, reg, t1, flags=(OF,SF,ZF,PF,CF)
};

def macroop OR_R_I
{
    limm t1, imm
    or reg, reg, t1, flags=(OF,SF,ZF,PF,CF)
};

def macroop XOR_R_R
{
    xor reg, reg, regm, flags=(OF,SF,ZF,PF,CF)
};

def macroop XOR_R_I
{
    limm t1, imm
    xor reg, reg, t1, flags=(OF,SF,ZF,PF,CF)
};

def macroop XOR_M_I
{
    limm t2, imm
    ldst t1, seg, sib, disp
    xor t1, t1, t2, flags=(OF,SF,ZF,PF,CF)
    st t1, seg, sib, disp
};

def macroop XOR_P_I
{
    limm t2, imm
    rdip t7
    ldst t1, seg, riprel, disp
    xor t1, t1, t2, flags=(OF,SF,ZF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop XOR_LOCKED_M_I
{
    limm t2, imm
    mfence
    ldstl t1, seg, sib, disp
    xor t1, t1, t2, flags=(OF,SF,ZF,PF,CF)
    stul t1, seg, sib, disp
    mfence
};

def macroop XOR_LOCKED_P_I
{
    limm t2, imm
    rdip t7
    mfence
    ldstl t1, seg, riprel, disp
    xor t1, t1, t2, flags=(OF,SF,ZF,PF,CF)
    stul t1, seg, riprel, disp
    mfence
};

def macroop XOR_M_R
{
    ldst t1, seg, sib, disp
    xor t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    st t1, seg, sib, disp
};

def macroop XOR_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    xor t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop XOR_LOCKED_M_R
{
    mfence
    ldstl t1, seg, sib, disp
    xor t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    stul t1, seg, sib, disp
    mfence
};

def macroop XOR_LOCKED_P_R
{
    rdip t7
    mfence
    ldstl t1, seg, riprel, disp
    xor t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    stul t1, seg, riprel, disp
    mfence
};

def macroop XOR_R_M
{
    ld t1, seg, sib, disp
    xor reg, reg, t1, flags=(OF,SF,ZF,PF,CF)
};

def macroop XOR_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    xor reg, reg, t1, flags=(OF,SF,ZF,PF,CF)
};

def macroop AND_R_R
{
    and reg, reg, regm, flags=(OF,SF,ZF,PF,CF)
};

def macroop AND_R_M
{
    ld t1, seg, sib, disp
    and reg, reg, t1, flags=(OF,SF,ZF,PF,CF)
};

def macroop AND_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    and reg, reg, t1, flags=(OF,SF,ZF,PF,CF)
};

def macroop AND_R_I
{
    limm t1, imm
    and reg, reg, t1, flags=(OF,SF,ZF,PF,CF)
};

def macroop AND_M_I
{
    ldst t2, seg, sib, disp
    limm t1, imm
    and t2, t2, t1, flags=(OF,SF,ZF,PF,CF)
    st t2, seg, sib, disp
};

def macroop AND_P_I
{
    rdip t7
    ldst t2, seg, riprel, disp
    limm t1, imm
    and t2, t2, t1, flags=(OF,SF,ZF,PF,CF)
    st t2, seg, riprel, disp
};

def macroop AND_LOCKED_M_I
{
    mfence
    ldstl t2, seg, sib, disp
    limm t1, imm
    and t2, t2, t1, flags=(OF,SF,ZF,PF,CF)
    stul t2, seg, sib, disp
    mfence
};

def macroop AND_LOCKED_P_I
{
    rdip t7
    mfence
    ldstl t2, seg, riprel, disp
    limm t1, imm
    and t2, t2, t1, flags=(OF,SF,ZF,PF,CF)
    stul t2, seg, riprel, disp
    mfence
};

def macroop AND_M_R
{
    ldst t1, seg, sib, disp
    and t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    st t1, seg, sib, disp
};

def macroop AND_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    and t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop AND_LOCKED_M_R
{
    mfence
    ldstl t1, seg, sib, disp
    and t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    stul t1, seg, sib, disp
    mfence
};

def macroop AND_LOCKED_P_R
{
    rdip t7
    mfence
    ldstl t1, seg, riprel, disp
    and t1, t1, reg, flags=(OF,SF,ZF,PF,CF)
    stul t1, seg, riprel, disp
    mfence
};

def macroop NOT_R
{
    limm t1, -1
    xor reg, reg, t1
};

def macroop NOT_M
{
    limm t1, -1
    ldst t2, seg, sib, disp
    xor t2, t2, t1
    st t2, seg, sib, disp
};

def macroop NOT_P
{
    limm t1, -1
    rdip t7
    ldst t2, seg, riprel, disp
    xor t2, t2, t1
    st t2, seg, riprel, disp
};

def macroop NOT_LOCKED_M
{
    limm t1, -1
    mfence
    ldstl t2, seg, sib, disp
    xor t2, t2, t1
    stul t2, seg, sib, disp
    mfence
};

def macroop NOT_LOCKED_P
{
    limm t1, -1
    rdip t7
    mfence
    ldstl t2, seg, riprel, disp
    xor t2, t2, t1
    stul t2, seg, riprel, disp
    mfence
};
"""
