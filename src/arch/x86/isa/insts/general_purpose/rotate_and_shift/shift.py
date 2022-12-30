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
def macroop SAL_R_I
{
    slli reg, reg, imm, flags=(CF,OF,SF,ZF,PF)
};

def macroop SAL_M_I
{
    ldst t1, seg, sib, disp
    slli t1, t1, imm, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SAL_P_I
{
    rdip t7
    ldst t1, seg, riprel, disp
    slli t1, t1, imm, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SAL_1_R
{
    slli reg, reg, 1, flags=(CF,OF,SF,ZF,PF)
};

def macroop SAL_1_M
{
    ldst t1, seg, sib, disp
    slli t1, t1, 1, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SAL_1_P
{
    rdip t7
    ldst t1, seg, riprel, disp
    slli t1, t1, 1, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SAL_R_R
{
    sll reg, reg, regm, flags=(CF,OF,SF,ZF,PF)
};

def macroop SAL_M_R
{
    ldst t1, seg, sib, disp
    sll t1, t1, reg, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SAL_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    sll t1, t1, reg, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SHLD_R_R
{
    mdbi regm, 0
    sld reg, reg, rcx, flags=(CF,OF,SF,ZF,PF)
};

def macroop SHLD_M_R
{
    ldst t1, seg, sib, disp
    mdbi reg, 0
    sld t1, t1, rcx, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SHLD_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    mdbi reg, 0
    sld t1, t1, rcx, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SHLD_R_R_I
{
    mdbi regm, 0
    sldi reg, reg, imm, flags=(CF,OF,SF,ZF,PF)
};

def macroop SHLD_M_R_I
{
    ldst t1, seg, sib, disp
    mdbi reg, 0
    sldi t1, t1, imm, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SHLD_P_R_I
{
    rdip t7
    ldst t1, seg, riprel, disp
    mdbi reg, 0
    sldi t1, t1, imm, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SHR_R_I
{
    srli reg, reg, imm, flags=(CF,OF,SF,ZF,PF)
};

def macroop SHR_M_I
{
    ldst t1, seg, sib, disp
    srli t1, t1, imm, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SHR_P_I
{
    rdip t7
    ldst t1, seg, riprel, disp
    srli t1, t1, imm, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SHR_1_R
{
    srli reg, reg, 1, flags=(CF,OF,SF,ZF,PF)
};

def macroop SHR_1_M
{
    ldst t1, seg, sib, disp
    srli t1, t1, 1, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SHR_1_P
{
    rdip t7
    ldst t1, seg, riprel, disp
    srli t1, t1, 1, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SHR_R_R
{
    srl reg, reg, regm, flags=(CF,OF,SF,ZF,PF)
};

def macroop SHR_M_R
{
    ldst t1, seg, sib, disp
    srl t1, t1, reg, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SHR_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    srl t1, t1, reg, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SHRD_R_R
{
    mdbi regm, 0
    srd reg, reg, rcx, flags=(CF,OF,SF,ZF,PF)
};

def macroop SHRD_M_R
{
    ldst t1, seg, sib, disp
    mdbi reg, 0
    srd t1, t1, rcx, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SHRD_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    mdbi reg, 0
    srd t1, t1, rcx, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SHRD_R_R_I
{
    mdbi regm, 0
    srdi reg, reg, imm, flags=(CF,OF,SF,ZF,PF)
};

def macroop SHRD_M_R_I
{
    ldst t1, seg, sib, disp
    mdbi reg, 0
    srdi t1, t1, imm, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SHRD_P_R_I
{
    rdip t7
    ldst t1, seg, riprel, disp
    mdbi reg, 0
    srdi t1, t1, imm, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SAR_R_I
{
    srai reg, reg, imm, flags=(CF,OF,SF,ZF,PF)
};

def macroop SAR_M_I
{
    ldst t1, seg, sib, disp
    srai t1, t1, imm, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SAR_P_I
{
    rdip t7
    ldst t1, seg, riprel, disp
    srai t1, t1, imm, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SAR_1_R
{
    srai reg, reg, 1, flags=(CF,OF,SF,ZF,PF)
};

def macroop SAR_1_M
{
    ldst t1, seg, sib, disp
    srai t1, t1, 1, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SAR_1_P
{
    rdip t7
    ldst t1, seg, riprel, disp
    srai t1, t1, 1, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};

def macroop SAR_R_R
{
    sra reg, reg, regm, flags=(CF,OF,SF,ZF,PF)
};

def macroop SAR_M_R
{
    ldst t1, seg, sib, disp
    sra t1, t1, reg, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, sib, disp
};

def macroop SAR_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    sra t1, t1, reg, flags=(CF,OF,SF,ZF,PF)
    st t1, seg, riprel, disp
};
"""
