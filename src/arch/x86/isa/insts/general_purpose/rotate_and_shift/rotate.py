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
def macroop ROL_R_I
{
    roli reg, reg, imm, flags=(OF,CF)
};

def macroop ROL_M_I
{
    ldst t1, seg, sib, disp
    roli t1, t1, imm, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop ROL_P_I
{
    rdip t7
    ldst t1, seg, riprel, disp
    roli t1, t1, imm, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop ROL_1_R
{
    roli reg, reg, 1, flags=(OF,CF)
};

def macroop ROL_1_M
{
    ldst t1, seg, sib, disp
    roli t1, t1, 1, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop ROL_1_P
{
    rdip t7
    ldst t1, seg, riprel, disp
    roli t1, t1, 1, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop ROL_R_R
{
    rol reg, reg, regm, flags=(OF,CF)
};

def macroop ROL_M_R
{
    ldst t1, seg, sib, disp
    rol t1, t1, reg, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop ROL_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    rol t1, t1, reg, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop ROR_R_I
{
    rori reg, reg, imm, flags=(OF,CF)
};

def macroop ROR_M_I
{
    ldst t1, seg, sib, disp
    rori t1, t1, imm, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop ROR_P_I
{
    rdip t7
    ldst t1, seg, riprel, disp
    rori t1, t1, imm, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop ROR_1_R
{
    rori reg, reg, 1, flags=(OF,CF)
};

def macroop ROR_1_M
{
    ldst t1, seg, sib, disp
    rori t1, t1, 1, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop ROR_1_P
{
    rdip t7
    ldst t1, seg, riprel, disp
    rori t1, t1, 1, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop ROR_R_R
{
    ror reg, reg, regm, flags=(OF,CF)
};

def macroop ROR_M_R
{
    ldst t1, seg, sib, disp
    ror t1, t1, reg, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop ROR_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    ror t1, t1, reg, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop RCL_R_I
{
    rcli reg, reg, imm, flags=(OF,CF)
};

def macroop RCL_M_I
{
    ldst t1, seg, sib, disp
    rcli t1, t1, imm, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop RCL_P_I
{
    rdip t7
    ldst t1, seg, riprel, disp
    rcli t1, t1, imm, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop RCL_1_R
{
    rcli reg, reg, 1, flags=(OF,CF)
};

def macroop RCL_1_M
{
    ldst t1, seg, sib, disp
    rcli t1, t1, 1, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop RCL_1_P
{
    rdip t7
    ldst t1, seg, riprel, disp
    rcli t1, t1, 1, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop RCL_R_R
{
    rcl reg, reg, regm, flags=(OF,CF)
};

def macroop RCL_M_R
{
    ldst t1, seg, sib, disp
    rcl t1, t1, reg, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop RCL_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    rcl t1, t1, reg, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop RCR_R_I
{
    rcri reg, reg, imm, flags=(OF,CF)
};

def macroop RCR_M_I
{
    ldst t1, seg, sib, disp
    rcri t1, t1, imm, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop RCR_P_I
{
    rdip t7
    ldst t1, seg, riprel, disp
    rcri t1, t1, imm, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop RCR_1_R
{
    rcri reg, reg, 1, flags=(OF,CF)
};

def macroop RCR_1_M
{
    ldst t1, seg, sib, disp
    rcri t1, t1, 1, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop RCR_1_P
{
    rdip t7
    ldst t1, seg, riprel, disp
    rcri t1, t1, 1, flags=(OF,CF)
    st t1, seg, riprel, disp
};

def macroop RCR_R_R
{
    rcr reg, reg, regm, flags=(OF,CF)
};

def macroop RCR_M_R
{
    ldst t1, seg, sib, disp
    rcr t1, t1, reg, flags=(OF,CF)
    st t1, seg, sib, disp
};

def macroop RCR_P_R
{
    rdip t7
    ldst t1, seg, riprel, disp
    rcr t1, t1, reg, flags=(OF,CF)
    st t1, seg, riprel, disp
};
"""
