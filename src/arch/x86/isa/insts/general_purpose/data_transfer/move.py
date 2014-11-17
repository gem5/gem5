# Copyright (c) 2007-2008 The Hewlett-Packard Development Company
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
#
# Authors: Gabe Black

microcode = '''

#
# Regular moves
#

def macroop MOV_R_MI {
    limm t1, imm, dataSize=asz
    ld reg, seg, [1, t0, t1]
};

def macroop MOV_MI_R {
    limm t1, imm, dataSize=asz
    st reg, seg, [1, t0, t1]
};

def macroop MOV_R_R {
    mov reg, reg, regm
};

def macroop MOV_M_R {
    st reg, seg, sib, disp
};

def macroop MOV_P_R {
    rdip t7
    st reg, seg, riprel, disp
};

def macroop MOV_R_M {
    ld reg, seg, sib, disp
};

def macroop MOV_R_P {
    rdip t7
    ld reg, seg, riprel, disp
};

def macroop MOV_R_I {
    limm reg, imm
};

def macroop MOV_M_I {
    limm t1, imm
    st t1, seg, sib, disp
};

def macroop MOV_P_I {
    rdip t7
    limm t1, imm
    st t1, seg, riprel, disp
};

#
# Sign extending moves
#

def macroop MOVSXD_R_R {
    sexti reg, regm, 31
};

def macroop MOVSXD_R_M {
    ld t1, seg, sib, disp, dataSize=4
    sexti reg, t1, 31
};

def macroop MOVSXD_R_P {
    rdip t7
    ld t1, seg, riprel, disp, dataSize=4
    sexti reg, t1, 31
};

def macroop MOVSX_B_R_R {
    mov t1, t1, regm, dataSize=1
    sexti reg, t1, 7
};

def macroop MOVSX_B_R_M {
    ld t1, seg, sib, disp, dataSize=1
    sexti reg, t1, 7
};

def macroop MOVSX_B_R_P {
    rdip t7
    ld t1, seg, riprel, disp, dataSize=1
    sexti reg, t1, 7
};

def macroop MOVSX_W_R_R {
    sexti reg, regm, 15
};

def macroop MOVSX_W_R_M {
    ld reg, seg, sib, disp, dataSize=2
    sexti reg, reg, 15
};

def macroop MOVSX_W_R_P {
    rdip t7
    ld reg, seg, riprel, disp, dataSize=2
    sexti reg, reg, 15
};

#
# Zero extending moves
#

def macroop MOVZX_B_R_R {
    mov t1, t1, regm, dataSize=1
    zexti reg, t1, 7
};

def macroop MOVZX_B_R_M {
    ld t1, seg, sib, disp, dataSize=1
    zexti reg, t1, 7
};

def macroop MOVZX_B_R_P {
    rdip t7
    ld t1, seg, riprel, disp, dataSize=1
    zexti reg, t1, 7
};

def macroop MOVZX_W_R_R {
    zexti reg, regm, 15
};

def macroop MOVZX_W_R_M {
    ld t1, seg, sib, disp, dataSize=2
    zexti reg, t1, 15
};

def macroop MOVZX_W_R_P {
    rdip t7
    ld t1, seg, riprel, disp, dataSize=2
    zexti reg, t1, 15
};

def macroop MOV_C_R {
    .serializing
    .adjust_env maxOsz
    wrcr reg, regm
};

def macroop MOV_R_C {
    .serializing
    .adjust_env maxOsz
    rdcr reg, regm
};

def macroop MOV_D_R {
    .serializing
    .adjust_env maxOsz
    wrdr reg, regm
};

def macroop MOV_R_D {
    .adjust_env maxOsz
    rddr reg, regm
};

def macroop MOV_R_S {
    rdsel reg, regm
};

def macroop MOV_M_S {
    rdsel t1, reg
    st t1, seg, sib, disp, dataSize=2
};

def macroop MOV_P_S {
    rdip t7
    rdsel t1, reg
    st t1, seg, riprel, disp, dataSize=2
};

def macroop MOV_REAL_S_R {
    zexti t2, regm, 15, dataSize=8
    slli t3, t2, 4, dataSize=8
    wrsel reg, regm
    wrbase reg, t3, dataSize=8
};

def macroop MOV_REAL_S_M {
    ld t1, seg, sib, disp, dataSize=2
    zexti t2, t1, 15, dataSize=8
    slli t3, t2, 4, dataSize=8
    wrsel reg, t1
    wrbase reg, t3, dataSize=8
};

def macroop MOV_REAL_S_P {
    panic "RIP relative addressing shouldn't happen in real mode"
};

def macroop MOV_S_R {
    andi t0, regm, 0xFC, flags=(EZF,), dataSize=2
    br label("processDescriptor"), flags=(CEZF,)
    andi t2, regm, 0xF8, dataSize=8
    andi t0, regm, 0x4, flags=(EZF,), dataSize=2
    br label("globalDescriptor"), flags=(CEZF,)
    ld t3, tsl, [1, t0, t2], dataSize=8, addressSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8, addressSize=8
processDescriptor:
    chks regm, t3, dataSize=8
    wrdl reg, t3, regm
    wrsel reg, regm
};

def macroop MOV_S_M {
    ld t1, seg, sib, disp, dataSize=2
    andi t0, t1, 0xFC, flags=(EZF,), dataSize=2
    br label("processDescriptor"), flags=(CEZF,)
    andi t2, t1, 0xF8, dataSize=8
    andi t0, t1, 0x4, flags=(EZF,), dataSize=2
    br label("globalDescriptor"), flags=(CEZF,)
    ld t3, tsl, [1, t0, t2], dataSize=8, addressSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8, addressSize=8
processDescriptor:
    chks t1, t3, dataSize=8
    wrdl reg, t3, t1
    wrsel reg, t1
};

def macroop MOV_S_P {
    rdip t7
    ld t1, seg, riprel, disp, dataSize=2
    andi t0, t1, 0xFC, flags=(EZF,), dataSize=2
    br label("processDescriptor"), flags=(CEZF,)
    andi t2, t1, 0xF8, dataSize=8
    andi t0, t1, 0x4, flags=(EZF,), dataSize=2
    br label("globalDescriptor"), flags=(CEZF,)
    ld t3, tsl, [1, t0, t2], dataSize=8, addressSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8, addressSize=8
processDescriptor:
    chks t1, t3, dataSize=8
    wrdl reg, t3, t1
    wrsel reg, t1
};

def macroop MOVSS_S_R {
    andi t0, regm, 0xFC, flags=(EZF,), dataSize=2
    br label("processDescriptor"), flags=(CEZF,)
    andi t2, regm, 0xF8, dataSize=8
    andi t0, regm, 0x4, flags=(EZF,), dataSize=2
    br label("globalDescriptor"), flags=(CEZF,)
    ld t3, tsl, [1, t0, t2], dataSize=8, addressSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8, addressSize=8
processDescriptor:
    chks regm, t3, SSCheck, dataSize=8
    wrdl reg, t3, regm
    wrsel reg, regm
};

def macroop MOVSS_S_M {
    ld t1, seg, sib, disp, dataSize=2
    andi t0, t1, 0xFC, flags=(EZF,), dataSize=2
    br label("processDescriptor"), flags=(CEZF,)
    andi t2, t1, 0xF8, dataSize=8
    andi t0, t1, 0x4, flags=(EZF,), dataSize=2
    br label("globalDescriptor"), flags=(CEZF,)
    ld t3, tsl, [1, t0, t2], dataSize=8, addressSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8, addressSize=8
processDescriptor:
    chks t1, t3, SSCheck, dataSize=8
    wrdl reg, t3, t1
    wrsel reg, t1
};

def macroop MOVSS_S_P {
    rdip t7
    ld t1, seg, riprel, disp, dataSize=2
    andi t0, t1, 0xFC, flags=(EZF,), dataSize=2
    br label("processDescriptor"), flags=(CEZF,)
    andi t2, t1, 0xF8, dataSize=8
    andi t0, t1, 0x4, flags=(EZF,), dataSize=2
    br label("globalDescriptor"), flags=(CEZF,)
    ld t3, tsl, [1, t0, t2], dataSize=8, addressSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8, addressSize=8
processDescriptor:
    chks t1, t3, SSCheck, dataSize=8
    wrdl reg, t3, t1
    wrsel reg, t1
};

def macroop MOVNTI_M_R {
    st reg, seg, sib, disp
};

def macroop MOVNTI_P_R {
    rdip t7
    st reg, seg, riprel, disp
};

def macroop MOVD_XMM_R {
   mov2fp xmml, regm, srcSize=dsz, destSize=8
   lfpimm xmmh, 0
};

def macroop MOVD_XMM_M {
    ldfp xmml, seg, sib, disp, dataSize=dsz
    lfpimm xmmh, 0
};

def macroop MOVD_XMM_P {
    rdip t7
    ldfp xmml, seg, riprel, disp, dataSize=dsz
    lfpimm xmmh, 0
};

def macroop MOVD_R_XMM {
    mov2int reg, xmmlm, size=dsz
};

def macroop MOVD_M_XMM {
    stfp xmml, seg, sib, disp, dataSize=dsz
};

def macroop MOVD_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, disp, dataSize=dsz
};

'''
#let {{
#    class MOVD(Inst):
#       "GenFault ${new UnimpInstFault}"
#}};
