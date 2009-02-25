# Copyright (c) 2007-2008 The Hewlett-Packard Development Company
# All rights reserved.
#
# Redistribution and use of this software in source and binary forms,
# with or without modification, are permitted provided that the
# following conditions are met:
#
# The software must be used only for Non-Commercial Use which means any
# use which is NOT directed to receiving any direct monetary
# compensation for, or commercial advantage from such use.  Illustrative
# examples of non-commercial use are academic research, personal study,
# teaching, education and corporate research & development.
# Illustrative examples of commercial use are distributing products for
# commercial advantage and providing services using the software for
# commercial advantage.
#
# If you wish to use this software or functionality therein that may be
# covered by patents for commercial use, please contact:
#     Director of Intellectual Property Licensing
#     Office of Strategy and Technology
#     Hewlett-Packard Company
#     1501 Page Mill Road
#     Palo Alto, California  94304
#
# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.  Redistributions
# in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.  Neither the name of
# the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.  No right of
# sublicense is granted herewith.  Derivatives of the software and
# output created using the software may be prepared, but only for
# Non-Commercial Uses.  Derivatives of the software may be shared with
# others provided: (i) the others agree to abide by the list of
# conditions herein which includes the Non-Commercial Use restrictions;
# and (ii) such Derivatives of the software include the above copyright
# notice to acknowledge the contribution from this software where
# applicable, this list of conditions and the disclaimer below.
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
    wrcr reg, regm
};

def macroop MOV_R_C {
    rdcr reg, regm
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
    slli t3, t2, 2, dataSize=8
    wrsel reg, regm
    wrbase reg, t3
};

def macroop MOV_REAL_S_M {
    ld t1, seg, sib, disp, dataSize=2
    zexti t2, t1, 15, dataSize=8
    slli t3, t2, 2, dataSize=8
    wrsel reg, t1
    wrbase reg, t3
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
    ld t3, tsl, [1, t0, t2], dataSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8
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
    ld t3, tsl, [1, t0, t2], dataSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8
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
    ld t3, tsl, [1, t0, t2], dataSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8
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
    ld t3, tsl, [1, t0, t2], dataSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8
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
    ld t3, tsl, [1, t0, t2], dataSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8
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
    ld t3, tsl, [1, t0, t2], dataSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t2], dataSize=8
processDescriptor:
    chks t1, t3, SSCheck, dataSize=8
    wrdl reg, t3, t1
    wrsel reg, t1
};
'''
#let {{
#    class MOVD(Inst):
#       "GenFault ${new UnimpInstFault}"
#    class MOVNTI(Inst):
#       "GenFault ${new UnimpInstFault}"
#}};
