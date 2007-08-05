# Copyright (c) 2007 The Hewlett-Packard Development Company
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
def macroop ADD_R_R
{
    add reg, reg, regm, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop ADD_R_I
{
    limm t1, imm
    add reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop ADD_M_I
{
    limm t2, imm
    ld t1, seg, sib, disp
    add t1, t1, t2, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, sib, disp
};

def macroop ADD_P_I
{
    rdip t7
    limm t2, imm
    ld t1, seg, riprel, disp
    add t1, t1, t2, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop ADD_M_R
{
    ld t1, seg, sib, disp
    add t1, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, sib, disp
};

def macroop ADD_P_R
{
    rdip t7
    ld t1, seg, riprel, disp
    add t1, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop ADD_R_M
{
    ld t1, seg, sib, disp
    add reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop ADD_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    add reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop SUB_R_R
{
    sub reg, reg, regm, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop SUB_R_I
{
    limm t1, imm
    sub reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop SUB_R_M
{
    ld t1, seg, sib, disp
    sub reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop SUB_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    sub reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop SUB_M_I
{
    limm t2, imm
    ld t1, seg, sib, disp
    sub t1, t1, t2, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, sib, disp
};

def macroop SUB_P_I
{
    rdip t7
    limm t2, imm
    ld t1, seg, riprel, disp
    sub t1, t1, t2, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop SUB_M_R
{
    ld t1, seg, sib, disp
    sub t1, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, sib, disp
};

def macroop SUB_P_R
{
    rdip t7
    ld t1, seg, riprel, disp
    sub t1, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop ADC_R_R
{
    adc reg, reg, regm, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop ADC_R_I
{
    limm t1, imm
    adc reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop ADC_M_I
{
    limm t2, imm
    ld t1, seg, sib, disp
    adc t1, t1, t2, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, sib, disp
};

def macroop ADC_P_I
{
    rdip t7
    limm t2, imm
    ld t1, seg, riprel, disp
    adc t1, t1, t2, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop ADC_M_R
{
    ld t1, seg, sib, disp
    adc t1, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, sib, disp
};

def macroop ADC_P_R
{
    rdip t7
    ld t1, seg, riprel, disp
    adc t1, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop ADC_R_M
{
    ld t1, seg, sib, disp
    adc reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop ADC_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    adc reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop SBB_R_R
{
    sbb reg, reg, regm, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop SBB_R_I
{
    limm t1, imm
    sbb reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop SBB_R_M
{
    ld t1, seg, sib, disp
    sbb reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop SBB_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    sbb reg, reg, t1, flags=(OF,SF,ZF,AF,PF,CF)
};

def macroop SBB_M_I
{
    limm t2, imm
    ld t1, seg, sib, disp
    sbb t1, t1, t2, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, sib, disp
};

def macroop SBB_P_I
{
    rdip t7
    limm t2, imm
    ld t1, seg, riprel, disp
    sbb t1, t1, t2, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop SBB_M_R
{
    ld t1, seg, sib, disp
    sbb t1, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, sib, disp
};

def macroop SBB_P_R
{
    rdip t7
    ld t1, seg, riprel, disp
    sbb t1, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    st t1, seg, riprel, disp
};

def macroop NEG_R
{
    sub reg, t0, reg, flags=(CF,OF,SF,ZF,AF,PF)
};

def macroop NEG_M
{
    ld t1, seg, sib, disp
    sub t1, t0, t1, flags=(CF,OF,SF,ZF,AF,PF)
    st t1, seg, sib, disp
};

def macroop NEG_P
{
    rdip t7
    ld t1, seg, riprel, disp
    sub t1, t0, t1, flags=(CF,OF,SF,ZF,AF,PF)
    st t1, seg, riprel, disp
};
'''
