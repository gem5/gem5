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
def macroop CMOVZ_R_R
{
    mov reg, reg, regm, flags=(CZF,)
};

def macroop CMOVZ_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(CZF,)
};

def macroop CMOVZ_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(CZF,)
};

def macroop CMOVNZ_R_R
{
    mov reg, reg, regm, flags=(nCZF,)
};

def macroop CMOVNZ_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(nCZF,)
};

def macroop CMOVNZ_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(nCZF,)
};

def macroop CMOVB_R_R
{
    mov reg, reg, regm, flags=(CCF,)
};

def macroop CMOVB_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(CCF,)
};

def macroop CMOVB_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(CCF,)
};

def macroop CMOVNB_R_R
{
    mov reg, reg, regm, flags=(nCCF,)
};

def macroop CMOVNB_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(nCCF,)
};

def macroop CMOVNB_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(nCCF,)
};

def macroop CMOVBE_R_R
{
    mov reg, reg, regm, flags=(CCvZF,)
};

def macroop CMOVBE_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(CCvZF,)
};

def macroop CMOVBE_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(CCvZF,)
};

def macroop CMOVNBE_R_R
{
    mov reg, reg, regm, flags=(nCCvZF,)
};

def macroop CMOVNBE_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(nCCvZF,)
};

def macroop CMOVNBE_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(nCCvZF,)
};

def macroop CMOVS_R_R
{
    mov reg, reg, regm, flags=(CSF,)
};

def macroop CMOVS_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(CSF,)
};

def macroop CMOVS_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(CSF,)
};

def macroop CMOVNS_R_R
{
    mov reg, reg, regm, flags=(nCSF,)
};

def macroop CMOVNS_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(nCSF,)
};

def macroop CMOVNS_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(nCSF,)
};

def macroop CMOVP_R_R
{
    mov reg, reg, regm, flags=(CPF,)
};

def macroop CMOVP_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(CPF,)
};

def macroop CMOVP_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(CPF,)
};

def macroop CMOVNP_R_R
{
    mov reg, reg, regm, flags=(nCPF,)
};

def macroop CMOVNP_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, regm, flags=(nCPF,)
};

def macroop CMOVNP_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, regm, flags=(nCPF,)
};

def macroop CMOVL_R_R
{
    mov reg, reg, regm, flags=(CSxOF,)
};

def macroop CMOVL_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(CSxOF,)
};

def macroop CMOVL_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(CSxOF,)
};

def macroop CMOVNL_R_R
{
    mov reg, reg, regm, flags=(nCSxOF,)
};

def macroop CMOVNL_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(nCSxOF,)
};

def macroop CMOVNL_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(nCSxOF,)
};

def macroop CMOVLE_R_R
{
    mov reg, reg, regm, flags=(CSxOvZF,)
};

def macroop CMOVLE_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(CSxOvZF,)
};

def macroop CMOVLE_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(CSxOvZF,)
};

def macroop CMOVNLE_R_R
{
    mov reg, reg, regm, flags=(nCSxOvZF,)
};

def macroop CMOVNLE_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(nCSxOvZF,)
};

def macroop CMOVNLE_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(nCSxOvZF,)
};

def macroop CMOVO_R_R
{
    mov reg, reg, regm, flags=(COF,)
};

def macroop CMOVO_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(COF,)
};

def macroop CMOVO_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(COF,)
};

def macroop CMOVNO_R_R
{
    mov reg, reg, regm, flags=(nCOF,)
};

def macroop CMOVNO_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, t1, flags=(nCOF,)
};

def macroop CMOVNO_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, t1, flags=(nCOF,)
};
'''
