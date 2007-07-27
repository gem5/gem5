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
def macroop SETZ_R
{
    movi reg, reg, 1, flags=(CZF,)
    movi reg, reg, 0, flags=(nCZF,)
};

def macroop SETZ_M
{
    movi t1, t1, 1, flags=(CZF,)
    movi t1, t1, 0, flags=(nCZF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETZ_P
{
    rdip t7
    movi t1, t1, 1, flags=(CZF,)
    movi t1, t1, 0, flags=(nCZF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETNZ_R
{
    movi reg, reg, 1, flags=(nCZF,)
    movi reg, reg, 0, flags=(CZF,)
};

def macroop SETNZ_M
{
    movi t1, t1, 1, flags=(nCZF,)
    movi t1, t1, 0, flags=(CZF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETNZ_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCZF,)
    movi t1, t1, 0, flags=(CZF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETB_R
{
    movi reg, reg, 1, flags=(CCF,)
    movi reg, reg, 0, flags=(nCCF,)
};

def macroop SETB_M
{
    movi t1, t1, 1, flags=(CCF,)
    movi t1, t1, 0, flags=(nCCF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETB_P
{
    rdip t7
    movi t1, t1, 1, flags=(CCF,)
    movi t1, t1, 0, flags=(nCCF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETNB_R
{
    movi reg, reg, 1, flags=(nCCF,)
    movi reg, reg, 0, flags=(CCF,)
};

def macroop SETNB_M
{
    movi t1, t1, 1, flags=(nCCF,)
    movi t1, t1, 0, flags=(CCF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETNB_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCCF,)
    movi t1, t1, 0, flags=(CCF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETBE_R
{
    movi reg, reg, 1, flags=(CCvZF,)
    movi reg, reg, 0, flags=(nCCvZF,)
};

def macroop SETBE_M
{
    movi t1, t1, 1, flags=(CCvZF,)
    movi t1, t1, 0, flags=(nCCvZF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETBE_P
{
    rdip t7
    movi t1, t1, 1, flags=(CCvZF,)
    movi t1, t1, 0, flags=(nCCvZF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETNBE_R
{
    movi reg, reg, 1, flags=(nCCvZF,)
    movi reg, reg, 0, flags=(CCvZF,)
};

def macroop SETNBE_M
{
    movi t1, t1, 1, flags=(nCCvZF,)
    movi t1, t1, 0, flags=(CCvZF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETNBE_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCCvZF,)
    movi t1, t1, 0, flags=(CCvZF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETS_R
{
    movi reg, reg, 1, flags=(CSF,)
    movi reg, reg, 0, flags=(nCSF,)
};

def macroop SETS_M
{
    movi t1, t1, 1, flags=(CSF,)
    movi t1, t1, 0, flags=(nCSF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETS_P
{
    rdip t7
    movi t1, t1, 1, flags=(CSF,)
    movi t1, t1, 0, flags=(nCSF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETNS_R
{
    movi reg, reg, 1, flags=(nCSF,)
    movi reg, reg, 0, flags=(CSF,)
};

def macroop SETNS_M
{
    movi t1, t1, 1, flags=(nCSF,)
    movi t1, t1, 0, flags=(CSF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETNS_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCSF,)
    movi t1, t1, 0, flags=(CSF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETP_R
{
    movi reg, reg, 1, flags=(CPF,)
    movi reg, reg, 0, flags=(nCPF,)
};

def macroop SETP_M
{
    movi t1, t1, 1, flags=(CPF,)
    movi t1, t1, 0, flags=(nCPF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETP_P
{
    rdip t7
    movi t1, t1, 1, flags=(CPF,)
    movi t1, t1, 0, flags=(nCPF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETNP_R
{
    movi reg, reg, 1, flags=(nCPF,)
    movi reg, reg, 0, flags=(CPF,)
};

def macroop SETNP_M
{
    movi t1, t1, 1, flags=(nCPF,)
    movi t1, t1, 0, flags=(CPF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETNP_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCPF,)
    movi t1, t1, 0, flags=(CPF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETL_R
{
    movi reg, reg, 1, flags=(CSxOF,)
    movi reg, reg, 0, flags=(nCSxOF,)
};

def macroop SETL_M
{
    movi t1, t1, 1, flags=(CSxOF,)
    movi t1, t1, 0, flags=(nCSxOF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETL_P
{
    rdip t7
    movi t1, t1, 1, flags=(CSxOF,)
    movi t1, t1, 0, flags=(nCSxOF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETNL_R
{
    movi reg, reg, 1, flags=(nCSxOF,)
    movi reg, reg, 0, flags=(CSxOF,)
};

def macroop SETNL_M
{
    movi t1, t1, 1, flags=(nCSxOF,)
    movi t1, t1, 0, flags=(CSxOF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETNL_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCSxOF,)
    movi t1, t1, 0, flags=(CSxOF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETLE_R
{
    movi reg, reg, 1, flags=(CSxOvZF,)
    movi reg, reg, 0, flags=(nCSxOvZF,)
};

def macroop SETLE_M
{
    movi t1, t1, 1, flags=(CSxOvZF,)
    movi t1, t1, 0, flags=(nCSxOvZF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETLE_P
{
    rdip t7
    movi t1, t1, 1, flags=(CSxOvZF,)
    movi t1, t1, 0, flags=(nCSxOvZF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETNLE_R
{
    movi reg, reg, 1, flags=(nCSxOvZF,)
    movi reg, reg, 0, flags=(CSxOvZF,)
};

def macroop SETNLE_M
{
    movi t1, t1, 1, flags=(nCSxOvZF,)
    movi t1, t1, 0, flags=(CSxOvZF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETNLE_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCSxOvZF,)
    movi t1, t1, 0, flags=(CSxOvZF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETO_R
{
    movi reg, reg, 1, flags=(COF,)
    movi reg, reg, 0, flags=(nCOF,)
};

def macroop SETO_M
{
    movi t1, t1, 1, flags=(COF,)
    movi t1, t1, 0, flags=(nCOF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETO_P
{
    rdip t7
    movi t1, t1, 1, flags=(COF,)
    movi t1, t1, 0, flags=(nCOF,)
    st t1, ds, [0, t0, t7], disp
};

def macroop SETNO_R
{
    movi reg, reg, 1, flags=(nCOF,)
    movi reg, reg, 0, flags=(COF,)
};

def macroop SETNO_M
{
    movi t1, t1, 1, flags=(nCOF,)
    movi t1, t1, 0, flags=(COF,)
    st t1, ds, [scale, index, base], disp
};

def macroop SETNO_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCOF,)
    movi t1, t1, 0, flags=(COF,)
    st t1, ds, [0, t0, t7], disp
};
'''
