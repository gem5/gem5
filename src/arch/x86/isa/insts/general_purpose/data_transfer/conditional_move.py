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

microcode = '''
def macroop CMOVZ_R_R
{
    mov reg, reg, reg, flags=(nCZF,)
    mov reg, reg, regm, flags=(CZF,)
};

def macroop CMOVZ_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(nCZF,)
    mov reg, reg, t1, flags=(CZF,)
};

def macroop CMOVZ_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(nCZF,)
    mov reg, reg, t1, flags=(CZF,)
};

def macroop CMOVNZ_R_R
{
    mov reg, reg, reg, flags=(CZF,)
    mov reg, reg, regm, flags=(nCZF,)
};

def macroop CMOVNZ_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(CZF,)
    mov reg, reg, t1, flags=(nCZF,)
};

def macroop CMOVNZ_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(CZF,)
    mov reg, reg, t1, flags=(nCZF,)
};

def macroop CMOVB_R_R
{
    mov reg, reg, reg, flags=(nCCF,)
    mov reg, reg, regm, flags=(CCF,)
};

def macroop CMOVB_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(nCCF,)
    mov reg, reg, t1, flags=(CCF,)
};

def macroop CMOVB_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(nCCF,)
    mov reg, reg, t1, flags=(CCF,)
};

def macroop CMOVNB_R_R
{
    mov reg, reg, reg, flags=(CCF,)
    mov reg, reg, regm, flags=(nCCF,)
};

def macroop CMOVNB_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(CCF,)
    mov reg, reg, t1, flags=(nCCF,)
};

def macroop CMOVNB_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(CCF,)
    mov reg, reg, t1, flags=(nCCF,)
};

def macroop CMOVBE_R_R
{
    mov reg, reg, reg, flags=(nCCvZF,)
    mov reg, reg, regm, flags=(CCvZF,)
};

def macroop CMOVBE_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(nCCvZF,)
    mov reg, reg, t1, flags=(CCvZF,)
};

def macroop CMOVBE_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(nCCvZF,)
    mov reg, reg, t1, flags=(CCvZF,)
};

def macroop CMOVNBE_R_R
{
    mov reg, reg, reg, flags=(CCvZF,)
    mov reg, reg, regm, flags=(nCCvZF,)
};

def macroop CMOVNBE_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(CCvZF,)
    mov reg, reg, t1, flags=(nCCvZF,)
};

def macroop CMOVNBE_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(CCvZF,)
    mov reg, reg, t1, flags=(nCCvZF,)
};

def macroop CMOVS_R_R
{
    mov reg, reg, reg, flags=(nCSF,)
    mov reg, reg, regm, flags=(CSF,)
};

def macroop CMOVS_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(nCSF,)
    mov reg, reg, t1, flags=(CSF,)
};

def macroop CMOVS_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(nCSF,)
    mov reg, reg, t1, flags=(CSF,)
};

def macroop CMOVNS_R_R
{
    mov reg, reg, reg, flags=(CSF,)
    mov reg, reg, regm, flags=(nCSF,)
};

def macroop CMOVNS_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(CSF,)
    mov reg, reg, t1, flags=(nCSF,)
};

def macroop CMOVNS_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(CSF,)
    mov reg, reg, t1, flags=(nCSF,)
};

def macroop CMOVP_R_R
{
    mov reg, reg, reg, flags=(nCPF,)
    mov reg, reg, regm, flags=(CPF,)
};

def macroop CMOVP_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(nCPF,)
    mov reg, reg, t1, flags=(CPF,)
};

def macroop CMOVP_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(nCPF,)
    mov reg, reg, t1, flags=(CPF,)
};

def macroop CMOVNP_R_R
{
    mov reg, reg, reg, flags=(CPF,)
    mov reg, reg, regm, flags=(nCPF,)
};

def macroop CMOVNP_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(CPF,)
    mov reg, reg, t1, flags=(nCPF,)
};

def macroop CMOVNP_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(CPF,)
    mov reg, reg, t1, flags=(nCPF,)
};

def macroop CMOVL_R_R
{
    mov reg, reg, reg, flags=(nCSxOF,)
    mov reg, reg, regm, flags=(CSxOF,)
};

def macroop CMOVL_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(nCSxOF,)
    mov reg, reg, t1, flags=(CSxOF,)
};

def macroop CMOVL_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(nCSxOF,)
    mov reg, reg, t1, flags=(CSxOF,)
};

def macroop CMOVNL_R_R
{
    mov reg, reg, reg, flags=(CSxOF,)
    mov reg, reg, regm, flags=(nCSxOF,)
};

def macroop CMOVNL_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(CSxOF,)
    mov reg, reg, t1, flags=(nCSxOF,)
};

def macroop CMOVNL_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(CSxOF,)
    mov reg, reg, t1, flags=(nCSxOF,)
};

def macroop CMOVLE_R_R
{
    mov reg, reg, reg, flags=(nCSxOvZF,)
    mov reg, reg, regm, flags=(CSxOvZF,)
};

def macroop CMOVLE_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(nCSxOvZF,)
    mov reg, reg, t1, flags=(CSxOvZF,)
};

def macroop CMOVLE_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(nCSxOvZF,)
    mov reg, reg, t1, flags=(CSxOvZF,)
};

def macroop CMOVNLE_R_R
{
    mov reg, reg, reg, flags=(CSxOvZF,)
    mov reg, reg, regm, flags=(nCSxOvZF,)
};

def macroop CMOVNLE_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(CSxOvZF,)
    mov reg, reg, t1, flags=(nCSxOvZF,)
};

def macroop CMOVNLE_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(CSxOvZF,)
    mov reg, reg, t1, flags=(nCSxOvZF,)
};

def macroop CMOVO_R_R
{
    mov reg, reg, reg, flags=(nCOF,)
    mov reg, reg, regm, flags=(COF,)
};

def macroop CMOVO_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(nCOF,)
    mov reg, reg, t1, flags=(COF,)
};

def macroop CMOVO_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(nCOF,)
    mov reg, reg, t1, flags=(COF,)
};

def macroop CMOVNO_R_R
{
    mov reg, reg, reg, flags=(COF,)
    mov reg, reg, regm, flags=(nCOF,)
};

def macroop CMOVNO_R_M
{
    ld t1, seg, sib, disp
    mov reg, reg, reg, flags=(COF,)
    mov reg, reg, t1, flags=(nCOF,)
};

def macroop CMOVNO_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mov reg, reg, reg, flags=(COF,)
    mov reg, reg, t1, flags=(nCOF,)
};
'''
