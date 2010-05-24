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
#
# Authors: Gabe Black

microcode = '''
def macroop SALC_R
{
    sbb reg, reg, reg, dataSize=1
};

def macroop SETZ_R
{
    movi reg, reg, 1, flags=(CZF,)
    movi reg, reg, 0, flags=(nCZF,)
};

def macroop SETZ_M
{
    movi t1, t1, 1, flags=(CZF,)
    movi t1, t1, 0, flags=(nCZF,)
    st t1, seg, sib, disp
};

def macroop SETZ_P
{
    rdip t7
    movi t1, t1, 1, flags=(CZF,)
    movi t1, t1, 0, flags=(nCZF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETNZ_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCZF,)
    movi t1, t1, 0, flags=(CZF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETB_P
{
    rdip t7
    movi t1, t1, 1, flags=(CCF,)
    movi t1, t1, 0, flags=(nCCF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETNB_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCCF,)
    movi t1, t1, 0, flags=(CCF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETBE_P
{
    rdip t7
    movi t1, t1, 1, flags=(CCvZF,)
    movi t1, t1, 0, flags=(nCCvZF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETNBE_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCCvZF,)
    movi t1, t1, 0, flags=(CCvZF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETS_P
{
    rdip t7
    movi t1, t1, 1, flags=(CSF,)
    movi t1, t1, 0, flags=(nCSF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETNS_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCSF,)
    movi t1, t1, 0, flags=(CSF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETP_P
{
    rdip t7
    movi t1, t1, 1, flags=(CPF,)
    movi t1, t1, 0, flags=(nCPF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETNP_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCPF,)
    movi t1, t1, 0, flags=(CPF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETL_P
{
    rdip t7
    movi t1, t1, 1, flags=(CSxOF,)
    movi t1, t1, 0, flags=(nCSxOF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETNL_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCSxOF,)
    movi t1, t1, 0, flags=(CSxOF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETLE_P
{
    rdip t7
    movi t1, t1, 1, flags=(CSxOvZF,)
    movi t1, t1, 0, flags=(nCSxOvZF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETNLE_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCSxOvZF,)
    movi t1, t1, 0, flags=(CSxOvZF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETO_P
{
    rdip t7
    movi t1, t1, 1, flags=(COF,)
    movi t1, t1, 0, flags=(nCOF,)
    st t1, seg, riprel, disp
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
    st t1, seg, sib, disp
};

def macroop SETNO_P
{
    rdip t7
    movi t1, t1, 1, flags=(nCOF,)
    movi t1, t1, 0, flags=(COF,)
    st t1, seg, riprel, disp
};
'''
