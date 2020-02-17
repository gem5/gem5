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
def macroop FLD_M {
    ldfp87 ufp1, seg, sib, disp
    movfp st(-1), ufp1, spm=-1
};

def macroop FLD_P {
    rdip t7
    ldfp87 ufp1, seg, riprel, disp
    movfp st(-1), ufp1, spm=-1
};

def macroop FLD_R {
    movfp st(-1), sti, spm=-1
};

def macroop FLD80_M {
    ld t1, seg, sib, "DISPLACEMENT", dataSize=8
    ld t2, seg, sib, "DISPLACEMENT + 8", dataSize=2
    cvtint_fp80 st(-1), t1, t2, spm=-1
};

def macroop FLD80_P {
    rdip t7
    ld t1, seg, riprel, "DISPLACEMENT", dataSize=8
    ld t2, seg, riprel, "DISPLACEMENT + 8", dataSize=2
    cvtint_fp80 st(-1), t1, t2, spm=-1
};

def macroop FST_R {
    movfp sti, st(0)
};

def macroop FST_M {
    stfp87 st(0), seg, sib, disp
};

def macroop FST_P {
    rdip t7
    stfp87 st(0), seg, riprel, disp
};

def macroop FSTP_R {
    movfp sti, st(0), spm=1
};

def macroop FSTP_M {
    movfp ufp1, st(0)
    stfp87 ufp1, seg, sib, disp
    pop87
};

def macroop FSTP_P {
    movfp ufp1, st(0)
    rdip t7
    stfp87 ufp1, seg, riprel, disp
    pop87
};

def macroop FST80P_M  {
    cvtfp80h_int t1, st(0)
    cvtfp80l_int t2, st(0)
    st t1, seg, sib, "DISPLACEMENT + 0", dataSize=8
    st t2, seg, sib, "DISPLACEMENT + 8", dataSize=2
    pop87
};

def macroop FST80P_P  {
    rdip t7
    cvtfp80h_int t1, st(0)
    cvtfp80l_int t2, st(0)
    st t1, seg, riprel, "DISPLACEMENT + 0", dataSize=8
    st t2, seg, riprel, "DISPLACEMENT + 8", dataSize=2
    pop87
};

'''
