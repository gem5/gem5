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
def macroop PUNPCKLBW_XMM_XMM {
    unpack xmmh, xmml, xmmlm, ext=1, size=1
    unpack xmml, xmml, xmmlm, ext=0, size=1
};

def macroop PUNPCKLBW_XMM_M {
    ldfp ufp1, seg, sib, disp, dataSize=8
    unpack xmmh, xmml, ufp1, ext=1, size=1
    unpack xmml, xmml, ufp1, ext=0, size=1
};

def macroop PUNPCKLBW_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, disp, dataSize=8
    unpack xmmh, xmml, ufp1, ext=1, size=1
    unpack xmml, xmml, ufp1, ext=0, size=1
};

def macroop PUNPCKLWD_XMM_XMM {
    unpack xmmh, xmml, xmmlm, ext=1, size=2
    unpack xmml, xmml, xmmlm, ext=0, size=2
};

def macroop PUNPCKLWD_XMM_M {
    ldfp ufp1, seg, sib, disp, dataSize=8
    unpack xmmh, xmml, ufp1, ext=1, size=2
    unpack xmml, xmml, ufp1, ext=0, size=2
};

def macroop PUNPCKLWD_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, disp, dataSize=8
    unpack xmmh, xmml, ufp1, ext=1, size=2
    unpack xmml, xmml, ufp1, ext=0, size=2
};

def macroop PUNPCKLDQ_XMM_XMM {
    unpack xmmh, xmml, xmmlm, ext=1, size=4
    unpack xmml, xmml, xmmlm, ext=0, size=4
};

def macroop PUNPCKLDQ_XMM_M {
    ldfp ufp1, seg, sib, disp, dataSize=8
    unpack xmmh, xmml, ufp1, ext=1, size=4
    unpack xmml, xmml, ufp1, ext=0, size=4
};

def macroop PUNPCKLDQ_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, disp, dataSize=8
    unpack xmmh, xmml, ufp1, ext=1, size=4
    unpack xmml, xmml, ufp1, ext=0, size=4
};

def macroop PUNPCKHBW_XMM_XMM {
    unpack xmml, xmmh, xmmhm, ext=0, size=1
    unpack xmmh, xmmh, xmmhm, ext=1, size=1
};

def macroop PUNPCKHBW_XMM_M {
    lea t1, seg, sib, disp, dataSize=asz
    ldfp ufp1, seg, [1, t0, t1], 8, dataSize=8
    unpack xmml, xmmh, ufp1, ext=0, size=1
    unpack xmmh, xmmh, ufp1, ext=1, size=1
};

def macroop PUNPCKHBW_XMM_P {
    rdip t7
    lea t1, seg, riprel, disp, dataSize=asz
    ldfp ufp1, seg, [1, t0, t1], 8, dataSize=8
    unpack xmml, xmmh, ufp1, ext=0, size=1
    unpack xmmh, xmmh, ufp1, ext=1, size=1
};

def macroop PUNPCKHWD_XMM_XMM {
    unpack xmml, xmmh, xmmhm, ext=0, size=2
    unpack xmmh, xmmh, xmmhm, ext=1, size=2
};

def macroop PUNPCKHWD_XMM_M {
    lea t1, seg, sib, disp, dataSize=asz
    ldfp ufp1, seg, [1, t0, t1], 8, dataSize=8
    unpack xmml, xmmh, ufp1, ext=0, size=2
    unpack xmmh, xmmh, ufp1, ext=1, size=2
};

def macroop PUNPCKHWD_XMM_P {
    rdip t7
    lea t1, seg, riprel, disp, dataSize=asz
    ldfp ufp1, seg, [1, t0, t1], 8, dataSize=8
    unpack xmml, xmmh, ufp1, ext=0, size=2
    unpack xmmh, xmmh, ufp1, ext=1, size=2
};

def macroop PUNPCKHDQ_XMM_XMM {
    unpack xmml, xmmh, xmmhm, ext=0, size=4
    unpack xmmh, xmmh, xmmhm, ext=1, size=4
};

def macroop PUNPCKHDQ_XMM_M {
    lea t1, seg, sib, disp, dataSize=asz
    ldfp ufp1, seg, [1, t0, t1], 8, dataSize=8
    unpack xmml, xmmh, ufp1, ext=0, size=4
    unpack xmmh, xmmh, ufp1, ext=1, size=4
};

def macroop PUNPCKHDQ_XMM_P {
    rdip t7
    lea t1, seg, riprel, disp, dataSize=asz
    ldfp ufp1, seg, [1, t0, t1], 8, dataSize=8
    unpack xmml, xmmh, ufp1, ext=0, size=4
    unpack xmmh, xmmh, ufp1, ext=1, size=4
};

def macroop PUNPCKHQDQ_XMM_XMM {
    movfp xmml, xmmh
    movfp xmmh, xmmhm
};

def macroop PUNPCKHQDQ_XMM_M {
    lea t1, seg, sib, disp, dataSize=asz
    ldfp ufp1, seg, [1, t0, t1], 8, dataSize=8
    movfp xmml, xmmh
    movfp xmmh, ufp1
};

def macroop PUNPCKHQDQ_XMM_P {
    rdip t7
    lea t1, seg, riprel, disp, dataSize=asz
    ldfp ufp1, seg, riprel, 8, dataSize=8
    movfp xmml, xmmh
    movfp xmmh, ufp1
};

def macroop PUNPCKLQDQ_XMM_XMM {
    movfp xmmh, xmmlm
};

def macroop PUNPCKLQDQ_XMM_M {
    ldfp xmmh, seg, sib, disp, dataSize=8
};

def macroop PUNPCKLQDQ_XMM_P {
    rdip t7
    ldfp xmmh, seg, riprel, disp, dataSize=8
};
'''
