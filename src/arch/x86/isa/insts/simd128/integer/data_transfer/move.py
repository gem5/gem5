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
def macroop MOVQ_XMM_XMM {
    movfp xmml, xmmlm
    lfpimm xmmh, 0
};

def macroop MOVQ_XMM_M {
    ldfp xmml, seg, sib, disp, dataSize=8
    lfpimm xmmh, 0
};

def macroop MOVQ_XMM_P {
    rdip t7
    ldfp xmml, seg, riprel, disp, dataSize=8
    lfpimm xmmh, 0
};

def macroop MOVQ_M_XMM {
    stfp xmml, seg, sib, disp, dataSize=8
};

def macroop MOVQ_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, disp, dataSize=8
};

def macroop MOVDQ2Q_MMX_XMM {
    movfp mmx, xmmlm, dataSize=8
};

def macroop MOVQ2DQ_XMM_MMX {
    movfp xmml, mmxm, dataSize=8
    lfpimm xmmh, 0
};

def macroop MOVDQA_XMM_XMM {
    movfp xmml, xmmlm
    movfp xmmh, xmmhm
};

def macroop MOVDQA_XMM_M {
    ldfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVDQA_XMM_P {
    rdip t7
    ldfp xmml, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVDQA_M_XMM {
    stfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    stfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVDQA_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, "DISPLACEMENT", dataSize=8
    stfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVDQU_XMM_XMM {
    movfp xmml, xmmlm
    movfp xmmh, xmmhm
};

def macroop MOVDQU_XMM_M {
    ldfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVDQU_XMM_P {
    rdip t7
    ldfp xmml, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVDQU_M_XMM {
    stfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    stfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVDQU_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, "DISPLACEMENT", dataSize=8
    stfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
};

def macroop LDDQU_XMM_M {
    ldfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop LDDQU_XMM_P {
    rdip t7
    ldfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop PMOVSXDQ_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=8, srcSize=4, ext = Signed
    extmove xmmh, ufp1, destSize=8, srcSize=4, ext = Signed + "|" + PartHi
};

def macroop PMOVSXDQ_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=8, srcSize=4, ext = Signed
    extmove xmmh, ufp1, destSize=8, srcSize=4, ext = Signed + "|" + PartHi
};

def macroop PMOVSXDQ_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=8, srcSize=4, ext = Signed
    extmove xmmh, ufp1, destSize=8, srcSize=4, ext = Signed + "|" + PartHi
};

def macroop PMOVSXWQ_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=8, srcSize=2, ext = Signed
    extmove xmmh, ufp1, destSize=8, srcSize=2, ext = Signed + "|" + PartHi
};

def macroop PMOVSXWQ_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=4
    extmove xmml, ufp1, destSize=8, srcSize=2, ext = Signed
    extmove xmmh, ufp1, destSize=8, srcSize=2, ext = Signed + "|" + PartHi
};

def macroop PMOVSXWQ_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=4
    extmove xmml, ufp1, destSize=8, srcSize=2, ext = Signed
    extmove xmmh, ufp1, destSize=8, srcSize=2, ext = Signed + "|" + PartHi
};

def macroop PMOVSXWD_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=4, srcSize=2, ext = Signed
    extmove xmmh, ufp1, destSize=4, srcSize=2, ext = Signed + "|" + PartHi
};

def macroop PMOVSXWD_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=4, srcSize=2, ext = Signed
    extmove xmmh, ufp1, destSize=4, srcSize=2, ext = Signed + "|" + PartHi
};

def macroop PMOVSXWD_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=4, srcSize=2, ext = Signed
    extmove xmmh, ufp1, destSize=4, srcSize=2, ext = Signed + "|" + PartHi
};

def macroop PMOVSXBQ_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=8, srcSize=1, ext = Signed
    extmove xmmh, ufp1, destSize=8, srcSize=1, ext = Signed + "|" + PartHi
};

def macroop PMOVSXBQ_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=2
    extmove xmml, ufp1, destSize=8, srcSize=1, ext = Signed
    extmove xmmh, ufp1, destSize=8, srcSize=1, ext = Signed + "|" + PartHi
};

def macroop PMOVSXBQ_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=2
    extmove xmml, ufp1, destSize=8, srcSize=1, ext = Signed
    extmove xmmh, ufp1, destSize=8, srcSize=1, ext = Signed + "|" + PartHi
};

def macroop PMOVSXBD_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=4, srcSize=1, ext = Signed
    extmove xmmh, ufp1, destSize=4, srcSize=1, ext = Signed + "|" + PartHi
};

def macroop PMOVSXBD_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=4
    extmove xmml, ufp1, destSize=4, srcSize=1, ext = Signed
    extmove xmmh, ufp1, destSize=4, srcSize=1, ext = Signed + "|" + PartHi
};

def macroop PMOVSXBD_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=4
    extmove xmml, ufp1, destSize=4, srcSize=1, ext = Signed
    extmove xmmh, ufp1, destSize=4, srcSize=1, ext = Signed + "|" + PartHi
};

def macroop PMOVSXBW_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=2, srcSize=1, ext = Signed
    extmove xmmh, ufp1, destSize=2, srcSize=1, ext = Signed + "|" + PartHi
};

def macroop PMOVSXBW_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=2, srcSize=1, ext = Signed
    extmove xmmh, ufp1, destSize=2, srcSize=1, ext = Signed + "|" + PartHi
};

def macroop PMOVSXBW_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=2, srcSize=1, ext = Signed
    extmove xmmh, ufp1, destSize=2, srcSize=1, ext = Signed + "|" + PartHi
};


def macroop PMOVZXDQ_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=8, srcSize=4
    extmove xmmh, ufp1, destSize=8, srcSize=4, ext=PartHi
};

def macroop PMOVZXDQ_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=8, srcSize=4
    extmove xmmh, ufp1, destSize=8, srcSize=4, ext=PartHi
};

def macroop PMOVZXDQ_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=8, srcSize=4
    extmove xmmh, ufp1, destSize=8, srcSize=4, ext=PartHi
};

def macroop PMOVZXWQ_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=8, srcSize=2
    extmove xmmh, ufp1, destSize=8, srcSize=2, ext=PartHi
};

def macroop PMOVZXWQ_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=4
    extmove xmml, ufp1, destSize=8, srcSize=2
    extmove xmmh, ufp1, destSize=8, srcSize=2, ext=PartHi
};

def macroop PMOVZXWQ_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=4
    extmove xmml, ufp1, destSize=8, srcSize=2
    extmove xmmh, ufp1, destSize=8, srcSize=2, ext=PartHi
};

def macroop PMOVZXWD_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=4, srcSize=2
    extmove xmmh, ufp1, destSize=4, srcSize=2, ext=PartHi
};

def macroop PMOVZXWD_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=4, srcSize=2
    extmove xmmh, ufp1, destSize=4, srcSize=2, ext=PartHi
};

def macroop PMOVZXWD_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=4, srcSize=2
    extmove xmmh, ufp1, destSize=4, srcSize=2, ext=PartHi
};

def macroop PMOVZXBQ_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=8, srcSize=1
    extmove xmmh, ufp1, destSize=8, srcSize=1, ext=PartHi
};

def macroop PMOVZXBQ_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=2
    extmove xmml, ufp1, destSize=8, srcSize=1
    extmove xmmh, ufp1, destSize=8, srcSize=1, ext=PartHi
};

def macroop PMOVZXBQ_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=2
    extmove xmml, ufp1, destSize=8, srcSize=1
    extmove xmmh, ufp1, destSize=8, srcSize=1, ext=PartHi
};

def macroop PMOVZXBD_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=4, srcSize=1
    extmove xmmh, ufp1, destSize=4, srcSize=1, ext=PartHi
};

def macroop PMOVZXBD_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=4
    extmove xmml, ufp1, destSize=4, srcSize=1
    extmove xmmh, ufp1, destSize=4, srcSize=1, ext=PartHi
};

def macroop PMOVZXBD_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=4
    extmove xmml, ufp1, destSize=4, srcSize=1
    extmove xmmh, ufp1, destSize=4, srcSize=1, ext=PartHi
};

def macroop PMOVZXBW_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    extmove xmml, ufp1, destSize=2, srcSize=1
    extmove xmmh, ufp1, destSize=2, srcSize=1, ext=PartHi
};

def macroop PMOVZXBW_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=2, srcSize=1
    extmove xmmh, ufp1, destSize=2, srcSize=1, ext=PartHi
};

def macroop PMOVZXBW_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    extmove xmml, ufp1, destSize=2, srcSize=1
    extmove xmmh, ufp1, destSize=2, srcSize=1, ext=PartHi
};
"""
