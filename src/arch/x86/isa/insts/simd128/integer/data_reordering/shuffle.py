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
def macroop PSHUFD_XMM_XMM_I {
    shuffle ufp1, xmmlm, xmmhm, size=4, ext="IMMEDIATE"
    shuffle xmmh, xmmlm, xmmhm, size=4, ext="IMMEDIATE >> 4"
    movfp xmml, ufp1, dataSize=8
};

def macroop PSHUFD_XMM_M_I {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, sib, "DISPLACEMENT + 8", dataSize=8
    shuffle xmml, ufp1, ufp2, size=4, ext="IMMEDIATE"
    shuffle xmmh, ufp1, ufp2, size=4, ext="IMMEDIATE >> 4"
};

def macroop PSHUFD_XMM_P_I {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    shuffle xmml, ufp1, ufp2, size=4, ext="IMMEDIATE"
    shuffle xmmh, ufp1, ufp2, size=4, ext="IMMEDIATE >> 4"
};

def macroop PSHUFHW_XMM_XMM_I {
    shuffle xmmh, xmmhm, xmmhm, size=2, ext=imm
};

def macroop PSHUFHW_XMM_M_I {
    ldfp ufp1, seg, sib, "DISPLACEMENT + 8", dataSize=8
    shuffle xmmh, ufp1, ufp1, size=2, ext=imm
};

def macroop PSHUFHW_XMM_P_I {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    shuffle xmmh, ufp1, ufp1, size=2, ext=imm
};

def macroop PSHUFLW_XMM_XMM_I {
    shuffle xmml, xmmlm, xmmlm, size=2, ext=imm
};

def macroop PSHUFLW_XMM_M_I {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    shuffle xmml, ufp1, ufp1, size=2, ext=imm
};

def macroop PSHUFLW_XMM_P_I {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    shuffle xmml, ufp1, ufp1, size=2, ext=imm
};

def macroop PSHUFB_XMM_XMM {
    movfp ufp1, xmmlm, dataSize=8
    movfp ufp2, xmmhm, dataSize=8
    shuffle ufp1, xmml, xmmh, size=1, ext=0
    shuffle ufp2, xmml, xmmh, size=1, ext=0
    movfp xmml, ufp1, dataSize=8
    movfp xmmh, ufp2, dataSize=8
};

def macroop PSHUFB_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, sib, "DISPLACEMENT + 8", dataSize=8
    shuffle ufp1, xmml, xmmh, size=1, ext=0
    shuffle ufp2, xmml, xmmh, size=1, ext=0
    movfp xmml, ufp1, dataSize=8
    movfp xmmh, ufp2, dataSize=8
};

def macroop PSHUFB_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    shuffle ufp1, xmml, xmmh, size=1, ext=0
    shuffle ufp2, xmml, xmmh, size=1, ext=0
    movfp xmml, ufp1, dataSize=8
    movfp xmmh, ufp2, dataSize=8
};

def macroop PBLENDW_XMM_XMM_I {
    blend xmml, xmmlm, "IMMEDIATE & mask(8)", size=2, ext=0
    blend xmmh, xmmhm, "IMMEDIATE & mask(8)", size=2, ext=1
};

def macroop PBLENDW_XMM_M_I {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, sib, "DISPLACEMENT + 8", dataSize=8
    blend xmml, ufp1, "IMMEDIATE & mask(8)", size=2, ext=0
    blend xmmh, ufp2, "IMMEDIATE & mask(8)", size=2, ext=1
};

def macroop PBLENDW_XMM_P_I {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    blend xmml, ufp1, "IMMEDIATE & mask(8)", size=2, ext=0
    blend xmmh, ufp2, "IMMEDIATE & mask(8)", size=2, ext=1
};

def macroop BLENDPS_XMM_XMM_I {
    blend xmml, xmmlm, "IMMEDIATE & mask(4)", size=4, ext=0
    blend xmmh, xmmhm, "IMMEDIATE & mask(4)", size=4, ext=1
};

def macroop BLENDPS_XMM_M_I {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, sib, "DISPLACEMENT + 8", dataSize=8
    blend xmml, ufp1, "IMMEDIATE & mask(4)", size=4, ext=0
    blend xmmh, ufp2, "IMMEDIATE & mask(4)", size=4, ext=1
};

def macroop BLENDPS_XMM_P_I {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    blend xmml, ufp1, "IMMEDIATE & mask(4)", size=4, ext=0
    blend xmmh, ufp2, "IMMEDIATE & mask(4)", size=4, ext=1
};

def macroop BLENDPD_XMM_XMM_I {
    blend xmml, xmmlm, "IMMEDIATE & mask(2)", size=8, ext=0
    blend xmmh, xmmhm, "IMMEDIATE & mask(2)", size=8, ext=1
};

def macroop BLENDPD_XMM_M_I {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, sib, "DISPLACEMENT + 8", dataSize=8
    blend xmml, ufp1, "IMMEDIATE & mask(2)", size=8, ext=0
    blend xmmh, ufp2, "IMMEDIATE & mask(2)", size=8, ext=1
};

def macroop BLENDPD_XMM_P_I {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    blend xmml, ufp1, "IMMEDIATE & mask(2)", size=8, ext=0
    blend xmmh, ufp2, "IMMEDIATE & mask(2)", size=8, ext=1
};

def macroop BLENDVPD_XMM_XMM {
    blendxmm xmml, xmmlm, fpRegIdx("float_reg::xmmLow(0)"), size=8
    blendxmm xmmh, xmmhm, fpRegIdx("float_reg::xmmHigh(0)"), size=8
};

def macroop BLENDVPD_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, sib, "DISPLACEMENT + 8", dataSize=8
    blendxmm xmml, ufp1, fpRegIdx("float_reg::xmmLow(0)"), size=8
    blendxmm xmmh, ufp2, fpRegIdx("float_reg::xmmHigh(0)"), size=8
};

def macroop BLENDVPD_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    blendxmm xmml, ufp1, fpRegIdx("float_reg::xmmLow(0)"), size=8
    blendxmm xmmh, ufp2, fpRegIdx("float_reg::xmmHigh(0)"), size=8
};

def macroop BLENDVPS_XMM_XMM {
    blendxmm xmml, xmmlm, fpRegIdx("float_reg::xmmLow(0)"), size=4
    blendxmm xmmh, xmmhm, fpRegIdx("float_reg::xmmHigh(0)"), size=4
};

def macroop BLENDVPS_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, sib, "DISPLACEMENT + 8", dataSize=8
    blendxmm xmml, ufp1, fpRegIdx("float_reg::xmmLow(0)"), size=4
    blendxmm xmmh, ufp2, fpRegIdx("float_reg::xmmHigh(0)"), size=4
};

def macroop BLENDVPS_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    blendxmm xmml, ufp1, fpRegIdx("float_reg::xmmLow(0)"), size=4
    blendxmm xmmh, ufp2, fpRegIdx("float_reg::xmmHigh(0)"), size=4
};

def macroop PBLENDVB_XMM_XMM {
    blendxmm xmml, xmmlm, fpRegIdx("float_reg::xmmLow(0)"), size=1
    blendxmm xmmh, xmmhm, fpRegIdx("float_reg::xmmHigh(0)"), size=1
};

def macroop PBLENDVB_XMM_M {
    ldfp ufp1, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, sib, "DISPLACEMENT + 8", dataSize=8
    blendxmm xmml, ufp1, fpRegIdx("float_reg::xmmLow(0)"), size=1
    blendxmm xmmh, ufp2, fpRegIdx("float_reg::xmmHigh(0)"), size=1
};

def macroop PBLENDVB_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp ufp2, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    blendxmm xmml, ufp1, fpRegIdx("float_reg::xmmLow(0)"), size=1
    blendxmm xmmh, ufp2, fpRegIdx("float_reg::xmmHigh(0)"), size=1
};
"""
