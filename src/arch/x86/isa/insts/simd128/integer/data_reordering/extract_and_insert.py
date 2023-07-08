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
def macroop PEXTRB_R_XMM_I {
    mov2int reg, xmmlm, "IMMEDIATE & mask(4)", size=1, ext=1
    mov2int reg, xmmhm, "IMMEDIATE & mask(4)", size=1, ext=1
};

def macroop PEXTRB_M_XMM_I {
    mov2int t1, xmmlm, "IMMEDIATE & mask(4)", size=1, ext=1
    mov2int t1, xmmhm, "IMMEDIATE & mask(4)", size=1, ext=1
    st t1, seg, sib, disp, dataSize=1
};

def macroop PEXTRB_P_XMM_I {
    rdip t7
    mov2int t1, xmmlm, "IMMEDIATE & mask(4)", size=1, ext=1
    mov2int t1, xmmhm, "IMMEDIATE & mask(4)", size=1, ext=1
    st t1, seg, riprel, disp, dataSize=1
};

def macroop PEXTRW_R_XMM_I {
    mov2int reg, xmmlm, "IMMEDIATE & mask(3)", size=2, ext=1
    mov2int reg, xmmhm, "IMMEDIATE & mask(3)", size=2, ext=1
};

def macroop PEXTRW_M_XMM_I {
    mov2int t1, xmmlm, "IMMEDIATE & mask(3)", size=2, ext=1
    mov2int t1, xmmhm, "IMMEDIATE & mask(3)", size=2, ext=1
    st t1, seg, sib, disp, dataSize=2
};

def macroop PEXTRW_P_XMM_I {
    rdip t7
    mov2int t1, xmmlm, "IMMEDIATE & mask(2)", size=2, ext=1
    mov2int t1, xmmhm, "IMMEDIATE & mask(2)", size=2, ext=1
    st t1, seg, riprel, disp, dataSize=2
};

def macroop PEXTRD_R_XMM_I {
    mov2int reg, xmmlm, "IMMEDIATE & mask(2)", size=4, ext=1
    mov2int reg, xmmhm, "IMMEDIATE & mask(2)", size=4, ext=1
};

def macroop PEXTRD_M_XMM_I {
    mov2int t1, xmmlm, "IMMEDIATE & mask(2)", size=4, ext=1
    mov2int t1, xmmhm, "IMMEDIATE & mask(2)", size=4, ext=1
    st t1, seg, sib, disp, dataSize=4
};

def macroop PEXTRD_P_XMM_I {
    rdip t7
    mov2int t1, xmmlm, "IMMEDIATE & mask(2)", size=4, ext=1
    mov2int t1, xmmhm, "IMMEDIATE & mask(2)", size=4, ext=1
    st t1, seg, riprel, disp, dataSize=4
};

def macroop PEXTRQ_R_XMM_I {
    mov2int reg, xmmlm, "IMMEDIATE & mask(1)", size=8, ext=1
    mov2int reg, xmmhm, "IMMEDIATE & mask(1)", size=8, ext=1
};

def macroop PEXTRQ_M_XMM_I {
    mov2int t1, xmmlm, "IMMEDIATE & mask(1)", size=8, ext=1
    mov2int t1, xmmhm, "IMMEDIATE & mask(1)", size=8, ext=1
    st t1, seg, sib, disp, dataSize=8
};

def macroop PEXTRQ_P_XMM_I {
    rdip t7
    mov2int t1, xmmlm, "IMMEDIATE & mask(1)", size=8, ext=1
    mov2int t1, xmmhm, "IMMEDIATE & mask(1)", size=8, ext=1
    st t1, seg, riprel, disp, dataSize=8
};

def macroop PINSRB_XMM_R_I {
    mov2fp xmml, regm, "IMMEDIATE & mask(4)", size=1, ext=1
    mov2fp xmmh, regm, "IMMEDIATE & mask(4)", size=1, ext=1
};

def macroop PINSRB_XMM_M_I {
    ld t1, seg, sib, disp, dataSize=1
    mov2fp xmml, t1, "IMMEDIATE & mask(4)", size=1, ext=1
    mov2fp xmmh, t1, "IMMEDIATE & mask(4)", size=1, ext=1
};

def macroop PINSRB_XMM_P_I {
    rdip t7
    ld t1, seg, riprel, disp, dataSize=1
    mov2fp xmml, t1, "IMMEDIATE & mask(4)", size=1, ext=1
    mov2fp xmmh, t1, "IMMEDIATE & mask(4)", size=1, ext=1
};

def macroop PINSRW_XMM_R_I {
    mov2fp xmml, regm, "IMMEDIATE & mask(3)", size=2, ext=1
    mov2fp xmmh, regm, "IMMEDIATE & mask(3)", size=2, ext=1
};

def macroop PINSRW_XMM_M_I {
    ld t1, seg, sib, disp, dataSize=2
    mov2fp xmml, t1, "IMMEDIATE & mask(3)", size=2, ext=1
    mov2fp xmmh, t1, "IMMEDIATE & mask(3)", size=2, ext=1
};

def macroop PINSRW_XMM_P_I {
    rdip t7
    ld t1, seg, riprel, disp, dataSize=2
    mov2fp xmml, t1, "IMMEDIATE & mask(3)", size=2, ext=1
    mov2fp xmmh, t1, "IMMEDIATE & mask(3)", size=2, ext=1
};

def macroop PINSRD_XMM_R_I {
    mov2fp xmml, regm, "IMMEDIATE & mask(2)", size=4, ext=1
    mov2fp xmmh, regm, "IMMEDIATE & mask(2)", size=4, ext=1
};

def macroop PINSRD_XMM_M_I {
    ld t1, seg, sib, disp, dataSize=4
    mov2fp xmml, t1, "IMMEDIATE & mask(2)", size=4, ext=1
    mov2fp xmmh, t1, "IMMEDIATE & mask(2)", size=4, ext=1
};

def macroop PINSRD_XMM_P_I {
    rdip t7
    ld t1, seg, riprel, disp, dataSize=4
    mov2fp xmml, t1, "IMMEDIATE & mask(2)", size=4, ext=1
    mov2fp xmmh, t1, "IMMEDIATE & mask(2)", size=4, ext=1
};

def macroop PINSRQ_XMM_R_I {
    mov2fp xmml, regm, "IMMEDIATE & mask(1)", size=8, ext=1
    mov2fp xmmh, regm, "IMMEDIATE & mask(1)", size=8, ext=1
};

def macroop PINSRQ_XMM_M_I {
    ld t1, seg, sib, disp, dataSize=8
    mov2fp xmml, t1, "IMMEDIATE & mask(1)", size=8, ext=1
    mov2fp xmmh, t1, "IMMEDIATE & mask(1)", size=8, ext=1
};

def macroop PINSRQ_XMM_P_I {
    rdip t7
    ld t1, seg, riprel, disp, dataSize=8
    mov2fp xmml, t1, "IMMEDIATE & mask(1)", size=8, ext=1
    mov2fp xmmh, t1, "IMMEDIATE & mask(1)", size=8, ext=1
};
"""
