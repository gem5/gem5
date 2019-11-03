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
def macroop MOVAPS_XMM_M {
    # Check low address.
    ldfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
    ldfp xmml, seg, sib, disp, dataSize=8
};

def macroop MOVAPS_XMM_P {
    rdip t7
    # Check low address.
    ldfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    ldfp xmml, seg, riprel, disp, dataSize=8
};

def macroop MOVAPS_M_XMM {
    # Check low address.
    stfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
    stfp xmml, seg, sib, disp, dataSize=8
};

def macroop MOVAPS_P_XMM {
    rdip t7
    # Check low address.
    stfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
    stfp xmml, seg, riprel, disp, dataSize=8
};

def macroop MOVAPS_XMM_XMM {
    # Check low address.
    movfp xmml, xmmlm, dataSize=8
    movfp xmmh, xmmhm, dataSize=8
};

def macroop MOVAPD_XMM_XMM {
    movfp xmml, xmmlm, dataSize=8
    movfp xmmh, xmmhm, dataSize=8
};

def macroop MOVAPD_XMM_M {
    ldfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVAPD_XMM_P {
    rdip t7
    ldfp xmml, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVAPD_M_XMM {
    stfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    stfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVAPD_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, "DISPLACEMENT", dataSize=8
    stfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVUPS_XMM_XMM {
    movfp xmml, xmmlm, dataSize=8
    movfp xmmh, xmmhm, dataSize=8
};

def macroop MOVUPS_XMM_M {
    ldfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVUPS_XMM_P {
    rdip t7
    ldfp xmml, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVUPS_M_XMM {
    stfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    stfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVUPS_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, "DISPLACEMENT", dataSize=8
    stfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVUPD_XMM_XMM {
    movfp xmml, xmmlm, dataSize=8
    movfp xmmh, xmmhm, dataSize=8
};

def macroop MOVUPD_XMM_M {
    ldfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVUPD_XMM_P {
    rdip t7
    ldfp xmml, seg, riprel, "DISPLACEMENT", dataSize=8
    ldfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVUPD_M_XMM {
    stfp xmml, seg, sib, "DISPLACEMENT", dataSize=8
    stfp xmmh, seg, sib, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVUPD_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, "DISPLACEMENT", dataSize=8
    stfp xmmh, seg, riprel, "DISPLACEMENT + 8", dataSize=8
};

def macroop MOVHPS_XMM_M {
    ldfp xmmh, seg, sib, disp, dataSize=8
};

def macroop MOVHPS_XMM_P {
    rdip t7
    ldfp xmmh, seg, riprel, disp, dataSize=8
};

def macroop MOVHPS_M_XMM {
    stfp xmmh, seg, sib, disp, dataSize=8
};

def macroop MOVHPS_P_XMM {
    rdip t7
    stfp xmmh, seg, riprel, disp, dataSize=8
};

def macroop MOVHPD_XMM_M {
    ldfp xmmh, seg, sib, disp, dataSize=8
};

def macroop MOVHPD_XMM_P {
    rdip t7
    ldfp xmmh, seg, riprel, disp, dataSize=8
};

def macroop MOVHPD_M_XMM {
    stfp xmmh, seg, sib, disp, dataSize=8
};

def macroop MOVHPD_P_XMM {
    rdip t7
    stfp xmmh, seg, riprel, disp, dataSize=8
};

def macroop MOVLPS_XMM_M {
    ldfp xmml, seg, sib, disp, dataSize=8
};

def macroop MOVLPS_XMM_P {
    rdip t7
    ldfp xmml, seg, riprel, disp, dataSize=8
};

def macroop MOVLPS_M_XMM {
    stfp xmml, seg, sib, disp, dataSize=8
};

def macroop MOVLPS_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, disp, dataSize=8
};

def macroop MOVLPD_XMM_M {
    ldfp xmml, seg, sib, disp, dataSize=8
};

def macroop MOVLPD_XMM_P {
    rdip t7
    ldfp xmml, seg, riprel, disp, dataSize=8
};

def macroop MOVLPD_M_XMM {
    stfp xmml, seg, sib, disp, dataSize=8
};

def macroop MOVLPD_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, disp, dataSize=8
};

def macroop MOVHLPS_XMM_XMM {
    movfp xmml, xmmhm, dataSize=8
};

def macroop MOVLHPS_XMM_XMM {
    movfp xmmh, xmmlm, dataSize=8
};

def macroop MOVSS_XMM_XMM {
    movfp xmml, xmmlm, dataSize=4
};

def macroop MOVSS_XMM_M {
    lfpimm xmml, 0
    lfpimm xmmh, 0
    ldfp xmml, seg, sib, disp, dataSize=4
};

def macroop MOVSS_XMM_P {
    rdip t7
    lfpimm xmml, 0
    lfpimm xmmh, 0
    ldfp xmml, seg, riprel, disp, dataSize=4
};

def macroop MOVSS_M_XMM {
    stfp xmml, seg, sib, disp, dataSize=4
};

def macroop MOVSS_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, disp, dataSize=4
};

def macroop MOVSD_XMM_M {
    lfpimm xmmh, 0
    ldfp xmml, seg, sib, disp, dataSize=8
};

def macroop MOVSD_XMM_P {
    rdip t7
    lfpimm xmmh, 0
    ldfp xmml, seg, riprel, disp, dataSize=8
};

def macroop MOVSD_M_XMM {
    stfp xmml, seg, sib, disp, dataSize=8
};

def macroop MOVSD_P_XMM {
    rdip t7
    stfp xmml, seg, riprel, disp, dataSize=8
};

def macroop MOVSD_XMM_XMM {
    movfp xmml, xmmlm, dataSize=8
};
'''
