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
def macroop HSUBPS_XMM_XMM {
    shuffle ufp1, xmml, xmmh, ext=((0 << 0) | (2 << 2)), size=4
    shuffle ufp2, xmml, xmmh, ext=((1 << 0) | (3 << 2)), size=4
    shuffle ufp3, xmmlm, xmmhm, ext=((0 << 0) | (2 << 2)), size=4
    shuffle ufp4, xmmlm, xmmhm, ext=((1 << 0) | (3 << 2)), size=4
    msubf xmml, ufp1, ufp2, size=4
    msubf xmmh, ufp3, ufp4, size=4
};
def macroop HSUBPS_XMM_M {
    ldfp ufp1, seg, sib, disp, dataSize=8
    ldfp ufp2, seg, sib, "DISPLACEMENT+8", dataSize=8
    shuffle ufp3, xmml, xmmh, ext=((0 << 0) | (2 << 2)), size=4
    shuffle ufp4, xmml, xmmh, ext=((1 << 0) | (3 << 2)), size=4
    shuffle ufp5, ufp1, ufp2, ext=((0 << 0) | (2 << 2)), size=4
    shuffle ufp6, ufp1, ufp2, ext=((1 << 0) | (3 << 2)), size=4
    msubf xmml, ufp3, ufp4, size=4
    msubf xmmh, ufp5, ufp6, size=4
};
def macroop HSUBPS_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, disp, dataSize=8
    ldfp ufp2, seg, riprel, "DISPLACEMENT+8", dataSize=8
    shuffle ufp3, xmml, xmmh, ext=((0 << 0) | (2 << 2)), size=4
    shuffle ufp4, xmml, xmmh, ext=((1 << 0) | (3 << 2)), size=4
    shuffle ufp5, ufp1, ufp2, ext=((0 << 0) | (2 << 2)), size=4
    shuffle ufp6, ufp1, ufp2, ext=((1 << 0) | (3 << 2)), size=4
    msubf xmml, ufp3, ufp4, size=4
    msubf xmmh, ufp5, ufp6, size=4
};
def macroop HSUBPD_XMM_XMM {
    msubf ufp1, xmmh , xmml, size=8, ext=Scalar
    msubf xmmh, xmmlm, xmmhm, size=8, ext=Scalar
    movfp xmml, ufp1
};
def macroop HSUBPD_XMM_M {
    ldfp ufp1, seg, sib, disp, dataSize=8
    ldfp ufp2, seg, sib, "DISPLACEMENT+8", dataSize=8
    msubf xmml, xmml, xmmh, size=8, ext=Scalar
    msubf xmmh, ufp1, ufp2, size=8, ext=Scalar
};
def macroop HSUBPD_XMM_P {
    rdip t7
    ldfp ufp1, seg, riprel, disp, dataSize=8
    ldfp ufp2, seg, riprel, "DISPLACEMENT+8", dataSize=8
    msubf xmml, xmml, xmmh, size=8, ext=Scalar
    msubf xmmh, ufp1, ufp2, size=8, ext=Scalar
};
'''
