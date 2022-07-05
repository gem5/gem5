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
def macroop CALL_NEAR_I
{
    # Make the default data size of calls 64 bits in 64 bit mode
    .adjust_env oszIn64Override
    .function_call
    .control_direct

    limm t1, imm
    rdip t7
    # Check target of call
    st t7, ss, [0, t0, rsp], "-env.dataSize", addressSize=ssz
    subi rsp, rsp, dsz, dataSize=ssz
    wrip t7, t1
};

def macroop CALL_NEAR_R
{
    # Make the default data size of calls 64 bits in 64 bit mode
    .adjust_env oszIn64Override
    .function_call
    .control_indirect

    rdip t1
    # Check target of call
    st t1, ss, [0, t0, rsp], "-env.dataSize", addressSize=ssz
    subi rsp, rsp, dsz, dataSize=ssz
    wripi reg, 0
};

def macroop CALL_NEAR_M
{
    # Make the default data size of calls 64 bits in 64 bit mode
    .adjust_env oszIn64Override
    .function_call
    .control_indirect

    rdip t7
    ld t1, seg, sib, disp
    # Check target of call
    st t7, ss, [0, t0, rsp], "-env.dataSize", addressSize=ssz
    subi rsp, rsp, dsz, dataSize=ssz
    wripi t1, 0
};

def macroop CALL_NEAR_P
{
    # Make the default data size of calls 64 bits in 64 bit mode
    .adjust_env oszIn64Override
    .function_call
    .control_indirect

    rdip t7
    ld t1, seg, riprel, disp
    # Check target of call
    st t7, ss, [0, t0, rsp], "-env.dataSize", addressSize=ssz
    subi rsp, rsp, dsz, dataSize=ssz
    wripi t1, 0
};

def macroop CALL_FAR_REAL_I {
    .function_call
    .control_indirect

    limm t1, imm, dataSize=8

    # Multiply the data size by 8 to get the size of the offset in bits.
    limm t3, dsz, dataSize=8
    slli t3, t3, 3, dataSize=8

    # Extract the selector from the far pointer.
    srl t2, t1, t3, dataSize=8
    zexti t2, t2, 15, dataSize=8
    # Extract the offset from the far pointer.
    zext t1, t1, t3, dataSize=8

    # Compute the base.
    slli t3, t2, 4, dataSize=8

    # Make sure pushing the RIP will work after we push cs.
    cda ss, [1, t0, rsp], "-env.dataSize * 2", addressSize=ssz

    # Push CS.
    rdsel t4, cs, dataSize=8
    st t4, ss, [1, t0, rsp], "-env.dataSize", addressSize=ssz
    # Push RIP.
    rdip t7
    st t7, ss, [1, t0, rsp], "-env.dataSize * 2", addressSize=ssz
    subi rsp, rsp, "env.dataSize * 2", dataSize=ssz

    # Install the new selector, base and RIP values.
    wrsel cs, t2, dataSize=2
    wrbase cs, t3, dataSize=8
    wrip t1, t0
};

def macroop CALL_FAR_REAL_M {
    .function_call
    .control_indirect

    lea t1, seg, sib, disp, dataSize=asz
    # Load the selector.
    ld t2, seg, [1, t0, t1], dsz, dataSize=2
    # Load the offset.
    ld t1, seg, [1, t0, t1]

    # Compute the base.
    zexti t3, t2, 15, dataSize=8
    slli t3, t3, 4, dataSize=8

    # Make sure pushing the RIP will work after we push cs.
    cda ss, [1, t0, rsp], "-env.dataSize * 2", addressSize=ssz

    # Push CS.
    rdsel t4, cs, dataSize=8
    st t4, ss, [1, t0, rsp], "-env.dataSize", addressSize=ssz
    # Push RIP.
    rdip t7
    st t7, ss, [1, t0, rsp], "-env.dataSize * 2", addressSize=ssz
    subi rsp, rsp, "env.dataSize * 2", dataSize=ssz

    # Install the new selector, base and RIP values.
    wrsel cs, t2, dataSize=2
    wrbase cs, t3, dataSize=8
    wrip t1, t0
};

def macroop CALL_FAR_REAL_P {
    panic "Far call in real mode doesn't support RIP addressing."
};
"""
# let {{
#    class CALL(Inst):
#       "GenFault ${new UnimpInstFault}"
# }};
