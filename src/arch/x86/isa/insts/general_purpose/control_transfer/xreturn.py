# Copyright (c) 2007-2008 The Hewlett-Packard Development Company
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
def macroop RET_NEAR
{
    # Make the default data size of rets 64 bits in 64 bit mode
    .adjust_env oszIn64Override
    .function_return
    .control_indirect

    ld t1, ss, [1, t0, rsp], addressSize=ssz
    # Check address of return
    addi rsp, rsp, dsz, dataSize=ssz
    wripi t1, 0
};

def macroop RET_NEAR_I
{
    # Make the default data size of rets 64 bits in 64 bit mode
    .adjust_env oszIn64Override
    .function_return
    .control_indirect

    limm t2, imm
    ld t1, ss, [1, t0, rsp], addressSize=ssz
    # Check address of return
    addi rsp, rsp, dsz, dataSize=ssz
    add rsp, rsp, t2, dataSize=ssz
    wripi t1, 0
};

def macroop RET_FAR_REAL {
    .function_return
    .control_indirect

    # Pop the return RIP.
    ld t1, ss, [1, t0, rsp], addressSize=ssz
    # Pop the return CS.
    ld t2, ss, [1, t0, rsp], dsz, addressSize=ssz
    # Adjust RSP.
    addi rsp, rsp, "2 * env.dataSize", dataSize=ssz

    # Set the CS selector.
    wrsel cs, t2
    # Make sure there isn't any junk in the upper bits of the base.
    mov t2, t0, t2
    # Compute and set CS base.
    slli t2, t2, 4, dataSize=8
    wrbase cs, t2, dataSize=8

    # Set the new RIP.
    wrip t1, t0
};

def macroop RET_FAR_REAL_I {
    .function_return
    .control_indirect

    # Pop the return RIP.
    ld t1, ss, [1, t0, rsp], addressSize=ssz
    # Pop the return CS.
    ld t2, ss, [1, t0, rsp], dsz, addressSize=ssz
    # Adjust RSP.
    addi rsp, rsp, "2 * env.dataSize", dataSize=ssz

    addi rsp, rsp, imm, dataSize=ssz

    # Set the CS selector.
    wrsel cs, t2
    # Make sure there isn't any junk in the upper bits of the base.
    mov t2, t0, t2
    # Compute and set CS base.
    slli t2, t2, 4, dataSize=8
    wrbase cs, t2, dataSize=8

    # Set the new RIP.
    wrip t1, t0
};

def macroop RET_FAR {
    .adjust_env oszIn64Override
    .function_return
    .control_indirect

    # Get the return RIP
    ld t1, ss, [1, t0, rsp], addressSize=ssz

    # Get the return CS
    ld t2, ss, [1, t0, rsp], dsz, addressSize=ssz

    # increment the stack pointer to pop the instruction pointer
    # and the code segment from the stack.
    addi rsp, rsp, dsz, dataSize=ssz
    addi rsp, rsp, dsz, dataSize=ssz

    # Get the rpl
    andi t3, t2, 0x3

    # Get the cpl

    # Here we'd check if we're changing priviledge levels. We'll just hope
    # that doesn't happen yet.

    # Do stuff if they're equal
    andi t0, t2, 0xFC, flags=(EZF,), dataSize=2
    br label("processDescriptor"), flags=(CEZF,)
    andi t3, t2, 0xF8, dataSize=8
    andi t0, t2, 0x4, flags=(EZF,), dataSize=2
    br label("globalDescriptor"), flags=(CEZF,)
    ld t3, tsl, [1, t0, t3], dataSize=8, addressSize=8
    br label("processDescriptor")
globalDescriptor:
    ld t3, tsg, [1, t0, t3], dataSize=8, addressSize=8
processDescriptor:
    chks t2, t3, IretCheck, dataSize=8
    # There should be validity checks on the RIP checks here, but I'll do
    # that later.
    wrdl cs, t3, t2
    wrsel cs, t2
    wrip t0, t1
#    br label("end")

    # Do other stuff if they're not.
#end:
#    fault "NoFault"
};
"""
