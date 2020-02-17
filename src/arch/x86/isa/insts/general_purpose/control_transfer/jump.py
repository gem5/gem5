# Copyright (c) 2007 The Hewlett-Packard Development Company
# Copyright (c) 2012-2013 AMD
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
def macroop JMP_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2
};

def macroop JMP_R
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    wripi reg, 0
};

def macroop JMP_M
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    ld t1, seg, sib, disp
    wripi t1, 0
};

def macroop JMP_P
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t7
    ld t1, seg, riprel, disp
    wripi t1, 0
};

def macroop JMP_FAR_M
{
    limm t1, 0, dataSize=8
    limm t2, 0, dataSize=8
    lea t1, seg, sib, disp, dataSize=asz
    ld t2, seg, [1, t0, t1], dsz
    ld t1, seg, [1, t0, t1]
    br rom_label("jmpFarWork")
};

def macroop JMP_FAR_P
{
    limm t1, 0, dataSize=8
    limm t2, 0, dataSize=8
    rdip t7, dataSize=asz
    lea t1, seg, riprel, disp, dataSize=asz
    ld t2, seg, [1, t0, t1], dsz
    ld t1, seg, [1, t0, t1]
    br rom_label("jmpFarWork")
};

def macroop JMP_FAR_I
{
    # Put the whole far pointer into a register.
    limm t2, imm, dataSize=8
    # Figure out the width of the offset.
    limm t3, dsz, dataSize=8
    slli t3, t3, 3, dataSize=8
    # Get the offset into t1.
    mov t1, t0, t2
    # Get the selector into t2.
    srl t2, t2, t3, dataSize=8
    mov t2, t0, t2, dataSize=2
    br rom_label("jmpFarWork")
};

def rom
{
    extern jmpFarWork:
    # t1 has the offset and t2 has the new selector.
    # This is intended to run in protected mode.
    andi t0, t2, 0xFC, flags=(EZF,), dataSize=2
    fault "std::make_shared<GeneralProtection>(0)", flags=(CEZF,)
    andi t3, t2, 0xF8, dataSize=8
    andi t0, t2, 0x4, flags=(EZF,), dataSize=2
    br rom_local_label("farJmpGlobalDescriptor"), flags=(CEZF,)
    ld t4, tsl, [1, t0, t3], dataSize=8, addressSize=8, atCPL0=True
    br rom_local_label("farJmpProcessDescriptor")
farJmpGlobalDescriptor:
    ld t4, tsg, [1, t0, t3], dataSize=8, addressSize=8, atCPL0=True
farJmpProcessDescriptor:
    rcri t0, t4, 13, flags=(ECF,), dataSize=2
    br rom_local_label("farJmpSystemDescriptor"), flags=(nCECF,)
    chks t2, t4, CSCheck, dataSize=8
    wrdl cs, t4, t2, dataSize=4
    wrsel cs, t2, dataSize=4
    wrip t0, t1, dataSize=4
    eret

farJmpSystemDescriptor:
    panic "Far jumps to system descriptors aren't implemented"
    eret
};

def macroop JMP_FAR_REAL_M
{
    lea t1, seg, sib, disp, dataSize=asz
    ld t2, seg, [1, t0, t1], dsz
    ld t1, seg, [1, t0, t1]
    zexti t3, t1, 15, dataSize=8
    slli t3, t3, 4, dataSize=8
    wrsel cs, t1, dataSize=2
    wrbase cs, t3, dataSize=8
    wrip t0, t2, dataSize=asz
};

def macroop JMP_FAR_REAL_P
{
    panic "Real mode far jump executed in 64 bit mode!"
};

def macroop JMP_FAR_REAL_I
{
    # Put the whole far pointer into a register.
    limm t2, imm, dataSize=8
    # Figure out the width of the offset.
    limm t3, dsz, dataSize=8
    slli t3, t3, 3, dataSize=8
    # Get the selector into t1.
    srl t1, t2, t3, dataSize=8
    mov t1, t0, t1, dataSize=2
    # And get the offset into t2
    mov t2, t0, t2
    slli t3, t1, 4, dataSize=8
    wrsel cs, t1, dataSize=2
    wrbase cs, t3, dataSize=8
    wrip t0, t2, dataSize=asz
};
'''
