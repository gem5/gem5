# Copyright (c) 2007 The Hewlett-Packard Development Company
# All rights reserved.
#
# Redistribution and use of this software in source and binary forms,
# with or without modification, are permitted provided that the
# following conditions are met:
#
# The software must be used only for Non-Commercial Use which means any
# use which is NOT directed to receiving any direct monetary
# compensation for, or commercial advantage from such use.  Illustrative
# examples of non-commercial use are academic research, personal study,
# teaching, education and corporate research & development.
# Illustrative examples of commercial use are distributing products for
# commercial advantage and providing services using the software for
# commercial advantage.
#
# If you wish to use this software or functionality therein that may be
# covered by patents for commercial use, please contact:
#     Director of Intellectual Property Licensing
#     Office of Strategy and Technology
#     Hewlett-Packard Company
#     1501 Page Mill Road
#     Palo Alto, California  94304
#
# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.  Redistributions
# in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.  Neither the name of
# the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.  No right of
# sublicense is granted herewith.  Derivatives of the software and
# output created using the software may be prepared, but only for
# Non-Commercial Uses.  Derivatives of the software may be shared with
# others provided: (i) the others agree to abide by the list of
# conditions herein which includes the Non-Commercial Use restrictions;
# and (ii) such Derivatives of the software include the above copyright
# notice to acknowledge the contribution from this software where
# applicable, this list of conditions and the disclaimer below.
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
    sll t3, t3, 3, dataSize=8
    # Get the selector into t1.
    sll t1, t2, t3, dataSize=8
    mov t1, t0, t1, dataSize=2
    # And get the offset into t2
    mov t2, t0, t2
    br rom_label("jmpFarWork")
};

def rom
{
    extern jmpFarWork:
    # t1 has the offset and t2 has the new selector.
    # This is intended to run in protected mode.
    andi t0, t2, 0xFC, flags=(EZF,), dataSize=2
    fault "new GeneralProtection(0)", flags=(CEZF,)
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
    wrdl cs, t4, t2
    wrsel cs, t2
    wrip t0, t1
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
    wrbase cs, t3
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
    sll t3, t3, 3, dataSize=8
    # Get the selector into t1.
    sll t1, t2, t3, dataSize=8
    mov t1, t0, t1, dataSize=2
    # And get the offset into t2
    mov t2, t0, t2
    slli t3, t3, 4, dataSize=8
    wrsel cs, t1, dataSize=2
    wrbase cs, t3
    wrip t0, t2, dataSize=asz
};
'''
