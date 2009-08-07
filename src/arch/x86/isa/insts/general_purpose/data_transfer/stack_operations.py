# Copyright (c) 2007-2008 The Hewlett-Packard Development Company
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
def macroop POP_R {
    # Make the default data size of pops 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    ld t1, ss, [1, t0, rsp], dataSize=ssz
    addi rsp, rsp, ssz, dataSize=asz
    mov reg, reg, t1
};

def macroop POP_M {
    # Make the default data size of pops 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    ld t1, ss, [1, t0, rsp], dataSize=ssz
    cda seg, sib, disp, dataSize=ssz
    addi rsp, rsp, ssz, dataSize=asz
    st t1, seg, sib, disp, dataSize=ssz
};

def macroop POP_P {
    # Make the default data size of pops 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t7
    ld t1, ss, [1, t0, rsp], dataSize=ssz
    cda seg, sib, disp, dataSize=ssz
    addi rsp, rsp, ssz, dataSize=asz
    st t1, seg, riprel, disp, dataSize=ssz
};

def macroop PUSH_R {
    # Make the default data size of pops 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    stupd reg, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
};

def macroop PUSH_I {
    # Make the default data size of pops 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    limm t1, imm
    stupd t1, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
};

def macroop PUSH_M {
    # Make the default data size of pops 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    ld t1, seg, sib, disp, dataSize=ssz
    stupd t1, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
};

def macroop PUSH_P {
    # Make the default data size of pops 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t7
    ld t1, seg, riprel, disp, dataSize=ssz
    stupd t1, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
};

def macroop PUSHA {
    # Check all the stack addresses. We'll assume that if the beginning and
    # end are ok, then the stuff in the middle should be as well.
    cda ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
    cda ss, [1, t0, rsp], "-8 * env.stackSize", dataSize=ssz
    stupd rax, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
    stupd rcx, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
    stupd rdx, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
    stupd rbx, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
    stupd rsp, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
    stupd rbp, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
    stupd rsi, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
    stupd rdi, ss, [1, t0, rsp], "-env.stackSize", dataSize=ssz
};

def macroop POPA {
    # Check all the stack addresses. We'll assume that if the beginning and
    # end are ok, then the stuff in the middle should be as well.
    ld t1, ss, [1, t0, rsp], "0 * env.stackSize", dataSize=ssz
    ld t2, ss, [1, t0, rsp], "7 * env.stackSize", dataSize=ssz
    mov rdi, rdi, t1, dataSize=ssz
    ld rsi, ss, [1, t0, rsp], "1 * env.stackSize", dataSize=ssz
    ld rbp, ss, [1, t0, rsp], "2 * env.stackSize", dataSize=ssz
    ld rbx, ss, [1, t0, rsp], "4 * env.stackSize", dataSize=ssz
    ld rdx, ss, [1, t0, rsp], "5 * env.stackSize", dataSize=ssz
    ld rcx, ss, [1, t0, rsp], "6 * env.stackSize", dataSize=ssz
    mov rax, rax, t2, dataSize=ssz
    addi rsp, rsp, "8 * env.stackSize", dataSize=asz
};

def macroop LEAVE {
    # Make the default data size of pops 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    mov t1, t1, rbp, dataSize=ssz
    ld rbp, ss, [1, t0, t1], dataSize=ssz
    mov rsp, rsp, t1, dataSize=ssz
    addi rsp, rsp, ssz, dataSize=ssz
};

def macroop ENTER_I_I {
    .adjust_env oszIn64Override
    # This needs to check all the addresses it writes to before it actually
    # writes any values.

    # Pull the different components out of the immediate
    limm t1, imm, dataSize=8
    zexti t2, t1, 15, dataSize=8
    srli t1, t1, 16, dataSize=8
    zexti t1, t1, 5, dataSize=8
    # t1 is now the masked nesting level, and t2 is the amount of storage.

    # Push rbp.
    stupd rbp, ss, [1, t0, rsp], "-env.dataSize"

    # Save the stack pointer for later
    mov t6, t6, rsp

    # If the nesting level is zero, skip all this stuff.
    sub t0, t1, t0, flags=(EZF,), dataSize=2
    br label("skipLoop"), flags=(CEZF,)

    # If the level was 1, only push the saved rbp
    subi t0, t1, 1, flags=(EZF,)
    br label("bottomOfLoop"), flags=(CEZF,)

    limm t4, "ULL(-1)", dataSize=8
topOfLoop:
    ld t5, ss, [dsz, t4, rbp]
    stupd t5, ss, [1, t0, rsp], "-env.dataSize"

    # If we're not done yet, loop
    subi t4, t4, 1, dataSize=8
    add t0, t4, t1, flags=(EZF,)
    br label("topOfLoop"), flags=(nCEZF,)

bottomOfLoop:
    # Push the old rbp onto the stack
    stupd t6, ss, [1, t0, rsp], "-env.dataSize"

skipLoop:
    sub rsp, rsp, t2, dataSize=ssz
    mov rbp, rbp, t6
};
'''
