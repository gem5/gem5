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

#
# Byte version of one operand unsigned multiply.
#

def macroop MUL_B_R
{
    mul1u rax, reg, flags=(OF,CF)
    mulel rax
    muleh ah
};

def macroop MUL_B_M
{
    ld t1, seg, sib, disp
    mul1u rax, t1, flags=(OF,CF)
    mulel rax
    muleh ah
};

def macroop MUL_B_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mul1u rax, t1, flags=(OF,CF)
    mulel rax
    muleh ah
};

#
# One operand unsigned multiply.
#

def macroop MUL_R
{
    mul1u rax, reg, flags=(OF,CF)
    mulel rax
    muleh rdx
};

def macroop MUL_M
{
    ld t1, seg, sib, disp
    mul1u rax, t1, flags=(OF,CF)
    mulel rax
    muleh rdx
};

def macroop MUL_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mul1u rax, t1, flags=(OF,CF)
    mulel rax
    muleh rdx
};

#
# Byte version of one operand signed multiply.
#

def macroop IMUL_B_R
{
    mul1s rax, reg, flags=(OF,CF)
    mulel rax
    muleh ah
};

def macroop IMUL_B_M
{
    ld t1, seg, sib, disp
    mul1s rax, t1, flags=(OF,CF)
    mulel rax
    muleh ah
};

def macroop IMUL_B_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mul1s rax, t1, flags=(OF,CF)
    mulel rax
    muleh ah
};

#
# One operand signed multiply.
#

def macroop IMUL_R
{
    mul1s rax, reg, flags=(OF,CF)
    mulel rax
    muleh rdx
};

def macroop IMUL_M
{
    ld t1, seg, sib, disp
    mul1s rax, t1, flags=(OF,CF)
    mulel rax
    muleh rdx
};

def macroop IMUL_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mul1s rax, t1, flags=(OF,CF)
    mulel rax
    muleh rdx
};

def macroop IMUL_R_R
{
    mul1s reg, regm, flags=(OF,CF)
    mulel reg
    muleh t0
};

def macroop IMUL_R_M
{
    ld t1, seg, sib, disp
    mul1s reg, t1, flags=(CF,OF)
    mulel reg
    muleh t0
};

def macroop IMUL_R_P
{
    rdip t7
    ld t1, seg, riprel, disp
    mul1s reg, t1, flags=(CF,OF)
    mulel reg
    muleh t0
};

#
# Three operand signed multiply.
#

def macroop IMUL_R_R_I
{
    limm t1, imm
    mul1s regm, t1, flags=(OF,CF)
    mulel reg
    muleh t0
};

def macroop IMUL_R_M_I
{
    limm t1, imm
    ld t2, seg, sib, disp
    mul1s t2, t1, flags=(OF,CF)
    mulel reg
    muleh t0
};

def macroop IMUL_R_P_I
{
    rdip t7
    limm t1, imm
    ld t2, seg, riprel
    mul1s t2, t1, flags=(OF,CF)
    mulel reg
    muleh t0
};

#
# One byte version of unsigned division
#

def macroop DIV_B_R
{
    # Do the initial part of the division
    div1 ah, reg, dataSize=1

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t1, rax, 8, dataSize=1
    div2 t1, rax, t1, dataSize=1

    #Loop until we're out of bits to shift in
divLoopTop:
    div2 t1, rax, t1, dataSize=1
    div2 t1, rax, t1, flags=(EZF,), dataSize=1
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq rax, dataSize=1
    divr ah, dataSize=1
};

def macroop DIV_B_M
{
    ld t2, seg, sib, disp

    # Do the initial part of the division
    div1 ah, t2, dataSize=1

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t1, rax, 8, dataSize=1
    div2 t1, rax, t1, dataSize=1

    #Loop until we're out of bits to shift in
divLoopTop:
    div2 t1, rax, t1, dataSize=1
    div2 t1, rax, t1, flags=(EZF,), dataSize=1
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq rax, dataSize=1
    divr ah, dataSize=1
};

def macroop DIV_B_P
{
    rdip t7
    ld t2, seg, riprel, disp

    # Do the initial part of the division
    div1 ah, t2, dataSize=1

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t1, rax, 8, dataSize=1
    div2 t1, rax, t1, dataSize=1

    #Loop until we're out of bits to shift in
divLoopTop:
    div2 t1, rax, t1, dataSize=1
    div2 t1, rax, t1, flags=(EZF,), dataSize=1
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq rax, dataSize=1
    divr ah, dataSize=1
};

#
# Unsigned division
#

def macroop DIV_R
{
    # Do the initial part of the division
    div1 rdx, reg

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t1, rax, "env.dataSize * 8"
    div2 t1, rax, t1

    #Loop until we're out of bits to shift in
    #The amount of unrolling here could stand some tuning
divLoopTop:
    div2 t1, rax, t1
    div2 t1, rax, t1
    div2 t1, rax, t1
    div2 t1, rax, t1, flags=(EZF,)
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq rax
    divr rdx
};

def macroop DIV_M
{
    ld t2, seg, sib, disp

    # Do the initial part of the division
    div1 rdx, t2

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t1, rax, "env.dataSize * 8"
    div2 t1, rax, t1

    #Loop until we're out of bits to shift in
    #The amount of unrolling here could stand some tuning
divLoopTop:
    div2 t1, rax, t1
    div2 t1, rax, t1
    div2 t1, rax, t1
    div2 t1, rax, t1, flags=(EZF,)
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq rax
    divr rdx
};

def macroop DIV_P
{
    rdip t7
    ld t2, seg, riprel, disp

    # Do the initial part of the division
    div1 rdx, t2

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t1, rax, "env.dataSize * 8"
    div2 t1, rax, t1

    #Loop until we're out of bits to shift in
    #The amount of unrolling here could stand some tuning
divLoopTop:
    div2 t1, rax, t1
    div2 t1, rax, t1
    div2 t1, rax, t1
    div2 t1, rax, t1, flags=(EZF,)
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq rax
    divr rdx
};

#
# One byte version of signed division
#

def macroop IDIV_B_R
{
    # Negate dividend
    sub t1, t0, rax, flags=(ECF,), dataSize=1
    ruflag t4, 3
    sub t2, t0, ah, dataSize=1
    sub t2, t2, t4

    #Find the sign of the divisor
    slli t0, reg, 1, flags=(ECF,), dataSize=1

    # Negate divisor
    sub t3, t0, reg, dataSize=1
    # Put the divisor's absolute value into t3
    mov t3, t3, reg, flags=(nCECF,), dataSize=1

    #Find the sign of the dividend
    slli t0, ah, 1, flags=(ECF,), dataSize=1

    # Put the dividend's absolute value into t1 and t2
    mov t1, t1, rax, flags=(nCECF,), dataSize=1
    mov t2, t2, ah, flags=(nCECF,), dataSize=1

    # Do the initial part of the division
    div1 t2, t3, dataSize=1

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t4, t1, 8, dataSize=1
    div2 t4, t1, t4, dataSize=1

    #Loop until we're out of bits to shift in
divLoopTop:
    div2 t4, t1, t4, dataSize=1
    div2 t4, t1, t4, flags=(EZF,), dataSize=1
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq t5, dataSize=1
    divr t6, dataSize=1

    # Fix up signs. The sign of the dividend is still lying around in ECF.
    # The sign of the remainder, ah, is the same as the dividend. The sign
    # of the quotient is negated if the signs of the divisor and dividend
    # were different.

    # Negate the remainder
    sub t4, t0, t6, dataSize=1
    # If the dividend was negitive, put the negated remainder in ah.
    mov ah, ah, t4, (CECF,), dataSize=1
    # Otherwise put the regular remainder in ah.
    mov ah, ah, t6, (nCECF,), dataSize=1

    # Negate the quotient.
    sub t4, t0, t5, dataSize=1
    # If the dividend was negative, start using the negated quotient
    mov t5, t5, t4, (CECF,), dataSize=1

    # Check the sign of the divisor
    slli t0, reg, 1, flags=(ECF,), dataSize=1

    # Negate the (possibly already negated) quotient
    sub t4, t0, t5, dataSize=1
    # If the divisor was negative, put the negated quotient in rax.
    mov rax, rax, t4, (CECF,), dataSize=1
    # Otherwise put the one that wasn't negated (at least here) in rax.
    mov rax, rax, t5, (nCECF,), dataSize=1
};

def macroop IDIV_B_M
{
    # Negate dividend
    sub t1, t0, rax, flags=(ECF,), dataSize=1
    ruflag t4, 3
    sub t2, t0, ah, dataSize=1
    sub t2, t2, t4

    ld t8, seg, sib, disp

    #Find the sign of the divisor
    slli t0, t3, 1, flags=(ECF,), dataSize=1

    # Negate divisor
    sub t3, t0, t8, dataSize=1
    # Put the divisor's absolute value into t3
    mov t3, t3, t8, flags=(nCECF,), dataSize=1

    #Find the sign of the dividend
    slli t0, ah, 1, flags=(ECF,), dataSize=1

    # Put the dividend's absolute value into t1 and t2
    mov t1, t1, rax, flags=(nCECF,), dataSize=1
    mov t2, t2, ah, flags=(nCECF,), dataSize=1

    # Do the initial part of the division
    div1 t2, t3, dataSize=1

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t4, t1, 8, dataSize=1
    div2 t4, t1, t4, dataSize=1

    #Loop until we're out of bits to shift in
divLoopTop:
    div2 t4, t1, t4, dataSize=1
    div2 t4, t1, t4, flags=(EZF,), dataSize=1
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq t5, dataSize=1
    divr t6, dataSize=1

    # Fix up signs. The sign of the dividend is still lying around in ECF.
    # The sign of the remainder, ah, is the same as the dividend. The sign
    # of the quotient is negated if the signs of the divisor and dividend
    # were different.

    # Negate the remainder
    sub t4, t0, t6, dataSize=1
    # If the dividend was negitive, put the negated remainder in ah.
    mov ah, ah, t4, (CECF,), dataSize=1
    # Otherwise put the regular remainder in ah.
    mov ah, ah, t6, (nCECF,), dataSize=1

    # Negate the quotient.
    sub t4, t0, t5, dataSize=1
    # If the dividend was negative, start using the negated quotient
    mov t5, t5, t4, (CECF,), dataSize=1

    # Check the sign of the divisor
    slli t0, t8, 1, flags=(ECF,), dataSize=1

    # Negate the (possibly already negated) quotient
    sub t4, t0, t5, dataSize=1
    # If the divisor was negative, put the negated quotient in rax.
    mov rax, rax, t4, (CECF,), dataSize=1
    # Otherwise put the one that wasn't negated (at least here) in rax.
    mov rax, rax, t5, (nCECF,), dataSize=1
};

def macroop IDIV_B_P
{
    # Negate dividend
    sub t1, t0, rax, flags=(ECF,), dataSize=1
    ruflag t4, 3
    sub t2, t0, ah, dataSize=1
    sub t2, t2, t4

    rdip t7
    ld t8, seg, riprel, disp

    #Find the sign of the divisor
    slli t0, t3, 1, flags=(ECF,), dataSize=1

    # Negate divisor
    sub t3, t0, t8, dataSize=1
    # Put the divisor's absolute value into t3
    mov t3, t3, t8, flags=(nCECF,), dataSize=1

    #Find the sign of the dividend
    slli t0, ah, 1, flags=(ECF,), dataSize=1

    # Put the dividend's absolute value into t1 and t2
    mov t1, t1, rax, flags=(nCECF,), dataSize=1
    mov t2, t2, ah, flags=(nCECF,), dataSize=1

    # Do the initial part of the division
    div1 t2, t3, dataSize=1

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t4, t1, 8, dataSize=1
    div2 t4, t1, t4, dataSize=1

    #Loop until we're out of bits to shift in
divLoopTop:
    div2 t4, t1, t4, dataSize=1
    div2 t4, t1, t4, flags=(EZF,), dataSize=1
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq t5, dataSize=1
    divr t6, dataSize=1

    # Fix up signs. The sign of the dividend is still lying around in ECF.
    # The sign of the remainder, ah, is the same as the dividend. The sign
    # of the quotient is negated if the signs of the divisor and dividend
    # were different.

    # Negate the remainder
    sub t4, t0, t6, dataSize=1
    # If the dividend was negitive, put the negated remainder in ah.
    mov ah, ah, t4, (CECF,), dataSize=1
    # Otherwise put the regular remainder in ah.
    mov ah, ah, t6, (nCECF,), dataSize=1

    # Negate the quotient.
    sub t4, t0, t5, dataSize=1
    # If the dividend was negative, start using the negated quotient
    mov t5, t5, t4, (CECF,), dataSize=1

    # Check the sign of the divisor
    slli t0, t8, 1, flags=(ECF,), dataSize=1

    # Negate the (possibly already negated) quotient
    sub t4, t0, t5, dataSize=1
    # If the divisor was negative, put the negated quotient in rax.
    mov rax, rax, t4, (CECF,), dataSize=1
    # Otherwise put the one that wasn't negated (at least here) in rax.
    mov rax, rax, t5, (nCECF,), dataSize=1
};

#
# Signed division
#

def macroop IDIV_R
{
    # Negate dividend
    sub t1, t0, rax, flags=(ECF,)
    ruflag t4, 3
    sub t2, t0, rdx
    sub t2, t2, t4

    #Find the sign of the divisor
    slli t0, reg, 1, flags=(ECF,)

    # Negate divisor
    sub t3, t0, reg
    # Put the divisor's absolute value into t3
    mov t3, t3, reg, flags=(nCECF,)

    #Find the sign of the dividend
    slli t0, rdx, 1, flags=(ECF,)

    # Put the dividend's absolute value into t1 and t2
    mov t1, t1, rax, flags=(nCECF,)
    mov t2, t2, rdx, flags=(nCECF,)

    # Do the initial part of the division
    div1 t2, t3

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t4, t1, "env.dataSize * 8"
    div2 t4, t1, t4

    #Loop until we're out of bits to shift in
divLoopTop:
    div2 t4, t1, t4
    div2 t4, t1, t4
    div2 t4, t1, t4
    div2 t4, t1, t4, flags=(EZF,)
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq t5
    divr t6

    # Fix up signs. The sign of the dividend is still lying around in ECF.
    # The sign of the remainder, ah, is the same as the dividend. The sign
    # of the quotient is negated if the signs of the divisor and dividend
    # were different.

    # Negate the remainder
    sub t4, t0, t6
    # If the dividend was negitive, put the negated remainder in rdx.
    mov rdx, rdx, t4, (CECF,)
    # Otherwise put the regular remainder in rdx.
    mov rdx, rdx, t6, (nCECF,)

    # Negate the quotient.
    sub t4, t0, t5
    # If the dividend was negative, start using the negated quotient
    mov t5, t5, t4, (CECF,)

    # Check the sign of the divisor
    slli t0, reg, 1, flags=(ECF,)

    # Negate the (possibly already negated) quotient
    sub t4, t0, t5
    # If the divisor was negative, put the negated quotient in rax.
    mov rax, rax, t4, (CECF,)
    # Otherwise put the one that wasn't negated (at least here) in rax.
    mov rax, rax, t5, (nCECF,)
};

def macroop IDIV_M
{
    # Negate dividend
    sub t1, t0, rax, flags=(ECF,)
    ruflag t4, 3
    sub t2, t0, rdx
    sub t2, t2, t4

    ld t8, seg, sib, disp

    #Find the sign of the divisor
    #FIXME!!! This depends on shifts setting the carry flag correctly.
    slli t0, t3, 1, flags=(ECF,)

    # Negate divisor
    sub t3, t0, t8
    # Put the divisor's absolute value into t3
    mov t3, t3, t8, flags=(nCECF,)

    #Find the sign of the dividend
    #FIXME!!! This depends on shifts setting the carry flag correctly.
    slli t0, rdx, 1, flags=(ECF,)

    # Put the dividend's absolute value into t1 and t2
    mov t1, t1, rax, flags=(nCECF,)
    mov t2, t2, rdx, flags=(nCECF,)

    # Do the initial part of the division
    div1 t2, t3

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t4, t1, "env.dataSize * 8"
    div2 t4, t1, t4

    #Loop until we're out of bits to shift in
divLoopTop:
    div2 t4, t1, t4
    div2 t4, t1, t4
    div2 t4, t1, t4
    div2 t4, t1, t4, flags=(EZF,)
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq t5
    divr t6

    # Fix up signs. The sign of the dividend is still lying around in ECF.
    # The sign of the remainder, ah, is the same as the dividend. The sign
    # of the quotient is negated if the signs of the divisor and dividend
    # were different.

    # Negate the remainder
    sub t4, t0, t6
    # If the dividend was negitive, put the negated remainder in rdx.
    mov rdx, rdx, t4, (CECF,)
    # Otherwise put the regular remainder in rdx.
    mov rdx, rdx, t6, (nCECF,)

    # Negate the quotient.
    sub t4, t0, t5
    # If the dividend was negative, start using the negated quotient
    mov t5, t5, t4, (CECF,)

    # Check the sign of the divisor
    slli t0, t8, 1, flags=(ECF,)

    # Negate the (possibly already negated) quotient
    sub t4, t0, t5
    # If the divisor was negative, put the negated quotient in rax.
    mov rax, rax, t4, (CECF,)
    # Otherwise put the one that wasn't negated (at least here) in rax.
    mov rax, rax, t5, (nCECF,)
};

def macroop IDIV_P
{
    # Negate dividend
    sub t1, t0, rax, flags=(ECF,)
    ruflag t4, 3
    sub t2, t0, rdx
    sub t2, t2, t4

    rdip t7
    ld t8, seg, riprel, disp

    #Find the sign of the divisor
    #FIXME!!! This depends on shifts setting the carry flag correctly.
    slli t0, t3, 1, flags=(ECF,)

    # Negate divisor
    sub t3, t0, t8
    # Put the divisor's absolute value into t3
    mov t3, t3, t4, flags=(nCECF,)

    #Find the sign of the dividend
    #FIXME!!! This depends on shifts setting the carry flag correctly.
    slli t0, rdx, 1, flags=(ECF,)

    # Put the dividend's absolute value into t1 and t2
    mov t1, t1, rax, flags=(nCECF,)
    mov t2, t2, rdx, flags=(nCECF,)

    # Do the initial part of the division
    div1 t2, t3

    #These are split out so we can initialize the number of bits in the
    #second register
    div2i t4, t1, "env.dataSize * 8"
    div2 t4, t1, t4

    #Loop until we're out of bits to shift in
divLoopTop:
    div2 t4, t1, t4
    div2 t4, t1, t4
    div2 t4, t1, t4
    div2 t4, t1, t4, flags=(EZF,)
    br label("divLoopTop"), flags=(nCEZF,)

    #Unload the answer
    divq t5
    divr t6

    # Fix up signs. The sign of the dividend is still lying around in ECF.
    # The sign of the remainder, ah, is the same as the dividend. The sign
    # of the quotient is negated if the signs of the divisor and dividend
    # were different.

    # Negate the remainder
    sub t4, t0, t6
    # If the dividend was negitive, put the negated remainder in rdx.
    mov rdx, rdx, t4, (CECF,)
    # Otherwise put the regular remainder in rdx.
    mov rdx, rdx, t6, (nCECF,)

    # Negate the quotient.
    sub t4, t0, t5
    # If the dividend was negative, start using the negated quotient
    mov t5, t5, t4, (CECF,)

    # Check the sign of the divisor
    slli t0, t8, 1, flags=(ECF,)

    # Negate the (possibly already negated) quotient
    sub t4, t0, t5
    # If the divisor was negative, put the negated quotient in rax.
    mov rax, rax, t4, (CECF,)
    # Otherwise put the one that wasn't negated (at least here) in rax.
    mov rax, rax, t5, (nCECF,)
};
'''
