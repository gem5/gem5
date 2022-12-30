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
    ld t2, seg, riprel, disp
    mul1s t2, t1, flags=(OF,CF)
    mulel reg
    muleh t0
};
"""

pcRel = """
    rdip t7
    ld %s, seg, riprel, disp
"""
sibRel = """
    ld %s, seg, sib, disp
"""

#
# One byte version of unsigned division
#

divcode = """
def macroop DIV_B_%(suffix)s
{
    %(readOp1)s
    # Do the initial part of the division
    div1 ah, %(op1)s, dataSize=1

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
"""

#
# Unsigned division
#

divcode += """
def macroop DIV_%(suffix)s
{
    %(readOp1)s
    # Do the initial part of the division
    div1 rdx, %(op1)s

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
"""

#
# One byte version of signed division
#

divcode += """
def macroop IDIV_B_%(suffix)s
{
    # Negate dividend
    sub t1, t0, rax, flags=(ECF,), dataSize=1
    ruflag t4, 3
    sub t2, t0, ah, dataSize=1
    sub t2, t2, t4

    %(readOp1)s

    #Find the sign of the divisor
    slli t0, %(op1)s, 1, flags=(ECF,), dataSize=1

    # Negate divisor
    sub t3, t0, %(op1)s, dataSize=1
    # Put the divisor's absolute value into t3
    mov t3, t3, %(op1)s, flags=(nCECF,), dataSize=1

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
    slli t0, %(op1)s, 1, flags=(ECF,), dataSize=1

    # Negate the (possibly already negated) quotient
    sub t4, t0, t5, dataSize=1
    # If the divisor was negative, put the negated quotient in rax.
    mov rax, rax, t4, (CECF,), dataSize=1
    # Otherwise put the one that wasn't negated (at least here) in rax.
    mov rax, rax, t5, (nCECF,), dataSize=1
};
"""

#
# Signed division
#

divcode += """
def macroop IDIV_%(suffix)s
{
    # Negate dividend
    sub t1, t0, rax, flags=(ECF,)
    ruflag t4, 3
    sub t2, t0, rdx
    sub t2, t2, t4

    %(readOp1)s

    #Find the sign of the divisor
    slli t0, %(op1)s, 1, flags=(ECF,)

    # Negate divisor
    sub t3, t0, %(op1)s
    # Put the divisor's absolute value into t3
    mov t3, t3, %(op1)s, flags=(nCECF,)

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
    slli t0, %(op1)s, 1, flags=(ECF,)

    # Negate the (possibly already negated) quotient
    sub t4, t0, t5
    # If the divisor was negative, put the negated quotient in rax.
    mov rax, rax, t4, (CECF,)
    # Otherwise put the one that wasn't negated (at least here) in rax.
    mov rax, rax, t5, (nCECF,)
};
"""

microcode += divcode % {"suffix": "R", "readOp1": "", "op1": "reg"}
microcode += divcode % {"suffix": "M", "readOp1": sibRel % "t2", "op1": "t2"}
microcode += divcode % {"suffix": "P", "readOp1": pcRel % "t2", "op1": "t2"}
