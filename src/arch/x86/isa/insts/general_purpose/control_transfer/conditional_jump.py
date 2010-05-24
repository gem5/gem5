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
def macroop JZ_I
{
    # Make the defualt data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(CZF,)
};

def macroop JNZ_I
{
    # Make the defualt data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(nCZF,)
};

def macroop JB_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(CCF,)
};

def macroop JNB_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(nCCF,)
};

def macroop JBE_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(CCvZF,)
};

def macroop JNBE_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(nCCvZF,)
};

def macroop JS_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(CSF,)
};

def macroop JNS_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(nCSF,)
};

def macroop JP_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(CPF,)
};

def macroop JNP_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(nCPF,)
};

def macroop JL_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(CSxOF,)
};

def macroop JNL_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(nCSxOF,)
};

def macroop JLE_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(CSxOvZF,)
};

def macroop JNLE_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(nCSxOvZF,)
};

def macroop JO_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(COF,)
};

def macroop JNO_I
{
    # Make the default data size of jumps 64 bits in 64 bit mode
    .adjust_env oszIn64Override

    rdip t1
    limm t2, imm
    wrip t1, t2, flags=(nCOF,)
};

def macroop JRCX_I
{
    rdip t1
    add t0, t0, rcx, flags=(EZF,), dataSize=asz
    wripi t1, imm, flags=(CEZF,)
};
'''
