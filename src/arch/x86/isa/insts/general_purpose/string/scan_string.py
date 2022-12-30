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
def macroop SCAS_M {
    # Find the constant we need to either add or subtract from rdi
    ruflag t0, 10
    movi t2, t2, dsz, flags=(CEZF,), dataSize=asz
    subi t3, t0, dsz, dataSize=asz
    mov t2, t2, t3, flags=(nCEZF,), dataSize=asz

    ld t1, es, [1, t0, rdi]
    sub t0, rax, t1, flags=(OF, SF, ZF, AF, PF, CF)

    add rdi, rdi, t2, dataSize=asz
};

#
# Versions which have the rep prefix. These could benefit from some loop
# unrolling.
#

def macroop SCAS_E_M {
    and t0, rcx, rcx, flags=(EZF,), dataSize=asz
    br label("end"), flags=(CEZF,)

    # Find the constant we need to either add or subtract from rdi
    ruflag t0, 10
    movi t2, t2, dsz, flags=(CEZF,), dataSize=asz
    subi t3, t0, dsz, dataSize=asz
    mov t2, t2, t3, flags=(nCEZF,), dataSize=asz

topOfLoop:
    ld t1, es, [1, t0, rdi]
    sub t0, rax, t1, flags=(OF, SF, ZF, AF, PF, CF)

    subi rcx, rcx, 1, flags=(EZF,), dataSize=asz
    add rdi, rdi, t2, dataSize=asz
    br label("topOfLoop"), flags=(CSTRZnEZF,)
end:
    fault "NoFault"
};

def macroop SCAS_N_M {
    and t0, rcx, rcx, flags=(EZF,), dataSize=asz
    br label("end"), flags=(CEZF,)

    # Find the constant we need to either add or subtract from rdi
    ruflag t0, 10
    movi t2, t2, dsz, flags=(CEZF,), dataSize=asz
    subi t3, t0, dsz, dataSize=asz
    mov t2, t2, t3, flags=(nCEZF,), dataSize=asz

topOfLoop:
    ld t1, es, [1, t0, rdi]
    sub t0, rax, t1, flags=(OF, SF, ZF, AF, PF, CF)

    subi rcx, rcx, 1, flags=(EZF,), dataSize=asz
    add rdi, rdi, t2, dataSize=asz
    br label("topOfLoop"), flags=(CSTRnZnEZF,)
end:
    fault "NoFault"
};

"""
