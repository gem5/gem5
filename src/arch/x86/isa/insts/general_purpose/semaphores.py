# Copyright (c) 2007 The Hewlett-Packard Development Company
# Copyright (c) 2015 Advanced Micro Devices, Inc.
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
def macroop CMPXCHG_R_R {
    sub t0, rax, reg, flags=(OF, SF, ZF, AF, PF, CF)
    mov reg, reg, regm, flags=(CZF,)
    mov rax, rax, reg, flags=(nCZF,)
};

def macroop CMPXCHG_M_R {
    ldst t1, seg, sib, disp
    sub t0, rax, t1, flags=(OF, SF, ZF, AF, PF, CF)

    mov t1, t1, reg, flags=(CZF,)
    st t1, seg, sib, disp
    mov rax, rax, t1, flags=(nCZF,)
};

def macroop CMPXCHG_P_R {
    rdip t7
    ldst t1, seg, riprel, disp
    sub t0, rax, t1, flags=(OF, SF, ZF, AF, PF, CF)

    mov t1, t1, reg, flags=(CZF,)
    st t1, seg, riprel, disp
    mov rax, rax, t1, flags=(nCZF,)
};

def macroop CMPXCHG_LOCKED_M_R {
    mfence
    ldstl t1, seg, sib, disp
    sub t0, rax, t1, flags=(OF, SF, ZF, AF, PF, CF)

    mov t1, t1, reg, flags=(CZF,)
    stul t1, seg, sib, disp
    mfence
    mov rax, rax, t1, flags=(nCZF,)
};

def macroop CMPXCHG_LOCKED_P_R {
    rdip t7
    mfence
    ldstl t1, seg, riprel, disp
    sub t0, rax, t1, flags=(OF, SF, ZF, AF, PF, CF)

    mov t1, t1, reg, flags=(CZF,)
    stul t1, seg, riprel, disp
    mfence
    mov rax, rax, t1, flags=(nCZF,)
};

def macroop XADD_M_R {
    ldst t1, seg, sib, disp
    add t2, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    st t2, seg, sib, disp
    mov reg, reg, t1
};

def macroop XADD_P_R {
    rdip t7
    ldst t1, seg, riprel, disp
    add t2, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    st t2, seg, riprel, disp
    mov reg, reg, t1
};

def macroop XADD_LOCKED_M_R {
    mfence
    ldstl t1, seg, sib, disp
    add t2, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    stul t2, seg, sib, disp
    mfence
    mov reg, reg, t1
};

def macroop XADD_LOCKED_P_R {
    rdip t7
    mfence
    ldstl t1, seg, riprel, disp
    add t2, t1, reg, flags=(OF,SF,ZF,AF,PF,CF)
    stul t2, seg, riprel, disp
    mfence
    mov reg, reg, t1
};

def macroop XADD_R_R {
    add t2, regm, reg, flags=(OF,SF,ZF,AF,PF,CF)
    mov regm, regm, reg
    mov reg, reg, t2
};

"""

# Despite the name, this microcode sequence implements both
# cmpxchg8b and cmpxchg16b, depending on the dynamic value
# of dataSize.
cmpxchg8bCode = """
def macroop CMPXCHG8B_%(suffix)s {
    .adjust_env clampOsz
    %(rdip)s
    %(mfence)s
    lea t1, seg, %(sib)s, disp, dataSize=asz
    ldsplit%(l)s (t2, t3), seg, [1, t0, t1], disp=0

    sub t0, rax, t2, flags=(ZF,)
    br label("doneComparing"), flags=(nCZF,)
    sub t0, rdx, t3, flags=(ZF,)
doneComparing:

    # If they're equal, set t3:t2 to rbx:rcx to write to memory
    mov t2, t2, rbx, flags=(CZF,)
    mov t3, t3, rcx, flags=(CZF,)

    # If they're not equal, set rdx:rax to the value from memory.
    mov rax, rax, t2, flags=(nCZF,)
    mov rdx, rdx, t3, flags=(nCZF,)

    # Write to memory
    stsplit%(ul)s (t2, t3), seg, [1, t0, t1], disp=0
    %(mfence)s
};
"""

microcode += cmpxchg8bCode % {
    "rdip": "",
    "sib": "sib",
    "l": "",
    "ul": "",
    "mfence": "",
    "suffix": "M",
}
microcode += cmpxchg8bCode % {
    "rdip": "rdip t7",
    "sib": "riprel",
    "l": "",
    "ul": "",
    "mfence": "",
    "suffix": "P",
}
microcode += cmpxchg8bCode % {
    "rdip": "",
    "sib": "sib",
    "l": "l",
    "ul": "ul",
    "mfence": "mfence",
    "suffix": "LOCKED_M",
}
microcode += cmpxchg8bCode % {
    "rdip": "rdip t7",
    "sib": "riprel",
    "l": "l",
    "ul": "ul",
    "mfence": "mfence",
    "suffix": "LOCKED_P",
}

# let {{
#    class XCHG(Inst):
#       "GenFault ${new UnimpInstFault}"
# }};
