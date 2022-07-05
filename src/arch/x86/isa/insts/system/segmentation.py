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

microcode = """
def macroop LGDT_M
{
    .serialize_after
    .adjust_env maxOsz

    # Get the limit
    ld t1, seg, sib, disp, dataSize=2
    # Get the base
    ld t2, seg, sib, 'adjustedDisp + 2'
    wrbase tsg, t2
    wrlimit tsg, t1
};

def macroop LGDT_P
{
    .serialize_after
    .adjust_env maxOsz

    rdip t7
    # Get the limit
    ld t1, seg, riprel, disp, dataSize=2
    # Get the base
    ld t2, seg, riprel, 'adjustedDisp + 2'
    wrbase tsg, t2
    wrlimit tsg, t1
};

#
# These versions are for when the original data size was 16 bits. The base is
# still 32 bits, but the top byte is zeroed before being used.
#

def macroop LGDT_16_M
{
    .serialize_after
    .adjust_env maxOsz

    # Get the limit
    ld t1, seg, sib, disp, dataSize=2
    # Get the base
    ld t2, seg, sib, 'adjustedDisp + 2', dataSize=4
    zexti t2, t2, 23, dataSize=8
    wrbase tsg, t2, dataSize=8
    wrlimit tsg, t1
};

def macroop LGDT_16_P
{
    .serialize_after
    .adjust_env maxOsz

    rdip t7
    # Get the limit
    ld t1, seg, riprel, disp, dataSize=2
    # Get the base
    ld t2, seg, riprel, 'adjustedDisp + 2', dataSize=4
    zexti t2, t2, 23, dataSize=8
    wrbase tsg, t2
    wrlimit tsg, t1
};

def macroop LIDT_M
{
    .serialize_after
    .adjust_env maxOsz

    # Get the limit
    ld t1, seg, sib, disp, dataSize=2
    # Get the base
    ld t2, seg, sib, 'adjustedDisp + 2'
    wrbase idtr, t2
    wrlimit idtr, t1
};

def macroop LIDT_P
{
    .serialize_after
    .adjust_env maxOsz

    rdip t7
    # Get the limit
    ld t1, seg, riprel, disp, dataSize=2
    # Get the base
    ld t2, seg, riprel, 'adjustedDisp + 2'
    wrbase idtr, t2
    wrlimit idtr, t1
};

#
# These versions are for when the original data size was 16 bits. The base is
# still 32 bits, but the top byte is zeroed before being used.
#

def macroop LIDT_16_M
{
    .serialize_after
    .adjust_env maxOsz

    # Get the limit
    ld t1, seg, sib, disp, dataSize=2
    # Get the base
    ld t2, seg, sib, 'adjustedDisp + 2', dataSize=4
    zexti t2, t2, 23, dataSize=8
    wrbase idtr, t2, dataSize=8
    wrlimit idtr, t1
};

def macroop LIDT_16_P
{
    .serialize_after
    .adjust_env maxOsz

    rdip t7
    # Get the limit
    ld t1, seg, riprel, disp, dataSize=2
    # Get the base
    ld t2, seg, riprel, 'adjustedDisp + 2', dataSize=4
    zexti t2, t2, 23, dataSize=8
    wrbase idtr, t2
    wrlimit idtr, t1
};

def macroop LTR_64_R
{
    .serialize_after
    chks reg, t0, TRCheck
    limm t4, 0, dataSize=8
    srli t4, reg, 3, dataSize=2
    ldst t1, tsg, [8, t4, t0], dataSize=8
    ld t2, tsg, [8, t4, t0], 8, dataSize=8
    chks reg, t1, TSSCheck
    wrdh t3, t1, t2
    wrdl tr, t1, reg
    wrbase tr, t3, dataSize=8
    limm t5, (1 << 9)
    or t1, t1, t5
    st t1, tsg, [8, t4, t0], dataSize=8
};

def macroop LTR_64_M
{
    .serialize_after
    ld t5, seg, sib, disp, dataSize=2
    chks t5, t0, TRCheck
    limm t4, 0, dataSize=8
    srli t4, t5, 3, dataSize=2
    ldst t1, tsg, [8, t4, t0], dataSize=8
    ld t2, tsg, [8, t4, t0], 8, dataSize=8
    chks t5, t1, TSSCheck
    wrdh t3, t1, t2
    wrdl tr, t1, t5
    wrbase tr, t3, dataSize=8
    limm t5, (1 << 9)
    or t1, t1, t5
    st t1, tsg, [8, t4, t0], dataSize=8
};

def macroop LTR_64_P
{
    .serialize_after
    rdip t7
    ld t5, seg, riprel, disp, dataSize=2
    chks t5, t0, TRCheck
    limm t4, 0, dataSize=8
    srli t4, t5, 3, dataSize=2
    ldst t1, tsg, [8, t4, t0], dataSize=8
    ld t2, tsg, [8, t4, t0], 8, dataSize=8
    chks t5, t1, TSSCheck
    wrdh t3, t1, t2
    wrdl tr, t1, t5
    wrbase tr, t3, dataSize=8
    limm t5, (1 << 9)
    or t1, t1, t5
    st t1, tsg, [8, t4, t0], dataSize=8
};

def macroop LTR_R
{
    .serialize_after
    chks reg, t0, TRCheck
    limm t4, 0, dataSize=8
    srli t4, reg, 3, dataSize=2
    ldst t1, tsg, [8, t4, t0], dataSize=8
    chks reg, t1, TSSCheck
    wrdl tr, t1, reg
    limm t5, (1 << 9)
    or t1, t1, t5
    st t1, tsg, [8, t4, t0], dataSize=8
};

def macroop LTR_M
{
    .serialize_after
    ld t5, seg, sib, disp, dataSize=2
    chks t5, t0, TRCheck
    limm t4, 0, dataSize=8
    srli t4, t5, 3, dataSize=2
    ldst t1, tsg, [8, t4, t0], dataSize=8
    chks t5, t1, TSSCheck
    wrdl tr, t1, t5
    limm t5, (1 << 9)
    or t1, t1, t5
    st t1, tsg, [8, t4, t0], dataSize=8
};

def macroop LTR_P
{
    panic "LTR in non-64 bit mode doesn't support RIP addressing."
};

def macroop LLDT_64_R
{
    .serialize_after
    chks reg, t0, InGDTCheck, flags=(EZF,)
    br label("end"), flags=(CEZF,)
    limm t4, 0, dataSize=8
    srli t4, reg, 3, dataSize=2
    ld t1, tsg, [8, t4, t0], dataSize=8
    ld t2, tsg, [8, t4, t0], 8, dataSize=8
    chks reg, t1, LDTCheck
    wrdh t3, t1, t2
    wrdl tsl, t1, reg
    wrbase tsl, t3, dataSize=8
end:
    fault "NoFault"
};

def macroop LLDT_64_M
{
    .serialize_after
    ld t5, seg, sib, disp, dataSize=2
    chks t5, t0, InGDTCheck, flags=(EZF,)
    br label("end"), flags=(CEZF,)
    limm t4, 0, dataSize=8
    srli t4, t5, 3, dataSize=2
    ld t1, tsg, [8, t4, t0], dataSize=8
    ld t2, tsg, [8, t4, t0], 8, dataSize=8
    chks t5, t1, LDTCheck
    wrdh t3, t1, t2
    wrdl tsl, t1, t5
    wrbase tsl, t3, dataSize=8
end:
    fault "NoFault"
};

def macroop LLDT_64_P
{
    .serialize_after
    rdip t7
    ld t5, seg, riprel, disp, dataSize=2
    chks t5, t0, InGDTCheck, flags=(EZF,)
    br label("end"), flags=(CEZF,)
    limm t4, 0, dataSize=8
    srli t4, t5, 3, dataSize=2
    ld t1, tsg, [8, t4, t0], dataSize=8
    ld t2, tsg, [8, t4, t0], 8, dataSize=8
    chks t5, t1, LDTCheck
    wrdh t3, t1, t2
    wrdl tsl, t1, t5
    wrbase tsl, t3, dataSize=8
end:
    fault "NoFault"
};

def macroop LLDT_R
{
    .serialize_after
    chks reg, t0, InGDTCheck, flags=(EZF,)
    br label("end"), flags=(CEZF,)
    limm t4, 0, dataSize=8
    srli t4, reg, 3, dataSize=2
    ld t1, tsg, [8, t4, t0], dataSize=8
    chks reg, t1, LDTCheck
    wrdl tsl, t1, reg
end:
    fault "NoFault"
};

def macroop LLDT_M
{
    .serialize_after
    ld t5, seg, sib, disp, dataSize=2
    chks t5, t0, InGDTCheck, flags=(EZF,)
    br label("end"), flags=(CEZF,)
    limm t4, 0, dataSize=8
    srli t4, t5, 3, dataSize=2
    ld t1, tsg, [8, t4, t0], dataSize=8
    chks t5, t1, LDTCheck
    wrdl tsl, t1, t5
end:
    fault "NoFault"
};

def macroop LLDT_P
{
    panic "LLDT in non-64 bit mode doesn't support RIP addressing."
};

def macroop SWAPGS
{
    rdval t1, kernel_gs_base, dataSize=8
    rdbase t2, gs, dataSize=8
    wrbase gs, t1, dataSize=8
    wrval kernel_gs_base, t2, dataSize=8
};
"""
