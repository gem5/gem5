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
def macroop SYSCALL_64
{
    # All 1s.
    limm t1, "(uint64_t)(-1)", dataSize=8

    # Save the next RIP.
    rdip rcx

    # Stick rflags with RF masked into r11.
    rflags t2
    limm t3, "~RFBit", dataSize=8
    and r11, t2, t3, dataSize=8

    rdval t3, star
    srli t3, t3, 32, dataSize=8
    andi t3, t3, 0xFC, dataSize=1

    # Set up CS.
    wrsel cs, t3
    wrbase cs, t0, dataSize=8
    wrlimit cs, t1, dataSize=4
    # Not writable, read/execute-able, not expandDown,
    # dpl=0, defaultSize=0, long mode
    limm t4, ((0 << 0)  | (0  << 2)  | (0 << 3)   | \
              (1 << 4)  | (0  << 5)  | (1 << 6)   | \
              (1 << 7)  | (10 << 8)  | (0 << 12)  | \
              (1 << 13) | (0  << 14) | (1 << 15)), dataSize=8
    wrattr cs, t4

    # Set up SS.
    addi t3, t3, 8
    wrsel ss, t3
    wrbase ss, t0, dataSize=8
    wrlimit ss, t1, dataSize=4
    # Writable, readable, not expandDown,
    # dpl=0, defaultSize=0, not long mode
    limm t4, ((0 << 0)  | (0  << 2)  | (1 << 3)   | \
              (0 << 4)  | (0  << 5)  | (1 << 6)   | \
              (1 << 7)  | (2  << 8)  | (1 << 12)  | \
              (1 << 13) | (0  << 14) | (1 << 15)), dataSize=8
    wrattr ss, t4

    # Set the new rip.
    rdval t7, lstar, dataSize=8
    wrip t0, t7, dataSize=8

    # Mask the flags against sf_mask and leave RF turned off.
    rdval t3, sf_mask, dataSize=8
    xor t3, t3, t1, dataSize=8
    and t3, t3, r11, dataSize=8
    wrflags t3, t0
};

def macroop SYSCALL_COMPAT
{
    # All 1s.
    limm t1, "(uint64_t)(-1)", dataSize=8

    # Save the next RIP.
    rdip rcx

    # Stick rflags with RF masked into r11.
    rflags t2
    limm t3, "~RFBit", dataSize=8
    and r11, t2, t3, dataSize=8

    rdval t3, star
    srli t3, t3, 32, dataSize=8
    andi t3, t3, 0xFC, dataSize=1

    # Set up CS.
    wrsel cs, t3
    wrbase cs, t0, dataSize=8
    wrlimit cs, t1, dataSize=4
    # Not writable, read/execute-able, not expandDown,
    # dpl=0, defaultSize=0, long mode
    limm t4, ((0 << 0)  | (0  << 2)  | (0 << 3)   | \
              (1 << 4)  | (0  << 5)  | (1 << 6)   | \
              (1 << 7)  | (10 << 8)  | (0 << 12)  | \
              (1 << 13) | (0  << 14) | (1 << 15)), dataSize=8
    wrattr cs, t4

    # Set up SS.
    addi t3, t3, 8
    wrsel ss, t3
    wrbase ss, t0, dataSize=8
    wrlimit ss, t1, dataSize=4
    # Writable, readable, not expandDown,
    # dpl=0, defaultSize=0, not long mode
    limm t4, ((0 << 0)  | (0  << 2)  | (1 << 3)   | \
              (0 << 4)  | (0  << 5)  | (1 << 6)   | \
              (1 << 7)  | (2  << 8)  | (1 << 12)  | \
              (1 << 13) | (0  << 14) | (1 << 15)), dataSize=8
    wrattr ss, t4

    # Set the new rip.
    rdval t7, cstar
    wrip t0, t7

    # Mask the flags against sf_mask and leave RF turned off.
    rdval t3, sf_mask, dataSize=8
    xor t3, t3, t1, dataSize=8
    and t3, t3, r11, dataSize=8
    wrflags t3, t0
};

def macroop SYSCALL_LEGACY
{
    panic "The syscall instruction isn't implemented in legacy mode."
};

def macroop SYSRET_TO_64
{
    # All 1s.
    limm t1, "(uint64_t)(-1)", dataSize=8

    rdval t3, star
    srli t3, t3, 48, dataSize=8
    ori t3, t3, 3, dataSize=1

    # Set rflags to r11 with RF and VM cleared.
    limm t4, "~(RFBit | VMBit)", dataSize=8
    and t4, t4, r11, dataSize=8
    wrflags t4, t0

    # Set up CS.
    addi t4, t3, 16, dataSize=8
    wrsel cs, t4
    wrbase cs, t0, dataSize=8
    wrlimit cs, t1, dataSize=4
    # Not writable, read/execute-able, not expandDown,
    # dpl=3, defaultSize=0, long mode
    limm t4, ((3 << 0)  | (0  << 2)  | (0 << 3)   | \
              (1 << 4)  | (0  << 5)  | (1 << 6)   | \
              (1 << 7)  | (10 << 8)  | (0 << 12)  | \
              (1 << 13) | (0  << 14) | (1 << 15)), dataSize=8
    wrattr cs, t4

    # Only the selector is changed for SS.
    addi t4, t3, 8, dataSize=8
    wrsel ss, t4

    # Set the RIP back.
    wrip rcx, t0, dataSize=8
};

def macroop SYSRET_TO_COMPAT
{
    # All 1s.
    limm t1, "(uint64_t)(-1)", dataSize=8

    rdval t3, star
    srli t3, t3, 48, dataSize=8
    ori t3, t3, 3, dataSize=1

    # Set rflags to r11 with RF and VM cleared.
    limm t4, "~(RFBit | VMBit)", dataSize=8
    and t4, t4, r11, dataSize=8
    wrflags t4, t0

    # Set up CS.
    wrsel cs, t3
    wrbase cs, t0, dataSize=8
    wrlimit cs, t1, dataSize=4
    # Not writable, read/execute-able, not expandDown,
    # dpl=3, defaultSize=1, not long mode
    limm t4, ((3 << 0)  | (0  << 2)  | (0 << 3)   | \
              (1 << 4)  | (0  << 5)  | (1 << 6)   | \
              (1 << 7)  | (10 << 8)  | (0 << 12)  | \
              (1 << 13) | (0  << 14) | (1 << 15)), dataSize=8
    wrattr cs, t4

    # Only the selector is changed for SS.
    addi t4, t3, 8, dataSize=8
    wrsel ss, t4

    # Set the RIP back.
    wrip rcx, t0, dataSize=8
};

def macroop SYSRET_NON_64
{
    panic "The sysret instruction isn't implemented in legacy mode."
};
"""
# let {{
#    class SYSENTER(Inst):
#       "GenFault ${new UnimpInstFault}"
#    class SYSEXIT(Inst):
#       "GenFault ${new UnimpInstFault}"
# }};
