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
# Real mode versions of the load far pointer instructions.
#

def macroop LDS_REAL_R_M {
    # Calculate the address of the pointer.
    lea t1, seg, sib, disp, dataSize=asz

    # Load the selector into a temporary.
    ld t2, seg, [1, t0, t1], dsz, dataSize=2
    # Load the offset.
    ld reg, seg, [1, t0, t1], 0
    # Install the selector value.
    wrsel ds, t2
    # Make sure there isn't any junk in the upper bits of the base.
    mov t2, t0, t2
    # Compute and set the base.
    slli t2, t2, 4, dataSize=8
    wrbase ds, t2, dataSize=8
};

def macroop LES_REAL_R_M {
    # Calculate the address of the pointer.
    lea t1, seg, sib, disp, dataSize=asz

    # Load the selector into a temporary.
    ld t2, seg, [1, t0, t1], dsz, dataSize=2
    # Load the offset.
    ld reg, seg, [1, t0, t1], 0
    # Install the selector value.
    wrsel es, t2
    # Make sure there isn't any junk in the upper bits of the base.
    mov t2, t0, t2
    # Compute and set the base.
    slli t2, t2, 4, dataSize=8
    wrbase es, t2, dataSize=8
};

def macroop LFS_REAL_R_M {
    # Calculate the address of the pointer.
    lea t1, seg, sib, disp, dataSize=asz

    # Load the selector into a temporary.
    ld t2, seg, [1, t0, t1], dsz, dataSize=2
    # Load the offset.
    ld reg, seg, [1, t0, t1], 0
    # Install the selector value.
    wrsel fs, t2
    # Make sure there isn't any junk in the upper bits of the base.
    mov t2, t0, t2
    # Compute and set the base.
    slli t2, t2, 4, dataSize=8
    wrbase fs, t2, dataSize=8
};

def macroop LGS_REAL_R_M {
    # Calculate the address of the pointer.
    lea t1, seg, sib, disp, dataSize=asz

    # Load the selector into a temporary.
    ld t2, seg, [1, t0, t1], dsz, dataSize=2
    # Load the offset.
    ld reg, seg, [1, t0, t1], 0
    # Install the selector value.
    wrsel gs, t2
    # Make sure there isn't any junk in the upper bits of the base.
    mov t2, t0, t2
    # Compute and set the base.
    slli t2, t2, 4, dataSize=8
    wrbase gs, t2, dataSize=8
};

def macroop LSS_REAL_R_M {
    # Calculate the address of the pointer.
    lea t1, seg, sib, disp, dataSize=asz

    # Load the selector into a temporary.
    ld t2, seg, [1, t0, t1], dsz, dataSize=2
    # Load the offset.
    ld reg, seg, [1, t0, t1], 0
    # Install the selector value.
    wrsel ss, t2
    # Make sure there isn't any junk in the upper bits of the base.
    mov t2, t0, t2
    # Compute and set the base.
    slli t2, t2, 4, dataSize=8
    wrbase ss, t2, dataSize=8
};

# Versions for RIP relative addressing, even though these instructions are
# illegal in 64 bit mode, where RIP relative addressing is available.

def macroop LDS_REAL_R_P {
    panic "Real mode LDS doesn't support RIP relative addressing."
};

def macroop LES_REAL_R_P {
    panic "Real mode LES doesn't support RIP relative addressing."
};

def macroop LFS_REAL_R_P {
    panic "Real mode LFS doesn't support RIP relative addressing."
};

def macroop LGS_REAL_R_P {
    panic "Real mode LGS doesn't support RIP relative addressing."
};

def macroop LSS_REAL_R_P {
    panic "Real mode LSS doesn't support RIP relative addressing."
};

"""
# let {{
#    class LDS(Inst):
#       "GenFault ${new UnimpInstFault}"
#    class LES(Inst):
#       "GenFault ${new UnimpInstFault}"
#    class LFS(Inst):
#       "GenFault ${new UnimpInstFault}"
#    class LGS(Inst):
#       "GenFault ${new UnimpInstFault}"
#    class LSS(Inst):
#       "GenFault ${new UnimpInstFault}"
#    class MOV_SEG(Inst):
#       "GenFault ${new UnimpInstFault}"
#    class POP(Inst):
#       "GenFault ${new UnimpInstFault}"
# }};
