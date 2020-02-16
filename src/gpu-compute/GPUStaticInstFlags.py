# Copyright (c) 2016 Advanced Micro Devices, Inc.
# All rights reserved.
#
# For use for simulation and test purposes only
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software
# without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from m5.params import *

class GPUStaticInstFlags(Enum):
    wrapper_name = 'GPUStaticInstFlags'
    wrapper_is_struct = True
    enum_name = 'Flags'

    vals = [
        # Op types
        'ALU',               # ALU op
        'Branch',            # Branch instruction
        'Nop',               # No-op (no effect at all)
        'Return',            # Return instruction
        'UnconditionalJump', #
        'SpecialOp',         # Special op
        'Waitcnt',           # Is a waitcnt instruction

        # Memory ops
        'MemBarrier',        # Barrier instruction
        'MemFence',          # Memory fence instruction
        'MemoryRef',         # References memory (load, store, or atomic)
        'Flat',              # Flat memory op
        'Load',              # Reads from memory
        'Store',             # Writes to memory

        # Atomic ops
        'AtomicReturn',      # Atomic instruction that returns data
        'AtomicNoReturn',    # Atomic instruction that doesn't return data

        # Instruction attributes
        'Scalar',            # A scalar (not vector) operation
        'ReadsSCC',          # The instruction reads SCC
        'WritesSCC',         # The instruction writes SCC
        'ReadsVCC',          # The instruction reads VCC
        'WritesVCC',         # The instruction writes VCC

        # Atomic OP types
        'AtomicAnd',
        'AtomicOr',
        'AtomicXor',
        'AtomicCAS',
        'AtomicExch',
        'AtomicAdd',
        'AtomicSub',
        'AtomicInc',
        'AtomicDec',
        'AtomicMax',
        'AtomicMin',

        # Memory order flags
        'RelaxedOrder',
        'Acquire',           # Has acquire semantics
        'Release',           # Has release semantics
        'AcquireRelease',    # Has acquire and release semantics
        'NoOrder',           # Has no ordering restrictions

        # Segment access flags
        'ArgSegment',        # Accesses the arg segment
        'GlobalSegment',     # Accesses global memory
        'GroupSegment',      # Accesses local memory (LDS), aka shared memory
        'KernArgSegment',    # Accesses the kernel argument segment
        'PrivateSegment',    # Accesses the private segment
        'ReadOnlySegment',   # Accesses read only memory
        'SpillSegment',      # Accesses the spill segment
        'NoSegment',         # Does not have an associated segment

        # Scope flags
        'WorkitemScope',
        'WavefrontScope',
        'WorkgroupScope',
        'DeviceScope',
        'SystemScope',
        'NoScope',           # Does not have an associated scope

        # Coherence flags
        'GloballyCoherent',  # Coherent with other workitems on same device
        'SystemCoherent'     # Coherent with a different device, or the host
        ]
