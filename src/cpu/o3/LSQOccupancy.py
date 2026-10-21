# Copyright (c) 2026 Barcelona Supercomputing Center
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

from m5.objects.FuncUnit import OpClass
from m5.params import *
from m5.SimObject import SimObject


# LSQ occupancy of the instructions of some op classes. Whether an
# instruction uses the LQ or the SQ depends on the instruction, not on its op
# class: atomics of a load op class, such as RISC-V AMOs, use the SQ. So a
# reservation larger than both queues is rejected at construction, but one
# larger than only one of them is reported when the first instruction that
# uses that queue is renamed.
class LSQOpDesc(SimObject):
    type = "LSQOpDesc"
    cxx_class = "gem5::o3::LSQOpDesc"
    cxx_header = "cpu/o3/lsq_occupancy.hh"

    opClasses = VectorParam.OpClass("Op classes this description applies to")
    entries = Param.Unsigned(
        1,
        "LSQ entries occupied by each instruction of these op classes; at "
        "most the entries per thread of the queue it uses",
    )
