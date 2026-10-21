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


# How the LSQ handles an access whose policy asks for more entries than
# its instruction reserved at dispatch:
#   Bound: the reserved entries are an upper bound; the access is clamped.
#   Overcommit: the access occupies the entries it asks for, which may
#       temporarily exceed the size of the queue.
class LSQOverflowMode(ScopedEnum):
    vals = ["Bound", "Overcommit"]


class LSQOccupancyPolicy(SimObject):
    type = "LSQOccupancyPolicy"
    abstract = True
    cxx_class = "gem5::o3::LSQOccupancyPolicy"
    cxx_header = "cpu/o3/lsq_occupancy.hh"


class ConstantOccupancy(LSQOccupancyPolicy):
    type = "ConstantOccupancy"
    cxx_class = "gem5::o3::ConstantOccupancy"
    cxx_header = "cpu/o3/lsq_occupancy.hh"

    entries = Param.Unsigned(1, "LSQ entries occupied by each access")


class PerSegmentOccupancy(LSQOccupancyPolicy):
    type = "PerSegmentOccupancy"
    cxx_class = "gem5::o3::PerSegmentOccupancy"
    cxx_header = "cpu/o3/lsq_occupancy.hh"


class PerBlockOccupancy(LSQOccupancyPolicy):
    type = "PerBlockOccupancy"
    cxx_class = "gem5::o3::PerBlockOccupancy"
    cxx_header = "cpu/o3/lsq_occupancy.hh"

    blockSize = Param.MemorySize("16B", "Size of the aligned blocks")
    coalesce = Param.Bool(
        True, "Count a block touched by several segments only once"
    )


class PerCacheLineOccupancy(LSQOccupancyPolicy):
    type = "PerCacheLineOccupancy"
    cxx_class = "gem5::o3::PerCacheLineOccupancy"
    cxx_header = "cpu/o3/lsq_occupancy.hh"

    coalesce = Param.Bool(
        True, "Count a cache line touched by several segments only once"
    )


class LSQOpDesc(SimObject):
    type = "LSQOpDesc"
    cxx_class = "gem5::o3::LSQOpDesc"
    cxx_header = "cpu/o3/lsq_occupancy.hh"

    opClasses = VectorParam.OpClass("Op classes this description applies to")
    entries = Param.Unsigned(
        1,
        "LSQ entries reserved at dispatch by each instruction of these op "
        "classes",
    )
    policy = Param.LSQOccupancyPolicy(
        NULL,
        "Policy computing the entries occupied once the access is known; "
        "without one, instructions keep the entries reserved at dispatch",
    )
