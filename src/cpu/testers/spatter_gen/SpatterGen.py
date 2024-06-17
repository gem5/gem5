# Copyright (c) 2024 The Regents of The University of California
# All rights reserved.
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

from m5.citations import add_citation
from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *
from m5.util.pybind import PyBindMethod


class SpatterKernelType(Enum):
    vals = ["scatter", "gather"]


class SpatterProcessingMode(Enum):
    vals = ["synchronous", "asynchronous"]


class SpatterGen(ClockedObject):
    type = "SpatterGen"
    cxx_header = "cpu/testers/spatter_gen/spatter_gen.hh"
    cxx_class = "gem5::SpatterGen"

    system = Param.System(Parent.any, "System this SpatterGen is a part of.")

    processing_mode = Param.SpatterProcessingMode(
        "How to process kernels accross multiple SpatterGen cores. "
        "Whether to synchronize on kernel boundaries or not."
    )

    port = RequestPort("Port to send memory requests.")

    int_regfile_size = Param.Int("Size of the integer register file.")
    fp_regfile_size = Param.Int("Size of the floating point register file.")
    request_gen_latency = Param.Cycles(
        "Number of cycles to spend for creating a request."
    )
    request_gen_rate = Param.Int("Number of requests generate per cycle.")
    request_buffer_entries = Param.Int("Size of the request buffer.")
    send_rate = Param.Int(
        "Number of requests to send in parallel."
        "Emulates the number of dcache ports."
    )

    cxx_exports = [
        PyBindMethod("addKernel"),
        PyBindMethod("proceedPastSyncPoint"),
    ]


add_citation(
    SpatterGen,
    """@inproceedings{10.1145/3422575.3422794,
author = {Lavin, Patrick and Young, Jeffrey and Vuduc, Richard and Riedy,
Jason and Vose, Aaron and Ernst, Daniel},
title = {Evaluating Gather and Scatter Performance on CPUs and GPUs},
year = {2021},
isbn = {9781450388993},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3422575.3422794},
doi = {10.1145/3422575.3422794},
abstract = {This paper describes a new benchmark tool,
Spatter, for assessing memory system architectures in the context of a
specific category of indexed accesses known as gather and scatter.
These types of operations are increasingly used to express sparse and
irregular data access patterns, and they have widespread utility in many
modern HPC applications including scientific simulations, data mining and
analysis computations, and graph processing. However, many traditional
benchmarking tools like STREAM, STRIDE, and GUPS focus on characterizing
only uniform stride or fully random accesses despite evidence that modern
applications use varied sets of more complex access patterns. Spatter is an
open-source benchmark that provides a tunable and configurable framework to
benchmark a variety of indexed access patterns, including variations of gather
/ scatter that are seen in HPC mini-apps evaluated in this work. The design of
Spatter includes backends for OpenMP and CUDA, and experiments show how it can
be used to evaluate 1) uniform access patterns for CPU and GPU, 2) prefetching
regimes for gather / scatter, 3) compiler implementations of vectorization for
gather / scatter, and 4) trace-driven “proxy patterns” that reflect the
patterns found in multiple applications. The results from Spatter experiments
show, for instance, that GPUs typically outperform CPUs for these operations
in absolute bandwidth but not fraction of peak bandwidth, and that Spatter can
better represent the performance of some cache-dependent mini-apps than
traditional STREAM bandwidth measurements.},
booktitle = {Proceedings of the International Symposium on Memory Systems},
pages = {209–222},
numpages = {14},
location = {Washington, DC, USA},
series = {MEMSYS '20}
}
""",
)
