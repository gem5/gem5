# Copyright (c) 2017-2021 Advanced Micro Devices, Inc.
# All rights reserved.
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
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
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
from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class ProtocolTester(ClockedObject):
    type = "ProtocolTester"
    cxx_header = "cpu/testers/gpu_ruby_test/protocol_tester.hh"
    cxx_class = "gem5::ProtocolTester"

    cpu_ports = VectorRequestPort("Ports for CPUs")
    dma_ports = VectorRequestPort("Ports for DMAs")
    cu_vector_ports = VectorRequestPort("Vector ports for GPUs")
    cu_sqc_ports = VectorRequestPort("SQC ports for GPUs")
    cu_scalar_ports = VectorRequestPort("Scalar ports for GPUs")
    cu_token_ports = VectorRequestPort("Token ports for GPU")

    cus_per_sqc = Param.Int(4, "Number of CUs per SQC")
    cus_per_scalar = Param.Int(4, "Number of CUs per scalar cache")

    wavefronts_per_cu = Param.Int(1, "Number of wavefronts per CU")
    workitems_per_wavefront = Param.Int(64, "Number of workitems per wf")

    max_cu_tokens = Param.Int(
        4,
        "Maximum number of tokens, i.e., the number"
        " of instructions that can be uncoalesced"
        " before back-pressure occurs from the"
        " coalescer.",
    )

    cpu_threads = VectorParam.CpuThread("All cpus")
    dma_threads = VectorParam.DmaThread("All DMAs")
    wavefronts = VectorParam.GpuWavefront("All wavefronts")

    num_atomic_locations = Param.Int(2, "Number of atomic locations")
    num_normal_locs_per_atomic = Param.Int(
        1000,
        "Number of normal locations per atomic",
    )

    episode_length = Param.Int(10, "Number of actions per episode")
    max_num_episodes = Param.Int(20, "Maximum number of episodes")
    debug_tester = Param.Bool(False, "Are we debugging the tester?")
    random_seed = Param.Int(
        0,
        "Random seed number. Default value (0) means \
                                using base/random.hh without seed.",
    )
    log_file = Param.String("Log file's name")
    system = Param.System(Parent.any, "System we belong to")
