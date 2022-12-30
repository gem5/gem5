# Copyright (c) 2021 Advanced Micro Devices, Inc.
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


def addAmdGPUOptions(parser):
    parser.add_argument(
        "-u",
        "--num-compute-units",
        type=int,
        default=4,
        help="number of GPU compute units",
    ),
    parser.add_argument(
        "--num-cp",
        type=int,
        default=0,
        help="Number of GPU Command Processors (CP)",
    )

    # not super important now, but to avoid putting the number 4 everywhere,
    # make it an option/knob
    parser.add_argument(
        "--cu-per-sqc",
        type=int,
        default=4,
        help="number of CUs sharing an SQC" " (icache, and thus icache TLB)",
    )
    parser.add_argument(
        "--cu-per-scalar-cache",
        type=int,
        default=4,
        help="Number of CUs sharing a scalar cache",
    )
    parser.add_argument(
        "--simds-per-cu", type=int, default=4, help="SIMD units per CU"
    )
    parser.add_argument(
        "--cu-per-sa",
        type=int,
        default=4,
        help="Number of CUs per shader array. This must be a"
        " multiple of options.cu-per-sqc and "
        " options.cu-per-scalar",
    )
    parser.add_argument(
        "--sa-per-complex",
        type=int,
        default=1,
        help="Number of shader arrays per complex",
    )
    parser.add_argument(
        "--num-gpu-complexes",
        type=int,
        default=1,
        help="Number of GPU complexes",
    )
    parser.add_argument(
        "--wf-size", type=int, default=64, help="Wavefront size(in workitems)"
    )
    parser.add_argument(
        "--sp-bypass-path-length",
        type=int,
        default=4,
        help="Number of stages of bypass path in vector ALU "
        "for Single Precision ops",
    )
    parser.add_argument(
        "--dp-bypass-path-length",
        type=int,
        default=4,
        help="Number of stages of bypass path in vector ALU "
        "for Double Precision ops",
    )
    # issue period per SIMD unit: number of cycles before issuing another vector
    parser.add_argument(
        "--issue-period",
        type=int,
        default=4,
        help="Number of cycles per vector instruction issue" " period",
    )
    parser.add_argument(
        "--glbmem-wr-bus-width",
        type=int,
        default=32,
        help="VGPR to Coalescer (Global Memory) data bus width" " in bytes",
    )
    parser.add_argument(
        "--glbmem-rd-bus-width",
        type=int,
        default=32,
        help="Coalescer to VGPR (Global Memory) data bus width" " in bytes",
    )
    # Currently we only support 1 local memory pipe
    parser.add_argument(
        "--shr-mem-pipes-per-cu",
        type=int,
        default=1,
        help="Number of Shared Memory pipelines per CU",
    )
    # Currently we only support 1 global memory pipe
    parser.add_argument(
        "--glb-mem-pipes-per-cu",
        type=int,
        default=1,
        help="Number of Global Memory pipelines per CU",
    )
    parser.add_argument(
        "--wfs-per-simd",
        type=int,
        default=10,
        help="Number of WF slots per SIMD",
    )

    parser.add_argument(
        "--registerManagerPolicy",
        type=str,
        default="static",
        help="Register manager policy",
    )
    parser.add_argument(
        "--vreg-file-size",
        type=int,
        default=2048,
        help="number of physical vector registers per SIMD",
    )
    parser.add_argument(
        "--vreg-min-alloc",
        type=int,
        default=4,
        help="vector register reservation unit",
    )

    parser.add_argument(
        "--sreg-file-size",
        type=int,
        default=2048,
        help="number of physical scalar registers per SIMD",
    )
    parser.add_argument(
        "--sreg-min-alloc",
        type=int,
        default=4,
        help="scalar register reservation unit",
    )

    parser.add_argument(
        "--bw-scalor",
        type=int,
        default=0,
        help="bandwidth scalor for scalability analysis",
    )
    parser.add_argument(
        "--CPUClock", type=str, default="2GHz", help="CPU clock"
    )
    parser.add_argument(
        "--gpu-clock", type=str, default="1GHz", help="GPU clock"
    )
    parser.add_argument(
        "--cpu-voltage",
        action="store",
        type=str,
        default="1.0V",
        help="CPU voltage domain",
    )
    parser.add_argument(
        "--gpu-voltage",
        action="store",
        type=str,
        default="1.0V",
        help="GPU voltage domain",
    )
    parser.add_argument(
        "--CUExecPolicy",
        type=str,
        default="OLDEST-FIRST",
        help="WF exec policy (OLDEST-FIRST, ROUND-ROBIN)",
    )
    parser.add_argument(
        "--LocalMemBarrier",
        action="store_true",
        help="Barrier does not wait for writethroughs to " " complete",
    )
    parser.add_argument(
        "--countPages",
        action="store_true",
        help="Count Page Accesses and output in " " per-CU output files",
    )
    parser.add_argument(
        "--TLB-prefetch", type=int, help="prefetch depth for" "TLBs"
    )
    parser.add_argument(
        "--pf-type",
        type=str,
        help="type of prefetch: " "PF_CU, PF_WF, PF_PHASE, PF_STRIDE",
    )
    parser.add_argument("--pf-stride", type=int, help="set prefetch stride")
    parser.add_argument(
        "--numLdsBanks",
        type=int,
        default=32,
        help="number of physical banks per LDS module",
    )
    parser.add_argument(
        "--ldsBankConflictPenalty",
        type=int,
        default=1,
        help="number of cycles per LDS bank conflict",
    )
    parser.add_argument(
        "--lds-size", type=int, default=65536, help="Size of the LDS in bytes"
    )
    parser.add_argument(
        "--num-hw-queues",
        type=int,
        default=10,
        help="number of hw queues in packet processor",
    )
    parser.add_argument(
        "--reg-alloc-policy",
        type=str,
        default="simple",
        help="register allocation policy (simple/dynamic)",
    )
