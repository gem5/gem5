# Copyright (c) 2024 The Regents of the University of California
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


from testlib import *

resource_path = joinpath(absdirpath(__file__), "..", "gpu-pannotia-resources")

gem5_verify_config(
    name="gpu-apu-se-pannotia-bc-1k-128k",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "-n3",
        "--mem-size=8GB",
        "-c",
        joinpath(resource_path, "pannotia-bins/bc.gem5"),
        "--options",
        joinpath(resource_path, "pannotia-datasets/bc/1k_128k.gr"),
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.very_long_tag,
)

gem5_verify_config(
    name="gpu-apu-se-pannotia-bc-2k-1M",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "-n3",
        "--mem-size=8GB",
        "-c",
        joinpath(resource_path, "pannotia-bins/bc.gem5"),
        "--options",
        joinpath(resource_path, "pannotia-datasets/bc/2k_1M.gr"),
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.very_long_tag,
)

gem5_verify_config(
    name="gpu-apu-se-pannotia-color-maxmin-ecology",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "-n3",
        "--mem-size=8GB",
        "-c",
        joinpath(resource_path, "pannotia-bins/color_maxmin.gem5"),
        "--options",
        f'{joinpath(resource_path, "pannotia-datasets/color/ecology1.graph")} 0',
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.very_long_tag,
)

gem5_verify_config(
    name="gpu-apu-se-pannotia-color-maxmin-g3-circuit",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "-n3",
        "--mem-size=8GB",
        "-c",
        joinpath(resource_path, "pannotia-bins/color_maxmin.gem5"),
        "--options",
        f'{joinpath(resource_path, "pannotia-datasets/color/G3_circuit.graph")} 0',
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.very_long_tag,
)


gem5_verify_config(
    name="gpu-apu-se-pannotia-fw-hip-256",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "-n3",
        "--mem-size=8GB",
        "-c",
        joinpath(resource_path, "pannotia-bins/fw_hip.gem5"),
        "--options",
        f'-f {joinpath(resource_path, "pannotia-datasets/floydwarshall/256_16384.gr")} -m default',
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.very_long_tag,
)

gem5_verify_config(
    name="gpu-apu-se-pannotia-fw-hip-512",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "-n3",
        "--mem-size=8GB",
        "-c",
        joinpath(resource_path, "pannotia-bins/fw_hip.gem5"),
        "--options",
        f'-f {joinpath(resource_path, "pannotia-datasets/floydwarshall/512_65536.gr")} -m default',
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.very_long_tag,
)

# This test fails with
# ERROR: hipMalloc row_d (size:-202182160) => hipErrorOutOfMemory
# even when mem-size is set to 64GiB.
# gem5_verify_config(
#     name="gpu-apu-se-pannotia-mis-hip-ecology",
#     fixtures=(),
#     verifiers=(),
#     config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
#     config_args=[
#         "-n3",
#         "--mem-size=64GiB",
#         "-c",
#         joinpath(resource_path, "pannotia-bins/mis_hip.gem5"),
#         "--options",
#         f'{joinpath(resource_path, "pannotia-datasets/mis/ecology1.graph")} 0'
#     ],
#     valid_isas=(constants.vega_x86_tag,),
#     valid_hosts=(constants.host_gcn_gpu_tag,),
#     length=constants.very_long_tag,
# )

# This test also fails, likely because it also runs out of memory, but
# this hasn't been closely investigated.
# gem5_verify_config(
#     name="gpu-apu-se-pannotia-mis-hip-g3-circuit",
#     fixtures=(),
#     verifiers=(),
#     config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
#     config_args=[
#         "-n3",
#         "--mem-size=32GiB",
#         "-c",
#         joinpath(resource_path, "pannotia-bins/mis_hip.gem5"),
#         "--options",
#         f'{joinpath(resource_path, "pannotia-datasets/mis/G3_circuit.graph")} 0'
#     ],
#     valid_isas=(constants.vega_x86_tag,),
#     valid_hosts=(constants.host_gcn_gpu_tag,),
#     length=constants.very_long_tag,
# )

gem5_verify_config(
    name="gpu-apu-se-pannotia-pagerank-spmv",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "-n3",
        "--mem-size=8GB",
        "-c",
        joinpath(resource_path, "pannotia-bins/pagerank_spmv.gem5"),
        "--options",
        f'{joinpath(resource_path, "pannotia-datasets/pagerank/coAuthorsDBLP.graph")} 0',
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.very_long_tag,
)

gem5_verify_config(
    name="gpu-apu-se-pannotia-sssp-ell",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "-n3",
        "--mem-size=8GB",
        "-c",
        joinpath(resource_path, "pannotia-bins/sssp_ell.gem5"),
        "--options",
        f'{joinpath(resource_path, "pannotia-datasets/sssp/USA-road-d.NY.gr")} 0',
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.very_long_tag,
)
