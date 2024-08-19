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

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "resources")

gem5_verify_config(
    name="gpu-apu-se-square",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "--download-resource",
        "square-gpu-test",
        "--download-dir",
        resource_path,
        "--reg-alloc-policy=dynamic",
        "-n3",
        "-c",
        joinpath(resource_path, "square-gpu-test"),
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="gpu-apu-se-sleepMutex",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "--download-resource",
        "allSyncPrims-1kernel",
        "--download-dir",
        resource_path,
        "--reg-alloc-policy=dynamic",
        "-n3",
        "-c",
        joinpath(resource_path, "allSyncPrims-1kernel"),
        "--options",
        "'sleepMutex 10 16 4'",
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.long_tag,
)

gem5_verify_config(
    name="gpu-apu-se-lftreebarruniq",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "--download-resource",
        "allSyncPrims-1kernel",
        "--download-dir",
        resource_path,
        "--reg-alloc-policy=dynamic",
        "-n3",
        "-c",
        joinpath(resource_path, "allSyncPrims-1kernel"),
        "--options",
        "'lfTreeBarrUniq 10 16 4 10 16 4'",
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.long_tag,
)

gem5_verify_config(
    name="gpu-apu-se-lulesh",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "--download-resource",
        "lulesh",
        "--download-dir",
        resource_path,
        "--reg-alloc-policy=dynamic",
        "-n3",
        "--mem-size=8GB",
        "--dgpu",
        "--gfx-version",
        "gfx900",
        "-c",
        joinpath(resource_path, "lulesh"),
        "--options",
        "'0.01 2'",
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.very_long_tag,
)

gem5_verify_config(
    name="gpu-apu-se-hacc",
    fixtures=(),
    verifiers=(),
    config=joinpath(config.base_dir, "configs", "example", "apu_se.py"),
    config_args=[
        "--download-resource",
        "hacc-force-tree",
        "--download-dir",
        resource_path,
        "--reg-alloc-policy=dynamic",
        "-n3",
        "-c",
        joinpath(resource_path, "hacc-force-tree"),
        "--options",
        "'0.5 0.1 64 0.1 1 N 12 rcb'",
    ],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=(constants.host_gcn_gpu_tag,),
    length=constants.very_long_tag,
)
