# Copyright (c) 2018 Advanced Micro Devices, Inc.
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

import operator
from os import (
    fsync,
    getpid,
    listdir,
    makedirs,
    mkdir,
)
from os.path import isdir
from os.path import join as joinpath
from shutil import (
    copyfile,
    rmtree,
)

import m5
from m5.util.convert import (
    toFrequency,
    toMemorySize,
)


def file_append(path, contents):
    with open(joinpath(*path), "a") as f:
        f.write(str(contents))
        f.flush()
        fsync(f.fileno())


def remake_dir(path):
    if isdir(path):
        rmtree(path)
    makedirs(path)


# This fakes out a dGPU setup so the runtime operates correctly.  The spoofed
# system has a single dGPU and a single socket CPU.  Note that more complex
# topologies (multi-GPU, multi-socket CPUs) need to have a different setup
# here or the runtime won't be able to issue Memcpies from one node to another.
#
# TODO: There is way too much hardcoded here.  It doesn't effect anything in
# our current ROCm stack (1.6), but it is highly possible that it will in the
# future.  We might need to scrub through this and extract the appropriate
# fields from the simulator in the future.
def createVegaTopology(options):
    topology_dir = joinpath(
        m5.options.outdir, "fs/sys/devices/virtual/kfd/kfd/topology"
    )
    remake_dir(topology_dir)

    amdgpu_dir = joinpath(m5.options.outdir, "fs/sys/module/amdgpu/parameters")
    remake_dir(amdgpu_dir)

    pci_ids_dir = joinpath(m5.options.outdir, "fs/usr/share/hwdata/")
    remake_dir(pci_ids_dir)

    # Vega reported VM size in GB.  Used to reserve an allocation from CPU
    # to implement SVM (i.e. GPUVM64 pointers and X86 pointers agree)
    file_append((amdgpu_dir, "vm_size"), 256)

    # Ripped from real Vega platform to appease KMT version checks
    file_append((topology_dir, "generation_id"), 2)

    # Set up system properties.  Regiter as ast-rocm server
    sys_prop = (
        "platform_oem 35498446626881\n"
        + "platform_id 71791775140929\n"
        + "platform_rev 2\n"
    )
    file_append((topology_dir, "system_properties"), sys_prop)

    # Populate the topology tree
    # Our dGPU system is two nodes.  Node 0 is a CPU and Node 1 is a dGPU
    node_dir = joinpath(topology_dir, "nodes/0")
    remake_dir(node_dir)

    # Register as a CPU
    file_append((node_dir, "gpu_id"), 0)
    file_append((node_dir, "name"), "")

    # CPU links.  Only thing that matters is we tell the runtime that GPU is
    # connected through PCIe to CPU socket 0.
    io_links = 1
    io_dir = joinpath(node_dir, "io_links/0")
    remake_dir(io_dir)
    io_prop = (
        "type 2\n"
        + "version_major 0\n"
        + "version_minor 0\n"
        + "node_from 0\n"
        + "node_to 1\n"
        + "weight 20\n"
        + "min_latency 0\n"
        + "max_latency 0\n"
        + "min_bandwidth 0\n"
        + "max_bandwidth 0\n"
        + "recommended_transfer_size 0\n"
        + "flags 13\n"
    )
    file_append((io_dir, "properties"), io_prop)

    # Populate CPU node properties
    node_prop = (
        f"cpu_cores_count {options.num_cpus}\n"
        + "simd_count 0\n"
        + "mem_banks_count 1\n"
        + "caches_count 0\n"
        + f"io_links_count {io_links}\n"
        + "cpu_core_id_base 0\n"
        + "simd_id_base 0\n"
        + "max_waves_per_simd 0\n"
        + "lds_size_in_kb 0\n"
        + "gds_size_in_kb 0\n"
        + "wave_front_size 64\n"
        + "array_count 0\n"
        + "simd_arrays_per_engine 0\n"
        + "cu_per_simd_array 0\n"
        + "simd_per_cu 0\n"
        + "max_slots_scratch_cu 0\n"
        + "vendor_id 0\n"
        + "device_id 0\n"
        + "location_id 0\n"
        + "drm_render_minor 0\n"
        + "max_engine_clk_ccompute 3400\n"
    )

    file_append((node_dir, "properties"), node_prop)

    # CPU memory reporting
    mem_dir = joinpath(node_dir, "mem_banks/0")
    remake_dir(mem_dir)
    # Heap type value taken from real system, heap type values:
    # https://github.com/RadeonOpenCompute/ROCT-Thunk-Interface/blob/roc-4.0.x/include/hsakmttypes.h#L317
    mem_prop = (
        "heap_type 0\n"
        + "size_in_bytes 33704329216\n"
        + "flags 0\n"
        + "width 72\n"
        + "mem_clk_max 2400\n"
    )

    file_append((mem_dir, "properties"), mem_prop)

    # Build the GPU node
    node_dir = joinpath(topology_dir, "nodes/1")
    remake_dir(node_dir)

    # Register as a Vega
    file_append((node_dir, "gpu_id"), 22124)
    file_append((node_dir, "name"), "Vega\n")

    # Should be the same as the render driver filename (dri/renderD<drm_num>)
    drm_num = 128

    # 96 in real Vega
    # Random comment for comparison purposes
    caches = 0

    # GPU links.  Only thing that matters is we tell the runtime that GPU is
    # connected through PCIe to CPU socket 0.
    io_links = 1
    io_dir = joinpath(node_dir, "io_links/0")
    remake_dir(io_dir)
    io_prop = (
        "type 2\n"
        + "version_major 0\n"
        + "version_minor 0\n"
        + "node_from 1\n"
        + "node_to 0\n"
        + "weight 20\n"
        + "min_latency 0\n"
        + "max_latency 0\n"
        + "min_bandwidth 0\n"
        + "max_bandwidth 0\n"
        + "recommended_transfer_size 0\n"
        + "flags 1\n"
    )
    file_append((io_dir, "properties"), io_prop)

    # Populate GPU node properties
    cu_scratch = options.simds_per_cu * options.wfs_per_simd
    node_prop = (
        "cpu_cores_count 0\n"
        + "simd_count 256\n"
        + "mem_banks_count 1\n"
        + f"caches_count {caches}\n"
        + f"io_links_count {io_links}\n"
        + "cpu_core_id_base 0\n"
        + "simd_id_base 2147487744\n"
        + "max_waves_per_simd 10\n"
        + "lds_size_in_kb 64\n"
        + "gds_size_in_kb 0\n"
        + "wave_front_size 64\n"
        + "array_count 4\n"
        + "simd_arrays_per_engine 1\n"
        + "cu_per_simd_array 16\n"
        + "simd_per_cu 4\n"
        + f"max_slots_scratch_cu {cu_scratch}\n"
        + "vendor_id 4098\n"
        + "device_id 26720\n"
        + "location_id 1024\n"
        + f"drm_render_minor {drm_num}\n"
        + "hive_id 0\n"
        + "num_sdma_engines 2\n"
        + "num_sdma_xgmi_engines 0\n"
        + "max_engine_clk_fcompute 1500\n"
        + "local_mem_size 17163091968\n"
        + "fw_version 421\n"
        + "capability 238208\n"
        + "debug_prop 32768\n"
        + "sdma_fw_version 430\n"
        + "max_engine_clk_ccompute 3400\n"
    )

    file_append((node_dir, "properties"), node_prop)

    # Vega HBM reporting
    # TODO: Extract size, clk, and width from sim paramters
    mem_dir = joinpath(node_dir, "mem_banks/0")
    remake_dir(mem_dir)
    # Heap type value taken from real system, heap type values:
    # https://github.com/RadeonOpenCompute/ROCT-Thunk-Interface/blob/roc-4.0.x/include/hsakmttypes.h#L317
    mem_prop = (
        "heap_type 1\n"
        + "size_in_bytes 17163091968\n"
        + "flags 0\n"
        + "width 2048\n"
        + "mem_clk_max 945\n"
    )

    file_append((mem_dir, "properties"), mem_prop)


def createRavenTopology(options):
    topology_dir = joinpath(
        m5.options.outdir, "fs/sys/devices/virtual/kfd/kfd/topology"
    )
    remake_dir(topology_dir)

    # Ripped from real Kaveri platform to appease kmt version checks
    # Set up generation_id
    file_append((topology_dir, "generation_id"), 1)

    # Set up system properties
    sys_prop = (
        "platform_oem 2314885673410447169\n"
        + "platform_id 35322352389441\n"
        + "platform_rev 1\n"
    )
    file_append((topology_dir, "system_properties"), sys_prop)

    # Populate the topology tree
    # TODO: Just the bare minimum to pass for now
    node_dir = joinpath(topology_dir, "nodes/0")
    remake_dir(node_dir)

    # must show valid kaveri gpu id or massive meltdown
    file_append((node_dir, "gpu_id"), 2765)

    gfx_dict = {
        "gfx902": {"name": "Raven\n", "id": 5597},
    }

    # must have marketing name
    file_append((node_dir, "name"), gfx_dict[options.gfx_version]["name"])

    mem_banks_cnt = 1

    # Should be the same as the render driver filename (dri/renderD<drm_num>)
    drm_num = 128

    device_id = gfx_dict[options.gfx_version]["id"]

    # populate global node properties
    # NOTE: SIMD count triggers a valid GPU agent creation
    node_prop = (
        f"cpu_cores_count {options.num_cpus}\n"
        + f"simd_count {options.num_compute_units * options.simds_per_cu}\n"
        + f"mem_banks_count {mem_banks_cnt}\n"
        + "caches_count 0\n"
        + "io_links_count 0\n"
        + "cpu_core_id_base 16\n"
        + "simd_id_base 2147483648\n"
        + f"max_waves_per_simd {options.wfs_per_simd}\n"
        + f"lds_size_in_kb {int(options.lds_size / 1024)}\n"
        + "gds_size_in_kb 0\n"
        + f"wave_front_size {options.wf_size}\n"
        + "array_count 1\n"
        + f"simd_arrays_per_engine {options.sa_per_complex}\n"
        + f"cu_per_simd_array {options.cu_per_sa}\n"
        + f"simd_per_cu {options.simds_per_cu}\n"
        + "max_slots_scratch_cu 32\n"
        + "vendor_id 4098\n"
        + f"device_id {device_id}\n"
        + "location_id 8\n"
        + f"drm_render_minor {drm_num}\n"
        + f"max_engine_clk_fcompute {int(toFrequency(options.gpu_clock) / 1000000.0)}\n"
        + "local_mem_size 0\n"
        + "fw_version 699\n"
        + "capability 4738\n"
        + f"max_engine_clk_ccompute {int(toFrequency(options.CPUClock) / 1000000.0)}\n"
    )

    file_append((node_dir, "properties"), node_prop)

    for i in range(mem_banks_cnt):
        mem_dir = joinpath(node_dir, f"mem_banks/{i}")
        remake_dir(mem_dir)

        # Heap type value taken from real system, heap type values:
        # https://github.com/RadeonOpenCompute/ROCT-Thunk-Interface/blob/roc-4.0.x/include/hsakmttypes.h#L317
        mem_prop = (
            f"heap_type 0\n"
            + f"size_in_bytes {toMemorySize(options.mem_size)}"
            + f"flags 0\n"
            + f"width 64\n"
            + f"mem_clk_max 1600\n"
        )
        file_append((mem_dir, "properties"), mem_prop)
