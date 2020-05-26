# Copyright (c) 2018 Advanced Micro Devices, Inc.
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

import m5

import operator
from os import mkdir, makedirs, getpid, listdir, fsync
from os.path import join as joinpath
from os.path import isdir
from shutil import rmtree, copyfile
from m5.util.convert import toFrequency

def file_append(path, contents):
    with open(joinpath(*path), 'a') as f:
        f.write(str(contents))
        f.flush()
        fsync(f.fileno())

def remake_dir(path):
    if isdir(path):
        rmtree(path)
    makedirs(path)

def createHsaTopology(options):
    topology_dir = joinpath(m5.options.outdir, \
        'fs/sys/devices/virtual/kfd/kfd/topology')
    remake_dir(topology_dir)

    # Ripped from real Kaveri platform to appease kmt version checks
    # Set up generation_id
    file_append((topology_dir, 'generation_id'), 1)

    # Set up system properties
    sys_prop = 'platform_oem 2314885673410447169\n' + \
               'platform_id 35322352389441\n'       + \
               'platform_rev 1\n'
    file_append((topology_dir, 'system_properties'), sys_prop)

    # Populate the topology tree
    # TODO: Just the bare minimum to pass for now
    node_dir = joinpath(topology_dir, 'nodes/0')
    remake_dir(node_dir)

    # must show valid kaveri gpu id or massive meltdown
    file_append((node_dir, 'gpu_id'), 2765)

    # must have marketing name
    file_append((node_dir, 'name'), 'Carrizo\n')

    # populate global node properties
    # NOTE: SIMD count triggers a valid GPU agent creation
    node_prop = 'cpu_cores_count %s\n' % options.num_cpus                   + \
                'simd_count %s\n'                                             \
                    % (options.num_compute_units * options.simds_per_cu)    + \
                'mem_banks_count 0\n'                                       + \
                'caches_count 0\n'                                          + \
                'io_links_count 0\n'                                        + \
                'cpu_core_id_base 16\n'                                     + \
                'simd_id_base 2147483648\n'                                 + \
                'max_waves_per_simd %s\n' % options.wfs_per_simd            + \
                'lds_size_in_kb %s\n' % int(options.lds_size / 1024)        + \
                'gds_size_in_kb 0\n'                                        + \
                'wave_front_size %s\n' % options.wf_size                    + \
                'array_count 1\n'                                           + \
                'simd_arrays_per_engine %s\n' % options.sa_per_complex      + \
                'cu_per_simd_array %s\n' % options.cu_per_sa                + \
                'simd_per_cu %s\n' % options.simds_per_cu                   + \
                'max_slots_scratch_cu 32\n'                                 + \
                'vendor_id 4098\n'                                          + \
                'device_id 39028\n'                                         + \
                'location_id 8\n'                                           + \
                'max_engine_clk_fcompute %s\n'                                \
                    % int(toFrequency(options.gpu_clock) / 1e6)             + \
                'local_mem_size 0\n'                                        + \
                'fw_version 699\n'                                          + \
                'capability 4738\n'                                         + \
                'max_engine_clk_ccompute %s\n'                                \
                    % int(toFrequency(options.CPUClock) / 1e6)

    file_append((node_dir, 'properties'), node_prop)
