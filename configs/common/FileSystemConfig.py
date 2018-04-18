# Copyright (c) 2015 Advanced Micro Devices, Inc.
# All rights reserved
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
#
# Authors: David Hashe

from __future__ import print_function

import m5
from m5.objects import *
from m5.util.convert import *

import operator, os, platform, getpass
from os import mkdir, makedirs, getpid, listdir, stat, access
from pwd import getpwuid
from os.path import join as joinpath
from os.path import isdir
from shutil import rmtree, copyfile

def hex_mask(terms):
    dec_mask = reduce(operator.or_, [2**i for i in terms], 0)
    return "%08x" % dec_mask

def file_append(path, contents):
    with open(joinpath(*path), 'a') as f:
        f.write(str(contents))

def replace_tree(path):
    if isdir(path):
        rmtree(path)
    mkdir(path)

def config_filesystem(options):
    fsdir = joinpath(m5.options.outdir, 'fs')
    replace_tree(fsdir)

    # Set up /proc
    procdir = joinpath(fsdir, 'proc')
    mkdir(procdir)

    for i in xrange(options.num_cpus):
        one_cpu = 'processor       : %d\n' % (i)                  + \
                  'vendor_id       : Generic\n'                   + \
                  'cpu family      : 0\n'                         + \
                  'model           : 0\n'                         + \
                  'model name      : Generic\n'                   + \
                  'stepping        : 0\n'                         + \
                  'cpu MHz         : %0.3d\n'                       \
                        % (toFrequency(options.cpu_clock)/mega)   + \
                  'cache size:     : %dK\n'                         \
                        % (toMemorySize(options.l2_size)/kibi)    + \
                  'physical id     : 0\n'                         + \
                  'siblings        : %s\n'                          \
                        % options.num_cpus                        + \
                  'core id         : %d\n'                          \
                        % i                                       + \
                  'cpu cores       : %d\n'                          \
                        % options.num_cpus                        + \
                  'fpu             : yes\n'                       + \
                  'fpu exception   : yes\n'                       + \
                  'cpuid level     : 1\n'                         + \
                  'wp              : yes\n'                       + \
                  'flags           : fpu\n'                       + \
                  'cache alignment : %d\n'                          \
                        % options.cacheline_size                  + \
                  '\n'
        file_append((procdir, 'cpuinfo'), one_cpu)

    file_append((procdir, 'stat'), 'cpu 0 0 0 0 0 0 0\n')
    for i in xrange(options.num_cpus):
        file_append((procdir, 'stat'), 'cpu%d 0 0 0 0 0 0 0\n' % i)

    # Set up /sys
    sysdir = joinpath(fsdir, 'sys')
    mkdir(sysdir)

    # Set up /sys/devices/system/cpu
    cpudir = joinpath(sysdir, 'devices', 'system', 'cpu')
    makedirs(cpudir)

    file_append((cpudir, 'online'), '0-%d' % (options.num_cpus-1))
    file_append((cpudir, 'possible'), '0-%d' % (options.num_cpus-1))

    # Set up /tmp
    tmpdir = joinpath(fsdir, 'tmp')
    replace_tree(tmpdir)

def register_node(cpu_list, mem, node_number):
    nodebasedir = joinpath(m5.options.outdir, 'fs', 'sys', 'devices',
                           'system', 'node')

    nodedir = joinpath(nodebasedir,'node%d' % node_number)
    makedirs(nodedir)

    file_append((nodedir, 'cpumap'), hex_mask(cpu_list))
    file_append((nodedir, 'meminfo'),
                'Node %d MemTotal: %dkB' % (node_number,
                toMemorySize(str(mem))/kibi))

def register_cpu(physical_package_id, core_siblings,
                 core_id, thread_siblings):
    cpudir = joinpath(m5.options.outdir, 'fs',  'sys', 'devices', 'system',
                      'cpu', 'cpu%d' % core_id)

    if not isdir(joinpath(cpudir, 'topology')):
        makedirs(joinpath(cpudir, 'topology'))
    if not isdir(joinpath(cpudir, 'cache')):
        makedirs(joinpath(cpudir, 'cache'))

    file_append((cpudir, 'online'), '1')
    file_append((cpudir, 'topology', 'physical_package_id'),
                physical_package_id)
    file_append((cpudir, 'topology', 'core_siblings'),
                hex_mask(core_siblings))
    file_append((cpudir, 'topology', 'core_id'), core_id)
    file_append((cpudir, 'topology', 'thread_siblings'),
                hex_mask(thread_siblings))

def register_cache(level, idu_type, size, line_size, assoc, cpus):
    fsdir = joinpath(m5.options.outdir, 'fs')
    for i in cpus:
        cachedir = joinpath(fsdir, 'sys', 'devices', 'system', 'cpu',
                            'cpu%d' % i, 'cache')

        j = 0
        while isdir(joinpath(cachedir, 'index%d' % j)):
            j += 1
        indexdir = joinpath(cachedir, 'index%d' % j)
        makedirs(indexdir)

        file_append((indexdir, 'level'), level)
        file_append((indexdir, 'type'), idu_type)
        file_append((indexdir, 'size'), "%dK" % (toMemorySize(size)/kibi))
        file_append((indexdir, 'coherency_line_size'), line_size)

        # Since cache size = number of indices * associativity * block size
        num_sets = toMemorySize(size) / int(assoc) * int(line_size)

        file_append((indexdir, 'number_of_sets'), num_sets)
        file_append((indexdir, 'physical_line_partition'), '1')
        file_append((indexdir, 'shared_cpu_map'), hex_mask(cpus))

def redirect_paths(chroot):
    # Redirect filesystem syscalls from src to the first matching dests
    redirect_paths = [RedirectPath(app_path = "/proc",
                          host_paths = ["%s/fs/proc" % m5.options.outdir]),
                      RedirectPath(app_path = "/sys",
                          host_paths = ["%s/fs/sys"  % m5.options.outdir]),
                      RedirectPath(app_path = "/tmp",
                          host_paths = ["%s/fs/tmp"  % m5.options.outdir]),
                      RedirectPath(app_path = "/",
                          host_paths = ["%s"         % chroot])]
    return redirect_paths
