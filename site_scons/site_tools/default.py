# Copyright (c) 2013, 2015-2017 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2011 Advanced Micro Devices, Inc.
# Copyright (c) 2009 The Hewlett-Packard Development Company
# Copyright (c) 2004-2005 The Regents of The University of Michigan
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

import os
import sys

import SCons.Tool
import SCons.Tool.default

from gem5_python_paths import extra_python_paths

def common_config(env):
    # export TERM so that clang reports errors in color
    use_vars = set([ 'AS', 'AR', 'CC', 'CXX', 'HOME', 'LD_LIBRARY_PATH',
                     'LIBRARY_PATH', 'PATH', 'PKG_CONFIG_PATH', 'PROTOC',
                     'PYTHONPATH', 'RANLIB', 'TERM' ])

    use_prefixes = [
        "ASAN_",           # address sanitizer symbolizer path and settings
        "CCACHE_",         # ccache (caching compiler wrapper) configuration
        "CCC_",            # clang static analyzer configuration
        "DISTCC_",         # distcc (distributed compiler wrapper) config
        "INCLUDE_SERVER_", # distcc pump server settings
        "M5",              # M5 configuration (e.g., path to kernels)
        ]

    for key,val in sorted(os.environ.items()):
        if key in use_vars or \
                any([key.startswith(prefix) for prefix in use_prefixes]):
            env['ENV'][key] = val

    # Tell scons to avoid implicit command dependencies to avoid issues
    # with the param wrappes being compiled twice (see
    # https://github.com/SCons/scons/issues/2811
    env['IMPLICIT_COMMAND_DEPENDENCIES'] = 0
    env.Decider('MD5-timestamp')
    env.root = env.Dir('#')
    env.srcdir = env.root.Dir('src')

    # add useful python code PYTHONPATH so it can be used by subprocesses
    # as well
    env.AppendENVPath('PYTHONPATH', extra_python_paths)

gem5_tool_list = [
    'git',
]

def generate(env):
    common_config(env)
    SCons.Tool.default.generate(env)
    for tool in gem5_tool_list:
        SCons.Tool.Tool(tool)(env)

def exists(env):
    return 1
