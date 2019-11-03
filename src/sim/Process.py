# Copyright (c) 2005-2008 The Regents of The University of Michigan
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
#
# Authors: Nathan Binkert

from m5.SimObject import *
from m5.params import *
from m5.proxy import *
from os import getcwd

class Process(SimObject):
    type = 'Process'
    cxx_header = "sim/process.hh"

    @cxxMethod
    def map(self, vaddr, paddr, size, cacheable=False):
        pass

    input = Param.String('cin', "filename for stdin")
    output = Param.String('cout', 'filename for stdout')
    errout = Param.String('cerr', 'filename for stderr')
    system = Param.System(Parent.any, "system process will run on")
    useArchPT = Param.Bool('false', 'maintain an in-memory version of the page\
                            table in an architecture-specific format')
    kvmInSE = Param.Bool('false', 'initialize the process for KvmCPU in SE')
    maxStackSize = Param.MemorySize('64MB', 'maximum size of the stack')

    uid = Param.Int(100, 'user id')
    euid = Param.Int(100, 'effective user id')
    gid = Param.Int(100, 'group id')
    egid = Param.Int(100, 'effective group id')
    pid = Param.Int(100, 'process id')
    ppid = Param.Int(0, 'parent process id')
    pgid = Param.Int(100, 'process group id')

    executable = Param.String('', "executable (overrides cmd[0] if set)")
    cmd = VectorParam.String("command line (executable plus arguments)")
    env = VectorParam.String([], "environment settings")
    cwd = Param.String(getcwd(), "current working directory")
    simpoint = Param.UInt64(0, 'simulation point at which to start simulation')
    drivers = VectorParam.EmulatedDriver([], 'Available emulated drivers')
    release = Param.String('5.1.0', "Linux kernel uname release")

    @classmethod
    def export_methods(cls, code):
        code('bool map(Addr vaddr, Addr paddr, int sz, bool cacheable=true);')

class EmulatedDriver(SimObject):
    type = 'EmulatedDriver'
    cxx_header = "sim/emul_driver.hh"
    abstract = True
    filename = Param.String("device file name (under /dev)")
