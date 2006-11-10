# Copyright (c) 2006 The Regents of The University of Michigan
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
# Authors: Ali Saidi

import os, sys
from os.path import isdir, join as joinpath
from os import environ as env

config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)

def disk(file):
    system()
    return joinpath(disk.dir, file)

def binary(file):
    system()
    return joinpath(binary.dir, file)

def script(file):
    system()
    return joinpath(script.dir, file)

def system():
    if not system.dir:
        try:
                path = env['M5_PATH'].split(':')
        except KeyError:
                path = [ '/dist/m5/system', '/n/poolfs/z/dist/m5/system' ]

        for system.dir in path:
            if os.path.isdir(system.dir):
                break
        else:
            raise ImportError, "Can't find a path to system files."

    if not binary.dir:
        binary.dir = joinpath(system.dir, 'binaries')
    if not disk.dir:
        disk.dir = joinpath(system.dir, 'disks')
    if not script.dir:
        script.dir = joinpath(config_root, 'boot')

system.dir = None
binary.dir = None
disk.dir = None
script.dir = None
