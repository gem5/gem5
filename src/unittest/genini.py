#!/usr/bin/env python2
# Copyright (c) 2005 The Regents of The University of Michigan
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

import getopt, os, os.path, sys
from os.path import join as joinpath, realpath

mypath = sys.path[0]
sys.path.append(joinpath(mypath, '..'))
sys.path.append(joinpath(mypath, '../python'))
sys.path.append(joinpath(mypath, '../util/pbs'))

pathlist = [ '.' ]

m5_build_env = {}

try:
    opts, args = getopt.getopt(sys.argv[1:], '-E:I:')
    for opt,arg in opts:
        if opt == '-E':
            offset = arg.find('=')
            if offset == -1:
                name = arg
                value = 'True'
            else:
                name = arg[:offset]
                value = arg[offset+1:]
            os.environ[name] = value
            m5_build_env[name] = value
        if opt == '-I':
            pathlist.append(arg)
except getopt.GetoptError:
    sys.exit('Improper Usage')

import __main__
__main__.m5_build_env = m5_build_env

from m5 import *

for path in pathlist:
    AddToPath(path)

for arg in args:
    m5execfile(arg, globals())

if globals().has_key('root') and isinstance(root, Root):
    instantiate(root)
else:
    print "Instantiation skipped: no root object found."
