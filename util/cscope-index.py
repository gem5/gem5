#! /usr/bin/python
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
# Authors: Steve Reinhardt

# Generate list of files to index with cscope and then generate cscope index.

# Should be run from root of m5 tree (i.e. as 'util/cscope-index.py').

import os

# absolute paths to skip
skipdirs = [ 'src/unittest', 'src/doxygen' ]

# suffixes of files to index
suffixes = [ '.cc', '.hh', '.c', '.h' ]

def oksuffix(f):
    for s in suffixes:
        if f.endswith(s):
            return True
    return False

file_list = file('cscope.files', 'w')

for dirpath,subdirs,files in os.walk('src'):
    # filter out undesirable subdirectories
    for i,dir in enumerate(subdirs):
        if dir == 'SCCS':
            del subdirs[i]
            break

    # filter out undesirable absolute paths
    if dirpath in skipdirs:
        del subdirs[:]
        continue

    # find C/C++ sources
    okfiles = [f for f in files if oksuffix(f)]
    if okfiles:
        print >> file_list, \
              '\n'.join([os.path.join(dirpath, f) for f in okfiles])

file_list.close()

# run cscope to generate index
os.system("cscope -b")
