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

config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)

class PathSearchFunc(object):
    _sys_paths = None

    def __init__(self, *subdirs):
        self._subdir = os.path.join(*subdirs)

    def __call__(self, filename):
        if self._sys_paths is None:
            try:
                paths = os.environ['M5_PATH'].split(':')
            except KeyError:
                paths = [ '/dist/m5/system', '/n/poolfs/z/dist/m5/system' ]

            # expand '~' and '~user' in paths
            paths = map(os.path.expanduser, paths)

            # filter out non-existent directories
            paths = filter(os.path.isdir, paths)

            if not paths:
                raise IOError, "Can't find a path to system files."

            self._sys_paths = paths

        filepath = os.path.join(self._subdir, filename)
        paths = (os.path.join(p, filepath) for p in self._sys_paths)
        try:
            return next(p for p in paths if os.path.exists(p))
        except StopIteration:
            raise IOError, "Can't find file '%s' on path." % filename

disk = PathSearchFunc('disks')
binary = PathSearchFunc('binaries')
script = PathSearchFunc('boot')
