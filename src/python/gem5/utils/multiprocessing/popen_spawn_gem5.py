# Copyright (c) 2022 The Regents of The University of California
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
"""
This file contains extensions of the multiprocessing module to be used with gem5.
Specifically, it contains the code to spawn a new gem5 process with Popen.
Some code is from the Python standard library implementation of the
multiprocessing module (i.e., cpython/Lib/multiprocessing/).
"""
import io
import os
from multiprocessing import popen_spawn_posix
from multiprocessing import spawn
from multiprocessing import util
from multiprocessing.context import reduction
from multiprocessing.context import set_spawning_popen

from ._command_line import get_command_line

__all__ = ["Popen"]


class Popen(popen_spawn_posix.Popen):
    method = "spawn_gem5"

    def __init__(self, process_obj):
        super().__init__(process_obj)

    # Copyright (c) 2001-2022 Python Software Foundation; All Rights Reserved
    # from cpython/Lib/multiprocessing/popen_spawn_posix.py
    def _launch(self, process_obj):
        from multiprocessing import resource_tracker

        tracker_fd = resource_tracker.getfd()
        self._fds.append(tracker_fd)
        prep_data = spawn.get_preparation_data(process_obj._name)
        fp = io.BytesIO()
        set_spawning_popen(self)
        try:
            reduction.dump(prep_data, fp)
            reduction.dump(process_obj, fp)
        finally:
            set_spawning_popen(None)

        parent_r = child_w = child_r = parent_w = None
        try:
            parent_r, child_w = os.pipe()
            child_r, parent_w = os.pipe()
            # Note: This next line is the only modification
            cmd = get_command_line(
                tracker_fd=tracker_fd,
                pipe_handle=child_r,
                name=process_obj.name,
            )
            self._fds.extend([child_r, child_w])
            self.pid = util.spawnv_passfds(
                spawn.get_executable(), cmd, self._fds
            )
            self.sentinel = parent_r
            with open(parent_w, "wb", closefd=False) as f:
                f.write(fp.getbuffer())
        finally:
            fds_to_close = []
            for fd in (parent_r, parent_w):
                if fd is not None:
                    fds_to_close.append(fd)
            self.finalizer = util.Finalize(self, util.close_fds, fds_to_close)

            for fd in (child_r, child_w):
                if fd is not None:
                    os.close(fd)
