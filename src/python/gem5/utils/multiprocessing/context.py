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
This file contains extensions of the multiprocessing module to be used with gem5
Some code inspired by the Python standard library implementation of the
multiprocessing module (i.e., cpython/Lib/multiprocessing/).
"""

from multiprocessing import context, process
from multiprocessing.context import DefaultContext

# The `_start_method` must be `None` for the `Spawn_gem5Process` class.
# Otherwise, in `_bootstrap` in the `BaseProcess` it will try to force the
# `_start_method` to be gem5-specific, which the `multiprocessing` module
# doesn't understand.
class Spawn_gem5Process(process.BaseProcess):
    _start_method = None

    @staticmethod
    def _Popen(process_obj):
        from .popen_spawn_gem5 import Popen

        return Popen(process_obj)


class Process(process.BaseProcess):
    _start_method = None

    @staticmethod
    def _Popen(process_obj):
        return _default_context.get_context().Process._Popen(process_obj)


class gem5Context(context.BaseContext):
    _name = "spawn_gem5"
    Process = Spawn_gem5Process

    def get_context(self, method=None):
        if method is None:
            return self
        try:
            ctx = _concrete_contexts[method]
        except KeyError:
            raise ValueError(f"cannot find context for {method!r}") from None
        ctx._check_available()
        return ctx


_concrete_contexts = {"spawn_gem5": gem5Context()}

_default_context = DefaultContext(_concrete_contexts["spawn_gem5"])
