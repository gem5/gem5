# Copyright (c) 2008 The Hewlett-Packard Development Company
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

from collections.abc import Mapping

from m5.util import printList

import _m5.debug
from _m5.debug import (
    CompoundFlag,
    SimpleFlag,
    schedBreak,
)


def help():
    sorted_flags = sorted(flags.items(), key=lambda kv: kv[0])

    print("Base Flags:")
    for name, flag in filter(
        lambda kv: isinstance(kv[1], SimpleFlag) and not kv[1].isFormat,
        sorted_flags,
    ):
        print(f"    {name}: {flag.desc}")
    print()
    print("Compound Flags:")
    for name, flag in filter(
        lambda kv: isinstance(kv[1], CompoundFlag), sorted_flags
    ):
        print(f"    {name}: {flag.desc}")
        # The list of kids for flag "All" is too long, so it is not printed
        if name != "All":
            printList([c.name for c in flag.kids()], indent=8)
        else:
            print("        All Base Flags")
    print()
    print("Formatting Flags:")
    for name, flag in filter(
        lambda kv: isinstance(kv[1], SimpleFlag) and kv[1].isFormat,
        sorted_flags,
    ):
        print(f"    {name}: {flag.desc}")
    print()


class AllFlags(Mapping):
    def __init__(self):
        self._version = -1
        self._dict = {}

    def _update(self):
        current_version = _m5.debug.getAllFlagsVersion()
        if self._version == current_version:
            return

        self._dict.clear()
        for name, flag in _m5.debug.allFlags().items():
            self._dict[name] = flag
        self._version = current_version

    def __contains__(self, item):
        self._update()
        return item in self._dict

    def __getitem__(self, item):
        self._update()
        return self._dict[item]

    def __iter__(self):
        self._update()
        return iter(self._dict)

    def __len__(self):
        self._update()
        return len(self._dict)

    def keys(self):
        self._update()
        return self._dict.keys()

    def values(self):
        self._update()
        return self._dict.values()

    def items(self):
        self._update()
        return self._dict.items()


flags = AllFlags()
