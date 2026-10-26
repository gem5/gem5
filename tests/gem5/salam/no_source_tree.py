# Copyright (c) 2026 Akanksha Chaudhari
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


# Runs a configuration script, given with its arguments, with the source
# tree's configs/ and util/ directories off the module path, and fails
# if any module was loaded from them.

import os
import runpy
import sys

base = os.path.realpath(os.path.join(os.path.dirname(__file__), "../../.."))
banned = [os.path.join(base, d) + os.sep for d in ("configs", "util")]


def in_source_tree(path):
    path = os.path.realpath(path) + os.sep
    return any(path.startswith(b) for b in banned)


sys.path[:] = [p for p in sys.path if not in_source_tree(p)]
script, *sys.argv[1:] = sys.argv[1:]
sys.argv[0] = script
runpy.run_path(script, run_name="__main__")

loaded = sorted(
    module.__file__
    for module in list(sys.modules.values())
    if getattr(module, "__file__", None)
    and in_source_tree(os.path.dirname(module.__file__))
)
if loaded:
    print(
        "modules loaded from the source tree: " + ", ".join(loaded),
        file=sys.stderr,
    )
    sys.exit(1)
print("No modules were loaded from configs/ or util/.")
