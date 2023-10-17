# Copyright (c) 2023 The Regents of the University of California
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

from mypy.stubgen import generate_stubs, parse_options

"""
This allows us to generate stubs for the modules in gem5. The output will be
a "typings" directory which can be used by Pylance (Python IntelliSense) to
infer typings in Visual Studio Code.

Note: A "typings" directory in the root of the workspace is the default
location for Pylance to look for typings. This can be changed via
`python.analysis.stubPath` in "settings.json".

Usage
=====

```sh
pip3 install -r requirements.txt
scons build/ALL/gem5.opt -j$(nproc)
./build/ALL/gem5.opt util/gem5-stubgen.py
```

"""

if __name__ == "__m5_main__":
    import m5

    # get a list of all modules exported by gem5
    modules = m5.__spec__.loader_state

    options = parse_options(
        ("--module " + " --module ".join(modules)).split(" ")
        + ["--output", "typings"]
    )
    generate_stubs(options)

if __name__ == "__main__":
    print("Error: This script is meant to be run with the gem5 binary")
    exit(1)
