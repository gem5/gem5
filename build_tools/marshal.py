# Copyright (c) 2019 ARM Limited
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
"""Marshal another python script.

This script compiles another script, marshals the resulting code object, and
writes it to stdout. Marshalling an object in this sense is essentially what
would happen to write it out in a pyc file.

This code object can then be read back in by another python script later and
executed, without having to have access to the original file. This is how
python code is embedded into the gem5 binary.

The output of the marshal module is *not* generally compatible accross python
interpretters, and so the exact same interpretter should be used both to run
this script, and to read in and execute the marshalled code later.
"""
import locale
import marshal
import os
import sys
import zlib

from blob import bytesToCppArray
from code_formatter import code_formatter

# Embed python files.  All .py files that have been indicated by a
# PySource() call in a SConscript need to be embedded into the M5
# library.  To do that, we compile the file to byte code, marshal the
# byte code, compress it, and then generate a c++ file that
# inserts the result into an array.

if len(sys.argv) < 4:
    print(f"Usage: {sys.argv[0]} CPP PY MODPATH ABSPATH", file=sys.stderr)
    sys.exit(1)

# Set the Python's locale settings manually based on the `LC_CTYPE`
# environment variable
if "LC_CTYPE" in os.environ:
    locale.setlocale(locale.LC_CTYPE, os.environ["LC_CTYPE"])

_, cpp, python, modpath, abspath = sys.argv

with open(python, "r") as f:
    src = f.read()

compiled = compile(src, python, "exec")
marshalled = marshal.dumps(compiled)

compressed = zlib.compress(marshalled)

code = code_formatter()
code(
    """\
#include "python/embedded.hh"

namespace gem5
{
namespace
{

""",
)

bytesToCppArray(code, "embedded_module_data", compressed)

# The name of the EmbeddedPython object doesn't matter since it's in an
# anonymous namespace, and it's constructor takes care of installing it into a
# global list.
code(
    """
EmbeddedPython embedded_module_info(
    "${abspath}",
    "${modpath}",
    embedded_module_data,
    ${{len(compressed)}},
    ${{len(marshalled)}});

} // anonymous namespace
} // namespace gem5
""",
)

code.write(cpp)
