# Copyright 2004-2006 The Regents of The University of Michigan
# Copyright 2010-20013 Advanced Micro Devices, Inc.
# Copyright 2013 Mark D. Hill and David A. Wood
# Copyright 2017-2020 ARM Limited
# Copyright 2021 Google, Inc.
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
import argparse
import importlib
import os.path
import sys

import importer
from code_formatter import code_formatter

parser = argparse.ArgumentParser()
parser.add_argument("modpath", help="module the enum belongs to")
parser.add_argument("enum_cc", help="enum cc file to generate")
parser.add_argument(
    "use_python", help="whether python is enabled in gem5 (True or False)"
)

args = parser.parse_args()

use_python = args.use_python.lower()
if use_python == "true":
    use_python = True
elif use_python == "false":
    use_python = False
else:
    print(f'Unrecognized "use_python" value {use_python}', file=sys.stderr)
    sys.exit(1)

basename = os.path.basename(args.enum_cc)
enum_name = os.path.splitext(basename)[0]

importer.install()
module = importlib.import_module(args.modpath)
enum = getattr(module, enum_name)

code = code_formatter()

wrapper_name = enum.wrapper_name
file_name = enum.__name__
name = enum.__name__ if enum.enum_name is None else enum.enum_name

code(
    """#include "base/compiler.hh"
#include "enums/$file_name.hh"

namespace gem5
{

"""
)

if enum.wrapper_is_struct:
    code("const char *${wrapper_name}::${name}Strings[Num_${name}] =")
else:
    if enum.is_class:
        code(
            """\
const char *${name}Strings[static_cast<int>(${name}::Num_${name})] =
"""
        )
    else:
        code(
            """namespace ${wrapper_name}
{"""
        )
        code.indent(1)
        code("const char *${name}Strings[Num_${name}] =")

code("{")
code.indent(1)
for val in enum.vals:
    code('"$val",')
code.dedent(1)
code("};")

if not enum.wrapper_is_struct and not enum.is_class:
    code.dedent(1)
    code("} // namespace ${wrapper_name}")

code("} // namespace gem5")


if use_python:
    name = enum.__name__
    enum_name = enum.__name__ if enum.enum_name is None else enum.enum_name
    wrapper_name = enum_name if enum.is_class else enum.wrapper_name

    code(
        """#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include <sim/init.hh>

namespace py = pybind11;

namespace gem5
{

static void
module_init(py::module_ &m_internal)
{
    py::module_ m = m_internal.def_submodule("enum_${name}");

"""
    )
    if enum.is_class:
        code('py::enum_<${enum_name}>(m, "enum_${name}")')
    else:
        code('py::enum_<${wrapper_name}::${enum_name}>(m, "enum_${name}")')

    code.indent()
    code.indent()
    for val in enum.vals:
        code('.value("${val}", ${wrapper_name}::${val})')
    code('.value("Num_${name}", ${wrapper_name}::Num_${enum_name})')
    if not enum.is_class:
        code(".export_values()")
    code(";")
    code.dedent()

    code("}")
    code.dedent()
    code(
        """
static EmbeddedPyBind embed_enum("enum_${name}", module_init);

} // namespace gem5
    """
    )

code.write(args.enum_cc)
