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
parser.add_argument("enum_hh", help="enum header file to generate")

args = parser.parse_args()

basename = os.path.basename(args.enum_hh)
enum_name = os.path.splitext(basename)[0]

importer.install()
module = importlib.import_module(args.modpath)
enum = getattr(module, enum_name)

code = code_formatter()

# Generate C++ class declaration for this enum type.
# Note that we wrap the enum in a class/struct to act as a namespace,
# so that the enum strings can be brief w/o worrying about collisions.
wrapper_name = enum.wrapper_name
wrapper = "struct" if enum.wrapper_is_struct else "namespace"
name = enum.__name__ if enum.enum_name is None else enum.enum_name
idem_macro = "__ENUM__%s__%s__" % (wrapper_name, name)

code(
    """\
#ifndef $idem_macro
#define $idem_macro

namespace gem5
{
"""
)
if enum.is_class:
    code(
        """\
enum class $name
{
"""
    )
else:
    code(
        """\
$wrapper $wrapper_name {
enum $name
{
"""
    )
    code.indent(1)
code.indent(1)
for val in enum.vals:
    code("$val = ${{enum.map[val]}},")
code("Num_$name = ${{len(enum.vals)}}")
code.dedent(1)
code("};")

if enum.is_class:
    code(
        """\
extern const char *${name}Strings[static_cast<int>(${name}::Num_${name})];
"""
    )
elif enum.wrapper_is_struct:
    code("static const char *${name}Strings[Num_${name}];")
else:
    code("extern const char *${name}Strings[Num_${name}];")

if not enum.is_class:
    code.dedent(1)
    code("}; // $wrapper_name")

code()
code("} // namespace gem5")

code()
code("#endif // $idem_macro")

code.write(args.enum_hh)
