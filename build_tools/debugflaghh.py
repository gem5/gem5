# Copyright (c) 2018, 2020 ARM Limited
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
# Copyright (c) 2004-2005 The Regents of The University of Michigan
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
import argparse
import sys

from code_formatter import code_formatter

parser = argparse.ArgumentParser()
parser.add_argument("hh", help="the path of the debug flag header file")
parser.add_argument("name", help="the name of the debug flag")
parser.add_argument("desc", help="a description of the debug flag")
parser.add_argument(
    "fmt",
    help="whether the flag is a format flag (True or False)",
)
parser.add_argument(
    "components",
    help="components of a compound flag, if applicable, joined with :",
)

args = parser.parse_args()

fmt = args.fmt.lower()
if fmt == "true":
    fmt = True
elif fmt == "false":
    fmt = False
else:
    print(f'Unrecognized "FMT" value {fmt}', file=sys.stderr)
    sys.exit(1)
components = args.components.split(":") if args.components else []

code = code_formatter()

code(
    """
#ifndef __DEBUG_${{args.name}}_HH__
#define __DEBUG_${{args.name}}_HH__

#include "base/compiler.hh" // For namespace deprecation
#include "base/debug.hh"
""",
)
for flag in components:
    code('#include "debug/${flag}.hh"')
code(
    """
namespace gem5
{

namespace debug
{

namespace unions
{
""",
)

# Use unions to prevent debug flags from being destructed. It's the
# responsibility of the programmer to handle object destruction for members
# of the union. We purposefully leave that destructor empty so that we can
# use debug flags even in the destructors of other objects.
if components:
    code(
        """
inline union ${{args.name}}
{
    ~${{args.name}}() {}
    CompoundFlag ${{args.name}} = {
        "${{args.name}}", "${{args.desc}}", {
            ${{",\\n            ".join(
                f"(Flag *)&::gem5::debug::{flag}" for flag in components)}}
        }
    };
} ${{args.name}};
""",
    )
else:
    code(
        """
inline union ${{args.name}}
{
    ~${{args.name}}() {}
    SimpleFlag ${{args.name}} = {
        "${{args.name}}", "${{args.desc}}", ${{"true" if fmt else "false"}}
    };
} ${{args.name}};
""",
    )

code(
    """
} // namespace unions

inline constexpr const auto& ${{args.name}} =
    ::gem5::debug::unions::${{args.name}}.${{args.name}};

} // namespace debug
} // namespace gem5

#endif // __DEBUG_${{args.name}}_HH__
""",
)

code.write(args.hh)
