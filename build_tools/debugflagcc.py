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

import collections
import sys

from code_formatter import code_formatter

def usage():
    print(f"Usage: {sys.argv[0]} CC [NAME DESC FMT COMPONENTS]...",
            file=sys.stderr)
    sys.exit(1)

if len(sys.argv) < 2:
    usage()

cc = sys.argv[1]


FlagInfo = collections.namedtuple('FlagInfo',
        ['name', 'desc', 'fmt', 'components'])

flags = []

pos = 2
# Extract groups of arguments for each flag.
while pos < len(sys.argv):
    if len(sys.argv) < pos + 4:
        usage()

    name, desc, fmt, components = sys.argv[pos:pos+4]
    pos += 4
    fmt = fmt.lower()
    if fmt == 'true':
        fmt = True
    elif fmt == 'false':
        fmt = False
    else:
        print(f'Unrecognized "FMT" value {fmt}', file=sys.stderr)
        sys.exit(1)
    components = components.split(':') if components else []
    flags.append(FlagInfo(name, desc, fmt, components))


code = code_formatter()

# File header.
code('''
#include "base/compiler.hh" // For namespace deprecation
#include "base/debug.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Debug, debug);
namespace debug
{
''')

# Group the flags into either simple flags or compound flags.
simple_flags = sorted(filter(lambda f: not f.components, flags))
compound_flags = sorted(filter(lambda f: f.components, flags))

# We intentionally make flag a reference to a heap allocated object so
# (1) It has a similar interface to a global object like before
# (2) It does not get destructed at the end of simulation
# The second property is desirable as global objects from different
# translation units do not have a defined destruction order, so it'll
# be unsafe to access debug flags in their destructor in such cases.
for flag in simple_flags:
    name, desc, components, fmt = \
            flag.name, flag.desc, flag.components, flag.fmt
    fmt = 'true' if fmt else 'false'
    code('''
SimpleFlag& $name = *(
    new SimpleFlag("$name", "$desc", $fmt));''')

for flag in compound_flags:
    name, desc, components = flag.name, flag.desc, flag.components
    code('''
CompoundFlag& $name = *(new CompoundFlag("$name", "$desc", {
    ${{', '.join('&' + simple for simple in components)}}
}));''')

code('''
} // namespace debug
} // namespace gem5''')

code.write(cc)
