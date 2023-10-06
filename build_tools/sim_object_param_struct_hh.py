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
parser.add_argument("modpath", help="module the simobject belongs to")
parser.add_argument("param_hh", help="parameter header file to generate")

args = parser.parse_args()

basename = os.path.basename(args.param_hh)
sim_object_name = os.path.splitext(basename)[0]

importer.install()
module = importlib.import_module(args.modpath)
sim_object = getattr(module, sim_object_name)

from m5.objects.SimObject import SimObject
from m5.params import Enum

code = code_formatter()

# The 'local' attribute restricts us to the params declared in
# the object itself, not including inherited params (which
# will also be inherited from the base class's param struct
# here). Sort the params based on their key
params = list(
    map(lambda k_v: k_v[1], sorted(sim_object._params.local.items())),
)
ports = sim_object._ports.local
try:
    ptypes = [p.ptype for p in params]
except:
    print(sim_object)
    print(params)
    raise

warned_about_nested_templates = False


class CxxClass(object):
    def __init__(self, sig, template_params=[]):
        # Split the signature into its constituent parts. This could
        # potentially be done with regular expressions, but
        # it's simple enough to pick appart a class signature
        # manually.
        parts = sig.split("<", 1)
        base = parts[0]
        t_args = []
        if len(parts) > 1:
            # The signature had template arguments.
            text = parts[1].rstrip(" \t\n>")
            arg = ""
            # Keep track of nesting to avoid splitting on ","s embedded
            # in the arguments themselves.
            depth = 0
            for c in text:
                if c == "<":
                    depth = depth + 1
                    if depth > 0 and not warned_about_nested_templates:
                        warned_about_nested_templates = True
                        print(
                            "Nested template argument in cxx_class."
                            " This feature is largely untested and "
                            " may not work.",
                        )
                elif c == ">":
                    depth = depth - 1
                elif c == "," and depth == 0:
                    t_args.append(arg.strip())
                    arg = ""
                else:
                    arg = arg + c
            if arg:
                t_args.append(arg.strip())
        # Split the non-template part on :: boundaries.
        class_path = base.split("::")

        # The namespaces are everything except the last part of the class path.
        self.namespaces = class_path[:-1]
        # And the class name is the last part.
        self.name = class_path[-1]

        self.template_params = template_params
        self.template_arguments = []
        # Iterate through the template arguments and their values. This
        # will likely break if parameter packs are used.
        for arg, param in zip(t_args, template_params):
            type_keys = ("class", "typename")
            # If a parameter is a type, parse it recursively. Otherwise
            # assume it's a constant, and store it verbatim.
            if any(param.strip().startswith(kw) for kw in type_keys):
                self.template_arguments.append(CxxClass(arg))
            else:
                self.template_arguments.append(arg)

    def declare(self, code):
        # First declare any template argument types.
        for arg in self.template_arguments:
            if isinstance(arg, CxxClass):
                arg.declare(code)
        # Re-open the target namespace.
        for ns in self.namespaces:
            code("namespace $ns {")
        # If this is a class template...
        if self.template_params:
            code('template <${{", ".join(self.template_params)}}>')
        # The actual class declaration.
        code("class ${{self.name}};")
        # Close the target namespaces.
        for ns in reversed(self.namespaces):
            code("} // namespace $ns")


code(
    """\
#ifndef __PARAMS__${sim_object}__
#define __PARAMS__${sim_object}__

""",
)


# The base SimObject has a couple of params that get
# automatically set from Python without being declared through
# the normal Param mechanism; we slip them in here (needed
# predecls now, actual declarations below)
if sim_object == SimObject:
    code("""#include <string>""")

cxx_class = CxxClass(
    sim_object._value_dict["cxx_class"],
    sim_object._value_dict["cxx_template_params"],
)

# A forward class declaration is sufficient since we are just
# declaring a pointer.
cxx_class.declare(code)

for param in params:
    param.cxx_predecls(code)
for port in ports.values():
    port.cxx_predecls(code)
code()

if sim_object._base:
    code('#include "params/${{sim_object._base.type}}.hh"')
    code()

for ptype in ptypes:
    if issubclass(ptype, Enum):
        code('#include "enums/${{ptype.__name__}}.hh"')
        code()

code("namespace gem5")
code("{")
code("")

# now generate the actual param struct
code("struct ${sim_object}Params")
if sim_object._base:
    code("    : public ${{sim_object._base.type}}Params")
code("{")
if not hasattr(sim_object, "abstract") or not sim_object.abstract:
    if "type" in sim_object.__dict__:
        code("    ${{sim_object.cxx_type}} create() const;")

code.indent()
if sim_object == SimObject:
    code(
        """
SimObjectParams() {}
virtual ~SimObjectParams() {}

std::string name;
    """,
    )

for param in params:
    param.cxx_decl(code)
for port in ports.values():
    port.cxx_decl(code)

code.dedent()
code("};")
code()
code("} // namespace gem5")

code()
code("#endif // __PARAMS__${sim_object}__")

code.write(args.param_hh)
