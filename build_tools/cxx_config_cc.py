# Copyright 2004-2006 The Regents of The University of Michigan
# Copyright 2010-20013 Advanced Micro Devices, Inc.
# Copyright 2013 Mark D. Hill and David A. Wood
# Copyright 2017-2020 ARM Limited
# Copyright 2021 Google, Inc.
# Copyright 2023 COSEDA Technologies GmbH
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
parser.add_argument("cxx_config_cc", help="cxx config cc file to generate")

args = parser.parse_args()

basename = os.path.basename(args.cxx_config_cc)
sim_object_name = os.path.splitext(basename)[0]

importer.install()
module = importlib.import_module(args.modpath)
sim_object = getattr(module, sim_object_name)

import m5.params
from m5.params import isSimObjectClass

code = code_formatter()

entry_class = "CxxConfigDirectoryEntry_%s" % sim_object_name
param_class = "%sCxxConfigParams" % sim_object_name


def cxx_bool(b):
    return "true" if b else "false"


code('#include "params/%s.hh"' % sim_object_name)

for param in sim_object._params.values():
    if isSimObjectClass(param.ptype):
        code('#include "%s"' % param.ptype._value_dict["cxx_header"])
        code('#include "params/%s.hh"' % param.ptype.__name__)
    else:
        param.ptype.cxx_ini_predecls(code)

code(
    """#include "${{sim_object._value_dict['cxx_header']}}"
#include "base/str.hh"
#include "cxx_config/${sim_object_name}.hh"

namespace gem5
{

${param_class}::DirectoryEntry::DirectoryEntry()
{
"""
)
code.indent()
for param in sim_object._params.values():
    is_vector = isinstance(param, m5.params.VectorParamDesc)
    is_simobj = issubclass(param.ptype, m5.SimObject.SimObject)

    code(
        'parameters["%s"] = new ParamDesc("%s", %s, %s);'
        % (param.name, param.name, cxx_bool(is_vector), cxx_bool(is_simobj))
    )

for port in sim_object._ports.values():
    is_vector = isinstance(port, m5.params.VectorPort)
    is_requestor = port.is_source

    code(
        'ports["%s"] = new PortDesc("%s", %s, %s);'
        % (port.name, port.name, cxx_bool(is_vector), cxx_bool(is_requestor))
    )

code.dedent()

code(
    """}

bool
${param_class}::setSimObject(const std::string &name, SimObject *simObject)
{
    bool ret = true;
    if (false) {
"""
)

code.indent()
for param in sim_object._params.values():
    is_vector = isinstance(param, m5.params.VectorParamDesc)
    is_simobj = issubclass(param.ptype, m5.SimObject.SimObject)

    if is_simobj and not is_vector:
        code('} else if (name == "${{param.name}}") {')
        code.indent()
        code(
            "this->${{param.name}} = "
            "dynamic_cast<${{param.ptype.cxx_type}}>(simObject);"
        )
        code("if (simObject && !this->${{param.name}})")
        code("   ret = false;")
        code.dedent()
code.dedent()

code(
    """
    } else {
        ret = false;
    }

    return ret;
}

bool
${param_class}::setSimObjectVector(const std::string &name,
    const std::vector<SimObject *> &simObjects)
{
    bool ret = true;

    if (false) {
"""
)

code.indent()
for param in sim_object._params.values():
    is_vector = isinstance(param, m5.params.VectorParamDesc)
    is_simobj = issubclass(param.ptype, m5.SimObject.SimObject)

    if is_simobj and is_vector:
        code('} else if (name == "${{param.name}}") {')
        code.indent()
        code("this->${{param.name}}.clear();")
        code(
            "for (auto i = simObjects.begin(); "
            "ret && i != simObjects.end(); i ++)"
        )
        code("{")
        code.indent()
        code(
            "${{param.ptype.cxx_type}} object = "
            "dynamic_cast<${{param.ptype.cxx_type}}>(*i);"
        )
        code("if (*i && !object)")
        code("    ret = false;")
        code("else")
        code("    this->${{param.name}}.push_back(object);")
        code.dedent()
        code("}")
        code.dedent()
code.dedent()

code(
    """
    } else {
        ret = false;
    }

    return ret;
}

void
${param_class}::setName(const std::string &name_)
{
    this->name = name_;
}

bool
${param_class}::setParam(const std::string &name,
    const std::string &value, const Flags flags)
{
    bool ret = true;

    if (false) {
"""
)

code.indent()
for param in sim_object._params.values():
    is_vector = isinstance(param, m5.params.VectorParamDesc)
    is_simobj = issubclass(param.ptype, m5.SimObject.SimObject)

    if not is_simobj and not is_vector:
        code('} else if (name == "${{param.name}}") {')
        code.indent()
        param.ptype.cxx_ini_parse(
            code, "value", "this->%s" % param.name, "ret ="
        )
        code.dedent()
code.dedent()

code(
    """
    } else {
        ret = false;
    }

    return ret;
}

bool
${param_class}::setParamVector(const std::string &name,
    const std::vector<std::string> &values, const Flags flags)
{
    bool ret = true;

    if (false) {
"""
)

code.indent()
for param in sim_object._params.values():
    is_vector = isinstance(param, m5.params.VectorParamDesc)
    is_simobj = issubclass(param.ptype, m5.SimObject.SimObject)

    if not is_simobj and is_vector:
        code('} else if (name == "${{param.name}}") {')
        code.indent()
        code("${{param.name}}.clear();")
        code("for (auto i = values.begin(); ret && i != values.end(); i ++)")
        code("{")
        code.indent()
        code("${{param.ptype.cxx_type}} elem;")
        param.ptype.cxx_ini_parse(code, "*i", "elem", "ret =")
        code("if (ret)")
        code("    this->${{param.name}}.push_back(elem);")
        code.dedent()
        code("}")
        code.dedent()
code.dedent()

code(
    """
    } else {
        ret = false;
    }

    return ret;
}

bool
${param_class}::setPortConnectionCount(const std::string &name,
    unsigned int count)
{
    bool ret = true;

    if (false) {
"""
)

code.indent()
for port in sim_object._ports.values():
    code('} else if (name == "${{port.name}}") {')
    code("    this->port_${{port.name}}_connection_count = count;")
code.dedent()

code(
    """
    } else {
        ret = false;
    }

    return ret;
}

SimObject *
${param_class}::simObjectCreate()
{
"""
)

code.indent()
if hasattr(sim_object, "abstract") and sim_object.abstract:
    code("return nullptr;")
else:
    code("return this->create();")
code.dedent()

code(
    """}

} // namespace gem5
"""
)

code.write(args.cxx_config_cc)
