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
parser.add_argument("param_cc", help="parameter cc file to generate")
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

basename = os.path.basename(args.param_cc)
no_ext = os.path.splitext(basename)[0]
sim_object_name = "_".join(no_ext.split("_")[1:])

importer.install()
module = importlib.import_module(args.modpath)
sim_object = getattr(module, sim_object_name)

from m5.objects.SimObject import PyBindProperty

code = code_formatter()

py_class_name = sim_object.pybind_class

# The 'local' attribute restricts us to the params declared in
# the object itself, not including inherited params (which
# will also be inherited from the base class's param struct
# here). Sort the params based on their key
params = list(
    map(lambda k_v: k_v[1], sorted(sim_object._params.local.items()))
)
ports = sim_object._ports.local

# only include pybind if python is enabled in the build
if use_python:
    code(
        """#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include <type_traits>

#include "base/compiler.hh"
#include "params/$sim_object.hh"
#include "sim/init.hh"
#include "sim/sim_object.hh"

#include "${{sim_object.cxx_header}}"

"""
    )
else:
    code(
        """
#include <type_traits>

#include "base/compiler.hh"
#include "params/$sim_object.hh"

#include "${{sim_object.cxx_header}}"

"""
    )
# only include the python params code if python is enabled.
if use_python:
    for param in params:
        param.pybind_predecls(code)

    code(
        """namespace py = pybind11;

namespace gem5
{

static void
module_init(py::module_ &m_internal)
{
py::module_ m = m_internal.def_submodule("param_${sim_object}");
"""
    )
    code.indent()
    if sim_object._base:
        code(
            "py::class_<${sim_object}Params, "
            "${{sim_object._base.type}}Params, "
            "std::unique_ptr<${{sim_object}}Params, py::nodelete>>("
            'm, "${sim_object}Params")'
        )
    else:
        code(
            "py::class_<${sim_object}Params, "
            "std::unique_ptr<${sim_object}Params, py::nodelete>>("
            'm, "${sim_object}Params")'
        )

    code.indent()
    if not hasattr(sim_object, "abstract") or not sim_object.abstract:
        code(".def(py::init<>())")
        code('.def("create", &${sim_object}Params::create)')

    param_exports = (
        sim_object.cxx_param_exports
        + [
            PyBindProperty(k)
            for k, v in sorted(sim_object._params.local.items())
        ]
        + [
            PyBindProperty(f"port_{port.name}_connection_count")
            for port in ports.values()
        ]
    )
    for exp in param_exports:
        exp.export(code, f"{sim_object}Params")

    code(";")
    code()
    code.dedent()

    bases = []
    if "cxx_base" in sim_object._value_dict:
        # If the c++ base class implied by python inheritance was
        # overridden, use that value.
        if sim_object.cxx_base:
            bases.append(sim_object.cxx_base)
    elif sim_object._base:
        # If not and if there was a SimObject base, use its c++ class
        # as this class' base.
        bases.append(sim_object._base.cxx_class)
    # Add in any extra bases that were requested.
    bases.extend(sim_object.cxx_extra_bases)

    if bases:
        base_str = ", ".join(bases)
        code(
            "py::class_<${{sim_object.cxx_class}}, ${base_str}, "
            "std::unique_ptr<${{sim_object.cxx_class}}, py::nodelete>>("
            'm, "${py_class_name}")'
        )
    else:
        code(
            "py::class_<${{sim_object.cxx_class}}, "
            "std::unique_ptr<${{sim_object.cxx_class}}, py::nodelete>>("
            'm, "${py_class_name}")'
        )
    code.indent()
    for exp in sim_object.cxx_exports:
        exp.export(code, sim_object.cxx_class)
    code(";")
    code.dedent()
    code()
    code.dedent()
    code("}")
    code()
    code(
        "static EmbeddedPyBind " 'embed_obj("${0}", module_init, "${1}");',
        sim_object,
        sim_object._base.type if sim_object._base else "",
    )
    code()
    code("} // namespace gem5")

# include the create() methods whether or not python is enabled.
if not hasattr(sim_object, "abstract") or not sim_object.abstract:
    if "type" in sim_object.__dict__:
        code(
            """
namespace gem5
{

namespace
{

/*
 * If we can't define a default create() method for this params
 * struct because the SimObject doesn't have the right
 * constructor, use template magic to make it so we're actually
 * defining a create method for this class instead.
 */
class Dummy${sim_object}ParamsClass
{
  public:
    ${{sim_object.cxx_class}} *create() const;
};

template <class CxxClass, class Enable=void>
class Dummy${sim_object}Shunt;

/*
 * This version directs to the real Params struct and the
 * default behavior of create if there's an appropriate
 * constructor.
 */
template <class CxxClass>
class Dummy${sim_object}Shunt<CxxClass, std::enable_if_t<
    std::is_constructible_v<CxxClass, const ${sim_object}Params &>>>
{
  public:
    using Params = ${sim_object}Params;
    static ${{sim_object.cxx_class}} *
    create(const Params &p)
    {
        return new CxxClass(p);
    }
};

/*
 * This version diverts to the DummyParamsClass and a dummy
 * implementation of create if the appropriate constructor does
 * not exist.
 */
template <class CxxClass>
class Dummy${sim_object}Shunt<CxxClass, std::enable_if_t<
    !std::is_constructible_v<CxxClass, const ${sim_object}Params &>>>
{
  public:
    using Params = Dummy${sim_object}ParamsClass;
    static ${{sim_object.cxx_class}} *
    create(const Params &p)
    {
        return nullptr;
    }
};

} // anonymous namespace

/*
 * An implementation of either the real Params struct's create
 * method, or the Dummy one. Either an implementation is
 * mandantory since this was shunted off to the dummy class, or
 * one is optional which will override this weak version.
 */
[[maybe_unused]] ${{sim_object.cxx_class}} *
Dummy${sim_object}Shunt<${{sim_object.cxx_class}}>::Params::create() const
{
    return Dummy${sim_object}Shunt<${{sim_object.cxx_class}}>::create(*this);
}

} // namespace gem5
"""
        )

code.write(args.param_cc)
