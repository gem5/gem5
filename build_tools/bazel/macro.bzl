# Copyright 2025 Google, Inc.
# All Rights Reserved.
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

load("//build_tools/bazel:rules.bzl", "gem5_py_source", "get_py_name", "sim_object")

def macro_py_source(module_prefix, py_file):
    py_name = get_py_name(py_file)
    gem5_py_source(
        name_pyo = py_name + "_py_pyo",
        name_o = py_name + "_py_o",
        src = py_file,
        module_prefix = module_prefix,
    )
    return py_name + "_py_pyo"

def macro_sim_object(py_file, sim_objects, enums = []):
    """SConscript's direct correspondence of SimObject() in Bazel.

    The core implementation `sim_objects()` yields two targets, named `name_pyo`
    and `name_o` respectively.  The interface of `sim_objects()` is akin to a
    Bazel rule in that the `name*` arguments reflect to the target names directly.
    However, BUILD file writers are encouraged to use this `macro_sim_object()`
    wrapper version instead, which provides an interface closer to which
    SConscript is using, to declare a SimObject.
    """
    py_name = get_py_name(py_file)
    sim_object(
        name_pyo = py_name + "_py_pyo",
        name_o = py_name + "_py_o",
        src = py_file,
        sim_objects = sim_objects,
        enums = enums,
    )
    return py_name + "_py_pyo"
