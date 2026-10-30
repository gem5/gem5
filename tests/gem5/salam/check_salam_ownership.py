# Copyright (c) 2026 Akanksha Chaudhari
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

# Runs inside gem5 and uses the selected profile embedded in the binary.

import copy
import json
import sys

import m5
import m5.objects
from m5 import _salam_profile
from m5._salam_config import install_profile
from m5.objects import (
    FunctionalUnit,
    FunctionalUnits,
    InstConfig,
    InstructionConfig,
    Root,
    SubSystem,
)
from m5.proxy import isproxy

STRING_FIELDS = {
    "alias",
    "int_size",
    "int_sign",
    "fp_size",
    "fp_sign",
    "ptr_size",
    "ptr_sign",
}

BOOL_FIELDS = {"int_apmode", "fp_apmode", "ptr_apmode"}


def parameter_value(name, value):
    if name in STRING_FIELDS:
        return "String", str(value)
    if name in BOOL_FIELDS:
        assert isinstance(value, bool), (name, value)
        return "Bool", value
    return "UInt32", int(value)


def json_value(value):
    return json.loads(json.dumps(value))


def independent_data(first, second):
    assert first == second
    if isinstance(first, dict):
        assert first is not second
        for name in first:
            independent_data(first[name], second[name])
    elif isinstance(first, list):
        assert first is not second
        for left, right in zip(first, second):
            independent_data(left, right)


def check_aliases(container_class, entry_class, vector_name, entries, path):
    aliases = [
        "class",
        "_private",
        "bad-name",
        "path",
        "type",
        "eventq_index",
        "getCCParams",
        "cxx_class_path",
        "cxx_class_name",
        "cxx_namespaces",
        "pybind_class",
        "init",
        "startup",
        "name",
        "from_profile",
        vector_name,
        "any",
        "all",
        "search_self",
        "search_up",
    ]
    if container_class is FunctionalUnits:
        aliases.append("characterization")
    children = dict(container_class._children)
    vector = container_class._values[vector_name]

    valid_alias = "ownership_probe_unused"
    while valid_alias in children:
        valid_alias += "_entry"

    def record(alias):
        parameters = copy.deepcopy(entries[0][1]) if entries else {}
        if entry_class is FunctionalUnit:
            parameters["alias"] = alias
        return (alias, parameters)

    cases = [
        (alias, [record(valid_alias), record(alias)]) for alias in aliases
    ]
    cases.append((valid_alias, [record(valid_alias)] * 2))
    for alias, records in cases:
        try:
            install_profile(
                container_class, entry_class, vector_name, records, path
            )
        except ValueError as error:
            text = str(error)
            assert alias in text and path in text, text
            assert container_class.__name__ in text, text
        else:
            raise AssertionError("accepted colliding alias " + alias)
        assert dict(container_class._children) == children
        assert container_class._values[vector_name] is vector


def check_profile(expected):
    classes = {
        "FunctionalUnit",
        "FunctionalUnits",
        "InstructionConfig",
        "InstConfig",
    }
    modules = {"m5.objects.FunctionalUnits", "m5.objects.InstConfig"}
    registered = {
        name
        for name, cls in vars(m5.objects).items()
        if isinstance(cls, type) and cls.__module__ in modules
    }
    assert registered == classes, (registered, classes)
    obsolete = (
        (
            FunctionalUnit,
            (
                "power_units",
                "energy_units",
                "time_units",
                "area_units",
                "fu_latency",
                "internal_power",
                "switch_power",
                "dynamic_power",
                "dynamic_energy",
                "leakage_power",
                "area",
                "path_delay",
            ),
        ),
        (InstructionConfig, ("runtime_cycles", "functional_unit_limit")),
    )
    for entry_class, fields in obsolete:
        assert not set(fields) & set(entry_class._params.keys())
        for field in fields:
            try:
                entry_class(**{field: 0})
            except AttributeError as error:
                assert field in str(error), str(error)
            else:
                raise AssertionError("accepted obsolete parameter " + field)
    assert _salam_profile.PROFILE_PATH == expected["profile"]
    assert (
        json_value(_salam_profile.FUNCTIONAL_UNITS)
        == expected["functional_units"]
    )
    assert json_value(_salam_profile.INSTRUCTIONS) == expected["instructions"]
    assert _salam_profile.CHARACTERIZATION == expected["characterization"]
    first = FunctionalUnits.characterization()
    second = FunctionalUnits.characterization()
    independent_data(first, second)
    independent_data(first, _salam_profile.CHARACTERIZATION)
    assert first == expected["characterization"]

    collections = (
        (
            FunctionalUnits,
            FunctionalUnit,
            "functional_unit_list",
            "functional_units",
            "limit",
        ),
        (
            InstConfig,
            InstructionConfig,
            "inst_list",
            "instructions",
            "opcode_num",
        ),
    )
    for cls, entry_cls, vector_name, key, _field in collections:
        assert cls._params[vector_name].ptype is entry_cls
        check_aliases(
            cls, entry_cls, vector_name, expected[key], expected["profile"]
        )

    # Native cloning is allowed for a detached hierarchy, with one Root.
    owner = SubSystem()
    owner.functional_units = FunctionalUnits.from_profile()
    owner.instructions = InstConfig.from_profile()
    inherited = {}
    for _cls, _entry_cls, _vector_name, key, field in collections:
        entries = copy.deepcopy(expected[key])
        if entries:
            alias, parameters = entries[0]
            parameters[field] = (int(parameters[field]) + 1) % 2**32
            setattr(
                getattr(getattr(owner, key), alias), field, parameters[field]
            )
        inherited[key] = entries
    cloned = owner()
    root = Root(full_system=False)
    root.original = owner
    root.cloned = cloned
    checks = []
    for cls, entry_cls, vector_name, key, field in collections:
        for suffix, container in (
            ("bare", cls()),
            ("first", cls.from_profile()),
            ("second", cls.from_profile()),
        ):
            setattr(root, key + "_" + suffix, container)
            checks.append((container, expected[key], entry_cls, vector_name))
        container = getattr(owner, key)
        entries = copy.deepcopy(inherited[key])
        if entries:
            alias, parameters = entries[0]
            parameters[field] = (int(parameters[field]) + 1) % 2**32
            previous = getattr(container, alias)
            replacement = entry_cls(**parameters)
            setattr(container, alias, replacement)
            assert not previous.has_parent()
            assert getattr(container, alias) is replacement
        checks.append((container, entries, entry_cls, vector_name))
        checks.append(
            (getattr(cloned, key), inherited[key], entry_cls, vector_name)
        )

    # Resolve only through the normal instantiate path, after every override.
    children = []
    for container, entries, entry_cls, vector_name in checks:
        aliases = [alias for alias, _ in entries]
        assert set(container._children.keys()) == set(aliases)
        if entries:
            assert isproxy(getattr(container, vector_name))
        else:
            assert len(getattr(container, vector_name)) == 0
        for alias, parameters in entries:
            child = getattr(container, alias)
            assert type(child) is entry_cls
            assert child.get_parent() is container
            assert child.path() == container.path() + "." + alias
            assert set(child._params.keys()) == set(parameters) | {
                "eventq_index"
            }
            for name, value in parameters.items():
                param_type, converted = parameter_value(name, value)
                assert child._params[name].ptype.__name__ == param_type
                assert getattr(child, name).config_value() == converted, (
                    child.path(),
                    name,
                    getattr(child, name),
                    converted,
                )
            assert all(child is not other for other in children)
            children.append(child)
    m5.instantiate()
    pointers = []
    for container, entries, _entry_cls, vector_name in checks:
        python_entries = getattr(container, vector_name)
        cpp_entries = getattr(container.getCCParams(), vector_name)
        assert len(python_entries) == len(cpp_entries) == len(entries)
        for index, (alias, parameters) in enumerate(entries):
            child = getattr(container, alias)
            pointer = child.getCCObject()
            assert python_entries[index] is child
            assert cpp_entries[index] is pointer, (container.path(), alias)
            assert all(pointer is not other for other in pointers)
            for name, value in parameters.items():
                _param_type, converted = parameter_value(name, value)
                assert getattr(child.getCCParams(), name) == converted, (
                    child.path(),
                    name,
                    converted,
                )
            pointers.append(pointer)
    assert FunctionalUnits.characterization() == expected["characterization"]
    print("SALAM_PROFILE_OWNERSHIP_PASSED", len(checks), len(pointers))


if __name__ == "__m5_main__":
    check_profile(json.loads(sys.argv[1]))
