# Copyright (c) 2026 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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


import copy
import os
import re
from dataclasses import dataclass

import yaml
from SALAMClassGenerator import (
    generate_fu_base_header,
    generate_fu_header,
    generate_fu_list_header,
    generate_fu_list_source,
    generate_fu_source,
    generate_functional_unit_declaration,
    generate_functional_units_declaration,
    generate_instconfig_declaration,
    generate_instruction_base_header,
    generate_instruction_config_header,
    generate_instruction_config_preamble,
    generate_instruction_config_source,
    generate_instruction_declaration,
    generate_instruction_unit_header,
    generate_instruction_unit_source,
    simobject_classname,
)

_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_SKIP_INSTRUCTIONS = ("any", "none")
# Members named by HWInterface::availableFunctionalUnit and
# clearFunctionalUnit. A smaller hardware profile does not require
# them; a build that compiles that consumer does.
REQUIRED_FU_ALIASES = (
    "integer_adder",
    "integer_multiplier",
    "bit_shifter",
    "bitwise_operations",
    "float_adder",
    "double_adder",
    "float_multiplier",
    "float_divider",
    "double_multiplier",
    "double_divider",
    "bit_register",
)


class SourceConfigError(Exception):
    pass


@dataclass(frozen=True)
class SourceInputs:

    functional_units: str
    functional_unit_order: tuple
    instructions: str
    config_path: str | None = None


@dataclass(frozen=True)
class FunctionalUnit:

    alias: str
    path: str
    stages: object
    cycles: object
    enum_value: object
    int_size: object
    int_sign: object
    int_apmode: object
    fp_size: object
    fp_sign: object
    fp_apmode: object
    ptr_size: object
    ptr_sign: object
    ptr_apmode: object
    limit: object
    instruction_names: tuple
    power_units: object
    energy_units: object
    time_units: object
    area_units: object
    fu_latency: object
    internal_power: object
    switch_power: object
    dynamic_power: object
    dynamic_energy: object
    leakage_power: object
    area: object
    path_delay: object

    @property
    def class_name(self):
        return simobject_classname(self.alias)


@dataclass(frozen=True)
class Instruction:

    name: str
    functional_unit: object
    functional_unit_limit: object
    opcode_num: object
    runtime_cycles: object

    @property
    def class_name(self):
        return simobject_classname(self.name)


@dataclass(frozen=True)
class GeneratedSources:

    spec: SourceInputs
    functional_units: tuple
    instructions: tuple
    instruction_document: dict

    @property
    def fu_aliases(self):
        return tuple(unit.alias for unit in self.functional_units)

    @property
    def fu_input_paths(self):
        return tuple(unit.path for unit in self.functional_units)

    @property
    def fu_class_names(self):
        return ("FunctionalUnits",) + tuple(
            unit.class_name for unit in self.functional_units
        )

    @property
    def instruction_names(self):
        return tuple(inst.name for inst in self.instructions)

    @property
    def instruction_class_names(self):
        return ("InstConfig",) + tuple(
            inst.class_name for inst in self.instructions
        )

    def generated_paths(self):
        return tuple(
            _content_path_list(self.functional_units, self.instructions)
        )


def load_source_config(path):
    if not path or not os.path.isfile(path):
        raise SourceConfigError("missing SALAM hardware config: " + str(path))
    abs_path = os.path.abspath(path)
    with open(abs_path, encoding="utf-8") as handle:
        text = handle.read()
    document = _load_yaml(text, abs_path)
    if not isinstance(document, dict):
        raise SourceConfigError(
            "SALAM hardware config is not a mapping: " + abs_path
        )
    allowed = {"functional_units", "functional_unit_order", "instructions"}
    unknown = sorted(set(document) - allowed)
    if unknown:
        raise SourceConfigError(
            "unknown field " + ", ".join(unknown) + " in " + abs_path
        )
    for key in ("functional_units", "instructions"):
        if key not in document or document[key] in (None, ""):
            raise SourceConfigError("missing " + key + " in " + abs_path)
        if not isinstance(document[key], str):
            raise SourceConfigError("malformed " + key + " in " + abs_path)
    if "functional_unit_order" not in document:
        raise SourceConfigError("missing functional_unit_order in " + abs_path)
    order = document["functional_unit_order"]
    if not isinstance(order, list):
        raise SourceConfigError(
            "functional_unit_order must be a list in " + abs_path
        )
    seen = set()
    clean_order = []
    for item in order:
        if not isinstance(item, str):
            raise SourceConfigError(
                "non-string functional_unit_order entry in " + abs_path
            )
        _check_name(item, abs_path)
        if item in seen:
            raise SourceConfigError(
                "duplicate functional_unit_order entry: " + item
            )
        seen.add(item)
        clean_order.append(item)
    base = os.path.dirname(abs_path)
    return SourceInputs(
        functional_units=_resolve_config_path(
            base, document["functional_units"], abs_path, "functional_units"
        ),
        functional_unit_order=tuple(clean_order),
        instructions=_resolve_config_path(
            base, document["instructions"], abs_path, "instructions"
        ),
        config_path=abs_path,
    )


def load_generated_sources(spec):
    fu_dir = spec.functional_units
    if not os.path.isdir(fu_dir):
        raise SourceConfigError("missing FU directory: " + fu_dir)
    if not os.path.isfile(spec.instructions):
        raise SourceConfigError(
            "missing instruction list: " + spec.instructions
        )
    _validate_unit_order(fu_dir, spec.functional_unit_order)
    units = tuple(
        _load_functional_unit(fu_dir, alias)
        for alias in spec.functional_unit_order
    )
    _reject_enum_conflicts(units)
    _reject_instruction_conflicts(units)
    with open(spec.instructions, encoding="utf-8") as handle:
        text = handle.read()
    document = _load_yaml(text, spec.instructions)
    if not isinstance(document, dict) or "instructions" not in document:
        raise SourceConfigError("missing instructions in " + spec.instructions)
    raw = document["instructions"]
    if not isinstance(raw, dict):
        raise SourceConfigError(
            "malformed instructions in " + spec.instructions
        )
    enriched = _enrich(document, units, spec.instructions)
    instructions = []
    for name, entry in enriched["instructions"].items():
        _check_name(name, spec.instructions)
        if not isinstance(entry, dict):
            raise SourceConfigError(
                "malformed instruction " + name + " in " + spec.instructions
            )
        instructions.append(
            Instruction(
                name=name,
                functional_unit=entry.get("functional_unit"),
                functional_unit_limit=entry.get("functional_unit_limit", 0),
                opcode_num=_require_field(entry, "opcode_num", name),
                runtime_cycles=_require_field(entry, "runtime_cycles", name),
            )
        )
    _reject_reserved_outputs(units, tuple(instructions))
    _validate_generated_set(units, tuple(instructions))
    return GeneratedSources(
        spec=spec,
        functional_units=units,
        instructions=tuple(instructions),
        instruction_document=enriched,
    )


def generate_sources(generated):
    aliases = list(generated.fu_aliases)
    names = list(generated.instruction_names)
    pairs = [
        ("FunctionalUnits.py", _generate_functional_units_py(generated)),
        ("InstConfig.py", _generate_inst_config_py(generated)),
        ("HWModeling/functional_units.hh", generate_fu_list_header(aliases)),
        ("HWModeling/functional_units.cc", generate_fu_list_source(aliases)),
        ("HWModeling/functional_units/base.hh", generate_fu_base_header()),
        (
            "HWModeling/instruction_config.hh",
            generate_instruction_config_header(names),
        ),
        (
            "HWModeling/instruction_config.cc",
            generate_instruction_config_source(names),
        ),
        (
            "HWModeling/instructions/base.hh",
            generate_instruction_base_header(),
        ),
    ]
    for unit in generated.functional_units:
        base = "HWModeling/functional_units/" + unit.alias
        pairs.append(
            (base + ".hh", generate_fu_header(unit.alias, unit.class_name))
        )
        pairs.append(
            (base + ".cc", generate_fu_source(unit.alias, unit.class_name))
        )
    for inst in generated.instructions:
        base = "HWModeling/instructions/" + inst.name
        pairs.append(
            (
                base + ".hh",
                generate_instruction_unit_header(inst.name, inst.class_name),
            )
        )
        pairs.append(
            (
                base + ".cc",
                generate_instruction_unit_source(inst.name, inst.class_name),
            )
        )
    paths = [path for path, _text in pairs]
    _reject_duplicate_values(paths, "output path")
    declared = list(generated.generated_paths())
    _reject_duplicate_values(declared, "output path")
    if set(paths) != set(declared):
        raise SourceConfigError(
            "generated content keys do not match the declared "
            "content target set"
        )
    return dict(pairs)


def write_content_files(root, outputs):
    for rel, text in outputs.items():
        _write_if_changed(_output_path(root, rel), text)


def require_compatible_functional_units(model):
    present = set(model.fu_aliases)
    missing = [name for name in REQUIRED_FU_ALIASES if name not in present]
    if missing:
        raise SourceConfigError(
            "hardware profile is missing functional units required by "
            "HWInterface: " + ", ".join(missing)
        )


def _discovered_fu_aliases(fu_dir):
    names = []
    for name in sorted(os.listdir(fu_dir)):
        if name.startswith("."):
            continue
        path = os.path.join(fu_dir, name)
        if os.path.islink(path):
            raise SourceConfigError(
                "functional unit path is a symlink: " + path
            )
        if os.path.isdir(path):
            names.append(name)
    return tuple(names)


def _validate_unit_order(fu_dir, order):
    discovered = _discovered_fu_aliases(fu_dir)
    present = set(discovered)
    listed = set(order)
    extra = [name for name in order if name not in present]
    missing = [name for name in discovered if name not in listed]
    if extra:
        raise SourceConfigError(
            "extra functional_unit_order entry: " + ", ".join(extra)
        )
    if missing:
        raise SourceConfigError(
            "missing functional_unit_order entry: " + ", ".join(missing)
        )


def _load_functional_unit(fu_dir, alias):
    _check_name(alias, fu_dir)
    path = os.path.join(fu_dir, alias, alias + ".yml")
    if not os.path.isfile(path):
        raise SourceConfigError("missing FU YAML: " + path)
    with open(path, encoding="utf-8") as handle:
        document = _load_yaml(handle.read(), path)
    try:
        params = document["functional_unit"]["parameters"]
        power = document["functional_unit"]["power_model"]
        units = power["units"]
    except (TypeError, KeyError) as exc:
        raise SourceConfigError("malformed FU YAML: " + path) from exc
    declared = params.get("alias")
    if declared != alias:
        raise SourceConfigError(
            "FU alias " + str(declared) + " does not match " + path
        )
    instruction_names = params.get("instructions") or []
    if not isinstance(instruction_names, list):
        raise SourceConfigError("malformed instructions in " + path)
    return FunctionalUnit(
        alias=alias,
        path=os.path.abspath(path),
        stages=params.get("stages"),
        cycles=params.get("cycles"),
        enum_value=params.get("enum_value"),
        int_size=_datatype(params, "integer", "size"),
        int_sign=_datatype(params, "integer", "sign"),
        int_apmode=_datatype(params, "integer", "APMode"),
        fp_size=_datatype(params, "floating_point", "size"),
        fp_sign=_datatype(params, "floating_point", "sign"),
        fp_apmode=_datatype(params, "floating_point", "APMode"),
        ptr_size=_datatype(params, "pointer", "size"),
        ptr_sign=_datatype(params, "pointer", "sign"),
        ptr_apmode=_datatype(params, "pointer", "APMode"),
        limit=params.get("limit"),
        instruction_names=tuple(instruction_names),
        power_units=units.get("power"),
        energy_units=units.get("energy"),
        time_units=units.get("time"),
        area_units=units.get("area"),
        fu_latency=power.get("latency"),
        internal_power=power.get("internal_power"),
        switch_power=power.get("switch_power"),
        dynamic_power=power.get("dynamic_power"),
        dynamic_energy=power.get("dynamic_energy"),
        leakage_power=power.get("leakage_power"),
        area=power.get("area"),
        path_delay=power.get("path_delay"),
    )


def _datatype(params, group, field):
    try:
        return params["datatypes"][group][field]
    except (TypeError, KeyError) as exc:
        raise SourceConfigError(
            "missing datatypes." + group + "." + field
        ) from exc


def _enrich(document, units, inst_path):
    enriched = copy.deepcopy(document)
    instructions = enriched.get("instructions")
    if not isinstance(instructions, dict):
        raise SourceConfigError("malformed instructions in " + inst_path)
    for unit in units:
        for name in unit.instruction_names:
            if name in _SKIP_INSTRUCTIONS:
                continue
            _check_name(str(name), unit.path)
            if name not in instructions:
                raise SourceConfigError(
                    "instruction "
                    + str(name)
                    + " from "
                    + unit.path
                    + " is missing in "
                    + inst_path
                )
            entry = instructions[name]
            if not isinstance(entry, dict):
                raise SourceConfigError(
                    "malformed instruction " + str(name) + " in " + inst_path
                )
            entry["functional_unit"] = unit.enum_value
    return enriched


def _generate_functional_units_py(model):
    text = generate_functional_units_declaration(
        list(model.fu_aliases),
        "salam/HWModeling/functional_units.hh",
    )
    parts = [text]
    for unit in model.functional_units:
        parts.append(
            generate_functional_unit_declaration(
                unit.alias,
                unit.class_name,
                "salam/HWModeling/functional_units/" + unit.alias + ".hh",
                stages=unit.stages,
                cycles=unit.cycles,
                enum_value=unit.enum_value,
                int_size=unit.int_size,
                int_sign=unit.int_sign,
                int_apmode=unit.int_apmode,
                fp_size=unit.fp_size,
                fp_sign=unit.fp_sign,
                fp_apmode=unit.fp_apmode,
                ptr_size=unit.ptr_size,
                ptr_sign=unit.ptr_sign,
                ptr_apmode=unit.ptr_apmode,
                limit=unit.limit,
                power_units=unit.power_units,
                energy_units=unit.energy_units,
                time_units=unit.time_units,
                area_units=unit.area_units,
                fu_latency=unit.fu_latency,
                internal_power=unit.internal_power,
                switch_power=unit.switch_power,
                dynamic_power=unit.dynamic_power,
                dynamic_energy=unit.dynamic_energy,
                leakage_power=unit.leakage_power,
                area=unit.area,
                path_delay=unit.path_delay,
            )
        )
    return "".join(parts)


def _generate_inst_config_py(model):
    parts = [generate_instruction_config_preamble()]
    for inst in model.instructions:
        parts.append(
            generate_instruction_declaration(
                inst.name,
                functional_unit=inst.functional_unit,
                functional_unit_limit=inst.functional_unit_limit,
                opcode_num=inst.opcode_num,
                runtime_cycles=inst.runtime_cycles,
            )
        )
    parts.append(generate_instconfig_declaration(model.instruction_names))
    return "".join(parts)


def _resolve_config_path(base, relative, config, key):
    if not isinstance(relative, str):
        raise SourceConfigError("malformed " + key + " in " + config)
    if os.path.isabs(relative) or ".." in relative.split("/"):
        raise SourceConfigError(
            "unsafe " + key + " path in " + config + ": " + relative
        )
    return os.path.abspath(os.path.join(base, relative))


def _load_yaml(text, path):
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise SourceConfigError("malformed YAML: " + path) from exc
    return document


def _check_name(name, where):
    if not isinstance(name, str) or not _NAME.fullmatch(name):
        raise SourceConfigError("invalid name " + str(name) + " in " + where)


def _require_field(entry, field, name):
    if field not in entry:
        raise SourceConfigError(
            "missing " + field + " for instruction " + name
        )
    return entry[field]


def _reject_reserved_outputs(units, instructions):
    for unit in units:
        if unit.alias == "base":
            raise SourceConfigError(
                "functional unit alias base collides with "
                "functional_units/base.hh"
            )
    for inst in instructions:
        if inst.name == "base":
            raise SourceConfigError(
                "instruction name base collides with instructions/base.hh"
            )


def _content_path_list(units, instructions):
    paths = [
        "FunctionalUnits.py",
        "InstConfig.py",
        "HWModeling/functional_units.hh",
        "HWModeling/functional_units.cc",
        "HWModeling/functional_units/base.hh",
    ]
    for unit in units:
        paths.append("HWModeling/functional_units/" + unit.alias + ".hh")
        paths.append("HWModeling/functional_units/" + unit.alias + ".cc")
    paths.extend(
        [
            "HWModeling/instruction_config.hh",
            "HWModeling/instruction_config.cc",
            "HWModeling/instructions/base.hh",
        ]
    )
    for inst in instructions:
        paths.append("HWModeling/instructions/" + inst.name + ".hh")
        paths.append("HWModeling/instructions/" + inst.name + ".cc")
    return paths


def _reject_enum_conflicts(units):
    seen = {}
    for unit in units:
        previous = seen.get(unit.enum_value)
        if previous is not None:
            raise SourceConfigError(
                "duplicate enum_value "
                + str(unit.enum_value)
                + " for "
                + previous
                + " and "
                + unit.alias
            )
        seen[unit.enum_value] = unit.alias


def _reject_instruction_conflicts(units):
    owners = {}
    for unit in units:
        for name in unit.instruction_names:
            if name in _SKIP_INSTRUCTIONS:
                continue
            previous = owners.get(name)
            if previous is not None:
                raise SourceConfigError(
                    "instruction "
                    + str(name)
                    + " is claimed by "
                    + previous
                    + " and "
                    + unit.alias
                )
            owners[name] = unit.alias


def _reject_duplicate_values(values, label):
    seen = set()
    for value in values:
        if value in seen:
            raise SourceConfigError("duplicate " + label + ": " + str(value))
        seen.add(value)


def _validate_generated_set(units, instructions):
    classes = ["FunctionalUnits", "InstConfig"]
    classes.extend(unit.class_name for unit in units)
    classes.extend(inst.class_name for inst in instructions)
    _reject_duplicate_values(classes, "SimObject class")
    _reject_duplicate_values(
        _content_path_list(units, instructions), "output path"
    )


def _output_path(root, relative):
    if os.path.isabs(relative) or ".." in relative.split("/"):
        raise SourceConfigError("unsafe output path: " + relative)
    path = os.path.abspath(os.path.join(root, relative))
    root_abs = os.path.abspath(root)
    if os.path.commonpath([root_abs, path]) != root_abs:
        raise SourceConfigError("unsafe output path: " + relative)
    return path


def _write_if_changed(path, text):
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as handle:
            if handle.read() == text:
                return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)
