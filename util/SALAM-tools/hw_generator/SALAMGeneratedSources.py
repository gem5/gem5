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
import importlib.util
import keyword
import os
import re
from dataclasses import dataclass
from pprint import pformat

import yaml

_power_path = os.path.join(
    os.path.dirname(__file__), "..", "SALAM-Configurator", "power_model.py"
)
_power_spec = importlib.util.spec_from_file_location(
    "salam_power_input", _power_path
)
power_model = importlib.util.module_from_spec(_power_spec)
_power_spec.loader.exec_module(power_model)

_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_SKIP_INSTRUCTIONS = ("any", "none")
# Ownership entries retain SALAM names; characterization uses LLVM opcodes.
_LLVM_OPERATION_NAMES = {
    "and_inst": "and",
    "or_inst": "or",
    "xor_inst": "xor",
    "gep": "getelementptr",
}


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
    initiation_interval: object
    instruction_names: tuple
    characterization: dict
    characterization_path: str | None


@dataclass(frozen=True)
class Instruction:

    name: str
    functional_unit: object
    opcode_num: object


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
        return tuple(
            path
            for unit in self.functional_units
            for path in (unit.path, unit.characterization_path)
            if path
        )

    @property
    def fu_class_names(self):
        return ("FunctionalUnit", "FunctionalUnits")

    @property
    def instruction_names(self):
        return tuple(inst.name for inst in self.instructions)

    @property
    def instruction_class_names(self):
        return ("InstructionConfig", "InstConfig")

    def generated_paths(self):
        return ("_salam_profile.py",)


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
        obsolete = sorted(set(entry) & _OBSOLETE_INSTRUCTION_FIELDS)
        if obsolete:
            raise SourceConfigError(
                spec.instructions
                + ": instructions."
                + name
                + ": obsolete field(s) "
                + ", ".join(obsolete)
                + "; use CycleCounts or workload runtime_cycles for latency"
                + " and limit in the functional-unit profile for instance counts"
            )
        unknown = sorted(set(entry) - set(_INSTRUCTION_FIELDS), key=str)
        if unknown:
            raise SourceConfigError(
                spec.instructions
                + ": instructions."
                + name
                + ": unknown field(s) "
                + ", ".join(str(field) for field in unknown)
            )
        instructions.append(
            Instruction(
                name=name,
                functional_unit=entry.get("functional_unit"),
                opcode_num=_require_field(entry, "opcode_num", name),
            )
        )
    return GeneratedSources(
        spec=spec,
        functional_units=units,
        instructions=tuple(instructions),
        instruction_document=enriched,
    )


_FU_FIELDS = (
    "alias",
    "stages",
    "cycles",
    "enum_value",
    "int_size",
    "int_sign",
    "int_apmode",
    "fp_size",
    "fp_sign",
    "fp_apmode",
    "ptr_size",
    "ptr_sign",
    "ptr_apmode",
    "limit",
    "initiation_interval",
)
_INSTRUCTION_FIELDS = (
    "functional_unit",
    "opcode_num",
)
_OBSOLETE_INSTRUCTION_FIELDS = frozenset(
    ("functional_unit_limit", "runtime_cycles")
)
_OBSOLETE_POWER_FIELDS = frozenset(
    (
        "units",
        "latency",
        "internal_power",
        "switch_power",
        "dynamic_power",
        "dynamic_energy",
        "leakage_power",
        "area",
        "path_delay",
    )
)


def generate_sources(generated):
    """Embed the selected profile as data for the common SimObject types."""
    units = tuple(
        (unit.alias, {field: getattr(unit, field) for field in _FU_FIELDS})
        for unit in generated.functional_units
    )
    instructions = tuple(
        (
            inst.name,
            {field: getattr(inst, field) for field in _INSTRUCTION_FIELDS},
        )
        for inst in generated.instructions
    )
    values = {
        "PROFILE_PATH": (
            generated.spec.config_path or generated.spec.instructions
        ),
        "FUNCTIONAL_UNITS": units,
        "INSTRUCTIONS": instructions,
        "CHARACTERIZATION": power_model.merge(
            unit.characterization for unit in generated.functional_units
        ),
    }
    text = "# Generated SALAM profile data. Do not edit.\n\n"
    for name, value in values.items():
        text += name + " = " + pformat(value, sort_dicts=False, width=75)
        text += "\n\n"
    return {"_salam_profile.py": text}


def write_content_files(root, outputs):
    for rel, text in outputs.items():
        _write_if_changed(_output_path(root, rel), text)


def generate_sources_action(target, source, env):
    spec = load_source_config(env["SALAM_HW_CONFIG"])
    model = load_generated_sources(spec)
    outputs = generate_sources(model)
    marker = os.sep + "generated" + os.sep + "salam" + os.sep
    written = {}
    for node in target:
        abspath = os.path.abspath(str(node))
        idx = abspath.rfind(marker)
        if idx < 0:
            raise SourceConfigError(
                "generated target is outside generated/salam: " + abspath
            )
        written[abspath[idx + len(marker) :]] = abspath
    if set(written) != set(outputs):
        raise SourceConfigError(
            "generated targets do not match generated outputs"
        )
    for rel, text in outputs.items():
        _write_if_changed(written[rel], text)


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
        unit = document["functional_unit"]
        params = unit["parameters"]
        power = unit.get("power_model", {})
    except (TypeError, KeyError, AttributeError) as exc:
        raise SourceConfigError("malformed FU YAML: " + path) from exc
    if not isinstance(params, dict):
        raise SourceConfigError(
            "malformed functional_unit.parameters in " + path
        )
    if not isinstance(power, dict):
        raise SourceConfigError(
            "functional_unit.power_model must be a mapping in " + path
        )
    obsolete = sorted(set(power) & _OBSOLETE_POWER_FIELDS)
    if obsolete:
        raise SourceConfigError(
            path
            + ": functional_unit.power_model: obsolete field(s) "
            + ", ".join(obsolete)
            + "; remove legacy values and use power_model.characterization"
            + " for offline PPA records"
        )
    unknown_power = sorted(set(power) - {"characterization"}, key=str)
    if unknown_power:
        raise SourceConfigError(
            path
            + ": functional_unit.power_model: unknown field(s) "
            + ", ".join(str(field) for field in unknown_power)
        )
    declared = params.get("alias")
    if declared != alias:
        raise SourceConfigError(
            "FU alias " + str(declared) + " does not match " + path
        )
    unknown = sorted(set(params) - _FU_PARAMETER_KEYS)
    if unknown:
        raise SourceConfigError(
            "unknown functional unit parameter "
            + ", ".join(str(key) for key in unknown)
            + " in "
            + path
        )
    for field in ("stages", "cycles", "limit"):
        _require_uint32(params, field, path, minimum=0)
    _require_uint32(params, "enum_value", path, minimum=1)
    # An omitted initiation_interval is 0: not pipelined.
    initiation_interval = 0
    if "initiation_interval" in params:
        initiation_interval = _require_uint32(
            params, "initiation_interval", path, minimum=1
        )
    instruction_names = params.get("instructions") or []
    if not isinstance(instruction_names, list):
        raise SourceConfigError("malformed instructions in " + path)
    characterization = power_model.normalize({"version": 1, "records": []})
    characterization_path = None
    if "characterization" in power:
        reference = power["characterization"]
        if not isinstance(reference, str) or not reference.strip():
            raise SourceConfigError(
                "invalid characterization reference: " + path
            )
        characterization_path = os.path.abspath(
            os.path.join(os.path.dirname(path), reference)
        )
        try:
            characterization = power_model.load(characterization_path)
        except (OSError, power_model.CharacterizationError) as exc:
            raise SourceConfigError(str(exc)) from exc
        operation_names = {
            _LLVM_OPERATION_NAMES.get(name, name) for name in instruction_names
        }
        for binding in characterization["keys"]:
            if binding.startswith("fu:") and binding != "fu:" + alias:
                raise SourceConfigError(
                    "characterization FU alias mismatch: " + binding
                )
            if binding.startswith("op:"):
                _, unit_alias, operation, _ = binding.split(":", 3)
                if unit_alias != alias or operation not in operation_names:
                    raise SourceConfigError(
                        "characterization operation mismatch: " + binding
                    )
            if binding == "memory":
                raise SourceConfigError(
                    "memory characterization is not a FU input"
                )
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
        initiation_interval=initiation_interval,
        instruction_names=tuple(instruction_names),
        characterization=characterization,
        characterization_path=characterization_path,
    )


_FU_PARAMETER_KEYS = frozenset(
    (
        "alias",
        "stages",
        "cycles",
        "enum_value",
        "datatypes",
        "instructions",
        "limit",
        "initiation_interval",
    )
)


def _require_uint32(params, field, path, minimum):
    # A required functional-unit field that fits a Param.UInt32.
    if field not in params:
        raise SourceConfigError("missing " + field + " in " + path)
    value = params[field]
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= 2**32 - 1
    ):
        raise SourceConfigError(
            field
            + " must be an integer from "
            + str(minimum)
            + " to 4294967295 in "
            + path
            + ", not "
            + repr(value)
        )
    return value


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
    if (
        not isinstance(name, str)
        or not _NAME.fullmatch(name)
        or name.startswith("_")
        or keyword.iskeyword(name)
    ):
        raise SourceConfigError("invalid name " + str(name) + " in " + where)


def _require_field(entry, field, name):
    if field not in entry:
        raise SourceConfigError(
            "missing " + field + " for instruction " + name
        )
    return entry[field]


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
