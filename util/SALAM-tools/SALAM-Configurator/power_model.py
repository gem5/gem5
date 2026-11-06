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

import hashlib
import json
import math
import os
import re

import yaml


class CharacterizationError(ValueError):
    pass


# These are input conversions, not technology or width scaling factors.
_UNITS = {
    "area": {"um2": 1.0, "mm2": 1e6},
    "leakage": {"W": 1.0, "mW": 1e-3, "uW": 1e-6},
    "event": {"J": 1.0, "nJ": 1e-9, "pJ": 1e-12},
    "read": {"J": 1.0, "nJ": 1e-9, "pJ": 1e-12},
    "write": {"J": 1.0, "nJ": 1e-9, "pJ": 1e-12},
}
_VALUES = {
    "area": "area_um2",
    "leakage": "leakage_w",
    "event": "event_j",
    "read": "read_j",
    "write": "write_j",
}
_STATUSES = {"unavailable", "characterized", "derived", "proxy"}
_TIMING_FIELDS = (
    "timing_available",
    "minimum_latency_s",
    "minimum_issue_interval_s",
    "pipelined",
)
_SOURCE_KEYS = {
    "url",
    "revision",
    "path",
    "row",
    "sha256",
    "library",
    "technology_nm",
    "voltage_v",
    "temperature_c",
    "reference_interval_s",
    "activity",
    "datatype",
    "implementation",
    "conversion",
    "notes",
    "proxy_for",
    "synthetic",
}
_GEOMETRY_KEYS = {
    "capacity_bytes",
    "word_bytes",
    "banks",
    "read_ports",
    "write_ports",
    "rw_ports",
    "mask_model",
    "minimum_access_interval_s",
    "minimum_access_latency_s",
    "storage_physical",
    "storage_scope",
}
_TYPE = r"(?:[ifp][1-9][0-9]*|v)"
_SIGNATURE = re.compile(rf"^(?:{_TYPE}(?:,{_TYPE})*)?->{_TYPE}$")
_NAME = r"[A-Za-z_][A-Za-z0-9_]*"
_FU_KEY = re.compile(rf"^fu:{_NAME}$")
_OP_KEY = re.compile(rf"^op:({_NAME}):({_NAME}):(.+)$")
_REG_KEY = re.compile(r"^register:(bit|[ifp][1-9][0-9]*)$")


class _UniqueLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str) or key in result:
            raise CharacterizationError(f"invalid or duplicate key {key!r}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping
)


def _keys(value, allowed, scope):
    if not isinstance(value, dict):
        raise CharacterizationError(f"{scope}: expected a mapping")
    unknown = set(value) - allowed
    if unknown:
        raise CharacterizationError(
            f"{scope}: unknown fields {sorted(unknown)}"
        )


def _number(value, scope):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CharacterizationError(f"{scope}: expected a number")
    if not math.isfinite(value) or value < 0:
        raise CharacterizationError(
            f"{scope}: expected finite nonnegative value"
        )
    return float(value)


def _positive_integer(value, scope, zero=False):
    if type(value) is not int or value < (0 if zero else 1):
        raise CharacterizationError(f"{scope}: invalid integer {value!r}")
    return value


def _quantity(record, kind):
    if kind not in record:
        return 0.0, "unavailable"
    quantity = record[kind]
    _keys(quantity, {"value", "unit", "status"}, kind)
    status = quantity.get("status")
    if status not in _STATUSES:
        raise CharacterizationError(f"{kind}: invalid coverage status")
    if status == "unavailable":
        if "value" in quantity or "unit" in quantity:
            raise CharacterizationError(
                f"{kind}: unavailable is not numeric zero"
            )
        return 0.0, status
    if quantity.get("unit") not in _UNITS[kind]:
        raise CharacterizationError(f"{kind}: unsupported or absent unit")
    value = _number(quantity.get("value"), kind)
    normalized = value * _UNITS[kind][quantity["unit"]]
    if not math.isfinite(normalized):
        raise CharacterizationError(f"{kind}: conversion overflow")
    return normalized, status


def _type_supported(type_name):
    if type_name in {"v", "f32", "f64", "i1", "i8", "i16", "i32", "i64"}:
        return True
    # Pointer precision is supplied by the module's DataLayout. Execution
    # uses a maximum 64-bit pointer representation.
    return type_name.startswith("p") and 0 < int(type_name[1:]) <= 64


def _signature_types(signature):
    operands, result = signature.split("->")
    operands = operands.split(",") if operands else []
    if not all(_type_supported(t) for t in [*operands, result]):
        raise CharacterizationError(
            f"unsupported scalar signature {signature}"
        )
    if "v" in operands:
        raise CharacterizationError("void is not an operand type")
    return operands, result


def _operation_signature(operation, signature):
    operands, result = _signature_types(signature)
    integer = {
        "add",
        "sub",
        "mul",
        "udiv",
        "sdiv",
        "urem",
        "srem",
        "shl",
        "lshr",
        "ashr",
        "and",
        "or",
        "xor",
    }
    floating = {"fadd", "fsub", "fmul", "fdiv", "frem"}
    if operation in integer | floating:
        kind = "i" if operation in integer else "f"
        valid = (
            len(operands) == 2
            and operands[0] == operands[1] == result
            and result.startswith(kind)
        )
    elif operation in {"icmp", "fcmp"}:
        valid = (
            len(operands) == 2
            and operands[0] == operands[1]
            and result == "i1"
            and operands[0][0] in ("ip" if operation == "icmp" else "f")
        )
    elif operation in {"trunc", "zext", "sext", "fptrunc", "fpext"}:
        kind = "f" if operation.startswith("fp") else "i"
        valid = (
            len(operands) == 1
            and operands[0].startswith(kind)
            and result.startswith(kind)
        )
        if valid:
            source_width, target_width = int(operands[0][1:]), int(result[1:])
            valid = (
                source_width > target_width
                if "trunc" in operation
                else source_width < target_width
            )
    elif operation in {
        "fptoui",
        "fptosi",
        "uitofp",
        "sitofp",
        "ptrtoint",
        "inttoptr",
    }:
        source_kind, target_kind = {
            "fptoui": ("f", "i"),
            "fptosi": ("f", "i"),
            "uitofp": ("i", "f"),
            "sitofp": ("i", "f"),
            "ptrtoint": ("p", "i"),
            "inttoptr": ("i", "p"),
        }[operation]
        valid = (
            len(operands) == 1
            and operands[0].startswith(source_kind)
            and result.startswith(target_kind)
        )
    elif operation == "bitcast":
        valid = (
            len(operands) == 1
            and result != "v"
            and operands[0].startswith("p") == result.startswith("p")
            and int(operands[0][1:]) == int(result[1:])
        )
    elif operation == "getelementptr":
        valid = (
            bool(operands)
            and operands[0].startswith("p")
            and result == operands[0]
            and all(t.startswith("i") for t in operands[1:])
        )
    elif operation == "select":
        valid = (
            len(operands) == 3
            and operands[0] == "i1"
            and operands[1] == operands[2] == result
        )
    else:
        raise CharacterizationError(
            f"unsupported FU operation binding {operation}"
        )
    if not valid:
        raise CharacterizationError(
            f"incompatible operation/signature {operation}:{signature}"
        )


def _constraints(record, key):
    if "constraints" not in record:
        # These inert values are not inferred physical timing or pipeline data.
        return False, 0.0, 0.0, False
    if not key.startswith("fu:"):
        raise CharacterizationError("constraints only belong to physical FUs")
    constraints = record["constraints"]
    fields = {"minimum_latency_s", "minimum_issue_interval_s", "pipelined"}
    _keys(constraints, fields, key + " constraints")
    if set(constraints) != fields:
        raise CharacterizationError("all physical FU constraints are required")
    if type(constraints["pipelined"]) is not bool:
        raise CharacterizationError("physical FU pipelined must be boolean")
    return (
        True,
        _number(constraints["minimum_latency_s"], "minimum_latency_s"),
        _number(
            constraints["minimum_issue_interval_s"], "minimum_issue_interval_s"
        ),
        constraints["pipelined"],
    )


def normalize(document, origin="<characterization>"):
    _keys(document, {"version", "records", "geometry"}, origin)
    if type(document.get("version")) is not int or document["version"] != 1:
        raise CharacterizationError(f"{origin}: version must be 1")
    records = document.get("records")
    if not isinstance(records, list):
        raise CharacterizationError(f"{origin}: records must be a list")
    result = {
        name: []
        for name in ("keys", "ids", "sources", "signatures", *_TIMING_FIELDS)
    }
    for kind, field in _VALUES.items():
        result[field] = []
        result[kind + "_status"] = []
    seen = set()
    for record in records:
        _keys(
            record,
            {"key", "id", "source", "signatures", "constraints", *_VALUES},
            origin,
        )
        key = record.get("key")
        if not isinstance(key, str) or key in seen:
            raise CharacterizationError(
                f"{origin}: duplicate/invalid binding {key!r}"
            )
        seen.add(key)
        op = _OP_KEY.fullmatch(key)
        if not (
            _FU_KEY.fullmatch(key)
            or _REG_KEY.fullmatch(key)
            or key == "memory"
            or (op and _SIGNATURE.fullmatch(op[3]))
        ):
            raise CharacterizationError(f"{origin}: invalid binding {key!r}")
        if op:
            _operation_signature(op[2], op[3])
        register_match = _REG_KEY.fullmatch(key)
        if (
            register_match
            and register_match[1] != "bit"
            and not _type_supported(register_match[1])
        ):
            raise CharacterizationError(f"unsupported register binding {key}")
        identity = record.get("id")
        if not isinstance(identity, str) or not identity.strip():
            raise CharacterizationError(
                f"{key}: nonempty implementation id required"
            )
        source = record.get("source")
        _keys(source, _SOURCE_KEYS, key + " source")
        for required in (
            "revision",
            "path",
            "row",
            "library",
            "datatype",
            "implementation",
        ):
            if (
                not isinstance(source.get(required), str)
                or not source[required]
            ):
                raise CharacterizationError(
                    f"{key}: source {required} required"
                )
        if not re.fullmatch(r"[0-9a-f]{64}", str(source.get("sha256", ""))):
            raise CharacterizationError(f"{key}: source sha256 required")
        for field in ("technology_nm", "voltage_v", "reference_interval_s"):
            if source.get(field) is not None:
                _number(source[field], "source " + field)
        if source.get("temperature_c") is not None:
            t = source["temperature_c"]
            if (
                isinstance(t, bool)
                or not isinstance(t, (int, float))
                or not math.isfinite(t)
            ):
                raise CharacterizationError(f"{key}: invalid temperature")
        if "synthetic" in source and type(source["synthetic"]) is not bool:
            raise CharacterizationError(f"{key}: synthetic must be boolean")
        signatures = record.get("signatures", [])
        if (
            not isinstance(signatures, list)
            or any(
                not isinstance(s, str) or not _SIGNATURE.fullmatch(s)
                for s in signatures
            )
            or len(set(signatures)) != len(signatures)
        ):
            raise CharacterizationError(f"{key}: invalid/duplicate signatures")
        for signature in signatures:
            _signature_types(signature)
        if key.startswith("fu:") and not signatures:
            raise CharacterizationError(
                f"{key}: physical implementation signatures required"
            )
        if not key.startswith("fu:") and signatures:
            raise CharacterizationError(
                f"{key}: signatures only belong to physical FUs"
            )
        for kind, field in _VALUES.items():
            value, status = _quantity(record, kind)
            if status == "proxy" and not source.get("proxy_for"):
                raise CharacterizationError(
                    f"{key}: proxy_for explanation required"
                )
            if status == "derived" and not source.get("conversion"):
                raise CharacterizationError(
                    f"{key}: derived conversion required"
                )
            if (
                key.startswith("op:")
                and kind != "event"
                and status != "unavailable"
            ):
                raise CharacterizationError(
                    f"{key}: operation record only supplies event energy"
                )
            if (
                key.startswith("fu:")
                and kind not in ("area", "leakage")
                and status != "unavailable"
            ):
                raise CharacterizationError(
                    f"{key}: physical record only supplies area/leakage"
                )
            result[field].append(value)
            result[kind + "_status"].append(status)
        for field, value in zip(_TIMING_FIELDS, _constraints(record, key)):
            result[field].append(value)
        provenance = dict(source)
        provenance["declaring_record"] = origin
        result["keys"].append(key)
        result["ids"].append(identity)
        result["sources"].append(
            json.dumps(provenance, sort_keys=True, separators=(",", ":"))
        )
        result["signatures"].append(";".join(signatures))
    by_key = {key: i for i, key in enumerate(result["keys"])}
    for key, i in by_key.items():
        op = _OP_KEY.fullmatch(key)
        if not op:
            continue
        physical = by_key.get("fu:" + op[1])
        if physical is None or result["ids"][physical] != result["ids"][i]:
            raise CharacterizationError(
                f"{key}: matching physical implementation required"
            )
        if op[3] not in result["signatures"][physical].split(";"):
            raise CharacterizationError(
                f"{key}: signature incompatible with physical implementation"
            )
    if "geometry" in document:
        geometry = document["geometry"]
        _keys(geometry, _GEOMETRY_KEYS, "geometry")
        if "memory" not in seen or len(seen) != 1:
            raise CharacterizationError(
                "memory geometry requires exactly one memory record"
            )
        for field in ("capacity_bytes", "word_bytes", "banks"):
            _positive_integer(geometry.get(field), field)
        for field in ("read_ports", "write_ports", "rw_ports"):
            _positive_integer(geometry.get(field), field, zero=True)
        if not sum(
            geometry[x] for x in ("read_ports", "write_ports", "rw_ports")
        ):
            raise CharacterizationError(
                "memory must have a characterized port"
            )
        if geometry.get("mask_model") not in {
            "byte_enable",
            "read_modify_write",
            "unsupported",
        }:
            raise CharacterizationError("memory mask_model required")
        if geometry.get("storage_scope") not in {
            "whole_device",
            "storage_cells",
        }:
            raise CharacterizationError(
                "explicit memory storage_scope required"
            )
        if geometry.get("storage_physical") not in {"sram", "flip_flop"}:
            raise CharacterizationError("memory storage_physical required")
        if (
            geometry["storage_physical"] == "flip_flop"
            and geometry["mask_model"] == "read_modify_write"
        ):
            raise CharacterizationError(
                "flip-flop read-modify-write is unsupported"
            )
        timing_fields = (
            "minimum_access_interval_s",
            "minimum_access_latency_s",
        )
        if any(field not in geometry for field in timing_fields):
            raise CharacterizationError(
                "explicit memory timing fields required"
            )
        timing_values = [geometry[field] for field in timing_fields]
        timing_available = all(value is not None for value in timing_values)
        if not timing_available:
            if geometry["storage_scope"] != "storage_cells" or any(
                value is not None for value in timing_values
            ):
                raise CharacterizationError(
                    "null paired timing only belongs to storage_cells"
                )
        else:
            for field in timing_fields:
                _number(geometry[field], field)
        if geometry["capacity_bytes"] % geometry["word_bytes"]:
            raise CharacterizationError(
                "memory capacity must contain whole characterized words"
            )
        result.update(geometry)
        result["memory_timing_available"] = timing_available
        if not timing_available:
            for field in timing_fields:
                result[field] = 0.0
        provenance = json.loads(result["sources"][0])
        provenance["declared_memory_timing"] = {
            field: geometry[field] for field in timing_fields
        }
        result["sources"][0] = json.dumps(
            provenance, sort_keys=True, separators=(",", ":")
        )
    elif "memory" in seen:
        raise CharacterizationError("memory geometry is required")
    return result


def _json_mapping(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise CharacterizationError(f"duplicate key {key!r}")
        result[key] = value
    return result


def load(path):
    with open(path, encoding="utf-8") as stream:
        try:
            if os.path.splitext(os.fspath(path))[1].lower() == ".json":
                # JSON exponent syntax is not always numeric in YAML 1.1.
                document = json.load(stream, object_pairs_hook=_json_mapping)
            else:
                document = yaml.load(stream, Loader=_UniqueLoader)
        except (json.JSONDecodeError, yaml.YAMLError) as exc:
            raise CharacterizationError(f"{path}: {exc}") from exc
    return normalize(document, os.path.abspath(path))


def merge(models):
    merged = normalize({"version": 1, "records": []})
    for model in models:
        if set(model) != set(merged):
            raise CharacterizationError(
                "FU/SSA records cannot contain memory geometry"
            )
        duplicate = set(merged["keys"]) & set(model["keys"])
        if duplicate:
            raise CharacterizationError(
                f"duplicate bindings {sorted(duplicate)}"
            )
        for field in merged:
            merged[field].extend(model[field])
    return merged
