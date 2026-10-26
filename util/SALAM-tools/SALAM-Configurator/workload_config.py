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

import math
import re
import sys

import yaml

_DOCUMENT_KEYS = {"acc_cluster", "hw_config"}
_CLUSTER_KEYS = {"Name", "DMA", "Accelerator", "SysPath", "HWPath", "Clocks"}
# A cluster's clock domains: frequency or period strings.
DEFAULT_CLOCKS = {"Compute": "100MHz", "Fabric": "1GHz"}
_CLOCK = re.compile(r"^(\d+(?:\.\d+)?)(THz|GHz|MHz|kHz|Hz|s|ms|us|ns|ps)$")
_DMA_KEYS = {
    "Name",
    "MaxReqSize",
    "BufferSize",
    "PIOMaster",
    "Type",
    "InterruptNum",
    "ReadInt",
    "WriteInt",
}
_ACCEL_KEYS = {
    "Name",
    "IrPath",
    "PIOSize",
    "InterruptNum",
    "PIOMaster",
    "LocalSlaves",
    "Debug",
    "Var",
    "StreamIn",
    "StreamOut",
    "HWPath",
    "Profile",
    "TopName",
    "ClockPeriod_ns",
}
_LEGACY_ACCEL_KEYS = {"TopName", "ClockPeriod_ns"}
_SPM_KEYS = {
    "Name",
    "Type",
    "Size",
    "Ports",
    "ReadyMode",
    "ResetOnRead",
    "ReadOnInvalid",
    "WriteOnValid",
    "Connections",
}
_REGISTER_KEYS = {"Name", "Type", "Size"}
# RegisterBank has no consumer for Ports or ReadyMode. Its single
# reg_port connects to the owning accelerator only, so extra
# Connections cannot be honored.
_UNSUPPORTED_REGISTER_KEYS = ("Ports", "ReadyMode", "Connections")
_STREAM_KEYS = {
    "Name",
    "Type",
    "StreamSize",
    "BufferSize",
    "InCon",
    "OutCon",
}
_CACHE_KEYS = {"Name", "Type", "Size"}
# Parameters of the CycleCounts SimObject (src/salam/CycleCounts.py).
CYCLE_COUNT_PARAMS = frozenset(
    (
        "counter",
        "gep",
        "phi",
        "select",
        "ret",
        "br",
        "switch_inst",
        "indirectbr",
        "invoke",
        "resume",
        "unreachable",
        "icmp",
        "fcmp",
        "trunc",
        "zext",
        "sext",
        "fptrunc",
        "fpext",
        "fptoui",
        "fptosi",
        "uitofp",
        "sitofp",
        "ptrtoint",
        "inttoptr",
        "bitcast",
        "addrspacecast",
        "call",
        "vaarg",
        "landingpad",
        "catchpad",
        "alloca",
        "load",
        "store",
        "fence",
        "cmpxchg",
        "atomicrmw",
        "extractvalue",
        "insertvalue",
        "extractelement",
        "insertelement",
        "shufflevector",
        "shl",
        "lshr",
        "ashr",
        "and_inst",
        "or_inst",
        "xor_inst",
        "add",
        "sub",
        "mul",
        "udiv",
        "sdiv",
        "urem",
        "srem",
        "fadd",
        "fsub",
        "fmul",
        "fdiv",
        "frem",
    )
)
# Fields of an hw_config instruction entry. Only runtime_cycles is read;
# the others follow the inst_list.yml format.
_INSTRUCTION_ENTRY_KEYS = {
    "runtime_cycles",
    "functional_unit",
    "functional_unit_limit",
    "opcode_num",
}
_UINT32_MAX = (1 << 32) - 1
# Load and store latency comes from the memory system, not CycleCounts.
_MEMORY_INSTRUCTIONS = ("load", "store")
_VAR_KEYS = {
    "SPM": _SPM_KEYS,
    "RegisterBank": _REGISTER_KEYS,
    "Stream": _STREAM_KEYS,
    "Cache": _CACHE_KEYS,
}


# Raised when a workload YAML value does not match the shared schema.
class WorkloadConfigError(Exception):

    def __init__(self, path, scope, field, value, reason):
        self.path = path
        self.scope = scope
        self.field = field
        self.value = value
        self.reason = reason
        super().__init__(f"SALAM config: {path}: {scope}: {reason}")


def load_documents(config_path):
    with open(config_path, encoding="utf-8") as stream:
        loaded = list(yaml.safe_load_all(stream))
    documents = []
    for document in loaded:
        if document is None:
            raise WorkloadConfigError(
                config_path,
                "document",
                None,
                None,
                "empty document",
            )
        if not isinstance(document, dict):
            raise WorkloadConfigError(
                config_path,
                "document",
                None,
                document,
                "top-level value must be a mapping",
            )
        _reject_unknown(
            document,
            _DOCUMENT_KEYS,
            "document",
            config_path,
            "workload",
        )
        _validate_cluster_list(
            document.get("acc_cluster", []),
            config_path,
        )
        if "hw_config" in document:
            validate_hw_config(document["hw_config"], config_path)
        documents.append(document)
    return documents


def _validate_cluster_list(cluster_items, config_path):
    if cluster_items is None:
        return
    if not isinstance(cluster_items, list):
        raise WorkloadConfigError(
            config_path,
            "cluster",
            "acc_cluster",
            cluster_items,
            "acc_cluster must be a list",
        )
    for item in cluster_items:
        if not isinstance(item, dict):
            raise WorkloadConfigError(
                config_path,
                "cluster",
                None,
                item,
                "cluster entry must be a mapping",
            )
        if "ConfigPath" in item:
            _unsupported(
                config_path, "cluster", "ConfigPath", item["ConfigPath"]
            )
        _reject_unknown(item, _CLUSTER_KEYS, "cluster", config_path, "cluster")
        for dma_group in item.get("DMA", []) or []:
            _validate_device_group(
                dma_group,
                "DMA",
                _DMA_KEYS,
                config_path,
            )
        if "Accelerator" in item:
            _validate_accelerator(item["Accelerator"], config_path)
        if "Clocks" in item:
            parse_clocks(item["Clocks"], config_path)
    _validate_connection_targets(cluster_items, config_path)


def parse_clocks(clocks, config_path):
    # The cluster's Compute and Fabric clocks, each a positive frequency
    # ("100MHz") or period ("10ns"), with defaults for those omitted.
    if not isinstance(clocks, dict):
        raise WorkloadConfigError(
            config_path,
            "cluster",
            "Clocks",
            clocks,
            "Clocks must be a mapping of Compute and Fabric clocks",
        )
    _reject_unknown(clocks, DEFAULT_CLOCKS, "cluster", config_path, "Clocks")
    parsed = dict(DEFAULT_CLOCKS)
    for name, value in clocks.items():
        match = _CLOCK.match(value) if isinstance(value, str) else None
        if match is None or float(match.group(1)) <= 0:
            raise WorkloadConfigError(
                config_path,
                "cluster",
                f"Clocks.{name}",
                value,
                f"Clocks {name} must be a positive frequency such as "
                f"'100MHz' or a period such as '10ns'",
            )
        parsed[name] = value
    return parsed


def _validate_device_group(device, scope, allowed, config_path):
    if not isinstance(device, dict):
        raise WorkloadConfigError(
            config_path,
            scope,
            None,
            device,
            f"{scope} entry must be a mapping",
        )
    name = device.get("Name", scope)
    if "ConfigPath" in device:
        _unsupported(config_path, scope, "ConfigPath", device["ConfigPath"])
    _reject_unknown(device, allowed, scope, config_path, name)


def _validate_accelerator(accelerator, config_path):
    if not isinstance(accelerator, list):
        raise WorkloadConfigError(
            config_path,
            "accelerator",
            "Accelerator",
            accelerator,
            "Accelerator must be a list",
        )
    name = "accelerator"
    for entry in accelerator:
        if isinstance(entry, dict) and "Name" in entry:
            name = entry["Name"]
            break
    for entry in accelerator:
        if not isinstance(entry, dict):
            raise WorkloadConfigError(
                config_path,
                f"accelerator {name}",
                None,
                entry,
                "accelerator entry must be a mapping",
            )
        if "ConfigPath" in entry:
            _unsupported(
                config_path,
                f"accelerator {name}",
                "ConfigPath",
                entry["ConfigPath"],
            )
        _reject_unknown(
            entry,
            _ACCEL_KEYS,
            f"accelerator {name}",
            config_path,
            name,
        )
        for field in _LEGACY_ACCEL_KEYS:
            if field in entry:
                hint = (
                    "; set the cluster's Clocks Compute clock instead"
                    if field == "ClockPeriod_ns"
                    else ""
                )
                print(
                    f"SALAM config: {config_path}: accelerator {name}: "
                    f"ignoring {field}{hint}",
                    file=sys.stderr,
                )
        for variable in entry.get("Var", []) or []:
            _validate_variable(variable, config_path, name)
    if not any(
        isinstance(entry, dict) and entry.get("Profile")
        for entry in accelerator
    ):
        raise WorkloadConfigError(
            config_path,
            f"accelerator {name}",
            "Profile",
            None,
            "missing Profile",
        )


def _validate_variable(variable, config_path, acc_name):
    if not isinstance(variable, dict):
        raise WorkloadConfigError(
            config_path,
            f"accelerator {acc_name}",
            "Var",
            variable,
            "variable must be a mapping",
        )
    var_type = variable.get("Type")
    var_name = variable.get("Name", "variable")
    scope = f"variable {var_name}"
    allowed = _VAR_KEYS.get(var_type)
    if allowed is None:
        raise WorkloadConfigError(
            config_path,
            scope,
            "Type",
            var_type,
            f"unknown variable type {var_type!r}",
        )
    if var_type == "RegisterBank":
        for field in _UNSUPPORTED_REGISTER_KEYS:
            if field in variable:
                _unsupported(config_path, scope, field, variable[field])
    _reject_unknown(variable, allowed, scope, config_path, var_name)
    if var_type == "SPM":
        variable["Ports"] = _spm_ports(
            variable.get("Ports", 1),
            config_path,
            var_name,
            "Ports" not in variable,
        )


def _accelerator_name(accelerator):
    for entry in accelerator:
        if isinstance(entry, dict) and "Name" in entry:
            return entry["Name"]
    return None


def _validate_connection_targets(cluster_items, config_path):
    # SPM Connections must name an accelerator of the same cluster.
    accelerators = [
        item["Accelerator"]
        for item in cluster_items
        if isinstance(item, dict) and isinstance(item.get("Accelerator"), list)
    ]
    names = {
        str(name).lower()
        for name in map(_accelerator_name, accelerators)
        if name is not None
    }
    for accelerator in accelerators:
        for entry in accelerator:
            for variable in entry.get("Var", []) or []:
                if "Connections" not in variable:
                    continue
                var_name = variable.get("Name", "variable")
                for target, _count in parse_connections(
                    variable["Connections"], config_path, var_name
                ):
                    if target.lower() not in names:
                        raise WorkloadConfigError(
                            config_path,
                            f"variable {var_name}",
                            "Connections",
                            variable["Connections"],
                            f"Connections names unknown accelerator "
                            f"{target!r}",
                        )


def parse_connections(value, config_path, var_name):
    # Parse "Name:N[,Name:N...]" into (name, ports) pairs.
    scope = f"variable {var_name}"
    if not isinstance(value, str) or not value.strip():
        raise WorkloadConfigError(
            config_path,
            scope,
            "Connections",
            value,
            "Connections must be a string of Name:N entries",
        )
    connections = []
    for item in value.split(","):
        name, sep, count = item.strip().partition(":")
        name = name.strip()
        ports = None
        if sep and name:
            try:
                ports = int(count.strip(), 10)
            except ValueError:
                pass
        if ports is None or ports < 1:
            raise WorkloadConfigError(
                config_path,
                scope,
                "Connections",
                value,
                f"Connections entry {item.strip()!r} must be Name:N with "
                "a positive integer N",
            )
        connections.append((name, ports))
    return connections


def validate_hw_config(hw_config, config_path):
    # Shared by inline hw_config and files selected with HWPath.
    if not isinstance(hw_config, dict):
        raise WorkloadConfigError(
            config_path,
            "hw_config",
            "hw_config",
            hw_config,
            "hw_config must be a mapping of profiles",
        )
    for profile, node in hw_config.items():
        scope = f"hw_config profile {profile!r}"
        # An explicit null profile means no overrides.
        if node is None:
            continue
        if (
            not isinstance(node, dict)
            or set(node) != {"instructions"}
            or not isinstance(node["instructions"], dict)
        ):
            raise WorkloadConfigError(
                config_path,
                scope,
                None,
                node,
                "profile must be null or contain only an instructions "
                "mapping",
            )
        for name, entry in node["instructions"].items():
            _validate_instruction_entry(name, entry, config_path, scope)


def _validate_instruction_entry(name, entry, config_path, scope):
    if name not in CYCLE_COUNT_PARAMS:
        raise WorkloadConfigError(
            config_path,
            scope,
            name,
            entry,
            f"unknown instruction {name!r}; expected a CycleCounts parameter",
        )
    if not isinstance(entry, dict):
        raise WorkloadConfigError(
            config_path,
            scope,
            name,
            entry,
            f"instruction {name!r} must be a mapping",
        )
    for field in entry:
        if field not in _INSTRUCTION_ENTRY_KEYS:
            raise WorkloadConfigError(
                config_path,
                scope,
                field,
                entry[field],
                f"unknown field {field!r} on instruction {name!r}",
            )
    if "runtime_cycles" not in entry:
        raise WorkloadConfigError(
            config_path,
            scope,
            "runtime_cycles",
            None,
            f"instruction {name!r} has no runtime_cycles",
        )
    cycles = entry["runtime_cycles"]
    if (
        isinstance(cycles, bool)
        or not isinstance(cycles, int)
        or not 0 <= cycles <= _UINT32_MAX
    ):
        raise WorkloadConfigError(
            config_path,
            scope,
            "runtime_cycles",
            cycles,
            f"runtime_cycles of {name!r} must be an integer from 0 to "
            f"{_UINT32_MAX}, got {cycles!r}",
        )
    if name in _MEMORY_INSTRUCTIONS and cycles != 0:
        raise WorkloadConfigError(
            config_path,
            scope,
            "runtime_cycles",
            cycles,
            f"nonzero runtime_cycles for {name!r} is not supported; "
            "memory latency comes from the memory system",
        )


def _spm_ports(value, config_path, var_name, omitted):
    if omitted:
        return 1
    ports = None
    if isinstance(value, bool):
        pass
    elif isinstance(value, int):
        ports = value
    elif isinstance(value, float):
        if math.isfinite(value) and value.is_integer():
            ports = int(value)
    elif isinstance(value, str):
        try:
            ports = int(value, 10)
        except ValueError:
            pass
    if ports is None or ports < 1:
        raise WorkloadConfigError(
            config_path,
            f"variable {var_name}",
            "Ports",
            value,
            f"Ports must be a positive integer, got {value!r}",
        )
    return ports


def _reject_unknown(mapping, allowed, scope, config_path, obj_name):
    for field in mapping:
        if field not in allowed:
            raise WorkloadConfigError(
                config_path,
                scope,
                field,
                mapping[field],
                f"unknown field {field!r} on {obj_name}",
            )


def _unsupported(config_path, scope, field, value):
    raise WorkloadConfigError(
        config_path,
        scope,
        field,
        value,
        f"unsupported field {field}",
    )
