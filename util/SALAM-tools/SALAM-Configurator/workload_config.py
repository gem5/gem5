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
# These supported terminators bypass the timed compute queue.
_IMMEDIATE_TERMINATORS = ("br", "ret", "switch_inst")
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


class WorkloadLoader(yaml.SafeLoader):
    """Reject repeated explicit keys while preserving YAML merge rules."""

    def __init__(self, stream):
        super().__init__(stream)
        self._checked_mappings = set()

    def flatten_mapping(self, node):
        # A merged mapping can be visited again through an alias. Check
        # only its original explicit keys, before flattening merges.
        if node not in self._checked_mappings:
            self._checked_mappings.add(node)
            seen = set()
            for key_node, _ in node.value:
                key = (
                    "<<"
                    if key_node.tag == "tag:yaml.org,2002:merge"
                    else self.construct_object(key_node)
                )
                try:
                    repeated = key in seen
                    seen.add(key)
                except TypeError as exc:
                    raise WorkloadConfigError(
                        self.name,
                        f"line {key_node.start_mark.line + 1}",
                        None,
                        key,
                        "mapping keys must be scalar values",
                    ) from exc
                if repeated:
                    raise WorkloadConfigError(
                        self.name,
                        f"line {key_node.start_mark.line + 1}",
                        key,
                        None,
                        f"duplicate explicit key {key!r}",
                    )
        super().flatten_mapping(node)


def read_yaml(config_path):
    """Read workload or HWPath YAML without sharing mutable aliases."""
    try:
        with open(config_path, encoding="utf-8") as stream:
            documents = list(yaml.load_all(stream, Loader=WorkloadLoader))
    except OSError as exc:
        raise WorkloadConfigError(
            config_path,
            "document",
            None,
            None,
            f"cannot read YAML file: {exc.strerror}",
        ) from exc
    except yaml.YAMLError as exc:
        mark = getattr(exc, "problem_mark", None)
        scope = f"line {mark.line + 1}" if mark else "document"
        raise WorkloadConfigError(
            config_path, scope, None, None, str(exc)
        ) from exc

    def detached(value, active):
        if not isinstance(value, (dict, list)):
            return value
        if id(value) in active:
            raise WorkloadConfigError(
                config_path,
                "document",
                None,
                None,
                "recursive YAML aliases are unsupported",
            )
        active = active | {id(value)}
        if isinstance(value, dict):
            return {key: detached(item, active) for key, item in value.items()}
        return [detached(item, active) for item in value]

    return [detached(document, set()) for document in documents]


def load_documents(config_path):
    loaded = read_yaml(config_path)
    if not loaded:
        raise WorkloadConfigError(
            config_path, "document", None, None, "empty document"
        )
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


# The shared ARM workload allocator uses the PCI I/O aperture. The next
# region begins at 0x30000000 and belongs to platform PCI configuration.
WORKLOAD_ADDR_START = 0x2F000000
WORKLOAD_ADDR_END = 0x30000000
_UINT64_MAX = (1 << 64) - 1
_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*\Z")
_CACHE_PREFIXES = {
    "Ki": 1,
    "Mi": 2,
    "Gi": 3,
    "Ti": 4,
    "Pi": 5,
    "Ei": 6,
    "k": 1,
    "M": 2,
    "G": 3,
    "T": 4,
    "P": 5,
    "E": 6,
}
_RESERVED_CHILDREN = {
    "local_bus",
    "coherency_bus",
    "cluster_cache",
    "mem2cls",
    "system",
    "compute_voltage_domain",
    "compute_clk_domain",
    "fabric_voltage_domain",
    "fabric_clk_domain",
}


def _name(value, path, scope, field="Name"):
    if not isinstance(value, str) or not value.strip():
        raise WorkloadConfigError(
            path, scope, field, value, f"{field} must be a nonempty string"
        )
    if _NAME.fullmatch(value) is None:
        raise WorkloadConfigError(
            path,
            scope,
            field,
            value,
            f"{field} must start with a letter and contain only letters, "
            "digits, and underscores",
        )
    return value


def _path(value, path, scope, field):
    if not isinstance(value, str) or not value.strip():
        raise WorkloadConfigError(
            path, scope, field, value, f"{field} must be a nonempty path"
        )


def _positive_bytes(value, path, scope, field, maximum=_UINT64_MAX):
    if type(value) is not int or not 1 <= value <= maximum:
        raise WorkloadConfigError(
            path,
            scope,
            field,
            value,
            f"{field} must be an integer from 1 to {maximum} bytes",
        )
    return value


def _cache_bytes(value, path, scope):
    original = value
    if isinstance(value, str):
        # Match gem5's integer-magnitude and binary-prefix conversion of
        # the previously accepted prefix-only spelling plus its B suffix.
        prefix = next((p for p in _CACHE_PREFIXES if value.endswith(p)), "")
        magnitude = value[: -len(prefix)] if prefix else value
        try:
            value = int(magnitude, 0) * 1024 ** _CACHE_PREFIXES.get(prefix, 0)
        except ValueError:
            pass
    if type(value) is not int or not 1 <= value <= _UINT64_MAX:
        raise WorkloadConfigError(
            path,
            scope,
            "Size",
            original,
            "Cache Size must be positive integer bytes or an integer "
            "with a binary prefix, such as '32Ki' (without a B suffix)",
        )
    return value


def _boolean(value, path, scope, field):
    if isinstance(value, bool):
        return value
    if type(value) is int and value in (0, 1):
        return value == 1
    if isinstance(value, str):
        if value.lower() in ("true", "t", "yes", "y", "1"):
            return True
        if value.lower() in ("false", "f", "no", "n", "0"):
            return False
    raise WorkloadConfigError(
        path, scope, field, value, f"{field} must be a boolean"
    )


def _names(value, path, scope, field):
    if not isinstance(value, str) or not value.strip():
        raise WorkloadConfigError(
            path,
            scope,
            field,
            value,
            f"{field} must be a nonempty string of device names",
        )
    names = [item.strip() for item in value.split(",")]
    for name in names:
        _name(name, path, scope, field)
    return names


def _collect_fields(entries, path, scope, repeatable):
    fields = {}
    for entry in entries:
        if not isinstance(entry, dict):
            raise WorkloadConfigError(
                path, scope, None, entry, f"{scope} entry must be a mapping"
            )
        for field, value in entry.items():
            if field in repeatable:
                continue
            if field in fields:
                raise WorkloadConfigError(
                    path,
                    scope,
                    field,
                    value,
                    f"duplicate {field} across {scope} entries",
                )
            fields[field] = value
    return fields


def _validate_cluster_list(cluster_items, config_path):
    if not isinstance(cluster_items, list):
        raise WorkloadConfigError(
            config_path,
            "cluster",
            "acc_cluster",
            cluster_items,
            "acc_cluster must be a list",
        )
    fields = _collect_fields(
        cluster_items, config_path, "cluster", {"DMA", "Accelerator"}
    )
    if cluster_items:
        if "SysPath" in fields:
            _path(fields["SysPath"], config_path, "cluster", "SysPath")
            if any(key in fields for key in ("Name", "Clocks")) or any(
                "DMA" in item or "Accelerator" in item
                for item in cluster_items
            ):
                raise WorkloadConfigError(
                    config_path,
                    "cluster",
                    "SysPath",
                    fields["SysPath"],
                    "SysPath cannot be combined with local device definitions",
                )
        else:
            _name(fields.get("Name"), config_path, "cluster")
        if "HWPath" in fields:
            _path(fields["HWPath"], config_path, "cluster", "HWPath")
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
        if "DMA" in item:
            if not isinstance(item["DMA"], list):
                raise WorkloadConfigError(
                    config_path,
                    "DMA",
                    "DMA",
                    item["DMA"],
                    "DMA must be a list",
                )
            for dma in item["DMA"]:
                _validate_dma(dma, config_path)
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
        match = _CLOCK.fullmatch(value) if isinstance(value, str) else None
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


def _validate_dma(device, config_path):
    if not isinstance(device, dict):
        raise WorkloadConfigError(
            config_path, "DMA", None, device, "DMA entry must be a mapping"
        )
    name = device.get("Name", "DMA")
    scope = f"DMA {name}"
    if "ConfigPath" in device:
        _unsupported(config_path, scope, "ConfigPath", device["ConfigPath"])
    _reject_unknown(device, _DMA_KEYS, scope, config_path, name)
    dma_type = device.get("Type")
    if dma_type not in ("NonCoherent", "Stream"):
        raise WorkloadConfigError(
            config_path,
            scope,
            "Type",
            dma_type,
            "Type must be NonCoherent or Stream",
        )
    for field in ("Name", "PIOMaster"):
        value = device.get(field)
        if not isinstance(value, str) or not value.strip():
            raise WorkloadConfigError(
                config_path,
                scope,
                field,
                value,
                f"{field} must be a nonempty string",
            )
    _name(device["Name"], config_path, scope)
    masters = _names(device["PIOMaster"], config_path, scope, "PIOMaster")
    if not all(masters):
        raise WorkloadConfigError(
            config_path,
            scope,
            "PIOMaster",
            device["PIOMaster"],
            "PIOMaster must list nonempty device names",
        )
    if any(
        master.lower() not in ("localbus", "local_bus") for master in masters
    ):
        raise WorkloadConfigError(
            config_path,
            scope,
            "PIOMaster",
            device["PIOMaster"],
            "PIOMaster supports only LocalBus",
        )
    device["PIOMaster"] = "LocalBus"
    unsupported = (
        ("ReadInt", "WriteInt")
        if dma_type == "NonCoherent"
        else ("InterruptNum",)
    )
    for field in unsupported:
        if field in device:
            _unsupported(config_path, scope, field, device[field])
    # Omission leaves the SimObject's default in effect. An explicit null,
    # bool, float or numeric string is not a size or interrupt number.
    for field, minimum, maximum in (
        ("BufferSize", 1, (1 << 64) - 1),
        ("MaxReqSize", 1, _UINT32_MAX),
        ("InterruptNum", 0, _UINT32_MAX),
        ("ReadInt", 0, _UINT32_MAX),
        ("WriteInt", 0, _UINT32_MAX),
    ):
        if field not in device:
            continue
        value = device[field]
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not minimum <= value <= maximum
        ):
            raise WorkloadConfigError(
                config_path,
                scope,
                field,
                value,
                f"{field} must be an integer from {minimum} to {maximum}, "
                f"got {value!r}",
            )


def _validate_accelerator(accelerator, config_path):
    if not isinstance(accelerator, list):
        raise WorkloadConfigError(
            config_path,
            "accelerator",
            "Accelerator",
            accelerator,
            "Accelerator must be a list",
        )
    fields = _collect_fields(accelerator, config_path, "accelerator", {"Var"})
    name = fields.get("Name", "accelerator")
    scope = f"accelerator {name}"
    for entry in accelerator:
        if "ConfigPath" in entry:
            _unsupported(config_path, scope, "ConfigPath", entry["ConfigPath"])
        _reject_unknown(entry, _ACCEL_KEYS, scope, config_path, name)
        for field in _LEGACY_ACCEL_KEYS:
            if field in entry:
                hint = (
                    "; use Clocks.Compute on the cluster"
                    if field == "ClockPeriod_ns"
                    else "; the entry function is top"
                )
                raise WorkloadConfigError(
                    config_path,
                    scope,
                    field,
                    entry[field],
                    f"unsupported field {field}{hint}",
                )
        if "Var" in entry:
            if not isinstance(entry["Var"], list):
                raise WorkloadConfigError(
                    config_path,
                    scope,
                    "Var",
                    entry["Var"],
                    "Var must be a list",
                )
            for variable in entry["Var"]:
                _validate_variable(variable, config_path, name)
    if "Profile" not in fields:
        raise WorkloadConfigError(
            config_path, scope, "Profile", None, "missing Profile"
        )
    _name(fields.get("Name"), config_path, scope)
    _path(fields.get("IrPath"), config_path, scope, "IrPath")
    if not isinstance(fields["Profile"], str) or not fields["Profile"].strip():
        raise WorkloadConfigError(
            config_path,
            scope,
            "Profile",
            fields["Profile"],
            "Profile must be a nonempty string",
        )
    _positive_bytes(fields.get("PIOSize"), config_path, scope, "PIOSize")
    masters = _names(fields.get("PIOMaster"), config_path, scope, "PIOMaster")
    if any(
        master.lower() not in ("localbus", "local_bus") for master in masters
    ):
        raise WorkloadConfigError(
            config_path,
            scope,
            "PIOMaster",
            fields["PIOMaster"],
            "PIOMaster supports only LocalBus",
        )
    for entry in accelerator:
        if "PIOMaster" in entry:
            entry["PIOMaster"] = "LocalBus"
        if "Debug" in entry:
            entry["Debug"] = _boolean(
                entry["Debug"], config_path, scope, "Debug"
            )
        if "HWPath" in entry:
            _path(entry["HWPath"], config_path, scope, "HWPath")
        if "InterruptNum" in entry:
            value = entry["InterruptNum"]
            if type(value) is not int or not 0 <= value <= (1 << 31) - 1:
                raise WorkloadConfigError(
                    config_path,
                    scope,
                    "InterruptNum",
                    value,
                    "InterruptNum must be a nonnegative signed 32-bit integer",
                )
        for field in ("LocalSlaves", "StreamIn", "StreamOut"):
            if field in entry:
                entry[field] = ",".join(
                    _names(entry[field], config_path, scope, field)
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
    _name(variable.get("Name"), config_path, scope)
    allowed = _VAR_KEYS.get(var_type) if isinstance(var_type, str) else None
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
    if var_type in ("SPM", "RegisterBank"):
        variable["Size"] = _positive_bytes(
            variable.get("Size"), config_path, scope, "Size"
        )
    elif var_type == "Cache":
        variable["Size"] = _cache_bytes(
            variable.get("Size"), config_path, scope
        )
    elif var_type == "Stream":
        variable["StreamSize"] = _positive_bytes(
            variable.get("StreamSize"), config_path, scope, "StreamSize"
        )
        if "BufferSize" in variable:
            _positive_bytes(
                variable["BufferSize"], config_path, scope, "BufferSize"
            )
        for field in ("InCon", "OutCon"):
            variable[field] = _name(
                variable.get(field), config_path, scope, field
            ).lower()
    if var_type == "SPM":
        for field in (
            "ReadyMode",
            "ResetOnRead",
            "ReadOnInvalid",
            "WriteOnValid",
        ):
            if field in variable:
                variable[field] = _boolean(
                    variable[field], config_path, scope, field
                )
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
    names = {name.lower() for name in map(_accelerator_name, accelerators)}
    dmas = [dma for item in cluster_items for dma in item.get("DMA", [])]
    streams = {dma["Name"].lower() for dma in dmas if dma["Type"] == "Stream"}
    children = set(_RESERVED_CHILDREN)

    def claim(name, scope):
        if name in children:
            raise WorkloadConfigError(
                config_path,
                scope,
                "Name",
                name,
                f"Name {name!r} collides with another cluster child",
            )
        children.add(name)

    for dma in dmas:
        claim(dma["Name"].lower(), f"DMA {dma['Name']}")
    for accelerator in accelerators:
        acc_name = _accelerator_name(accelerator)
        claim(acc_name.lower(), f"accelerator {acc_name}")
    stream_owners = {}
    for accelerator in accelerators:
        acc_name = _accelerator_name(accelerator)
        for entry in accelerator:
            for variable in entry.get("Var", []):
                child = variable["Name"]
                if variable["Type"] != "Cache":
                    child = child.lower()
                claim(child, f"variable {variable['Name']}")
                if variable["Type"] == "Stream":
                    for field in ("InCon", "OutCon"):
                        if variable[field] not in names:
                            raise WorkloadConfigError(
                                config_path,
                                f"variable {variable['Name']}",
                                field,
                                variable[field],
                                f"{field} names unknown accelerator {variable[field]!r}",
                            )
            for field in ("LocalSlaves", "StreamIn", "StreamOut"):
                if field not in entry:
                    continue
                targets = entry[field].split(",")
                # PIO ports already belong to LocalBus. Stream buffers
                # already have both endpoints bound by InCon/OutCon.
                allowed = (
                    {"localbus", "local_bus"}
                    if field == "LocalSlaves"
                    else streams
                )
                for target in targets:
                    if target.lower() not in allowed:
                        raise WorkloadConfigError(
                            config_path,
                            f"accelerator {_accelerator_name(accelerator)}",
                            field,
                            target,
                            f"{field} names unsupported target {target!r}",
                        )
                    if field != "LocalSlaves":
                        # Each StreamDMA direction is a scalar port. The
                        # same accelerator may own both distinct ports.
                        endpoint = (target.lower(), field)
                        if endpoint in stream_owners:
                            owner = stream_owners[endpoint]
                            raise WorkloadConfigError(
                                config_path,
                                f"accelerator {acc_name}",
                                field,
                                target,
                                f"{field} target {target!r} is already "
                                f"connected to accelerator {owner!r}",
                            )
                        stream_owners[endpoint] = acc_name
                entry[field] = ",".join(
                    (
                        "LocalBus"
                        if target.lower() in ("localbus", "local_bus")
                        else target.lower()
                    )
                    for target in targets
                )
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

    if name in _IMMEDIATE_TERMINATORS and cycles != 0:
        raise WorkloadConfigError(
            config_path,
            scope,
            "runtime_cycles",
            cycles,
            f"nonzero runtime_cycles for {name!r} is not supported; "
            "terminator latency is not modeled",
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
