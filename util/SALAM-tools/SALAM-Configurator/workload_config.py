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

import sys

import yaml

_DOCUMENT_KEYS = {"acc_cluster", "hw_config"}
_CLUSTER_KEYS = {"Name", "DMA", "Accelerator", "SysPath", "HWPath"}
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
_REGISTER_KEYS = {"Name", "Type", "Size", "Ports", "ReadyMode", "Connections"}
_STREAM_KEYS = {
    "Name",
    "Type",
    "StreamSize",
    "BufferSize",
    "InCon",
    "OutCon",
}
_CACHE_KEYS = {"Name", "Type", "Size"}
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
                print(
                    f"SALAM config: {config_path}: accelerator {name}: "
                    f"ignoring {field}",
                    file=sys.stderr,
                )
        for variable in entry.get("Var", []) or []:
            _validate_variable(variable, config_path, name)


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
    _reject_unknown(variable, allowed, scope, config_path, var_name)
    if var_type == "SPM":
        variable["Ports"] = _spm_ports(
            variable.get("Ports", 1),
            config_path,
            var_name,
            "Ports" not in variable,
        )


def _spm_ports(value, config_path, var_name, omitted):
    if omitted:
        return 1
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise WorkloadConfigError(
            config_path,
            f"variable {var_name}",
            "Ports",
            value,
            f"invalid Ports={value!r}",
        )
    if isinstance(value, float):
        return int(value)
    if isinstance(value, int):
        return value
    try:
        return int(value, 10)
    except ValueError as exc:
        raise WorkloadConfigError(
            config_path,
            f"variable {var_name}",
            "Ports",
            value,
            f"invalid Ports={value!r}",
        ) from exc


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
