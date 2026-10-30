# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
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

import os

try:
    # Embedded in gem5 as gem5.salam.config_parser.
    from . import workload_config
except ImportError:
    # Run from util/SALAM-tools/SALAM-Configurator.
    import workload_config


def open_yaml(yml_path: str):
    return workload_config.load_documents(yml_path)


def _resolve_path(declaring_file, path):
    # A relative path is relative to the YAML file that declares it.
    return os.path.normpath(
        os.path.join(os.path.dirname(declaring_file), path)
    )


def parse_yaml(
    parent_config,
    base_address,
    working_dir: str,
    parent_path: str = None,
    hw_path: str = None,
    hw_config_document=None,
    hw_config_source: str = None,
    _include_stack=(),
):
    current = os.path.realpath(parent_path) if parent_path else None
    if current in _include_stack:
        raise workload_config.WorkloadConfigError(
            parent_path,
            "cluster",
            "SysPath",
            parent_path,
            "recursive SysPath inclusion is unsupported",
        )
    _include_stack = _include_stack + (current,)
    clusters = []
    # Load in each acc cluster and add it to the list
    for cluster_dict in parent_config:
        # Each document starts from the inherited hw_config selection.
        # hw_config_document is the YAML document whose inline hw_config
        # was selected, or None when hw_path names an external file.
        doc_hw_path = hw_path
        doc_hw_document = hw_config_document
        # YAML file that selected doc_hw_path, for diagnostics.
        doc_hw_source = hw_config_source
        if (
            doc_hw_document is None
            and doc_hw_path is not None
            and doc_hw_path == parent_path
        ):
            doc_hw_document = cluster_dict
        for list_type, params in cluster_dict.items():
            FOUND_SYS_PATH = False
            FOUND_HW_PATH = False
            FOUND_DEVICE = False
            if list_type == "acc_cluster":
                for param in params:
                    if "SysPath" in param:
                        FOUND_SYS_PATH = True
                        cur_path = os.path.abspath(
                            os.path.join(working_dir, param["SysPath"])
                        )
                        cur_config = open_yaml(yml_path=cur_path)
                    if "HWPath" in param:
                        FOUND_HW_PATH = True
                        doc_hw_path = _resolve_path(
                            parent_path, param["HWPath"]
                        )
                        doc_hw_source = parent_path
                        doc_hw_document = (
                            cluster_dict
                            if doc_hw_path == parent_path
                            else None
                        )
                    if "SysPath" not in param and "HWPath" not in param:
                        FOUND_DEVICE = True
                        cur_config = parent_config
                        # Use this document's inline hw_config unless an
                        # HWPath already selected one.
                        if not FOUND_HW_PATH and doc_hw_path is None:
                            doc_hw_path = parent_path
                            doc_hw_document = cluster_dict
                            doc_hw_source = parent_path
            if FOUND_SYS_PATH and FOUND_DEVICE:
                raise Exception(
                    "Found device definitions in a cluster with a"
                    " path to another YAML file."
                )
            if FOUND_SYS_PATH:
                # Recursion Alert!
                base_address, temp_cluster = parse_yaml(
                    parent_config=cur_config,
                    base_address=base_address,
                    working_dir=working_dir,
                    parent_path=cur_path,
                    hw_path=doc_hw_path,
                    hw_config_document=doc_hw_document,
                    hw_config_source=doc_hw_source,
                    _include_stack=_include_stack,
                )
                clusters.extend(temp_cluster)
            elif FOUND_HW_PATH:
                raise Exception(
                    "HW Path should be defined with a System Config file"
                )

        cluster_name = None
        dmas = []
        accs = []
        clocks = dict(workload_config.DEFAULT_CLOCKS)
        for list_type, devices in cluster_dict.items():
            if list_type == "acc_cluster":
                for device in devices:
                    if "Name" in device:
                        cluster_name = device["Name"]
                    if "Clocks" in device:
                        clocks = workload_config.parse_clocks(
                            device["Clocks"], parent_path
                        )
                    if "DMA" in device:
                        dmas.append(device)
                    if "Accelerator" in device:
                        accs.append(device)
        if cluster_name is None:
            continue
        clusters.append(
            AccCluster(
                name=cluster_name,
                dmas=dmas,
                accs=accs,
                base_address=base_address,
                working_dir=working_dir,
                config_path=parent_path,
                hw_config_path=doc_hw_path,
                hw_config_document=doc_hw_document,
                hw_config_source=doc_hw_source,
                document=cluster_dict,
                clocks=clocks,
            )
        )
        end = clusters[-1].top_address
        base_address = _advance_address(
            end,
            64 - end % 64,
            parent_path,
            f"cluster {cluster_name}",
            "Name",
        )
        if (int(base_address) % 64) != 0:
            print("Address Alignment Error: " + hex(base_address))
    return base_address, clusters


def load_clusters(
    working_dir, config_name="config.yml", base_address=0x2F000000
):
    working_dir = os.path.abspath(working_dir)
    main_yml_path = os.path.abspath(os.path.join(working_dir, config_name))
    if type(base_address) is not int or not 0 <= base_address < (1 << 64):
        raise workload_config.WorkloadConfigError(
            main_yml_path,
            "cluster",
            "base_address",
            base_address,
            "base_address must be an unsigned 64-bit integer",
        )
    config = open_yaml(yml_path=main_yml_path)
    _, clusters = parse_yaml(
        parent_config=config,
        base_address=base_address,
        working_dir=working_dir,
        parent_path=main_yml_path,
    )
    names = set()
    for cluster in clusters:
        child = cluster.name.lower()
        if child in names:
            raise workload_config.WorkloadConfigError(
                cluster.config_path,
                "cluster",
                "Name",
                cluster.name,
                f"duplicate cluster Name {cluster.name!r}",
            )
        names.add(child)
    return clusters


def _advance_address(address, increment, path, scope, field):
    if address < 0 or increment > (1 << 64) - 1 - address:
        raise workload_config.WorkloadConfigError(
            path,
            scope,
            field,
            increment,
            f"{field} allocation exceeds the 64-bit address space",
        )
    return address + increment


def _reserve(address, size, path, scope, field):
    # Preserve the existing extra 64-byte boundary even for aligned sizes.
    increment = size + (64 - size % 64)
    return _advance_address(address, increment, path, scope, field)


class AccCluster:
    def __init__(
        self,
        name: str,
        dmas,
        accs,
        base_address: int,
        working_dir: str,
        config_path: str,
        hw_config_path: str = None,
        hw_config_document=None,
        hw_config_source: str = None,
        document=None,
        clocks=None,
    ):
        self.name = name
        # Compute and Fabric clock strings for the cluster's domains.
        self.clocks = dict(clocks or workload_config.DEFAULT_CLOCKS)
        self.dmas = dmas
        self.accs = accs
        self.base_address = base_address
        self.top_address = base_address
        self.config_path = config_path
        # Do this to point the hardware configuration to the
        # sys config YAML file when HWPath isn't defined
        self.hw_config_path = hw_config_path
        # YAML document whose inline hw_config is selected, or None when
        # hw_config_path names an external file.
        self.hw_config_document = hw_config_document
        # YAML file that selected hw_config_path, for diagnostics.
        self.hw_config_source = hw_config_source
        # YAML document that defined this cluster. An accelerator HWPath
        # naming config_path selects its inline hw_config.
        self.document = document
        self.process_config(working_dir=working_dir)

    def process_config(self, working_dir):
        dma_class = []
        acc_class = []
        top_address = self.base_address

        # DMA records have already passed the shared workload validator.
        for dma in self.dmas:
            for device in dma["DMA"]:
                params = dict(
                    name=device["Name"],
                    pio_masters=device["PIOMaster"].split(","),
                    address=top_address,
                    dmaType=device["Type"],
                    size=device.get("BufferSize"),
                    maxReq=device.get("MaxReqSize"),
                )
                if device["Type"] == "NonCoherent":
                    pio_size = 21
                    dma_class.append(
                        DMA(
                            pio=pio_size,
                            int_num=device.get("InterruptNum"),
                            **params,
                        )
                    )
                    top_address = _reserve(
                        top_address,
                        pio_size,
                        self.config_path,
                        f"DMA {device['Name']}",
                        "PIOSize",
                    )
                else:
                    pio_size = 32
                    status_size = 4
                    aligned_inc = pio_size + (64 - pio_size % 64)
                    dma_class.append(
                        StreamDMA(
                            pio=pio_size,
                            statusAddress=top_address + aligned_inc,
                            rd_int=device.get("ReadInt"),
                            wr_int=device.get("WriteInt"),
                            **params,
                        )
                    )
                    data_end = _reserve(
                        top_address,
                        pio_size,
                        self.config_path,
                        f"DMA {device['Name']}",
                        "PIOSize",
                    )
                    top_address = _reserve(
                        data_end,
                        status_size,
                        self.config_path,
                        f"DMA {device['Name']}",
                        "PIOSize",
                    )
        # Parse Accelerators
        for acc in self.accs:
            name = None
            pio_masters = []
            stream_in = []
            stream_out = []
            local_connections = []
            variables = []
            pio_address = None
            pio_size = None
            int_num = None
            ir_path = None
            hw_config_path = self.hw_config_path
            hw_config_document = self.hw_config_document
            hw_config_source = self.hw_config_source
            profile_name = None
            debug = False

            # Find the name first...
            # Also, find a non-stupid way to find the name first
            for device_dict in acc["Accelerator"]:
                if "Name" in device_dict:
                    name = device_dict["Name"]
            # Parse the rest of the parameters
            for device_dict in acc["Accelerator"]:
                if "PIOSize" in device_dict:
                    pio_address = top_address
                    pio_size = device_dict["PIOSize"] + (
                        64 - (device_dict["PIOSize"] % 64)
                    )
                    top_address = _reserve(
                        top_address,
                        device_dict["PIOSize"],
                        self.config_path,
                        f"accelerator {name}",
                        "PIOSize",
                    )
                    if ((top_address + pio_size) % 64) != 0:
                        print("Acc Error: " + hex(pio_address))
                if "IrPath" in device_dict:
                    ir_path = device_dict["IrPath"]
                if "Profile" in device_dict:
                    profile_name = device_dict["Profile"]
                if "HWPath" in device_dict:
                    hw_config_path = _resolve_path(
                        self.config_path, device_dict["HWPath"]
                    )
                    hw_config_document = (
                        self.document
                        if hw_config_path == self.config_path
                        else None
                    )
                    hw_config_source = self.config_path
                if "PIOMaster" in device_dict:
                    pio_masters.extend(device_dict["PIOMaster"].split(","))
                if "StreamIn" in device_dict:
                    stream_in.extend(device_dict["StreamIn"].split(","))
                if "StreamOut" in device_dict:
                    stream_out.extend(device_dict["StreamOut"].split(","))
                if "LocalSlaves" in device_dict:
                    local_connections.extend(
                        device_dict["LocalSlaves"].split(",")
                    )
                if "InterruptNum" in device_dict:
                    int_num = device_dict["InterruptNum"]
                if "Debug" in device_dict:
                    debug = device_dict["Debug"]
                if "Var" in device_dict:
                    for var in device_dict["Var"]:
                        # Setup the variable's parameters to pass
                        varParams = dict(var)
                        varParams["Address"] = top_address
                        varParams["AccName"] = name
                        if "Characterization" in varParams:
                            varParams["Characterization"] = _resolve_path(
                                self.config_path, varParams["Characterization"]
                            )

                        if varParams["Type"] == "Stream":
                            aligned_inc = int(var["StreamSize"] + 4) + (
                                64 - (int(var["StreamSize"] + 4) % 64)
                            )
                            statusAddress = top_address + aligned_inc
                            varParams["StatusAddress"] = statusAddress

                        # Create and append a new variable
                        variables.append(Variable(**varParams))
                        # Increment the current address based on size
                        if "SPM" in var["Type"]:
                            aligned_inc = int(var["Size"]) + (
                                64 - (int(var["Size"]) % 64)
                            )
                            top_address = _reserve(
                                top_address,
                                var["Size"],
                                self.config_path,
                                f"variable {var['Name']}",
                                "Size",
                            )
                        elif "Stream" in var["Type"]:
                            statusSize = 4
                            aligned_inc = int(var["StreamSize"] + 4) + (
                                64 - (int(var["StreamSize"] + 4) % 64)
                            )
                            status_inc = int(statusSize) + (
                                64 - (int(statusSize) % 64)
                            )
                            data_end = _reserve(
                                top_address,
                                var["StreamSize"] + 4,
                                self.config_path,
                                f"variable {var['Name']}",
                                "StreamSize",
                            )
                            top_address = _reserve(
                                data_end,
                                statusSize,
                                self.config_path,
                                f"variable {var['Name']}",
                                "StreamSize",
                            )
                        elif "RegisterBank" in var["Type"]:
                            aligned_inc = int(var["Size"]) + (
                                64 - (int(var["Size"]) % 64)
                            )
                            top_address = _reserve(
                                top_address,
                                var["Size"],
                                self.config_path,
                                f"variable {var['Name']}",
                                "Size",
                            )
                        elif "Cache" in var["Type"]:
                            # Don't need to change anything for cache
                            top_address = top_address
                        else:
                            # Should never get here... but just in case
                            # throw an exception
                            exceptionString = (
                                "The Variable: "
                                + name
                                + " has an invalid type named: "
                                + self.type
                            )
                            raise Exception(exceptionString)
            # Append accelerator to the cluster
            acc_class.append(
                Accelerator(
                    name=name,
                    pio_masters=pio_masters,
                    local_connections=local_connections,
                    address=pio_address,
                    size=pio_size,
                    stream_in=stream_in,
                    stream_out=stream_out,
                    int_num=int_num,
                    working_dir=working_dir,
                    ir_path=ir_path,
                    profile_name=profile_name,
                    config_path=self.config_path,
                    hw_config_path=hw_config_path,
                    hw_config_document=hw_config_document,
                    hw_config_source=hw_config_source,
                    variables=variables,
                    debug=debug,
                )
            )

        self.accs = acc_class
        self.dmas = dma_class
        self.top_address = top_address


class Accelerator:

    def __init__(
        self,
        name: str,
        pio_masters: str,
        local_connections: str,
        address: int,
        size: int,
        stream_in: str,
        stream_out: str,
        int_num: int,
        working_dir: str,
        ir_path: str,
        profile_name: str,
        config_path: str,
        hw_config_path: str,
        hw_config_document=None,
        hw_config_source: str = None,
        variables=None,
        debug: bool = False,
    ):

        self.name = name.lower()
        self.pio_masters = pio_masters
        self.local_connections = local_connections
        self.address = address
        self.size = size
        self.stream_in = stream_in
        self.stream_out = stream_out
        self.int_num = int_num

        self.working_dir = working_dir
        self.ir_path = ir_path
        self.profile_name = profile_name
        self.config_path = config_path
        self.hw_config_path = hw_config_path
        self.hw_config_document = hw_config_document
        self.hw_config_source = hw_config_source
        self.variables = variables
        self.debug = debug


class StreamDMA:
    def __init__(
        self,
        name: str,
        pio: int,
        pio_masters: str,
        address: int,
        statusAddress: int,
        dmaType: str,
        rd_int: int = None,
        wr_int: int = None,
        size: int = None,
        maxReq: int = None,
    ):
        self.name = name.lower()
        self.pio = pio
        self.pio_masters = pio_masters
        self.size = size
        self.address = address
        self.statusAddress = statusAddress
        self.dmaType = dmaType
        self.rd_int = rd_int
        self.wr_int = wr_int
        self.maxReq = maxReq
        self.pio_masters = [
            "local_bus" if master.lower() == "localbus" else master.lower()
            for master in pio_masters
        ]


class DMA:
    def __init__(
        self,
        name: str,
        pio: int,
        pio_masters: str,
        address: int,
        dmaType: str,
        int_num=None,
        size: int = None,
        maxReq: int = None,
    ):
        self.name = name.lower()
        self.pio = pio
        self.pio_masters = pio_masters
        self.size = size
        self.address = address
        self.dmaType = dmaType
        self.int_num = int_num
        self.maxReq = maxReq

        self.pio_masters = [
            "local_bus" if master.lower() == "localbus" else master.lower()
            for master in pio_masters
        ]


class PortedConnection:
    def __init__(self, conName: str, numPorts: int):
        self.conName = conName
        self.numPorts = int(numPorts)


class Variable:
    def __init__(self, **kwargs):
        # Read the type first
        self.type = kwargs.get("Type")
        self.characterization = kwargs.get("Characterization")
        if self.type == "SPM":
            self.connections = []
            # Read in SPM args
            self.name = kwargs.get("Name")
            self.accName = kwargs.get("AccName")
            self.size = kwargs.get("Size")
            self.ports = kwargs.get("Ports", 1)
            self.address = kwargs.get("Address")
            self.readyMode = kwargs.get("ReadyMode", False)
            self.resetOnRead = kwargs.get("ResetOnRead", True)
            self.readOnInvalid = kwargs.get("ReadOnInvalid", False)
            self.writeOnValid = kwargs.get("WriteOnValid", True)
            # Append the default connection here...
            # probably need to be more elegant
            self.connections.append(PortedConnection(self.accName, self.ports))
            # Append other connections to the connections list
            # Connections were validated when the workload was loaded.
            if "Connections" in kwargs:
                for con, numPorts in workload_config.parse_connections(
                    kwargs["Connections"], None, self.name
                ):
                    self.connections.append(PortedConnection(con, numPorts))
        elif self.type == "Stream":
            # Read in Stream args
            self.name = kwargs.get("Name")
            self.accName = kwargs.get("AccName")
            self.inCon = kwargs.get("InCon")
            self.outCon = kwargs.get("OutCon")
            self.streamSize = kwargs.get("StreamSize")
            self.bufferSize = kwargs.get("BufferSize")
            self.address = kwargs.get("Address")
            self.statusAddress = kwargs.get("StatusAddress")
            # Convert connection definitions to lowercase
            self.inCon = self.inCon.lower()
            self.outCon = self.outCon.lower()
        elif self.type == "RegisterBank":
            self.connections = []
            # Read in SPM args
            self.name = kwargs.get("Name")
            self.accName = kwargs.get("AccName")
            self.size = kwargs.get("Size")
            self.address = kwargs.get("Address")
            # Append the default connection here...
            # probably need to be more elegant
            # The reg_port connects to the owning accelerator only;
            # the workload reader rejects extra Connections.
            self.connections.append(PortedConnection(self.accName, 1))
        elif self.type == "Cache":
            self.name = kwargs.get("Name")
            self.accName = kwargs.get("AccName")
            self.size = kwargs.get("Size")
        else:
            # Throw an exception if we don't know the type
            exceptionString = (
                "The variable: "
                + kwargs.get("Name")
                + " has an invalid type named: "
                + self.type
            )
            raise Exception(exceptionString)


# Raised when the selected Profile is missing or is not a cycle map.
class CycleCountError(Exception):

    def __init__(self, path, profile_name, reason):
        self.path = path
        self.profile_name = profile_name
        self.reason = reason
        scope = "" if profile_name is None else f"profile {profile_name!r}: "
        super().__init__(f"SALAM config: {path}: {scope}{reason}")


def resolve_cycle_counts(
    hw_config_document, hw_config_path, profile_name, hw_config_source=None
):
    if hw_config_document is not None:
        hw_config = hw_config_document.get("hw_config")
    else:
        hw_config = _load_external_hw_config(hw_config_path, hw_config_source)
    return _cycle_counts(hw_config, profile_name, hw_config_path)


def _cycle_counts(hw_config, profile_name, source):
    if not isinstance(hw_config, dict) or profile_name not in hw_config:
        raise CycleCountError(source, profile_name, "profile key is missing")
    node = hw_config[profile_name]
    if node is None:
        return None
    if not isinstance(node, dict) or not isinstance(
        node.get("instructions"), dict
    ):
        raise CycleCountError(
            source, profile_name, "profile is not a null or instruction map"
        )
    workload_config.validate_hw_config({profile_name: node}, source)
    return {
        name: spec["runtime_cycles"]
        for name, spec in node["instructions"].items()
    }


def _load_external_hw_config(path, source=None):
    try:
        documents = [
            document
            for document in workload_config.read_yaml(path)
            if document is not None
        ]
    except workload_config.WorkloadConfigError as exc:
        if not isinstance(exc.__cause__, OSError):
            raise
        cause = exc.__cause__
        raise workload_config.WorkloadConfigError(
            source or path,
            "HWPath",
            "HWPath",
            path,
            f"cannot read HWPath file {path}: {cause.strerror}",
        ) from cause
    if len(documents) != 1 or not isinstance(documents[0], dict):
        raise CycleCountError(
            path, None, "HWPath file must contain one YAML document"
        )
    hw_config = documents[0].get("hw_config")
    if not isinstance(hw_config, dict):
        raise CycleCountError(path, None, "HWPath file has no hw_config")
    workload_config.validate_hw_config(hw_config, path)
    return hw_config
