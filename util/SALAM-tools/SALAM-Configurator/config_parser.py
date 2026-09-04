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

import yaml


def open_yaml(yml_path: str):
    stream = open(yml_path)
    config = yaml.safe_load_all(stream)
    return config


def parse_yaml(
    parent_config,
    base_address,
    working_dir: str,
    parent_path: str = None,
    hw_path: str = None,
):
    clusters = []
    # Load in each acc cluster and add it to the list
    for cluster_dict in parent_config:
        for list_type, params in cluster_dict.items():
            FOUND_SYS_PATH = False
            FOUND_HW_PATH = False
            FOUND_DEVICE = False
            if list_type == "acc_cluster":
                for param in params:
                    if "SysPath" in param:
                        FOUND_SYS_PATH = True
                        cur_config = open_yaml(
                            yml_path=(working_dir + param["SysPath"])
                        )
                        cur_path = working_dir + param["SysPath"]
                    elif "HWPath" in param:
                        FOUND_HW_PATH = True
                        hw_path = working_dir + param["HWPath"]
                    else:
                        FOUND_DEVICE = True
                        cur_config = parent_config
                        # Use the parent YAML path as the hardware profile
                        # path unless this cluster overrides it with HWPath.
                        if not FOUND_HW_PATH and hw_path is None:
                            hw_path = parent_path
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
                    hw_path=hw_path,
                )
                clusters.extend(temp_cluster)
            elif FOUND_HW_PATH:
                raise Exception(
                    "HW Path should be defined with a System Config file"
                )

        cluster_name = None
        dmas = []
        accs = []
        for list_type, devices in cluster_dict.items():
            if list_type == "acc_cluster":
                for device in devices:
                    if "Name" in device:
                        cluster_name = device["Name"]
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
                hw_config_path=hw_path,
            )
        )
        base_address = clusters[-1].top_address + (
            64 - (int(clusters[-1].top_address) % 64)
        )
        if (int(base_address) % 64) != 0:
            print("Address Alignment Error: " + hex(base_address))
    return base_address, clusters


def load_clusters(
    working_dir, config_name="config.yml", base_address=0x2F000000
):
    if not working_dir.endswith("/"):
        working_dir = working_dir + "/"
    main_yml_path = working_dir + config_name
    config = open_yaml(yml_path=main_yml_path)
    _, clusters = parse_yaml(
        parent_config=config,
        base_address=base_address,
        working_dir=working_dir,
        parent_path=main_yml_path,
    )
    return clusters


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
    ):
        self.name = name
        self.dmas = dmas
        self.accs = accs
        self.base_address = base_address
        self.top_address = base_address
        self.config_path = config_path
        # Do this to point the hardware configuration to the
        # sys config YAML file when HWPath isn't defined
        self.hw_config_path = hw_config_path
        self.process_config(working_dir=working_dir)

    def process_config(self, working_dir):
        dma_class = []
        acc_class = []
        top_address = self.base_address

        # Parse DMAs
        for dma in self.dmas:
            for device_dict in dma["DMA"]:
                # Decide whether the DMA is NonCoherent or Stream
                if "NonCoherent" in device_dict["Type"]:
                    pio_size = 21
                    pio_masters = []
                    if "PIOMaster" in device_dict:
                        pio_masters.extend(device_dict["PIOMaster"].split(","))
                    if "InterruptNum" in device_dict:
                        dma_class.append(
                            DMA(
                                name=device_dict["Name"],
                                pio=pio_size,
                                pio_masters=pio_masters,
                                address=top_address,
                                dmaType=device_dict["Type"],
                                int_num=device_dict["InterruptNum"],
                                size=device_dict["BufferSize"],
                                maxReq=device_dict["MaxReqSize"],
                            )
                        )
                    else:
                        dma_class.append(
                            DMA(
                                name=device_dict["Name"],
                                pio=pio_size,
                                pio_masters=pio_masters,
                                address=top_address,
                                dmaType=device_dict["Type"],
                                size=device_dict["BufferSize"],
                                maxReq=device_dict["MaxReqSize"],
                            )
                        )
                    aligned_inc = int(pio_size) + (64 - (int(pio_size) % 64))
                    top_address = top_address + aligned_inc
                elif "Stream" in device_dict["Type"]:
                    pio_size = 32
                    statusSize = 4
                    pio_masters = []

                    alignedStatusInc = int(statusSize) + (
                        64 - (int(statusSize) % 64)
                    )
                    aligned_inc = int(pio_size) + (64 - (int(pio_size) % 64))

                    statusAddress = top_address + aligned_inc

                    if "PIOMaster" in device_dict:
                        pio_masters.extend(device_dict["PIOMaster"].split(","))
                    # Can come back and get rid of this if/else tree
                    if "ReadInt" in device_dict:
                        if "WriteInt" in device_dict:
                            dma_class.append(
                                StreamDMA(
                                    name=device_dict["Name"],
                                    pio=pio_size,
                                    pio_masters=pio_masters,
                                    address=top_address,
                                    statusAddress=statusAddress,
                                    dmaType=device_dict["Type"],
                                    rd_int=device_dict["ReadInt"],
                                    wr_int=device_dict["WriteInt"],
                                    size=device_dict["BufferSize"],
                                )
                            )
                        else:
                            dma_class.append(
                                StreamDMA(
                                    name=device_dict["Name"],
                                    pio=pio_size,
                                    pio_masters=pio_masters,
                                    address=top_address,
                                    statusAddress=statusAddress,
                                    dmaType=device_dict["Type"],
                                    rd_int=device_dict["ReadInt"],
                                    wr_int=None,
                                    size=device_dict["BufferSize"],
                                )
                            )
                    elif "WriteInt" in device_dict:
                        dma_class.append(
                            StreamDMA(
                                name=device_dict["Name"],
                                pio=pio_size,
                                pio_masters=pio_masters,
                                address=top_address,
                                statusAddress=statusAddress,
                                dmaType=device_dict["Type"],
                                rd_int=None,
                                wr_int=device_dict["WriteInt"],
                                size=device_dict["BufferSize"],
                            )
                        )
                    else:
                        dma_class.append(
                            StreamDMA(
                                name=device_dict["Name"],
                                pio=pio_size,
                                pio_masters=pio_masters,
                                address=top_address,
                                statusAddress=statusAddress,
                                dmaType=device_dict["Type"],
                                rd_int=None,
                                wr_int=None,
                                size=device_dict["BufferSize"],
                            )
                        )

                    # Increment Top Address
                    top_address = top_address + aligned_inc + alignedStatusInc
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
                    top_address = top_address + pio_size
                    if ((top_address + pio_size) % 64) != 0:
                        print("Acc Error: " + hex(pio_address))
                if "IrPath" in device_dict:
                    ir_path = device_dict["IrPath"]
                if "HWPath" in device_dict:
                    hw_config_path = device_dict["HWPath"]
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
                            top_address = top_address + aligned_inc
                        elif "Stream" in var["Type"]:
                            statusSize = 4
                            aligned_inc = int(var["StreamSize"] + 4) + (
                                64 - (int(var["StreamSize"] + 4) % 64)
                            )
                            status_inc = int(statusSize) + (
                                64 - (int(statusSize) % 64)
                            )
                            top_address = (
                                top_address + aligned_inc + status_inc
                            )
                        elif "RegisterBank" in var["Type"]:
                            aligned_inc = int(var["Size"]) + (
                                64 - (int(var["Size"]) % 64)
                            )
                            top_address = top_address + aligned_inc
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
                    config_path=self.config_path,
                    hw_config_path=hw_config_path,
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
        config_path: str,
        hw_config_path: str,
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
        self.config_path = config_path
        self.hw_config_path = hw_config_path
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
        size: int = 64,
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

        for master in self.pio_masters:
            count = 0
            if "localbus" in master.lower():
                pio_masters[count] = "local_bus"
                count += 1


class DMA:
    def __init__(
        self,
        name: str,
        pio: int,
        pio_masters: str,
        address: int,
        dmaType: str,
        int_num=None,
        size: int = 64,
        maxReq: int = 4,
    ):
        self.name = name.lower()
        self.pio = pio
        self.pio_masters = pio_masters
        self.size = size
        self.address = address
        self.dmaType = dmaType
        self.int_num = int_num
        self.maxReq = maxReq

        for master in self.pio_masters:
            count = 0
            if "localbus" in master.lower():
                pio_masters[count] = "local_bus"
                count += 1


class PortedConnection:
    def __init__(self, conName: str, numPorts: int):
        self.conName = conName
        self.numPorts = numPorts


class Variable:
    def __init__(self, **kwargs):
        # Read the type first
        self.type = kwargs.get("Type")
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
            if "Connections" in kwargs:
                for conDef in kwargs.get("Connections").split(","):
                    con, numPorts = conDef.split(":")
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
            self.connections.append(PortedConnection(self.accName, 1))
            # Append other connections to the connections list
            if "Connections" in kwargs:
                for conDef in kwargs.get("Connections").split(","):
                    con, numPorts = conDef.split(":")
                    self.connections.append(PortedConnection(con, numPorts))
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
