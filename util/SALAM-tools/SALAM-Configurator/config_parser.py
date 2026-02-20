# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


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
                                int_num=device_dict["BufferSize"],
                                size=device_dict["MaxReqSize"],
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

    def genConfig(self):
        lines = []
        # Need to add some customization here. Consider this a placeholder
        # Also need to edit AccCluster.py's addresses to match the gem5
        # supported ones
        lines.append(
            "def build" + self.name + "(options, system, clstr):" + "\n"
        )
        lines.append("	local_low = " + hex(self.base_address))
        lines.append("	local_high = " + hex(self.top_address))
        lines.append("	local_range = AddrRange(local_low, local_high)")
        lines.append(
            "	external_range = [AddrRange(0x00000000, local_low-1),"
            " AddrRange(local_high+1, 0xFFFFFFFF)]"
        )
        lines.append(
            "	system.iobus.mem_side_ports = clstr.local_bus.cpu_side_ports"
        )
        # Need to define l2coherency in the YAML file?
        lines.append(
            "	clstr._connect_caches(system, options, l2coherent=False)"
        )
        lines.append("	gic = system.realview.gic")
        lines.append("")

        return lines


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

    def genDefinition(self):
        lines = []
        lines.append("# " + self.name + " Definition")
        lines.append("acc = " + '"' + self.name + '"')
        lines.append(
            "ir = " + '"' + self.working_dir + "/" + self.ir_path + '"'
        )
        lines.append("hw_config = " '"' + self.hw_config_path + '"')

        # Add interrupt number if it exists
        if self.int_num is not None:
            lines.append(
                "clstr."
                + self.name
                + " = CommInterface(devicename=acc, gic=gic, pio_addr="
                + str(hex(self.address))
                + ", pio_size="
                + str(self.size)
                + ", int_num="
                + str(self.int_num)
                + ")"
            )
        else:
            lines.append(
                "clstr."
                + self.name
                + " = CommInterface(devicename=acc, gic=gic, pio_addr="
                + str(hex(self.address))
                + ", pio_size="
                + str(self.size)
                + ")"
            )

        lines.append("AccConfig(clstr." + self.name + ", ir, hw_config)")
        lines.append("")

        return lines

    def genConfig(self):
        lines = []

        lines.append("# " + self.name + " Config")

        for connection in self.local_connections:
            if "LocalBus" in connection:
                lines.append(
                    "clstr."
                    + self.name
                    + ".local = clstr.local_bus.cpu_side_ports"
                )
            else:
                lines.append(
                    "clstr."
                    + self.name
                    + ".local = clstr."
                    + connection.lower()
                    + ".pio"
                )

        # Assign PIO Masters
        for master in self.pio_masters:
            if "LocalBus" in master:
                lines.append(
                    "clstr."
                    + self.name
                    + ".pio = clstr.local_bus.mem_side_ports"
                )
            else:
                assert False, "Shouldn't be here?"
                # lines.append("clstr." + self.name + ".pio " +
                #              "=" " clstr." + i + ".local")
        # Add StreamIn
        for inCon in self.stream_in:
            lines.append(
                "clstr."
                + self.name
                + ".stream = clstr."
                + inCon.lower()
                + ".stream_in"
            )
        # Add StreamOut
        for outCon in self.stream_out:
            lines.append(
                "clstr."
                + self.name
                + ".stream = clstr."
                + outCon.lower()
                + ".stream_out"
            )

        lines.append(
            "clstr." + self.name + ".enable_debug_msgs = " + str(self.debug)
        )
        lines.append("")

        # Add scratchpad variables
        for var in self.variables:
            # Have the variable create its config
            lines = var.genConfig(lines)
            lines.append("")
        # Return finished config portion
        return lines


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

    # Probably could apply the style used here in other genConfigs

    def genConfig(self):
        lines = []
        dmaPath = "clstr." + self.name + "."
        # Need to fix max_pending?
        lines.append("# Stream DMA")
        lines.append(
            "clstr."
            + self.name
            + " = StreamDma(pio_addr="
            + hex(self.address)
            + ", status_addr="
            + hex(self.statusAddress)
            + ", pio_size = "
            + str(self.pio)
            + ", gic=gic, max_pending = "
            + str(self.pio)
            + ")"
        )
        lines.append(
            dmaPath
            + "stream_addr = "
            + hex(self.address)
            + " + "
            + str(self.pio)
        )
        lines.append(dmaPath + "stream_size = " + str(self.size))
        lines.append(dmaPath + "pio_delay = '1ns'")
        if self.rd_int != None:
            lines.append(dmaPath + "rd_int = " + str(self.rd_int))
        if self.wr_int != None:
            lines.append(dmaPath + "wr_int = " + str(self.wr_int))
        lines.append(
            "clstr." + self.name + ".dma = clstr.coherency_bus.cpu_side_ports"
        )
        if self.pio_masters is not None:
            for master in self.pio_masters:
                lines.append(
                    "clstr."
                    + master.lower()
                    + ".mem_side_ports = clstr."
                    + self.name
                    + ".pio"
                )
        lines.append("")

        return lines


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

    # Probably could apply the style used here in other genConfigs

    def genConfig(self):
        lines = []
        dmaPath = "clstr." + self.name + "."
        systemPath = "clstr."
        lines.append("# Noncoherent DMA")
        lines.append(
            "clstr."
            + self.name
            + " = NoncoherentDma(pio_addr="
            + hex(self.address)
            + ", pio_size = "
            + str(self.pio)
            + ", gic=gic, int_num="
            + str(self.int_num)
            + ")"
        )
        lines.append(
            dmaPath
            + "cluster_dma = "
            + systemPath
            + "local_bus.cpu_side_ports"
        )
        lines.append(dmaPath + "max_req_size = " + str(self.maxReq))
        lines.append(dmaPath + "buffer_size = " + str(self.size))
        lines.append(
            "clstr." + self.name + ".dma = clstr.coherency_bus.cpu_side_ports"
        )
        if self.pio_masters is not None:
            for master in self.pio_masters:
                lines.append(
                    "clstr."
                    + master.lower()
                    + ".mem_side_ports = clstr."
                    + self.name
                    + ".pio"
                )
        lines.append("")

        return lines


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

    def genConfig(self, lines):
        # Add new variable configs here
        # Stream Buffer Variable
        if self.type == "Stream":
            lines.append("# " + self.name + " (Stream Variable)")
            lines.append("addr = " + hex(self.address))
            lines.append(
                "clstr."
                + self.name.lower()
                + " = StreamBuffer(stream_address = addr, status_address= "
                + hex(self.statusAddress)
                + ", stream_size = "
                + str(self.streamSize)
                + ", buffer_size = "
                + str(self.bufferSize)
                + ")"
            )
            lines.append(
                "clstr."
                + self.inCon
                + ".stream = "
                + "clstr."
                + self.name.lower()
                + ".stream_in"
            )
            lines.append(
                "clstr."
                + self.outCon
                + ".stream = "
                + "clstr."
                + self.name.lower()
                + ".stream_out"
            )
            lines.append("")
        # Scratchpad Memory
        elif self.type == "SPM":
            lines.append("# " + self.name + " (Variable)")
            lines.append("addr = " + hex(self.address))
            lines.append(
                "spmRange = AddrRange(addr, addr + " + hex(self.size) + ")"
            )
            # When appending convert all connections to lowercase
            # for standardization
            lines.append(
                "clstr."
                + self.name.lower()
                + " = ScratchpadMemory(range = spmRange)"
            )
            # Probably need to add table and read mode to the YAML File
            lines.append(
                "clstr."
                + self.name.lower()
                + "."
                + "conf_table_reported = False"
            )
            lines.append(
                "clstr."
                + self.name.lower()
                + "."
                + "ready_mode = "
                + str(self.readyMode)
            )
            lines.append(
                "clstr."
                + self.name.lower()
                + "."
                + "reset_on_scratchpad_read = "
                + str(self.resetOnRead)
            )
            lines.append(
                "clstr."
                + self.name.lower()
                + "."
                + "read_on_invalid = "
                + str(self.readOnInvalid)
            )
            lines.append(
                "clstr."
                + self.name.lower()
                + "."
                + "write_on_valid = "
                + str(self.writeOnValid)
            )
            lines.append(
                "clstr."
                + self.name.lower()
                + "."
                + "port"
                + " = "
                + "clstr.local_bus.mem_side_ports"
            )
            for con in self.connections:
                lines.append("")
                lines.append(
                    "# Connecting " + self.name + " to " + con.conName
                )
                lines.append("for i in range(" + str(con.numPorts) + "):")
                lines.append(
                    "	clstr."
                    + con.conName.lower()
                    + ".spm = "
                    + "clstr."
                    + self.name.lower()
                    + ".spm_ports"
                )
        # RegisterBank
        elif self.type == "RegisterBank":
            lines.append("# " + self.name + " (Variable)")
            lines.append("addr = " + hex(self.address))
            lines.append(
                "regRange = AddrRange(addr, addr + " + hex(self.size) + ")"
            )
            # When appending convert all connections to lowercase
            # for standardization
            lines.append(
                "clstr."
                + self.name.lower()
                + " = RegisterBank(range = regRange)"
            )
            lines.append(
                "clstr."
                + self.name.lower()
                + "."
                + "load_port"
                + " = "
                + "clstr.local_bus.mem_side_ports"
            )
            for con in self.connections:
                lines.append("")
                lines.append(
                    "# Connecting " + self.name + " to " + con.conName
                )
                lines.append(
                    "clstr."
                    + con.conName.lower()
                    + ".reg = "
                    + "clstr."
                    + self.name.lower()
                    + ".reg_port"
                )
        # L1 Cache, need to add L2 still...
        elif self.type == "Cache":
            lines.append("# " + self.name + " (Cache)")
            lines.append(
                "clstr."
                + self.name
                + " = L1Cache(size = '"
                + str(self.size)
                + "B')"
            )
            lines.append(
                "clstr."
                + self.name
                + ".mem_side = clstr.coherency_bus.cpu_side_ports"
            )
            lines.append(
                "clstr."
                + self.name
                + ".cpu_side = clstr."
                + self.accName
                + ".local"
            )
        else:
            # Should never get here... but just in case throw an exception
            exceptionString = (
                "The variable: "
                + self.name
                + " has an invalid type named: "
                + self.type
            )
            raise Exception(exceptionString)
        return lines
