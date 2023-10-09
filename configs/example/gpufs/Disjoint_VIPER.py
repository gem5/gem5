# Copyright (c) 2021 Advanced Micro Devices, Inc.
# All rights reserved.
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
from example.gpufs.DisjointNetwork import *
from m5.defines import buildEnv
from m5.objects import *
from m5.util import fatal
from ruby import Ruby
from ruby.GPU_VIPER import *


class DummySystem:
    def __init__(self, mem_ranges):

        self.mem_ctrls = []
        self.mem_ranges = mem_ranges


class Disjoint_VIPER(RubySystem):
    def __init__(self):
        if buildEnv["PROTOCOL"] != "GPU_VIPER":
            fatal("This ruby config only supports the GPU_VIPER protocol")

        super(Disjoint_VIPER, self).__init__()

    def create(self, options, system, piobus, dma_devices):

        # Disjoint network topology
        if "garnet" in options.network:
            self.network_cpu = DisjointGarnet(self)
            self.network_gpu = DisjointGarnet(self)
        else:
            self.network_cpu = DisjointSimple(self)
            self.network_gpu = DisjointSimple(self)

        # Construct CPU controllers
        cpu_dir_nodes = construct_dirs(options, system, self, self.network_cpu)
        (cp_sequencers, cp_cntrl_nodes) = construct_corepairs(
            options, system, self, self.network_cpu
        )

        # Construct GPU controllers
        (tcp_sequencers, tcp_cntrl_nodes) = construct_tcps(
            options, system, self, self.network_gpu
        )
        (sqc_sequencers, sqc_cntrl_nodes) = construct_sqcs(
            options, system, self, self.network_gpu
        )
        (scalar_sequencers, scalar_cntrl_nodes) = construct_scalars(
            options, system, self, self.network_gpu
        )
        tcc_cntrl_nodes = construct_tccs(
            options, system, self, self.network_gpu
        )

        # Construct CPU memories
        Ruby.setup_memory_controllers(system, self, cpu_dir_nodes, options)

        # Construct GPU memories
        (gpu_dir_nodes, gpu_mem_ctrls) = construct_gpudirs(
            options, system, self, self.network_gpu
        )

        # Configure the directories based on which network they are in
        for cpu_dir_node in cpu_dir_nodes:
            cpu_dir_node.CPUonly = True
            cpu_dir_node.GPUonly = False
        for gpu_dir_node in gpu_dir_nodes:
            gpu_dir_node.CPUonly = False
            gpu_dir_node.GPUonly = True

        # Set access backing store if specified
        if options.access_backing_store:
            self.access_backing_store = True

        # Assign the memory controllers to the system
        cpu_abstract_mems = []
        for mem_ctrl in system.mem_ctrls:
            cpu_abstract_mems.append(mem_ctrl.dram)
        system.memories = cpu_abstract_mems

        gpu_abstract_mems = []
        for mem_ctrl in gpu_mem_ctrls:
            gpu_abstract_mems.append(mem_ctrl.dram)
        system.pc.south_bridge.gpu.memories = gpu_abstract_mems

        # Setup DMA controllers
        gpu_dma_types = ["VegaPagetableWalker", "AMDGPUMemoryManager"]

        cpu_dma_ctrls = []
        gpu_dma_ctrls = []
        dma_cntrls = []
        for i, dma_device in enumerate(dma_devices):
            dma_seq = DMASequencer(version=i, ruby_system=self)
            dma_cntrl = DMA_Controller(
                version=i, dma_sequencer=dma_seq, ruby_system=self
            )

            # Handle inconsistently named ports on various DMA devices:
            if not hasattr(dma_device, "type"):
                # IDE doesn't have a .type but seems like everything else does.
                dma_seq.in_ports = dma_device
            elif dma_device.type in gpu_dma_types:
                dma_seq.in_ports = dma_device.port
            else:
                dma_seq.in_ports = dma_device.dma

            if (
                hasattr(dma_device, "type")
                and dma_device.type in gpu_dma_types
            ):
                dma_cntrl.requestToDir = MessageBuffer(buffer_size=0)
                dma_cntrl.requestToDir.out_port = self.network_gpu.in_port
                dma_cntrl.responseFromDir = MessageBuffer(buffer_size=0)
                dma_cntrl.responseFromDir.in_port = self.network_gpu.out_port
                dma_cntrl.mandatoryQueue = MessageBuffer(buffer_size=0)

                gpu_dma_ctrls.append(dma_cntrl)
            else:
                dma_cntrl.requestToDir = MessageBuffer(buffer_size=0)
                dma_cntrl.requestToDir.out_port = self.network_cpu.in_port
                dma_cntrl.responseFromDir = MessageBuffer(buffer_size=0)
                dma_cntrl.responseFromDir.in_port = self.network_cpu.out_port
                dma_cntrl.mandatoryQueue = MessageBuffer(buffer_size=0)

                cpu_dma_ctrls.append(dma_cntrl)

            dma_cntrls.append(dma_cntrl)

        system.dma_cntrls = dma_cntrls

        # Collect CPU and GPU controllers into seperate lists
        cpu_cntrls = cpu_dir_nodes + cp_cntrl_nodes + cpu_dma_ctrls
        gpu_cntrls = (
            tcp_cntrl_nodes
            + sqc_cntrl_nodes
            + scalar_cntrl_nodes
            + tcc_cntrl_nodes
            + gpu_dma_ctrls
            + gpu_dir_nodes
        )

        # Setup number of vnets
        self.number_of_virtual_networks = 11
        self.network_cpu.number_of_virtual_networks = 11
        self.network_gpu.number_of_virtual_networks = 11

        # Set up the disjoint topology
        self.network_cpu.connectCPU(options, cpu_cntrls)
        self.network_gpu.connectGPU(options, gpu_cntrls)

        # Create port proxy for connecting system port. System port is used
        # for loading from outside guest, e.g., binaries like vmlinux.
        system.sys_port_proxy = RubyPortProxy(ruby_system=self)
        system.sys_port_proxy.pio_request_port = piobus.cpu_side_ports
        system.system_port = system.sys_port_proxy.in_ports

        # Only CPU sequencers connect to PIO bus. This acts as the "default"
        # destination for unknown address ranges. PCIe requests fall under
        # this category.
        for i in range(len(cp_sequencers)):
            cp_sequencers[i].pio_request_port = piobus.cpu_side_ports
            cp_sequencers[i].mem_request_port = piobus.cpu_side_ports

            # The CorePairs in MOESI_AMD_Base round up when constructing
            # sequencers, but if the CPU does not exit there would be no
            # sequencer to send a range change, leading to assert.
            if i < options.num_cpus:
                cp_sequencers[i].pio_response_port = piobus.mem_side_ports

        # Setup ruby port. Both CPU and GPU are actually connected here.
        all_sequencers = (
            cp_sequencers + tcp_sequencers + sqc_sequencers + scalar_sequencers
        )
        self._cpu_ports = all_sequencers
        self.num_of_sequencers = len(all_sequencers)
