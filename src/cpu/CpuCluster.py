# Copyright (c) 2022-2023 Arm Limited
# All rights reserved.
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
from m5.objects.SubSystem import SubSystem
from m5.params import *


class CpuCluster(SubSystem):
    type = "CpuCluster"
    cxx_header = "cpu/cluster.hh"
    cxx_class = "gem5::CpuCluster"

    _NUM_CPUS = 0
    _NUM_CLUSTERS = 0

    voltage_domain = Param.VoltageDomain("Voltage domain")
    clk_domain = Param.ClockDomain("Clock domain")

    def __iter__(self):
        return iter(self.cpus)

    def __len__(self):
        return len(self.cpus)

    def generate_cpus(self, cpu_type: "BaseCPU", num_cpus: int):
        """
        Instantiates the cpus within the cluster provided
        theit type and their number.

        :param cpu_type: The cpu class
        :param num_cpus: The number of cpus within the cluster
        """
        self.cpus = [
            cpu_type(
                cpu_id=CpuCluster._NUM_CPUS + idx,
                clk_domain=self.clk_domain,
            )
            for idx in range(num_cpus)
        ]

        for cpu in self.cpus:
            cpu.createThreads()
            cpu.createInterruptController()
            cpu.socket_id = CpuCluster._NUM_CLUSTERS

        # "Register" the cluster/cpus by augmenting the
        # class variables
        CpuCluster._NUM_CPUS += num_cpus
        CpuCluster._NUM_CLUSTERS += 1

    def connect(self, membus: "SystemXBar"):
        """
        Connects every cpu within the cluster with the
        provided bus

        :param membus: The system crossbar
        """
        for cpu in self.cpus:
            cpu.connectBus(membus)

    def memory_mode(self) -> "MemoryMode":
        return type(self.cpus[0]).memory_mode()

    def require_caches(self) -> bool:
        return type(self.cpus[0]).require_caches()
