# Copyright (c) 2012-2021 Arm Limited
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
# Copyright (c) 2013 Amin Farmahini-Farahani
# Copyright (c) 2015 University of Kaiserslautern
# Copyright (c) 2015 The University of Bologna
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

"""Interfaces for HMC memory devices

These memory "interfaces" contain the timing, energy, etc parameters for each
memory type and are usually based on datasheets for the memory devices.

You can use these interfaces in the MemCtrl object as the ``dram`` timing
interface.

Note that HMC is configured differently than some other DRAM interfaces.
"""


from m5.objects import MemCtrl
from m5.objects.DRAMInterface import DDR3_1600_8x8


class HMC_2500_1x32(DDR3_1600_8x8):
    """
    A single HMC-2500 x32 model based on:
    [1] DRAMSpec: a high-level DRAM bank modelling tool
    developed at the University of Kaiserslautern. This high level tool
    uses RC (resistance-capacitance) and CV (capacitance-voltage) models to
    estimate the DRAM bank latency and power numbers.
    [2] High performance AXI-4.0 based interconnect for extensible smart memory
    cubes (E. Azarkhish et. al).
    Assumed for the HMC model is a 30 nm technology node.
    The modelled HMC consists of 4 Gbit layers which sum up to 2GiB of memory
    (4 layers).
    Each layer has 16 vaults and each vault consists of 2 banks per layer.
    In order to be able to use the same controller used for 2D DRAM generations
    for HMC, the following analogy is done:

    Channel (DDR) => Vault (HMC)

    device_size (DDR) => size of a single layer in a vault

    ranks per channel (DDR) => number of layers

    banks per rank (DDR) => banks per layer

    devices per rank (DDR) => devices per layer ( 1 for HMC).

    The parameters for which no input is available are inherited from the DDR3
    configuration.

    This configuration includes the latencies from the DRAM to the logic layer
    of the HMC.
    """

    # size of device
    # two banks per device with each bank 4MiB [2]
    device_size = "8MiB"

    # 1x32 configuration, 1 device with 32 TSVs [2]
    device_bus_width = 32

    # HMC is a BL8 device [2]
    burst_length = 8

    # Each device has a page (row buffer) size of 256 bytes [2]
    device_rowbuffer_size = "256B"

    # 1x32 configuration, so 1 device [2]
    devices_per_rank = 1

    # 4 layers so 4 ranks [2]
    ranks_per_channel = 4

    # HMC has 2 banks per layer [2]
    # Each layer represents a rank. With 4 layers and 8 banks in total, each
    # layer has 2 banks; thus 2 banks per rank.
    banks_per_rank = 2

    # 1250 MHz [2]
    tCK = "0.8ns"

    # 8 beats across an x32 interface translates to 4 clocks @ 1250 MHz
    tBURST = "3.2ns"

    # Values using DRAMSpec HMC model [1]
    tRCD = "10.2ns"
    tCL = "9.9ns"
    tRP = "7.7ns"
    tRAS = "21.6ns"

    # tRRD depends on the power supply network for each vendor.
    # We assume a tRRD of a double bank approach to be equal to 4 clock
    # cycles (Assumption)
    tRRD = "3.2ns"

    # activation limit is set to 0 since there are only 2 banks per vault
    # layer.
    activation_limit = 0

    # Values using DRAMSpec HMC model [1]
    tRFC = "59ns"
    tWR = "8ns"
    tRTP = "4.9ns"

    # Default different rank bus delay assumed to 1 CK for TSVs, @1250 MHz =
    # 0.8 ns (Assumption)
    tCS = "0.8ns"

    # Value using DRAMSpec HMC model [1]
    tREFI = "3.9us"

    # The default page policy in the vault controllers is simple closed page
    # [2] nevertheless 'close' policy opens and closes the row multiple times
    # for bursts largers than 32Bytes. For this reason we use 'close_adaptive'
    page_policy = "close_adaptive"

    # RoCoRaBaCh resembles the default address mapping in HMC
    addr_mapping = "RoCoRaBaCh"

    # These parameters do not directly correlate with buffer_size in real
    # hardware. Nevertheless, their value has been tuned to achieve a
    # bandwidth similar to the cycle-accurate model in [2]
    write_buffer_size = 32
    read_buffer_size = 32

    def controller(self):
        """
        Instantiate the memory controller and bind it to
        the current interface.
        """
        controller = MemCtrl(
            min_writes_per_switch=8,
            static_backend_latency="4ns",
            static_frontend_latency="4ns",
        )
        controller.dram = self
        return controller
