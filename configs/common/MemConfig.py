# Copyright (c) 2013, 2017, 2020-2021 Arm Limited
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

import m5.objects
from common import ObjectList
from common import HMC


def create_mem_intf(intf, r, i, intlv_bits, intlv_size, xor_low_bit):
    """
    Helper function for creating a single memory controller from the given
    options.  This function is invoked multiple times in config_mem function
    to create an array of controllers.
    """

    import math

    intlv_low_bit = int(math.log(intlv_size, 2))

    # Use basic hashing for the channel selection, and preferably use
    # the lower tag bits from the last level cache. As we do not know
    # the details of the caches here, make an educated guess. 4 MByte
    # 4-way associative with 64 byte cache lines is 6 offset bits and
    # 14 index bits.
    if xor_low_bit:
        xor_high_bit = xor_low_bit + intlv_bits - 1
    else:
        xor_high_bit = 0

    # Create an instance so we can figure out the address
    # mapping and row-buffer size
    interface = intf()

    # Only do this for DRAMs
    if issubclass(intf, m5.objects.DRAMInterface):
        # If the channel bits are appearing after the column
        # bits, we need to add the appropriate number of bits
        # for the row buffer size
        if interface.addr_mapping.value == "RoRaBaChCo":
            # This computation only really needs to happen
            # once, but as we rely on having an instance we
            # end up having to repeat it for each and every
            # one
            rowbuffer_size = (
                interface.device_rowbuffer_size.value
                * interface.devices_per_rank.value
            )

            intlv_low_bit = int(math.log(rowbuffer_size, 2))

    # Also adjust interleaving bits for NVM attached as memory
    # Will have separate range defined with unique interleaving
    if issubclass(intf, m5.objects.NVMInterface):
        # If the channel bits are appearing after the low order
        # address bits (buffer bits), we need to add the appropriate
        # number of bits for the buffer size
        if interface.addr_mapping.value == "RoRaBaChCo":
            # This computation only really needs to happen
            # once, but as we rely on having an instance we
            # end up having to repeat it for each and every
            # one
            buffer_size = interface.per_bank_buffer_size.value

            intlv_low_bit = int(math.log(buffer_size, 2))

    # We got all we need to configure the appropriate address
    # range
    interface.range = m5.objects.AddrRange(
        r.start,
        size=r.size(),
        intlvHighBit=intlv_low_bit + intlv_bits - 1,
        xorHighBit=xor_high_bit,
        intlvBits=intlv_bits,
        intlvMatch=i,
    )
    return interface


def config_mem(options, system):
    """
    Create the memory controllers based on the options and attach them.

    If requested, we make a multi-channel configuration of the
    selected memory controller class by creating multiple instances of
    the specific class. The individual controllers have their
    parameters set such that the address range is interleaved between
    them.
    """

    # Mandatory options
    opt_mem_channels = options.mem_channels

    # Semi-optional options
    # Must have either mem_type or nvm_type or both
    opt_mem_type = getattr(options, "mem_type", None)
    opt_nvm_type = getattr(options, "nvm_type", None)
    if not opt_mem_type and not opt_nvm_type:
        fatal("Must have option for either mem-type or nvm-type, or both")

    # Optional options
    opt_tlm_memory = getattr(options, "tlm_memory", None)
    opt_external_memory_system = getattr(
        options, "external_memory_system", None
    )
    opt_elastic_trace_en = getattr(options, "elastic_trace_en", False)
    opt_mem_ranks = getattr(options, "mem_ranks", None)
    opt_nvm_ranks = getattr(options, "nvm_ranks", None)
    opt_hybrid_channel = getattr(options, "hybrid_channel", False)
    opt_dram_powerdown = getattr(options, "enable_dram_powerdown", None)
    opt_mem_channels_intlv = getattr(options, "mem_channels_intlv", 128)
    opt_xor_low_bit = getattr(options, "xor_low_bit", 0)

    if opt_mem_type == "HMC_2500_1x32":
        HMChost = HMC.config_hmc_host_ctrl(options, system)
        HMC.config_hmc_dev(options, system, HMChost.hmc_host)
        subsystem = system.hmc_dev
        xbar = system.hmc_dev.xbar
    else:
        subsystem = system
        xbar = system.membus

    if opt_tlm_memory:
        system.external_memory = m5.objects.ExternalSlave(
            port_type="tlm_slave",
            port_data=opt_tlm_memory,
            port=system.membus.mem_side_ports,
            addr_ranges=system.mem_ranges,
        )
        system.workload.addr_check = False
        return

    if opt_external_memory_system:
        subsystem.external_memory = m5.objects.ExternalSlave(
            port_type=opt_external_memory_system,
            port_data="init_mem0",
            port=xbar.mem_side_ports,
            addr_ranges=system.mem_ranges,
        )
        subsystem.workload.addr_check = False
        return

    nbr_mem_ctrls = opt_mem_channels

    import math
    from m5.util import fatal

    intlv_bits = int(math.log(nbr_mem_ctrls, 2))
    if 2**intlv_bits != nbr_mem_ctrls:
        fatal("Number of memory channels must be a power of 2")

    if opt_mem_type:
        intf = ObjectList.mem_list.get(opt_mem_type)
    if opt_nvm_type:
        n_intf = ObjectList.mem_list.get(opt_nvm_type)

    nvm_intfs = []
    mem_ctrls = []

    if opt_elastic_trace_en and not issubclass(intf, m5.objects.SimpleMemory):
        fatal(
            "When elastic trace is enabled, configure mem-type as "
            "simple-mem."
        )

    # The default behaviour is to interleave memory channels on 128
    # byte granularity, or cache line granularity if larger than 128
    # byte. This value is based on the locality seen across a large
    # range of workloads.
    intlv_size = max(opt_mem_channels_intlv, system.cache_line_size.value)

    # For every range (most systems will only have one), create an
    # array of memory interfaces and set their parameters to match
    # their address mapping in the case of a DRAM
    range_iter = 0
    for r in system.mem_ranges:
        # As the loops iterates across ranges, assign them alternatively
        # to DRAM and NVM if both configured, starting with DRAM
        range_iter += 1

        for i in range(nbr_mem_ctrls):
            if opt_mem_type and (not opt_nvm_type or range_iter % 2 != 0):
                # Create the DRAM interface
                dram_intf = create_mem_intf(
                    intf, r, i, intlv_bits, intlv_size, opt_xor_low_bit
                )

                # Set the number of ranks based on the command-line
                # options if it was explicitly set
                if (
                    issubclass(intf, m5.objects.DRAMInterface)
                    and opt_mem_ranks
                ):
                    dram_intf.ranks_per_channel = opt_mem_ranks

                # Enable low-power DRAM states if option is set
                if issubclass(intf, m5.objects.DRAMInterface):
                    dram_intf.enable_dram_powerdown = opt_dram_powerdown

                if opt_elastic_trace_en:
                    dram_intf.latency = "1ns"
                    print(
                        "For elastic trace, over-riding Simple Memory "
                        "latency to 1ns."
                    )

                # Create the controller that will drive the interface
                mem_ctrl = dram_intf.controller()

                mem_ctrls.append(mem_ctrl)

            elif opt_nvm_type and (not opt_mem_type or range_iter % 2 == 0):
                nvm_intf = create_mem_intf(
                    n_intf, r, i, intlv_bits, intlv_size, opt_xor_low_bit
                )

                # Set the number of ranks based on the command-line
                # options if it was explicitly set
                if (
                    issubclass(n_intf, m5.objects.NVMInterface)
                    and opt_nvm_ranks
                ):
                    nvm_intf.ranks_per_channel = opt_nvm_ranks

                # Create a controller if not sharing a channel with DRAM
                # in which case the controller has already been created
                if not opt_hybrid_channel:
                    mem_ctrl = m5.objects.HeteroMemCtrl()
                    mem_ctrl.nvm = nvm_intf

                    mem_ctrls.append(mem_ctrl)
                else:
                    nvm_intfs.append(nvm_intf)

    # hook up NVM interface when channel is shared with DRAM + NVM
    for i in range(len(nvm_intfs)):
        mem_ctrls[i].nvm = nvm_intfs[i]

    # Connect the controller to the xbar port
    for i in range(len(mem_ctrls)):
        if opt_mem_type == "HMC_2500_1x32":
            # Connect the controllers to the membus
            mem_ctrls[i].port = xbar[i // 4].mem_side_ports
            # Set memory device size. There is an independent controller
            # for each vault. All vaults are same size.
            mem_ctrls[i].dram.device_size = options.hmc_dev_vault_size
        else:
            # Connect the controllers to the membus
            mem_ctrls[i].port = xbar.mem_side_ports

    subsystem.mem_ctrls = mem_ctrls
