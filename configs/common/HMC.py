# Copyright (c) 2012-2013 ARM Limited
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
#
# Authors: Erfan Azarkhish
#          Abdul Mutaal Ahmad

# A Simplified model of a complete HMC device. Based on:
#  [1] http://www.hybridmemorycube.org/specification-download/
#  [2] High performance AXI-4.0 based interconnect for extensible smart memory
#      cubes(E. Azarkhish et. al)
#  [3] Low-Power Hybrid Memory Cubes With Link Power Management and Two-Level
#      Prefetching (J. Ahn et. al)
#  [4] Memory-centric system interconnect design with Hybrid Memory Cubes
#      (G. Kim et. al)
#  [5] Near Data Processing, Are we there yet? (M. Gokhale)
#      http://www.cs.utah.edu/wondp/gokhale.pdf
#  [6] openHMC - A Configurable Open-Source Hybrid Memory Cube Controller
#      (J. Schmidt)
#  [7] Hybrid Memory Cube performance characterization on data-centric
#      workloads (M. Gokhale)
#
# This script builds a complete HMC device composed of vault controllers,
# serial links, the main internal crossbar, and an external hmc controller.
#
# - VAULT CONTROLLERS:
#   Instances of the HMC_2500_1x32 class with their functionality specified in
#   dram_ctrl.cc
#
# - THE MAIN XBAR:
#   This component is simply an instance of the NoncoherentXBar class, and its
#   parameters are tuned to [2].
#
# - SERIAL LINKS CONTROLLER:
#   SerialLink is a simple variation of the Bridge class, with the ability to
#   account for the latency of packet serialization and controller latency. We
#   assume that the serializer component at the transmitter side does not need
#   to receive the whole packet to start the serialization. But the
#   deserializer waits for the complete packet to check its integrity first.
#
#   * Bandwidth of the serial links is not modeled in the SerialLink component
#     itself.
#
#   * Latency of serial link controller is composed of SerDes latency + link
#     controller
#
#   * It is inferred from the standard [1] and the literature [3] that serial
#     links share the same address range and packets can travel over any of
#     them so a load distribution mechanism is required among them.
#
#   -----------------------------------------
#   | Host/HMC Controller                   |
#   |        ----------------------         |
#   |        |  Link Aggregator   |  opt    |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  Serial Link + Ser | * 4     |
#   |        ----------------------         |
#   |---------------------------------------
#   -----------------------------------------
#   | Device
#   |        ----------------------         |
#   |        |       Xbar         | * 4     |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  Vault Controller  | * 16    |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |     Memory         |         |
#   |        ----------------------         |
#   |---------------------------------------|
#
#   In this version we have present 3 different HMC archiecture along with
#   alongwith their corresponding test script.
#
#   same: It has 4 crossbars in HMC memory. All the crossbars are connected
#   to each other, providing complete memory range. This archicture also covers
#   the added latency for sending a request to non-local vault(bridge in b/t
#   crossbars). All the 4 serial links can access complete memory. So each
#   link can be connected to separate processor.
#
#   distributed: It has 4 crossbars inside the HMC. Crossbars are not
#   connected.Through each crossbar only local vaults can be accessed. But to
#   support this architecture we need a crossbar between serial links and
#   processor.
#
#   mixed: This is a hybrid architecture. It has 4 crossbars inside the HMC.
#   2 Crossbars are connected to only local vaults. From other 2 crossbar, a
#   request can be forwarded to any other vault.

import optparse

import m5
from m5.objects import *

# A single Hybrid Memory Cube (HMC)
class HMCSystem(SubSystem):
    #*****************************CROSSBAR PARAMETERS*************************
    # Flit size of the main interconnect [1]
    xbar_width = Param.Unsigned(32, "Data width of the main XBar (Bytes)")

    # Clock frequency of the main interconnect [1]
    # This crossbar, is placed on the logic-based of the HMC and it has its
    # own voltage and clock domains, different from the DRAM dies or from the
    # host.
    xbar_frequency = Param.Frequency('1GHz', "Clock Frequency of the main "
        "XBar")

    # Arbitration latency of the HMC XBar [1]
    xbar_frontend_latency = Param.Cycles(1, "Arbitration latency of the XBar")

    # Latency to forward a packet via the interconnect [1](two levels of FIFOs
    # at the input and output of the inteconnect)
    xbar_forward_latency = Param.Cycles(2, "Forward latency of the XBar")

    # Latency to forward a response via the interconnect [1](two levels of
    # FIFOs at the input and output of the inteconnect)
    xbar_response_latency = Param.Cycles(2, "Response latency of the XBar")

    # number of cross which connects 16 Vaults to serial link[7]
    number_mem_crossbar  = Param.Unsigned(4, "Number of crossbar in HMC"
            )

    #*****************************SERIAL LINK PARAMETERS***********************
    # Number of serial links controllers [1]
    num_links_controllers = Param.Unsigned(4, "Number of serial links")

    # Number of packets (not flits) to store at the request side of the serial
    #  link. This number should be adjusted to achive required bandwidth
    link_buffer_size_req = Param.Unsigned(10, "Number of packets to buffer "
        "at the request side of the serial link")

    # Number of packets (not flits) to store at the response side of the serial
    #  link. This number should be adjusted to achive required bandwidth
    link_buffer_size_rsp = Param.Unsigned(10, "Number of packets to buffer "
        "at the response side of the serial link")

    # Latency of the serial link composed by SER/DES latency (1.6ns [4]) plus
    # the PCB trace latency (3ns Estimated based on [5])
    link_latency = Param.Latency('4.6ns', "Latency of the serial links")

    # Clock frequency of the each serial link(SerDes) [1]
    link_frequency = Param.Frequency('10GHz', "Clock Frequency of the serial"
        "links")

    # Clock frequency of serial link Controller[6]
    # clk_hmc[Mhz]= num_lanes_per_link * lane_speed [Gbits/s] /
    # data_path_width * 10^6
    # clk_hmc[Mhz]= 16 * 10 Gbps / 256 * 10^6 = 625 Mhz
    link_controller_frequency = Param.Frequency('625MHz',
            "Clock Frequency of the link controller")

    # Latency of the serial link controller to process the packets[1][6]
    # (ClockDomain = 625 Mhz )
    # used here for calculations only
    link_ctrl_latency = Param.Cycles(4, "The number of cycles required for the"
        "controller to process the packet")

    # total_ctrl_latency = link_ctrl_latency + link_latency
    # total_ctrl_latency = 4(Cycles) * 1.6 ns +  4.6 ns
    total_ctrl_latency = Param.Latency('11ns', "The latency experienced by"
            "every packet regardless of size of packet")

    # Number of parallel lanes in each serial link [1]
    num_lanes_per_link = Param.Unsigned( 16, "Number of lanes per each link")

    # Number of serial links [1]
    num_serial_links = Param.Unsigned(4, "Number of serial links")

    # speed of each lane of serial link - SerDes serial interface 10 Gb/s
    serial_link_speed = Param.UInt64(10, "Gbs/s speed of each lane of"
            "serial link")

   #*****************************PERFORMANCE MONITORING************************
    # The main monitor behind the HMC Controller
    enable_global_monitor = Param.Bool(False, "The main monitor behind the "
        "HMC Controller")

    # The link performance monitors
    enable_link_monitor = Param.Bool(False, "The link monitors" )

    # link aggregator enable - put a cross between buffers & links
    enable_link_aggr = Param.Bool(False, "The crossbar between port and "
        "Link Controller")

    enable_buff_div  = Param.Bool(True, "Memory Range of Buffer is"
            "divided between total range")

   #*****************************HMC ARCHITECTURE ************************
    # Memory chunk for 16 vault - numbers of vault / number of crossbars
    mem_chunk = Param.Unsigned(4, "Chunk of memory range for each cross bar "
            "in arch 0")

    # size of req buffer within crossbar, used for modelling extra latency
    # when the reuqest go to non-local vault
    xbar_buffer_size_req = Param.Unsigned(10, "Number of packets to buffer "
        "at the request side of the crossbar")

    # size of response buffer within crossbar, used for modelling extra latency
    # when the response received from non-local vault
    xbar_buffer_size_resp = Param.Unsigned(10, "Number of packets to buffer "
        "at the response side of the crossbar")

# configure host system with Serial Links
def config_host_hmc(options, system):

    system.hmc_host=HMCSystem()

    try:
        system.hmc_host.enable_global_monitor = options.enable_global_monitor
    except:
        pass;

    try:
        system.hmc_host.enable_link_monitor = options.enable_link_monitor
    except:
        pass;

    # Serial link Controller with 16 SerDes links at 10 Gbps
    # with serial link ranges w.r.t to architecture
    system.hmc_host.seriallink = [SerialLink(ranges = options.ser_ranges[i],
        req_size=system.hmc_host.link_buffer_size_req,
        resp_size=system.hmc_host.link_buffer_size_rsp,
        num_lanes=system.hmc_host.num_lanes_per_link,
        link_speed=system.hmc_host.serial_link_speed,
        delay=system.hmc_host.total_ctrl_latency)
        for i in xrange(system.hmc_host.num_serial_links)]

    # enable global monitor
    if system.hmc_host.enable_global_monitor:
        system.hmc_host.lmonitor = [ CommMonitor()
        for i in xrange(system.hmc_host.num_serial_links)]

    # set the clock frequency for serial link
    for i in xrange(system.hmc_host.num_serial_links):
        system.hmc_host.seriallink[i].clk_domain = SrcClockDomain(clock=system.
                hmc_host.link_controller_frequency, voltage_domain=
                VoltageDomain(voltage = '1V'))

    # Connect membus/traffic gen to Serial Link Controller for differrent HMC
    # architectures
    if options.arch == "distributed":
        for i in xrange(system.hmc_host.num_links_controllers):
            if system.hmc_host.enable_global_monitor:
                system.membus.master = system.hmc_host.lmonitor[i].slave
                system.hmc_host.lmonitor[i].master = \
                    system.hmc_host.seriallink[i].slave
            else:
                system.membus.master = system.hmc_host.seriallink[i].slave
    if options.arch == "mixed":
        if system.hmc_host.enable_global_monitor:
            system.membus.master = system.hmc_host.lmonitor[0].slave
            system.hmc_host.lmonitor[0].master = \
                system.hmc_host.seriallink[0].slave

            system.membus.master = system.hmc_host.lmonitor[1].slave
            system.hmc_host.lmonitor[1].master = \
                system.hmc_host.seriallink[1].slave

            system.tgen[2].port = system.hmc_host.lmonitor[2].slave
            system.hmc_host.lmonitor[2].master = \
                 system.hmc_host.seriallink[2].slave

            system.tgen[3].port = system.hmc_host.lmonitor[3].slave
            system.hmc_host.lmonitor[3].master = \
                system.hmc_host.seriallink[3].slave
        else:
            system.membus.master = system.hmc_host.seriallink[0].slave
            system.membus.master = system.hmc_host.seriallink[1].slave
            system.tgen[2].port = system.hmc_host.seriallink[2].slave
            system.tgen[3].port = system.hmc_host.seriallink[3].slave
    if options.arch == "same" :
        for i in xrange(system.hmc_host.num_links_controllers):
            if system.hmc_host.enable_global_monitor:
                system.tgen[i].port = system.hmc_host.lmonitor[i].slave
                system.hmc_host.lmonitor[i].master = \
                    system.hmc_host.seriallink[i].slave
            else:
                system.tgen[i].port = system.hmc_host.seriallink[i].slave

    return system

# Create an HMC device and attach it to the current system
def config_hmc(options, system, hmc_host):

    # Create HMC device
    system.hmc_dev = HMCSystem()

    # Global monitor
    try:
        system.hmc_dev.enable_global_monitor = options.enable_global_monitor
    except:
        pass;

    try:
        system.hmc_dev.enable_link_monitor = options.enable_link_monitor
    except:
        pass;


    if system.hmc_dev.enable_link_monitor:
        system.hmc_dev.lmonitor = [ CommMonitor()
        for i in xrange(system.hmc_dev.num_links_controllers)]

    # 4 HMC Crossbars located in its logic-base (LoB)
    system.hmc_dev.xbar = [ NoncoherentXBar(width=system.hmc_dev.xbar_width,
        frontend_latency=system.hmc_dev.xbar_frontend_latency,
        forward_latency=system.hmc_dev.xbar_forward_latency,
        response_latency=system.hmc_dev.xbar_response_latency )
        for i in xrange(system.hmc_host.number_mem_crossbar)]

    for i in xrange(system.hmc_dev.number_mem_crossbar):
        system.hmc_dev.xbar[i].clk_domain = SrcClockDomain(
                clock=system.hmc_dev.xbar_frequency,voltage_domain=
                VoltageDomain(voltage='1V'))

    # Attach 4 serial link to 4 crossbar/s
    for i in xrange(system.hmc_dev.num_serial_links):
        if system.hmc_dev.enable_link_monitor:
            system.hmc_host.seriallink[i].master = \
                system.hmc_dev.lmonitor[i].slave
            system.hmc_dev.lmonitor[i].master = system.hmc_dev.xbar[i].slave
        else:
            system.hmc_host.seriallink[i].master = system.hmc_dev.xbar[i].slave

    # Connecting xbar with each other for request arriving at the wrong xbar,
    # then it will be forward to correct xbar. Bridge is used to connect xbars
    if options.arch == "same":
        numx = len(system.hmc_dev.xbar)

        # create a list of buffers
        system.hmc_dev.buffers = [ Bridge(
            req_size=system.hmc_dev.xbar_buffer_size_req,
            resp_size=system.hmc_dev.xbar_buffer_size_resp)
            for i in xrange(numx * (system.hmc_dev.mem_chunk - 1))]

        # Buffer iterator
        it = iter(range(len(system.hmc_dev.buffers)))

        # necesarry to add system_port to one of the xbar
        system.system_port = system.hmc_dev.xbar[3].slave

        # iterate over all the crossbars and connect them as required
        for i in range(numx):
            for j in range(numx):
                # connect xbar to all other xbars except itself
                if i != j:
                    # get the next index of buffer
                    index = it.next()

                    # Change the default values for ranges of bridge
                    system.hmc_dev.buffers[index].ranges = system.mem_ranges[
                            j * int(system.hmc_dev.mem_chunk):
                            (j + 1) * int(system.hmc_dev.mem_chunk)]

                    # Connect the bridge between corssbars
                    system.hmc_dev.xbar[i].master = system.hmc_dev.buffers[
                            index].slave
                    system.hmc_dev.buffers[
                            index].master = system.hmc_dev.xbar[j].slave
                else:
                    # Don't connect the xbar to itself
                    pass

    # Two crossbars are connected to all other crossbars-Other 2 vault
    # can only direct traffic to it local vaults
    if options.arch == "mixed":

        system.hmc_dev.buffer30 = Bridge(ranges=system.mem_ranges[0:4])
        system.hmc_dev.xbar[3].master = system.hmc_dev.buffer30.slave
        system.hmc_dev.buffer30.master = system.hmc_dev.xbar[0].slave

        system.hmc_dev.buffer31 = Bridge(ranges=system.mem_ranges[4:8])
        system.hmc_dev.xbar[3].master = system.hmc_dev.buffer31.slave
        system.hmc_dev.buffer31.master = system.hmc_dev.xbar[1].slave

        system.hmc_dev.buffer32 = Bridge(ranges=system.mem_ranges[8:12])
        system.hmc_dev.xbar[3].master = system.hmc_dev.buffer32.slave
        system.hmc_dev.buffer32.master = system.hmc_dev.xbar[2].slave


        system.hmc_dev.buffer20 = Bridge(ranges=system.mem_ranges[0:4])
        system.hmc_dev.xbar[2].master = system.hmc_dev.buffer20.slave
        system.hmc_dev.buffer20.master = system.hmc_dev.xbar[0].slave

        system.hmc_dev.buffer21 = Bridge(ranges=system.mem_ranges[4:8])
        system.hmc_dev.xbar[2].master = system.hmc_dev.buffer21.slave
        system.hmc_dev.buffer21.master = system.hmc_dev.xbar[1].slave

        system.hmc_dev.buffer23 = Bridge(ranges=system.mem_ranges[12:16])
        system.hmc_dev.xbar[2].master = system.hmc_dev.buffer23.slave
        system.hmc_dev.buffer23.master = system.hmc_dev.xbar[3].slave

