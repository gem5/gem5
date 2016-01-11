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
#
# This script builds a complete HMC device composed of vault controllers,
# serial links, the main internal crossbar, and an external hmc controller.
#
# - VAULT CONTROLLERS:
#   Instances of the HMC_2500_x32 class with their functionality specified in
#   dram_ctrl.cc
#
# - THE MAIN XBAR:
#   This component is simply an instance of the NoncoherentXBar class, and its
#   parameters are tuned to [2].
#
# - SERIAL LINKS:
#   SerialLink is a simple variation of the Bridge class, with the ability to
#   account for the latency of packet serialization. We assume that the
#   serializer component at the transmitter side does not need to receive the
#   whole packet to start the serialization. But the deserializer waits for
#   the complete packet to check its integrity first.
#   * Bandwidth of the serial links is not modeled in the SerialLink component
#     itself. Instead bandwidth/port of the HMCController has been adjusted to
#     reflect the bandwidth delivered by 1 serial link.
#
# - HMC CONTROLLER:
#   Contains a large buffer (modeled with Bridge) to hide the access latency
#   of the memory cube. Plus it simply forwards the packets to the serial
#   links in a round-robin fashion to balance load among them.
#   * It is inferred from the standard [1] and the literature [3] that serial
#     links share the same address range and packets can travel over any of
#     them so a load distribution mechanism is required among them.

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

    #*****************************SERIAL LINK PARAMETERS**********************
    # Number of serial links [1]
    num_serial_links = Param.Unsigned(4, "Number of serial links")

    # Number of packets (not flits) to store at the request side of the serial
    #  link. This number should be adjusted to achive required bandwidth
    link_buffer_size_req = Param.Unsigned(16, "Number of packets to buffer "
        "at the request side of the serial link")

    # Number of packets (not flits) to store at the response side of the serial
    #  link. This number should be adjusted to achive required bandwidth
    link_buffer_size_rsp = Param.Unsigned(16, "Number of packets to buffer "
        "at the response side of the serial link")

    # Latency of the serial link composed by SER/DES latency (1.6ns [4]) plus
    # the PCB trace latency (3ns Estimated based on [5])
    link_latency = Param.Latency('4.6ns', "Latency of the serial links")

    # Header overhead of the serial links: Header size is 128bits in HMC [1],
    #  and we have 16 lanes, so the overhead is 8 cycles
    link_overhead = Param.Cycles(8, "The number of cycles required to"
        " transmit the packet header over the serial link")

    # Clock frequency of the serial links [1]
    link_frequency = Param.Frequency('10GHz', "Clock Frequency of the serial"
        "links")

    # Number of parallel lanes in each serial link [1]
    num_lanes_per_link =  Param.Unsigned(16, "Number of lanes per each link")

    # Number of serial links [1]
    num_serial_links =  Param.Unsigned(4, "Number of serial links")

    #*****************************HMC CONTROLLER PARAMETERS*******************
    # Number of packets (not flits) to store at the HMC controller. This
    # number should be high enough to be able to hide the high latency of HMC
    ctrl_buffer_size_req = Param.Unsigned(256, "Number of packets to buffer "
        "at the HMC controller (request side)")

    # Number of packets (not flits) to store at the response side of the HMC
    #  controller.
    ctrl_buffer_size_rsp = Param.Unsigned(256, "Number of packets to buffer "
        "at the HMC controller (response side)")

    # Latency of the HMC controller to process the packets
    # (ClockDomain = Host clock domain)
    ctrl_latency = Param.Cycles(4, "The number of cycles required for the "
        " controller to process the packet")

    # Wiring latency from the SoC crossbar to the HMC controller
    ctrl_static_latency = Param.Latency('500ps', "Static latency of the HMC"
        "controller")

    #*****************************PERFORMANCE MONITORING**********************
    # The main monitor behind the HMC Controller
    enable_global_monitor = Param.Bool(True, "The main monitor behind the "
        "HMC Controller")

    # The link performance monitors
    enable_link_monitor = Param.Bool(True, "The link monitors")

# Create an HMC device and attach it to the current system
def config_hmc(options, system):

    system.hmc = HMCSystem()

    system.buffer = Bridge(ranges=system.mem_ranges,
                           req_size=system.hmc.ctrl_buffer_size_req,
                           resp_size=system.hmc.ctrl_buffer_size_rsp,
                           delay=system.hmc.ctrl_static_latency)
    try:
        system.hmc.enable_global_monitor = options.enable_global_monitor
    except:
        pass;

    try:
        system.hmc.enable_link_monitor = options.enable_link_monitor
    except:
        pass;

    system.membus.master = system.buffer.slave

    # The HMC controller (Clock domain is the same as the host)
    system.hmccontroller = HMCController(width=(system.hmc.num_lanes_per_link.
        value * system.hmc.num_serial_links/8),
        frontend_latency=system.hmc.ctrl_latency,
        forward_latency=system.hmc.link_overhead,
        response_latency=system.hmc.link_overhead)

    system.hmccontroller.clk_domain = SrcClockDomain(clock=system.hmc.
        link_frequency, voltage_domain = VoltageDomain(voltage = '1V'))

    # Serial Links
    system.hmc.seriallink =[ SerialLink(ranges = system.mem_ranges,
        req_size=system.hmc.link_buffer_size_req,
        resp_size=system.hmc.link_buffer_size_rsp,
        num_lanes=system.hmc.num_lanes_per_link,
        delay=system.hmc.link_latency)
        for i in xrange(system.hmc.num_serial_links)]

    if system.hmc.enable_link_monitor:
        system.hmc.lmonitor = [ CommMonitor()
        for i in xrange(system.hmc.num_serial_links)]

    # The HMC Crossbar located in its logic-base (LoB)
    system.hmc.xbar = NoncoherentXBar(width = system.hmc.xbar_width,
        frontend_latency=system.hmc.xbar_frontend_latency,
        forward_latency=system.hmc.xbar_forward_latency,
        response_latency=system.hmc.xbar_response_latency )
    system.hmc.xbar.clk_domain = SrcClockDomain(clock =
        system.hmc.xbar_frequency, voltage_domain =
        VoltageDomain(voltage = '1V'))

    if system.hmc.enable_global_monitor:
        system.gmonitor = CommMonitor()
        system.buffer.master = system.gmonitor.slave
        system.gmonitor.master = system.hmccontroller.slave
    else:
        system.hmccontroller.slave = system.buffer.master

    for i in xrange(system.hmc.num_serial_links):
        system.hmccontroller.master = system.hmc.seriallink[i].slave
        system.hmc.seriallink[i].clk_domain = system.hmccontroller.clk_domain;
        if system.hmc.enable_link_monitor:
            system.hmc.seriallink[i].master = system.hmc.lmonitor[i].slave
            system.hmc.lmonitor[i].master = system.hmc.xbar.slave
        else:
            system.hmc.seriallink[i].master = system.hmc.xbar.slave
