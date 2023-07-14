# Copyright (c) 2012-2020 ARM Limited
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

from m5.params import *
from m5.proxy import *
from m5.objects.QoSMemCtrl import *
from m5.citations import add_citation

# Enum for memory scheduling algorithms, currently First-Come
# First-Served and a First-Row Hit then First-Come First-Served
class MemSched(Enum):
    vals = ["fcfs", "frfcfs"]


# MemCtrl is a single-channel single-ported Memory controller model
# that aims to model the most important system-level performance
# effects of a memory controller, interfacing with media specific
# interfaces
class MemCtrl(QoSMemCtrl):
    type = "MemCtrl"
    cxx_header = "mem/mem_ctrl.hh"
    cxx_class = "gem5::memory::MemCtrl"

    # single-ported on the system interface side, instantiate with a
    # bus in front of the controller for multiple ports
    port = ResponsePort("This port responds to memory requests")

    # Interface to memory media
    dram = Param.MemInterface(
        "Memory interface, can be a DRAMor an NVM interface "
    )

    # read and write buffer depths are set in the interface
    # the controller will read these values when instantiated

    # threshold in percent for when to forcefully trigger writes and
    # start emptying the write buffer
    write_high_thresh_perc = Param.Percent(85, "Threshold to force writes")

    # threshold in percentage for when to start writes if the read
    # queue is empty
    write_low_thresh_perc = Param.Percent(50, "Threshold to start writes")

    # minimum write bursts to schedule before switching back to reads
    min_writes_per_switch = Param.Unsigned(
        16, "Minimum write bursts before switching to reads"
    )

    # minimum read bursts to schedule before switching back to writes
    min_reads_per_switch = Param.Unsigned(
        16, "Minimum read bursts before switching to writes"
    )

    # scheduler, address map and page policy
    mem_sched_policy = Param.MemSched("frfcfs", "Memory scheduling policy")

    # pipeline latency of the controller and PHY, split into a
    # frontend part and a backend part, with reads and writes serviced
    # by the queues only seeing the frontend contribution, and reads
    # serviced by the memory seeing the sum of the two
    static_frontend_latency = Param.Latency("10ns", "Static frontend latency")
    static_backend_latency = Param.Latency("10ns", "Static backend latency")

    command_window = Param.Latency("10ns", "Static backend latency")
    disable_sanity_check = Param.Bool(False, "Disable port resp Q size check")

add_citation(MemCtrl,
"""@inproceedings{Hansson:2014:dram-controller,
  author       = {Andreas Hansson and
                  Neha Agarwal and
                  Aasheesh Kolli and
                  Thomas F. Wenisch and
                  Aniruddha N. Udipi},
  title        = {Simulating {DRAM} controllers for future system architecture exploration},
  booktitle    = {2014 {IEEE} International Symposium on Performance Analysis of Systems
                  and Software, {ISPASS} 2014, Monterey, CA, USA, March 23-25, 2014},
  pages        = {201--210},
  publisher    = {{IEEE} Computer Society},
  year         = {2014},
  url          = {https://doi.org/10.1109/ISPASS.2014.6844484},
  doi          = {10.1109/ISPASS.2014.6844484},
  timestamp    = {Fri, 24 Mar 2023 00:02:25 +0100},
  biburl       = {https://dblp.org/rec/conf/ispass/HanssonAKWU14.bib},
  bibsource    = {dblp computer science bibliography, https://dblp.org}
}
""")
