# Copyright (c) 2013 ARM Limited
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
from citations import add_citation
from m5.objects.AbstractMemory import *
from m5.params import *


# A wrapper for DRAMSim2 multi-channel memory controller
class DRAMSim2(AbstractMemory):
    type = "DRAMSim2"
    cxx_header = "mem/dramsim2.hh"
    cxx_class = "gem5::memory::DRAMSim2"

    # A single port for now
    port = ResponsePort("This port sends responses and receives requests")

    deviceConfigFile = Param.String(
        "ini/DDR3_micron_32M_8B_x8_sg15.ini", "Device configuration file"
    )
    systemConfigFile = Param.String(
        "system.ini.example", "Memory organisation configuration file"
    )
    filePath = Param.String(
        "ext/dramsim2/DRAMSim2/", "Directory to prepend to file names"
    )
    traceFile = Param.String("", "Output file for trace generation")
    enableDebug = Param.Bool(False, "Enable DRAMSim2 debug output")


add_citation(
    DRAMSim2,
    """@article{Rosenfeld:2011:dramsim2,
  author       = {Paul Rosenfeld and
                  Elliott Cooper{-}Balis and
                  Bruce L. Jacob},
  title        = {DRAMSim2: {A} Cycle Accurate Memory System Simulator},
  journal      = {{IEEE} Compututer Architecture Letters},
  volume       = {10},
  number       = {1},
  pages        = {16--19},
  year         = {2011},
  url          = {https://doi.org/10.1109/L-CA.2011.4},
  doi          = {10.1109/L-CA.2011.4}
}
""",
)
