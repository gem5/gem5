# Copyright (c) 2022 Fraunhofer IESE
# All rights reserved
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
from m5.citations import add_citation
from m5.objects.AbstractMemory import *
from m5.objects.Tlm import TlmTargetSocket
from m5.params import *
from m5.proxy import *
from m5.SimObject import *


class DRAMSys(AbstractMemory):
    type = "DRAMSys"
    cxx_class = "gem5::memory::DRAMSys"
    cxx_header = "mem/dramsys.hh"
    tlm = TlmTargetSocket(32, "TLM target port")

    configuration = Param.String("Path to the DRAMSys configuration")
    resource_directory = Param.String("Path to the DRAMSys resource directory")
    recordable = Param.Bool(True, "Whether DRAMSys should record a trace file")


add_citation(
    DRAMSys,
    """@inproceedings{Steiner:2020:dramsys4,
  author       = {Lukas Steiner and
                  Matthias Jung and
                  Felipe S. Prado and
                  Kirill Bykov and
                  Norbert Wehn},
  editor       = {Alex Orailoglu and
                  Matthias Jung and
                  Marc Reichenbach},
  title        = {DRAMSys4.0: {A} Fast and Cycle-Accurate SystemC/TLM-Based {DRAM} Simulator},
  booktitle    = {Embedded Computer Systems: Architectures, Modeling, and Simulation
                  - 20th International Conference, {SAMOS} 2020, Samos, Greece, July
                  5-9, 2020, Proceedings},
  series       = {Lecture Notes in Computer Science},
  volume       = {12471},
  pages        = {110--126},
  publisher    = {Springer},
  year         = {2020},
  url          = {https://doi.org/10.1007/978-3-030-60939-9\_8},
  doi          = {10.1007/978-3-030-60939-9\_8}
}
""",
)
