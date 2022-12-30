# Copyright (c) 2015, 2021 Arm Limited
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

from m5.SimObject import *
from m5.objects.ClockedObject import ClockedObject

from m5.params import *
from m5.objects import ThermalDomain


# Represents a thermal node
class ThermalNode(SimObject):
    type = "ThermalNode"
    cxx_header = "sim/power/thermal_node.hh"
    cxx_class = "gem5::ThermalNode"


# Represents a thermal resistor
class ThermalResistor(SimObject):
    type = "ThermalResistor"
    cxx_header = "sim/power/thermal_model.hh"
    cxx_class = "gem5::ThermalResistor"

    cxx_exports = [PyBindMethod("setNodes")]

    resistance = Param.Float(
        1.0, "Thermal resistance, expressed in Kelvin per Watt"
    )


# Represents a thermal capacitor
class ThermalCapacitor(SimObject):
    type = "ThermalCapacitor"
    cxx_header = "sim/power/thermal_model.hh"
    cxx_class = "gem5::ThermalCapacitor"

    cxx_exports = [PyBindMethod("setNodes")]

    capacitance = Param.Float(
        1.0, "Thermal capacitance, expressed in Joules per Kelvin"
    )


# Represents a fixed temperature node (ie. air)
class ThermalReference(SimObject, object):
    type = "ThermalReference"
    cxx_header = "sim/power/thermal_model.hh"
    cxx_class = "gem5::ThermalReference"

    cxx_exports = [PyBindMethod("setNode")]

    # Static temperature which may change over time
    temperature = Param.Temperature("25.0C", "Operational temperature")


# Represents a thermal capacitor
class ThermalModel(ClockedObject):
    type = "ThermalModel"
    cxx_header = "sim/power/thermal_model.hh"
    cxx_class = "gem5::ThermalModel"

    cxx_exports = [
        PyBindMethod("addCapacitor"),
        PyBindMethod("addResistor"),
        PyBindMethod("addReference"),
        PyBindMethod("addDomain"),
        PyBindMethod("addNode"),
        PyBindMethod("doStep"),
    ]

    step = Param.Float(
        0.01, "Simulation step (in seconds) for thermal simulation"
    )

    def populate(self):
        if not hasattr(self, "_capacitors"):
            self._capacitors = []
        if not hasattr(self, "_resistors"):
            self._resistors = []
        if not hasattr(self, "_domains"):
            self._domains = []
        if not hasattr(self, "_references"):
            self._references = []
        if not hasattr(self, "_nodes"):
            self._nodes = []

    def init(self):
        self.populate()

        for ref, node in self._references:
            ref.getCCObject().setNode(node.getCCObject())
            self.getCCObject().addReference(ref.getCCObject())

        for dom, node in self._domains:
            dom.getCCObject().setNode(node.getCCObject())
            self.getCCObject().addDomain(dom.getCCObject())

        for cap, node1, node2 in self._capacitors:
            cap.getCCObject().setNodes(
                node1.getCCObject(), node2.getCCObject()
            )
            self.getCCObject().addCapacitor(cap.getCCObject())

        for res, node1, node2 in self._resistors:
            res.getCCObject().setNodes(
                node1.getCCObject(), node2.getCCObject()
            )
            self.getCCObject().addResistor(res.getCCObject())

        for node in self._nodes:
            self.getCCObject().addNode(node.getCCObject())

    def addCapacitor(self, cap, node1, node2):
        self.populate()
        self._capacitors.append((cap, node1, node2))
        self._parent.thermal_components.append(cap)
        self.addNodes(node1, node2)

    def addResistor(self, res, node1, node2):
        self.populate()
        self._resistors.append((res, node1, node2))
        self._parent.thermal_components.append(res)
        self.addNodes(node1, node2)

    def addReference(self, ref, node):
        self.populate()
        self._references.append((ref, node))
        self._parent.thermal_components.append(ref)
        self.addNodes(node)

    def addDomain(self, dom, node):
        self.populate()
        self._domains.append((dom, node))
        self.addNodes(node)

    def addNodes(self, *nodes):
        for node in nodes:
            if node not in self._nodes:
                self._nodes.append(node)
                self._parent.thermal_components.append(node)

    def doStep(self):
        self.getCCObject().doStep()
