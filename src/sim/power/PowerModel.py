# Copyright (c) 2016-2018, 2021 Arm Limited
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
from m5.params import *
from m5.proxy import Parent


# Enum for a type of  power model
class PMType(Enum):
    vals = ["All", "Static", "Dynamic"]


# Represents a power model for a simobj
# The model itself is also a SimObject so we can make use some
# nice features available such as Parent.any
class PowerModel(SimObject):
    type = "PowerModel"
    cxx_header = "sim/power/power_model.hh"
    cxx_class = "gem5::PowerModel"

    cxx_exports = [
        PyBindMethod("getDynamicPower"),
        PyBindMethod("getStaticPower"),
    ]

    # Keep a list of every model for every power state
    pm = VectorParam.PowerModelState([], "List of per-state power models.")

    # Need a reference to the system so we can query the thermal domain
    # about temperature (temperature is needed for leakage calculation)
    subsystem = Param.SubSystem(Parent.any, "subsystem")

    # Type of power model
    pm_type = Param.PMType("All", "Type of power model")

    # Ambient temperature to be used when no thermal model is present
    ambient_temp = Param.Temperature("25.0C", "Ambient temperature")
