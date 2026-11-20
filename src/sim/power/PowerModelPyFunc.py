# Copyright (c) 2026 - The Unviersity of Wisconsin
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

from m5.objects.PowerModelState import PowerModelState
from m5.params import *
from m5.SimObject import (
    SimObject,
    cxxMethod,
)
from m5.util.pybind import *

# Dynamic and static power is modeled by Python functions (which return
# doubles) rather than using strings like in MathExprPowerModel. This also
# allows more expressive power models (e.g., using linear algebra operators
# or using ML/weight-based power models).


class PowerModelPyFunc(PowerModelState):
    type = "PowerModelPyFunc"
    cxx_header = "sim/power/power_model_pyfunc.hh"
    cxx_class = "gem5::PowerModelPyFunc"

    dyn = Param.PyFunc("Function to call for Dynamic Power")
    st = Param.PyFunc("Function to call for Static Power")
    pwr_interval = Param.Cycles(
        0, "Cycles between power samples, 0 disables sampling"
    )
    auto_start = Param.Bool(
        False, "Start sampling at startup instead of on startSampling()"
    )
    on_sample_start = Param.PyFunc(
        None, "Optional counter snapshot on sampling or region start and reset"
    )

    power_log = Param.PyFunc(
        None,
        "Optional formatter(tick, label, dyn_w, st_w, temp_k) returning str "
        "or None to skip the report",
    )
    power_log_file = Param.String(
        "",
        "Dedicated power report destination (file, stdout or stderr). "
        "Relative paths use the output directory. "
        "Empty uses PowerLog tracing.",
    )

    cxx_exports = [
        PyBindMethod("startSampling"),
        PyBindMethod("stopSampling"),
        PyBindMethod("inPowerAtInterval"),
        PyBindMethod("getSampleDurationSeconds"),
        PyBindMethod("addRegion"),
        PyBindMethod("beginRegion"),
        PyBindMethod("endRegion"),
    ]
