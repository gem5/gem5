from m5.objects.PowerModelState import PowerModelState
from m5.params import *
from m5.SimObject import (
    SimObject,
    cxxMethod,
)
from m5.util.pybind import *


# Dynamic and static power equations represented by arithmetic operators than strings in MathExprPowerModel
class PowerModelPyFunc(PowerModelState):
    type = "PowerModelPyFunc"
    cxx_header = "sim/power/power_model_pyfunc.hh"
    cxx_class = "gem5::PowerModelPyFunc"

    # Equations for dynamic and static power in Watts
    # Equations may use gem5 stats ie. "1.1*ipc + 2.3*l2_cache.overall_misses"
    # It is possible to use automatic variables such as "temp"
    # You may also use stat names (relative path to the simobject)
    dyn = Param.PyFunc("Function to call for Dynamic Power")
    st = Param.PyFunc("Function to call for Static Power")
    pwr_interval = Param.Cycles(0, "Interval in which power is calculated")
    clock_stat = Param.String(
        "", "Stat corresponding to your clock (used for sampling)"
    )

    cxx_exports = [PyBindMethod("startSampling"), PyBindMethod("stopSampling")]
