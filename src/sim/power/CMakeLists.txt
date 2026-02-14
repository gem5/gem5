# src/sim/power/CMakeLists.txt
# Converted from src/sim/power/SConscript

gem5_add_source(mathexpr_powermodel.cc)
gem5_add_source(power_model.cc)
gem5_add_source(thermal_domain.cc)
gem5_add_source(thermal_model.cc)
gem5_add_source(thermal_node.cc)

gem5_add_debug_flag(ThermalDomain "ThermalDomain debugging")

gem5_add_simobject(MathExprPowerModel.py SIM_OBJECTS MathExprPowerModel)
gem5_add_simobject(PowerModel.py SIM_OBJECTS PowerModel ENUMS PMType)
gem5_add_simobject(PowerModelState.py SIM_OBJECTS PowerModelState)
gem5_add_simobject(ThermalDomain.py SIM_OBJECTS ThermalDomain)
gem5_add_simobject(ThermalModel.py SIM_OBJECTS
    ThermalNode ThermalResistor ThermalCapacitor ThermalReference ThermalModel)
