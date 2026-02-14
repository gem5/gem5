# src/cpu/simple/probes/CMakeLists.txt
# Converted from src/cpu/simple/probes/SConscript

gem5_add_source(simpoint.cc)
gem5_add_source(looppoint_analysis.cc)

gem5_add_debug_flag(LooppointAnalysis "LooppointAnalysis debugging")

gem5_add_simobject(SimPoint.py
    SIM_OBJECTS SimPoint)
gem5_add_simobject(LooppointAnalysis.py
    SIM_OBJECTS LooppointAnalysis LooppointAnalysisManager)
