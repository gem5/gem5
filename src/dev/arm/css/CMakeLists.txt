# src/dev/arm/css/CMakeLists.txt
# Converted from src/dev/arm/css/SConscript

gem5_add_source(mhu.cc)
gem5_add_source(scmi_platform.cc)
gem5_add_source(scmi_protocols.cc)

gem5_add_debug_flag(MHU "MHU debugging")
gem5_add_debug_flag(SCMI "SCMI debugging")

gem5_add_simobject(MHU.py SIM_OBJECTS
    MhuDoorbell Scp2ApDoorbell Ap2ScpDoorbell MHU)
gem5_add_simobject(Scmi.py SIM_OBJECTS
    ScmiChannel ScmiAgentChannel ScmiPlatformChannel ScmiCommunication ScmiPlatform)
gem5_add_simobject(Scp.py SIM_OBJECTS Scp)
