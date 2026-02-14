# src/sst/CMakeLists.txt
# Converted from src/sst/SConscript

gem5_add_sources(
    outgoing_request_bridge.cc
    sst_responder_interface.cc
)

gem5_add_simobject(OutgoingRequestBridge.py
    SIM_OBJECTS OutgoingRequestBridge)
