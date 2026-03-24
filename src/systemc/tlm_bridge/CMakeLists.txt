# src/systemc/tlm_bridge/CMakeLists.txt
# Converted from src/systemc/tlm_bridge/SConscript

gem5_add_sources(
    gem5_to_tlm.cc
    sc_ext.cc
    sc_mm.cc
    tlm_to_gem5.cc
)

gem5_add_simobject(TlmBridge.py
    SIM_OBJECTS
        Gem5ToTlmBridgeBase TlmToGem5BridgeBase
        Gem5ToTlmBridge32 Gem5ToTlmBridge64 Gem5ToTlmBridge128
        Gem5ToTlmBridge256 Gem5ToTlmBridge512
        TlmToGem5Bridge32 TlmToGem5Bridge64 TlmToGem5Bridge128
        TlmToGem5Bridge256 TlmToGem5Bridge512)

gem5_add_debug_flag(TlmBridge "TLM bridge debugging")
