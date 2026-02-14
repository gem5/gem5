# src/dev/pci/CMakeLists.txt
# Converted from src/dev/pci/SConscript

gem5_add_source(device.cc)
gem5_add_source(upstream.cc)
gem5_add_source(up_down_bridge.cc)
gem5_add_source(bus.cc)
gem5_add_source(host.cc)
gem5_add_source(copy_engine.cc)

gem5_add_debug_flag(PciDevice "PciDevice debugging")
gem5_add_debug_flag(PciUpstream "PciUpstream debugging")
gem5_add_debug_flag(PciHost "PciHost debugging")
gem5_add_debug_flag(DMACopyEngine "DMACopyEngine debugging")

gem5_add_simobject(PciDevice.py SIM_OBJECTS
    PciBar PciBarNone PciIoBar PciLegacyIoBar PciMemBar
    PciMemUpperBar PciDevice PciEndpoint PciType1Device)
gem5_add_simobject(PciUpstream.py SIM_OBJECTS
    PciUpstream PciUpDownBridge PciBus PciConfigError)
gem5_add_simobject(PciHost.py SIM_OBJECTS PciHost GenericPciHost)
gem5_add_simobject(CopyEngine.py SIM_OBJECTS CopyEngine)
