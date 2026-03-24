# src/mem/ruby/network/garnet/CMakeLists.txt
# Converted from src/mem/ruby/network/garnet/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(Credit.cc)
gem5_add_source(CrossbarSwitch.cc)
gem5_add_source(GarnetLink.cc)
gem5_add_source(GarnetNetwork.cc)
gem5_add_source(InputUnit.cc)
gem5_add_source(NetworkBridge.cc)
gem5_add_source(NetworkInterface.cc)
gem5_add_source(NetworkLink.cc)
gem5_add_source(OutVcState.cc)
gem5_add_source(OutputUnit.cc)
gem5_add_source(Router.cc)
gem5_add_source(RoutingUnit.cc)
gem5_add_source(SwitchAllocator.cc)
gem5_add_source(VirtualChannel.cc)
gem5_add_source(flit.cc)
gem5_add_source(flitBuffer.cc)

# SimObjects
gem5_add_simobject(GarnetLink.py
    SIM_OBJECTS NetworkLink CreditLink NetworkBridge GarnetIntLink GarnetExtLink
    ENUMS CDCType)
gem5_add_simobject(GarnetNetwork.py
    SIM_OBJECTS GarnetNetwork GarnetNetworkInterface GarnetRouter)
