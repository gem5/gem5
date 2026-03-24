gem5_add_simobject(SystemC_Example.py
    SIM_OBJECTS SystemC_Printer Gem5_Feeder)

gem5_add_source(printer.cc)
gem5_add_source(feeder.cc)
