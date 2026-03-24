# src/cpu/testers/rubytest/CMakeLists.txt
# Converted from src/cpu/testers/rubytest/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(RubyTester.cc)
gem5_add_source(Check.cc)
gem5_add_source(CheckTable.cc)

gem5_add_debug_flag(RubyTest "RubyTest debugging")

gem5_add_simobject(RubyTester.py
    SIM_OBJECTS RubyTester)
