# src/cpu/testers/directedtest/CMakeLists.txt
# Converted from src/cpu/testers/directedtest/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(RubyDirectedTester.cc)
gem5_add_source(DirectedGenerator.cc)
gem5_add_source(SeriesRequestGenerator.cc)
gem5_add_source(InvalidateGenerator.cc)

gem5_add_debug_flag(DirectedTest "DirectedTest debugging")

gem5_add_simobject(RubyDirectedTester.py
    SIM_OBJECTS DirectedGenerator SeriesRequestGenerator
                InvalidateGenerator RubyDirectedTester)
