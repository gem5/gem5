# src/learning_gem5/part2/CMakeLists.txt
# Converted from src/learning_gem5/part2/SConscript

gem5_add_sources(
    simple_object.cc
    hello_object.cc
    goodbye_object.cc
    simple_memobj.cc
    simple_cache.cc
)

gem5_add_simobject(SimpleObject.py SIM_OBJECTS SimpleObject)
gem5_add_simobject(HelloObject.py SIM_OBJECTS HelloObject GoodbyeObject)
gem5_add_simobject(SimpleMemobj.py SIM_OBJECTS SimpleMemobj)
gem5_add_simobject(SimpleCache.py SIM_OBJECTS SimpleCache)

gem5_add_debug_flag(HelloExample "For Learning gem5 Part 2. Simple example debug flag")
gem5_add_debug_flag(SimpleMemobj "For Learning gem5 Part 2.")
gem5_add_debug_flag(SimpleCache "For Learning gem5 Part 2.")
