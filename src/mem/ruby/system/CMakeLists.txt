# src/mem/ruby/system/CMakeLists.txt
# Converted from src/mem/ruby/system/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(CacheRecorder.cc)
gem5_add_source(DMASequencer.cc)
gem5_add_source(HTMSequencer.cc)
gem5_add_source(RubyPort.cc)
gem5_add_source(RubyPortProxy.cc)
gem5_add_source(RubySystem.cc)
gem5_add_source(Sequencer.cc)

# GPU-specific sources
if(CONF_BUILD_GPU)
    gem5_add_source(GPUCoalescer.cc)
    gem5_add_source(VIPERCoalescer.cc)
    gem5_add_source(VIPERSequencer.cc)
endif()

# SimObjects
gem5_add_simobject(RubySystem.py SIM_OBJECTS RubySystem)
gem5_add_simobject(Sequencer.py
    SIM_OBJECTS RubyPort RubyPortProxy RubySequencer RubyHTMSequencer
               DMASequencer)

if(CONF_BUILD_GPU)
    gem5_add_simobject(GPUCoalescer.py SIM_OBJECTS RubyGPUCoalescer)
    gem5_add_simobject(VIPERCoalescer.py SIM_OBJECTS VIPERCoalescer)
    gem5_add_simobject(VIPERSequencer.py SIM_OBJECTS VIPERSequencer)
endif()
