# src/cpu/testers/gpu_ruby_test/CMakeLists.txt
# Converted from src/cpu/testers/gpu_ruby_test/SConscript
# Conditional on CONF_BUILD_GPU AND CONF_RUBY (guarded by parent)

gem5_add_source(address_manager.cc)
gem5_add_source(episode.cc)
gem5_add_source(protocol_tester.cc)
gem5_add_source(cpu_thread.cc)
gem5_add_source(dma_thread.cc)
gem5_add_source(gpu_wavefront.cc)
gem5_add_source(tester_thread.cc)

gem5_add_debug_flag(ProtocolTest "ProtocolTest debugging")
gem5_add_debug_flag(EpisodeCount "EpisodeCount debugging")

gem5_add_simobject(ProtocolTester.py
    SIM_OBJECTS ProtocolTester)
gem5_add_simobject(CpuThread.py
    SIM_OBJECTS CpuThread)
gem5_add_simobject(DmaThread.py
    SIM_OBJECTS DmaThread)
gem5_add_simobject(GpuWavefront.py
    SIM_OBJECTS GpuWavefront)
gem5_add_simobject(TesterThread.py
    SIM_OBJECTS TesterThread)
gem5_add_simobject(TesterDma.py
    SIM_OBJECTS TesterDma)
