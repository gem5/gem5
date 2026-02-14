# src/mem/ruby/CMakeLists.txt
# Converted from src/mem/ruby/SConscript
# Conditional on CONF_RUBY (guarded by parent)

# Debug flags
gem5_add_debug_flag(ProtocolTrace "ProtocolTrace debugging")
gem5_add_debug_flag(RubyCache "RubyCache debugging")
gem5_add_debug_flag(RubyCacheTrace "RubyCacheTrace debugging")
gem5_add_debug_flag(RubyDma "RubyDma debugging")
gem5_add_debug_flag(RubyGenerated "RubyGenerated debugging")
gem5_add_debug_flag(RubyHitMiss "RubyHitMiss debugging")
gem5_add_debug_flag(RubyNetwork "RubyNetwork debugging")
gem5_add_debug_flag(RubyPort "RubyPort debugging")
gem5_add_debug_flag(RubyPrefetcher "RubyPrefetcher debugging")
gem5_add_debug_flag(RubyProtocol "RubyProtocol debugging")
gem5_add_debug_flag(RubyQueue "RubyQueue debugging")
gem5_add_debug_flag(RubyResourceStalls "RubyResourceStalls debugging")
gem5_add_debug_flag(RubySequencer "RubySequencer debugging")
gem5_add_debug_flag(RubySlicc "RubySlicc debugging")
gem5_add_debug_flag(RubyStats "RubyStats debugging")
gem5_add_debug_flag(RubySystem "RubySystem debugging")
gem5_add_debug_flag(RubyTester "RubyTester debugging")

gem5_add_debug_flag(Ruby "Ruby compound flag"
    COMPONENTS RubyQueue RubyNetwork RubyTester RubyGenerated
              RubySlicc RubySystem RubyCache RubyDma RubyPort
              RubySequencer RubyCacheTrace RubyPrefetcher
              RubyProtocol RubyHitMiss)

# TODO: MakeInclude() calls generate protocol-directory include wrappers.
# These will be handled by the SLICC/protocol generation phase.

# Subdirectories
add_subdirectory(common)
add_subdirectory(network)
add_subdirectory(profiler)
add_subdirectory(protocol)
add_subdirectory(slicc_interface)
add_subdirectory(structures)
add_subdirectory(system)
