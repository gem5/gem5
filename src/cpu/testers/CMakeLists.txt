# src/cpu/testers/CMakeLists.txt

# Unconditional testers
add_subdirectory(memtest)
add_subdirectory(spatter_gen)
add_subdirectory(traffic_gen)

# Ruby-conditional testers
if(CONF_RUBY)
    add_subdirectory(directedtest)
    add_subdirectory(garnet_synthetic_traffic)
    add_subdirectory(rubytest)
    if(CONF_BUILD_GPU)
        add_subdirectory(gpu_ruby_test)
    endif()
endif()
