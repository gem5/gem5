load("@rules_python//python:defs.bzl", "py_binary")
load("@rules_shell//shell:sh_test.bzl", "sh_test")

package(
    default_visibility = ["//visibility:public"],
)

py_binary(
    name = "test",
    srcs = ["test.py"],
    deps = [
        "//src/learning_gem5/part2:goodbye_object_sim_object",
        "//src/learning_gem5/part2:hello_object_sim_object",
        "//src/learning_gem5/part2:simple_object_sim_object",
        "//bazel/pybind:_m5",
        "//src/python/m5",
        "//src/python/m5:core",
        "//src/python/m5:event",
        "//src/python/m5:options",
        "//src/python/m5:simulate",
        "//src/python/m5/stats",
        "//src/sim:root_sim_object",
    ],
)

py_binary(
    name = "test2",
    srcs = ["test2.py"],
    deps = [
        "//bazel/pybind:_m5",
        "//src/python/gem5/components/boards:test_board",
        "//src/python/gem5/components/memory:simple",
        "//src/python/gem5/components/processors:linear_generator",
        "//src/python/gem5/simulate:simulator",
        "//src/sim:root_sim_object",
        "//src/mem:simple_memory_sim_object",
        "//src/sim:system_sim_object",
        "//src/cpu/simple:base_atomic_simple_cpu_sim_object",
        "//src/python/m5/stats",
    ],
)

py_binary(
    name = "test3",
    srcs = ["test3.py"],
    deps = [
        "//bazel/pybind:_m5",
        "//src/python/gem5/components/boards:test_board",
        "//src/python/gem5/components/cachehierarchies/classic:no_cache",
        "//src/python/gem5/components/memory:simple",
        "//src/python/gem5/components/processors:linear_generator",
        "//src/python/gem5/simulate:simulator",
        "//src/sim:root_sim_object",
        "//src/mem:simple_memory_sim_object",
        "//src/sim:system_sim_object",
        "//src/mem:noncoherent_x_bar_sim_object",
        "//src/cpu/simple:base_timing_simple_cpu_sim_object",
        "//src/python/m5/stats",
    ],
)

py_binary(
    name = "test4",
    srcs = ["test4.py"],
    deps = [
        "//bazel/pybind:_m5",
        "//src/python/gem5/components/boards:test_board",
        "//src/python/gem5/components/cachehierarchies/classic:private_l1_shared_l2_cache_hierarchy",
        "//src/python/gem5/components/memory:simple",
        "//src/python/gem5/components/processors:linear_generator",
        "//src/python/gem5/simulate:simulator",
        "//src/sim:root_sim_object",
        "//src/mem:simple_memory_sim_object",
        "//src/sim:system_sim_object",
        "//src/mem:noncoherent_x_bar_sim_object",
        "//src/mem:coherent_x_bar_sim_object",
        "//src/mem/cache:cache_sim_object",
        "//src/cpu/simple:base_timing_simple_cpu_sim_object",
        "//src/python/m5/stats",
    ],
)

py_binary(
    name = "test5",
    srcs = ["test5.py"],
    deps = [
        "//bazel/pybind:_m5",
        "//src/python/gem5/components/boards:test_board",
        "//src/python/gem5/components/cachehierarchies/ruby:mesi_two_level_cache_hierarchy",
        "//src/python/gem5/components/memory:simple",
        "//src/python/gem5/components/processors:linear_generator",
        "//src/python/gem5/simulate:simulator",
        "//src/sim:root_sim_object",
        "//src/mem:simple_memory_sim_object",
        "//src/sim:system_sim_object",
        "//src/mem:noncoherent_x_bar_sim_object",
        "//src/mem/ruby/protocol:mesi_two_level_directory_controller_sim_object",
        "//src/mem/ruby/protocol:mesi_two_level_l1_cache_controller_sim_object",
        "//src/mem/ruby/protocol:mesi_two_level_l2_cache_controller_sim_object",
        "//src/mem/ruby/network:message_buffer_sim_object",
        "//src/mem/ruby/structures:ruby_cache_sim_object",
        "//src/mem/ruby/system:ruby_system_sim_object",
        "//src/mem/ruby/system:ruby_sequencer_sim_object",
        "//src/cpu/simple:base_timing_simple_cpu_sim_object",
        "//src/python/m5/stats",
    ],
)

py_binary(
    name = "test6",
    srcs = ["test6.py"],
    deps = [
        "//bazel/pybind:_m5",
        "//src/python/gem5/components/boards:test_board",
        "//src/python/gem5/components/cachehierarchies/chi:private_l1_cache_hierarchy",
        "//src/python/gem5/components/memory:simple",
        "//src/python/gem5/components/processors:linear_generator",
        "//src/python/gem5/simulate:simulator",
        "//src/sim:root_sim_object",
        "//src/mem:simple_memory_sim_object",
        "//src/sim:system_sim_object",
        "//src/mem:noncoherent_x_bar_sim_object",
        "//src/mem/ruby/protocol/chi:chi_cache_controller_sim_object",
        "//src/mem/ruby/protocol/chi:chi_memory_controller_sim_object",
        "//src/mem/ruby/network:message_buffer_sim_object",
        "//src/mem/ruby/structures:ruby_cache_sim_object",
        "//src/mem/ruby/system:ruby_system_sim_object",
        "//src/mem/ruby/system:ruby_sequencer_sim_object",
        "//src/cpu/testers/traffic_gen:py_traffic_gen_sim_object",
        "//src/python/m5/stats",
    ],
)

py_binary(
    name = "test7",
    srcs = ["test7.py"],
    deps = [
        "//bazel/pybind:_m5",
        "//src/python/gem5/components/boards:test_board",
        "//src/python/gem5/components/cachehierarchies/classic:no_cache",
        "//src/python/gem5/components/memory:multi_channel",
        "//src/python/gem5/components/processors:linear_generator",
        "//src/python/gem5/simulate:simulator",
        "//src/sim:root_sim_object",
        "//src/sim:system_sim_object",
        "//src/mem:mem_ctrl_sim_object",
        "//src/mem:dram_interface_sim_object",
        "//src/mem/qos:qo_s_mem_ctrl_sim_object",
        "//src/mem/qos:qo_s_policy_sim_object",
        "//src/cpu/testers/traffic_gen:py_traffic_gen_sim_object",
        "//src/python/m5/stats",
    ],
)

py_binary(
    name = "arm_hello_se",
    srcs = ["arm_hello_se.py"],
    deps = [
        "//bazel/pybind:_m5",
        "//src/python/gem5:isas",
        "//src/python/gem5/components/boards:simple_board",
        "//src/python/gem5/components/cachehierarchies/classic:no_cache",
        "//src/python/gem5/components/memory:simple",
        "//src/python/gem5/components/processors:cpu_types",
        "//src/python/gem5/components/processors:simple_processor",
        "//src/python/gem5/resources:resource",
        "//src/python/gem5/simulate:simulator",
        "//src/python/gem5/utils:requires",
        "//src/sim:root_sim_object",
        "//src/mem:simple_memory_sim_object",
        "//src/sim:system_sim_object",
        "//src/mem:coherent_x_bar_sim_object",
        "//src/mem:bridge_sim_object",
        "//src/dev:isa_fake_sim_object",
        "//src/arch/arm:arm_mmu_sim_object",
        "//src/arch/arm:arm_isa_sim_object",
        "//src/arch/arm:arm_interrupts_sim_object",
        "//src/arch/arm:arm_decoder_sim_object",
        "//src/dev/arm:generic_timer_sim_object",
        "//src/arch/arm:arm_se_workload_sim_object",
        "//src/python/m5:debug",
        "//src/python/m5:event",
        "//src/python/m5/stats",
        "//src/python/m5:trace",
    ],
)

py_binary(
    name = "test8",
    srcs = ["test8.py"],
    deps = [
        "//bazel/pybind:_m5",
        "//src/python/gem5:isas",
        "//src/python/gem5/components/boards:arm_board",
        "//src/python/gem5/components/cachehierarchies/classic:private_l1_private_l2_cache_hierarchy",
        "//src/python/gem5/components/memory:multi_channel",
        "//src/python/gem5/components/processors:cpu_types",
        "//src/python/gem5/components/processors:simple_processor",
        "//src/python/gem5/resources:resource",
        "//src/python/gem5/simulate:exit_handler",
        "//src/python/gem5/simulate:simulator",
        "//src/python/gem5/utils:override",
        "//src/python/m5",
        "//src/python/m5/objects",
        "//src/python/m5/objects:ArmCPU",
        "//src/python/m5/objects:ArmSystem",
        "//src/python/m5/objects:QoSMemCtrl",
        "//src/python/m5/objects:QoSPolicy",
        "//src/python/m5/objects:RealView",
    ],
)

[sh_test(
    name = "%s_test" % name,
    srcs = ["run_test.sh"],
    args = [name],
    data = [":%s" % name],
    deps = [
        "//testing/shbase",
        ":%s" % name,
    ],
) for name in [
    "test",
    "test2",
    "test3",
    "test4",
    "test5",
    "test6",
    "test7",
]]

# This test needs to be done in guitar since it depends on an arm binary.
# Alternatively, we could put the binary in google3, but I would rather not.
# sh_test(
#     name = "arm_hello_se_test",
#     srcs = ["run_test.sh"],
#     args = ["arm_hello_se"],
#     data = [":arm_hello_se"],
#     tags = [
#         "external",
#         "requires-net:external",
#     ],
#     deps = ["//testing/shbase"],
# )
py_binary(
    name = "test_m5",
    srcs = ["test_m5.py"],
    deps = ["//src/python/m5"],
    main = "test_m5.py",
)
