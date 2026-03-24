# src/python/CMakeLists.txt
# Python integration layer for gem5.
#
# Converted from src/python/SConscript.
#
# NOTE: embedded.cc, importer.cc, gem5py.cc, gem5py_m5.cc, and the
# m5ImporterCode blob are built by cmake/Gem5Py.cmake -- do NOT add
# them here.

# ---------------------------------------------------------------------------
# gem5 standard library (gem5.*)
# ---------------------------------------------------------------------------

gem5_add_pysource("gem5" "gem5/__init__.py")
gem5_add_pysource("gem5" "gem5/coherence_protocol.py")
gem5_add_pysource("gem5" "gem5/isas.py")
gem5_add_pysource("gem5" "gem5/runtime.py")
gem5_add_pysource("gem5" "gem5/gem5_default_config.py")

# gem5.simulate
gem5_add_pysource("gem5.simulate" "gem5/simulate/__init__.py")
gem5_add_pysource("gem5.simulate" "gem5/simulate/simulator.py")
gem5_add_pysource("gem5.simulate" "gem5/simulate/exit_event.py")
gem5_add_pysource("gem5.simulate" "gem5/simulate/exit_event_generators.py")
gem5_add_pysource("gem5.simulate" "gem5/simulate/exit_handler.py")

# gem5.components
gem5_add_pysource("gem5.components" "gem5/components/__init__.py")

# gem5.components.boards
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/__init__.py")
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/abstract_board.py")
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/abstract_system_board.py")
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/mem_mode.py")
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/riscv_board.py")
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/simple_board.py")
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/test_board.py")
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/x86_board.py")
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/arm_board.py")
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/kernel_disk_workload.py")
gem5_add_pysource("gem5.components.boards" "gem5/components/boards/se_binary_workload.py")

# gem5.components.boards.experimental
gem5_add_pysource("gem5.components.boards.experimental" "gem5/components/boards/experimental/__init__.py")
gem5_add_pysource("gem5.components.boards.experimental" "gem5/components/boards/experimental/lupv_board.py")

# gem5.components.cachehierarchies
gem5_add_pysource("gem5.components.cachehierarchies" "gem5/components/cachehierarchies/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies" "gem5/components/cachehierarchies/abstract_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies" "gem5/components/cachehierarchies/abstract_two_level_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies" "gem5/components/cachehierarchies/abstract_three_level_cache_hierarchy.py")

# gem5.components.cachehierarchies.chi
gem5_add_pysource("gem5.components.cachehierarchies.chi" "gem5/components/cachehierarchies/chi/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.chi" "gem5/components/cachehierarchies/chi/private_l1_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.chi" "gem5/components/cachehierarchies/chi/private_l1_private_l2_cache_hierarchy.py")

# gem5.components.cachehierarchies.chi.nodes
gem5_add_pysource("gem5.components.cachehierarchies.chi.nodes" "gem5/components/cachehierarchies/chi/nodes/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.chi.nodes" "gem5/components/cachehierarchies/chi/nodes/abstract_node.py")
gem5_add_pysource("gem5.components.cachehierarchies.chi.nodes" "gem5/components/cachehierarchies/chi/nodes/directory.py")
gem5_add_pysource("gem5.components.cachehierarchies.chi.nodes" "gem5/components/cachehierarchies/chi/nodes/dma_requestor.py")
gem5_add_pysource("gem5.components.cachehierarchies.chi.nodes" "gem5/components/cachehierarchies/chi/nodes/l1_cache.py")
gem5_add_pysource("gem5.components.cachehierarchies.chi.nodes" "gem5/components/cachehierarchies/chi/nodes/l2_cache.py")
gem5_add_pysource("gem5.components.cachehierarchies.chi.nodes" "gem5/components/cachehierarchies/chi/nodes/memory_controller.py")

# gem5.components.cachehierarchies.classic
gem5_add_pysource("gem5.components.cachehierarchies.classic" "gem5/components/cachehierarchies/classic/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic" "gem5/components/cachehierarchies/classic/abstract_classic_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic" "gem5/components/cachehierarchies/classic/no_cache.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic" "gem5/components/cachehierarchies/classic/private_l1_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic" "gem5/components/cachehierarchies/classic/private_l1_walk_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic" "gem5/components/cachehierarchies/classic/private_l1_private_l2_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic" "gem5/components/cachehierarchies/classic/two_level_fdp_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic" "gem5/components/cachehierarchies/classic/private_l1_private_l2_walk_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic" "gem5/components/cachehierarchies/classic/private_l1_shared_l2_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic" "gem5/components/cachehierarchies/classic/private_l1_shared_l2_walk_cache_hierarchy.py")

# gem5.components.cachehierarchies.classic.caches
gem5_add_pysource("gem5.components.cachehierarchies.classic.caches" "gem5/components/cachehierarchies/classic/caches/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic.caches" "gem5/components/cachehierarchies/classic/caches/l1dcache.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic.caches" "gem5/components/cachehierarchies/classic/caches/l1icache.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic.caches" "gem5/components/cachehierarchies/classic/caches/l2cache.py")
gem5_add_pysource("gem5.components.cachehierarchies.classic.caches" "gem5/components/cachehierarchies/classic/caches/mmu_cache.py")

# gem5.components.cachehierarchies.ruby
gem5_add_pysource("gem5.components.cachehierarchies.ruby" "gem5/components/cachehierarchies/ruby/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby" "gem5/components/cachehierarchies/ruby/abstract_ruby_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby" "gem5/components/cachehierarchies/ruby/mesi_two_level_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby" "gem5/components/cachehierarchies/ruby/mesi_three_level_cache_hierarchy.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby" "gem5/components/cachehierarchies/ruby/mi_example_cache_hierarchy.py")

# gem5.components.cachehierarchies.ruby.caches
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches" "gem5/components/cachehierarchies/ruby/caches/__init__.py")

# gem5.components.cachehierarchies.ruby.caches.mesi_two_level
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_two_level" "gem5/components/cachehierarchies/ruby/caches/mesi_two_level/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_two_level" "gem5/components/cachehierarchies/ruby/caches/mesi_two_level/directory.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_two_level" "gem5/components/cachehierarchies/ruby/caches/mesi_two_level/dma_controller.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_two_level" "gem5/components/cachehierarchies/ruby/caches/mesi_two_level/l1_cache.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_two_level" "gem5/components/cachehierarchies/ruby/caches/mesi_two_level/l2_cache.py")

# gem5.components.cachehierarchies.ruby.caches.mesi_three_level
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_three_level" "gem5/components/cachehierarchies/ruby/caches/mesi_three_level/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_three_level" "gem5/components/cachehierarchies/ruby/caches/mesi_three_level/directory.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_three_level" "gem5/components/cachehierarchies/ruby/caches/mesi_three_level/dma_controller.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_three_level" "gem5/components/cachehierarchies/ruby/caches/mesi_three_level/l1_cache.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_three_level" "gem5/components/cachehierarchies/ruby/caches/mesi_three_level/l2_cache.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mesi_three_level" "gem5/components/cachehierarchies/ruby/caches/mesi_three_level/l3_cache.py")

# gem5.components.cachehierarchies.ruby.caches.prebuilt
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.prebuilt" "gem5/components/cachehierarchies/ruby/caches/prebuilt/__init__.py")

# gem5.components.cachehierarchies.ruby.caches.prebuilt.octopi_cache
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.prebuilt.octopi_cache" "gem5/components/cachehierarchies/ruby/caches/prebuilt/octopi_cache/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.prebuilt.octopi_cache" "gem5/components/cachehierarchies/ruby/caches/prebuilt/octopi_cache/octopi.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.prebuilt.octopi_cache" "gem5/components/cachehierarchies/ruby/caches/prebuilt/octopi_cache/core_complex.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.prebuilt.octopi_cache" "gem5/components/cachehierarchies/ruby/caches/prebuilt/octopi_cache/octopi_network.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.prebuilt.octopi_cache" "gem5/components/cachehierarchies/ruby/caches/prebuilt/octopi_cache/ruby_network_components.py")

# gem5.components.cachehierarchies.ruby.caches.mi_example
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mi_example" "gem5/components/cachehierarchies/ruby/caches/mi_example/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mi_example" "gem5/components/cachehierarchies/ruby/caches/mi_example/directory.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mi_example" "gem5/components/cachehierarchies/ruby/caches/mi_example/dma_controller.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.mi_example" "gem5/components/cachehierarchies/ruby/caches/mi_example/l1_cache.py")

# gem5.components.cachehierarchies.ruby.caches.viper
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.viper" "gem5/components/cachehierarchies/ruby/caches/viper/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.viper" "gem5/components/cachehierarchies/ruby/caches/viper/corepair_cache.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.viper" "gem5/components/cachehierarchies/ruby/caches/viper/directory.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.viper" "gem5/components/cachehierarchies/ruby/caches/viper/dma_controller.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.viper" "gem5/components/cachehierarchies/ruby/caches/viper/tcp.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.viper" "gem5/components/cachehierarchies/ruby/caches/viper/sqc.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.caches.viper" "gem5/components/cachehierarchies/ruby/caches/viper/tcc.py")

# gem5.components.cachehierarchies.ruby.topologies
gem5_add_pysource("gem5.components.cachehierarchies.ruby.topologies" "gem5/components/cachehierarchies/ruby/topologies/__init__.py")
gem5_add_pysource("gem5.components.cachehierarchies.ruby.topologies" "gem5/components/cachehierarchies/ruby/topologies/simple_pt2pt.py")

# gem5.components.devices
gem5_add_pysource("gem5.components.devices" "gem5/components/devices/__init__.py")

# gem5.components.devices.gpus
gem5_add_pysource("gem5.components.devices.gpus" "gem5/components/devices/gpus/__init__.py")
gem5_add_pysource("gem5.components.devices.gpus" "gem5/components/devices/gpus/amdgpu.py")
gem5_add_pysource("gem5.components.devices.gpus" "gem5/components/devices/gpus/viper_shader.py")

# gem5.components.memory
gem5_add_pysource("gem5.components.memory" "gem5/components/memory/__init__.py")
gem5_add_pysource("gem5.components.memory" "gem5/components/memory/abstract_memory_system.py")
gem5_add_pysource("gem5.components.memory" "gem5/components/memory/dramsim_3.py")
if(HAVE_DRAMSYS)
    gem5_add_pysource("gem5.components.memory" "gem5/components/memory/dramsys.py")
endif()
gem5_add_pysource("gem5.components.memory" "gem5/components/memory/simple.py")
gem5_add_pysource("gem5.components.memory" "gem5/components/memory/memory.py")
gem5_add_pysource("gem5.components.memory" "gem5/components/memory/single_channel.py")
gem5_add_pysource("gem5.components.memory" "gem5/components/memory/multi_channel.py")
gem5_add_pysource("gem5.components.memory" "gem5/components/memory/hbm.py")

# gem5.components.memory.dram_interfaces
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/__init__.py")
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/ddr3.py")
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/ddr4.py")
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/ddr5.py")
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/gddr.py")
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/hbm.py")
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/hmc.py")
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/lpddr2.py")
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/lpddr3.py")
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/lpddr5.py")
gem5_add_pysource("gem5.components.memory.dram_interfaces" "gem5/components/memory/dram_interfaces/wideio.py")

# gem5.components.processors
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/__init__.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/abstract_core.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/abstract_generator_core.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/abstract_generator.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/abstract_processor.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/complex_generator_core.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/complex_generator.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/cpu_types.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/gups_generator_core.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/gups_generator.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/gups_generator_ep.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/gups_generator_par.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/linear_generator_core.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/linear_generator.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/strided_generator_core.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/strided_generator.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/random_generator_core.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/random_generator.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/simple_core.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/base_cpu_core.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/simple_processor.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/decoupled_processor.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/base_cpu_processor.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/simple_switchable_processor.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/switchable_processor.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/traffic_generator_core.py")
gem5_add_pysource("gem5.components.processors" "gem5/components/processors/traffic_generator.py")

# gem5.components.processors.spatter_gen
gem5_add_pysource("gem5.components.processors.spatter_gen" "gem5/components/processors/spatter_gen/__init__.py")
gem5_add_pysource("gem5.components.processors.spatter_gen" "gem5/components/processors/spatter_gen/spatter_generator_core.py")
gem5_add_pysource("gem5.components.processors.spatter_gen" "gem5/components/processors/spatter_gen/spatter_generator.py")
gem5_add_pysource("gem5.components.processors.spatter_gen" "gem5/components/processors/spatter_gen/spatter_kernel.py")

# gem5.prebuilt
gem5_add_pysource("gem5.prebuilt" "gem5/prebuilt/__init__.py")

# gem5.prebuilt.demo
gem5_add_pysource("gem5.prebuilt.demo" "gem5/prebuilt/demo/__init__.py")
gem5_add_pysource("gem5.prebuilt.demo" "gem5/prebuilt/demo/x86_demo_board.py")
gem5_add_pysource("gem5.prebuilt.demo" "gem5/prebuilt/demo/riscv_demo_board.py")
gem5_add_pysource("gem5.prebuilt.demo" "gem5/prebuilt/demo/arm_demo_board.py")

# gem5.prebuilt.riscvmatched
gem5_add_pysource("gem5.prebuilt.riscvmatched" "gem5/prebuilt/riscvmatched/__init__.py")
gem5_add_pysource("gem5.prebuilt.riscvmatched" "gem5/prebuilt/riscvmatched/riscvmatched_board.py")
gem5_add_pysource("gem5.prebuilt.riscvmatched" "gem5/prebuilt/riscvmatched/riscvmatched_cache.py")
gem5_add_pysource("gem5.prebuilt.riscvmatched" "gem5/prebuilt/riscvmatched/riscvmatched_processor.py")
gem5_add_pysource("gem5.prebuilt.riscvmatched" "gem5/prebuilt/riscvmatched/riscvmatched_core.py")

# gem5.prebuilt.viper
gem5_add_pysource("gem5.prebuilt.viper" "gem5/prebuilt/viper/__init__.py")
gem5_add_pysource("gem5.prebuilt.viper" "gem5/prebuilt/viper/board.py")
gem5_add_pysource("gem5.prebuilt.viper" "gem5/prebuilt/viper/cpu_cache_hierarchy.py")
gem5_add_pysource("gem5.prebuilt.viper" "gem5/prebuilt/viper/gpu_cache_hierarchy.py")
gem5_add_pysource("gem5.prebuilt.viper" "gem5/prebuilt/viper/viper_network.py")

# gem5.resources
gem5_add_pysource("gem5.resources" "gem5/resources/__init__.py")
gem5_add_pysource("gem5.resources" "gem5/resources/client.py")
gem5_add_pysource("gem5.resources" "gem5/resources/downloader.py")
gem5_add_pysource("gem5.resources" "gem5/resources/md5_utils.py")
gem5_add_pysource("gem5.resources" "gem5/resources/resource.py")
gem5_add_pysource("gem5.resources" "gem5/resources/workload.py")
gem5_add_pysource("gem5.resources" "gem5/resources/looppoint.py")
gem5_add_pysource("gem5.resources" "gem5/resources/elfie.py")

# gem5.resources.client_api
gem5_add_pysource("gem5.resources.client_api" "gem5/resources/client_api/__init__.py")
gem5_add_pysource("gem5.resources.client_api" "gem5/resources/client_api/jsonclient.py")
gem5_add_pysource("gem5.resources.client_api" "gem5/resources/client_api/azure_functions_client.py")
gem5_add_pysource("gem5.resources.client_api" "gem5/resources/client_api/abstract_client.py")
gem5_add_pysource("gem5.resources.client_api" "gem5/resources/client_api/client_query.py")

# gem5.utils
gem5_add_pysource("gem5.utils" "gem5/utils/__init__.py")
gem5_add_pysource("gem5.utils" "gem5/utils/simpoint.py")
gem5_add_pysource("gem5.utils" "gem5/utils/filelock.py")
gem5_add_pysource("gem5.utils" "gem5/utils/override.py")
gem5_add_pysource("gem5.utils" "gem5/utils/progress_bar.py")
gem5_add_pysource("gem5.utils" "gem5/utils/requires.py")
gem5_add_pysource("gem5.utils" "gem5/utils/socks_ssl_context.py")

# gem5.utils.multisim
gem5_add_pysource("gem5.utils.multisim" "gem5/utils/multisim/__init__.py")
gem5_add_pysource("gem5.utils.multisim" "gem5/utils/multisim/multisim.py")
gem5_add_pysource("gem5.utils.multisim" "gem5/utils/multisim/__main__.py")

# gem5.utils.multiprocessing
gem5_add_pysource("gem5.utils.multiprocessing" "gem5/utils/multiprocessing/__init__.py")
gem5_add_pysource("gem5.utils.multiprocessing" "gem5/utils/multiprocessing/_command_line.py")
gem5_add_pysource("gem5.utils.multiprocessing" "gem5/utils/multiprocessing/context.py")
gem5_add_pysource("gem5.utils.multiprocessing" "gem5/utils/multiprocessing/popen_spawn_gem5.py")

# ---------------------------------------------------------------------------
# m5 core Python modules
# ---------------------------------------------------------------------------

gem5_add_pysource("m5" "m5/__init__.py")
gem5_add_pysource("m5" "m5/SimObject.py")
gem5_add_pysource("m5" "m5/citations.py")
gem5_add_pysource("m5" "m5/core.py")
gem5_add_pysource("m5" "m5/debug.py")
gem5_add_pysource("m5" "m5/event.py")
gem5_add_pysource("m5" "m5/main.py")
gem5_add_pysource("m5" "m5/options.py")
gem5_add_pysource("m5" "m5/proxy.py")
gem5_add_pysource("m5" "m5/simulate.py")
gem5_add_pysource("m5" "m5/ticks.py")
gem5_add_pysource("m5" "m5/trace.py")

# m5.objects
gem5_add_pysource("m5.objects" "m5/objects/__init__.py")

# m5.params
gem5_add_pysource("m5.params" "m5/params/__init__.py")
gem5_add_pysource("m5.params" "m5/params/base_params.py")
gem5_add_pysource("m5.params" "m5/params/deprecated_params.py")
gem5_add_pysource("m5.params" "m5/params/enum_params.py")
gem5_add_pysource("m5.params" "m5/params/network_params.py")
gem5_add_pysource("m5.params" "m5/params/null_params.py")
gem5_add_pysource("m5.params" "m5/params/pin_params.py")
gem5_add_pysource("m5.params" "m5/params/param_types.py")
gem5_add_pysource("m5.params" "m5/params/port_params.py")
gem5_add_pysource("m5.params" "m5/params/power_params.py")
gem5_add_pysource("m5.params" "m5/params/time_params.py")

# m5.stats
gem5_add_pysource("m5.stats" "m5/stats/__init__.py")
gem5_add_pysource("m5.stats" "m5/stats/gem5stats.py")

# m5.util
gem5_add_pysource("m5.util" "m5/util/__init__.py")
gem5_add_pysource("m5.util" "m5/util/attrdict.py")
gem5_add_pysource("m5.util" "m5/util/convert.py")
gem5_add_pysource("m5.util" "m5/util/dot_writer.py")
gem5_add_pysource("m5.util" "m5/util/dot_writer_ruby.py")
gem5_add_pysource("m5.util" "m5/util/fdthelper.py")
gem5_add_pysource("m5.util" "m5/util/multidict.py")
gem5_add_pysource("m5.util" "m5/util/pybind.py")
gem5_add_pysource("m5.util" "m5/util/terminal.py")
gem5_add_pysource("m5.util" "m5/util/terminal_formatter.py")

# m5.ext
gem5_add_pysource("m5.ext" "m5/ext/__init__.py")

# m5.ext.pyfdt
gem5_add_pysource("m5.ext.pyfdt" "m5/ext/pyfdt/pyfdt.py")
gem5_add_pysource("m5.ext.pyfdt" "m5/ext/pyfdt/__init__.py")

# m5.ext.pystats
gem5_add_pysource("m5.ext.pystats" "m5/ext/pystats/__init__.py")
gem5_add_pysource("m5.ext.pystats" "m5/ext/pystats/serializable_stat.py")
gem5_add_pysource("m5.ext.pystats" "m5/ext/pystats/abstract_stat.py")
gem5_add_pysource("m5.ext.pystats" "m5/ext/pystats/group.py")
gem5_add_pysource("m5.ext.pystats" "m5/ext/pystats/simstat.py")
gem5_add_pysource("m5.ext.pystats" "m5/ext/pystats/statistic.py")
gem5_add_pysource("m5.ext.pystats" "m5/ext/pystats/storagetype.py")
gem5_add_pysource("m5.ext.pystats" "m5/ext/pystats/timeconversion.py")
gem5_add_pysource("m5.ext.pystats" "m5/ext/pystats/jsonloader.py")

# ---------------------------------------------------------------------------
# C++ sources (pybind11 bindings)
# embedded.cc and importer.cc define the EmbeddedPython infrastructure used
# by all marshalled .py.cc files. They must be in gem5_all (not just gem5py_m5).
# importer.cc depends on the m5ImporterCode blob header.
#
# All of these require pybind11/Python headers and are excluded from
# no-Python (C++ config / shared-library) builds.
# ---------------------------------------------------------------------------
if(NOT GEM5_WITHOUT_PYTHON)
    gem5_add_source(embedded.cc)
    gem5_add_source(importer.cc)

    gem5_add_sources(
        pybind11/core.cc
        pybind11/debug.cc
        pybind11/event.cc
        pybind11/object_file.cc
        pybind11/port.cc
        pybind11/stats.cc
    )
endif()

# ---------------------------------------------------------------------------
# SimObjects
# ---------------------------------------------------------------------------

gem5_add_simobject(m5/objects/SimObject.py
    SIM_OBJECTS SimObject
    ENUMS ByteOrder
)
