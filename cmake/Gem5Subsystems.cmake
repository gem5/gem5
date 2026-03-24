# cmake/Gem5Subsystems.cmake
# Subsystem STATIC libraries: groups per-directory OBJECT libraries into
# logical subsystem archives with scoped ext/ library dependencies.
#
# Included FROM WITHIN cmake/Gem5Targets.cmake after codegen setup
# (gem5_generated, gem5_pysources, codegen dependency propagation)
# but before final target creation (gem5 executable, gem5_shared, tests).
#
# Each subsystem STATIC library:
#   - Composes related OBJECT libraries via $<TARGET_OBJECTS:...>
#   - Declares scoped ext/ link dependencies (actual link-time ownership)
#   - Declares inter-subsystem dependencies (for link-library transitivity;
#     header visibility is flat via gem5_deps)
#
# Ext/ library scoping contract:
#   Scoped ext/ libraries (gem5_ext_elf, gem5_ext_drampower, gem5_ext_libfdt,
#   gem5_ext_nomali, gem5_ext_dramsim*, gem5_ext_dramsys, gem5_ext_systemc)
#   are linked ONLY by their owning subsystem LINK_DEPS. Their include
#   directories are propagated globally via _gem5_propagate_ext_includes()
#   in Gem5Targets.cmake so OBJECT targets can compile, but the link
#   dependency flows only through the owning subsystem.
#
#   Global ext/ libraries (gem5_ext_fputils, gem5_ext_iostream3,
#   gem5_ext_magic_enum, gem5_ext_softfloat) remain on gem5_deps because
#   they are used pervasively across multiple subsystems.
#
# See Gem5Targets.cmake for the gem5_deps definition and scoping details.

# ---------------------------------------------------------------------------
# Subsystem Dependency Diagram (Simplified)
# ---------------------------------------------------------------------------
#
#                     gem5_base
#                    /    |    \
#               gem5_sim  |   gem5_kern
#              /    |     |
#         gem5_mem  |   gem5_dev
#            |      |      |
#       gem5_cpu    |   gem5_dev_{isa}
#            |      |
#     gem5_arch_{isa}
#            |
#     gem5_gpu_compute (optional)
#

# ---------------------------------------------------------------------------
# Global property for subsystem library names
# ---------------------------------------------------------------------------
_gem5_define_global_property(GEM5_SUBSYSTEM_LIBS
    "Subsystem STATIC library targets for gem5")

# ---------------------------------------------------------------------------
# gem5_define_subsystem(<name>
#     OBJECT_LIBS <lib1> [lib2 ...]
#     [LINK_DEPS <dep1> [dep2 ...]]
#     [SUBSYSTEM_DEPS <subsys1> [subsys2 ...]]
#     [CONDITION <cond>])
#
# Create a subsystem STATIC library from an explicit OBJECT library manifest.
# OBJECT_LIBS that do not exist as targets are silently skipped (conditional
# directories may not register sources depending on build configuration).
#
# LINK_DEPS: ext/ or system library link dependencies scoped to this subsystem.
# SUBSYSTEM_DEPS: other subsystem STATIC libraries this one depends on
#     (for link transitivity and documentation).
# CONDITION: if set and evaluates to FALSE, the subsystem is not created.
# ---------------------------------------------------------------------------
function(gem5_define_subsystem name)
    cmake_parse_arguments(ARG "" "CONDITION" "OBJECT_LIBS;LINK_DEPS;SUBSYSTEM_DEPS" ${ARGN})

    # Skip if condition is not met
    if(DEFINED ARG_CONDITION AND NOT "${ARG_CONDITION}" STREQUAL "")
        if(NOT ${ARG_CONDITION})
            return()
        endif()
    endif()

    set(_target "gem5_${name}")

    # Collect TARGET_OBJECTS from existing OBJECT libraries
    set(_obj_sources "")
    set(_found_any FALSE)
    foreach(_lib ${ARG_OBJECT_LIBS})
        if(TARGET "${_lib}")
            list(APPEND _obj_sources "$<TARGET_OBJECTS:${_lib}>")
            set(_found_any TRUE)
        endif()
    endforeach()

    if(NOT _found_any)
        # No OBJECT libraries exist for this subsystem (all conditional and
        # disabled). Skip creating the STATIC library.
        return()
    endif()

    # Create STATIC library from composed OBJECT targets
    add_library(${_target} STATIC ${_obj_sources})
    set_target_properties(${_target} PROPERTIES POSITION_INDEPENDENT_CODE ON)

    # OBJECT targets already compile with gem5_deps. The STATIC library
    # links gem5_deps PRIVATE so CMake records the dependency for
    # archiving; PRIVATE means this is NOT propagated to consumers
    # (final targets get gem5_deps through their own link lines).
    target_link_libraries(${_target} PRIVATE gem5_deps)

    # Scoped ext/ library dependencies
    if(ARG_LINK_DEPS)
        target_link_libraries(${_target} PUBLIC ${ARG_LINK_DEPS})
    endif()

    # Inter-subsystem dependencies
    if(ARG_SUBSYSTEM_DEPS)
        foreach(_dep ${ARG_SUBSYSTEM_DEPS})
            if(TARGET "${_dep}")
                target_link_libraries(${_target} PUBLIC ${_dep})
            endif()
        endforeach()
    endif()

    # Propagate codegen dependencies
    if(_codegen_deps)
        add_dependencies(${_target} ${_codegen_deps})
    endif()

    # Register in global subsystem list
    set_property(GLOBAL APPEND PROPERTY GEM5_SUBSYSTEM_LIBS "${_target}")
endfunction()

# ---------------------------------------------------------------------------
# gem5_get_subsystem_libs(<out_var>)
#
# Collect all registered subsystem STATIC library names.
# ---------------------------------------------------------------------------
function(gem5_get_subsystem_libs out_var)
    get_property(_libs GLOBAL PROPERTY GEM5_SUBSYSTEM_LIBS)
    set(${out_var} ${_libs} PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_subsystem_link_dep(<subsystem_name> <library>)
#
# Declare an additional link dependency for a subsystem STATIC library.
# The subsystem target must already exist (created by gem5_define_subsystem).
# This is used for conditional ext/ library additions (e.g., DRAMSim/DRAMSys)
# that are determined after the subsystem is initially defined.
# ---------------------------------------------------------------------------
function(gem5_add_subsystem_link_dep subsystem_name library)
    set(_target "gem5_${subsystem_name}")
    if(NOT TARGET "${_target}")
        return()
    endif()
    target_link_libraries(${_target} PUBLIC ${library})
endfunction()

# ===================================================================
# Subsystem Definitions (Explicit Manifest)
# ===================================================================
# Each subsystem lists its OBJECT libraries explicitly (no prefix matching)
# to avoid misclassification (e.g., prefix matching would put gem5_obj_dev_arm
# into gem5_dev instead of gem5_dev_arm, since both start with "dev").
# The manifest must be updated when a new src/ subdirectory is added.

# --- Core subsystems (always built) ---

gem5_define_subsystem(base
    OBJECT_LIBS
        gem5_obj_base
        gem5_obj_base_filters
        gem5_obj_base_loader
        gem5_obj_base_stats
        gem5_obj_base_vnc
    LINK_DEPS
        gem5_ext_elf
)

gem5_define_subsystem(sim
    OBJECT_LIBS
        gem5_obj_sim
        gem5_obj_sim_power
        gem5_obj_sim_probe
    SUBSYSTEM_DEPS
        gem5_base
)

gem5_define_subsystem(mem
    OBJECT_LIBS
        gem5_obj_mem
        gem5_obj_mem_cache
        gem5_obj_mem_cache_compressors
        gem5_obj_mem_cache_compressors_encoders
        gem5_obj_mem_cache_prefetch
        gem5_obj_mem_cache_replacement_policies
        gem5_obj_mem_cache_tags
        gem5_obj_mem_cache_tags_indexing_policies
        gem5_obj_mem_cache_tags_partitioning_policies
        gem5_obj_mem_probes
        gem5_obj_mem_qos
    LINK_DEPS
        gem5_ext_drampower
    SUBSYSTEM_DEPS
        gem5_base
        gem5_sim
)
# Conditional DRAMSim/DRAMSys ext/ libraries scoped to gem5_mem
if(HAVE_DRAMSIM)
    gem5_add_subsystem_link_dep(mem gem5_ext_dramsim2)
endif()
if(HAVE_DRAMSIM3)
    gem5_add_subsystem_link_dep(mem gem5_ext_dramsim3)
endif()
if(HAVE_DRAMSYS)
    gem5_add_subsystem_link_dep(mem gem5_ext_dramsys)
endif()

gem5_define_subsystem(mem_ruby
    OBJECT_LIBS
        gem5_obj_mem_ruby_common
        gem5_obj_mem_ruby_network
        gem5_obj_mem_ruby_network_fault_model
        gem5_obj_mem_ruby_network_garnet
        gem5_obj_mem_ruby_network_simple
        gem5_obj_mem_ruby_profiler
        gem5_obj_mem_ruby_protocol_chi_generic
        gem5_obj_mem_ruby_slicc_interface
        gem5_obj_mem_ruby_structures
        gem5_obj_mem_ruby_system
        gem5_obj_mem_protocol
    CONDITION CONF_RUBY
    SUBSYSTEM_DEPS
        gem5_mem
)

gem5_define_subsystem(cpu
    OBJECT_LIBS
        gem5_obj_cpu
        gem5_obj_cpu_o3
        gem5_obj_cpu_o3_probe
        gem5_obj_cpu_minor
        gem5_obj_cpu_simple
        gem5_obj_cpu_simple_probes
        gem5_obj_cpu_pred
        gem5_obj_cpu_checker
        gem5_obj_cpu_trace
        gem5_obj_cpu_testers_directedtest
        gem5_obj_cpu_testers_garnet_synthetic_traffic
        gem5_obj_cpu_testers_gpu_ruby_test
        gem5_obj_cpu_testers_memtest
        gem5_obj_cpu_testers_rubytest
        gem5_obj_cpu_testers_spatter_gen
        gem5_obj_cpu_testers_traffic_gen
        gem5_obj_cpu_probes
    SUBSYSTEM_DEPS
        gem5_base
        gem5_sim
        gem5_mem
        gem5_arch_generic
)

gem5_define_subsystem(dev
    OBJECT_LIBS
        gem5_obj_dev
        gem5_obj_dev_net
        gem5_obj_dev_pci
        gem5_obj_dev_serial
        gem5_obj_dev_i2c
        gem5_obj_dev_ps2
        gem5_obj_dev_storage
        gem5_obj_dev_virtio
        gem5_obj_dev_qemu
        gem5_obj_dev_lupio
    LINK_DEPS
        gem5_ext_libfdt
        gem5_ext_nomali
    SUBSYSTEM_DEPS
        gem5_base
        gem5_sim
        gem5_mem
)

gem5_define_subsystem(kern
    OBJECT_LIBS
        gem5_obj_kern
    SUBSYSTEM_DEPS
        gem5_base
        gem5_sim
        gem5_mem
)

gem5_define_subsystem(arch_generic
    OBJECT_LIBS
        gem5_obj_arch_generic
)

# --- ISA-conditional subsystems ---

gem5_define_subsystem(arch_x86
    OBJECT_LIBS gem5_obj_arch_x86
    CONDITION CONF_USE_X86_ISA
    SUBSYSTEM_DEPS gem5_base gem5_sim gem5_mem gem5_cpu gem5_arch_generic
)

gem5_define_subsystem(arch_arm
    OBJECT_LIBS gem5_obj_arch_arm
    CONDITION CONF_USE_ARM_ISA
    SUBSYSTEM_DEPS gem5_base gem5_sim gem5_mem gem5_cpu gem5_arch_generic
)

gem5_define_subsystem(arch_riscv
    OBJECT_LIBS gem5_obj_arch_riscv
    CONDITION CONF_USE_RISCV_ISA
    SUBSYSTEM_DEPS gem5_base gem5_sim gem5_mem gem5_cpu gem5_arch_generic
)

gem5_define_subsystem(arch_mips
    OBJECT_LIBS gem5_obj_arch_mips
    CONDITION CONF_USE_MIPS_ISA
    SUBSYSTEM_DEPS gem5_base gem5_sim gem5_mem gem5_cpu gem5_arch_generic
)

gem5_define_subsystem(arch_power
    OBJECT_LIBS gem5_obj_arch_power
    CONDITION CONF_USE_POWER_ISA
    SUBSYSTEM_DEPS gem5_base gem5_sim gem5_mem gem5_cpu gem5_arch_generic
)

gem5_define_subsystem(arch_sparc
    OBJECT_LIBS gem5_obj_arch_sparc
    CONDITION CONF_USE_SPARC_ISA
    SUBSYSTEM_DEPS gem5_base gem5_sim gem5_mem gem5_cpu gem5_arch_generic
)

gem5_define_subsystem(arch_amdgpu
    OBJECT_LIBS gem5_obj_arch_amdgpu
    CONDITION CONF_BUILD_GPU
    SUBSYSTEM_DEPS gem5_base gem5_sim gem5_mem gem5_cpu gem5_arch_generic
)

gem5_define_subsystem(dev_arm
    OBJECT_LIBS gem5_obj_dev_arm gem5_obj_dev_arm_css
    CONDITION CONF_USE_ARM_ISA
    SUBSYSTEM_DEPS gem5_dev gem5_arch_arm
)

gem5_define_subsystem(dev_x86
    OBJECT_LIBS gem5_obj_dev_x86
    CONDITION CONF_USE_X86_ISA
    SUBSYSTEM_DEPS gem5_dev gem5_arch_x86
)

gem5_define_subsystem(dev_riscv
    OBJECT_LIBS gem5_obj_dev_riscv
    CONDITION CONF_USE_RISCV_ISA
    SUBSYSTEM_DEPS gem5_dev gem5_arch_riscv
)

gem5_define_subsystem(dev_mips
    OBJECT_LIBS gem5_obj_dev_mips
    CONDITION CONF_USE_MIPS_ISA
    SUBSYSTEM_DEPS gem5_dev gem5_arch_mips
)

gem5_define_subsystem(dev_sparc
    OBJECT_LIBS gem5_obj_dev_sparc
    CONDITION CONF_USE_SPARC_ISA
    SUBSYSTEM_DEPS gem5_dev gem5_arch_sparc
)

gem5_define_subsystem(cpu_kvm
    OBJECT_LIBS gem5_obj_cpu_kvm
    CONDITION CONF_USE_KVM
    SUBSYSTEM_DEPS gem5_cpu
)

# --- Feature-conditional subsystems ---

if(NOT GEM5_WITHOUT_PYTHON)
    gem5_define_subsystem(python
        OBJECT_LIBS gem5_obj_python
        SUBSYSTEM_DEPS gem5_base gem5_sim
    )
endif()

gem5_define_subsystem(proto
    OBJECT_LIBS gem5_obj_proto
    CONDITION HAVE_PROTOBUF
    SUBSYSTEM_DEPS gem5_base gem5_sim
)

gem5_define_subsystem(gpu_compute
    OBJECT_LIBS gem5_obj_gpu-compute
    CONDITION CONF_BUILD_GPU
    SUBSYSTEM_DEPS gem5_base gem5_sim gem5_mem gem5_cpu
)

gem5_define_subsystem(dev_hsa
    OBJECT_LIBS gem5_obj_dev_hsa
    CONDITION CONF_BUILD_GPU
    SUBSYSTEM_DEPS gem5_dev
)

gem5_define_subsystem(dev_amdgpu
    OBJECT_LIBS gem5_obj_dev_amdgpu
    CONDITION CONF_BUILD_GPU
    SUBSYSTEM_DEPS gem5_dev
)

gem5_define_subsystem(systemc
    OBJECT_LIBS
        gem5_obj_systemc_channel
        gem5_obj_systemc_core
        gem5_obj_systemc_dt
        gem5_obj_systemc_dt_bit
        gem5_obj_systemc_dt_fx
        gem5_obj_systemc_dt_int
        gem5_obj_systemc_dt_misc
        gem5_obj_systemc_tlm_bridge
        gem5_obj_systemc_tlm_core_2_generic_payload
        gem5_obj_systemc_tlm_core_2_quantum
        gem5_obj_systemc_tlm_utils
        gem5_obj_systemc_utils
    LINK_DEPS
        gem5_ext_systemc
    CONDITION CONF_USE_SYSTEMC
)

gem5_define_subsystem(sst
    OBJECT_LIBS gem5_obj_sst
)

gem5_define_subsystem(test_objects
    OBJECT_LIBS gem5_obj_test_objects
    CONDITION CONF_USE_TEST_OBJECTS
)

gem5_define_subsystem(learning
    OBJECT_LIBS gem5_obj_learning_gem5_part2
)

# ===================================================================
# Orphan OBJECT Library Detection
# ===================================================================
# Safety check: compare registered OBJECT libraries against the subsystem
# manifests. Warn if any OBJECT library is not assigned to a subsystem.

function(_gem5_check_subsystem_orphans)
    get_property(_all_obj_libs GLOBAL PROPERTY GEM5_OBJECT_LIBS)
    get_property(_subsystem_libs GLOBAL PROPERTY GEM5_SUBSYSTEM_LIBS)

    # Collect all OBJECT libraries referenced by subsystems
    set(_assigned "")
    foreach(_subsys ${_subsystem_libs})
        get_target_property(_sources ${_subsys} SOURCES)
        # TARGET_OBJECTS generator expressions encode the OBJECT lib names
        foreach(_src ${_sources})
            if(_src MATCHES "\\$<TARGET_OBJECTS:([^>]+)>")
                list(APPEND _assigned "${CMAKE_MATCH_1}")
            endif()
        endforeach()
    endforeach()
    list(REMOVE_DUPLICATES _assigned)

    # Check for orphans
    foreach(_obj ${_all_obj_libs})
        if(NOT "${_obj}" IN_LIST _assigned)
            message(WARNING
                "OBJECT library '${_obj}' is not assigned to any subsystem.\n"
                "Add it to the appropriate gem5_define_subsystem() call in "
                "cmake/Gem5Subsystems.cmake.")
        endif()
    endforeach()
endfunction()

_gem5_check_subsystem_orphans()
