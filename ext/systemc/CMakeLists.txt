# Detect architecture for coroutine library selection
include(CheckCSourceCompiles)
check_c_source_compiles("
    int main() {
    #ifndef __i386__
        #error not i386
    #endif
        return 0;
    }
" SYSTEMC_ARCH_I386)

check_c_source_compiles("
    int main() {
    #ifndef __x86_64__
        #error not x86_64
    #endif
        return 0;
    }
" SYSTEMC_ARCH_X86_64)

if(SYSTEMC_ARCH_I386)
    set(SYSTEMC_COROUTINE_LIB "qt")
    set(SYSTEMC_QT_ARCH "i386")
elseif(SYSTEMC_ARCH_X86_64)
    set(SYSTEMC_COROUTINE_LIB "qt")
    set(SYSTEMC_QT_ARCH "iX86_64")
else()
    message(WARNING "Unrecognized architecture for systemc, using pthreads coroutines")
    set(SYSTEMC_COROUTINE_LIB "pthreads")
endif()

# SystemC core sources (from SConscript.sc files)
set(SYSTEMC_SOURCES
    # kernel
    src/sysc/kernel/sc_attribute.cpp
    src/sysc/kernel/sc_cthread_process.cpp
    src/sysc/kernel/sc_event.cpp
    src/sysc/kernel/sc_except.cpp
    src/sysc/kernel/sc_join.cpp
    src/sysc/kernel/sc_main.cpp
    src/sysc/kernel/sc_main_main.cpp
    src/sysc/kernel/sc_method_process.cpp
    src/sysc/kernel/sc_module.cpp
    src/sysc/kernel/sc_module_name.cpp
    src/sysc/kernel/sc_module_registry.cpp
    src/sysc/kernel/sc_name_gen.cpp
    src/sysc/kernel/sc_object.cpp
    src/sysc/kernel/sc_object_manager.cpp
    src/sysc/kernel/sc_phase_callback_registry.cpp
    src/sysc/kernel/sc_process.cpp
    src/sysc/kernel/sc_reset.cpp
    src/sysc/kernel/sc_sensitive.cpp
    src/sysc/kernel/sc_simcontext.cpp
    src/sysc/kernel/sc_spawn_options.cpp
    src/sysc/kernel/sc_thread_process.cpp
    src/sysc/kernel/sc_time.cpp
    src/sysc/kernel/sc_ver.cpp
    src/sysc/kernel/sc_wait.cpp
    src/sysc/kernel/sc_wait_cthread.cpp

    # communication
    src/sysc/communication/sc_clock.cpp
    src/sysc/communication/sc_event_finder.cpp
    src/sysc/communication/sc_event_queue.cpp
    src/sysc/communication/sc_export.cpp
    src/sysc/communication/sc_interface.cpp
    src/sysc/communication/sc_mutex.cpp
    src/sysc/communication/sc_port.cpp
    src/sysc/communication/sc_prim_channel.cpp
    src/sysc/communication/sc_semaphore.cpp
    src/sysc/communication/sc_signal.cpp
    src/sysc/communication/sc_signal_ports.cpp
    src/sysc/communication/sc_signal_resolved.cpp
    src/sysc/communication/sc_signal_resolved_ports.cpp

    # datatypes/bit
    src/sysc/datatypes/bit/sc_bit.cpp
    src/sysc/datatypes/bit/sc_bv_base.cpp
    src/sysc/datatypes/bit/sc_logic.cpp
    src/sysc/datatypes/bit/sc_lv_base.cpp

    # datatypes/fx
    src/sysc/datatypes/fx/sc_fxcast_switch.cpp
    src/sysc/datatypes/fx/sc_fxdefs.cpp
    src/sysc/datatypes/fx/sc_fxnum.cpp
    src/sysc/datatypes/fx/sc_fxnum_observer.cpp
    src/sysc/datatypes/fx/sc_fxtype_params.cpp
    src/sysc/datatypes/fx/sc_fxval.cpp
    src/sysc/datatypes/fx/sc_fxval_observer.cpp
    src/sysc/datatypes/fx/scfx_mant.cpp
    src/sysc/datatypes/fx/scfx_pow10.cpp
    src/sysc/datatypes/fx/scfx_rep.cpp
    src/sysc/datatypes/fx/scfx_utils.cpp

    # datatypes/int
    src/sysc/datatypes/int/sc_int_base.cpp
    src/sysc/datatypes/int/sc_int32_mask.cpp
    src/sysc/datatypes/int/sc_int64_io.cpp
    src/sysc/datatypes/int/sc_int64_mask.cpp
    src/sysc/datatypes/int/sc_length_param.cpp
    src/sysc/datatypes/int/sc_nbdefs.cpp
    src/sysc/datatypes/int/sc_nbexterns.cpp
    src/sysc/datatypes/int/sc_nbutils.cpp
    src/sysc/datatypes/int/sc_signed.cpp
    src/sysc/datatypes/int/sc_uint_base.cpp
    src/sysc/datatypes/int/sc_unsigned.cpp

    # datatypes/misc
    src/sysc/datatypes/misc/sc_concatref.cpp
    src/sysc/datatypes/misc/sc_value_base.cpp

    # tracing
    src/sysc/tracing/sc_trace.cpp
    src/sysc/tracing/sc_trace_file_base.cpp
    src/sysc/tracing/sc_vcd_trace.cpp
    src/sysc/tracing/sc_wif_trace.cpp

    # utils
    src/sysc/utils/sc_hash.cpp
    src/sysc/utils/sc_list.cpp
    src/sysc/utils/sc_mempool.cpp
    src/sysc/utils/sc_pq.cpp
    src/sysc/utils/sc_report.cpp
    src/sysc/utils/sc_report_handler.cpp
    src/sysc/utils/sc_stop_here.cpp
    src/sysc/utils/sc_string.cpp
    src/sysc/utils/sc_utils_ids.cpp
    src/sysc/utils/sc_vector.cpp
)

# Add coroutine-specific sources
if(SYSTEMC_COROUTINE_LIB STREQUAL "qt")
    list(APPEND SYSTEMC_SOURCES
        src/sysc/kernel/sc_cor_qt.cpp
        src/sysc/qt/qt.c
        src/sysc/qt/md/${SYSTEMC_QT_ARCH}.s
    )
elseif(SYSTEMC_COROUTINE_LIB STREQUAL "pthreads")
    list(APPEND SYSTEMC_SOURCES
        src/sysc/kernel/sc_cor_pthread.cpp
    )
endif()

add_library(gem5_ext_systemc STATIC
    ${SYSTEMC_SOURCES}
)

target_include_directories(gem5_ext_systemc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_definitions(gem5_ext_systemc PRIVATE
    SC_INCLUDE_FX
)

if(SYSTEMC_COROUTINE_LIB STREQUAL "pthreads")
    target_compile_definitions(gem5_ext_systemc PRIVATE
        SC_USE_PTHREADS
    )
    find_package(Threads REQUIRED)
    target_link_libraries(gem5_ext_systemc PUBLIC Threads::Threads)
    target_compile_options(gem5_ext_systemc PRIVATE -pthread)
endif()

if(APPLE)
    target_link_options(gem5_ext_systemc PUBLIC -undefined dynamic_lookup)
endif()

# Enable assembly language for .s files
enable_language(ASM)

set_target_properties(gem5_ext_systemc PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
