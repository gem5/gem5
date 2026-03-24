# src/systemc/core/CMakeLists.txt
# Converted from src/systemc/core/SConscript

gem5_add_sources(
    channel.cc
    event.cc
    kernel.cc
    messages.cc
    module.cc
    object.cc
    port.cc
    process.cc
    sched_event.cc
    scheduler.cc
    sensitivity.cc
    time.cc
    sc_attr.cc
    sc_event.cc
    sc_export.cc
    sc_interface.cc
    sc_join.cc
    sc_main.cc
    sc_main_fiber.cc
    sc_module.cc
    sc_module_name.cc
    sc_object.cc
    sc_port.cc
    sc_process_handle.cc
    sc_prim.cc
    sc_sensitive.cc
    sc_simcontext.cc
    sc_spawn.cc
    sc_time.cc
)

if(NOT GEM5_WITHOUT_PYTHON)
    gem5_add_sources(
        python.cc
        sc_main_python.cc
        sc_time_python.cc
    )
endif()

gem5_add_simobject(SystemC.py
    SIM_OBJECTS SystemC_Kernel SystemC_ScObject SystemC_ScModule)
