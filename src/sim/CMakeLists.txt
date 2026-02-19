# src/sim/CMakeLists.txt
# Converted from src/sim/SConscript

# --- Sources ---
gem5_add_source(async.cc)
gem5_add_source(backtrace_${GEM5_BACKTRACE_IMPL}.cc)
gem5_add_source(bufval.cc)
gem5_add_source(core.cc)
gem5_add_source(cur_tick.cc)

# tags.cc is generated at configure time by cpt_upgrader.py
set(_tags_cc "${GEM5_GEN_DIR}/sim/tags.cc")
file(MAKE_DIRECTORY "${GEM5_GEN_DIR}/sim")
execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/util/cpt_upgrader.py" --get-cc-file
    OUTPUT_FILE "${_tags_cc}"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE _tags_result
)
if(NOT _tags_result EQUAL 0)
    message(FATAL_ERROR "Failed to generate sim/tags.cc")
endif()
gem5_add_generated_source("${_tags_cc}")

gem5_add_source(cxx_config.cc)
gem5_add_source(cxx_manager.cc)
gem5_add_source(cxx_config_ini.cc)
gem5_add_source(debug.cc)
gem5_add_source(drain.cc)
gem5_add_source(eventq.cc)
gem5_add_source(futex_map.cc)
gem5_add_source(global_event.cc)
gem5_add_source(globals.cc)
gem5_add_source(init_signals.cc)
gem5_add_source(main.cc)
gem5_add_source(kernel_workload.cc)
gem5_add_source(port.cc)
gem5_add_source(redirect_path.cc)
gem5_add_source(root.cc)
gem5_add_source(serialize.cc)
gem5_add_source(se_workload.cc)
gem5_add_source(sim_events.cc)
gem5_add_source(sim_object.cc)
gem5_add_source(sub_system.cc)
gem5_add_source(ticked_object.cc)
gem5_add_source(simulate.cc)
gem5_add_source(stat_control.cc)

# Python bridge files: require pybind11/Python headers, excluded from
# no-Python (C++ config / shared-library) builds.
if(NOT GEM5_WITHOUT_PYTHON)
    gem5_add_source(py_interact.cc)
    gem5_add_source(init.cc)
    gem5_add_source(python.cc)
    gem5_add_source(stat_register.cc)
endif()
gem5_add_source(clock_domain.cc)
gem5_add_source(voltage_domain.cc)
gem5_add_source(se_signal.cc)
gem5_add_source(linear_solver.cc)
gem5_add_source(system.cc)
gem5_add_source(dvfs_handler.cc)
gem5_add_source(clocked_object.cc)
gem5_add_source(mathexpr.cc)
gem5_add_source(power_state.cc)
gem5_add_source(power_domain.cc)
gem5_add_source(stats.cc)
gem5_add_source(workload.cc)
gem5_add_source(mem_pool.cc)
gem5_add_source(faults.cc)
gem5_add_source(process.cc)
gem5_add_source(fd_array.cc)
gem5_add_source(fd_entry.cc)
gem5_add_source(mem_state.cc)
gem5_add_source(pseudo_inst.cc)
gem5_add_source(syscall_emul.cc)
gem5_add_source(syscall_desc.cc)
gem5_add_source(vma.cc)

# --- Debug flags ---
gem5_add_debug_flag(Checkpoint "Checkpoint debugging")
gem5_add_debug_flag(Config "Config debugging")
gem5_add_debug_flag(CxxConfig "CxxConfig debugging")
gem5_add_debug_flag(Drain "Drain debugging")
gem5_add_debug_flag(Event "Event debugging")
gem5_add_debug_flag(ExternalSignal "External signal debugging")
gem5_add_debug_flag(Flow "Flow debugging")
gem5_add_debug_flag(IPI "IPI debugging")
gem5_add_debug_flag(IPR "IPR debugging")
gem5_add_debug_flag(Interrupt "Interrupt debugging")
gem5_add_debug_flag(Loader "Loader debugging")
gem5_add_debug_flag(PseudoInst "PseudoInst debugging")
gem5_add_debug_flag(Stack "Stack debugging")
gem5_add_debug_flag(SyscallBase "Syscall base debugging")
gem5_add_debug_flag(SyscallVerbose "Syscall verbose debugging")
gem5_add_debug_flag(TimeSync "TimeSync debugging")
gem5_add_debug_flag(Thread "Thread debugging")
gem5_add_debug_flag(Timer "Timer debugging")
gem5_add_debug_flag(VtoPhys "VtoPhys debugging")
gem5_add_debug_flag(WorkItems "WorkItems debugging")
gem5_add_debug_flag(ClockDomain "ClockDomain debugging")
gem5_add_debug_flag(VoltageDomain "VoltageDomain debugging")
gem5_add_debug_flag(DVFS "DVFS debugging")
gem5_add_debug_flag(Vma "Vma debugging")
gem5_add_debug_flag(PowerDomain "PowerDomain debugging")
gem5_add_debug_flag(EnteringEventQueue "EnteringEventQueue debugging")

# Compound flags
gem5_add_debug_flag(SyscallAll "All syscall debugging"
    COMPONENTS SyscallBase SyscallVerbose)

# --- Subdirectories ---
add_subdirectory(power)
add_subdirectory(probe)

# --- SimObjects ---
gem5_add_simobject(ClockedObject.py SIM_OBJECTS ClockedObject)
gem5_add_simobject(TickedObject.py SIM_OBJECTS TickedObject)
gem5_add_simobject(Workload.py SIM_OBJECTS
    Workload StubWorkload KernelWorkload SEWorkload
    ENUMS KernelPanicOopsBehaviour)
gem5_add_simobject(Root.py SIM_OBJECTS Root)
gem5_add_simobject(ClockDomain.py SIM_OBJECTS
    ClockDomain SrcClockDomain DerivedClockDomain)
gem5_add_simobject(VoltageDomain.py SIM_OBJECTS VoltageDomain)
gem5_add_simobject(System.py SIM_OBJECTS System ENUMS MemoryMode)
gem5_add_simobject(DVFSHandler.py SIM_OBJECTS DVFSHandler)
gem5_add_simobject(SubSystem.py SIM_OBJECTS SubSystem)
gem5_add_simobject(RedirectPath.py SIM_OBJECTS RedirectPath)
gem5_add_simobject(PowerState.py SIM_OBJECTS PowerState ENUMS PwrState)
gem5_add_simobject(PowerDomain.py SIM_OBJECTS PowerDomain)
gem5_add_simobject(SignalPort.py)
gem5_add_simobject(InstTracer.py SIM_OBJECTS InstTracer InstDisassembler)
gem5_add_simobject(Process.py SIM_OBJECTS Process EmulatedDriver)
