# src/cpu/CMakeLists.txt
# Converted from src/cpu/SConscript

# --- Unconditional sources ---
gem5_add_source(func_unit.cc)
gem5_add_source(pc_event.cc)
gem5_add_source(activity.cc)
gem5_add_source(base.cc)
gem5_add_source(exetrace.cc)
gem5_add_source(inteltrace.cc)
gem5_add_source(nativetrace.cc)
gem5_add_source(nop_static_inst.cc)
gem5_add_source(null_static_inst.cc)
gem5_add_source(profile.cc)
gem5_add_source(reg_class.cc)
gem5_add_source(static_inst.cc)
gem5_add_source(simple_thread.cc)
gem5_add_source(thread_context.cc)
gem5_add_source(thread_state.cc)
gem5_add_source(timing_expr.cc)

# Conditional: protobuf instruction tracer
if(HAVE_PROTOBUF)
    gem5_add_source(inst_pb_trace.cc)
endif()

# Conditional: Capstone disassembler
if(CONF_USE_CAPSTONE)
    gem5_add_source(capstone.cc)
endif()

# --- Debug flags ---
gem5_add_debug_flag(Activity "Pipeline activity")
gem5_add_debug_flag(Commit "Commit debugging")
gem5_add_debug_flag(Context "Context debugging")
gem5_add_debug_flag(Decode "Decode debugging")
gem5_add_debug_flag(DynInst "DynInst debugging")
gem5_add_debug_flag(ExecEnable "Filter: Enable exec tracing (no tracing without this)")
gem5_add_debug_flag(ExecCPSeq "Format: Instruction sequence number")
gem5_add_debug_flag(ExecEffAddr "Format: Include effective address")
gem5_add_debug_flag(ExecFaulting "Trace faulting instructions")
gem5_add_debug_flag(ExecFetchSeq "Format: Fetch sequence number")
gem5_add_debug_flag(ExecOpClass "Format: Include operand class")
gem5_add_debug_flag(ExecRegDelta "ExecRegDelta debugging")
gem5_add_debug_flag(ExecResult "Format: Include results from execution")
gem5_add_debug_flag(ExecSymbol "Format: Try to include symbol names")
gem5_add_debug_flag(ExecThread "Format: Include thread ID in trace")
gem5_add_debug_flag(ExecMicro "Filter: Include microops")
gem5_add_debug_flag(ExecMacro "Filter: Include macroops")
gem5_add_debug_flag(ExecUser "Filter: Trace user mode instructions")
gem5_add_debug_flag(ExecKernel "Filter: Trace kernel mode instructions")
gem5_add_debug_flag(ExecAsid "Format: Include ASID in trace")
gem5_add_debug_flag(ExecFlags "Format: Include instruction flags in trace")
gem5_add_debug_flag(Fetch "Fetch debugging")
gem5_add_debug_flag(HtmCpu "Hardware Transactional Memory (CPU side)")
gem5_add_debug_flag(InvalidReg "InvalidReg debugging")
gem5_add_debug_flag(O3PipeView "O3PipeView debugging")
gem5_add_debug_flag(PCEvent "PCEvent debugging")
gem5_add_debug_flag(Quiesce "Quiesce debugging")
gem5_add_debug_flag(Mwait "Mwait debugging")
gem5_add_debug_flag(Checker "Checker debugging")

# Compound flags
gem5_add_debug_flag(ExecAll "All exec tracing flags"
    COMPONENTS ExecEnable ExecCPSeq ExecEffAddr ExecFaulting ExecFetchSeq
               ExecOpClass ExecRegDelta ExecResult ExecSymbol ExecThread
               ExecMicro ExecMacro ExecUser ExecKernel ExecAsid ExecFlags)
gem5_add_debug_flag(Exec "Common exec tracing flags"
    COMPONENTS ExecEnable ExecOpClass ExecThread ExecEffAddr ExecResult
               ExecSymbol ExecMicro ExecMacro ExecFaulting ExecUser ExecKernel)
gem5_add_debug_flag(ExecNoTicks "Exec tracing without ticks"
    COMPONENTS Exec FmtTicksOff)

# --- SimObjects ---
gem5_add_simobject(FuncUnit.py
    SIM_OBJECTS OpDesc FUDesc
    ENUMS OpClass)
gem5_add_simobject(StaticInstFlags.py
    ENUMS StaticInstFlags)
gem5_add_simobject(BaseCPU.py
    SIM_OBJECTS BaseCPU)
gem5_add_simobject(CpuCluster.py
    SIM_OBJECTS CpuCluster)
gem5_add_simobject(CPUTracers.py
    SIM_OBJECTS ExeTracer IntelTrace NativeTrace)
gem5_add_simobject(TimingExpr.py
    SIM_OBJECTS TimingExpr TimingExprLiteral TimingExprSrcReg TimingExprLet
                TimingExprRef TimingExprUn TimingExprBin TimingExprIf
    ENUMS TimingExprOp)
gem5_add_simobject(DummyChecker.py
    SIM_OBJECTS DummyChecker)

# Conditional: Capstone disassembler
if(CONF_USE_CAPSTONE)
    gem5_add_simobject(Capstone.py
        SIM_OBJECTS CapstoneDisassembler)
endif()

# Conditional: protobuf instruction tracer
if(HAVE_PROTOBUF)
    gem5_add_simobject(InstPBTrace.py
        SIM_OBJECTS InstPBTrace)
endif()

# --- Subdirectories ---
add_subdirectory(checker)
add_subdirectory(pred)
add_subdirectory(probes)

# ISA-conditional subdirectories
if(CONF_BUILD_ISA)
    add_subdirectory(minor)
    add_subdirectory(o3)
    add_subdirectory(simple)
endif()

# KVM
if(CONF_USE_KVM)
    add_subdirectory(kvm)
endif()

# Trace CPU (protobuf-dependent)
if(HAVE_PROTOBUF)
    add_subdirectory(trace)
endif()

# Testers
add_subdirectory(testers)
