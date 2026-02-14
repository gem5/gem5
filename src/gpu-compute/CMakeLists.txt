# src/gpu-compute/CMakeLists.txt
# Converted from src/gpu-compute/SConscript
# Conditional on CONF_BUILD_GPU (guarded by parent)

# --- Sources ---
gem5_add_sources(
    comm.cc
    compute_unit.cc
    dispatcher.cc
    exec_stage.cc
    fetch_stage.cc
    fetch_unit.cc
    global_memory_pipeline.cc
    gpu_command_processor.cc
    gpu_compute_driver.cc
    gpu_dyn_inst.cc
    gpu_exec_context.cc
    gpu_render_driver.cc
    gpu_static_inst.cc
    lds_state.cc
    local_memory_pipeline.cc
    pool_manager.cc
    register_file.cc
    register_manager.cc
    scalar_memory_pipeline.cc
    scalar_register_file.cc
    schedule_stage.cc
    scheduler.cc
    scoreboard_check_stage.cc
    shader.cc
    dyn_pool_manager.cc
    simple_pool_manager.cc
    static_register_manager_policy.cc
    vector_register_file.cc
    register_file_cache.cc
    wavefront.cc
)

# --- SimObjects ---
gem5_add_simobject(GPU.py
    SIM_OBJECTS
        PoolManager SimplePoolManager DynPoolManager RegisterFile
        ScalarRegisterFile VectorRegisterFile RegisterManager Wavefront
        ComputeUnit Shader GPUComputeDriver GPURenderDriver
        GPUDispatcher GPUCommandProcessor RegisterFileCache
    ENUMS
        PrefetchType GfxVersion StorageClassType)
gem5_add_simobject(GPUStaticInstFlags.py
    ENUMS GPUStaticInstFlags)
gem5_add_simobject(LdsState.py
    SIM_OBJECTS LdsState)

# --- Debug flags ---
gem5_add_debug_flag(GPUAgentDisp "")
gem5_add_debug_flag(GPUCoalescer "")
gem5_add_debug_flag(GPUCommandProc "")
gem5_add_debug_flag(GPUDriver "")
gem5_add_debug_flag(GPUInitAbi "")
gem5_add_debug_flag(GPUDisp "")
gem5_add_debug_flag(GPUExec "")
gem5_add_debug_flag(GPUFetch "")
gem5_add_debug_flag(GPUInst "")
gem5_add_debug_flag(GPUKernelInfo "")
gem5_add_debug_flag(GPULDS "")
gem5_add_debug_flag(GPUMem "")
gem5_add_debug_flag(GPUPort "")
gem5_add_debug_flag(GPUPrefetch "")
gem5_add_debug_flag(GPUReg "")
gem5_add_debug_flag(GPURename "")
gem5_add_debug_flag(GPURF "")
gem5_add_debug_flag(GPURfState "")
gem5_add_debug_flag(GPUSched "")
gem5_add_debug_flag(GPUShader "")
gem5_add_debug_flag(GPUSRF "")
gem5_add_debug_flag(GPUSync "")
gem5_add_debug_flag(GPUTLB "")
gem5_add_debug_flag(GPUTrace "")
gem5_add_debug_flag(GPUVRF "")
gem5_add_debug_flag(GPURFC "")
gem5_add_debug_flag(GPUVRFSched "")
gem5_add_debug_flag(GPUWgLatency "")
gem5_add_debug_flag(Predictor "")
gem5_add_debug_flag(WavefrontStack "")

gem5_add_debug_flag(GPUALL "All GPU debug flags"
    COMPONENTS GPUCoalescer GPUDisp GPUExec GPUFetch
               GPUMem GPUPort GPUSched GPUSRF GPUSync
               GPUTLB GPUVRF GPURFC GPUWgLatency
               GPUKernelInfo GPUInitAbi GPULDS)
