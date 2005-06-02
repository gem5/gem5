from m5 import *
class BaseCPU(SimObject):
    type = 'BaseCPU'
    abstract = True
    icache = Param.BaseMem(NULL, "L1 instruction cache object")
    dcache = Param.BaseMem(NULL, "L1 data cache object")

    if build_env['FULL_SYSTEM']:
        dtb = Param.AlphaDTB("Data TLB")
        itb = Param.AlphaITB("Instruction TLB")
        mem = Param.FunctionalMemory("memory")
        system = Param.BaseSystem(Parent.any, "system object")
    else:
        workload = VectorParam.Process("processes to run")

    max_insts_all_threads = Param.Counter(0,
        "terminate when all threads have reached this inst count")
    max_insts_any_thread = Param.Counter(0,
        "terminate when any thread reaches this inst count")
    max_loads_all_threads = Param.Counter(0,
        "terminate when all threads have reached this load count")
    max_loads_any_thread = Param.Counter(0,
        "terminate when any thread reaches this load count")

    defer_registration = Param.Bool(False,
        "defer registration with system (for sampling)")

    clock = Param.Clock(Parent.clock, "clock speed")
