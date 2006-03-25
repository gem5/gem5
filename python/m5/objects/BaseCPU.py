from m5 import *
class BaseCPU(SimObject):
    type = 'BaseCPU'
    abstract = True

    if build_env['FULL_SYSTEM']:
        dtb = Param.AlphaDTB("Data TLB")
        itb = Param.AlphaITB("Instruction TLB")
        system = Param.System(Parent.any, "system object")
        cpu_id = Param.Int(-1, "CPU identifier")
    else:
        mem = Param.MemObject("memory")
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
