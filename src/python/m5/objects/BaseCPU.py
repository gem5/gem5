from m5 import build_env
from m5.config import *

class BaseCPU(SimObject):
    type = 'BaseCPU'
    abstract = True
    mem = Param.MemObject("memory")

    system = Param.System(Parent.any, "system object")
    if build_env['FULL_SYSTEM']:
        dtb = Param.AlphaDTB("Data TLB")
        itb = Param.AlphaITB("Instruction TLB")
        cpu_id = Param.Int(-1, "CPU identifier")
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
