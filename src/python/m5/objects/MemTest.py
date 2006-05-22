from m5 import *
class MemTest(SimObject):
    type = 'MemTest'
    cache = Param.BaseCache("L1 cache")
    check_mem = Param.FunctionalMemory("check memory")
    main_mem = Param.FunctionalMemory("hierarchical memory")
    max_loads = Param.Counter("number of loads to execute")
    memory_size = Param.Int(65536, "memory size")
    percent_copies = Param.Percent(0, "target copy percentage")
    percent_dest_unaligned = Param.Percent(50,
        "percent of copy dest address that are unaligned")
    percent_reads = Param.Percent(65, "target read percentage")
    percent_source_unaligned = Param.Percent(50,
        "percent of copy source address that are unaligned")
    percent_uncacheable = Param.Percent(10,
        "target uncacheable percentage")
    progress_interval = Param.Counter(1000000,
        "progress report interval (in accesses)")
    trace_addr = Param.Addr(0, "address to trace")
