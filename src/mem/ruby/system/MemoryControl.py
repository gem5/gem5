from m5.params import *
from m5.SimObject import SimObject

class RubyMemoryControl(SimObject):
    type = 'RubyMemoryControl'
    cxx_class = 'MemoryControl'
    version = Param.Int("");
    mem_bus_cycle_multiplier = Param.Int(10, "");
    banks_per_rank = Param.Int(8, "");
    ranks_per_dimm = Param.Int(2, "");
    dimms_per_channel = Param.Int(2, "");
    bank_bit_0 = Param.Int(8, "");
    rank_bit_0 = Param.Int(11, "");
    dimm_bit_0 = Param.Int(12, "");
    bank_queue_size = Param.Int(12, "");
    bank_busy_time = Param.Int(11, "");
    rank_rank_delay = Param.Int(1, "");
    read_write_delay = Param.Int(2, "");
    basic_bus_busy_time = Param.Int(2, "");
    mem_ctl_latency = Param.Int(12, "");
    refresh_period = Param.Int(1560, "");
    tFaw = Param.Int(0, "");
    mem_random_arbitrate = Param.Int(0, "");
    mem_fixed_delay = Param.Int(0, "");
