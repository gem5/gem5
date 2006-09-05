from m5.params import *
from m5.proxy import *
from MemObject import *

class PhysicalMemory(MemObject):
    type = 'PhysicalMemory'
    port = Port("the access port")
    range = Param.AddrRange(AddrRange('128MB'), "Device Address")
    file = Param.String('', "memory mapped file")
    latency = Param.Latency(Parent.clock, "latency of an access")

class DRAMMemory(PhysicalMemory):
    type = 'DRAMMemory'
    # Many of these should be observed from the configuration
    cpu_ratio = Param.Int(5,"ratio between CPU speed and memory bus speed")
    mem_type = Param.String("SDRAM", "Type of DRAM (DRDRAM, SDRAM)")
    mem_actpolicy = Param.String("open", "Open/Close policy")
    memctrladdr_type = Param.String("interleaved", "Mapping interleaved or direct")
    bus_width = Param.Int(16, "")
    act_lat = Param.Int(2, "RAS to CAS delay")
    cas_lat = Param.Int(1, "CAS delay")
    war_lat = Param.Int(2, "write after read delay")
    pre_lat = Param.Int(2, "precharge delay")
    dpl_lat = Param.Int(2, "data in to precharge delay")
    trc_lat = Param.Int(6, "row cycle delay")
    num_banks = Param.Int(4, "Number of Banks")
    num_cpus = Param.Int(4, "Number of CPUs connected to DRAM")

