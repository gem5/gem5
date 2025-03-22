from m5.objects.Device import BasicPioDevice
from m5.objects.Gic import *
from m5.params import *


class VortexVADD(PioDevice):
    type = 'VortexVADD'
    cxx_class = 'gem5::VortexVADD'
    cxx_header = "vortex-vadd/vortex_vadd.hh"

    pio_addr = Param.Addr("Device bases addr")
    int_gpu = Param.UInt32("Interrupt Number for Vortex GPU")
    num_clusters = Param.UInt32("Device number of clusters")
    num_cores = Param.UInt32("Device number of cores")
    num_warps = Param.UInt32("Device number of warps")
    num_threads = Param.UInt32("Device number of threads")
