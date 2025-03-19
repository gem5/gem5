from m5.params import *
from m5.objects.Gic import *
from m5.objects.Device import BasicPioDevice

class Vortex(PioDevice):
    type = 'Vortex'
    cxx_class = 'gem5::Vortex'
    cxx_header = "vortex-gpu/vortex_gpu.hh"

    pio_addr = Param.Addr("Device bases addr")
    int_gpu = Param.UInt32("Interrupt Number for Vortex GPU")

