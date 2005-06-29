from m5 import *
from Device import PioDevice

class AlphaConsole(PioDevice):
    type = 'AlphaConsole'
    cpu = Param.BaseCPU(Parent.any, "Processor")
    disk = Param.SimpleDisk("Simple Disk")
    sim_console = Param.SimConsole(Parent.any, "The Simulator Console")
    system = Param.System(Parent.any, "system object")
