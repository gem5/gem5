from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice

class AlphaConsole(BasicPioDevice):
    type = 'AlphaConsole'
    cpu = Param.BaseCPU(Parent.any, "Processor")
    disk = Param.SimpleDisk("Simple Disk")
    sim_console = Param.SimConsole(Parent.any, "The Simulator Console")
    system = Param.AlphaSystem(Parent.any, "system object")
