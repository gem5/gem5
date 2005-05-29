from m5 import *
from Device import PioDevice

class Uart(PioDevice):
    type = 'Uart'
    console = Param.SimConsole(Parent.any, "The console")
    size = Param.Addr(0x8, "Device size")
