from m5 import *
from Device import PioDevice

class Uart(PioDevice):
    type = 'Uart'
    abstract = True
    console = Param.SimConsole(Parent.any, "The console")
    size = Param.Addr(0x8, "Device size")

class Uart8250(Uart):
    type = 'Uart8250'

if build_env['ALPHA_TLASER']:
    class Uart8530(Uart):
        type = 'Uart8530'

