from m5.params import *
from m5.proxy import *
from m5 import build_env
from Device import BasicPioDevice

class Uart(BasicPioDevice):
    type = 'Uart'
    abstract = True
    sim_console = Param.SimConsole(Parent.any, "The console")

class Uart8250(Uart):
    type = 'Uart8250'

if build_env['ALPHA_TLASER']:
    class Uart8530(Uart):
        type = 'Uart8530'

