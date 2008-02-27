from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice, PioDevice, IsaFake, BadAddr
from Uart import Uart8250
from Platform import Platform
from Pci import PciConfigAll
from SimConsole import SimConsole

class Opteron(Platform):
    type = 'Opteron'
    system = Param.System(Parent.any, "system")

    pciconfig = PciConfigAll()

    def attachIO(self, bus):
        self.pciconfig.pio = bus.default
        bus.responder_set = True
        bus.responder = self.pciconfig
