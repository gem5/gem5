from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice, IsaFake, BadAddr
from Uart import Uart8250
from Platform import Platform
from SimConsole import SimConsole, ConsoleListener

class T1000(Platform):
    type = 'T1000'
    system = Param.System(Parent.any, "system")

    fake_iob = IsaFake(pio_addr=0x8000000000, pio_size=0x7F00000000)

    uart = Uart8250(pio_addr=0xfff0c2c000)
    console = SimConsole(listener = ConsoleListener())

    # Attach I/O devices to specified bus object.  Can't do this
    # earlier, since the bus object itself is typically defined at the
    # System level.
    def attachIO(self, bus):
        self.fake_iob.pio = bus.port
        self.uart.pio = bus.port
