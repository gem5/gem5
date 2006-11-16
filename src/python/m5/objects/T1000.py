from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice
from Uart import Uart8250
from Platform import Platform
from SimConsole import SimConsole, ConsoleListener

class IsaFake(BasicPioDevice):
    type = 'IsaFake'
    pio_size = Param.Addr(0x8, "Size of address range")
    ret_data = Param.UInt8(0xFF, "Default data to return")
    ret_bad_addr = Param.Bool(False, "Return pkt status bad address on access")

class BadAddr(IsaFake):
    ret_bad_addr = Param.Bool(True, "Return pkt status bad address on access")

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
