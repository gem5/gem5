from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice, IsaFake, BadAddr
from Uart import Uart8250
from Platform import Platform
from SimConsole import SimConsole, ConsoleListener

class T1000(Platform):
    type = 'T1000'
    system = Param.System(Parent.any, "system")

    fake_clk = IsaFake(pio_addr=0x9600000000, pio_size=0x100000000,
            warn_access="Accessing Clock Unit -- Unimplemented!")

    fake_membnks = IsaFake(pio_addr=0x9700000000, pio_size=16384,
            ret_data64=0x0000000000000000, update_data=False,
            warn_access="Accessing Memory Banks -- Unimplemented!")

    fake_iob = IsaFake(pio_addr=0x9800000000, pio_size=0x100000000,
            warn_access="Accessing IOB -- Unimplemented!")

    fake_jbi = IsaFake(pio_addr=0x8000000000, pio_size=0x100000000,
            warn_access="Accessing JBI -- Unimplemented!")

    fake_l2_1 = IsaFake(pio_addr=0xA900000000, pio_size=0x8,
            ret_data64=0x0000000000000001, update_data=True,
            warn_access="Accessing L2 Cache Banks -- Unimplemented!")

    fake_l2_2 = IsaFake(pio_addr=0xA900000040, pio_size=0x8,
            ret_data64=0x0000000000000001, update_data=True,
            warn_access="Accessing L2 Cache Banks -- Unimplemented!")

    fake_l2_3 = IsaFake(pio_addr=0xA900000080, pio_size=0x8,
            ret_data64=0x0000000000000001, update_data=True,
            warn_access="Accessing L2 Cache Banks -- Unimplemented!")

    fake_l2_4 = IsaFake(pio_addr=0xA9000000C0, pio_size=0x8,
            ret_data64=0x0000000000000001, update_data=True,
            warn_access="Accessing L2 Cache Banks -- Unimplemented!")

    fake_ssi = IsaFake(pio_addr=0xff00000000, pio_size=0x10000000,
            warn_access="Accessing SSI -- Unimplemented!")

    hvuart = Uart8250(pio_addr=0xfff0c2c000)
    puart0 = Uart8250(pio_addr=0x1f10000000)
    console = SimConsole(listener = ConsoleListener())

    # Attach I/O devices to specified bus object.  Can't do this
    # earlier, since the bus object itself is typically defined at the
    # System level.
    def attachIO(self, bus):
        self.fake_clk.pio = bus.port
        self.fake_membnks.pio = bus.port
        self.fake_iob.pio = bus.port
        self.fake_jbi.pio = bus.port
        self.fake_l2_1.pio = bus.port
        self.fake_l2_2.pio = bus.port
        self.fake_l2_3.pio = bus.port
        self.fake_l2_4.pio = bus.port
        self.fake_ssi.pio = bus.port
        self.puart0.pio = bus.port
        self.hvuart.pio = bus.port
