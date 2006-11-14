from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice
from Platform import Platform
from AlphaConsole import AlphaConsole
from Uart import Uart8250
from Pci import PciConfigAll
from BadDevice import BadDevice

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

    fake_uart1 = IsaFake(pio_addr=0x801fc0002f8)
    fake_uart2 = IsaFake(pio_addr=0x801fc0003e8)
    fake_uart3 = IsaFake(pio_addr=0x801fc0002e8)
    fake_uart4 = IsaFake(pio_addr=0x801fc0003f0)

    fake_ppc = IsaFake(pio_addr=0x801fc0003bc)

    fake_OROM = IsaFake(pio_addr=0x800000a0000, pio_size=0x60000)

    fake_pnp_addr = IsaFake(pio_addr=0x801fc000279)
    fake_pnp_write = IsaFake(pio_addr=0x801fc000a79)
    fake_pnp_read0 = IsaFake(pio_addr=0x801fc000203)
    fake_pnp_read1 = IsaFake(pio_addr=0x801fc000243)
    fake_pnp_read2 = IsaFake(pio_addr=0x801fc000283)
    fake_pnp_read3 = IsaFake(pio_addr=0x801fc0002c3)
    fake_pnp_read4 = IsaFake(pio_addr=0x801fc000303)
    fake_pnp_read5 = IsaFake(pio_addr=0x801fc000343)
    fake_pnp_read6 = IsaFake(pio_addr=0x801fc000383)
    fake_pnp_read7 = IsaFake(pio_addr=0x801fc0003c3)

    fake_ata0 = IsaFake(pio_addr=0x801fc0001f0)
    fake_ata1 = IsaFake(pio_addr=0x801fc000170)

    fb = BadDevice(pio_addr=0x801fc0003d0, devicename='FrameBuffer')
    uart = Uart8250(pio_addr=0x801fc0003f8)

    # Attach I/O devices to specified bus object.  Can't do this
    # earlier, since the bus object itself is typically defined at the
    # System level.
    def attachIO(self, bus):
        self.fake_iob.pio = bus.port
        self.fake_uart1.pio = bus.port
        self.fake_uart2.pio = bus.port
        self.fake_uart3.pio = bus.port
        self.fake_uart4.pio = bus.port
        self.fake_ppc.pio = bus.port
        self.fake_OROM.pio = bus.port
        self.fake_pnp_addr.pio = bus.port
        self.fake_pnp_write.pio = bus.port
        self.fake_pnp_read0.pio = bus.port
        self.fake_pnp_read1.pio = bus.port
        self.fake_pnp_read2.pio = bus.port
        self.fake_pnp_read3.pio = bus.port
        self.fake_pnp_read4.pio = bus.port
        self.fake_pnp_read5.pio = bus.port
        self.fake_pnp_read6.pio = bus.port
        self.fake_pnp_read7.pio = bus.port
        self.fake_ata0.pio = bus.port
        self.fake_ata1.pio = bus.port
        self.fb.pio = bus.port
        self.io.pio = bus.port
        self.uart.pio = bus.port
