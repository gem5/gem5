from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice, IsaFake, BadAddr
from Platform import Platform
from AlphaConsole import AlphaConsole
from Uart import Uart8250
from Pci import PciConfigAll
from BadDevice import BadDevice

class TsunamiCChip(BasicPioDevice):
    type = 'TsunamiCChip'
    tsunami = Param.Tsunami(Parent.any, "Tsunami")

class TsunamiIO(BasicPioDevice):
    type = 'TsunamiIO'
    time = Param.Time('01/01/2006',
        "System time to use ('Now' for actual time)")
    year_is_bcd = Param.Bool(False,
        "The RTC should interpret the year as a BCD value")
    tsunami = Param.Tsunami(Parent.any, "Tsunami")
    frequency = Param.Frequency('1024Hz', "frequency of interrupts")

class TsunamiPChip(BasicPioDevice):
    type = 'TsunamiPChip'
    tsunami = Param.Tsunami(Parent.any, "Tsunami")

class Tsunami(Platform):
    type = 'Tsunami'
    system = Param.System(Parent.any, "system")

    cchip = TsunamiCChip(pio_addr=0x801a0000000)
    pchip = TsunamiPChip(pio_addr=0x80180000000)
    pciconfig = PciConfigAll()
    fake_sm_chip = IsaFake(pio_addr=0x801fc000370)

    fake_uart1 = IsaFake(pio_addr=0x801fc0002f8)
    fake_uart2 = IsaFake(pio_addr=0x801fc0003e8)
    fake_uart3 = IsaFake(pio_addr=0x801fc0002e8)
    fake_uart4 = IsaFake(pio_addr=0x801fc0003f0)

    fake_ppc = IsaFake(pio_addr=0x801fc0003bb)

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
    io = TsunamiIO(pio_addr=0x801fc000000)
    uart = Uart8250(pio_addr=0x801fc0003f8)
    console = AlphaConsole(pio_addr=0x80200000000, disk=Parent.simple_disk)

    # Attach I/O devices to specified bus object.  Can't do this
    # earlier, since the bus object itself is typically defined at the
    # System level.
    def attachIO(self, bus):
        self.cchip.pio = bus.port
        self.pchip.pio = bus.port
        self.pciconfig.pio = bus.default
        bus.responder_set = True
        bus.responder = self.pciconfig
        self.fake_sm_chip.pio = bus.port
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
        self.console.pio = bus.port
