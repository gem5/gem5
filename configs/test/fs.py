from m5 import *
import os
from SysPaths import *

# Base for tests is directory containing this file.
test_base = os.path.dirname(__file__)

linux_image = env.get('LINUX_IMAGE', disk('linux-latest.img'))

class IdeControllerPciData(PciConfigData):
    VendorID = 0x8086
    DeviceID = 0x7111
    Command = 0x0
    Status = 0x280
    Revision = 0x0
    ClassCode = 0x01
    SubClassCode = 0x01
    ProgIF = 0x85
    BAR0 = 0x00000001
    BAR1 = 0x00000001
    BAR2 = 0x00000001
    BAR3 = 0x00000001
    BAR4 = 0x00000001
    BAR5 = 0x00000001
    InterruptLine = 0x1f
    InterruptPin = 0x01
    BAR0Size = '8B'
    BAR1Size = '4B'
    BAR2Size = '8B'
    BAR3Size = '4B'
    BAR4Size = '16B'

class SinicPciData(PciConfigData):
    VendorID = 0x1291
    DeviceID = 0x1293
    Status = 0x0290
    SubClassCode = 0x00
    ClassCode = 0x02
    ProgIF = 0x00
    BAR0 = 0x00000000
    BAR1 = 0x00000000
    BAR2 = 0x00000000
    BAR3 = 0x00000000
    BAR4 = 0x00000000
    BAR5 = 0x00000000
    MaximumLatency = 0x34
    MinimumGrant = 0xb0
    InterruptLine = 0x1e
    InterruptPin = 0x01
    BAR0Size = '64kB'

class NSGigEPciData(PciConfigData):
    VendorID = 0x100B
    DeviceID = 0x0022
    Status = 0x0290
    SubClassCode = 0x00
    ClassCode = 0x02
    ProgIF = 0x00
    BAR0 = 0x00000001
    BAR1 = 0x00000000
    BAR2 = 0x00000000
    BAR3 = 0x00000000
    BAR4 = 0x00000000
    BAR5 = 0x00000000
    MaximumLatency = 0x34
    MinimumGrant = 0xb0
    InterruptLine = 0x1e
    InterruptPin = 0x01
    BAR0Size = '256B'
    BAR1Size = '4kB'

class LinuxRootDisk(IdeDisk):
    raw_image = RawDiskImage(image_file=linux_image, read_only=True)
    image = CowDiskImage(child=Parent.raw_image, read_only=False)

class LinuxSwapDisk(IdeDisk):
    raw_image = RawDiskImage(image_file = disk('linux-bigswap2.img'),
                                  read_only=True)
    image = CowDiskImage(child = Parent.raw_image, read_only=False)

class SpecwebFilesetDisk(IdeDisk):
    raw_image = RawDiskImage(image_file = disk('specweb-fileset.img'),
                                  read_only=True)
    image = CowDiskImage(child = Parent.raw_image, read_only=False)

class BaseTsunami(Tsunami):
    cchip = TsunamiCChip(pio_addr=0x801a0000000)
    pchip = TsunamiPChip(pio_addr=0x80180000000)
    pciconfig = PciConfigAll(pio_addr=0x801fe000000)
    fake_sm_chip = IsaFake(pio_addr=0x801fc000370)

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
    io = TsunamiIO(pio_addr=0x801fc000000)
    uart = Uart8250(pio_addr=0x801fc0003f8)
    ethernet = NSGigE(configdata=NSGigEPciData(),
                      pci_bus=0, pci_dev=1, pci_func=0)
    etherint = NSGigEInt(device=Parent.ethernet)
#    ethernet = Sinic(configdata=SinicPciData(),
#                      pci_bus=0, pci_dev=1, pci_func=0)
#    etherint = SinicInt(device=Parent.ethernet)
    console = AlphaConsole(pio_addr=0x80200000000, disk=Parent.simple_disk)
#    bridge = PciFake(configdata=BridgePciData(), pci_bus=0, pci_dev=2, pci_func=0)

#class FreeBSDTsunami(BaseTsunami):
#    disk0 = FreeBSDRootDisk(delay='0us', driveID='master')
#    ide = IdeController(disks=[Parent.disk0],
#                        configdata=IdeControllerPciData(),
#                        pci_func=0, pci_dev=0, pci_bus=0)

class LinuxTsunami(BaseTsunami):
    disk0 = LinuxRootDisk(driveID='master')
    disk1 = SpecwebFilesetDisk(driveID='slave')
    disk2 = LinuxSwapDisk(driveID='master')
    ide = IdeController(disks=[Parent.disk0, Parent.disk1, Parent.disk2],
                        configdata=IdeControllerPciData(),
                        pci_func=0, pci_dev=0, pci_bus=0)

class LinuxAlphaSystem(LinuxAlphaSystem):
    magicbus = Bus(bus_id=0)
    magicbus2 = Bus(bus_id=1)
    bridge = Bridge()
    physmem = PhysicalMemory(range = AddrRange('128MB'))
    c0a = Connector(side_a=Parent.magicbus, side_b=Parent.bridge, side_b_name="side_a")
    c0b = Connector(side_a=Parent.magicbus2, side_b=Parent.bridge, side_b_name="side_b")
    c1 = Connector(side_a=Parent.physmem, side_b=Parent.magicbus2)
    tsunami = LinuxTsunami()
    c2 = Connector(side_a=Parent.tsunami.cchip, side_a_name='pio', side_b=Parent.magicbus)
    c3 = Connector(side_a=Parent.tsunami.pchip, side_a_name='pio', side_b=Parent.magicbus)
    c4 = Connector(side_a=Parent.tsunami.pciconfig, side_a_name='pio', side_b=Parent.magicbus)
    c5 = Connector(side_a=Parent.tsunami.fake_sm_chip, side_a_name='pio', side_b=Parent.magicbus)
    c6 = Connector(side_a=Parent.tsunami.ethernet, side_a_name='pio', side_b=Parent.magicbus)
    c6a = Connector(side_a=Parent.tsunami.ethernet, side_a_name='dma', side_b=Parent.magicbus)
    c7 = Connector(side_a=Parent.tsunami.fake_uart1, side_a_name='pio', side_b=Parent.magicbus)
    c8 = Connector(side_a=Parent.tsunami.fake_uart2, side_a_name='pio', side_b=Parent.magicbus)
    c9 = Connector(side_a=Parent.tsunami.fake_uart3, side_a_name='pio', side_b=Parent.magicbus)
    c10 = Connector(side_a=Parent.tsunami.fake_uart4, side_a_name='pio', side_b=Parent.magicbus)
    c11 = Connector(side_a=Parent.tsunami.ide, side_a_name='pio', side_b=Parent.magicbus)
    c13 = Connector(side_a=Parent.tsunami.ide, side_a_name='dma', side_b=Parent.magicbus)
    c12 = Connector(side_a=Parent.tsunami.fake_ppc, side_a_name='pio', side_b=Parent.magicbus)
    c14 = Connector(side_a=Parent.tsunami.fake_OROM, side_a_name='pio', side_b=Parent.magicbus)
    c16 = Connector(side_a=Parent.tsunami.fake_pnp_addr, side_a_name='pio', side_b=Parent.magicbus)
    c17 = Connector(side_a=Parent.tsunami.fake_pnp_write, side_a_name='pio', side_b=Parent.magicbus)
    c18 = Connector(side_a=Parent.tsunami.fake_pnp_read0, side_a_name='pio', side_b=Parent.magicbus)
    c19 = Connector(side_a=Parent.tsunami.fake_pnp_read1, side_a_name='pio', side_b=Parent.magicbus)
    c20 = Connector(side_a=Parent.tsunami.fake_pnp_read2, side_a_name='pio', side_b=Parent.magicbus)
    c21 = Connector(side_a=Parent.tsunami.fake_pnp_read3, side_a_name='pio', side_b=Parent.magicbus)
    c22 = Connector(side_a=Parent.tsunami.fake_pnp_read4, side_a_name='pio', side_b=Parent.magicbus)
    c23 = Connector(side_a=Parent.tsunami.fake_pnp_read5, side_a_name='pio', side_b=Parent.magicbus)
    c24 = Connector(side_a=Parent.tsunami.fake_pnp_read6, side_a_name='pio', side_b=Parent.magicbus)
    c25 = Connector(side_a=Parent.tsunami.fake_pnp_read7, side_a_name='pio', side_b=Parent.magicbus)
    c27 = Connector(side_a=Parent.tsunami.fake_ata0, side_a_name='pio', side_b=Parent.magicbus)
    c28 = Connector(side_a=Parent.tsunami.fake_ata1, side_a_name='pio', side_b=Parent.magicbus)
    c30 = Connector(side_a=Parent.tsunami.fb, side_a_name='pio', side_b=Parent.magicbus)
    c31 = Connector(side_a=Parent.tsunami.io, side_a_name='pio', side_b=Parent.magicbus)
    c32 = Connector(side_a=Parent.tsunami.uart, side_a_name='pio', side_b=Parent.magicbus)
    c33 = Connector(side_a=Parent.tsunami.console, side_a_name='pio', side_b=Parent.magicbus)
    raw_image = RawDiskImage(image_file=disk('linux-latest.img'),
                             read_only=True)
    simple_disk = SimpleDisk(disk=Parent.raw_image)
    intrctrl = IntrControl()
    cpu = AtomicSimpleCPU(mem=Parent.magicbus2)
    sim_console = SimConsole(listener=ConsoleListener(port=3456))
    kernel = binary('vmlinux')
    pal = binary('ts_osfpal')
    console = binary('console')
    boot_osflags = 'root=/dev/hda1 console=ttyS0'
#    readfile = os.path.join(test_base, 'halt.sh')


BaseCPU.itb = AlphaITB()
BaseCPU.dtb = AlphaDTB()
BaseCPU.system = Parent.any

class TsunamiRoot(System):
    pass


def DualRoot(ClientSystem, ServerSystem):
    self = Root()
    self.client = ClientSystem()
    self.server = ServerSystem()

    self.etherdump = EtherDump(file='ethertrace')
    self.etherlink = EtherLink(int1 = Parent.client.tsunami.etherint[0],
                               int2 = Parent.server.tsunami.etherint[0],
                               dump = Parent.etherdump)
    self.clock = '5GHz'
    return self

root = DualRoot(ClientSystem = LinuxAlphaSystem(readfile=script('netperf-stream-nt-client.rcS')),
                ServerSystem = LinuxAlphaSystem(readfile=script('netperf-server.rcS')))

