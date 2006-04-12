import os
from SysPaths import *

# Base for tests is directory containing this file.
test_base = os.path.dirname(__file__)

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
#    ethernet = NSGigE(configdata=NSGigEPciData(),
#                      pci_bus=0, pci_dev=1, pci_func=0)
#    etherint = NSGigEInt(device=Parent.ethernet)
    console = AlphaConsole(pio_addr=0x80200000000, disk=Parent.simple_disk)
#    bridge = PciFake(configdata=BridgePciData(), pci_bus=0, pci_dev=2, pci_func=0)

#class FreeBSDTsunami(BaseTsunami):
#    disk0 = FreeBSDRootDisk(delay='0us', driveID='master')
#    ide = IdeController(disks=[Parent.disk0],
#                        configdata=IdeControllerPciData(),
#                        pci_func=0, pci_dev=0, pci_bus=0)

#class LinuxTsunami(BaseTsunami):
#    disk0 = LinuxRootDisk(delay='0us', driveID='master')
#    ide = IdeController(disks=[Parent.disk0],
#                        configdata=IdeControllerPciData(),
#                        pci_func=0, pci_dev=0, pci_bus=0)

class LinuxAlphaSystem(LinuxAlphaSystem):
    magicbus = Bus()
    physmem = PhysicalMemory(range = AddrRange('128MB'))
    c1 = Connector(side_a=Parent.physmem, side_b=Parent.magicbus)
    tsunami = BaseTsunami()
    c2 = Connector(side_a=Parent.tsunami.cchip, side_a_name='pio', side_b=Parent.magicbus)
    c3 = Connector(side_a=Parent.tsunami.pchip, side_a_name='pio', side_b=Parent.magicbus)
    c4 = Connector(side_a=Parent.tsunami.pciconfig, side_a_name='pio', side_b=Parent.magicbus)
    c5 = Connector(side_a=Parent.tsunami.fake_sm_chip, side_a_name='pio', side_b=Parent.magicbus)
    c7 = Connector(side_a=Parent.tsunami.fake_uart1, side_a_name='pio', side_b=Parent.magicbus)
    c8 = Connector(side_a=Parent.tsunami.fake_uart2, side_a_name='pio', side_b=Parent.magicbus)
    c9 = Connector(side_a=Parent.tsunami.fake_uart3, side_a_name='pio', side_b=Parent.magicbus)
    c10 = Connector(side_a=Parent.tsunami.fake_uart4, side_a_name='pio', side_b=Parent.magicbus)
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
    raw_image = RawDiskImage(image_file=disk('linux.img'),
                             read_only=True)
    simple_disk = SimpleDisk(disk=Parent.raw_image)
    intrctrl = IntrControl()
    cpu = SimpleCPU(mem=Parent.magicbus)
    sim_console = SimConsole(listener=ConsoleListener(port=3456))
    kernel = binary('vmlinux')
    pal = binary('ts_osfpal')
    console = binary('console')
    boot_osflags = 'root=/dev/hda1 console=ttyS0'
    readfile = os.path.join(test_base, 'halt.sh')


BaseCPU.itb = AlphaITB()
BaseCPU.dtb = AlphaDTB()
BaseCPU.system = Parent.any

class TsunamiRoot(Root):
    pass


root = TsunamiRoot(clock = '2GHz', system = LinuxAlphaSystem())
