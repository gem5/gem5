import optparse, os, sys

import m5
from m5.objects import *
from SysPaths import *

parser = optparse.OptionParser()

parser.add_option("-t", "--timing", action="store_true")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# Base for tests is directory containing this file.
test_base = os.path.dirname(__file__)

script.dir =  '/z/saidi/work/m5.newmem/configs/boot'

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
    pciconfig = PciConfigAll()
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
    console = AlphaConsole(pio_addr=0x80200000000, disk=Parent.simple_disk)

class LinuxTsunami(BaseTsunami):
    disk0 = LinuxRootDisk(driveID='master')
    disk1 = SpecwebFilesetDisk(driveID='slave')
    disk2 = LinuxSwapDisk(driveID='master')
    ide = IdeController(disks=[Parent.disk0, Parent.disk1, Parent.disk2],
                        configdata=IdeControllerPciData(),
                        pci_func=0, pci_dev=0, pci_bus=0)

class MyLinuxAlphaSystem(LinuxAlphaSystem):
    magicbus = Bus(bus_id=0)
    magicbus2 = Bus(bus_id=1)
    bridge = Bridge()
    physmem = PhysicalMemory(range = AddrRange('128MB'))
    bridge.side_a = magicbus.port
    bridge.side_b = magicbus2.port
    physmem.port = magicbus2.port
    tsunami = LinuxTsunami()
    tsunami.cchip.pio = magicbus.port
    tsunami.pchip.pio = magicbus.port
    tsunami.pciconfig.pio = magicbus.default
    tsunami.fake_sm_chip.pio = magicbus.port
    tsunami.ethernet.pio = magicbus.port
    tsunami.ethernet.dma = magicbus.port
    tsunami.ethernet.config = magicbus.port
    tsunami.fake_uart1.pio = magicbus.port
    tsunami.fake_uart2.pio = magicbus.port
    tsunami.fake_uart3.pio = magicbus.port
    tsunami.fake_uart4.pio = magicbus.port
    tsunami.ide.pio = magicbus.port
    tsunami.ide.dma = magicbus.port
    tsunami.ide.config = magicbus.port
    tsunami.fake_ppc.pio = magicbus.port
    tsunami.fake_OROM.pio = magicbus.port
    tsunami.fake_pnp_addr.pio = magicbus.port
    tsunami.fake_pnp_write.pio = magicbus.port
    tsunami.fake_pnp_read0.pio = magicbus.port
    tsunami.fake_pnp_read1.pio = magicbus.port
    tsunami.fake_pnp_read2.pio = magicbus.port
    tsunami.fake_pnp_read3.pio = magicbus.port
    tsunami.fake_pnp_read4.pio = magicbus.port
    tsunami.fake_pnp_read5.pio = magicbus.port
    tsunami.fake_pnp_read6.pio = magicbus.port
    tsunami.fake_pnp_read7.pio = magicbus.port
    tsunami.fake_ata0.pio = magicbus.port
    tsunami.fake_ata1.pio = magicbus.port
    tsunami.fb.pio = magicbus.port
    tsunami.io.pio = magicbus.port
    tsunami.uart.pio = magicbus.port
    tsunami.console.pio = magicbus.port
    raw_image = RawDiskImage(image_file=disk('linux-latest.img'),
                             read_only=True)
    simple_disk = SimpleDisk(disk=Parent.raw_image)
    intrctrl = IntrControl()
    if options.timing:
        cpu = TimingSimpleCPU()
        mem_mode = 'timing'
    else:
        cpu = AtomicSimpleCPU()
    cpu.mem = magicbus2
    cpu.icache_port = magicbus2.port
    cpu.dcache_port = magicbus2.port
    cpu.itb = AlphaITB()
    cpu.dtb = AlphaDTB()
    cpu.clock = '2GHz'
    sim_console = SimConsole(listener=ConsoleListener(port=3456))
    kernel = binary('vmlinux')
    pal = binary('ts_osfpal')
    console = binary('console')
    boot_osflags = 'root=/dev/hda1 console=ttyS0'
#    readfile = os.path.join(test_base, 'halt.sh')



class TsunamiRoot(System):
    pass


def DualRoot(clientSystem, serverSystem):
    self = Root()
    self.client = clientSystem
    self.server = serverSystem

    self.etherdump = EtherDump(file='ethertrace')
    self.etherlink = EtherLink(int1 = Parent.client.tsunami.etherint[0],
                               int2 = Parent.server.tsunami.etherint[0],
                               dump = Parent.etherdump)
    self.clock = '1THz'
    return self

root = DualRoot(
    MyLinuxAlphaSystem(readfile=script('netperf-stream-nt-client.rcS')),
    MyLinuxAlphaSystem(readfile=script('netperf-server.rcS')))

m5.instantiate(root)

#exit_event = m5.simulate(2600000000000)
#if exit_event.getCause() != "user interrupt received":
#    m5.checkpoint(root, 'cpt')
#    exit_event = m5.simulate(300000000000)
#    if exit_event.getCause() != "user interrupt received":
#        m5.checkpoint(root, 'cptA')


exit_event = m5.simulate()

print 'Exiting @ cycle', m5.curTick(), 'because', exit_event.getCause()
