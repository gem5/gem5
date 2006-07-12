import optparse, os, sys

import m5
from m5.objects import *
from SysPaths import *
from FullO3Config import *

parser = optparse.OptionParser()

parser.add_option("-d", "--detailed", action="store_true")
parser.add_option("-t", "--timing", action="store_true")
parser.add_option("-m", "--maxtick", type="int")
parser.add_option("--dual", help="Run full system using dual systems",
                  action="store_true")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# Base for tests is directory containing this file.
test_base = os.path.dirname(__file__)

linux_image = env.get('LINUX_IMAGE', disk('linux-latest.img'))

class CowIdeDisk(IdeDisk):
    image = CowDiskImage(child=RawDiskImage(read_only=True),
                         read_only=False)

    def childImage(self, ci):
        self.image.child.image_file = ci

class BaseTsunami(Tsunami):
    ethernet = NSGigE(configdata=NSGigEPciData(),
                      pci_bus=0, pci_dev=1, pci_func=0)
    etherint = NSGigEInt(device=Parent.ethernet)
    ide = IdeController(disks=[Parent.disk0, Parent.disk2],
                        pci_func=0, pci_dev=0, pci_bus=0)

class MyLinuxAlphaSystem(LinuxAlphaSystem):
    iobus = Bus(bus_id=0)
    membus = Bus(bus_id=1)
    bridge = Bridge()
    physmem = PhysicalMemory(range = AddrRange('128MB'))
    bridge.side_a = iobus.port
    bridge.side_b = membus.port
    physmem.port = membus.port
    disk0 = CowIdeDisk(driveID='master')
    disk2 = CowIdeDisk(driveID='master')
    disk0.childImage(linux_image)
    disk2.childImage(disk('linux-bigswap2.img'))
    tsunami = BaseTsunami()
    tsunami.attachIO(iobus)
    tsunami.ide.pio = iobus.port
    tsunami.ide.dma = iobus.port
    tsunami.ide.config = iobus.port
    tsunami.ethernet.pio = iobus.port
    tsunami.ethernet.dma = iobus.port
    tsunami.ethernet.config = iobus.port
    simple_disk = SimpleDisk(disk=RawDiskImage(image_file = linux_image,
                                               read_only = True))
    intrctrl = IntrControl()
    if options.detailed:
        cpu = DetailedO3CPU()
    elif options.timing:
        cpu = TimingSimpleCPU()
    else:
        cpu = AtomicSimpleCPU()
    cpu.mem = membus
    cpu.icache_port = membus.port
    cpu.dcache_port = membus.port
    cpu.itb = AlphaITB()
    cpu.dtb = AlphaDTB()
    sim_console = SimConsole(listener=ConsoleListener(port=3456))
    kernel = binary('vmlinux')
    pal = binary('ts_osfpal')
    console = binary('console')
    boot_osflags = 'root=/dev/hda1 console=ttyS0'

class TsunamiRoot(Root):
    pass

def DualRoot(clientSystem, serverSystem):
    self = Root()
    self.client = clientSystem
    self.server = serverSystem

    self.etherdump = EtherDump(file='ethertrace')
    self.etherlink = EtherLink(int1 = Parent.client.tsunami.etherint[0],
                               int2 = Parent.server.tsunami.etherint[0],
                               dump = Parent.etherdump)
    self.clock = '5GHz'
    return self

if options.dual:
    root = DualRoot(
        MyLinuxAlphaSystem(readfile=script('netperf-stream-nt-client.rcS')),
        MyLinuxAlphaSystem(readfile=script('netperf-server.rcS')))
else:
    root = TsunamiRoot(clock = '2GHz', system = MyLinuxAlphaSystem())

m5.instantiate(root)

if options.maxtick:
    exit_event = m5.simulate(options.maxtick)
else:
    exit_event = m5.simulate()

print 'Exiting @ cycle', m5.curTick(), 'because', exit_event.getCause()
