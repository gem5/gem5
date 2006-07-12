from m5.config import *
from Pci import PciDevice, PciConfigData

class IdeID(Enum): vals = ['master', 'slave']

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

class IdeDisk(SimObject):
    type = 'IdeDisk'
    delay = Param.Latency('1us', "Fixed disk delay in microseconds")
    driveID = Param.IdeID('master', "Drive ID")
    image = Param.DiskImage("Disk image")

class IdeController(PciDevice):
    type = 'IdeController'
    disks = VectorParam.IdeDisk("IDE disks attached to this controller")

    configdata =IdeControllerPciData()
