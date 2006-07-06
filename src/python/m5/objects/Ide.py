from m5.config import *
from Pci import PciDevice

class IdeID(Enum): vals = ['master', 'slave']

class IdeDisk(SimObject):
    type = 'IdeDisk'
    delay = Param.Latency('1us', "Fixed disk delay in microseconds")
    driveID = Param.IdeID('master', "Drive ID")
    image = Param.DiskImage("Disk image")

class IdeController(PciDevice):
    type = 'IdeController'
    disks = VectorParam.IdeDisk("IDE disks attached to this controller")
