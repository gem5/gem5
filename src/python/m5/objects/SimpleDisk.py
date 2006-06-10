from m5.config import *
class SimpleDisk(SimObject):
    type = 'SimpleDisk'
    disk = Param.DiskImage("Disk Image")
    system = Param.System(Parent.any, "Sysetm Pointer")
