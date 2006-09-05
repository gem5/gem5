from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
class SimpleDisk(SimObject):
    type = 'SimpleDisk'
    disk = Param.DiskImage("Disk Image")
    system = Param.System(Parent.any, "Sysetm Pointer")
