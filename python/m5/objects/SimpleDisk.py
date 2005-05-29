from m5 import *
class SimpleDisk(SimObject):
    type = 'SimpleDisk'
    disk = Param.DiskImage("Disk Image")
    physmem = Param.PhysicalMemory(Parent.any, "Physical Memory")
