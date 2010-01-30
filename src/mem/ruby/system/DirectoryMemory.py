from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject

class RubyDirectoryMemory(SimObject):
    type = 'RubyDirectoryMemory'
    cxx_class = 'DirectoryMemory'
    version = Param.Int(0, "")
    size = Param.MemorySize("1GB", "capacity in bytes")
