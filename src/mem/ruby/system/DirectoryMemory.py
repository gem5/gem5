from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject

class RubyDirectoryMemory(SimObject):
    type = 'RubyDirectoryMemory'
    cxx_class = 'DirectoryMemory'
    version = Param.Int(0, "")
    size_mb = Param.Int(1024, "")
    controller = Param.RubyController(Parent.any, "")
