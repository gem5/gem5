from m5.config import *
from MemObject import MemObject

class Bus(MemObject):
    type = 'Bus'
    bus_id = Param.Int(0, "blah")
