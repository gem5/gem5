from m5.config import *
from MemObject import MemObject

class Bus(MemObject):
    type = 'Bus'
    port = VectorPort("vector port for connecting devices")
    bus_id = Param.Int(0, "blah")
