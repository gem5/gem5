from m5.params import *
from m5.SimObject import SimObject

class RubyController(SimObject):
    type = 'RubyController'
    cxx_class = 'AbstractController'
    abstract = True
    version = Param.Int("")
    transitions_per_cycle = \
        Param.Int(32, "no. of  SLICC state machine transitions per cycle")
    buffer_size = Param.Int(0, "max buffer size 0 means infinite")
    recycle_latency = Param.Int(10, "")
    number_of_TBEs = Param.Int(256, "")
