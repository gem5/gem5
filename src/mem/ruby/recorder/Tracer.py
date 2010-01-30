from m5.params import *
from m5.SimObject import SimObject

class RubyTracer(SimObject):
    type = 'RubyTracer'
    cxx_class = 'Tracer'
    warmup_length = Param.Int(100000, "")
