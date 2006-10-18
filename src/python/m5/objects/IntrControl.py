from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
class IntrControl(SimObject):
    type = 'IntrControl'
    cpu = Param.BaseCPU(Parent.cpu[0], "the cpu")
