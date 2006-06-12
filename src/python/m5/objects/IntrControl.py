from m5.config import *
class IntrControl(SimObject):
    type = 'IntrControl'
    cpu = Param.BaseCPU(Parent.any, "the cpu")
