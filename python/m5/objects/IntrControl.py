from m5 import *
class IntrControl(SimObject):
    type = 'IntrControl'
    cpu = Param.BaseCPU(Parent.any, "the cpu")
