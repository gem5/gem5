from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
class Platform(SimObject):
    type = 'Platform'
    abstract = True
    intrctrl = Param.IntrControl(Parent.any, "interrupt controller")
