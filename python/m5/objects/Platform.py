from m5 import *
class Platform(SimObject):
    type = 'Platform'
    abstract = True
    intrctrl = Param.IntrControl(Parent.any, "interrupt controller")
