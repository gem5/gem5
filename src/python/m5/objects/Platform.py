from m5.config import *
class Platform(SimObject):
    type = 'Platform'
    abstract = True
    intrctrl = Param.IntrControl(Parent.any, "interrupt controller")
