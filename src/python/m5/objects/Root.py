from m5.SimObject import SimObject
from m5.params import *

class Root(SimObject):
    type = 'Root'
    dummy = Param.Int(0, "We don't support objects without params")
