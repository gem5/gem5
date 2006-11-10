from m5.SimObject import SimObject
from m5.params import *
class SparcTLB(SimObject):
    type = 'SparcTLB'
    abstract = True
    size = Param.Int("TLB size")

class SparcDTB(SparcTLB):
    type = 'SparcDTB'
    size = 64

class SparcITB(SparcTLB):
    type = 'SparcITB'
    size = 48
