from m5 import *
class AlphaTLB(SimObject):
    type = 'AlphaTLB'
    abstract = True
    size = Param.Int("TLB size")

class AlphaDTB(AlphaTLB):
    type = 'AlphaDTB'
    size = 64

class AlphaITB(AlphaTLB):
    type = 'AlphaITB'
    size = 48
