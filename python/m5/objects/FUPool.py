from m5 import *
from FullCPU import OpType
from FullCPU import OpDesc
from FullCPU import FUDesc

class FUPool(SimObject):
    type = 'FUPool'
    FUList = VectorParam.FUDesc("list of FU's for this pool")
