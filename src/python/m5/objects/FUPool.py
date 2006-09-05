from m5.SimObject import SimObject
from m5.params import *

class FUPool(SimObject):
    type = 'FUPool'
    FUList = VectorParam.FUDesc("list of FU's for this pool")
