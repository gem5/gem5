from m5.config import *

class FUPool(SimObject):
    type = 'FUPool'
    FUList = VectorParam.FUDesc("list of FU's for this pool")
