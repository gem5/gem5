from m5.SimObject import SimObject
from m5.params import *
from FuncUnit import *
from FuncUnitConfig import *

class FUPool(SimObject):
    type = 'FUPool'
    FUList = VectorParam.FUDesc("list of FU's for this pool")

class DefaultFUPool(FUPool):
    FUList = [ IntALU(), IntMultDiv(), FP_ALU(), FP_MultDiv(), ReadPort(),
               WritePort(), RdWrPort(), IprPort() ]
