from m5.SimObject import SimObject
from m5.params import *
from FuncUnit import *

class IntALU(FUDesc):
    opList = [ OpDesc(opClass='IntAlu') ]
    count = 6

class IntMultDiv(FUDesc):
    opList = [ OpDesc(opClass='IntMult', opLat=3),
               OpDesc(opClass='IntDiv', opLat=20, issueLat=19) ]
    count=2

class FP_ALU(FUDesc):
    opList = [ OpDesc(opClass='FloatAdd', opLat=2),
               OpDesc(opClass='FloatCmp', opLat=2),
               OpDesc(opClass='FloatCvt', opLat=2) ]
    count = 4

class FP_MultDiv(FUDesc):
    opList = [ OpDesc(opClass='FloatMult', opLat=4),
               OpDesc(opClass='FloatDiv', opLat=12, issueLat=12),
               OpDesc(opClass='FloatSqrt', opLat=24, issueLat=24) ]
    count = 2

class ReadPort(FUDesc):
    opList = [ OpDesc(opClass='MemRead') ]
    count = 0

class WritePort(FUDesc):
    opList = [ OpDesc(opClass='MemWrite') ]
    count = 0

class RdWrPort(FUDesc):
    opList = [ OpDesc(opClass='MemRead'), OpDesc(opClass='MemWrite') ]
    count = 4

class IprPort(FUDesc):
    opList = [ OpDesc(opClass='IprAccess', opLat = 3, issueLat = 3) ]
    count = 1

