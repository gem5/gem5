from m5.SimObject import SimObject
from m5.params import *
class MipsTLB(SimObject):
    type = 'MipsTLB'
    abstract = True
    #size = Param.Int("TLB size")

class MipsDTB(MipsTLB):
    type = 'MipsDTB'
    cxx_namespace = 'MipsISA'
    cxx_class = 'DTB'

    #size = 64

class MipsITB(MipsTLB):
    type = 'MipsITB'
    cxx_namespace = 'MipsISA'
    cxx_class = 'ITB'

    #size = 64
