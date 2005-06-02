from m5 import *
from BaseHier import BaseHier

class Bus(BaseHier):
    type = 'Bus'
    clock = Param.Clock("bus frequency")
    width = Param.Int("bus width in bytes")
