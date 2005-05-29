from m5 import *
from BaseHier import BaseHier

class Bus(BaseHier):
    type = 'Bus'
    clock_ratio = Param.Frequency("ratio of CPU to bus frequency")
    width = Param.Int("bus width in bytes")
