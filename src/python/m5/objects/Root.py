from m5.SimObject import SimObject
from m5.params import *

class Root(SimObject):
    type = 'Root'
    clock = Param.RootClock('1THz', "tick frequency")
    max_tick = Param.Tick('0', "maximum simulation ticks (0 = infinite)")
    progress_interval = Param.Tick('0',
        "print a progress message every n ticks (0 = never)")
    output_file = Param.String('cout', "file to dump simulator output to")
    checkpoint = Param.String('', "checkpoint file to load")
