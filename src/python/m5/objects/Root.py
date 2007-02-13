from m5.SimObject import SimObject
from m5.params import *
from Serialize import Serialize
from Serialize import Statreset
from Statistics import Statistics

class Root(SimObject):
    type = 'Root'
    clock = Param.RootClock('1THz', "tick frequency")
    max_tick = Param.Tick('0', "maximum simulation ticks (0 = infinite)")
    progress_interval = Param.Tick('0',
        "print a progress message every n ticks (0 = never)")
    output_file = Param.String('cout', "file to dump simulator output to")
    checkpoint = Param.String('', "checkpoint file to load")
#    stats = Param.Statistics(Statistics(), "statistics object")
#    serialize = Param.Serialize(Serialize(), "checkpoint generation options")
    stats = Statistics()
    serialize = Serialize()
