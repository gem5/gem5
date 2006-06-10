from m5.config import *
from Serialize import Serialize
from Statistics import Statistics
from Trace import Trace
from ExeTrace import ExecutionTrace
from Debug import Debug

class Root(SimObject):
    type = 'Root'
    clock = Param.RootClock('200MHz', "tick frequency")
    max_tick = Param.Tick('0', "maximum simulation ticks (0 = infinite)")
    progress_interval = Param.Tick('0',
        "print a progress message every n ticks (0 = never)")
    output_file = Param.String('cout', "file to dump simulator output to")
    checkpoint = Param.String('', "checkpoint file to load")
#    stats = Param.Statistics(Statistics(), "statistics object")
#    trace = Param.Trace(Trace(), "trace object")
#    serialize = Param.Serialize(Serialize(), "checkpoint generation options")
    stats = Statistics()
    trace = Trace()
    exetrace = ExecutionTrace()
    serialize = Serialize()
    debug = Debug()
