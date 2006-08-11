from m5 import *
from HierParams import HierParams
from Serialize import Serialize
from Serialize import Statreset
from Statistics import Statistics
from Trace import Trace
from ExeTrace import ExecutionTrace

class Root(SimObject):
    type = 'Root'
    clock = Param.RootClock('200MHz', "tick frequency")
    max_tick = Param.Tick('0', "maximum simulation ticks (0 = infinite)")
    progress_interval = Param.Tick('0',
        "print a progress message every n ticks (0 = never)")
    output_file = Param.String('cout', "file to dump simulator output to")
    checkpoint = Param.String('', "checkpoint file to load")
#    hier = Param.HierParams(HierParams(do_data = False, do_events = True),
#                            "shared memory hierarchy parameters")
#    stats = Param.Statistics(Statistics(), "statistics object")
#    trace = Param.Trace(Trace(), "trace object")
#    serialize = Param.Serialize(Serialize(), "checkpoint generation options")
    hier = HierParams(do_data = False, do_events = True)
    stats = Statistics()
    trace = Trace()
    exetrace = ExecutionTrace()
    serialize = Serialize()
    statsreset = Statreset()
