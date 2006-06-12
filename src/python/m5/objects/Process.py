from m5.config import *
class Process(SimObject):
    type = 'Process'
    abstract = True
    output = Param.String('cout', 'filename for stdout/stderr')
    system = Param.System(Parent.any, "system process will run on")

class LiveProcess(Process):
    type = 'LiveProcess'
    executable = Param.String('', "executable (overrides cmd[0] if set)")
    cmd = VectorParam.String("command line (executable plus arguments)")
    env = VectorParam.String('', "environment settings")
    input = Param.String('cin', "filename for stdin")

class AlphaLiveProcess(LiveProcess):
    type = 'AlphaLiveProcess'

class SparcLiveProcess(LiveProcess):
    type = 'SparcLiveProcess'

class MipsLiveProcess(LiveProcess):
    type = 'MipsLiveProcess'

class EioProcess(Process):
    type = 'EioProcess'
    chkpt = Param.String('', "EIO checkpoint file name (optional)")
    file = Param.String("EIO trace file name")
