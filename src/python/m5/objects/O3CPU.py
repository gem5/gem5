from m5 import build_env
from m5.config import *
from BaseCPU import BaseCPU
from Checker import O3Checker

class DerivO3CPU(BaseCPU):
    type = 'DerivO3CPU'
    activity = Param.Unsigned(0, "Initial count")
    numThreads = Param.Unsigned(1, "number of HW thread contexts")

    if build_env['USE_CHECKER']:
        if not build_env['FULL_SYSTEM']:
            checker = Param.BaseCPU(O3Checker(workload=Parent.workload,
                                              exitOnError=True,
                                              warnOnlyOnLoadError=False),
                                    "checker")
        else:
            checker = Param.BaseCPU(O3Checker(exitOnError=True, warnOnlyOnLoadError=False), "checker")
            checker.itb = Parent.itb
            checker.dtb = Parent.dtb

    cachePorts = Param.Unsigned("Cache Ports")
    icache_port = Port("Instruction Port")
    dcache_port = Port("Data Port")
    _mem_ports = ['icache_port', 'dcache_port']

    decodeToFetchDelay = Param.Unsigned(1, "Decode to fetch delay")
    renameToFetchDelay = Param.Unsigned(1 ,"Rename to fetch delay")
    iewToFetchDelay = Param.Unsigned(1, "Issue/Execute/Writeback to fetch "
                                     "delay")
    commitToFetchDelay = Param.Unsigned(1, "Commit to fetch delay")
    fetchWidth = Param.Unsigned(8, "Fetch width")

    renameToDecodeDelay = Param.Unsigned(1, "Rename to decode delay")
    iewToDecodeDelay = Param.Unsigned(1, "Issue/Execute/Writeback to decode "
               "delay")
    commitToDecodeDelay = Param.Unsigned(1, "Commit to decode delay")
    fetchToDecodeDelay = Param.Unsigned(1, "Fetch to decode delay")
    decodeWidth = Param.Unsigned(8, "Decode width")

    iewToRenameDelay = Param.Unsigned(1, "Issue/Execute/Writeback to rename "
               "delay")
    commitToRenameDelay = Param.Unsigned(1, "Commit to rename delay")
    decodeToRenameDelay = Param.Unsigned(1, "Decode to rename delay")
    renameWidth = Param.Unsigned(8, "Rename width")

    commitToIEWDelay = Param.Unsigned(1, "Commit to "
               "Issue/Execute/Writeback delay")
    renameToIEWDelay = Param.Unsigned(2, "Rename to "
               "Issue/Execute/Writeback delay")
    issueToExecuteDelay = Param.Unsigned(1, "Issue to execute delay (internal "
              "to the IEW stage)")
    dispatchWidth = Param.Unsigned(8, "Dispatch width")
    issueWidth = Param.Unsigned(8, "Issue width")
    wbWidth = Param.Unsigned(8, "Writeback width")
    wbDepth = Param.Unsigned(1, "Writeback depth")
    fuPool = Param.FUPool("Functional Unit pool")

    iewToCommitDelay = Param.Unsigned(1, "Issue/Execute/Writeback to commit "
               "delay")
    renameToROBDelay = Param.Unsigned(1, "Rename to reorder buffer delay")
    commitWidth = Param.Unsigned(8, "Commit width")
    squashWidth = Param.Unsigned(8, "Squash width")
    trapLatency = Param.Tick(13, "Trap latency")
    fetchTrapLatency = Param.Tick(1, "Fetch trap latency")

    backComSize = Param.Unsigned(5, "Time buffer size for backwards communication")
    forwardComSize = Param.Unsigned(5, "Time buffer size for forward communication")

    predType = Param.String("tournament", "Branch predictor type ('local', 'tournament')")
    localPredictorSize = Param.Unsigned(2048, "Size of local predictor")
    localCtrBits = Param.Unsigned(2, "Bits per counter")
    localHistoryTableSize = Param.Unsigned(2048, "Size of local history table")
    localHistoryBits = Param.Unsigned(11, "Bits for the local history")
    globalPredictorSize = Param.Unsigned(8192, "Size of global predictor")
    globalCtrBits = Param.Unsigned(2, "Bits per counter")
    globalHistoryBits = Param.Unsigned(4096, "Bits of history")
    choicePredictorSize = Param.Unsigned(8192, "Size of choice predictor")
    choiceCtrBits = Param.Unsigned(2, "Bits of choice counters")

    BTBEntries = Param.Unsigned(4096, "Number of BTB entries")
    BTBTagSize = Param.Unsigned(16, "Size of the BTB tags, in bits")

    RASSize = Param.Unsigned(16, "RAS size")

    LQEntries = Param.Unsigned(32, "Number of load queue entries")
    SQEntries = Param.Unsigned(32, "Number of store queue entries")
    LFSTSize = Param.Unsigned(1024, "Last fetched store table size")
    SSITSize = Param.Unsigned(1024, "Store set ID table size")

    numRobs = Param.Unsigned(1, "Number of Reorder Buffers");

    numPhysIntRegs = Param.Unsigned(256, "Number of physical integer registers")
    numPhysFloatRegs = Param.Unsigned(256, "Number of physical floating point "
                                      "registers")
    numIQEntries = Param.Unsigned(64, "Number of instruction queue entries")
    numROBEntries = Param.Unsigned(192, "Number of reorder buffer entries")

    instShiftAmt = Param.Unsigned(2, "Number of bits to shift instructions by")

    function_trace = Param.Bool(False, "Enable function trace")
    function_trace_start = Param.Tick(0, "Cycle to start function trace")

    smtNumFetchingThreads = Param.Unsigned("SMT Number of Fetching Threads")
    smtFetchPolicy = Param.String("SMT Fetch policy")
    smtLSQPolicy    = Param.String("SMT LSQ Sharing Policy")
    smtLSQThreshold = Param.String("SMT LSQ Threshold Sharing Parameter")
    smtIQPolicy    = Param.String("SMT IQ Sharing Policy")
    smtIQThreshold = Param.String("SMT IQ Threshold Sharing Parameter")
    smtROBPolicy   = Param.String("SMT ROB Sharing Policy")
    smtROBThreshold = Param.String("SMT ROB Threshold Sharing Parameter")
    smtCommitPolicy = Param.String("SMT Commit Policy")
