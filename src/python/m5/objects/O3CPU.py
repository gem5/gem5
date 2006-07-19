from m5 import build_env
from m5.config import *
from BaseCPU import BaseCPU

class DerivO3CPU(BaseCPU):
    type = 'DerivO3CPU'
    activity = Param.Unsigned("Initial count")
    numThreads = Param.Unsigned("number of HW thread contexts")

    checker = Param.BaseCPU(NULL, "checker")

    cachePorts = Param.Unsigned("Cache Ports")
    icache_port = Port("Instruction Port")
    dcache_port = Port("Data Port")

    decodeToFetchDelay = Param.Unsigned("Decode to fetch delay")
    renameToFetchDelay = Param.Unsigned("Rename to fetch delay")
    iewToFetchDelay = Param.Unsigned("Issue/Execute/Writeback to fetch "
               "delay")
    commitToFetchDelay = Param.Unsigned("Commit to fetch delay")
    fetchWidth = Param.Unsigned("Fetch width")

    renameToDecodeDelay = Param.Unsigned("Rename to decode delay")
    iewToDecodeDelay = Param.Unsigned("Issue/Execute/Writeback to decode "
               "delay")
    commitToDecodeDelay = Param.Unsigned("Commit to decode delay")
    fetchToDecodeDelay = Param.Unsigned("Fetch to decode delay")
    decodeWidth = Param.Unsigned("Decode width")

    iewToRenameDelay = Param.Unsigned("Issue/Execute/Writeback to rename "
               "delay")
    commitToRenameDelay = Param.Unsigned("Commit to rename delay")
    decodeToRenameDelay = Param.Unsigned("Decode to rename delay")
    renameWidth = Param.Unsigned("Rename width")

    commitToIEWDelay = Param.Unsigned("Commit to "
               "Issue/Execute/Writeback delay")
    renameToIEWDelay = Param.Unsigned("Rename to "
               "Issue/Execute/Writeback delay")
    issueToExecuteDelay = Param.Unsigned("Issue to execute delay (internal "
              "to the IEW stage)")
    dispatchWidth = Param.Unsigned("Dispatch width")
    issueWidth = Param.Unsigned("Issue width")
    wbWidth = Param.Unsigned("Writeback width")
    wbDepth = Param.Unsigned("Writeback depth")
    fuPool = Param.FUPool(NULL, "Functional Unit pool")

    iewToCommitDelay = Param.Unsigned("Issue/Execute/Writeback to commit "
               "delay")
    renameToROBDelay = Param.Unsigned("Rename to reorder buffer delay")
    commitWidth = Param.Unsigned("Commit width")
    squashWidth = Param.Unsigned("Squash width")
    trapLatency = Param.Tick("Trap latency")
    fetchTrapLatency = Param.Tick("Fetch trap latency")

    backComSize = Param.Unsigned("Time buffer size for backwards communication")
    forwardComSize = Param.Unsigned("Time buffer size for forward communication")

    predType = Param.String("Branch predictor type ('local', 'tournament')")
    localPredictorSize = Param.Unsigned("Size of local predictor")
    localCtrBits = Param.Unsigned("Bits per counter")
    localHistoryTableSize = Param.Unsigned("Size of local history table")
    localHistoryBits = Param.Unsigned("Bits for the local history")
    globalPredictorSize = Param.Unsigned("Size of global predictor")
    globalCtrBits = Param.Unsigned("Bits per counter")
    globalHistoryBits = Param.Unsigned("Bits of history")
    choicePredictorSize = Param.Unsigned("Size of choice predictor")
    choiceCtrBits = Param.Unsigned("Bits of choice counters")

    BTBEntries = Param.Unsigned("Number of BTB entries")
    BTBTagSize = Param.Unsigned("Size of the BTB tags, in bits")

    RASSize = Param.Unsigned("RAS size")

    LQEntries = Param.Unsigned("Number of load queue entries")
    SQEntries = Param.Unsigned("Number of store queue entries")
    LFSTSize = Param.Unsigned("Last fetched store table size")
    SSITSize = Param.Unsigned("Store set ID table size")

    numRobs = Param.Unsigned("Number of Reorder Buffers");

    numPhysIntRegs = Param.Unsigned("Number of physical integer registers")
    numPhysFloatRegs = Param.Unsigned("Number of physical floating point "
               "registers")
    numIQEntries = Param.Unsigned("Number of instruction queue entries")
    numROBEntries = Param.Unsigned("Number of reorder buffer entries")

    instShiftAmt = Param.Unsigned("Number of bits to shift instructions by")

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
