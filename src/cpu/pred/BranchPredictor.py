# Copyright (c) 2022-2023 The University of Edinburgh
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2012 Mark D. Hill and David A. Wood
# Copyright (c) 2015 The University of Wisconsin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *
from m5.SimObject import *


class BranchType(Enum):
    vals = [
        "NoBranch",
        "Return",
        "CallDirect",
        "CallIndirect",  # 'Call',
        "DirectCond",
        "DirectUncond",  # 'Direct',
        "IndirectCond",
        "IndirectUncond",  #'Indirect',
    ]


class TargetProvider(Enum):
    vals = [
        "NoTarget",
        "BTB",
        "RAS",
        "Indirect",
    ]


class ReturnAddrStack(SimObject):
    type = "ReturnAddrStack"
    cxx_class = "gem5::branch_prediction::ReturnAddrStack"
    cxx_header = "cpu/pred/ras.hh"

    numThreads = Param.Unsigned(Parent.numThreads, "Number of threads")
    numEntries = Param.Unsigned(16, "Number of RAS entries")


class BranchTargetBuffer(ClockedObject):
    type = "BranchTargetBuffer"
    cxx_class = "gem5::branch_prediction::BranchTargetBuffer"
    cxx_header = "cpu/pred/btb.hh"
    abstract = True

    numThreads = Param.Unsigned(Parent.numThreads, "Number of threads")


class SimpleBTB(BranchTargetBuffer):
    type = "SimpleBTB"
    cxx_class = "gem5::branch_prediction::SimpleBTB"
    cxx_header = "cpu/pred/simple_btb.hh"

    numEntries = Param.Unsigned(4096, "Number of BTB entries")
    tagBits = Param.Unsigned(16, "Size of the BTB tags, in bits")
    instShiftAmt = Param.Unsigned(
        Parent.instShiftAmt, "Number of bits to shift instructions by"
    )


class IndirectPredictor(SimObject):
    type = "IndirectPredictor"
    cxx_class = "gem5::branch_prediction::IndirectPredictor"
    cxx_header = "cpu/pred/indirect.hh"
    abstract = True

    numThreads = Param.Unsigned(Parent.numThreads, "Number of threads")


class SimpleIndirectPredictor(IndirectPredictor):
    type = "SimpleIndirectPredictor"
    cxx_class = "gem5::branch_prediction::SimpleIndirectPredictor"
    cxx_header = "cpu/pred/simple_indirect.hh"

    indirectHashGHR = Param.Bool(True, "Hash branch predictor GHR")
    indirectHashTargets = Param.Bool(True, "Hash path history targets")
    indirectSets = Param.Unsigned(256, "Cache sets for indirect predictor")
    indirectWays = Param.Unsigned(2, "Ways for indirect predictor")
    indirectTagSize = Param.Unsigned(16, "Indirect target cache tag bits")
    indirectPathLength = Param.Unsigned(
        3, "Previous indirect targets to use for path history"
    )
    speculativePathLength = Param.Unsigned(
        256,
        "Additional buffer space to store speculative path history. "
        "If there are more speculative branches in flight the history cannot "
        "be recovered. Set this to an appropriate value respective the CPU"
        "pipeline depth or a high value e.g. 256 to make it 'unlimited'.",
    )
    indirectGHRBits = Param.Unsigned(13, "Indirect GHR number of bits")
    instShiftAmt = Param.Unsigned(2, "Number of bits to shift instructions by")


class BranchPredictor(SimObject):
    type = "BranchPredictor"
    cxx_class = "gem5::branch_prediction::BPredUnit"
    cxx_header = "cpu/pred/bpred_unit.hh"
    abstract = True

    numThreads = Param.Unsigned(Parent.numThreads, "Number of threads")
    instShiftAmt = Param.Unsigned(2, "Number of bits to shift instructions by")

    btb = Param.BranchTargetBuffer(SimpleBTB(), "Branch target buffer (BTB)")
    ras = Param.ReturnAddrStack(
        ReturnAddrStack(), "Return address stack, set to NULL to disable RAS."
    )
    indirectBranchPred = Param.IndirectPredictor(
        SimpleIndirectPredictor(),
        "Indirect branch predictor, set to NULL to disable "
        "indirect predictions",
    )


class LocalBP(BranchPredictor):
    type = "LocalBP"
    cxx_class = "gem5::branch_prediction::LocalBP"
    cxx_header = "cpu/pred/2bit_local.hh"

    localPredictorSize = Param.Unsigned(2048, "Size of local predictor")
    localCtrBits = Param.Unsigned(2, "Bits per counter")


class TournamentBP(BranchPredictor):
    type = "TournamentBP"
    cxx_class = "gem5::branch_prediction::TournamentBP"
    cxx_header = "cpu/pred/tournament.hh"

    localPredictorSize = Param.Unsigned(2048, "Size of local predictor")
    localCtrBits = Param.Unsigned(2, "Bits per counter")
    localHistoryTableSize = Param.Unsigned(2048, "size of local history table")
    globalPredictorSize = Param.Unsigned(8192, "Size of global predictor")
    globalCtrBits = Param.Unsigned(2, "Bits per counter")
    choicePredictorSize = Param.Unsigned(8192, "Size of choice predictor")
    choiceCtrBits = Param.Unsigned(2, "Bits of choice counters")


class BiModeBP(BranchPredictor):
    type = "BiModeBP"
    cxx_class = "gem5::branch_prediction::BiModeBP"
    cxx_header = "cpu/pred/bi_mode.hh"

    globalPredictorSize = Param.Unsigned(8192, "Size of global predictor")
    globalCtrBits = Param.Unsigned(2, "Bits per counter")
    choicePredictorSize = Param.Unsigned(8192, "Size of choice predictor")
    choiceCtrBits = Param.Unsigned(2, "Bits of choice counters")


class TAGEBase(SimObject):
    type = "TAGEBase"
    cxx_class = "gem5::branch_prediction::TAGEBase"
    cxx_header = "cpu/pred/tage_base.hh"

    numThreads = Param.Unsigned(Parent.numThreads, "Number of threads")
    instShiftAmt = Param.Unsigned(
        Parent.instShiftAmt, "Number of bits to shift instructions by"
    )

    nHistoryTables = Param.Unsigned(7, "Number of history tables")
    minHist = Param.Unsigned(5, "Minimum history size of TAGE")
    maxHist = Param.Unsigned(130, "Maximum history size of TAGE")

    tagTableTagWidths = VectorParam.Unsigned(
        [0, 9, 9, 10, 10, 11, 11, 12], "Tag size in TAGE tag tables"
    )
    logTagTableSizes = VectorParam.Int(
        [13, 9, 9, 9, 9, 9, 9, 9], "Log2 of TAGE table sizes"
    )
    logRatioBiModalHystEntries = Param.Unsigned(
        2,
        "Log num of prediction entries for a shared hysteresis bit "
        "for the Bimodal",
    )

    tagTableCounterBits = Param.Unsigned(3, "Number of tag table counter bits")
    tagTableUBits = Param.Unsigned(2, "Number of tag table u bits")

    histBufferSize = Param.Unsigned(
        2097152,
        "A large number to track all branch histories(2MEntries default)",
    )

    pathHistBits = Param.Unsigned(16, "Path history size")
    logUResetPeriod = Param.Unsigned(
        18, "Log period in number of branches to reset TAGE useful counters"
    )
    numUseAltOnNa = Param.Unsigned(1, "Number of USE_ALT_ON_NA counters")
    initialTCounterValue = Param.Int(1 << 17, "Initial value of tCounter")
    useAltOnNaBits = Param.Unsigned(4, "Size of the USE_ALT_ON_NA counter(s)")

    maxNumAlloc = Param.Unsigned(
        1, "Max number of TAGE entries allocted on mispredict"
    )

    # List of enabled TAGE tables. If empty, all are enabled
    noSkip = VectorParam.Bool([], "Vector of enabled TAGE tables")

    speculativeHistUpdate = Param.Bool(
        True, "Use speculative update for histories"
    )


# TAGE branch predictor as described in https://www.jilp.org/vol8/v8paper1.pdf
# The default sizes below are for the 8C-TAGE configuration (63.5 Kbits)
class TAGE(BranchPredictor):
    type = "TAGE"
    cxx_class = "gem5::branch_prediction::TAGE"
    cxx_header = "cpu/pred/tage.hh"

    tage = Param.TAGEBase(TAGEBase(), "Tage object")


class LTAGE_TAGE(TAGEBase):
    nHistoryTables = 12
    minHist = 4
    maxHist = 640
    tagTableTagWidths = [0, 7, 7, 8, 8, 9, 10, 11, 12, 12, 13, 14, 15]
    logTagTableSizes = [14, 10, 10, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9]
    logUResetPeriod = 19


class LoopPredictor(SimObject):
    type = "LoopPredictor"
    cxx_class = "gem5::branch_prediction::LoopPredictor"
    cxx_header = "cpu/pred/loop_predictor.hh"

    logSizeLoopPred = Param.Unsigned(8, "Log size of the loop predictor")
    withLoopBits = Param.Unsigned(7, "Size of the WITHLOOP counter")
    loopTableAgeBits = Param.Unsigned(8, "Number of age bits per loop entry")
    loopTableConfidenceBits = Param.Unsigned(
        2, "Number of confidence bits per loop entry"
    )
    loopTableTagBits = Param.Unsigned(14, "Number of tag bits per loop entry")
    loopTableIterBits = Param.Unsigned(14, "Nuber of iteration bits per loop")
    logLoopTableAssoc = Param.Unsigned(2, "Log loop predictor associativity")

    # Parameters for enabling modifications to the loop predictor
    # They have been copied from TAGE-GSC-IMLI
    # (http://www.irisa.fr/alf/downloads/seznec/TAGE-GSC-IMLI.tar)
    #
    # All of them should be disabled to match the original LTAGE implementation
    # (http://hpca23.cse.tamu.edu/taco/camino/cbp2/cbp-src/realistic-seznec.h)

    # Add speculation
    useSpeculation = Param.Bool(False, "Use speculation")

    # Add hashing for calculating the loop table index
    useHashing = Param.Bool(False, "Use hashing")

    # Add a direction bit to the loop table entries
    useDirectionBit = Param.Bool(False, "Use direction info")

    # If true, use random to decide whether to allocate or not, and only try
    # with one entry
    restrictAllocation = Param.Bool(
        False, "Restrict the allocation conditions"
    )

    initialLoopIter = Param.Unsigned(1, "Initial iteration number")
    initialLoopAge = Param.Unsigned(255, "Initial age value")
    optionalAgeReset = Param.Bool(
        True, "Reset age bits optionally in some cases"
    )


class TAGE_SC_L_TAGE(TAGEBase):
    type = "TAGE_SC_L_TAGE"
    cxx_class = "gem5::branch_prediction::TAGE_SC_L_TAGE"
    cxx_header = "cpu/pred/tage_sc_l.hh"
    abstract = True

    tagTableTagWidths = [0]
    numUseAltOnNa = 16
    pathHistBits = 27
    maxNumAlloc = 2
    logUResetPeriod = 10
    initialTCounterValue = 1 << 9
    useAltOnNaBits = 5
    # TODO No speculation implemented as of now
    speculativeHistUpdate = False

    # This size does not set the final sizes of the tables (it is just used
    # for some calculations)
    # Instead, the number of TAGE entries comes from shortTagsTageEntries and
    # longTagsTageEntries
    logTagTableSize = Param.Unsigned("Log size of each tag table")

    shortTagsTageFactor = Param.Unsigned(
        "Factor for calculating the total number of short tags TAGE entries"
    )

    longTagsTageFactor = Param.Unsigned(
        "Factor for calculating the total number of long tags TAGE entries"
    )

    shortTagsSize = Param.Unsigned(8, "Size of the short tags")

    longTagsSize = Param.Unsigned("Size of the long tags")

    firstLongTagTable = Param.Unsigned("First table with long tags")

    truncatePathHist = Param.Bool(
        True, "Truncate the path history to its configured size"
    )


class TAGE_SC_L_TAGE_64KB(TAGE_SC_L_TAGE):
    type = "TAGE_SC_L_TAGE_64KB"
    cxx_class = "gem5::branch_prediction::TAGE_SC_L_TAGE_64KB"
    cxx_header = "cpu/pred/tage_sc_l_64KB.hh"

    nHistoryTables = 36

    minHist = 6
    maxHist = 3000

    tagTableUBits = 1

    logTagTableSizes = [13]

    # This is used to handle the 2-way associativity
    # (all odd entries are set to one, and if the corresponding even entry
    # is set to one, then there is a 2-way associativity for this pair)
    # Entry 0 is for the bimodal and it is ignored
    # Note: For this implementation, some odd entries are also set to 0 to save
    # some bits
    noSkip = [
        0,
        0,
        1,
        0,
        0,
        0,
        1,
        0,
        0,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        0,
        1,
        0,
        1,
        0,
        1,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        1,
    ]

    logTagTableSize = 10
    shortTagsTageFactor = 10
    longTagsTageFactor = 20

    longTagsSize = 12

    firstLongTagTable = 13


class TAGE_SC_L_TAGE_8KB(TAGE_SC_L_TAGE):
    type = "TAGE_SC_L_TAGE_8KB"
    cxx_class = "gem5::branch_prediction::TAGE_SC_L_TAGE_8KB"
    cxx_header = "cpu/pred/tage_sc_l_8KB.hh"

    nHistoryTables = 30

    minHist = 4
    maxHist = 1000

    logTagTableSize = 7
    shortTagsTageFactor = 9
    longTagsTageFactor = 17
    longTagsSize = 12

    logTagTableSizes = [12]

    firstLongTagTable = 11

    truncatePathHist = False

    noSkip = [
        0,
        0,
        1,
        0,
        1,
        0,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        0,
        1,
        0,
        1,
        0,
        1,
        0,
        1,
        0,
        1,
    ]

    tagTableUBits = 2


# LTAGE branch predictor as described in
# https://www.irisa.fr/caps/people/seznec/L-TAGE.pdf
# It is basically a TAGE predictor plus a loop predictor
# The differnt TAGE sizes are updated according to the paper values (256 Kbits)
class LTAGE(TAGE):
    type = "LTAGE"
    cxx_class = "gem5::branch_prediction::LTAGE"
    cxx_header = "cpu/pred/ltage.hh"

    tage = LTAGE_TAGE()

    loop_predictor = Param.LoopPredictor(LoopPredictor(), "Loop predictor")


class TAGE_SC_L_LoopPredictor(LoopPredictor):
    type = "TAGE_SC_L_LoopPredictor"
    cxx_class = "gem5::branch_prediction::TAGE_SC_L_LoopPredictor"
    cxx_header = "cpu/pred/tage_sc_l.hh"

    loopTableAgeBits = 4
    loopTableConfidenceBits = 4
    loopTableTagBits = 10
    loopTableIterBits = 10
    useSpeculation = False
    useHashing = True
    useDirectionBit = True
    restrictAllocation = True
    initialLoopIter = 0
    initialLoopAge = 7
    optionalAgeReset = False


class StatisticalCorrector(SimObject):
    type = "StatisticalCorrector"
    cxx_class = "gem5::branch_prediction::StatisticalCorrector"
    cxx_header = "cpu/pred/statistical_corrector.hh"
    abstract = True

    # Statistical corrector parameters

    numEntriesFirstLocalHistories = Param.Unsigned(
        "Number of entries for first local histories"
    )

    bwnb = Param.Unsigned("Num global backward branch GEHL lengths")
    bwm = VectorParam.Int("Global backward branch GEHL lengths")
    logBwnb = Param.Unsigned("Log num of global backward branch GEHL entries")
    bwWeightInitValue = Param.Int(
        "Initial value of the weights of the global backward branch GEHL entries"
    )

    lnb = Param.Unsigned("Num first local history GEHL lenghts")
    lm = VectorParam.Int("First local history GEHL lengths")
    logLnb = Param.Unsigned("Log number of first local history GEHL entries")
    lWeightInitValue = Param.Int(
        "Initial value of the weights of the first local history GEHL entries"
    )

    inb = Param.Unsigned(1, "Num IMLI GEHL lenghts")
    im = VectorParam.Int([8], "IMLI history GEHL lengths")
    logInb = Param.Unsigned("Log number of IMLI GEHL entries")
    iWeightInitValue = Param.Int(
        "Initial value of the weights of the IMLI history GEHL entries"
    )

    logBias = Param.Unsigned("Log size of Bias tables")

    logSizeUp = Param.Unsigned(
        6, "Log size of update threshold counters tables"
    )

    chooserConfWidth = Param.Unsigned(
        7, "Number of bits for the chooser counters"
    )

    updateThresholdWidth = Param.Unsigned(
        12, "Number of bits for the update threshold counter"
    )

    pUpdateThresholdWidth = Param.Unsigned(
        8, "Number of bits for the pUpdate threshold counters"
    )

    extraWeightsWidth = Param.Unsigned(
        6, "Number of bits for the extra weights"
    )

    scCountersWidth = Param.Unsigned(6, "Statistical corrector counters width")

    initialUpdateThresholdValue = Param.Int(
        0, "Initial pUpdate threshold counter value"
    )


# TAGE-SC-L branch predictor as desribed in
# https://www.jilp.org/cbp2016/paper/AndreSeznecLimited.pdf
# It is a modified LTAGE predictor plus a statistical corrector predictor
# The TAGE modifications include bank interleaving and partial associativity
# Two different sizes are proposed in the paper:
# 8KB => See TAGE_SC_L_8KB below
# 64KB => See TAGE_SC_L_64KB below
# The TAGE_SC_L_8KB and TAGE_SC_L_64KB classes differ not only on the values
# of some parameters, but also in some implementation details
# Given this, the TAGE_SC_L class is left abstract
# Note that as it is now, this branch predictor does not handle any type
# of speculation: All the structures/histories are updated at commit time
class TAGE_SC_L(LTAGE):
    type = "TAGE_SC_L"
    cxx_class = "gem5::branch_prediction::TAGE_SC_L"
    cxx_header = "cpu/pred/tage_sc_l.hh"
    abstract = True

    statistical_corrector = Param.StatisticalCorrector("Statistical Corrector")


class TAGE_SC_L_64KB_LoopPredictor(TAGE_SC_L_LoopPredictor):
    logSizeLoopPred = 5


class TAGE_SC_L_8KB_LoopPredictor(TAGE_SC_L_LoopPredictor):
    logSizeLoopPred = 3


class TAGE_SC_L_64KB_StatisticalCorrector(StatisticalCorrector):
    type = "TAGE_SC_L_64KB_StatisticalCorrector"
    cxx_class = "gem5::branch_prediction::TAGE_SC_L_64KB_StatisticalCorrector"
    cxx_header = "cpu/pred/tage_sc_l_64KB.hh"

    pnb = Param.Unsigned(3, "Num variation global branch GEHL lengths")
    pm = VectorParam.Int([25, 16, 9], "Variation global branch GEHL lengths")
    logPnb = Param.Unsigned(
        9, "Log number of variation global branch GEHL entries"
    )

    snb = Param.Unsigned(3, "Num second local history GEHL lenghts")
    sm = VectorParam.Int([16, 11, 6], "Second local history GEHL lengths")
    logSnb = Param.Unsigned(
        9, "Log number of second local history GEHL entries"
    )

    tnb = Param.Unsigned(2, "Num third local history GEHL lenghts")
    tm = VectorParam.Int([9, 4], "Third local history GEHL lengths")
    logTnb = Param.Unsigned(
        10, "Log number of third local history GEHL entries"
    )

    imnb = Param.Unsigned(2, "Num second IMLI GEHL lenghts")
    imm = VectorParam.Int([10, 4], "Second IMLI history GEHL lengths")
    logImnb = Param.Unsigned(9, "Log number of second IMLI GEHL entries")

    numEntriesSecondLocalHistories = Param.Unsigned(
        16, "Number of entries for second local histories"
    )
    numEntriesThirdLocalHistories = Param.Unsigned(
        16, "Number of entries for second local histories"
    )

    numEntriesFirstLocalHistories = 256

    logBias = 8

    bwnb = 3
    bwm = [40, 24, 10]
    logBwnb = 10
    bwWeightInitValue = 7

    lnb = 3
    lm = [11, 6, 3]
    logLnb = 10
    lWeightInitValue = 7

    logInb = 8
    iWeightInitValue = 7


class TAGE_SC_L_8KB_StatisticalCorrector(StatisticalCorrector):
    type = "TAGE_SC_L_8KB_StatisticalCorrector"
    cxx_class = "gem5::branch_prediction::TAGE_SC_L_8KB_StatisticalCorrector"
    cxx_header = "cpu/pred/tage_sc_l_8KB.hh"

    gnb = Param.Unsigned(2, "Num global branch GEHL lengths")
    gm = VectorParam.Int([6, 3], "Global branch GEHL lengths")
    logGnb = Param.Unsigned(7, "Log number of global branch GEHL entries")

    numEntriesFirstLocalHistories = 64

    logBias = 7

    bwnb = 2
    logBwnb = 7
    bwm = [16, 8]
    bwWeightInitValue = 7

    lnb = 2
    logLnb = 7
    lm = [6, 3]
    lWeightInitValue = 7

    logInb = 7
    iWeightInitValue = 7


# 64KB TAGE-SC-L branch predictor as described in
# http://www.jilp.org/cbp2016/paper/AndreSeznecLimited.pdf
class TAGE_SC_L_64KB(TAGE_SC_L):
    type = "TAGE_SC_L_64KB"
    cxx_class = "gem5::branch_prediction::TAGE_SC_L_64KB"
    cxx_header = "cpu/pred/tage_sc_l_64KB.hh"

    tage = TAGE_SC_L_TAGE_64KB()
    loop_predictor = TAGE_SC_L_64KB_LoopPredictor()
    statistical_corrector = TAGE_SC_L_64KB_StatisticalCorrector()


# 8KB TAGE-SC-L branch predictor as described in
# http://www.jilp.org/cbp2016/paper/AndreSeznecLimited.pdf
class TAGE_SC_L_8KB(TAGE_SC_L):
    type = "TAGE_SC_L_8KB"
    cxx_class = "gem5::branch_prediction::TAGE_SC_L_8KB"
    cxx_header = "cpu/pred/tage_sc_l_8KB.hh"

    tage = TAGE_SC_L_TAGE_8KB()
    loop_predictor = TAGE_SC_L_8KB_LoopPredictor()
    statistical_corrector = TAGE_SC_L_8KB_StatisticalCorrector()


class MultiperspectivePerceptron(BranchPredictor):
    type = "MultiperspectivePerceptron"
    cxx_class = "gem5::branch_prediction::MultiperspectivePerceptron"
    cxx_header = "cpu/pred/multiperspective_perceptron.hh"
    abstract = True

    num_filter_entries = Param.Int("Number of filter entries")
    num_local_histories = Param.Int("Number of local history entries")
    local_history_length = Param.Int(
        11, "Length in bits of each history entry"
    )

    block_size = Param.Int(
        21,
        "number of ghist bits in a 'block'; this is the width of an initial "
        "hash of ghist",
    )
    pcshift = Param.Int(-10, "Shift for hashing PC")
    threshold = Param.Int(1, "Threshold for deciding low/high confidence")
    bias0 = Param.Int(
        -5, "Bias perceptron output this much on all-bits-zero local history"
    )
    bias1 = Param.Int(
        5, "Bias perceptron output this much on all-bits-one local history"
    )
    biasmostly0 = Param.Int(
        -1,
        "Bias perceptron output this much on almost-all-bits-zero local "
        "history",
    )
    biasmostly1 = Param.Int(
        1,
        "Bias perceptron output this much on almost-all-bits-one local "
        "history",
    )
    nbest = Param.Int(
        20,
        "Use this many of the top performing tables on a low-confidence "
        "branch",
    )
    tunebits = Param.Int(24, "Number of bits in misprediction counters")
    hshift = Param.Int(
        -6,
        "How much to shift initial feauture hash before XORing with PC bits",
    )
    imli_mask1 = Param.UInt64(
        "Which tables should have their indices hashed with the first IMLI "
        "counter"
    )
    imli_mask4 = Param.UInt64(
        "Which tables should have their indices hashed with the fourth IMLI "
        "counter"
    )
    recencypos_mask = Param.UInt64(
        "Which tables should have their indices hashed with the recency "
        "position"
    )
    fudge = Param.Float(0.245, "Fudge factor to multiply by perceptron output")
    n_sign_bits = Param.Int(2, "Number of sign bits per magnitude")
    pcbit = Param.Int(2, "Bit from the PC to use for hashing global history")
    decay = Param.Int(0, "Whether and how often to decay a random weight")
    record_mask = Param.Int(
        191, "Which histories are updated with filtered branch outcomes"
    )
    hash_taken = Param.Bool(
        False, "Hash the taken/not taken value with a PC bit"
    )
    tuneonly = Param.Bool(
        True, "If true, only count mispredictions of low-confidence branches"
    )
    extra_rounds = Param.Int(
        1,
        "Number of extra rounds of training a single weight on a "
        "low-confidence prediction",
    )
    speed = Param.Int(9, "Adaptive theta learning speed")
    initial_theta = Param.Int(10, "Initial theta")
    budgetbits = Param.Int("Hardware budget in bits")
    speculative_update = Param.Bool(
        False, "Use speculative update for histories"
    )

    initial_ghist_length = Param.Int(1, "Initial GHist length value")
    ignore_path_size = Param.Bool(False, "Ignore the path storage")


class MultiperspectivePerceptron8KB(MultiperspectivePerceptron):
    type = "MultiperspectivePerceptron8KB"
    cxx_class = "gem5::branch_prediction::MultiperspectivePerceptron8KB"
    cxx_header = "cpu/pred/multiperspective_perceptron_8KB.hh"

    budgetbits = 8192 * 8 + 2048
    num_local_histories = 48
    num_filter_entries = 0
    imli_mask1 = 0x6
    imli_mask4 = 0x4400
    recencypos_mask = 0x100000090


class MultiperspectivePerceptron64KB(MultiperspectivePerceptron):
    type = "MultiperspectivePerceptron64KB"
    cxx_class = "gem5::branch_prediction::MultiperspectivePerceptron64KB"
    cxx_header = "cpu/pred/multiperspective_perceptron_64KB.hh"

    budgetbits = 65536 * 8 + 2048
    num_local_histories = 510
    num_filter_entries = 18025
    imli_mask1 = 0xC1000
    imli_mask4 = 0x80008000
    recencypos_mask = 0x100000090


class MPP_TAGE(TAGEBase):
    type = "MPP_TAGE"
    cxx_class = "gem5::branch_prediction::MPP_TAGE"
    cxx_header = "cpu/pred/multiperspective_perceptron_tage.hh"

    nHistoryTables = 15
    pathHistBits = 27
    instShiftAmt = 0
    histBufferSize = 16384
    maxHist = 4096
    tagTableTagWidths = [
        0,
        7,
        9,
        9,
        9,
        10,
        11,
        11,
        12,
        12,
        12,
        13,
        14,
        15,
        15,
        15,
    ]
    logTagTableSizes = [
        14,
        10,
        11,
        11,
        11,
        11,
        11,
        12,
        12,
        10,
        11,
        11,
        9,
        7,
        7,
        8,
    ]
    tunedHistoryLengths = VectorParam.Unsigned(
        [0, 5, 12, 15, 21, 31, 43, 64, 93, 137, 200, 292, 424, 612, 877, 1241],
        "Tuned history lengths",
    )

    logUResetPeriod = 10
    initialTCounterValue = 0
    numUseAltOnNa = 512
    speculativeHistUpdate = False


class MPP_LoopPredictor(LoopPredictor):
    type = "MPP_LoopPredictor"
    cxx_class = "gem5::branch_prediction::MPP_LoopPredictor"
    cxx_header = "cpu/pred/multiperspective_perceptron_tage.hh"

    useDirectionBit = True
    useHashing = True
    useSpeculation = False
    loopTableConfidenceBits = 4
    loopTableAgeBits = 4
    initialLoopAge = 7
    initialLoopIter = 0
    loopTableIterBits = 12
    optionalAgeReset = False
    restrictAllocation = True
    logSizeLoopPred = 6
    loopTableTagBits = 10


class MPP_StatisticalCorrector(StatisticalCorrector):
    type = "MPP_StatisticalCorrector"
    cxx_class = "gem5::branch_prediction::MPP_StatisticalCorrector"
    cxx_header = "cpu/pred/multiperspective_perceptron_tage.hh"
    abstract = True

    # Unused in this Statistical Corrector
    bwnb = 0
    bwm = []
    logBwnb = 0
    bwWeightInitValue = -1

    # Unused in this Statistical Corrector
    logInb = 0
    iWeightInitValue = -1

    extraWeightsWidth = 0
    pUpdateThresholdWidth = 10
    initialUpdateThresholdValue = 35
    logSizeUp = 5

    lnb = 3
    lm = [11, 6, 3]
    logLnb = 10
    lWeightInitValue = -1

    gnb = Param.Unsigned(4, "Num global branch GEHL lengths")
    gm = VectorParam.Int([27, 22, 17, 14], "Global branch GEHL lengths")
    logGnb = Param.Unsigned(10, "Log number of global branch GEHL entries")

    pnb = Param.Unsigned(4, "Num variation global branch GEHL lengths")
    pm = VectorParam.Int(
        [16, 11, 6, 3], "Variation global branch GEHL lengths"
    )
    logPnb = Param.Unsigned(
        9, "Log number of variation global branch GEHL entries"
    )


class MultiperspectivePerceptronTAGE(MultiperspectivePerceptron):
    type = "MultiperspectivePerceptronTAGE"
    cxx_class = "gem5::branch_prediction::MultiperspectivePerceptronTAGE"
    cxx_header = "cpu/pred/multiperspective_perceptron_tage.hh"
    abstract = True

    instShiftAmt = 4

    imli_mask1 = 0x70
    imli_mask4 = 0
    num_filter_entries = 0
    num_local_histories = 0
    recencypos_mask = 0  # Unused
    threshold = -1
    initial_ghist_length = 0
    ignore_path_size = True
    n_sign_bits = 1

    tage = Param.TAGEBase("Tage object")
    loop_predictor = Param.LoopPredictor("Loop predictor")
    statistical_corrector = Param.StatisticalCorrector("Statistical Corrector")


class MPP_StatisticalCorrector_64KB(MPP_StatisticalCorrector):
    type = "MPP_StatisticalCorrector_64KB"
    cxx_class = "gem5::branch_prediction::MPP_StatisticalCorrector_64KB"
    cxx_header = "cpu/pred/multiperspective_perceptron_tage_64KB.hh"

    logBias = 8

    snb = Param.Unsigned(4, "Num second local history GEHL lenghts")
    sm = VectorParam.Int([16, 11, 6, 3], "Second local history GEHL lengths")
    logSnb = Param.Unsigned(
        9, "Log number of second local history GEHL entries"
    )

    tnb = Param.Unsigned(3, "Num third local history GEHL lenghts")
    tm = VectorParam.Int([22, 17, 14], "Third local history GEHL lengths")
    logTnb = Param.Unsigned(
        9, "Log number of third local history GEHL entries"
    )

    numEntriesSecondLocalHistories = Param.Unsigned(
        16, "Number of entries for second local histories"
    )
    numEntriesThirdLocalHistories = Param.Unsigned(
        16, "Number of entries for second local histories"
    )

    numEntriesFirstLocalHistories = 256


class MultiperspectivePerceptronTAGE64KB(MultiperspectivePerceptronTAGE):
    type = "MultiperspectivePerceptronTAGE64KB"
    cxx_class = "gem5::branch_prediction::MultiperspectivePerceptronTAGE64KB"
    cxx_header = "cpu/pred/multiperspective_perceptron_tage_64KB.hh"

    budgetbits = 65536 * 8 + 2048

    tage = MPP_TAGE()
    loop_predictor = MPP_LoopPredictor()
    statistical_corrector = MPP_StatisticalCorrector_64KB()


class MPP_TAGE_8KB(MPP_TAGE):
    type = "MPP_TAGE_8KB"
    cxx_class = "gem5::branch_prediction::MPP_TAGE_8KB"
    cxx_header = "cpu/pred/multiperspective_perceptron_tage_8KB.hh"

    nHistoryTables = 10
    tagTableTagWidths = [0, 7, 7, 7, 8, 9, 10, 10, 11, 13, 13]
    logTagTableSizes = [12, 8, 8, 9, 9, 8, 8, 8, 7, 6, 7]
    tunedHistoryLengths = [0, 4, 8, 13, 23, 36, 56, 93, 145, 226, 359]


class MPP_LoopPredictor_8KB(MPP_LoopPredictor):
    type = "MPP_LoopPredictor_8KB"
    cxx_class = "gem5::branch_prediction::MPP_LoopPredictor_8KB"
    cxx_header = "cpu/pred/multiperspective_perceptron_tage_8KB.hh"

    loopTableIterBits = 10
    logSizeLoopPred = 4


class MPP_StatisticalCorrector_8KB(MPP_StatisticalCorrector):
    type = "MPP_StatisticalCorrector_8KB"
    cxx_class = "gem5::branch_prediction::MPP_StatisticalCorrector_8KB"
    cxx_header = "cpu/pred/multiperspective_perceptron_tage_8KB.hh"

    logBias = 7

    lnb = 2
    lm = [8, 3]
    logLnb = 9

    logGnb = 9

    logPnb = 7

    numEntriesFirstLocalHistories = 64


class MultiperspectivePerceptronTAGE8KB(MultiperspectivePerceptronTAGE):
    type = "MultiperspectivePerceptronTAGE8KB"
    cxx_class = "gem5::branch_prediction::MultiperspectivePerceptronTAGE8KB"
    cxx_header = "cpu/pred/multiperspective_perceptron_tage_8KB.hh"

    budgetbits = 8192 * 8 + 2048

    tage = MPP_TAGE_8KB()
    loop_predictor = MPP_LoopPredictor_8KB()
    statistical_corrector = MPP_StatisticalCorrector_8KB()
