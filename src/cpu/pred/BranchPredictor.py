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
#
# Authors: Nilay Vaish and Dibakar Gope

from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *

class BranchPredictor(SimObject):
    type = 'BranchPredictor'
    cxx_class = 'BPredUnit'
    cxx_header = "cpu/pred/bpred_unit.hh"
    abstract = True

    numThreads = Param.Unsigned(Parent.numThreads, "Number of threads")
    BTBEntries = Param.Unsigned(4096, "Number of BTB entries")
    BTBTagSize = Param.Unsigned(16, "Size of the BTB tags, in bits")
    RASSize = Param.Unsigned(16, "RAS size")
    instShiftAmt = Param.Unsigned(2, "Number of bits to shift instructions by")

    useIndirect = Param.Bool(True, "Use indirect branch predictor")
    indirectHashGHR = Param.Bool(True, "Hash branch predictor GHR")
    indirectHashTargets = Param.Bool(True, "Hash path history targets")
    indirectSets = Param.Unsigned(256, "Cache sets for indirect predictor")
    indirectWays = Param.Unsigned(2, "Ways for indirect predictor")
    indirectTagSize = Param.Unsigned(16, "Indirect target cache tag bits")
    indirectPathLength = Param.Unsigned(3,
        "Previous indirect targets to use for path history")



class LocalBP(BranchPredictor):
    type = 'LocalBP'
    cxx_class = 'LocalBP'
    cxx_header = "cpu/pred/2bit_local.hh"

    localPredictorSize = Param.Unsigned(2048, "Size of local predictor")
    localCtrBits = Param.Unsigned(2, "Bits per counter")


class TournamentBP(BranchPredictor):
    type = 'TournamentBP'
    cxx_class = 'TournamentBP'
    cxx_header = "cpu/pred/tournament.hh"

    localPredictorSize = Param.Unsigned(2048, "Size of local predictor")
    localCtrBits = Param.Unsigned(2, "Bits per counter")
    localHistoryTableSize = Param.Unsigned(2048, "size of local history table")
    globalPredictorSize = Param.Unsigned(8192, "Size of global predictor")
    globalCtrBits = Param.Unsigned(2, "Bits per counter")
    choicePredictorSize = Param.Unsigned(8192, "Size of choice predictor")
    choiceCtrBits = Param.Unsigned(2, "Bits of choice counters")


class BiModeBP(BranchPredictor):
    type = 'BiModeBP'
    cxx_class = 'BiModeBP'
    cxx_header = "cpu/pred/bi_mode.hh"

    globalPredictorSize = Param.Unsigned(8192, "Size of global predictor")
    globalCtrBits = Param.Unsigned(2, "Bits per counter")
    choicePredictorSize = Param.Unsigned(8192, "Size of choice predictor")
    choiceCtrBits = Param.Unsigned(2, "Bits of choice counters")

# TAGE branch predictor as described in https://www.jilp.org/vol8/v8paper1.pdf
# The default sizes below are for the 8C-TAGE configuration (63.5 Kbits)
class TAGE(BranchPredictor):
    type = 'TAGE'
    cxx_class = 'TAGE'
    cxx_header = "cpu/pred/tage.hh"

    nHistoryTables = Param.Unsigned(7, "Number of history tables")
    minHist = Param.Unsigned(5, "Minimum history size of TAGE")
    maxHist = Param.Unsigned(130, "Maximum history size of TAGE")

    tagTableTagWidths = VectorParam.Unsigned(
        [0, 9, 9, 10, 10, 11, 11, 12], "Tag size in TAGE tag tables")
    logTagTableSizes = VectorParam.Int(
        [13, 9, 9, 9, 9, 9, 9, 9], "Log2 of TAGE table sizes")
    logRatioBiModalHystEntries = Param.Unsigned(2,
        "Log num of prediction entries for a shared hysteresis bit " \
        "for the Bimodal")

    tagTableCounterBits = Param.Unsigned(3, "Number of tag table counter bits")
    tagTableUBits = Param.Unsigned(2, "Number of tag table u bits")

    histBufferSize = Param.Unsigned(2097152,
            "A large number to track all branch histories(2MEntries default)")

    pathHistBits = Param.Unsigned(16, "Path history size")
    logUResetPeriod = Param.Unsigned(18,
        "Log period in number of branches to reset TAGE useful counters")
    useAltOnNaBits = Param.Unsigned(4, "Size of the USE_ALT_ON_NA counter")


# LTAGE branch predictor as described in
# https://www.irisa.fr/caps/people/seznec/L-TAGE.pdf
# It is basically a TAGE predictor plus a loop predictor
# The differnt TAGE sizes are updated according to the paper values (256 Kbits)
class LTAGE(TAGE):
    type = 'LTAGE'
    cxx_class = 'LTAGE'
    cxx_header = "cpu/pred/ltage.hh"

    nHistoryTables = 12
    minHist = 4
    maxHist = 640
    tagTableTagWidths = [0, 7, 7, 8, 8, 9, 10, 11, 12, 12, 13, 14, 15]
    logTagTableSizes = [14, 10, 10, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9]
    logUResetPeriod = 19

    logSizeLoopPred = Param.Unsigned(8, "Log size of the loop predictor")
    withLoopBits = Param.Unsigned(7, "Size of the WITHLOOP counter")
    loopTableAgeBits = Param.Unsigned(8, "Number of age bits per loop entry")
    loopTableConfidenceBits = Param.Unsigned(2,
            "Number of confidence bits per loop entry")
    loopTableTagBits = Param.Unsigned(14, "Number of tag bits per loop entry")
    loopTableIterBits = Param.Unsigned(14, "Nuber of iteration bits per loop")
    logLoopTableAssoc = Param.Unsigned(2, "Log loop predictor associativity")

    # Parameters for enabling modifications to the loop predictor
    # They have been copied from ISL-TAGE
    # (https://www.jilp.org/jwac-2/program/03_seznec.tgz)
    #
    # All of them should be disabled to match the original LTAGE implementation
    # (http://hpca23.cse.tamu.edu/taco/camino/cbp2/cbp-src/realistic-seznec.h)

    # Add speculation
    useSpeculation = Param.Bool(False, "Use speculation")

    # Add hashing for calculating the loop table index
    useHashing = Param.Bool(False, "Use hashing")

    # Add a direction bit to the loop table entries
    useDirectionBit = Param.Bool(False, "Use direction info")

