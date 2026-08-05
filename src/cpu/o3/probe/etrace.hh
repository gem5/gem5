/*
 * Copyright (c) 2026 Rajesh Gangam
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __CPU_O3_PROBE_ETRACE_HH__
#define __CPU_O3_PROBE_ETRACE_HH__

#include <utility>
#include <vector>

#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/pagetable.hh"
#include "base/statistics.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "mem/packet.hh"
#include "params/ETrace.hh"
#include "proto/etrace.pb.h"
#include "proto/protoio.hh"
#include "sim/probe/probe_listener_object.hh"

namespace gem5
{

namespace o3
{

class CPU;

class ETrace : public ProbeListenerObject
{

  public:
    ETrace(const ETraceParams &params);
    ~ETrace();

    void regProbeListeners() override;

    std::string
    name() const override
    {
        return ProbeListenerObject::name() + ".etrace";
    }

  private:
    // E-Trace itype encoding per RISC-V Efficient Trace spec v2.0
    enum IType : uint8_t
    {
        ITYPE_NONE = 0,
        ITYPE_EXCEPTION = 1,
        ITYPE_INTERRUPT = 2,
        ITYPE_EXCEPT_RET = 3,
        ITYPE_NTAKEN_BRANCH = 4,
        ITYPE_TAKEN_BRANCH = 5,
        ITYPE_UNINF_JUMP = 6,
        ITYPE_UNINF_CALL = 8,
        ITYPE_INF_CALL = 9,
        ITYPE_UNINF_JUMP2 = 10,
        ITYPE_INF_JUMP2 = 11,
        ITYPE_COROUTINE = 12,
        ITYPE_RETURN = 13,
        ITYPE_OTHER_UNINF = 14,
        ITYPE_OTHER_INF = 15,
    };

    void traceCommit(const DynInstPtr &dynInst);
    void traceDataAccess(
        const std::pair<DynInstPtr, PacketPtr> &data);
    IType classifyInstruction(const DynInstPtr &dynInst);

    void emitSyncPacket(Addr addr, uint8_t priv, bool isBranch, bool taken);
    void emitTrapPacket(Addr addr, uint64_t cause, uint64_t tval,
                        bool isInterrupt, uint8_t priv);
    void emitBranchMapPacket(bool withAddress, Addr addr);
    void emitSupportPacket(bool isStart);
    void emitAddrOnlyPacket(Addr addr, bool notify, bool updiscon,
                            bool irreport, uint32_t irdepth);
    void emitContextPacket(uint64_t context);
    void resetBranchMap();
    void flushTrace();

    bool predictBranch(Addr pc);
    void updatePredictor(Addr pc, bool taken);
    void resetPredictor();

    bool jtCacheLookup(Addr pc, Addr target, uint32_t &index);
    void jtCacheUpdate(Addr pc, Addr target);
    void jtCacheInvalidate();

    bool isTrapAddrInferable(uint8_t priv, uint64_t cause,
                             bool isInterrupt);
    bool isSeqInferableJump(const DynInstPtr &dynInst);
    bool passesFilter(Addr pc, uint8_t priv);
    uint32_t classifyAtomicOp(const DynInstPtr &dynInst);

    CPU *cpu;
    ProtoOutputStream *traceStream;

    static constexpr uint32_t maxBranchMapBits = 31;

    // Branch map state
    uint32_t branchMap;
    uint32_t branchCount;

    // Address tracking for delta encoding
    Addr lastReportedAddr;

    // Expected next PC for exception detection
    Addr expectedNextPC;
    bool haveExpectedPC;
    Addr lastCommittedAddr;

    // Privilege tracking
    uint8_t lastPriv;

    // Resync tracking
    uint64_t instsSinceSync;
    uint64_t resyncPeriod;

    // Start tracing control
    uint64_t startTraceInst;
    uint64_t totalInstsCommitted;
    bool tracingActive;

    // Whether initial sync has been sent
    bool needsInitialSync;

    // Implicit exception mode
    bool implicitException;

    // Implicit return mode
    bool implicitReturn;
    uint32_t callCounter;
    uint32_t callCounterMax;

    // Branch prediction mode
    bool branchPrediction;
    uint32_t bpredSizeP;
    std::vector<uint8_t> bpredTable;
    uint32_t bpredCorrectCount;

    // Jump target cache
    bool jumpTargetCache;
    uint32_t cacheSizeP;
    std::vector<Addr> jtCache;

    // Sequentially inferable jump detection
    bool sijump;
    Addr prevInstPC;
    uint8_t prevInstOpcode;
    uint8_t prevInstRd;
    unsigned prevInstSize;
    bool havePrevInst;

    // Data trace
    bool dataTrace;
    uint32_t dataTraceMode;
    ProtoOutputStream *dataTraceStream;
    Addr lastDataAddr;

    // Context tracking
    uint32_t contextWidth;
    uint64_t lastContext;

    // Filtering
    uint32_t filterPrivMask;
    Addr filterAddrStart;
    Addr filterAddrEnd;
    bool filterAddrEnabled;
    bool wasFiltered;

    // Updiscon tracking
    bool pendingUpdiscon;

    struct ETraceStats : public statistics::Group
    {
        ETraceStats(statistics::Group *parent);
        statistics::Scalar numPackets;
        statistics::Scalar numSyncPackets;
        statistics::Scalar numTrapPackets;
        statistics::Scalar numBranchMapPackets;
        statistics::Scalar numBranches;
        statistics::Scalar totalInstsTraced;
        statistics::Scalar numAddrOnlyPackets;
        statistics::Scalar numSupportPackets;
        statistics::Scalar numContextPackets;
        statistics::Scalar numDataTracePackets;
        statistics::Scalar numImplicitReturns;
        statistics::Scalar numBpredCorrect;
        statistics::Scalar numJtCacheHits;
        statistics::Scalar numSijumpInferred;
        statistics::Scalar numFilteredInsts;
    } stats;
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_PROBE_ETRACE_HH__
