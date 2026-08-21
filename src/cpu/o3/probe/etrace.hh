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

#include <unordered_map>
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
    // (ingressPort.adoc tab:itype, 4-bit variant).
    enum IType : uint8_t
    {
        ITYPE_NONE = 0,
        ITYPE_EXCEPTION = 1,
        ITYPE_INTERRUPT = 2,
        ITYPE_EXCEPT_RET = 3,
        ITYPE_NTAKEN_BRANCH = 4,
        ITYPE_TAKEN_BRANCH = 5,
        // 6, 7 reserved in 4-bit mode
        ITYPE_UNINF_CALL = 8,
        ITYPE_INF_CALL = 9,
        ITYPE_UNINF_JUMP = 10,
        ITYPE_INF_JUMP = 11,
        ITYPE_COROUTINE = 12,
        ITYPE_RETURN = 13,
        ITYPE_OTHER_UNINF = 14,
        ITYPE_OTHER_INF = 15,
    };

    // qual_status wire values (spec: Format 3-3 support packet).
    enum QualStatus : uint8_t
    {
        QS_NO_CHANGE = 0x0,
        QS_ENDED_REP = 0x1,
        QS_TRACE_LOST = 0x2,
        QS_ENDED_NTR = 0x3,
    };

    void traceCommit(const DynInstPtr &dynInst);
    void traceDataAccess(
        const std::pair<DynInstPtr, PacketPtr> &data);
    IType classifyInstruction(const DynInstPtr &dynInst);

    // Packet emitters — one method per spec on-wire format/subformat.
    // Format 3-0
    void emitSyncStartPacket(Addr addr, uint8_t priv, bool branch);
    // Format 3-1
    void emitTrapPacket(Addr addr, uint64_t cause, uint64_t tval,
                        bool isInterrupt, uint8_t priv, bool branch);
    // Format 3-2
    void emitContextPacket(uint64_t context, uint8_t priv);
    // Format 3-3
    void emitSupportPacket(QualStatus qual);
    // Format 1 (with or without address).
    void emitBranchMapPacket(bool withAddress, Addr addr);
    // Format 2
    void emitAddrOnlyPacket(Addr addr);
    // Route to Format 1 with-address when branchCount > 0,
    // Format 2 when branchCount == 0.
    void emitAddressPacket(Addr addr);
    // Format 0-0 (correctly-predicted branch count). branch_fmt values
    // per spec: 0 = no-address, 2 = address (mispred taken),
    // 3 = address (mispred not-taken). `mispredTaken` is meaningful
    // only when `withAddress` is true.
    void emitBranchCountPacket(bool withAddress, Addr addr,
                               bool mispredTaken = true);
    // Format 0-1 (JTC hit). `target` is the actual jump target (byte
    // address); the encoder must advance its lastReportedAddr because
    // the decoder resets its reconstructed_addr to the JTC target
    // when it sees this packet.
    void emitJtcHitPacket(uint32_t index, Addr target);

    void resetBranchMap();
    void flushTrace();

    bool predictBranch(Addr pc);
    void updatePredictor(Addr pc, bool taken);
    void resetPredictor();

    bool jtCacheLookup(Addr target, uint32_t &index);
    void jtCacheUpdate(Addr target);
    void jtCacheInvalidate();

    bool isTrapAddrInferable(uint8_t priv, uint64_t cause,
                             bool isInterrupt);
    bool isSeqInferableJump(const DynInstPtr &dynInst);
    bool passesFilter(Addr pc, uint8_t priv);

    // Emit the correctly-predicted branch count if it has reached
    // pbcFlushThreshold, per payload.adoc's "branch count reaches its
    // maximum value" condition (Format 0-0, with-address form,
    // branch_fmt=10 -- the address is that of the branch which just
    // pushed the count to the threshold, and it was itself predicted
    // correctly by construction). addr is that triggering branch's PC.
    void maybeEmitPbc(Addr addr);

    // Format 0-0 "updiscon, interrupt or exception requires the
    // encoder to output an address" flush (payload.adoc bullet 2):
    // if a correctly-predicted-branch count is pending (>=31, the
    // minimum legally encodable value), emit it as a Format 0-0
    // with-address packet carrying addr instead of letting the
    // caller's own address packet (Format 1/2) report it. Returns
    // true if it flushed (and thus emitted addr's report) -- callers
    // must skip their own packet in that case.
    bool maybeFlushPbcForAddress(Addr addr);

    // Update pendingUpdiscon on uninferable discontinuities.
    void markUpdiscon() { pendingUpdiscon = true; }
    // Consume pendingUpdiscon and reset — called when emitting a
    // Format 3 packet that might follow an uninferable target.
    bool consumeUpdiscon()
    {
        bool p = pendingUpdiscon;
        pendingUpdiscon = false;
        return p;
    }

    // Compute the notify/updiscon/irreport XOR-chain bits for the
    // packet that carries this address. In gem5's proto, we store
    // the resolved bit values (not the XOR deltas). Decoder does the
    // inverse XOR chain to verify. The `depth` in irdepth is the
    // current call-counter value when `isExplicitReturn` is true,
    // and 0 otherwise.
    //
    // Spec semantics for the three chained bits (payload.adoc):
    //   notify   : set to differ from address MSB when the encoder
    //              wishes to force the decoder to notice the packet
    //              even under identical addresses (rarely used).
    //   updiscon : set to differ from notify when this reported
    //              instruction follows an uninferable discontinuity
    //              AND is immediately followed by a Format 3 packet
    //              (encoder latches pendingUpdiscon at each uninf jump
    //              and consumes here).
    //   irreport : set to differ from updiscon when this address is
    //              being reported "in the clear" because the implicit-
    //              return call counter was 0 or overflowed. When set,
    //              irdepth carries the counter value at emission time.
    void computeDisambigBits(Addr addr, bool isExplicitReturn,
                             bool &notify, bool &updiscon,
                             bool &irreport, uint32_t &irdepth);

    // Data trace helpers.
    enum DataDiff : uint8_t
    {
        DIFF_FULL_ADDR_FULL_DATA = 0x0,
        DIFF_DELTA_ADDR_XOR_DATA = 0x1,
        DIFF_DELTA_ADDR_FULL_DATA = 0x2,
        DIFF_DELTA_ADDR_DELTA_DATA = 0x3,
    };
    void emitDataPacket(const DynInstPtr &dynInst, Addr addr,
                        uint32_t bytes, bool isLoad, bool isStore,
                        bool isAtomic, bool isLR, bool isSC);
    void resetDataTraceBaselines();
    // Atomic sub-op per spec (3-bit field).
    uint32_t classifyAtomicSubtype(const DynInstPtr &dynInst,
                                   bool &isLR, bool &isSC);

    CPU *cpu = nullptr;
    ProtoOutputStream *traceStream = nullptr;

    static constexpr uint32_t maxBranchMapBits = 31;

    // Branch map state
    uint32_t branchMap = 0;
    uint32_t branchCount = 0;

    // Address tracking for delta encoding
    Addr lastReportedAddr = 0;

    // Expected next PC for exception detection
    Addr expectedNextPC = 0;
    bool haveExpectedPC = false;
    Addr lastCommittedAddr = 0;
    uint8_t lastCommittedPriv = 0;
    // True iff the most recently committed traced instruction was a
    // taken conditional branch. Used to compute the Format 3-1 trap
    // `branch` bit polarity (spec: 0 iff last-committed was a taken
    // branch, else 1).
    bool lastCommittedWasTakenBranch = false;

    // Privilege tracking
    uint8_t lastPriv = 0;

    // Resync tracking
    uint64_t instsSinceSync = 0;
    uint64_t resyncPeriod;

    // Start tracing control
    uint64_t startTraceInst;
    uint64_t totalInstsCommitted = 0;
    bool tracingActive;

    // Whether initial sync has been sent
    bool needsInitialSync = true;

    // Implicit exception mode
    bool implicitException;

    // Implicit return mode
    bool implicitReturn;
    uint32_t callCounter = 0;
    uint32_t callCounterMax;

    // Branch prediction mode
    bool branchPrediction;
    uint32_t bpredSizeP;
    std::vector<uint8_t> bpredTable;
    uint32_t bpredCorrectCount = 0;
    // Count at which to proactively flush a Format 0-0 packet
    // (payload.adoc "reaches its maximum value" condition). Always
    // >= 31, since branch_count is wire-encoded as (count - 31);
    // configurable up to the spec's true 0xffffffff ceiling.
    const uint32_t pbcFlushThreshold;

    // Jump target cache
    bool jumpTargetCache;
    uint32_t cacheSizeP;
    std::vector<Addr> jtCache;
    bool jtCacheValid = false;      // false immediately after sync/invalidate

    // Sequentially inferable jump detection
    bool sijump;
    Addr prevInstPC = 0;
    uint8_t prevInstOpcode = 0;   // 7-bit for full-width, 2-bit quadrant for RVC
    uint8_t prevInstRd = 0;
    unsigned prevInstSize = 0;
    bool prevInstIsRvc = false;       // If true, opcode/funct handled RVC-style
    uint8_t prevInstRvcFunct3 = 0;
    bool havePrevInst = false;

    // Data trace
    bool dataTrace;
    uint32_t dataTraceMode;
    ProtoOutputStream *dataTraceStream = nullptr;
    // Per-size (in bytes) last-address baseline for differential
    // encoding, per dataTracePayload.adoc.
    std::unordered_map<uint8_t, Addr> lastDataAddrBySize;
    // After any sync, first data packet of each size must be full-addr.
    std::unordered_map<uint8_t, bool> dataSizeNeedsFullAddr;

    // Context tracking
    uint32_t contextWidth;
    uint64_t lastContext = 0;

    // Filtering
    uint32_t filterPrivMask;
    Addr filterAddrStart;
    Addr filterAddrEnd;
    bool filterAddrEnabled;
    bool wasFiltered = false;

    // Updiscon tracking (set on uninferable discontinuities; consumed
    // by the next Format 3 packet if any).
    bool pendingUpdiscon = false;
    // Explicit-return tracking. Set when a return is being reported
    // "in the clear" — either the implicit-return call counter was
    // 0 (nothing to unwind) or the counter overflowed (nested deeper
    // than callCounterSizeP allows). Consumed by the next Format 1/2
    // emit, which sets irreport and emits the current call counter
    // as irdepth per payload.adoc §sec:implicit-return.
    bool pendingExplicitReturn = false;
    uint32_t pendingIrdepth = 0;

    // Spec discovery parameters (mirrored from Python params so the
    // header can advertise them and the encoder can size fields).
    uint32_t iaddressWidthP;
    uint32_t iaddressLsbP;
    uint32_t privilegeWidthP;
    uint32_t ecauseWidthP;
    uint32_t timeWidthP;
    uint32_t f0sWidthP;
    bool notimeP;
    bool nocontextP;

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
        statistics::Scalar numBpredMispredict;
        statistics::Scalar numBpcFlushes;
        statistics::Scalar numJtCacheHits;
        statistics::Scalar numSijumpInferred;
        statistics::Scalar numFilteredInsts;
    } stats;
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_PROBE_ETRACE_HH__
