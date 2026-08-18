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

#include "cpu/o3/probe/etrace.hh"

#include <cstring>

#include "arch/riscv/insts/amo.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/regs/misc.hh"
#include "base/callback.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/ETrace.hh"

namespace gem5
{

namespace o3
{

ETrace::ETrace(const ETraceParams &params)
    : ProbeListenerObject(params),
      cpu(nullptr),
      traceStream(nullptr),
      branchMap(0),
      branchCount(0),
      lastReportedAddr(0),
      expectedNextPC(0),
      haveExpectedPC(false),
      lastCommittedAddr(0),
      lastCommittedPriv(0),
      lastCommittedWasTakenBranch(false),
      lastPriv(0),
      instsSinceSync(0),
      resyncPeriod(params.resyncPeriod),
      startTraceInst(params.startTraceInst),
      totalInstsCommitted(0),
      tracingActive(params.startTraceInst == 0),
      needsInitialSync(true),
      implicitException(params.implicitException),
      implicitReturn(params.implicitReturn),
      callCounter(0),
      callCounterMax(params.callCounterSizeP > 0
                         ? (1U << params.callCounterSizeP) - 1
                         : 0),
      branchPrediction(params.branchPrediction),
      bpredSizeP(params.bpredSizeP),
      bpredCorrectCount(0),
      jumpTargetCache(params.jumpTargetCache),
      cacheSizeP(params.cacheSizeP),
      jtCacheValid(false),
      sijump(params.sijump),
      prevInstPC(0),
      prevInstOpcode(0),
      prevInstRd(0),
      prevInstSize(0),
      prevInstIsRvc(false),
      prevInstRvcFunct3(0),
      havePrevInst(false),
      dataTrace(params.dataTrace),
      dataTraceMode(params.dataTraceMode),
      dataTraceStream(nullptr),
      contextWidth(params.contextWidth),
      lastContext(0),
      filterPrivMask(params.filterPriv),
      filterAddrStart(params.filterAddrStart),
      filterAddrEnd(params.filterAddrEnd),
      filterAddrEnabled(params.filterAddrStart != 0 ||
                        params.filterAddrEnd != 0),
      wasFiltered(false),
      pendingUpdiscon(false),
      pendingExplicitReturn(false),
      pendingIrdepth(0),
      iaddressWidthP(params.iaddressWidthP),
      iaddressLsbP(params.iaddressLsbP),
      privilegeWidthP(params.privilegeWidthP),
      ecauseWidthP(params.ecauseWidthP),
      timeWidthP(params.timeWidthP),
      f0sWidthP(params.f0sWidthP),
      notimeP(params.notimeP),
      nocontextP(params.nocontextP),
      stats(this)
{
    cpu = dynamic_cast<CPU *>(params.manager);
    fatal_if(!cpu, "Manager of %s is not of type O3CPU.\n", name());
    fatal_if(cpu->numThreads > 1,
             "%s supports single-threaded "
             "workloads only.\n",
             name());

    std::string filename = simout.resolve(name() + "." + params.traceFile);
    traceStream = new ProtoOutputStream(filename);

    ProtoMessage::ETraceHeader header;
    header.set_obj_id(name());
    header.set_ver(2);
    header.set_tick_freq(sim_clock::Frequency);
    header.set_arch_width(64);
    header.set_iaddress_width_p(iaddressWidthP);
    header.set_iaddress_lsb_p(iaddressLsbP);
    header.set_privilege_width_p(privilegeWidthP);
    header.set_ecause_width_p(ecauseWidthP);
    header.set_context_width_p(contextWidth);
    header.set_time_width_p(notimeP ? 0 : timeWidthP);
    header.set_f0s_width_p(f0sWidthP);
    header.set_cache_size_p(cacheSizeP);
    header.set_call_counter_size_p(params.callCounterSizeP);
    header.set_bpred_size_p(bpredSizeP);
    // Document the bit assignments the encoder is using so decoders
    // built against this header can label the ioptions/doptions bits.
    uint32_t ioBits = 0;
    ioBits |= (implicitReturn && callCounterMax > 0) ? (1u << 0) : 0;
    ioBits |= (branchPrediction && bpredSizeP > 0) ? (1u << 1) : 0;
    ioBits |= (jumpTargetCache && cacheSizeP > 0) ? (1u << 2) : 0;
    ioBits |= sijump ? (1u << 3) : 0;
    ioBits |= implicitException ? (1u << 4) : 0;
    header.set_ioptions_bits(ioBits);
    uint32_t doBits = 0;
    // dataTraceMode == 1 -> no_data (address only); == 2 -> no_addr.
    if (dataTraceMode == 1) doBits |= (1u << 0);
    if (dataTraceMode == 2) doBits |= (1u << 1);
    header.set_doptions_bits(doBits);
    traceStream->write(header);

    if (branchPrediction && bpredSizeP > 0) {
        bpredTable.resize(1U << bpredSizeP, 0x01);
    }

    if (jumpTargetCache && cacheSizeP > 0) {
        jtCache.resize(1U << cacheSizeP, 0);
    }

    if (dataTrace) {
        std::string dataFilename =
            simout.resolve(name() + "." + params.dataTraceFile);
        dataTraceStream = new ProtoOutputStream(dataFilename);
        // Data trace has its own header (same schema) so the decoder
        // can identify the stream and align to packet boundaries.
        ProtoMessage::ETraceHeader dataHeader;
        dataHeader.set_obj_id(name() + ".data");
        dataHeader.set_ver(2);
        dataHeader.set_tick_freq(sim_clock::Frequency);
        dataHeader.set_arch_width(64);
        dataHeader.set_iaddress_width_p(iaddressWidthP);
        dataHeader.set_iaddress_lsb_p(iaddressLsbP);
        dataHeader.set_privilege_width_p(privilegeWidthP);
        dataHeader.set_ecause_width_p(ecauseWidthP);
        dataHeader.set_context_width_p(contextWidth);
        dataHeader.set_time_width_p(notimeP ? 0 : timeWidthP);
        dataTraceStream->write(dataHeader);
    }

    registerExitCallback([this]() { flushTrace(); });
}

ETrace::~ETrace()
{
    delete traceStream;
    delete dataTraceStream;
}

void
ETrace::regProbeListeners()
{
    typedef ProbeListenerArg<ETrace, DynInstPtr> DynInstListener;
    connectListener<DynInstListener>(this, "Commit", &ETrace::traceCommit);

    if (dataTrace) {
        typedef ProbeListenerArg<ETrace,
            std::pair<DynInstPtr, PacketPtr>> DataListener;
        connectListener<DataListener>(
            this, "DataAccessComplete", &ETrace::traceDataAccess);
    }
}

ETrace::IType
ETrace::classifyInstruction(const DynInstPtr &dynInst)
{
    // Exception-return: mret / sret. These are IsNonSpeculative in gem5
    // and IsReturn — matches spec itype 3.
    if (dynInst->isNonSpeculative() && dynInst->isReturn()) {
        return ITYPE_EXCEPT_RET;
    }

    if (dynInst->isCondCtrl()) {
        Addr pc = dynInst->pcState().instAddr();
        Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
        unsigned instSize =
            dynInst->pcState().as<RiscvISA::PCState>().compressed() ? 2 : 4;
        bool taken = (npc != pc + instSize);
        return taken ? ITYPE_TAKEN_BRANCH : ITYPE_NTAKEN_BRANCH;
    }

    // gem5 marks c.jalr x5 as IsReturn but not IsCall. Spec calls this
    // a coroutine swap; fix up by inspecting the raw instruction.
    auto *staticInst = dynInst->staticInst.get();
    auto *riscvStatic = dynamic_cast<RiscvISA::RiscvStaticInst *>(staticInst);
    if (riscvStatic) {
        auto machInst = riscvStatic->machInst;
        bool isRvc =
            dynInst->pcState().as<RiscvISA::PCState>().compressed();
        if (isRvc) {
            uint8_t quad = machInst & 0x3;
            uint8_t funct3 = (machInst >> 13) & 0x7;
            uint8_t rd_rs1 = (machInst >> 7) & 0x1F;
            uint8_t rs2 = (machInst >> 2) & 0x1F;
            // c.jalr = quad=10, funct4=1001, rs2=0 (per RVC spec)
            uint8_t funct4 = (machInst >> 12) & 0xF;
            if (quad == 2 && funct4 == 0x9 && rs2 == 0 && rd_rs1 == 5) {
                // c.jalr x5 -> coroutine swap
                return ITYPE_COROUTINE;
            }
            // c.jr  = quad=10, funct4=1000, rs2=0
            if (quad == 2 && funct4 == 0x8 && rs2 == 0 && rd_rs1 == 5) {
                // c.jr x5 -> also treat as coroutine (jump via link reg)
                return ITYPE_COROUTINE;
            }
            (void)funct3;  // Silence unused warning if RVC path narrows.
        } else {
            // Full-width JALR (opcode 0x67). Spec coroutine cases:
            //   jalr x1, x5, 0  OR  jalr x5, x1, 0
            uint8_t opcode = machInst & 0x7F;
            if (opcode == 0x67) {
                uint8_t rd = (machInst >> 7) & 0x1F;
                uint8_t rs1 = (machInst >> 15) & 0x1F;
                bool rd_link = (rd == 1 || rd == 5);
                bool rs1_link = (rs1 == 1 || rs1 == 5);
                if (rd_link && rs1_link && rd != rs1) {
                    return ITYPE_COROUTINE;
                }
            }
        }
    }

    if (dynInst->isCall() && dynInst->isReturn()) {
        return ITYPE_COROUTINE;
    }

    if (dynInst->isCall()) {
        // Inferable call = direct (jal with link register).
        return dynInst->isDirectCtrl() ? ITYPE_INF_CALL : ITYPE_UNINF_CALL;
    }

    if (dynInst->isReturn()) {
        return ITYPE_RETURN;
    }

    if (dynInst->isControl()) {
        if (dynInst->isDirectCtrl()) {
            // Direct jump. Spec distinguishes:
            //   itype 11 = inferable jump (jal x0, c.j)
            //   itype 15 = other inferable jump (jal rd, rd not link)
            // We don't need to distinguish; both are "target inferable
            // from opcode alone" for the decoder.
            return ITYPE_INF_JUMP;
        }
        // Uninferable indirect jump (jalr with non-link rd).
        return ITYPE_UNINF_JUMP;
    }

    return ITYPE_NONE;
}

bool
ETrace::passesFilter(Addr pc, uint8_t priv)
{
    uint32_t privBit = 1U << priv;
    if (!(filterPrivMask & privBit))
        return false;

    if (filterAddrEnabled) {
        if (pc < filterAddrStart || pc > filterAddrEnd)
            return false;
    }

    return true;
}

bool
ETrace::isTrapAddrInferable(uint8_t priv, uint64_t cause, bool isInterrupt)
{
    if (!implicitException)
        return false;

    gem5::ThreadContext *tc = cpu->getContext(0);
    uint64_t tvec;
    if (priv == RiscvISA::PRV_M) {
        tvec = tc->readMiscRegNoEffect(RiscvISA::MISCREG_MTVEC);
    } else {
        tvec = tc->readMiscRegNoEffect(RiscvISA::MISCREG_STVEC);
    }

    uint8_t mode = tvec & 0x3;
    // Both direct (mode 0) and vectored (mode 1) yield a
    // decoder-inferable address from cause/interrupt.
    return (mode == 0 || mode == 1);
}

bool
ETrace::isSeqInferableJump(const DynInstPtr &dynInst)
{
    if (!sijump || !havePrevInst)
        return false;

    auto *riscvInst = static_cast<RiscvISA::RiscvStaticInst *>(
        dynInst->staticInst.get());
    auto machInst = riscvInst->machInst;
    bool curIsRvc =
        dynInst->pcState().as<RiscvISA::PCState>().compressed();
    Addr curPC = dynInst->pcState().instAddr();

    uint8_t curRs1;
    if (curIsRvc) {
        // c.jr / c.jalr : rd/rs1 field is bits [11:7]
        uint8_t quad = machInst & 0x3;
        uint8_t funct4 = (machInst >> 12) & 0xF;
        uint8_t rs2 = (machInst >> 2) & 0x1F;
        // Must be c.jr (funct4=1000) or c.jalr (funct4=1001), rs2=0
        if (quad != 2 || (funct4 != 0x8 && funct4 != 0x9) || rs2 != 0)
            return false;
        curRs1 = (machInst >> 7) & 0x1F;
    } else {
        uint8_t curOpcode = machInst & 0x7F;
        if (curOpcode != 0x67)  // Full-width JALR
            return false;
        curRs1 = (machInst >> 15) & 0x1F;
    }

    // Previous must be LUI (0x37), AUIPC (0x17), or c.lui (RVC).
    bool prevIsLoader = false;
    if (prevInstIsRvc) {
        // c.lui: quad=01, funct3=011, rd != x0 && rd != x2
        if ((prevInstOpcode & 0x3) == 1 && prevInstRvcFunct3 == 3 &&
            prevInstRd != 0 && prevInstRd != 2) {
            prevIsLoader = true;
        }
    } else {
        if (prevInstOpcode == 0x37 || prevInstOpcode == 0x17)
            prevIsLoader = true;
    }
    if (!prevIsLoader)
        return false;

    // Previous rd must match current rs1
    if (prevInstRd != curRs1)
        return false;

    // Must be sequential (previous PC + previous size == current PC)
    if (prevInstPC + prevInstSize != curPC)
        return false;

    return true;
}

// 2-bit branch predictor per branchTrace.adoc §sec:branch-prediction.
// State encoding:  MSB = prediction (0=NT, 1=T); prediction must fail
// twice to flip.
//   00 (pred NT): taken -> 01, not-taken -> 00
//   01 (pred NT): taken -> 11, not-taken -> 00
//   11 (pred T):  taken -> 11, not-taken -> 10
//   10 (pred T):  taken -> 10, not-taken -> 00
bool
ETrace::predictBranch(Addr pc)
{
    if (!branchPrediction || bpredTable.empty())
        return false;

    uint32_t index = (pc >> 1) & ((1U << bpredSizeP) - 1);
    return (bpredTable[index] >> 1) & 1;
}

void
ETrace::updatePredictor(Addr pc, bool taken)
{
    if (!branchPrediction || bpredTable.empty())
        return;

    uint32_t index = (pc >> 1) & ((1U << bpredSizeP) - 1);
    uint8_t state = bpredTable[index];

    switch (state) {
      case 0x00:                                            // pred NT
        bpredTable[index] = taken ? 0x01 : 0x00;
        break;
      case 0x01:                                            // pred NT
        bpredTable[index] = taken ? 0x03 : 0x00;
        break;
      case 0x03:                                            // pred T (0b11)
        bpredTable[index] = taken ? 0x03 : 0x02;
        break;
      case 0x02:                                            // pred T (0b10)
        bpredTable[index] = taken ? 0x03 : 0x00;
        break;
    }
}

void
ETrace::resetPredictor()
{
    for (auto &entry : bpredTable) {
        entry = 0x01;
    }
    bpredCorrectCount = 0;
}

bool
ETrace::jtCacheLookup(Addr target, uint32_t &index)
{
    if (!jumpTargetCache || jtCache.empty() || !jtCacheValid)
        return false;

    // Spec: index by TARGET address (not source PC), bits
    // [cache_size_p:1] (RVC) or [cache_size_p+1:2] (no-RVC). gem5
    // supports RVC so use >>1.
    index = (target >> 1) & ((1U << cacheSizeP) - 1);
    return jtCache[index] == target;
}

void
ETrace::jtCacheUpdate(Addr target)
{
    if (!jumpTargetCache || jtCache.empty())
        return;

    uint32_t index = (target >> 1) & ((1U << cacheSizeP) - 1);
    jtCache[index] = target;
    jtCacheValid = true;
}

void
ETrace::jtCacheInvalidate()
{
    for (auto &entry : jtCache) {
        entry = 0;
    }
    jtCacheValid = false;
}

uint32_t
ETrace::classifyAtomicSubtype(const DynInstPtr &dynInst, bool &isLR,
                              bool &isSC)
{
    isLR = false;
    isSC = false;

    auto *riscvInst = static_cast<RiscvISA::RiscvStaticInst *>(
        dynInst->staticInst.get());
    auto machInst = riscvInst->machInst;
    uint8_t amofunct = (machInst >> 27) & 0x1F;

    switch (amofunct) {
      case 0x01: return ProtoMessage::ETraceDataPacket::AMOSWAP;
      case 0x00: return ProtoMessage::ETraceDataPacket::AMOADD;
      case 0x0C: return ProtoMessage::ETraceDataPacket::AMOAND;
      case 0x08: return ProtoMessage::ETraceDataPacket::AMOOR;
      case 0x04: return ProtoMessage::ETraceDataPacket::AMOXOR;
      case 0x14: return ProtoMessage::ETraceDataPacket::AMOMAX;
      case 0x10: return ProtoMessage::ETraceDataPacket::AMOMIN;
      // Spec table only has funct=000-110 in the atomic subtype;
      // MAXU/MINU are not in E-Trace v2.0 atomic subtypes and are
      // represented in the reserved slot (funct5 0x18, 0x1C).
      case 0x1C:
      case 0x18: return ProtoMessage::ETraceDataPacket::AMO_RESERVED;
      case 0x02: isLR = true; return ProtoMessage::ETraceDataPacket::AMOADD;
      case 0x03: isSC = true; return ProtoMessage::ETraceDataPacket::AMOADD;
      default:   return ProtoMessage::ETraceDataPacket::AMO_RESERVED;
    }
}

void
ETrace::computeDisambigBits(Addr addr, bool isExplicitReturn,
                            bool &notify_, bool &updiscon_,
                            bool &irreport_, uint32_t &irdepth_)
{
    // In gem5's proto we store the RESOLVED bit values (not the XOR
    // deltas the on-wire form uses); the decoder recovers the intent
    // by XORing back against the address MSB. See payload.adoc for
    // the wire-level XOR chain.
    //
    // Semantics of the resolved bit values:
    //   notify   = address_MSB  by default (no notify).
    //              Toggled to !address_MSB when the encoder wants to
    //              force the decoder to notice the packet under
    //              identical-address edge cases (rarely used).
    //   updiscon = notify       by default.
    //              Toggled to !notify when the reported instruction
    //              follows an uninferable discontinuity AND is
    //              immediately followed by a Format 3 packet (drives
    //              the "loop back-edge → interrupt" disambiguator).
    //   irreport = updiscon     by default.
    //              Toggled to !updiscon when this address is being
    //              reported "in the clear" — implicit-return counter
    //              was 0 (nothing to unwind) or overflowed. When
    //              toggled, irdepth carries the current counter.
    bool addrMsb = (addr >> 63) & 1;
    notify_   = addrMsb;                                     // default
    updiscon_ = notify_ ^ (pendingUpdiscon ? 1 : 0);
    irreport_ = updiscon_ ^ (isExplicitReturn ? 1 : 0);
    // Spec (payload.adoc Format 1/2/0-1): "if irreport is the same
    // value as updiscon, all bits in irdepth also equal updiscon."
    // So when the explicit-return path is NOT active, irdepth is
    // a filler run of the updiscon bit (all-1s if updiscon=1, else
    // all-0s). Only when irreport is toggled does irdepth carry the
    // latched call counter value.
    if (isExplicitReturn) {
        irdepth_ = pendingIrdepth;
    } else {
        // Fill with updiscon bit repeated across the field. gem5 uses
        // the widest irdepth (call_counter_size_p + return_stack_size_p
        // + 1) which is bounded by the width the encoder advertised
        // via header.call_counter_size_p. We fill uint32 fully; the
        // decoder masks to its known width.
        irdepth_ = updiscon_ ? 0xFFFFFFFFu : 0u;
    }
    pendingUpdiscon = false;
}

void
ETrace::maybeEmitPbc()
{
    if (bpredCorrectCount >= 31) {
        // Spec: emit Format 0-0 with branch_count = (pbc - 31) as
        // soon as pbc reaches 31, so the decoder's mirror counter
        // tracks. No address in this path (branch_fmt = 00).
        emitBranchCountPacket(false, 0);
    }
}

void
ETrace::traceCommit(const DynInstPtr &dynInst)
{
    totalInstsCommitted++;

    if (!tracingActive) {
        if (totalInstsCommitted >= startTraceInst) {
            tracingActive = true;
            needsInitialSync = true;
        } else {
            return;
        }
    }

    Addr pc = dynInst->pcState().instAddr();
    gem5::ThreadContext *tc = cpu->getContext(0);
    uint8_t curPriv = tc->readMiscRegNoEffect(RiscvISA::MISCREG_PRV);

    // Filtering
    if (!passesFilter(pc, curPriv)) {
        if (!wasFiltered) {
            // Exiting the trace region: flush pending state with the
            // *last committed (traced) address*, then support(ended_rep).
            if (branchCount > 0) {
                emitBranchMapPacket(true, lastCommittedAddr);
            } else if (haveExpectedPC) {
                emitAddrOnlyPacket(lastCommittedAddr);
            }
            emitSupportPacket(QS_ENDED_REP);
            wasFiltered = true;
        }
        stats.numFilteredInsts++;

        // Still track expected PC for discontinuity detection on
        // re-entry.
        expectedNextPC = dynInst->pcState().as<RiscvISA::PCState>().npc();
        haveExpectedPC = true;
        lastCommittedAddr = pc;
        lastCommittedPriv = curPriv;
        lastPriv = curPriv;
        return;
    }

    if (wasFiltered) {
        wasFiltered = false;
        needsInitialSync = true;
    }

    // Detect exception/interrupt via PC discontinuity
    if (haveExpectedPC && pc != expectedNextPC) {
        uint64_t cause = 0;
        uint64_t tval = 0;
        bool isInterrupt = false;

        if (curPriv == RiscvISA::PRV_M) {
            cause = tc->readMiscRegNoEffect(RiscvISA::MISCREG_MCAUSE);
            tval = tc->readMiscRegNoEffect(RiscvISA::MISCREG_MTVAL);
        } else {
            cause = tc->readMiscRegNoEffect(RiscvISA::MISCREG_SCAUSE);
            tval = tc->readMiscRegNoEffect(RiscvISA::MISCREG_STVAL);
        }

        isInterrupt = (cause >> 63) & 1;
        uint64_t causeCode = cause & ((1ULL << 63) - 1);

        // Drain pending branches with the last-committed address of
        // the pre-trap execution (NOT the trap handler PC). Format 1
        // if we have branches to report, Format 2 otherwise.
        if (branchCount > 0 || lastCommittedAddr != lastReportedAddr) {
            emitAddressPacket(lastCommittedAddr);
        }

        // Format 3-1 `branch` bit: 0 iff the pre-trap committed
        // instruction was a taken branch, else 1 (spec convention,
        // matches Format 3-0). We tracked this at last-commit time.
        bool trapBranchBit = !lastCommittedWasTakenBranch;
        emitTrapPacket(pc, causeCode, tval, isInterrupt, curPriv,
                       trapBranchBit);
        needsInitialSync = false;
        instsSinceSync = 0;
    }

    // Emit support + initial sync on first traced instruction
    if (needsInitialSync) {
        emitSupportPacket(QS_NO_CHANGE);

        IType itype = classifyInstruction(dynInst);
        bool isBranch =
            (itype == ITYPE_TAKEN_BRANCH || itype == ITYPE_NTAKEN_BRANCH);
        bool taken = (itype == ITYPE_TAKEN_BRANCH);
        // Spec Format 3-0 `branch` bit (payload.adoc):
        //   0 iff address IS a branch instruction AND it was TAKEN
        //   1 otherwise (not a branch, or branch not taken)
        bool branchBit = !(isBranch && taken);
        emitSyncStartPacket(pc, curPriv, branchBit);
        needsInitialSync = false;
        instsSinceSync = 0;
    }

    // Detect privilege change (not via trap — the trap path handles
    // that above with priv already updated).
    if (curPriv != lastPriv && !needsInitialSync) {
        // Drain pending branches with the *last-priv address*, not
        // the new-priv address.
        if (branchCount > 0 || lastCommittedAddr != lastReportedAddr) {
            emitAddressPacket(lastCommittedAddr);
        }
        IType itype = classifyInstruction(dynInst);
        bool isBranch =
            (itype == ITYPE_TAKEN_BRANCH || itype == ITYPE_NTAKEN_BRANCH);
        bool taken = (itype == ITYPE_TAKEN_BRANCH);
        // See spec branch-bit convention comment above.
        emitSyncStartPacket(pc, curPriv, !(isBranch && taken));
        instsSinceSync = 0;
    }

    // Context tracking (ASID changes).
    if (contextWidth > 0) {
        uint64_t satp = tc->readMiscRegNoEffect(RiscvISA::MISCREG_SATP);
        uint64_t asid = (satp >> 44) & ((1ULL << 16) - 1);
        // Mask context to the configured width.
        uint64_t maskedAsid = (contextWidth >= 64) ? asid :
            (asid & ((1ULL << contextWidth) - 1));
        if (maskedAsid != lastContext) {
            if (branchCount > 0 || lastCommittedAddr != lastReportedAddr) {
                emitAddressPacket(lastCommittedAddr);
            }
            emitContextPacket(maskedAsid, curPriv);
            lastContext = maskedAsid;
        }
    }

    lastPriv = curPriv;
    instsSinceSync++;
    stats.totalInstsTraced++;

    IType itype = classifyInstruction(dynInst);

    DPRINTF(ETrace, "Commit 0x%08x itype=%d %s\n", pc, itype,
            dynInst->staticInst->disassemble(pc));

    switch (itype) {
        case ITYPE_NONE:
            break;

        case ITYPE_NTAKEN_BRANCH: {
            if (branchPrediction) {
                bool pred = predictBranch(pc);
                updatePredictor(pc, false);
                if (!pred) {  // Correctly predicted not-taken.
                    bpredCorrectCount++;
                    stats.numBpredCorrect++;
                    maybeEmitPbc();
                    break;
                }
            }
            branchMap |= (1U << branchCount);
            branchCount++;
            stats.numBranches++;
            if (branchCount >= maxBranchMapBits) {
                emitBranchMapPacket(false, 0);
            }
            break;
        }

        case ITYPE_TAKEN_BRANCH: {
            if (branchPrediction) {
                bool pred = predictBranch(pc);
                updatePredictor(pc, true);
                if (pred) {  // Correctly predicted taken.
                    bpredCorrectCount++;
                    stats.numBpredCorrect++;
                    maybeEmitPbc();
                    break;
                }
            }
            branchCount++;
            stats.numBranches++;
            if (branchCount >= maxBranchMapBits) {
                emitBranchMapPacket(false, 0);
            }
            break;
        }

        case ITYPE_INF_CALL:
            if (implicitReturn && callCounterMax > 0) {
                if (callCounter < callCounterMax) {
                    callCounter++;
                }
                // If already at max the paired return will be reported
                // explicitly (with irreport+irdepth).
            }
            // Inferable call — target derivable from opcode, no packet
            // needed. Branches stay accumulated; they will be drained
            // by the next address-carrying packet or by the 31-branch
            // overflow of Format 1 no-address.
            break;

        case ITYPE_INF_JUMP:
        case ITYPE_OTHER_INF:
            // Inferable jump — same as above, no packet needed.
            break;

        case ITYPE_UNINF_CALL: {
            if (implicitReturn && callCounterMax > 0) {
                if (callCounter < callCounterMax) {
                    callCounter++;
                }
            }
            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            markUpdiscon();
            uint32_t jtcIndex;
            if (jtCacheLookup(npc, jtcIndex)) {
                stats.numJtCacheHits++;
                emitJtcHitPacket(jtcIndex, npc);
            } else {
                // emitAddressPacket → Format 1 or Format 2, both
                // update the JTC internally.
                emitAddressPacket(npc);
            }
            break;
        }

        case ITYPE_RETURN: {
            if (implicitReturn && callCounterMax > 0 && callCounter > 0) {
                callCounter--;
                stats.numImplicitReturns++;
                // Inferable return — no packet needed. Branches
                // continue accumulating for the next address packet.
                break;
            }
            // Explicit return: reported "in the clear" because either
            // implicit-return mode is off, or the call counter is 0
            // (return goes beyond what the counter has tracked, e.g.
            // after an overflow'd call in a deeply-nested region).
            // Spec: set irreport and emit current counter in irdepth
            // (payload.adoc §sec:implicit-return).
            if (implicitReturn && callCounterMax > 0) {
                pendingExplicitReturn = true;
                pendingIrdepth = callCounter;
            }
            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            markUpdiscon();
            emitAddressPacket(npc);
            break;
        }

        case ITYPE_COROUTINE: {
            // Dec-then-inc: net-zero on the counter.
            if (implicitReturn && callCounterMax > 0 && callCounter > 0) {
                callCounter--;
            }
            if (implicitReturn && callCounterMax > 0) {
                if (callCounter < callCounterMax) {
                    callCounter++;
                }
            }
            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            markUpdiscon();
            emitAddressPacket(npc);
            break;
        }

        case ITYPE_UNINF_JUMP:
        case ITYPE_OTHER_UNINF: {
            // Check sequentially inferable jump
            if (isSeqInferableJump(dynInst)) {
                stats.numSijumpInferred++;
                // Inferable target — no address packet needed.
                break;
            }

            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            markUpdiscon();
            uint32_t jtcIndex;
            if (jtCacheLookup(npc, jtcIndex)) {
                stats.numJtCacheHits++;
                emitJtcHitPacket(jtcIndex, npc);
            } else {
                // emitAddressPacket → Format 1 or Format 2, both
                // update the JTC internally.
                emitAddressPacket(npc);
            }
            break;
        }

        case ITYPE_EXCEPT_RET: {
            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            markUpdiscon();
            emitAddressPacket(npc);
            break;
        }

        case ITYPE_EXCEPTION:
        case ITYPE_INTERRUPT:
            // Handled via the PC-discontinuity path at top of function.
            break;
    }

    // Update SI-jump tracking state
    if (sijump) {
        auto *riscvInst = static_cast<RiscvISA::RiscvStaticInst *>(
            dynInst->staticInst.get());
        auto machInst = riscvInst->machInst;
        bool isRvc =
            dynInst->pcState().as<RiscvISA::PCState>().compressed();
        prevInstPC = pc;
        prevInstIsRvc = isRvc;
        prevInstSize = isRvc ? 2 : 4;
        if (isRvc) {
            prevInstOpcode = machInst & 0x3;
            prevInstRvcFunct3 = (machInst >> 13) & 0x7;
            prevInstRd = (machInst >> 7) & 0x1F;
        } else {
            prevInstOpcode = machInst & 0x7F;
            prevInstRd = (machInst >> 7) & 0x1F;
            prevInstRvcFunct3 = 0;
        }
        havePrevInst = true;
    }

    // Update expected next PC for discontinuity detection
    expectedNextPC = dynInst->pcState().as<RiscvISA::PCState>().npc();
    haveExpectedPC = true;
    lastCommittedAddr = pc;
    lastCommittedPriv = curPriv;
    lastCommittedWasTakenBranch = (itype == ITYPE_TAKEN_BRANCH);

    // Periodic resync — 3-state FSM: state 1 counts, state 2 drains
    // pending branches, state 3 emits Format 3-0. We fuse states 2+3
    // in a single commit boundary (which is safe here because a probe
    // commit is atomic w.r.t. the tracer).
    if (instsSinceSync >= resyncPeriod) {
        if (branchCount > 0) {
            emitBranchMapPacket(true, pc);
        }
        // (No addr-only needed here — the sync-start packet carries
        // the address of the next-committed instruction anyway.)
        IType curIType = classifyInstruction(dynInst);
        bool isBranch = (curIType == ITYPE_TAKEN_BRANCH ||
                         curIType == ITYPE_NTAKEN_BRANCH);
        bool taken = (curIType == ITYPE_TAKEN_BRANCH);
        // See spec branch-bit convention comment above.
        emitSyncStartPacket(pc, curPriv, !(isBranch && taken));
        instsSinceSync = 0;
    }
}

void
ETrace::traceDataAccess(
    const std::pair<DynInstPtr, PacketPtr> &data)
{
    if (!dataTrace || !dataTraceStream)
        return;

    const DynInstPtr &dynInst = data.first;

    if (!tracingActive)
        return;

    // Gate through the same filter as the instruction stream so data
    // packets don't leak from a region whose instructions were
    // filtered out (they'd be undecodable without the I-trace
    // context anyway).
    Addr pc = dynInst->pcState().instAddr();
    gem5::ThreadContext *tc = cpu->getContext(0);
    uint8_t curPriv = tc->readMiscRegNoEffect(RiscvISA::MISCREG_PRV);
    if (!passesFilter(pc, curPriv))
        return;

    bool isLoad = dynInst->isLoad();
    bool isStore = dynInst->isStore();
    bool isAtomic = dynInst->isAtomic();
    if (!isLoad && !isStore && !isAtomic)
        return;

    // gem5's RISC-V ISA does not flag load-reserved/store-conditional
    // as IsAtomic -- only true read-modify-write AMOs (AMOADD, AMOSWAP,
    // etc.) get that flag; LoadReservedMicro overrides no flags beyond
    // the default IsLoad, and StoreCondMicro only sets IsStoreConditional
    // (see isa/formats/amo.isa, AtomicMemOpRMWConstructor vs the LR/SC
    // constructors). So LR/SC must be detected separately here, or they
    // silently fall through as plain, unmarked loads/stores below.
    bool isSC = dynInst->isStoreConditional();
    bool isLR = isLoad && !isSC &&
        dynamic_cast<RiscvISA::LoadReservedMicro *>(
            dynInst->staticInst.get()) != nullptr;
    Addr addr = dynInst->effAddr;
    uint32_t size = dynInst->effSize;
    emitDataPacket(dynInst, addr, size, isLoad, isStore, isAtomic,
                   isLR, isSC);
}

void
ETrace::emitDataPacket(const DynInstPtr &dynInst, Addr addr, uint32_t bytes,
                       bool isLoad, bool isStore, bool isAtomic,
                       bool isLR_param, bool isSC_param)
{
    // Compute size as log2 of transfer bytes per spec.
    uint32_t logSize = 0;
    uint32_t tmp = bytes;
    while (tmp > 1) { tmp >>= 1; logSize++; }

    ProtoMessage::ETraceDataPacket pkt;
    pkt.set_tick(curTick());
    pkt.set_size(logSize);

    // Look up per-size baseline; determine whether to emit full or delta.
    // First access of any given size in the stream is full-addr, so we
    // default to needing full when we haven't seen this size yet.
    uint8_t szKey = static_cast<uint8_t>(logSize);
    bool haveSeenSize = lastDataAddrBySize.count(szKey) > 0;
    auto needIt = dataSizeNeedsFullAddr.find(szKey);
    bool needFull = !haveSeenSize ||
        (needIt != dataSizeNeedsFullAddr.end() && needIt->second);
    Addr baseline = haveSeenSize ? lastDataAddrBySize[szKey] : 0;

    // Determine format code + diff bits.
    ProtoMessage::ETraceDataPacket::DataFormat fmt;
    uint32_t diffBits;

    bool aligned = (addr & (bytes - 1)) == 0;

    // Detect LR/SC via atomic classification if we're on an atomic
    // instruction (the caller passed isAtomic; refine using funct5).
    bool isLR = isLR_param;
    bool isSC = isSC_param;
    if (isAtomic) {
        bool lrFlag = false, scFlag = false;
        uint32_t sub = classifyAtomicSubtype(dynInst, lrFlag, scFlag);
        pkt.set_atomic_subtype(
            static_cast<ProtoMessage::ETraceDataPacket::AtomicSubtype>(sub));
        isLR = lrFlag;
        isSC = scFlag;
    }

    if (isAtomic && !isLR && !isSC) {
        fmt = ProtoMessage::ETraceDataPacket::ATOMIC;
    } else if (isLoad || isLR) {
        fmt = aligned ? ProtoMessage::ETraceDataPacket::LOAD_ALIGNED
                      : ProtoMessage::ETraceDataPacket::LOAD_UNALIGNED;
        if (isLR) pkt.set_is_lr(true);
    } else if (isStore || isSC) {
        fmt = aligned ? ProtoMessage::ETraceDataPacket::STORE_ALIGNED
                      : ProtoMessage::ETraceDataPacket::STORE_UNALIGNED;
        if (isSC) pkt.set_is_sc(true);
    } else {
        return;
    }
    pkt.set_format(fmt);

    // Address encoding per dataTraceMode:
    //   0 = addr + data
    //   1 = addr only
    //   2 = data only
    bool includeAddr = (dataTraceMode == 0 || dataTraceMode == 1);
    bool includeData = (dataTraceMode == 0 || dataTraceMode == 2);

    if (includeAddr) {
        if (needFull) {
            diffBits = DIFF_FULL_ADDR_FULL_DATA;
            pkt.set_address(static_cast<int64_t>(addr));
            dataSizeNeedsFullAddr[szKey] = false;
        } else {
            diffBits = DIFF_DELTA_ADDR_FULL_DATA;
            int64_t delta = static_cast<int64_t>(addr - baseline);
            pkt.set_address(delta);
        }
        lastDataAddrBySize[szKey] = addr;
    } else {
        // Data-only variant: diff bits 2-bit, 10 = full data, 11 = diff.
        diffBits = 0x02;
    }
    pkt.set_diff(diffBits);

    if (includeData && dynInst->memData) {
        pkt.set_data(dynInst->memData, bytes);
    }

    // Operand (rs2's pre-operation value) is mandatory on true AMO
    // packets per dataTracePayload.adoc -- src[1] is rs2 for every
    // AtomicMemOp instruction (src[0] is rs1, used only for the
    // effective address). Confirmed empirically at the
    // DataAccessComplete probe point: e.g. "amoswap_w a5,a5,(s1)"
    // reports src[0]==s1's value (the address), src[1]==the actual
    // swap operand. LR/SC have no operand field (they're represented
    // as plain LOAD/STORE forms, not the ATOMIC format) so this is
    // skipped for them.
    if (fmt == ProtoMessage::ETraceDataPacket::ATOMIC && includeData &&
            dynInst->numSrcRegs() >= 2) {
        RegVal operandVal = dynInst->getRegOperand(
            dynInst->staticInst.get(), 1);
        uint8_t operandBytes[sizeof(RegVal)];
        std::memcpy(operandBytes, &operandVal, sizeof(RegVal));
        pkt.set_operand(operandBytes, bytes);
    }

    dataTraceStream->write(pkt);
    stats.numDataTracePackets++;

    DPRINTF(ETrace,
            "DataTrace: addr=0x%08x size=%u(log=%u) load=%d store=%d "
            "atomic=%d diff=%u\n",
            addr, bytes, logSize, isLoad, isStore, isAtomic, diffBits);
}

void
ETrace::resetDataTraceBaselines()
{
    lastDataAddrBySize.clear();
    dataSizeNeedsFullAddr.clear();
    // Populate flags so the first data packet of any size after
    // sync emits a full address regardless of whether we've seen it.
    for (uint8_t sz = 0; sz < 8; sz++) {
        dataSizeNeedsFullAddr[sz] = true;
    }
}

// Format 3-0 (start / resync).
void
ETrace::emitSyncStartPacket(Addr addr, uint8_t priv, bool branch)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::FORMAT_3);
    pkt.set_subformat(ProtoMessage::ETracePacket::START);
    // Field order per spec: branch, priv, time, context, address.
    pkt.set_branch(branch);
    pkt.set_priv(priv);
    if (!notimeP && timeWidthP > 0) {
        pkt.set_time(curTick());
    }
    if (!nocontextP && contextWidth > 0) {
        pkt.set_context(lastContext);
    }
    // Spec: on-wire address is shifted right by iaddress_lsb_p; the
    // decoder shifts back. gem5 tracks byte addresses internally.
    pkt.set_address(addr >> iaddressLsbP);
    traceStream->write(pkt);

    lastReportedAddr = addr;
    resetBranchMap();

    // Reset optional mode state on sync (per spec).
    callCounter = 0;
    resetPredictor();
    jtCacheInvalidate();
    pendingUpdiscon = false;
    bpredCorrectCount = 0;
    if (dataTrace)
        resetDataTraceBaselines();

    stats.numSyncPackets++;
    stats.numPackets++;

    DPRINTF(ETrace, "Sync-start: addr=0x%08x priv=%d branch=%d\n",
            addr, priv, branch);
}

// Format 3-1 (trap).
void
ETrace::emitTrapPacket(Addr addr, uint64_t cause, uint64_t tval,
                       bool isInterrupt, uint8_t priv, bool branch)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::FORMAT_3);
    pkt.set_subformat(ProtoMessage::ETracePacket::TRAP);
    pkt.set_branch(branch);
    pkt.set_priv(priv);
    if (!notimeP && timeWidthP > 0) {
        pkt.set_time(curTick());
    }
    if (!nocontextP && contextWidth > 0) {
        pkt.set_context(lastContext);
    }
    pkt.set_cause(cause);
    pkt.set_interrupt(isInterrupt);

    bool inferable = isTrapAddrInferable(priv, cause, isInterrupt);
    // Spec Format 3-1 thaddr:
    //   thaddr=1: address IS the trap handler PC. In implicit-
    //             exception mode, address may be omitted (decoder
    //             derives handler from cause + mtvec/stvec).
    //   thaddr=0: address is EPC of the last-committed instruction
    //             (used when the trap follows an uninferable PC
    //             discontinuity, so the decoder needs the EPC to
    //             resume). gem5 always has the handler PC available
    //             at trap time so this path is never taken here.
    pkt.set_thaddr(true);
    if (!inferable) {
        pkt.set_address(addr >> iaddressLsbP);
    }
    // else: address omitted, decoder infers from cause + tvec.
    if (!isInterrupt) {
        // tval field: only present for exceptions per spec.
        pkt.set_tval(tval);
    }

    traceStream->write(pkt);

    lastReportedAddr = addr;
    resetBranchMap();
    callCounter = 0;
    resetPredictor();
    jtCacheInvalidate();
    pendingUpdiscon = false;
    bpredCorrectCount = 0;
    if (dataTrace)
        resetDataTraceBaselines();

    stats.numTrapPackets++;
    stats.numPackets++;

    DPRINTF(ETrace, "Trap packet: addr=0x%08x cause=%llu int=%d priv=%d "
            "inferable=%d\n", addr, cause, isInterrupt, priv, inferable);
}

// Format 1 (branch-map, with or without address).
void
ETrace::emitBranchMapPacket(bool withAddress, Addr addr)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::FORMAT_1);

    // Encode branches per spec's tapered set. Value 0 signals the
    // no-address 31-bit form; values 1/3/7/15/31 signal the with-
    // address form at the corresponding tapered map width.
    uint32_t branchesField;
    if (!withAddress) {
        panic_if(branchCount != maxBranchMapBits,
                 "Format 1 no-address form requires exactly 31 branches, "
                 "got %u", branchCount);
        branchesField = 0;
    } else {
        panic_if(branchCount == 0,
                 "Format 1 with-address form requires >=1 branches; "
                 "use Format 2 (emitAddrOnlyPacket) for address-only");
        if (branchCount == 1) branchesField = 1;
        else if (branchCount <= 3) branchesField = 3;
        else if (branchCount <= 7) branchesField = 7;
        else if (branchCount <= 15) branchesField = 15;
        else branchesField = 31;
    }
    pkt.set_branches(branchesField);
    pkt.set_branch_map(branchMap);

    if (withAddress) {
        // On-wire delta is computed between shifted addresses per spec.
        int64_t delta = static_cast<int64_t>(
            (addr >> iaddressLsbP) - (lastReportedAddr >> iaddressLsbP));
        pkt.set_saddress(delta);
        lastReportedAddr = addr;
        // Populate JTC on every address-carrying Format 1 emit — the
        // decoder mirrors on every such packet, so the encoder must
        // too or a future JTC hit will resolve to a stale target.
        jtCacheUpdate(addr);

        bool isExplicitReturn = pendingExplicitReturn;
        pendingExplicitReturn = false;
        bool notify_bit, updiscon_bit, irreport_bit;
        uint32_t irdepth_val = 0;
        computeDisambigBits(addr, isExplicitReturn,
                            notify_bit, updiscon_bit,
                            irreport_bit, irdepth_val);
        pkt.set_notify(notify_bit);
        pkt.set_updiscon(updiscon_bit);
        pkt.set_irreport(irreport_bit);
        pkt.set_irdepth(irdepth_val);
    }

    // Flush any accumulated correctly-predicted branch count into
    // this packet so the decoder can consume it (Format 1 in
    // predicted-mode may carry a branch_count field).
    if (bpredCorrectCount > 0) {
        pkt.set_branch_count(bpredCorrectCount);
        bpredCorrectCount = 0;
    }

    traceStream->write(pkt);

    resetBranchMap();
    stats.numBranchMapPackets++;
    stats.numPackets++;

    DPRINTF(ETrace,
            "Format-1 packet: withAddr=%d addr=0x%08x map=0x%x count=%d "
            "branches_field=%u\n",
            withAddress, addr, branchMap, branchCount, branchesField);
}

void
ETrace::emitAddressPacket(Addr addr)
{
    if (branchCount > 0) {
        emitBranchMapPacket(true, addr);
    } else {
        emitAddrOnlyPacket(addr);
    }
}

// Format 2 (address only, no branch map).
void
ETrace::emitAddrOnlyPacket(Addr addr)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::FORMAT_2);
    int64_t delta = static_cast<int64_t>(
        (addr >> iaddressLsbP) - (lastReportedAddr >> iaddressLsbP));
    pkt.set_saddress(delta);
    // Populate JTC (see emitBranchMapPacket comment).
    jtCacheUpdate(addr);

    bool isExplicitReturn = pendingExplicitReturn;
    pendingExplicitReturn = false;
    bool notify_bit, updiscon_bit, irreport_bit;
    uint32_t irdepth_val = 0;
    computeDisambigBits(addr, isExplicitReturn,
                        notify_bit, updiscon_bit,
                        irreport_bit, irdepth_val);
    pkt.set_notify(notify_bit);
    pkt.set_updiscon(updiscon_bit);
    pkt.set_irreport(irreport_bit);
    pkt.set_irdepth(irdepth_val);

    traceStream->write(pkt);

    lastReportedAddr = addr;
    stats.numAddrOnlyPackets++;
    stats.numPackets++;

    DPRINTF(ETrace,
            "Format-2 packet: addr=0x%08x delta=%lld irdepth=%u\n",
            addr, static_cast<long long>(delta), irdepth_val);
}

// Format 0-0 (correctly-predicted branch count).
void
ETrace::emitBranchCountPacket(bool withAddress, Addr addr, bool mispredTaken)
{
    if (bpredCorrectCount < 31) return;
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::FORMAT_0);
    pkt.set_f0_subformat(ProtoMessage::ETracePacket::F0_BRANCH_COUNT);
    // Spec encodes as (pbc - 31).
    pkt.set_branch_count(bpredCorrectCount - 31);
    // branch_fmt per spec: 00 no-address; 10 address (mispred taken);
    // 11 address (mispred not-taken).
    uint32_t bf;
    if (!withAddress)      bf = 0x0;
    else if (mispredTaken) bf = 0x2;
    else                   bf = 0x3;
    pkt.set_branch_fmt(bf);
    if (withAddress) {
        int64_t delta = static_cast<int64_t>(
            (addr >> iaddressLsbP) - (lastReportedAddr >> iaddressLsbP));
        pkt.set_saddress(delta);
        lastReportedAddr = addr;
        bool isExplicitReturn = pendingExplicitReturn;
        pendingExplicitReturn = false;
        bool notify_bit, updiscon_bit, irreport_bit;
        uint32_t irdepth_val = 0;
        computeDisambigBits(addr, isExplicitReturn,
                            notify_bit, updiscon_bit,
                            irreport_bit, irdepth_val);
        pkt.set_notify(notify_bit);
        pkt.set_updiscon(updiscon_bit);
        pkt.set_irreport(irreport_bit);
        pkt.set_irdepth(irdepth_val);
    }
    traceStream->write(pkt);
    bpredCorrectCount = 0;
    stats.numBpcFlushes++;
    stats.numPackets++;
}

// Format 0-1 (JTC hit).
void
ETrace::emitJtcHitPacket(uint32_t index, Addr target)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::FORMAT_0);
    pkt.set_f0_subformat(ProtoMessage::ETracePacket::F0_JTC);
    pkt.set_jtc_index(index);

    // Branch map (if any) piggybacks per spec Format 0-1.
    uint32_t branchesField;
    if (branchCount == 0) branchesField = 0;
    else if (branchCount == 1) branchesField = 1;
    else if (branchCount <= 3) branchesField = 3;
    else if (branchCount <= 7) branchesField = 7;
    else if (branchCount <= 15) branchesField = 15;
    else branchesField = 31;
    pkt.set_branches(branchesField);
    if (branchCount > 0) {
        pkt.set_branch_map(branchMap);
    }
    // Format 0-1 carries irreport (spec: chained off branch_map MSB
    // or branches MSB when no map). It does not carry notify/updiscon
    // as separate fields — those are subsumed by the JTC lookup itself
    // being an implicit ack of the last uninferable discontinuity.
    // Consume the pending flags here so they don't leak into the next
    // Format 3 packet.
    bool isExplicitReturn = pendingExplicitReturn;
    pendingExplicitReturn = false;
    pendingUpdiscon = false;
    pkt.set_irreport(isExplicitReturn);
    pkt.set_irdepth(isExplicitReturn ? pendingIrdepth : 0);

    traceStream->write(pkt);
    // Decoder resets its reconstructed_addr to the JTC target on
    // this packet; encoder must do the same for lastReportedAddr so
    // subsequent saddress deltas are computed from the same anchor.
    lastReportedAddr = target;
    resetBranchMap();
    stats.numBranchMapPackets++;
    stats.numPackets++;
}

// Format 3-2 (context change). No address, no branch bit.
void
ETrace::emitContextPacket(uint64_t context, uint8_t priv)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::FORMAT_3);
    pkt.set_subformat(ProtoMessage::ETracePacket::CONTEXT);
    pkt.set_priv(priv);
    if (!notimeP && timeWidthP > 0) {
        pkt.set_time(curTick());
    }
    pkt.set_context(context);
    traceStream->write(pkt);

    // Spec: context change is a sync-class event; data-trace per-size
    // baselines must be invalidated so the next data packet emits a
    // full address.
    if (dataTrace)
        resetDataTraceBaselines();

    stats.numContextPackets++;
    stats.numPackets++;

    DPRINTF(ETrace, "Context packet: context=0x%llx priv=%d\n",
            (unsigned long long)context, priv);
}

// Format 3-3 (support).
void
ETrace::emitSupportPacket(QualStatus qual)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::FORMAT_3);
    pkt.set_subformat(ProtoMessage::ETracePacket::SUPPORT);
    pkt.set_ienable(true);
    pkt.set_encoder_mode(0);
    pkt.set_qual_status(
        static_cast<ProtoMessage::ETracePacket::QualStatus>(qual));

    uint32_t ioptions = 0;
    if (implicitReturn && callCounterMax > 0) ioptions |= (1u << 0);
    if (branchPrediction && bpredSizeP > 0)   ioptions |= (1u << 1);
    if (jumpTargetCache && cacheSizeP > 0)    ioptions |= (1u << 2);
    if (sijump)                                ioptions |= (1u << 3);
    if (implicitException)                     ioptions |= (1u << 4);
    pkt.set_ioptions(ioptions);

    pkt.set_denable(dataTrace);
    pkt.set_dloss(false);  // gem5 never drops data-trace packets.
    uint32_t doptions = 0;
    if (dataTraceMode == 1) doptions |= (1u << 0);
    if (dataTraceMode == 2) doptions |= (1u << 1);
    pkt.set_doptions(doptions);

    traceStream->write(pkt);
    stats.numSupportPackets++;
    stats.numPackets++;

    DPRINTF(ETrace, "Support packet: qual=0x%x ioptions=0x%x denable=%d\n",
            qual, ioptions, dataTrace ? 1 : 0);
}

void
ETrace::resetBranchMap()
{
    branchMap = 0;
    branchCount = 0;
}

void
ETrace::flushTrace()
{
    // Drain any pending Format-1 with the last committed address.
    if (branchCount > 0) {
        emitBranchMapPacket(true, lastCommittedAddr);
    } else if (haveExpectedPC && lastCommittedAddr != lastReportedAddr) {
        emitAddrOnlyPacket(lastCommittedAddr);
    }
    // Emit a Format 0-0 for any residual predicted-branch count.
    if (bpredCorrectCount >= 31) {
        emitBranchCountPacket(false, 0);
    }

    // Support(ended_rep): clean end of trace.
    emitSupportPacket(QS_ENDED_REP);

    DPRINTF(ETrace, "Flushing trace: %llu packets, %llu instructions\n",
            stats.numPackets.value(), stats.totalInstsTraced.value());

    delete traceStream;
    traceStream = nullptr;

    if (dataTraceStream) {
        delete dataTraceStream;
        dataTraceStream = nullptr;
    }
}

ETrace::ETraceStats::ETraceStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(numPackets, statistics::units::Count::get(),
               "Total E-Trace packets emitted"),
      ADD_STAT(numSyncPackets, statistics::units::Count::get(),
               "Sync packets emitted"),
      ADD_STAT(numTrapPackets, statistics::units::Count::get(),
               "Trap packets emitted"),
      ADD_STAT(numBranchMapPackets, statistics::units::Count::get(),
               "Branch map packets emitted"),
      ADD_STAT(numBranches, statistics::units::Count::get(),
               "Branches traced"),
      ADD_STAT(totalInstsTraced, statistics::units::Count::get(),
               "Total instructions traced"),
      ADD_STAT(numAddrOnlyPackets, statistics::units::Count::get(),
               "Address-only packets emitted"),
      ADD_STAT(numSupportPackets, statistics::units::Count::get(),
               "Support packets emitted"),
      ADD_STAT(numContextPackets, statistics::units::Count::get(),
               "Context packets emitted"),
      ADD_STAT(numDataTracePackets, statistics::units::Count::get(),
               "Data trace packets emitted"),
      ADD_STAT(numImplicitReturns, statistics::units::Count::get(),
               "Returns inferred via call counter"),
      ADD_STAT(numBpredCorrect, statistics::units::Count::get(),
               "Correctly predicted branches omitted"),
      ADD_STAT(numBpcFlushes, statistics::units::Count::get(),
               "Format 0-0 predicted-branch-count flushes"),
      ADD_STAT(numJtCacheHits, statistics::units::Count::get(),
               "Jump target cache hits"),
      ADD_STAT(numSijumpInferred, statistics::units::Count::get(),
               "Sequentially inferable jumps detected"),
      ADD_STAT(numFilteredInsts, statistics::units::Count::get(),
               "Instructions filtered out")
{}

} // namespace o3
} // namespace gem5
