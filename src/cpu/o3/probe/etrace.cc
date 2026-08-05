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
      sijump(params.sijump),
      prevInstPC(0),
      prevInstOpcode(0),
      prevInstRd(0),
      prevInstSize(0),
      havePrevInst(false),
      dataTrace(params.dataTrace),
      dataTraceMode(params.dataTraceMode),
      dataTraceStream(nullptr),
      lastDataAddr(0),
      contextWidth(params.contextWidth),
      lastContext(0),
      filterPrivMask(params.filterPriv),
      filterAddrStart(params.filterAddrStart),
      filterAddrEnd(params.filterAddrEnd),
      filterAddrEnabled(params.filterAddrStart != 0 ||
                        params.filterAddrEnd != 0),
      wasFiltered(false),
      pendingUpdiscon(false),
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
    header.set_tick_freq(sim_clock::Frequency);
    header.set_arch_width(64);
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

    if (dynInst->isCall() && dynInst->isReturn()) {
        return ITYPE_COROUTINE;
    }

    if (dynInst->isCall()) {
        return dynInst->isDirectCtrl() ? ITYPE_INF_CALL : ITYPE_UNINF_CALL;
    }

    if (dynInst->isReturn()) {
        return ITYPE_RETURN;
    }

    if (dynInst->isControl()) {
        if (dynInst->isDirectCtrl()) {
            return ITYPE_INF_JUMP2;
        }
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
    if (mode == 0) {
        // Direct mode: all traps go to BASE — always inferable
        return true;
    } else if (mode == 1) {
        // Vectored mode: interrupts go to BASE + 4*cause,
        // exceptions go to BASE — both inferable
        return true;
    }

    return false;
}

bool
ETrace::isSeqInferableJump(const DynInstPtr &dynInst)
{
    if (!sijump || !havePrevInst)
        return false;

    auto *riscvInst = static_cast<RiscvISA::RiscvStaticInst *>(
        dynInst->staticInst.get());
    auto machInst = riscvInst->machInst;

    uint8_t curOpcode = machInst & 0x7F;
    uint8_t curRs1 = (machInst >> 15) & 0x1F;
    Addr curPC = dynInst->pcState().instAddr();

    // Current must be JALR (opcode 0x67)
    if (curOpcode != 0x67)
        return false;

    // Previous must be LUI (0x37) or AUIPC (0x17)
    if (prevInstOpcode != 0x37 && prevInstOpcode != 0x17)
        return false;

    // Previous rd must match current rs1
    if (prevInstRd != curRs1)
        return false;

    // Must be sequential (previous PC + previous size == current PC)
    if (prevInstPC + prevInstSize != curPC)
        return false;

    return true;
}

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

    // 2-bit saturating counter state machine per E-Trace spec:
    // 00 (SNT): taken→01, not-taken stays 00
    // 01 (WNT): taken→11, not-taken→00
    // 10 (ST):  taken stays 11 (→11), not-taken→00
    // 11 (WT):  taken→11, not-taken→10
    switch (state) {
      case 0x00: // Strongly not-taken
        bpredTable[index] = taken ? 0x01 : 0x00;
        break;
      case 0x01: // Weakly not-taken
        bpredTable[index] = taken ? 0x03 : 0x00;
        break;
      case 0x02: // Strongly taken (mapped to WT per spec)
        bpredTable[index] = taken ? 0x03 : 0x00;
        break;
      case 0x03: // Weakly taken
        bpredTable[index] = taken ? 0x03 : 0x02;
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
ETrace::jtCacheLookup(Addr pc, Addr target, uint32_t &index)
{
    if (!jumpTargetCache || jtCache.empty())
        return false;

    index = (pc >> 1) & ((1U << cacheSizeP) - 1);
    return jtCache[index] == target;
}

void
ETrace::jtCacheUpdate(Addr pc, Addr target)
{
    if (!jumpTargetCache || jtCache.empty())
        return;

    uint32_t index = (pc >> 1) & ((1U << cacheSizeP) - 1);
    jtCache[index] = target;
}

void
ETrace::jtCacheInvalidate()
{
    for (auto &entry : jtCache) {
        entry = 0;
    }
}

uint32_t
ETrace::classifyAtomicOp(const DynInstPtr &dynInst)
{
    auto *riscvInst = static_cast<RiscvISA::RiscvStaticInst *>(
        dynInst->staticInst.get());
    auto machInst = riscvInst->machInst;
    uint8_t amofunct = (machInst >> 27) & 0x1F;

    switch (amofunct) {
      case 0x01: return ProtoMessage::ETraceDataPacket::SWAP;
      case 0x00: return ProtoMessage::ETraceDataPacket::ADD;
      case 0x0C: return ProtoMessage::ETraceDataPacket::AND;
      case 0x08: return ProtoMessage::ETraceDataPacket::OR;
      case 0x04: return ProtoMessage::ETraceDataPacket::XOR;
      case 0x14: return ProtoMessage::ETraceDataPacket::MAX;
      case 0x10: return ProtoMessage::ETraceDataPacket::MIN;
      case 0x1C: return ProtoMessage::ETraceDataPacket::MAXU;
      case 0x18: return ProtoMessage::ETraceDataPacket::MINU;
      case 0x02: return ProtoMessage::ETraceDataPacket::LR;
      case 0x03: return ProtoMessage::ETraceDataPacket::SC;
      default:   return ProtoMessage::ETraceDataPacket::SWAP;
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
            if (branchCount > 0) {
                emitBranchMapPacket(true, lastCommittedAddr);
            }
            wasFiltered = true;
        }
        stats.numFilteredInsts++;

        // Still track expected PC for discontinuity detection on re-entry
        expectedNextPC = dynInst->pcState().as<RiscvISA::PCState>().npc();
        haveExpectedPC = true;
        lastCommittedAddr = pc;
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

        if (branchCount > 0) {
            emitBranchMapPacket(true, lastCommittedAddr);
        }

        emitTrapPacket(pc, causeCode, tval, isInterrupt, curPriv);
        needsInitialSync = false;
        instsSinceSync = 0;
    }

    // Emit support + initial sync on first traced instruction
    if (needsInitialSync) {
        emitSupportPacket(true);

        IType itype = classifyInstruction(dynInst);
        bool isBranch =
            (itype == ITYPE_TAKEN_BRANCH || itype == ITYPE_NTAKEN_BRANCH);
        bool taken = (itype == ITYPE_TAKEN_BRANCH);
        emitSyncPacket(pc, curPriv, isBranch, taken);
        needsInitialSync = false;
        instsSinceSync = 0;
    }

    // Detect privilege change
    if (curPriv != lastPriv && !needsInitialSync) {
        if (branchCount > 0) {
            emitBranchMapPacket(true, pc);
        }
        IType itype = classifyInstruction(dynInst);
        bool isBranch =
            (itype == ITYPE_TAKEN_BRANCH || itype == ITYPE_NTAKEN_BRANCH);
        bool taken = (itype == ITYPE_TAKEN_BRANCH);
        emitSyncPacket(pc, curPriv, isBranch, taken);
        instsSinceSync = 0;
    }

    // Context tracking (ASID changes)
    if (contextWidth > 0) {
        uint64_t satp = tc->readMiscRegNoEffect(RiscvISA::MISCREG_SATP);
        uint64_t asid = (satp >> 44) & ((1ULL << 16) - 1);
        if (asid != lastContext) {
            if (branchCount > 0) {
                emitBranchMapPacket(true, pc);
            }
            emitContextPacket(asid);
            lastContext = asid;
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
            bool predicted = false;
            if (branchPrediction) {
                predicted = !predictBranch(pc);
                updatePredictor(pc, false);
                if (predicted) {
                    bpredCorrectCount++;
                    stats.numBpredCorrect++;
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
            bool predicted = false;
            if (branchPrediction) {
                predicted = predictBranch(pc);
                updatePredictor(pc, true);
                if (predicted) {
                    bpredCorrectCount++;
                    stats.numBpredCorrect++;
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
                if (callCounter < callCounterMax)
                    callCounter++;
            }
            emitBranchMapPacket(false, 0);
            break;

        case ITYPE_INF_JUMP2:
        case ITYPE_OTHER_INF:
            emitBranchMapPacket(false, 0);
            break;

        case ITYPE_UNINF_CALL: {
            if (implicitReturn && callCounterMax > 0) {
                if (callCounter < callCounterMax)
                    callCounter++;
            }
            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            uint32_t jtcIndex;
            if (jtCacheLookup(pc, npc, jtcIndex)) {
                stats.numJtCacheHits++;
                ProtoMessage::ETracePacket pkt;
                pkt.set_tick(curTick());
                pkt.set_format(ProtoMessage::ETracePacket::BRANCH_MAP_ADDR);
                pkt.set_branch_map(branchMap);
                pkt.set_branch_count(branchCount);
                pkt.set_jtc_index(jtcIndex);
                if (bpredCorrectCount > 0) {
                    pkt.set_branch_pred_count(bpredCorrectCount);
                    bpredCorrectCount = 0;
                }
                traceStream->write(pkt);
                resetBranchMap();
                stats.numBranchMapPackets++;
                stats.numPackets++;
            } else {
                jtCacheUpdate(pc, npc);
                emitBranchMapPacket(true, npc);
            }
            break;
        }

        case ITYPE_RETURN: {
            if (implicitReturn && callCounterMax > 0 && callCounter > 0) {
                callCounter--;
                stats.numImplicitReturns++;
                emitBranchMapPacket(false, 0);
                break;
            }
            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            emitBranchMapPacket(true, npc);
            break;
        }

        case ITYPE_COROUTINE: {
            if (implicitReturn && callCounterMax > 0 && callCounter > 0) {
                callCounter--;
            }
            if (implicitReturn && callCounterMax > 0) {
                if (callCounter < callCounterMax)
                    callCounter++;
            }
            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            emitBranchMapPacket(true, npc);
            break;
        }

        case ITYPE_UNINF_JUMP:
        case ITYPE_UNINF_JUMP2:
        case ITYPE_OTHER_UNINF: {
            // Check sequentially inferable jump
            if (isSeqInferableJump(dynInst)) {
                stats.numSijumpInferred++;
                emitBranchMapPacket(false, 0);
                break;
            }

            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            uint32_t jtcIndex;
            if (jtCacheLookup(pc, npc, jtcIndex)) {
                stats.numJtCacheHits++;
                ProtoMessage::ETracePacket pkt;
                pkt.set_tick(curTick());
                pkt.set_format(ProtoMessage::ETracePacket::BRANCH_MAP_ADDR);
                pkt.set_branch_map(branchMap);
                pkt.set_branch_count(branchCount);
                pkt.set_jtc_index(jtcIndex);
                if (bpredCorrectCount > 0) {
                    pkt.set_branch_pred_count(bpredCorrectCount);
                    bpredCorrectCount = 0;
                }
                traceStream->write(pkt);
                resetBranchMap();
                stats.numBranchMapPackets++;
                stats.numPackets++;
            } else {
                jtCacheUpdate(pc, npc);
                emitBranchMapPacket(true, npc);
            }
            break;
        }

        case ITYPE_EXCEPT_RET: {
            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            emitBranchMapPacket(true, npc);
            break;
        }

        case ITYPE_EXCEPTION:
        case ITYPE_INTERRUPT:
            break;
    }

    // Update SI-jump tracking state
    if (sijump) {
        auto *riscvInst = static_cast<RiscvISA::RiscvStaticInst *>(
            dynInst->staticInst.get());
        auto machInst = riscvInst->machInst;
        prevInstPC = pc;
        prevInstOpcode = machInst & 0x7F;
        prevInstRd = (machInst >> 7) & 0x1F;
        prevInstSize =
            dynInst->pcState().as<RiscvISA::PCState>().compressed() ? 2 : 4;
        havePrevInst = true;
    }

    // Update expected next PC for discontinuity detection
    expectedNextPC = dynInst->pcState().as<RiscvISA::PCState>().npc();
    haveExpectedPC = true;
    lastCommittedAddr = pc;

    // Periodic resync
    if (instsSinceSync >= resyncPeriod) {
        if (branchCount > 0) {
            emitBranchMapPacket(true, pc);
        }
        IType curIType = classifyInstruction(dynInst);
        bool isBranch = (curIType == ITYPE_TAKEN_BRANCH ||
                         curIType == ITYPE_NTAKEN_BRANCH);
        bool taken = (curIType == ITYPE_TAKEN_BRANCH);
        emitSyncPacket(pc, curPriv, isBranch, taken);
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

    ProtoMessage::ETraceDataPacket pkt;
    pkt.set_tick(curTick());

    bool isLoad = dynInst->isLoad();
    bool isStore = dynInst->isStore();
    bool isAtomic = dynInst->isAtomic();

    Addr addr = dynInst->effAddr;
    uint32_t size = dynInst->effSize;

    if (isAtomic) {
        pkt.set_format(ProtoMessage::ETraceDataPacket::ATOMIC);
        pkt.set_atomic_subtype(
            static_cast<ProtoMessage::ETraceDataPacket::AtomicSubtype>(
                classifyAtomicOp(dynInst)));
        pkt.set_address(addr - lastDataAddr);
        lastDataAddr = addr;
        pkt.set_size(size);
        if (dynInst->memData && (dataTraceMode == 0 || dataTraceMode == 2)) {
            pkt.set_data(dynInst->memData, size);
            pkt.set_data_len(size);
        }
    } else if (isLoad) {
        switch (dataTraceMode) {
          case 0:
            pkt.set_format(
                ProtoMessage::ETraceDataPacket::LOAD_ADDR_DATA);
            pkt.set_address(addr - lastDataAddr);
            lastDataAddr = addr;
            if (dynInst->memData) {
                pkt.set_data(dynInst->memData, size);
                pkt.set_data_len(size);
            }
            break;
          case 1:
            pkt.set_format(
                ProtoMessage::ETraceDataPacket::LOAD_ADDR_ONLY);
            pkt.set_address(addr - lastDataAddr);
            lastDataAddr = addr;
            break;
          case 2:
            pkt.set_format(
                ProtoMessage::ETraceDataPacket::LOAD_DATA_ONLY);
            if (dynInst->memData) {
                pkt.set_data(dynInst->memData, size);
                pkt.set_data_len(size);
            }
            break;
        }
    } else if (isStore) {
        switch (dataTraceMode) {
          case 0:
            pkt.set_format(
                ProtoMessage::ETraceDataPacket::STORE_ADDR_DATA);
            pkt.set_address(addr - lastDataAddr);
            lastDataAddr = addr;
            if (dynInst->memData) {
                pkt.set_data(dynInst->memData, size);
                pkt.set_data_len(size);
            }
            break;
          case 1:
            pkt.set_format(
                ProtoMessage::ETraceDataPacket::STORE_ADDR_ONLY);
            pkt.set_address(addr - lastDataAddr);
            lastDataAddr = addr;
            break;
          case 2:
            pkt.set_format(
                ProtoMessage::ETraceDataPacket::STORE_DATA_ONLY);
            if (dynInst->memData) {
                pkt.set_data(dynInst->memData, size);
                pkt.set_data_len(size);
            }
            break;
        }
    } else {
        return;
    }

    pkt.set_size(size);
    dataTraceStream->write(pkt);
    stats.numDataTracePackets++;

    DPRINTF(ETrace, "DataTrace: addr=0x%08x size=%u load=%d store=%d\n",
            addr, size, isLoad, isStore);
}

void
ETrace::emitSyncPacket(Addr addr, uint8_t priv, bool isBranch, bool taken)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::SYNC);
    pkt.set_subformat(ProtoMessage::ETracePacket::START);
    pkt.set_address(addr);
    pkt.set_priv(priv);
    pkt.set_branch_map(isBranch && taken ? 0 : 1);
    pkt.set_branch_count(1);
    traceStream->write(pkt);

    lastReportedAddr = addr;
    resetBranchMap();

    // Reset optional mode state on sync
    callCounter = 0;
    resetPredictor();
    jtCacheInvalidate();
    pendingUpdiscon = false;
    bpredCorrectCount = 0;

    stats.numSyncPackets++;
    stats.numPackets++;

    DPRINTF(ETrace, "Sync packet: addr=0x%08x priv=%d\n", addr, priv);
}

void
ETrace::emitTrapPacket(Addr addr, uint64_t cause, uint64_t tval,
                       bool isInterrupt, uint8_t priv)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::SYNC);
    pkt.set_subformat(ProtoMessage::ETracePacket::TRAP);
    pkt.set_priv(priv);
    pkt.set_cause(cause);
    pkt.set_tval(tval);
    pkt.set_interrupt(isInterrupt);

    bool inferable = isTrapAddrInferable(priv, cause, isInterrupt);
    pkt.set_thaddr(!inferable);
    if (!inferable) {
        pkt.set_address(addr);
    }

    traceStream->write(pkt);

    lastReportedAddr = addr;
    resetBranchMap();
    stats.numTrapPackets++;
    stats.numPackets++;

    DPRINTF(ETrace, "Trap packet: addr=0x%08x cause=%llu int=%d priv=%d "
            "thaddr=%d\n", addr, cause, isInterrupt, priv, !inferable);
}

void
ETrace::emitBranchMapPacket(bool withAddress, Addr addr)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());

    if (withAddress) {
        pkt.set_format(ProtoMessage::ETracePacket::BRANCH_MAP_ADDR);
        int64_t delta = static_cast<int64_t>(addr - lastReportedAddr);
        pkt.set_saddress(delta);
        pkt.set_address(addr);
        lastReportedAddr = addr;
    } else {
        pkt.set_format(ProtoMessage::ETracePacket::BRANCH_MAP);
    }

    pkt.set_branch_map(branchMap);
    pkt.set_branch_count(branchCount);

    if (bpredCorrectCount > 0) {
        pkt.set_branch_pred_count(bpredCorrectCount);
        bpredCorrectCount = 0;
    }

    traceStream->write(pkt);

    resetBranchMap();
    stats.numBranchMapPackets++;
    stats.numPackets++;

    DPRINTF(ETrace,
            "BranchMap packet: withAddr=%d addr=0x%08x "
            "map=0x%x count=%d\n",
            withAddress, addr, branchMap, branchCount);
}

void
ETrace::emitSupportPacket(bool isStart)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::SYNC);
    pkt.set_subformat(ProtoMessage::ETracePacket::SUPPORT);

    uint32_t ienable = 0x01;
    pkt.set_ienable(ienable);
    pkt.set_encoder_mode(0);

    pkt.set_qual_status(isStart ? 0x01 : 0x02);

    uint32_t ioptions = 0;
    if (implicitReturn && callCounterMax > 0)
        ioptions |= (1 << 0);
    if (branchPrediction && bpredSizeP > 0)
        ioptions |= (1 << 1);
    if (jumpTargetCache && cacheSizeP > 0)
        ioptions |= (1 << 2);
    if (sijump)
        ioptions |= (1 << 3);
    if (implicitException)
        ioptions |= (1 << 4);
    pkt.set_ioptions(ioptions);

    if (dataTrace) {
        pkt.set_denable(0x01);
        pkt.set_doptions(dataTraceMode);
    } else {
        pkt.set_denable(0x00);
    }

    traceStream->write(pkt);
    stats.numSupportPackets++;
    stats.numPackets++;

    DPRINTF(ETrace, "Support packet: isStart=%d ioptions=0x%x denable=%d\n",
            isStart, ioptions, dataTrace ? 1 : 0);
}

void
ETrace::emitAddrOnlyPacket(Addr addr, bool notify, bool updiscon,
                           bool irreport, uint32_t irdepth)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::ADDR_ONLY);
    int64_t delta = static_cast<int64_t>(addr - lastReportedAddr);
    pkt.set_saddress(delta);
    pkt.set_address(addr);
    pkt.set_notify(notify);
    pkt.set_updiscon(updiscon);
    pkt.set_irreport(irreport);
    pkt.set_irdepth(irdepth);
    traceStream->write(pkt);

    lastReportedAddr = addr;
    stats.numAddrOnlyPackets++;
    stats.numPackets++;

    DPRINTF(ETrace, "AddrOnly packet: addr=0x%08x notify=%d updiscon=%d "
            "irreport=%d irdepth=%d\n",
            addr, notify, updiscon, irreport, irdepth);
}

void
ETrace::emitContextPacket(uint64_t context)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::SYNC);
    pkt.set_subformat(ProtoMessage::ETracePacket::CONTEXT);
    pkt.set_context(context);
    traceStream->write(pkt);

    stats.numContextPackets++;
    stats.numPackets++;

    DPRINTF(ETrace, "Context packet: context=0x%x\n", context);
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
    if (branchCount > 0) {
        emitBranchMapPacket(true, lastCommittedAddr);
    }

    emitSupportPacket(false);

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
      ADD_STAT(numJtCacheHits, statistics::units::Count::get(),
               "Jump target cache hits"),
      ADD_STAT(numSijumpInferred, statistics::units::Count::get(),
               "Sequentially inferable jumps detected"),
      ADD_STAT(numFilteredInsts, statistics::units::Count::get(),
               "Instructions filtered out")
{}

} // namespace o3
} // namespace gem5
