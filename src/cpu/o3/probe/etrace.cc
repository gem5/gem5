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

    registerExitCallback([this]() { flushTrace(); });
}

ETrace::~ETrace()
{
    delete traceStream;
}

void
ETrace::regProbeListeners()
{
    typedef ProbeListenerArg<ETrace, DynInstPtr> DynInstListener;
    connectListener<DynInstListener>(this, "Commit", &ETrace::traceCommit);
}

ETrace::IType
ETrace::classifyInstruction(const DynInstPtr &dynInst)
{
    // Exception return: sret/mret are NonSpeculative returns in RISC-V
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
        // Co-routine swap: both call and return (e.g. jalr x1, x5)
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

        // MSB of cause indicates interrupt vs exception
        isInterrupt = (cause >> 63) & 1;
        uint64_t causeCode = cause & ((1ULL << 63) - 1);

        // Emit any pending branch map before the trap packet
        if (branchCount > 0) {
            emitBranchMapPacket(true, lastCommittedAddr);
        }

        emitTrapPacket(pc, causeCode, tval, isInterrupt, curPriv);
        needsInitialSync = false;
        instsSinceSync = 0;
    }

    // Emit initial sync packet on first traced instruction
    if (needsInitialSync) {
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

    lastPriv = curPriv;
    instsSinceSync++;
    stats.totalInstsTraced++;

    IType itype = classifyInstruction(dynInst);

    DPRINTF(ETrace, "Commit 0x%08x itype=%d %s\n", pc, itype,
            dynInst->staticInst->disassemble(pc));

    switch (itype) {
        case ITYPE_NONE:
            break;

        case ITYPE_NTAKEN_BRANCH:
            branchMap |= (1U << branchCount);
            branchCount++;
            stats.numBranches++;
            if (branchCount >= maxBranchMapBits) {
                emitBranchMapPacket(false, 0);
            }
            break;

        case ITYPE_TAKEN_BRANCH:
            // taken = 0 bit (already zero in branchMap)
            branchCount++;
            stats.numBranches++;
            if (branchCount >= maxBranchMapBits) {
                emitBranchMapPacket(false, 0);
            }
            break;

        case ITYPE_INF_CALL:
        case ITYPE_INF_JUMP2:
        case ITYPE_OTHER_INF:
            emitBranchMapPacket(false, 0);
            break;

        case ITYPE_UNINF_JUMP:
        case ITYPE_UNINF_CALL:
        case ITYPE_UNINF_JUMP2:
        case ITYPE_OTHER_UNINF:
        case ITYPE_RETURN:
        case ITYPE_COROUTINE:
        case ITYPE_EXCEPT_RET: {
            Addr npc = dynInst->pcState().as<RiscvISA::PCState>().npc();
            emitBranchMapPacket(true, npc);
            break;
        }

        case ITYPE_EXCEPTION:
        case ITYPE_INTERRUPT:
            // Handled above via PC discontinuity detection
            break;
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
ETrace::emitSyncPacket(Addr addr, uint8_t priv, bool isBranch, bool taken)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());
    pkt.set_format(ProtoMessage::ETracePacket::SYNC);
    pkt.set_subformat(ProtoMessage::ETracePacket::START);
    pkt.set_address(addr);
    pkt.set_priv(priv);
    // Per spec: branch field is 0 if address is a taken branch, 1 otherwise
    pkt.set_branch_map(isBranch && taken ? 0 : 1);
    pkt.set_branch_count(1);
    traceStream->write(pkt);

    lastReportedAddr = addr;
    resetBranchMap();
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
    pkt.set_address(addr);
    pkt.set_priv(priv);
    pkt.set_cause(cause);
    pkt.set_tval(tval);
    pkt.set_interrupt(isInterrupt);
    traceStream->write(pkt);

    lastReportedAddr = addr;
    resetBranchMap();
    stats.numTrapPackets++;
    stats.numPackets++;

    DPRINTF(ETrace, "Trap packet: addr=0x%08x cause=%llu int=%d priv=%d\n",
            addr, cause, isInterrupt, priv);
}

void
ETrace::emitBranchMapPacket(bool withAddress, Addr addr)
{
    ProtoMessage::ETracePacket pkt;
    pkt.set_tick(curTick());

    if (withAddress) {
        pkt.set_format(ProtoMessage::ETracePacket::BRANCH_MAP_ADDR);
        // Delta address encoding
        pkt.set_address(addr - lastReportedAddr);
        lastReportedAddr = addr;
    } else {
        pkt.set_format(ProtoMessage::ETracePacket::BRANCH_MAP);
    }

    pkt.set_branch_map(branchMap);
    pkt.set_branch_count(branchCount);
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
ETrace::resetBranchMap()
{
    branchMap = 0;
    branchCount = 0;
}

void
ETrace::flushTrace()
{
    // Emit any remaining branch map
    if (branchCount > 0) {
        emitBranchMapPacket(true, lastCommittedAddr);
    }

    DPRINTF(ETrace, "Flushing trace: %llu packets, %llu instructions\n",
            stats.numPackets.value(), stats.totalInstsTraced.value());

    delete traceStream;
    traceStream = nullptr;
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
               "Total instructions traced")
{}

} // namespace o3
} // namespace gem5
