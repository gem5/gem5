/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Lisa Hsu
 */

#include "gpu-compute/wavefront.hh"

#include "debug/GPUExec.hh"
#include "debug/WavefrontStack.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/vector_register_file.hh"

Wavefront*
WavefrontParams::create()
{
    return new Wavefront(this);
}

Wavefront::Wavefront(const Params *p)
  : SimObject(p), callArgMem(nullptr), _gpuISA(*this)
{
    lastTrace = 0;
    simdId = p->simdId;
    wfSlotId = p->wf_slot_id;
    status = S_STOPPED;
    reservedVectorRegs = 0;
    startVgprIndex = 0;
    outstandingReqs = 0;
    memReqsInPipe = 0;
    outstandingReqsWrGm = 0;
    outstandingReqsWrLm = 0;
    outstandingReqsRdGm = 0;
    outstandingReqsRdLm = 0;
    rdLmReqsInPipe = 0;
    rdGmReqsInPipe = 0;
    wrLmReqsInPipe = 0;
    wrGmReqsInPipe = 0;

    barrierCnt = 0;
    oldBarrierCnt = 0;
    stalledAtBarrier = false;

    memTraceBusy = 0;
    oldVgprTcnt = 0xffffffffffffffffll;
    oldDgprTcnt = 0xffffffffffffffffll;
    oldVgpr.resize(p->wfSize);

    pendingFetch = false;
    dropFetch = false;
    condRegState = new ConditionRegisterState();
    maxSpVgprs = 0;
    maxDpVgprs = 0;
    lastAddr.resize(p->wfSize);
    workItemFlatId.resize(p->wfSize);
    oldDgpr.resize(p->wfSize);
    barCnt.resize(p->wfSize);
    for (int i = 0; i < 3; ++i) {
        workItemId[i].resize(p->wfSize);
    }
}

void
Wavefront::regStats()
{
    SimObject::regStats();

    srcRegOpDist
        .init(0, 4, 2)
        .name(name() + ".src_reg_operand_dist")
        .desc("number of executed instructions with N source register operands")
        ;

    dstRegOpDist
        .init(0, 3, 2)
        .name(name() + ".dst_reg_operand_dist")
        .desc("number of executed instructions with N destination register "
              "operands")
        ;

    // FIXME: the name of the WF needs to be unique
    numTimesBlockedDueWAXDependencies
        .name(name() + ".timesBlockedDueWAXDependencies")
        .desc("number of times the wf's instructions are blocked due to WAW "
              "or WAR dependencies")
        ;

    // FIXME: the name of the WF needs to be unique
    numTimesBlockedDueRAWDependencies
        .name(name() + ".timesBlockedDueRAWDependencies")
        .desc("number of times the wf's instructions are blocked due to RAW "
              "dependencies")
        ;

    // FIXME: the name of the WF needs to be unique
    numTimesBlockedDueVrfPortAvail
        .name(name() + ".timesBlockedDueVrfPortAvail")
        .desc("number of times instructions are blocked due to VRF port "
              "availability")
        ;
}

void
Wavefront::init()
{
    reservedVectorRegs = 0;
    startVgprIndex = 0;
}

void
Wavefront::resizeRegFiles(int num_cregs, int num_sregs, int num_dregs)
{
    condRegState->init(num_cregs);
    maxSpVgprs = num_sregs;
    maxDpVgprs = num_dregs;
}

Wavefront::~Wavefront()
{
    if (callArgMem)
        delete callArgMem;
    delete condRegState;
}

void
Wavefront::start(uint64_t _wf_dyn_id,uint64_t _base_ptr)
{
    wfDynId = _wf_dyn_id;
    basePtr = _base_ptr;
    status = S_RUNNING;
}

bool
Wavefront::isGmInstruction(GPUDynInstPtr ii)
{
    if (ii->isGlobalMem() || ii->isFlat())
        return true;

    return false;
}

bool
Wavefront::isLmInstruction(GPUDynInstPtr ii)
{
    if (ii->isLocalMem()) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstALU()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && (ii->isNop() ||
        ii->isReturn() || ii->isBranch() ||
        ii->isALU() || (ii->isKernArgSeg() && ii->isLoad()))) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstBarrier()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isBarrier()) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstGMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isGlobalMem()) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstLMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isLocalMem()) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstPrivMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isPrivateSeg()) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstFlatMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isFlat()) {
        return true;
    }

    return false;
}

// Return true if the Wavefront's instruction
// buffer has branch instruction.
bool
Wavefront::instructionBufferHasBranch()
{
    for (auto it : instructionBuffer) {
        GPUDynInstPtr ii = it;

        if (ii->isReturn() || ii->isBranch()) {
            return true;
        }
    }

    return false;
}

// Remap HSAIL register to physical VGPR.
// HSAIL register = virtual register assigned to an operand by HLC compiler
uint32_t
Wavefront::remap(uint32_t vgprIndex, uint32_t size, uint8_t mode)
{
    assert((vgprIndex < reservedVectorRegs) && (reservedVectorRegs > 0));
    // add the offset from where the VGPRs of the wavefront have been assigned
    uint32_t physicalVgprIndex = startVgprIndex + vgprIndex;
    // HSAIL double precision (DP) register: calculate the physical VGPR index
    // assuming that DP registers are placed after SP ones in the VRF. The DP
    // and SP VGPR name spaces in HSAIL mode are separate so we need to adjust
    // the DP VGPR index before mapping it to the physical VRF address space
    if (mode == 1 && size > 4) {
        physicalVgprIndex = startVgprIndex + maxSpVgprs + (2 * vgprIndex);
    }

    assert((startVgprIndex <= physicalVgprIndex) &&
           (startVgprIndex + reservedVectorRegs - 1) >= physicalVgprIndex);

    // calculate absolute physical VGPR index
    return physicalVgprIndex % computeUnit->vrf[simdId]->numRegs();
}

// Return true if this wavefront is ready
// to execute an instruction of the specified type.
int
Wavefront::ready(itype_e type)
{
    // Check to make sure wave is running
    if (status == S_STOPPED || status == S_RETURNING ||
        instructionBuffer.empty()) {
        return 0;
    }

    // Is the wave waiting at a barrier
    if (stalledAtBarrier) {
        if (!computeUnit->AllAtBarrier(barrierId,barrierCnt,
                        computeUnit->getRefCounter(dispatchId, wgId))) {
            // Are all threads at barrier?
            return 0;
        }
        oldBarrierCnt = barrierCnt;
        stalledAtBarrier = false;
    }

    // Read instruction
    GPUDynInstPtr ii = instructionBuffer.front();

    bool ready_inst M5_VAR_USED = false;
    bool glbMemBusRdy = false;
    bool glbMemIssueRdy = false;
    if (type == I_GLOBAL || type == I_FLAT || type == I_PRIVATE) {
        for (int j=0; j < computeUnit->numGlbMemUnits; ++j) {
            if (computeUnit->vrfToGlobalMemPipeBus[j].prerdy())
                glbMemBusRdy = true;
            if (computeUnit->wfWait[j].prerdy())
                glbMemIssueRdy = true;
        }
    }
    bool locMemBusRdy = false;
    bool locMemIssueRdy = false;
    if (type == I_SHARED || type == I_FLAT) {
        for (int j=0; j < computeUnit->numLocMemUnits; ++j) {
            if (computeUnit->vrfToLocalMemPipeBus[j].prerdy())
                locMemBusRdy = true;
            if (computeUnit->wfWait[j].prerdy())
                locMemIssueRdy = true;
        }
    }

    // The following code is very error prone and the entire process for
    // checking readiness will be fixed eventually.  In the meantime, let's
    // make sure that we do not silently let an instruction type slip
    // through this logic and always return not ready.
    if (!(ii->isBarrier() || ii->isNop() || ii->isReturn() || ii->isBranch() ||
        ii->isALU() || ii->isLoad() || ii->isStore() || ii->isAtomic() ||
        ii->isMemFence() || ii->isFlat())) {
        panic("next instruction: %s is of unknown type\n", ii->disassemble());
    }

    DPRINTF(GPUExec, "CU%d: WF[%d][%d]: Checking Read for Inst : %s\n",
            computeUnit->cu_id, simdId, wfSlotId, ii->disassemble());

    if (type == I_ALU && ii->isBarrier()) {
        // Here for ALU instruction (barrier)
        if (!computeUnit->wfWait[simdId].prerdy()) {
            // Is wave slot free?
            return 0;
        }

        // Are there in pipe or outstanding memory requests?
        if ((outstandingReqs + memReqsInPipe) > 0) {
            return 0;
        }

        ready_inst = true;
    } else if (type == I_ALU && ii->isNop()) {
        // Here for ALU instruction (nop)
        if (!computeUnit->wfWait[simdId].prerdy()) {
            // Is wave slot free?
            return 0;
        }

        ready_inst = true;
    } else if (type == I_ALU && ii->isReturn()) {
        // Here for ALU instruction (return)
        if (!computeUnit->wfWait[simdId].prerdy()) {
            // Is wave slot free?
            return 0;
        }

        // Are there in pipe or outstanding memory requests?
        if ((outstandingReqs + memReqsInPipe) > 0) {
            return 0;
        }

        ready_inst = true;
    } else if (type == I_ALU && (ii->isBranch() ||
               ii->isALU() ||
               (ii->isKernArgSeg() && ii->isLoad()) ||
               ii->isArgSeg())) {
        // Here for ALU instruction (all others)
        if (!computeUnit->wfWait[simdId].prerdy()) {
            // Is alu slot free?
            return 0;
        }
        if (!computeUnit->vrf[simdId]->vrfOperandAccessReady(this, ii,
                    VrfAccessType::RD_WR)) {
            return 0;
        }

        if (!computeUnit->vrf[simdId]->operandsReady(this, ii)) {
            return 0;
        }
        ready_inst = true;
    } else if (type == I_GLOBAL && ii->isGlobalMem()) {
        // Here Global memory instruction
        if (ii->isLoad() || ii->isAtomic() || ii->isMemFence()) {
            // Are there in pipe or outstanding global memory write requests?
            if ((outstandingReqsWrGm + wrGmReqsInPipe) > 0) {
                return 0;
            }
        }

        if (ii->isStore() || ii->isAtomic() || ii->isMemFence()) {
            // Are there in pipe or outstanding global memory read requests?
            if ((outstandingReqsRdGm + rdGmReqsInPipe) > 0)
                return 0;
        }

        if (!glbMemIssueRdy) {
            // Is WV issue slot free?
            return 0;
        }

        if (!glbMemBusRdy) {
            // Is there an available VRF->Global memory read bus?
            return 0;
        }

        if (!computeUnit->globalMemoryPipe.
            isGMReqFIFOWrRdy(rdGmReqsInPipe + wrGmReqsInPipe)) {
            // Can we insert a new request to the Global Mem Request FIFO?
            return 0;
        }
        // can we schedule source & destination operands on the VRF?
        if (!computeUnit->vrf[simdId]->vrfOperandAccessReady(this, ii,
                    VrfAccessType::RD_WR)) {
            return 0;
        }
        if (!computeUnit->vrf[simdId]->operandsReady(this, ii)) {
            return 0;
        }
        ready_inst = true;
    } else if (type == I_SHARED && ii->isLocalMem()) {
        // Here for Shared memory instruction
        if (ii->isLoad() || ii->isAtomic() || ii->isMemFence()) {
            if ((outstandingReqsWrLm + wrLmReqsInPipe) > 0) {
                return 0;
            }
        }

        if (ii->isStore() || ii->isAtomic() || ii->isMemFence()) {
            if ((outstandingReqsRdLm + rdLmReqsInPipe) > 0) {
                return 0;
            }
        }

        if (!locMemBusRdy) {
            // Is there an available VRF->LDS read bus?
            return 0;
        }
        if (!locMemIssueRdy) {
            // Is wave slot free?
            return 0;
        }

        if (!computeUnit->localMemoryPipe.
            isLMReqFIFOWrRdy(rdLmReqsInPipe + wrLmReqsInPipe)) {
            // Can we insert a new request to the LDS Request FIFO?
            return 0;
        }
        // can we schedule source & destination operands on the VRF?
        if (!computeUnit->vrf[simdId]->vrfOperandAccessReady(this, ii,
                    VrfAccessType::RD_WR)) {
            return 0;
        }
        if (!computeUnit->vrf[simdId]->operandsReady(this, ii)) {
            return 0;
        }
        ready_inst = true;
    } else if (type == I_FLAT && ii->isFlat()) {
        if (!glbMemBusRdy) {
            // Is there an available VRF->Global memory read bus?
            return 0;
        }

        if (!locMemBusRdy) {
            // Is there an available VRF->LDS read bus?
            return 0;
        }

        if (!glbMemIssueRdy) {
            // Is wave slot free?
            return 0;
        }

        if (!locMemIssueRdy) {
            return 0;
        }
        if (!computeUnit->globalMemoryPipe.
            isGMReqFIFOWrRdy(rdGmReqsInPipe + wrGmReqsInPipe)) {
            // Can we insert a new request to the Global Mem Request FIFO?
            return 0;
        }

        if (!computeUnit->localMemoryPipe.
            isLMReqFIFOWrRdy(rdLmReqsInPipe + wrLmReqsInPipe)) {
            // Can we insert a new request to the LDS Request FIFO?
            return 0;
        }
        // can we schedule source & destination operands on the VRF?
        if (!computeUnit->vrf[simdId]->vrfOperandAccessReady(this, ii,
                    VrfAccessType::RD_WR)) {
            return 0;
        }
        // are all the operands ready? (RAW, WAW and WAR depedencies met?)
        if (!computeUnit->vrf[simdId]->operandsReady(this, ii)) {
            return 0;
        }
        ready_inst = true;
    } else {
        return 0;
    }

    assert(ready_inst);

    DPRINTF(GPUExec, "CU%d: WF[%d][%d]: Ready Inst : %s\n", computeUnit->cu_id,
            simdId, wfSlotId, ii->disassemble());
    return 1;
}

void
Wavefront::updateResources()
{
    // Get current instruction
    GPUDynInstPtr ii = instructionBuffer.front();
    assert(ii);
    computeUnit->vrf[simdId]->updateResources(this, ii);
    // Single precision ALU or Branch or Return or Special instruction
    if (ii->isALU() || ii->isSpecialOp() ||
        ii->isBranch() ||
        // FIXME: Kernel argument loads are currently treated as ALU operations
        // since we don't send memory packets at execution. If we fix that then
        // we should map them to one of the memory pipelines
        (ii->isKernArgSeg() && ii->isLoad()) || ii->isArgSeg() ||
        ii->isReturn()) {
        computeUnit->aluPipe[simdId].preset(computeUnit->shader->
                                            ticks(computeUnit->spBypassLength()));
        // this is to enforce a fixed number of cycles per issue slot per SIMD
        computeUnit->wfWait[simdId].preset(computeUnit->shader->
                                           ticks(computeUnit->issuePeriod));
    } else if (ii->isBarrier()) {
        computeUnit->wfWait[simdId].preset(computeUnit->shader->
                                           ticks(computeUnit->issuePeriod));
    } else if (ii->isLoad() && ii->isFlat()) {
        assert(Enums::SC_NONE != ii->executedAs());
        memReqsInPipe++;
        rdGmReqsInPipe++;
        if ( Enums::SC_SHARED == ii->executedAs() ) {
            computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
                preset(computeUnit->shader->ticks(4));
            computeUnit->wfWait[computeUnit->ShrMemUnitId()].
                preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
        } else {
            computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
                preset(computeUnit->shader->ticks(4));
            computeUnit->wfWait[computeUnit->GlbMemUnitId()].
                preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
        }
    } else if (ii->isStore() && ii->isFlat()) {
        assert(Enums::SC_NONE != ii->executedAs());
        memReqsInPipe++;
        wrGmReqsInPipe++;
        if (Enums::SC_SHARED == ii->executedAs()) {
            computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
                preset(computeUnit->shader->ticks(8));
            computeUnit->wfWait[computeUnit->ShrMemUnitId()].
                preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
        } else {
            computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
                preset(computeUnit->shader->ticks(8));
            computeUnit->wfWait[computeUnit->GlbMemUnitId()].
                preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
        }
    } else if (ii->isLoad() && ii->isGlobalMem()) {
        memReqsInPipe++;
        rdGmReqsInPipe++;
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            preset(computeUnit->shader->ticks(4));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (ii->isStore() && ii->isGlobalMem()) {
        memReqsInPipe++;
        wrGmReqsInPipe++;
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            preset(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if ((ii->isAtomic() || ii->isMemFence()) && ii->isGlobalMem()) {
        memReqsInPipe++;
        wrGmReqsInPipe++;
        rdGmReqsInPipe++;
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            preset(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (ii->isLoad() && ii->isLocalMem()) {
        memReqsInPipe++;
        rdLmReqsInPipe++;
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            preset(computeUnit->shader->ticks(4));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (ii->isStore() && ii->isLocalMem()) {
        memReqsInPipe++;
        wrLmReqsInPipe++;
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            preset(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if ((ii->isAtomic() || ii->isMemFence()) && ii->isLocalMem()) {
        memReqsInPipe++;
        wrLmReqsInPipe++;
        rdLmReqsInPipe++;
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            preset(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    }
}

void
Wavefront::exec()
{
    // ---- Exit if wavefront is inactive ----------------------------- //

    if (status == S_STOPPED || status == S_RETURNING ||
        instructionBuffer.empty()) {
        return;
    }

    // Get current instruction

    GPUDynInstPtr ii = instructionBuffer.front();

    const uint32_t old_pc = pc();
    DPRINTF(GPUExec, "CU%d: WF[%d][%d]: wave[%d] Executing inst: %s "
            "(pc: %i)\n", computeUnit->cu_id, simdId, wfSlotId, wfDynId,
            ii->disassemble(), old_pc);

    // update the instruction stats in the CU

    ii->execute(ii);
    computeUnit->updateInstStats(ii);
    // access the VRF
    computeUnit->vrf[simdId]->exec(ii, this);
    srcRegOpDist.sample(ii->numSrcRegOperands());
    dstRegOpDist.sample(ii->numDstRegOperands());
    computeUnit->numInstrExecuted++;
    computeUnit->execRateDist.sample(computeUnit->totalCycles.value() -
                                     computeUnit->lastExecCycle[simdId]);
    computeUnit->lastExecCycle[simdId] = computeUnit->totalCycles.value();
    if (pc() == old_pc) {
        uint32_t new_pc = _gpuISA.advancePC(old_pc, ii);
        // PC not modified by instruction, proceed to next or pop frame
        pc(new_pc);
        if (new_pc == rpc()) {
            popFromReconvergenceStack();
            discardFetch();
        } else {
            instructionBuffer.pop_front();
        }
    } else {
        discardFetch();
    }

    if (computeUnit->shader->hsail_mode==Shader::SIMT) {
        const int num_active_lanes = execMask().count();
        computeUnit->controlFlowDivergenceDist.sample(num_active_lanes);
        computeUnit->numVecOpsExecuted += num_active_lanes;
        if (isGmInstruction(ii)) {
            computeUnit->activeLanesPerGMemInstrDist.sample(num_active_lanes);
        } else if (isLmInstruction(ii)) {
            computeUnit->activeLanesPerLMemInstrDist.sample(num_active_lanes);
        }
    }

    // ---- Update Vector ALU pipeline and other resources ------------------ //
    // Single precision ALU or Branch or Return or Special instruction
    if (ii->isALU() || ii->isSpecialOp() ||
        ii->isBranch() ||
        // FIXME: Kernel argument loads are currently treated as ALU operations
        // since we don't send memory packets at execution. If we fix that then
        // we should map them to one of the memory pipelines
        (ii->isKernArgSeg() && ii->isLoad()) ||
        ii->isArgSeg() ||
        ii->isReturn()) {
        computeUnit->aluPipe[simdId].set(computeUnit->shader->
                                         ticks(computeUnit->spBypassLength()));

        // this is to enforce a fixed number of cycles per issue slot per SIMD
        computeUnit->wfWait[simdId].set(computeUnit->shader->
                                        ticks(computeUnit->issuePeriod));
    } else if (ii->isBarrier()) {
        computeUnit->wfWait[simdId].set(computeUnit->shader->
                                        ticks(computeUnit->issuePeriod));
    } else if (ii->isLoad() && ii->isFlat()) {
        assert(Enums::SC_NONE != ii->executedAs());

        if (Enums::SC_SHARED == ii->executedAs()) {
            computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
                set(computeUnit->shader->ticks(4));
            computeUnit->wfWait[computeUnit->ShrMemUnitId()].
                set(computeUnit->shader->ticks(computeUnit->issuePeriod));
        } else {
            computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
                set(computeUnit->shader->ticks(4));
            computeUnit->wfWait[computeUnit->GlbMemUnitId()].
                set(computeUnit->shader->ticks(computeUnit->issuePeriod));
        }
    } else if (ii->isStore() && ii->isFlat()) {
        assert(Enums::SC_NONE != ii->executedAs());
        if (Enums::SC_SHARED == ii->executedAs()) {
            computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
                set(computeUnit->shader->ticks(8));
            computeUnit->wfWait[computeUnit->ShrMemUnitId()].
                set(computeUnit->shader->ticks(computeUnit->issuePeriod));
        } else {
            computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
                set(computeUnit->shader->ticks(8));
            computeUnit->wfWait[computeUnit->GlbMemUnitId()].
                set(computeUnit->shader->ticks(computeUnit->issuePeriod));
        }
    } else if (ii->isLoad() && ii->isGlobalMem()) {
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            set(computeUnit->shader->ticks(4));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (ii->isStore() && ii->isGlobalMem()) {
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            set(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if ((ii->isAtomic() || ii->isMemFence()) && ii->isGlobalMem()) {
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            set(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (ii->isLoad() && ii->isLocalMem()) {
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            set(computeUnit->shader->ticks(4));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (ii->isStore() && ii->isLocalMem()) {
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            set(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if ((ii->isAtomic() || ii->isMemFence()) && ii->isLocalMem()) {
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            set(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    }
}

bool
Wavefront::waitingAtBarrier(int lane)
{
    return barCnt[lane] < maxBarCnt;
}

void
Wavefront::pushToReconvergenceStack(uint32_t pc, uint32_t rpc,
                                    const VectorMask& mask)
{
    assert(mask.count());
    reconvergenceStack.emplace_back(new ReconvergenceStackEntry{pc, rpc, mask});
}

void
Wavefront::popFromReconvergenceStack()
{
    assert(!reconvergenceStack.empty());

    DPRINTF(WavefrontStack, "[%2d, %2d, %2d, %2d] %s %3i => ",
            computeUnit->cu_id, simdId, wfSlotId, wfDynId,
            execMask().to_string<char, std::string::traits_type,
            std::string::allocator_type>().c_str(), pc());

    reconvergenceStack.pop_back();

    DPRINTF(WavefrontStack, "%3i %s\n", pc(),
            execMask().to_string<char, std::string::traits_type,
            std::string::allocator_type>().c_str());

}

void
Wavefront::discardFetch()
{
    instructionBuffer.clear();
    dropFetch |=pendingFetch;
}

uint32_t
Wavefront::pc() const
{
    return reconvergenceStack.back()->pc;
}

uint32_t
Wavefront::rpc() const
{
    return reconvergenceStack.back()->rpc;
}

VectorMask
Wavefront::execMask() const
{
    return reconvergenceStack.back()->execMask;
}

bool
Wavefront::execMask(int lane) const
{
    return reconvergenceStack.back()->execMask[lane];
}


void
Wavefront::pc(uint32_t new_pc)
{
    reconvergenceStack.back()->pc = new_pc;
}

uint32_t
Wavefront::getStaticContextSize() const
{
    return barCnt.size() * sizeof(int) + sizeof(wfId) + sizeof(maxBarCnt) +
           sizeof(oldBarrierCnt) + sizeof(barrierCnt) + sizeof(wgId) +
           sizeof(computeUnit->cu_id) + sizeof(barrierId) + sizeof(initMask) +
           sizeof(privBase) + sizeof(spillBase) + sizeof(ldsChunk) +
           computeUnit->wfSize() * sizeof(ReconvergenceStackEntry);
}

void
Wavefront::getContext(const void *out)
{
    uint8_t *iter = (uint8_t *)out;
    for (int i = 0; i < barCnt.size(); i++) {
        *(int *)iter = barCnt[i]; iter += sizeof(barCnt[i]);
    }
    *(int *)iter = wfId; iter += sizeof(wfId);
    *(int *)iter = maxBarCnt; iter += sizeof(maxBarCnt);
    *(int *)iter = oldBarrierCnt; iter += sizeof(oldBarrierCnt);
    *(int *)iter = barrierCnt; iter += sizeof(barrierCnt);
    *(int *)iter = computeUnit->cu_id; iter += sizeof(computeUnit->cu_id);
    *(uint32_t *)iter = wgId; iter += sizeof(wgId);
    *(uint32_t *)iter = barrierId; iter += sizeof(barrierId);
    *(uint64_t *)iter = initMask.to_ullong(); iter += sizeof(initMask.to_ullong());
    *(Addr *)iter = privBase; iter += sizeof(privBase);
    *(Addr *)iter = spillBase; iter += sizeof(spillBase);

    int stackSize = reconvergenceStack.size();
    ReconvergenceStackEntry empty = {std::numeric_limits<uint32_t>::max(),
                                    std::numeric_limits<uint32_t>::max(),
                                    std::numeric_limits<uint64_t>::max()};
    for (int i = 0; i < workItemId[0].size(); i++) {
        if (i < stackSize) {
            *(ReconvergenceStackEntry *)iter = *reconvergenceStack.back();
            iter += sizeof(ReconvergenceStackEntry);
            reconvergenceStack.pop_back();
        } else {
            *(ReconvergenceStackEntry *)iter = empty;
            iter += sizeof(ReconvergenceStackEntry);
        }
    }

    int wf_size = computeUnit->wfSize();
    for (int i = 0; i < maxSpVgprs; i++) {
        uint32_t vgprIdx = remap(i, sizeof(uint32_t), 1);
        for (int lane = 0; lane < wf_size; lane++) {
            uint32_t regVal = computeUnit->vrf[simdId]->
                            read<uint32_t>(vgprIdx,lane);
            *(uint32_t *)iter = regVal; iter += sizeof(regVal);
        }
    }

    for (int i = 0; i < maxDpVgprs; i++) {
        uint32_t vgprIdx = remap(i, sizeof(uint64_t), 1);
        for (int lane = 0; lane < wf_size; lane++) {
            uint64_t regVal = computeUnit->vrf[simdId]->
                            read<uint64_t>(vgprIdx,lane);
            *(uint64_t *)iter = regVal; iter += sizeof(regVal);
        }
    }

    for (int i = 0; i < condRegState->numRegs(); i++) {
        for (int lane = 0; lane < wf_size; lane++) {
            uint64_t regVal = condRegState->read<uint64_t>(i, lane);
            *(uint64_t *)iter = regVal; iter += sizeof(regVal);
        }
    }

    /* saving LDS content */
    if (ldsChunk)
        for (int i = 0; i < ldsChunk->size(); i++) {
            char val = ldsChunk->read<char>(i);
            *(char *) iter = val; iter += sizeof(val);
        }
}

void
Wavefront::setContext(const void *in)
{
    uint8_t *iter = (uint8_t *)in;
    for (int i = 0; i < barCnt.size(); i++) {
        barCnt[i] = *(int *)iter; iter += sizeof(barCnt[i]);
    }
    wfId = *(int *)iter; iter += sizeof(wfId);
    maxBarCnt = *(int *)iter; iter += sizeof(maxBarCnt);
    oldBarrierCnt = *(int *)iter; iter += sizeof(oldBarrierCnt);
    barrierCnt = *(int *)iter; iter += sizeof(barrierCnt);
    computeUnit->cu_id = *(int *)iter; iter += sizeof(computeUnit->cu_id);
    wgId = *(uint32_t *)iter; iter += sizeof(wgId);
    barrierId = *(uint32_t *)iter; iter += sizeof(barrierId);
    initMask = VectorMask(*(uint64_t *)iter); iter += sizeof(initMask);
    privBase = *(Addr *)iter; iter += sizeof(privBase);
    spillBase = *(Addr *)iter; iter += sizeof(spillBase);

    for (int i = 0; i < workItemId[0].size(); i++) {
        ReconvergenceStackEntry newEntry = *(ReconvergenceStackEntry *)iter;
        iter += sizeof(ReconvergenceStackEntry);
        if (newEntry.pc != std::numeric_limits<uint32_t>::max()) {
            pushToReconvergenceStack(newEntry.pc, newEntry.rpc,
                                     newEntry.execMask);
        }
    }
    int wf_size = computeUnit->wfSize();

    for (int i = 0; i < maxSpVgprs; i++) {
        uint32_t vgprIdx = remap(i, sizeof(uint32_t), 1);
        for (int lane = 0; lane < wf_size; lane++) {
            uint32_t regVal = *(uint32_t *)iter; iter += sizeof(regVal);
            computeUnit->vrf[simdId]->write<uint32_t>(vgprIdx, regVal, lane);
        }
    }

    for (int i = 0; i < maxDpVgprs; i++) {
        uint32_t vgprIdx = remap(i, sizeof(uint64_t), 1);
        for (int lane = 0; lane < wf_size; lane++) {
            uint64_t regVal = *(uint64_t *)iter; iter += sizeof(regVal);
            computeUnit->vrf[simdId]->write<uint64_t>(vgprIdx, regVal, lane);
        }
    }

    for (int i = 0; i < condRegState->numRegs(); i++) {
        for (int lane = 0; lane < wf_size; lane++) {
            uint64_t regVal = *(uint64_t *)iter; iter += sizeof(regVal);
            condRegState->write<uint64_t>(i, lane, regVal);
        }
    }
    /** Restoring LDS contents */
    if (ldsChunk)
        for (int i = 0; i < ldsChunk->size(); i++) {
            char val = *(char *) iter; iter += sizeof(val);
            ldsChunk->write<char>(i, val);
        }
}

void
Wavefront::computeActualWgSz(NDRange *ndr)
{
    actualWgSzTotal = 1;
    for (int d = 0; d < 3; ++d) {
        actualWgSz[d] = std::min(workGroupSz[d],
                                 gridSz[d] - ndr->wgId[d] * workGroupSz[d]);
        actualWgSzTotal *= actualWgSz[d];
    }
}
