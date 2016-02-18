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
#include "gpu-compute/code_enums.hh"
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
  : SimObject(p), callArgMem(nullptr)
{
    last_trace = 0;
    simdId = p->simdId;
    wfSlotId = p->wf_slot_id;

    status = S_STOPPED;
    reservedVectorRegs = 0;
    startVgprIndex = 0;
    outstanding_reqs = 0;
    mem_reqs_in_pipe = 0;
    outstanding_reqs_wr_gm = 0;
    outstanding_reqs_wr_lm = 0;
    outstanding_reqs_rd_gm = 0;
    outstanding_reqs_rd_lm = 0;
    rd_lm_reqs_in_pipe = 0;
    rd_gm_reqs_in_pipe = 0;
    wr_lm_reqs_in_pipe = 0;
    wr_gm_reqs_in_pipe = 0;

    barrier_cnt = 0;
    old_barrier_cnt = 0;
    stalledAtBarrier = false;

    mem_trace_busy = 0;
    old_vgpr_tcnt = 0xffffffffffffffffll;
    old_dgpr_tcnt = 0xffffffffffffffffll;

    pendingFetch = false;
    dropFetch = false;
    condRegState = new ConditionRegisterState();
    maxSpVgprs = 0;
    maxDpVgprs = 0;
}

void
Wavefront::regStats()
{
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
}

void
Wavefront::start(uint64_t _wfDynId,uint64_t _base_ptr)
{
    wfDynId = _wfDynId;
    base_ptr = _base_ptr;
    status = S_RUNNING;
}

bool
Wavefront::isGmInstruction(GPUDynInstPtr ii)
{
    if (IS_OT_READ_PM(ii->opType()) || IS_OT_WRITE_PM(ii->opType()) ||
        IS_OT_ATOMIC_PM(ii->opType())) {
        return true;
    }

    if (IS_OT_READ_GM(ii->opType()) || IS_OT_WRITE_GM(ii->opType()) ||
        IS_OT_ATOMIC_GM(ii->opType())) {
        return true;
    }

    if (IS_OT_FLAT(ii->opType())) {
        return true;
    }

    return false;
}

bool
Wavefront::isLmInstruction(GPUDynInstPtr ii)
{
    if (IS_OT_READ_LM(ii->opType()) || IS_OT_WRITE_LM(ii->opType()) ||
        IS_OT_ATOMIC_LM(ii->opType())) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstALU()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && (ii->opType() == Enums::OT_NOP ||
        ii->opType() == Enums::OT_RET || ii->opType() == Enums::OT_BRANCH ||
        ii->opType() == Enums::OT_ALU || IS_OT_LDAS(ii->opType()) ||
        ii->opType() == Enums::OT_KERN_READ)) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstBarrier()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->opType() == Enums::OT_BARRIER) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstGMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && (IS_OT_READ_GM(ii->opType()) ||
        IS_OT_WRITE_GM(ii->opType()) || IS_OT_ATOMIC_GM(ii->opType()))) {

        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstLMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && (IS_OT_READ_LM(ii->opType()) ||
        IS_OT_WRITE_LM(ii->opType()) || IS_OT_ATOMIC_LM(ii->opType()))) {

        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstPrivMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && (IS_OT_READ_PM(ii->opType()) ||
        IS_OT_WRITE_PM(ii->opType()) || IS_OT_ATOMIC_PM(ii->opType()))) {

        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstFlatMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && IS_OT_FLAT(ii->opType())) {

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

        if (ii->opType() == Enums::OT_RET || ii->opType() == Enums::OT_BRANCH) {
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
        if (!computeUnit->AllAtBarrier(barrier_id,barrier_cnt,
                        computeUnit->getRefCounter(dispatchid, wg_id))) {
            // Are all threads at barrier?
            return 0;
        }
        old_barrier_cnt = barrier_cnt;
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
    if (!(ii->opType() == Enums::OT_BARRIER || ii->opType() == Enums::OT_NOP ||
          ii->opType() == Enums::OT_RET || ii->opType() == Enums::OT_BRANCH ||
          ii->opType() == Enums::OT_ALU || IS_OT_LDAS(ii->opType()) ||
          ii->opType() == Enums::OT_KERN_READ ||
          ii->opType() == Enums::OT_ARG ||
          IS_OT_READ_GM(ii->opType()) || IS_OT_WRITE_GM(ii->opType()) ||
          IS_OT_ATOMIC_GM(ii->opType()) || IS_OT_READ_LM(ii->opType()) ||
          IS_OT_WRITE_LM(ii->opType()) || IS_OT_ATOMIC_LM(ii->opType()) ||
          IS_OT_READ_PM(ii->opType()) || IS_OT_WRITE_PM(ii->opType()) ||
          IS_OT_ATOMIC_PM(ii->opType()) || IS_OT_FLAT(ii->opType()))) {
        panic("next instruction: %s is of unknown type\n", ii->disassemble());
    }

    DPRINTF(GPUExec, "CU%d: WF[%d][%d]: Checking Read for Inst : %s\n",
            computeUnit->cu_id, simdId, wfSlotId, ii->disassemble());

    if (type == I_ALU && ii->opType() == Enums::OT_BARRIER) {
        // Here for ALU instruction (barrier)
        if (!computeUnit->wfWait[simdId].prerdy()) {
            // Is wave slot free?
            return 0;
        }

        // Are there in pipe or outstanding memory requests?
        if ((outstanding_reqs + mem_reqs_in_pipe) > 0) {
            return 0;
        }

        ready_inst = true;
    } else if (type == I_ALU && ii->opType() == Enums::OT_NOP) {
        // Here for ALU instruction (nop)
        if (!computeUnit->wfWait[simdId].prerdy()) {
            // Is wave slot free?
            return 0;
        }

        ready_inst = true;
    } else if (type == I_ALU && ii->opType() == Enums::OT_RET) {
        // Here for ALU instruction (return)
        if (!computeUnit->wfWait[simdId].prerdy()) {
            // Is wave slot free?
            return 0;
        }

        // Are there in pipe or outstanding memory requests?
        if ((outstanding_reqs + mem_reqs_in_pipe) > 0) {
            return 0;
        }

        ready_inst = true;
    } else if (type == I_ALU && (ii->opType() == Enums::OT_BRANCH ||
               ii->opType() == Enums::OT_ALU || IS_OT_LDAS(ii->opType()) ||
               ii->opType() == Enums::OT_KERN_READ ||
               ii->opType() == Enums::OT_ARG)) {
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
    } else if (type == I_GLOBAL && (IS_OT_READ_GM(ii->opType()) ||
               IS_OT_WRITE_GM(ii->opType()) || IS_OT_ATOMIC_GM(ii->opType()))) {
        // Here Global memory instruction
        if (IS_OT_READ_GM(ii->opType()) || IS_OT_ATOMIC_GM(ii->opType())) {
            // Are there in pipe or outstanding global memory write requests?
            if ((outstanding_reqs_wr_gm + wr_gm_reqs_in_pipe) > 0) {
                return 0;
            }
        }

        if (IS_OT_WRITE_GM(ii->opType()) || IS_OT_ATOMIC_GM(ii->opType()) ||
            IS_OT_HIST_GM(ii->opType())) {
            // Are there in pipe or outstanding global memory read requests?
            if ((outstanding_reqs_rd_gm + rd_gm_reqs_in_pipe) > 0)
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
            isGMReqFIFOWrRdy(rd_gm_reqs_in_pipe + wr_gm_reqs_in_pipe)) {
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
    } else if (type == I_SHARED && (IS_OT_READ_LM(ii->opType()) ||
               IS_OT_WRITE_LM(ii->opType()) || IS_OT_ATOMIC_LM(ii->opType()))) {
        // Here for Shared memory instruction
        if (IS_OT_READ_LM(ii->opType()) || IS_OT_ATOMIC_LM(ii->opType())) {
            if ((outstanding_reqs_wr_lm + wr_lm_reqs_in_pipe) > 0) {
                return 0;
            }
        }

        if (IS_OT_WRITE_LM(ii->opType()) || IS_OT_ATOMIC_LM(ii->opType()) ||
            IS_OT_HIST_LM(ii->opType())) {
            if ((outstanding_reqs_rd_lm + rd_lm_reqs_in_pipe) > 0) {
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
            isLMReqFIFOWrRdy(rd_lm_reqs_in_pipe + wr_lm_reqs_in_pipe)) {
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
    } else if (type == I_PRIVATE && (IS_OT_READ_PM(ii->opType()) ||
               IS_OT_WRITE_PM(ii->opType()) || IS_OT_ATOMIC_PM(ii->opType()))) {
        // Here for Private memory instruction ------------------------    //
        if (IS_OT_READ_PM(ii->opType()) || IS_OT_ATOMIC_PM(ii->opType())) {
            if ((outstanding_reqs_wr_gm + wr_gm_reqs_in_pipe) > 0) {
                return 0;
            }
        }

        if (IS_OT_WRITE_PM(ii->opType()) || IS_OT_ATOMIC_PM(ii->opType()) ||
            IS_OT_HIST_PM(ii->opType())) {
            if ((outstanding_reqs_rd_gm + rd_gm_reqs_in_pipe) > 0) {
                return 0;
            }
        }

        if (!glbMemBusRdy) {
            // Is there an available VRF->Global memory read bus?
            return 0;
        }

        if (!glbMemIssueRdy) {
             // Is wave slot free?
            return 0;
        }

        if (!computeUnit->globalMemoryPipe.
            isGMReqFIFOWrRdy(rd_gm_reqs_in_pipe + wr_gm_reqs_in_pipe)) {
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
    } else if (type == I_FLAT && IS_OT_FLAT(ii->opType())) {
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
            isGMReqFIFOWrRdy(rd_gm_reqs_in_pipe + wr_gm_reqs_in_pipe)) {
            // Can we insert a new request to the Global Mem Request FIFO?
            return 0;
        }

        if (!computeUnit->localMemoryPipe.
            isLMReqFIFOWrRdy(rd_lm_reqs_in_pipe + wr_lm_reqs_in_pipe)) {
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
    if (ii->opType() == Enums::OT_ALU || ii->opType() == Enums::OT_SPECIAL ||
        ii->opType() == Enums::OT_BRANCH || IS_OT_LDAS(ii->opType()) ||
        // FIXME: Kernel argument loads are currently treated as ALU operations
        // since we don't send memory packets at execution. If we fix that then
        // we should map them to one of the memory pipelines
        ii->opType()==Enums::OT_KERN_READ ||
        ii->opType()==Enums::OT_ARG ||
        ii->opType()==Enums::OT_RET) {
        computeUnit->aluPipe[simdId].preset(computeUnit->shader->
                                            ticks(computeUnit->spBypassLength()));
        // this is to enforce a fixed number of cycles per issue slot per SIMD
        computeUnit->wfWait[simdId].preset(computeUnit->shader->
                                           ticks(computeUnit->issuePeriod));
    } else if (ii->opType() == Enums::OT_BARRIER) {
        computeUnit->wfWait[simdId].preset(computeUnit->shader->
                                           ticks(computeUnit->issuePeriod));
    } else if (ii->opType() == Enums::OT_FLAT_READ) {
        assert(Enums::SC_NONE != ii->executedAs());
        mem_reqs_in_pipe++;
        rd_gm_reqs_in_pipe++;
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
    } else if (ii->opType() == Enums::OT_FLAT_WRITE) {
        assert(Enums::SC_NONE != ii->executedAs());
        mem_reqs_in_pipe++;
        wr_gm_reqs_in_pipe++;
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
    } else if (IS_OT_READ_GM(ii->opType())) {
        mem_reqs_in_pipe++;
        rd_gm_reqs_in_pipe++;
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            preset(computeUnit->shader->ticks(4));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_WRITE_GM(ii->opType())) {
        mem_reqs_in_pipe++;
        wr_gm_reqs_in_pipe++;
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            preset(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_ATOMIC_GM(ii->opType())) {
        mem_reqs_in_pipe++;
        wr_gm_reqs_in_pipe++;
        rd_gm_reqs_in_pipe++;
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            preset(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_READ_LM(ii->opType())) {
        mem_reqs_in_pipe++;
        rd_lm_reqs_in_pipe++;
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            preset(computeUnit->shader->ticks(4));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_WRITE_LM(ii->opType())) {
        mem_reqs_in_pipe++;
        wr_lm_reqs_in_pipe++;
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            preset(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_ATOMIC_LM(ii->opType())) {
        mem_reqs_in_pipe++;
        wr_lm_reqs_in_pipe++;
        rd_lm_reqs_in_pipe++;
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            preset(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_READ_PM(ii->opType())) {
        mem_reqs_in_pipe++;
        rd_gm_reqs_in_pipe++;
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            preset(computeUnit->shader->ticks(4));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_WRITE_PM(ii->opType())) {
        mem_reqs_in_pipe++;
        wr_gm_reqs_in_pipe++;
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            preset(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            preset(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_ATOMIC_PM(ii->opType())) {
        mem_reqs_in_pipe++;
        wr_gm_reqs_in_pipe++;
        rd_gm_reqs_in_pipe++;
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            preset(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
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
    ii->execute();
    // access the VRF
    computeUnit->vrf[simdId]->exec(ii, this);
    srcRegOpDist.sample(ii->numSrcRegOperands());
    dstRegOpDist.sample(ii->numDstRegOperands());
    computeUnit->numInstrExecuted++;
    computeUnit->execRateDist.sample(computeUnit->totalCycles.value() -
                                     computeUnit->lastExecCycle[simdId]);
    computeUnit->lastExecCycle[simdId] = computeUnit->totalCycles.value();
    if (pc() == old_pc) {
        uint32_t new_pc = old_pc + 1;
        // PC not modified by instruction, proceed to next or pop frame
        pc(new_pc);
        if (new_pc == rpc()) {
            popFromReconvergenceStack();
            discardFetch();
        } else {
            instructionBuffer.pop_front();
        }
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
    if (ii->opType() == Enums::OT_ALU || ii->opType() == Enums::OT_SPECIAL ||
        ii->opType() == Enums::OT_BRANCH || IS_OT_LDAS(ii->opType()) ||
        // FIXME: Kernel argument loads are currently treated as ALU operations
        // since we don't send memory packets at execution. If we fix that then
        // we should map them to one of the memory pipelines
        ii->opType() == Enums::OT_KERN_READ ||
        ii->opType() == Enums::OT_ARG ||
        ii->opType() == Enums::OT_RET) {
        computeUnit->aluPipe[simdId].set(computeUnit->shader->
                                         ticks(computeUnit->spBypassLength()));

        // this is to enforce a fixed number of cycles per issue slot per SIMD
        computeUnit->wfWait[simdId].set(computeUnit->shader->
                                        ticks(computeUnit->issuePeriod));
    } else if (ii->opType() == Enums::OT_BARRIER) {
        computeUnit->wfWait[simdId].set(computeUnit->shader->
                                        ticks(computeUnit->issuePeriod));
    } else if (ii->opType() == Enums::OT_FLAT_READ) {
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
    } else if (ii->opType() == Enums::OT_FLAT_WRITE) {
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
    } else if (IS_OT_READ_GM(ii->opType())) {
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            set(computeUnit->shader->ticks(4));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_WRITE_GM(ii->opType())) {
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            set(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_ATOMIC_GM(ii->opType())) {
        computeUnit->vrfToGlobalMemPipeBus[computeUnit->nextGlbRdBus()].
            set(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->GlbMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_READ_LM(ii->opType())) {
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            set(computeUnit->shader->ticks(4));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_WRITE_LM(ii->opType())) {
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            set(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    } else if (IS_OT_ATOMIC_LM(ii->opType())) {
        computeUnit->vrfToLocalMemPipeBus[computeUnit->nextLocRdBus()].
            set(computeUnit->shader->ticks(8));
        computeUnit->wfWait[computeUnit->ShrMemUnitId()].
            set(computeUnit->shader->ticks(computeUnit->issuePeriod));
    }
}

bool
Wavefront::waitingAtBarrier(int lane)
{
    return bar_cnt[lane] < max_bar_cnt;
}

void
Wavefront::pushToReconvergenceStack(uint32_t pc, uint32_t rpc,
                                    const VectorMask& mask)
{
    assert(mask.count());
    reconvergenceStack.emplace(new ReconvergenceStackEntry(pc, rpc, mask));
}

void
Wavefront::popFromReconvergenceStack()
{
    assert(!reconvergenceStack.empty());

    DPRINTF(WavefrontStack, "[%2d, %2d, %2d, %2d] %s %3i => ",
            computeUnit->cu_id, simdId, wfSlotId, wfDynId,
            execMask().to_string<char, std::string::traits_type,
            std::string::allocator_type>().c_str(), pc());

    reconvergenceStack.pop();

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
    return reconvergenceStack.top()->pc;
}

uint32_t
Wavefront::rpc() const
{
    return reconvergenceStack.top()->rpc;
}

VectorMask
Wavefront::execMask() const
{
    return reconvergenceStack.top()->execMask;
}

bool
Wavefront::execMask(int lane) const
{
    return reconvergenceStack.top()->execMask[lane];
}


void
Wavefront::pc(uint32_t new_pc)
{
    reconvergenceStack.top()->pc = new_pc;
}
