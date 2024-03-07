/*
 * Copyright (c) 2024 Advanced Micro Devices, Inc.
 * All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
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
 */

#include "arch/amdgpu/vega/insts/instructions.hh"
#include "debug/GPUSync.hh"
#include "gpu-compute/shader.hh"

namespace gem5
{

namespace VegaISA
{
    // --- Inst_SOPP__S_NOP class methods ---

    Inst_SOPP__S_NOP::Inst_SOPP__S_NOP(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_nop")
    {
        setFlag(Nop);
    } // Inst_SOPP__S_NOP

    Inst_SOPP__S_NOP::~Inst_SOPP__S_NOP()
    {
    } // ~Inst_SOPP__S_NOP

    // --- description from .arch file ---
    // Do nothing. Repeat NOP 1..8 times based on SIMM16[2:0] -- 0 = 1 time,
    // 7 = 8 times.
    // This instruction may be used to introduce wait states to resolve
    // hazards; see the shader programming guide for details. Compare with
    // S_SLEEP.
    void
    Inst_SOPP__S_NOP::execute(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_SOPP__S_ENDPGM class methods ---

    Inst_SOPP__S_ENDPGM::Inst_SOPP__S_ENDPGM(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_endpgm")
    {
        setFlag(EndOfKernel);
    } // Inst_SOPP__S_ENDPGM

    Inst_SOPP__S_ENDPGM::~Inst_SOPP__S_ENDPGM()
    {
    } // ~Inst_SOPP__S_ENDPGM

    // --- description from .arch file ---
    // End of program; terminate wavefront.
    // The hardware implicitly executes S_WAITCNT 0 before executing this
    // ---  instruction.
    // See S_ENDPGM_SAVED for the context-switch version of this instruction.
    void
    Inst_SOPP__S_ENDPGM::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ComputeUnit *cu = gpuDynInst->computeUnit();

        // delete extra instructions fetched for completed work-items
        wf->instructionBuffer.erase(wf->instructionBuffer.begin() + 1,
            wf->instructionBuffer.end());

        if (wf->pendingFetch) {
            wf->dropFetch = true;
        }

        wf->computeUnit->fetchStage.fetchUnit(wf->simdId)
            .flushBuf(wf->wfSlotId);
        wf->setStatus(Wavefront::S_STOPPED);

        int refCount = wf->computeUnit->getLds()
            .decreaseRefCounter(wf->dispatchId, wf->wgId);

        /**
         * The parent WF of this instruction is exiting, therefore
         * it should not participate in this barrier any longer. This
         * prevents possible deadlock issues if WFs exit early.
         */
        int bar_id = WFBarrier::InvalidID;
        if (wf->hasBarrier()) {
            assert(wf->getStatus() != Wavefront::S_BARRIER);
            bar_id = wf->barrierId();
            assert(bar_id != WFBarrier::InvalidID);
            wf->releaseBarrier();
            cu->decMaxBarrierCnt(bar_id);
            DPRINTF(GPUSync, "CU[%d] WF[%d][%d] Wave[%d] - Exiting the "
                    "program and decrementing max barrier count for "
                    "barrier Id%d. New max count: %d.\n", cu->cu_id,
                    wf->simdId, wf->wfSlotId, wf->wfDynId, bar_id,
                    cu->maxBarrierCnt(bar_id));
        }

        DPRINTF(GPUExec, "CU%d: decrease ref ctr WG[%d] to [%d]\n",
            wf->computeUnit->cu_id, wf->wgId, refCount);

        wf->computeUnit->registerManager->freeRegisters(wf);
        wf->computeUnit->stats.completedWfs++;
        wf->computeUnit->activeWaves--;

        panic_if(wf->computeUnit->activeWaves < 0, "CU[%d] Active waves less "
            "than zero\n", wf->computeUnit->cu_id);

        DPRINTF(GPUExec, "Doing return for CU%d: WF[%d][%d][%d]\n",
            wf->computeUnit->cu_id, wf->simdId, wf->wfSlotId, wf->wfDynId);

        for (int i = 0; i < wf->vecReads.size(); i++) {
            if (wf->rawDist.find(i) != wf->rawDist.end()) {
                wf->stats.readsPerWrite.sample(wf->vecReads.at(i));
            }
        }
        wf->vecReads.clear();
        wf->rawDist.clear();
        wf->lastInstExec = 0;

        if (!refCount) {
            /**
             * If all WFs have finished, and hence the WG has finished,
             * then we can free up the barrier belonging to the parent
             * WG, but only if we actually used a barrier (i.e., more
             * than one WF in the WG).
             */
            if (bar_id != WFBarrier::InvalidID) {
                DPRINTF(GPUSync, "CU[%d] WF[%d][%d] Wave[%d] - All waves are "
                        "now complete. Releasing barrier Id%d.\n", cu->cu_id,
                        wf->simdId, wf->wfSlotId, wf->wfDynId,
                        wf->barrierId());
                cu->releaseBarrier(bar_id);
            }

           /**
             * Last wavefront of the workgroup has executed return. If the
             * workgroup is not the final one in the kernel, then simply
             * retire it; however, if it is the final one, i.e., indicating
             * the kernel end, then release operation (i.e., GL2 WB) is
             * needed
             */

            //check whether the workgroup is indicating the kernel end, i.e.,
            //the last workgroup in the kernel
            bool kernelEnd =
                wf->computeUnit->shader->dispatcher().isReachingKernelEnd(wf);

            bool relNeeded =
                wf->computeUnit->shader->impl_kern_end_rel;

            //if it is not a kernel end, then retire the workgroup directly
            if (!kernelEnd || !relNeeded) {
                wf->computeUnit->shader->dispatcher().notifyWgCompl(wf);
                wf->setStatus(Wavefront::S_STOPPED);
                wf->computeUnit->stats.completedWGs++;

                return;
            }

            /**
             * if it is a kernel end, inject a memory sync, i.e., GL2 WB, and
             * retire the workgroup after receving response.
             * note that GL0V and GL1 are read only, and they just forward GL2
             * WB request. When forwarding, GL1 send the request to all GL2 in
             * the complex
             */
            setFlag(MemSync);
            setFlag(GlobalSegment);
            // Notify Memory System of Kernel Completion
            // Kernel End = isKernel + isMemSync
            wf->setStatus(Wavefront::S_RETURNING);
            gpuDynInst->simdId = wf->simdId;
            gpuDynInst->wfSlotId = wf->wfSlotId;
            gpuDynInst->wfDynId = wf->wfDynId;

            DPRINTF(GPUExec, "inject global memory fence for CU%d: "
                            "WF[%d][%d][%d]\n", wf->computeUnit->cu_id,
                            wf->simdId, wf->wfSlotId, wf->wfDynId);

            // call shader to prepare the flush operations
            wf->computeUnit->shader->prepareFlush(gpuDynInst);

            wf->computeUnit->stats.completedWGs++;
        } else {
            wf->computeUnit->shader->dispatcher().scheduleDispatch();
        }
    } // execute

    // --- Inst_SOPP__S_BRANCH class methods ---

    Inst_SOPP__S_BRANCH::Inst_SOPP__S_BRANCH(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_branch")
    {
        setFlag(Branch);
    } // Inst_SOPP__S_BRANCH

    Inst_SOPP__S_BRANCH::~Inst_SOPP__S_BRANCH()
    {
    } // ~Inst_SOPP__S_BRANCH

    // --- description from .arch file ---
    // PC = PC + signext(SIMM16 * 4) + 4 (short jump).
    // For a long jump, use S_SETPC.
    void
    Inst_SOPP__S_BRANCH::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        Addr pc = gpuDynInst->pc();
        ScalarRegI16 simm16 = instData.SIMM16;

        pc = pc + ((ScalarRegI64)simm16 * 4LL) + 4LL;

        wf->pc(pc);
    } // execute
    // --- Inst_SOPP__S_WAKEUP class methods ---

    Inst_SOPP__S_WAKEUP::Inst_SOPP__S_WAKEUP(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_wakeup")
    {
    } // Inst_SOPP__S_WAKEUP

    Inst_SOPP__S_WAKEUP::~Inst_SOPP__S_WAKEUP()
    {
    } // ~Inst_SOPP__S_WAKEUP

    // --- description from .arch file ---
    // Allow a wave to 'ping' all the other waves in its threadgroup to force
    // them to wake up immediately from an S_SLEEP instruction. The ping is
    // ignored if the waves are not sleeping.
    // This allows for more efficient polling on a memory location. The waves
    // which are polling can sit in a long S_SLEEP between memory reads, but
    // the wave which writes the value can tell them all to wake up early now
    // that the data is available. This is useful for fBarrier implementations
    // (speedup).
    // This method is also safe from races because if any wave misses the ping,
    // everything still works fine (whoever missed it just completes their
    // normal S_SLEEP).
    void
    Inst_SOPP__S_WAKEUP::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_CBRANCH_SCC0 class methods ---

    Inst_SOPP__S_CBRANCH_SCC0::Inst_SOPP__S_CBRANCH_SCC0(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_cbranch_scc0")
    {
        setFlag(Branch);
    } // Inst_SOPP__S_CBRANCH_SCC0

    Inst_SOPP__S_CBRANCH_SCC0::~Inst_SOPP__S_CBRANCH_SCC0()
    {
    } // ~Inst_SOPP__S_CBRANCH_SCC0

    // --- description from .arch file ---
    // if (SCC == 0) then PC = PC + signext(SIMM16 * 4) + 4;
    // else NOP.
    void
    Inst_SOPP__S_CBRANCH_SCC0::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        Addr pc = gpuDynInst->pc();
        ScalarRegI16 simm16 = instData.SIMM16;
        ConstScalarOperandU32 scc(gpuDynInst, REG_SCC);

        scc.read();

        if (!scc.rawData()) {
            pc = pc + ((ScalarRegI64)simm16 * 4LL) + 4LL;
        }

        wf->pc(pc);
    } // execute
    // --- Inst_SOPP__S_CBRANCH_SCC1 class methods ---

    Inst_SOPP__S_CBRANCH_SCC1::Inst_SOPP__S_CBRANCH_SCC1(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_cbranch_scc1")
    {
        setFlag(Branch);
    } // Inst_SOPP__S_CBRANCH_SCC1

    Inst_SOPP__S_CBRANCH_SCC1::~Inst_SOPP__S_CBRANCH_SCC1()
    {
    } // ~Inst_SOPP__S_CBRANCH_SCC1

    // --- description from .arch file ---
    // if (SCC == 1) then PC = PC + signext(SIMM16 * 4) + 4;
    // else NOP.
    void
    Inst_SOPP__S_CBRANCH_SCC1::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        Addr pc = gpuDynInst->pc();
        ScalarRegI16 simm16 = instData.SIMM16;
        ConstScalarOperandU32 scc(gpuDynInst, REG_SCC);

        scc.read();

        if (scc.rawData()) {
            pc = pc + ((ScalarRegI64)simm16 * 4LL) + 4LL;
        }

        wf->pc(pc);
    } // execute
    // --- Inst_SOPP__S_CBRANCH_VCCZ class methods ---

    Inst_SOPP__S_CBRANCH_VCCZ::Inst_SOPP__S_CBRANCH_VCCZ(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_cbranch_vccz")
    {
        setFlag(Branch);
        setFlag(ReadsVCC);
    } // Inst_SOPP__S_CBRANCH_VCCZ

    Inst_SOPP__S_CBRANCH_VCCZ::~Inst_SOPP__S_CBRANCH_VCCZ()
    {
    } // ~Inst_SOPP__S_CBRANCH_VCCZ

    // --- description from .arch file ---
    // if (VCC == 0) then PC = PC + signext(SIMM16 * 4) + 4;
    // else NOP.
    void
    Inst_SOPP__S_CBRANCH_VCCZ::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstScalarOperandU64 vcc(gpuDynInst, REG_VCC_LO);
        Addr pc = gpuDynInst->pc();
        ScalarRegI16 simm16 = instData.SIMM16;

        vcc.read();

        if (!vcc.rawData()) {
            pc = pc + ((ScalarRegI64)simm16 * 4LL) + 4LL;
        }

        wf->pc(pc);
    } // execute
    // --- Inst_SOPP__S_CBRANCH_VCCNZ class methods ---

    Inst_SOPP__S_CBRANCH_VCCNZ::Inst_SOPP__S_CBRANCH_VCCNZ(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_cbranch_vccnz")
    {
        setFlag(Branch);
        setFlag(ReadsVCC);
    } // Inst_SOPP__S_CBRANCH_VCCNZ

    Inst_SOPP__S_CBRANCH_VCCNZ::~Inst_SOPP__S_CBRANCH_VCCNZ()
    {
    } // ~Inst_SOPP__S_CBRANCH_VCCNZ

    // --- description from .arch file ---
    // if (VCC != 0) then PC = PC + signext(SIMM16 * 4) + 4;
    // else NOP.
    void
    Inst_SOPP__S_CBRANCH_VCCNZ::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ConstScalarOperandU64 vcc(gpuDynInst, REG_VCC_LO);

        vcc.read();

        if (vcc.rawData()) {
            Addr pc = gpuDynInst->pc();
            ScalarRegI16 simm16 = instData.SIMM16;
            pc = pc + ((ScalarRegI64)simm16 * 4LL) + 4LL;
            wf->pc(pc);
        }
    } // execute
    // --- Inst_SOPP__S_CBRANCH_EXECZ class methods ---

    Inst_SOPP__S_CBRANCH_EXECZ::Inst_SOPP__S_CBRANCH_EXECZ(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_cbranch_execz")
    {
        setFlag(Branch);
        setFlag(ReadsEXEC);
    } // Inst_SOPP__S_CBRANCH_EXECZ

    Inst_SOPP__S_CBRANCH_EXECZ::~Inst_SOPP__S_CBRANCH_EXECZ()
    {
    } // ~Inst_SOPP__S_CBRANCH_EXECZ

    // --- description from .arch file ---
    // if (EXEC == 0) then PC = PC + signext(SIMM16 * 4) + 4;
    // else NOP.
    void
    Inst_SOPP__S_CBRANCH_EXECZ::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (wf->execMask().none()) {
            Addr pc = gpuDynInst->pc();
            ScalarRegI16 simm16 = instData.SIMM16;
            pc = pc + ((ScalarRegI64)simm16 * 4LL) + 4LL;
            wf->pc(pc);
        }
    } // execute
    // --- Inst_SOPP__S_CBRANCH_EXECNZ class methods ---

    Inst_SOPP__S_CBRANCH_EXECNZ::Inst_SOPP__S_CBRANCH_EXECNZ(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_cbranch_execnz")
    {
        setFlag(Branch);
        setFlag(ReadsEXEC);
    } // Inst_SOPP__S_CBRANCH_EXECNZ

    Inst_SOPP__S_CBRANCH_EXECNZ::~Inst_SOPP__S_CBRANCH_EXECNZ()
    {
    } // ~Inst_SOPP__S_CBRANCH_EXECNZ

    // --- description from .arch file ---
    // if (EXEC != 0) then PC = PC + signext(SIMM16 * 4) + 4;
    // else NOP.
    void
    Inst_SOPP__S_CBRANCH_EXECNZ::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (wf->execMask().any()) {
            Addr pc = gpuDynInst->pc();
            ScalarRegI16 simm16 = instData.SIMM16;
            pc = pc + ((ScalarRegI64)simm16 * 4LL) + 4LL;
            wf->pc(pc);
        }
    } // execute
    // --- Inst_SOPP__S_BARRIER class methods ---

    Inst_SOPP__S_BARRIER::Inst_SOPP__S_BARRIER(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_barrier")
    {
        setFlag(MemBarrier);
    } // Inst_SOPP__S_BARRIER

    Inst_SOPP__S_BARRIER::~Inst_SOPP__S_BARRIER()
    {
    } // ~Inst_SOPP__S_BARRIER

    // --- description from .arch file ---
    // Synchronize waves within a threadgroup.
    // If not all waves of the threadgroup have been created yet, waits for
    // entire group before proceeding.
    // If some waves in the threadgroup have already terminated, this waits on
    // only the surviving waves.
    // Barriers are legal inside trap handlers.
    void
    Inst_SOPP__S_BARRIER::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        ComputeUnit *cu = gpuDynInst->computeUnit();

        if (wf->hasBarrier()) {
            int bar_id = wf->barrierId();
            assert(wf->getStatus() == Wavefront::S_BARRIER);
            cu->incNumAtBarrier(bar_id);
            DPRINTF(GPUSync, "CU[%d] WF[%d][%d] Wave[%d] - Stalling at "
                    "barrier Id%d. %d waves now at barrier, %d waves "
                    "remain.\n", cu->cu_id, wf->simdId, wf->wfSlotId,
                    wf->wfDynId, bar_id, cu->numAtBarrier(bar_id),
                    cu->numYetToReachBarrier(bar_id));
        }
    } // execute
    // --- Inst_SOPP__S_SETKILL class methods ---

    Inst_SOPP__S_SETKILL::Inst_SOPP__S_SETKILL(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_setkill")
    {
    } // Inst_SOPP__S_SETKILL

    Inst_SOPP__S_SETKILL::~Inst_SOPP__S_SETKILL()
    {
    } // ~Inst_SOPP__S_SETKILL

    // --- description from .arch file ---
    // set KILL bit to value of SIMM16[0].
    // Used primarily for debugging kill wave host command behavior.
    void
    Inst_SOPP__S_SETKILL::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_WAITCNT class methods ---

    Inst_SOPP__S_WAITCNT::Inst_SOPP__S_WAITCNT(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_waitcnt")
    {
        setFlag(ALU);
        setFlag(Waitcnt);
    } // Inst_SOPP__S_WAITCNT

    Inst_SOPP__S_WAITCNT::~Inst_SOPP__S_WAITCNT()
    {
    } // ~Inst_SOPP__S_WAITCNT

    // --- description from .arch file ---
    // Wait for the counts of outstanding lds, vector-memory and
    // ---  export/vmem-write-data to be at or below the specified levels.
    // SIMM16[3:0] = vmcount (vector memory operations),
    // SIMM16[6:4] = export/mem-write-data count,
    // SIMM16[12:8] = LGKM_cnt (scalar-mem/GDS/LDS count).
    void
    Inst_SOPP__S_WAITCNT::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI32 vm_cnt = 0;
        ScalarRegI32 exp_cnt = 0;
        ScalarRegI32 lgkm_cnt = 0;
        vm_cnt = bits<ScalarRegI16>(instData.SIMM16, 3, 0);
        exp_cnt = bits<ScalarRegI16>(instData.SIMM16, 6, 4);
        lgkm_cnt = bits<ScalarRegI16>(instData.SIMM16, 12, 8);
        gpuDynInst->wavefront()->setStatus(Wavefront::S_WAITCNT);
        gpuDynInst->wavefront()->setWaitCnts(vm_cnt, exp_cnt, lgkm_cnt);
    } // execute
    // --- Inst_SOPP__S_SETHALT class methods ---

    Inst_SOPP__S_SETHALT::Inst_SOPP__S_SETHALT(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_sethalt")
    {
    } // Inst_SOPP__S_SETHALT

    Inst_SOPP__S_SETHALT::~Inst_SOPP__S_SETHALT()
    {
    } // ~Inst_SOPP__S_SETHALT

    // --- description from .arch file ---
    // Set HALT bit to value of SIMM16[0]; 1 = halt, 0 = resume.
    // The halt flag is ignored while PRIV == 1 (inside trap handlers) but the
    // shader will halt immediately after the handler returns if HALT is still
    // set at that time.
    void
    Inst_SOPP__S_SETHALT::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_SLEEP class methods ---

    Inst_SOPP__S_SLEEP::Inst_SOPP__S_SLEEP(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_sleep")
    {
        setFlag(ALU);
        setFlag(Sleep);
    } // Inst_SOPP__S_SLEEP

    Inst_SOPP__S_SLEEP::~Inst_SOPP__S_SLEEP()
    {
    } // ~Inst_SOPP__S_SLEEP

    // --- description from .arch file ---
    // Cause a wave to sleep for (64 * SIMM16[2:0] + 1..64) clocks.
    // The exact amount of delay is approximate. Compare with S_NOP.
    void
    Inst_SOPP__S_SLEEP::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegI32 simm16 = (ScalarRegI32)instData.SIMM16;
        gpuDynInst->wavefront()->setStatus(Wavefront::S_STALLED_SLEEP);
        // sleep duration is specified in multiples of 64 cycles
        gpuDynInst->wavefront()->setSleepTime(64 * simm16);
    } // execute
    // --- Inst_SOPP__S_SETPRIO class methods ---

    Inst_SOPP__S_SETPRIO::Inst_SOPP__S_SETPRIO(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_setprio")
    {
        setFlag(ALU);
    } // Inst_SOPP__S_SETPRIO

    Inst_SOPP__S_SETPRIO::~Inst_SOPP__S_SETPRIO()
    {
    } // ~Inst_SOPP__S_SETPRIO

    // --- description from .arch file ---
    // User settable wave priority is set to SIMM16[1:0]. 0 = lowest,
    // 3 = highest.
    // The overall wave priority is {SPIPrio[1:0] + UserPrio[1:0],
    // WaveAge[3:0]}.
    void
    Inst_SOPP__S_SETPRIO::execute(GPUDynInstPtr gpuDynInst)
    {
        ScalarRegU16 simm16 = instData.SIMM16;
        ScalarRegU32 userPrio = simm16 & 0x3;

        warn_once("S_SETPRIO ignored -- Requested priority %d\n", userPrio);
    } // execute
    // --- Inst_SOPP__S_SENDMSG class methods ---

    Inst_SOPP__S_SENDMSG::Inst_SOPP__S_SENDMSG(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_sendmsg")
    {
    } // Inst_SOPP__S_SENDMSG

    Inst_SOPP__S_SENDMSG::~Inst_SOPP__S_SENDMSG()
    {
    } // ~Inst_SOPP__S_SENDMSG

    // --- description from .arch file ---
    // Send a message upstream to VGT or the interrupt handler.
    // SIMM16[9:0] contains the message type and is documented in the shader
    // ---  programming guide.
    void
    Inst_SOPP__S_SENDMSG::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_SENDMSGHALT class methods ---

    Inst_SOPP__S_SENDMSGHALT::Inst_SOPP__S_SENDMSGHALT(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_sendmsghalt")
    {
    } // Inst_SOPP__S_SENDMSGHALT

    Inst_SOPP__S_SENDMSGHALT::~Inst_SOPP__S_SENDMSGHALT()
    {
    } // ~Inst_SOPP__S_SENDMSGHALT

    // --- description from .arch file ---
    // Send a message and then HALT the wavefront; see S_SENDMSG for details.
    void
    Inst_SOPP__S_SENDMSGHALT::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_TRAP class methods ---

    Inst_SOPP__S_TRAP::Inst_SOPP__S_TRAP(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_trap")
    {
    } // Inst_SOPP__S_TRAP

    Inst_SOPP__S_TRAP::~Inst_SOPP__S_TRAP()
    {
    } // ~Inst_SOPP__S_TRAP

    // --- description from .arch file ---
    // TrapID = SIMM16[7:0];
    // Wait for all instructions to complete;
    // set {TTMP1, TTMP0} = {3'h0, PCRewind[3:0], HT[0], TrapID[7:0],
    // PC[47:0]};
    // PC = TBA (trap base address);
    // PRIV = 1.
    // Enter the trap handler. This instruction may be generated internally as
    // well in response to a host trap (HT = 1) or an exception.
    // TrapID 0 is reserved for hardware use and should not be used in a
    // shader-generated trap.
    void
    Inst_SOPP__S_TRAP::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_ICACHE_INV class methods ---

    Inst_SOPP__S_ICACHE_INV::Inst_SOPP__S_ICACHE_INV(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_icache_inv")
    {
        setFlag(MemBarrier);
        setFlag(GPUStaticInst::MemSync);
        setFlag(MemSync);
    } // Inst_SOPP__S_ICACHE_INV

    Inst_SOPP__S_ICACHE_INV::~Inst_SOPP__S_ICACHE_INV()
    {
    } // ~Inst_SOPP__S_ICACHE_INV

    // --- description from .arch file ---
    // Invalidate entire L1 instruction cache.
    // You must have 12 separate S_NOP instructions or a jump/branch
    // instruction after this instruction
    // to ensure the SQ instruction buffer is purged.
    void
    Inst_SOPP__S_ICACHE_INV::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        gpuDynInst->resetEntireStatusVector();
        gpuDynInst->setStatusVector(0, 1);
        RequestPtr req = std::make_shared<Request>(0, 0, 0,
                                   gpuDynInst->computeUnit()->
                                   requestorId(), 0,
                                   gpuDynInst->wfDynId);
        gpuDynInst->setRequestFlags(req);
        gpuDynInst->computeUnit()->scalarMemoryPipe.
            injectScalarMemFence(gpuDynInst, false, req);
    } // execute
    // --- Inst_SOPP__S_INCPERFLEVEL class methods ---

    Inst_SOPP__S_INCPERFLEVEL::Inst_SOPP__S_INCPERFLEVEL(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_incperflevel")
    {
    } // Inst_SOPP__S_INCPERFLEVEL

    Inst_SOPP__S_INCPERFLEVEL::~Inst_SOPP__S_INCPERFLEVEL()
    {
    } // ~Inst_SOPP__S_INCPERFLEVEL

    // --- description from .arch file ---
    // Increment performance counter specified in SIMM16[3:0] by 1.
    void
    Inst_SOPP__S_INCPERFLEVEL::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_DECPERFLEVEL class methods ---

    Inst_SOPP__S_DECPERFLEVEL::Inst_SOPP__S_DECPERFLEVEL(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_decperflevel")
    {
    } // Inst_SOPP__S_DECPERFLEVEL

    Inst_SOPP__S_DECPERFLEVEL::~Inst_SOPP__S_DECPERFLEVEL()
    {
    } // ~Inst_SOPP__S_DECPERFLEVEL

    // --- description from .arch file ---
    // Decrement performance counter specified in SIMM16[3:0] by 1.
    void
    Inst_SOPP__S_DECPERFLEVEL::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_TTRACEDATA class methods ---

    Inst_SOPP__S_TTRACEDATA::Inst_SOPP__S_TTRACEDATA(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_ttracedata")
    {
    } // Inst_SOPP__S_TTRACEDATA

    Inst_SOPP__S_TTRACEDATA::~Inst_SOPP__S_TTRACEDATA()
    {
    } // ~Inst_SOPP__S_TTRACEDATA

    // --- description from .arch file ---
    // Send M0 as user data to the thread trace stream.
    void
    Inst_SOPP__S_TTRACEDATA::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_CBRANCH_CDBGSYS class methods ---

    Inst_SOPP__S_CBRANCH_CDBGSYS::Inst_SOPP__S_CBRANCH_CDBGSYS(
          InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_cbranch_cdbgsys")
    {
        setFlag(Branch);
    } // Inst_SOPP__S_CBRANCH_CDBGSYS

    Inst_SOPP__S_CBRANCH_CDBGSYS::~Inst_SOPP__S_CBRANCH_CDBGSYS()
    {
    } // ~Inst_SOPP__S_CBRANCH_CDBGSYS

    // --- description from .arch file ---
    // if (conditional_debug_system != 0) then PC = PC + signext(SIMM16 * 4)
    // + 4;
    // else NOP.
    void
    Inst_SOPP__S_CBRANCH_CDBGSYS::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_CBRANCH_CDBGUSER class methods ---

    Inst_SOPP__S_CBRANCH_CDBGUSER::Inst_SOPP__S_CBRANCH_CDBGUSER(
          InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_cbranch_cdbguser")
    {
        setFlag(Branch);
    } // Inst_SOPP__S_CBRANCH_CDBGUSER

    Inst_SOPP__S_CBRANCH_CDBGUSER::~Inst_SOPP__S_CBRANCH_CDBGUSER()
    {
    } // ~Inst_SOPP__S_CBRANCH_CDBGUSER

    // --- description from .arch file ---
    // if (conditional_debug_user != 0) then PC = PC + signext(SIMM16 * 4) + 4;
    // else NOP.
    void
    Inst_SOPP__S_CBRANCH_CDBGUSER::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_CBRANCH_CDBGSYS_OR_USER class methods ---

    Inst_SOPP__S_CBRANCH_CDBGSYS_OR_USER::Inst_SOPP__S_CBRANCH_CDBGSYS_OR_USER(
          InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_cbranch_cdbgsys_or_user")
    {
        setFlag(Branch);
    } // Inst_SOPP__S_CBRANCH_CDBGSYS_OR_USER

    Inst_SOPP__S_CBRANCH_CDBGSYS_OR_USER::
        ~Inst_SOPP__S_CBRANCH_CDBGSYS_OR_USER()
    {
    } // ~Inst_SOPP__S_CBRANCH_CDBGSYS_OR_USER

    // --- description from .arch file ---
    // if (conditional_debug_system || conditional_debug_user) then PC = PC +
    // ---  signext(SIMM16 * 4) + 4;
    // else NOP.
    void
    Inst_SOPP__S_CBRANCH_CDBGSYS_OR_USER::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_CBRANCH_CDBGSYS_AND_USER class methods ---

    Inst_SOPP__S_CBRANCH_CDBGSYS_AND_USER::
        Inst_SOPP__S_CBRANCH_CDBGSYS_AND_USER(InFmt_SOPP *iFmt)
            : Inst_SOPP(iFmt, "s_cbranch_cdbgsys_and_user")
    {
        setFlag(Branch);
    } // Inst_SOPP__S_CBRANCH_CDBGSYS_AND_USER

    Inst_SOPP__S_CBRANCH_CDBGSYS_AND_USER::
        ~Inst_SOPP__S_CBRANCH_CDBGSYS_AND_USER()
    {
    } // ~Inst_SOPP__S_CBRANCH_CDBGSYS_AND_USER

    // --- description from .arch file ---
    // if (conditional_debug_system && conditional_debug_user) then PC = PC +
    // ---  signext(SIMM16 * 4) + 4;
    // else NOP.
    void
    Inst_SOPP__S_CBRANCH_CDBGSYS_AND_USER::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_ENDPGM_SAVED class methods ---

    Inst_SOPP__S_ENDPGM_SAVED::Inst_SOPP__S_ENDPGM_SAVED(InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_endpgm_saved")
    {
    } // Inst_SOPP__S_ENDPGM_SAVED

    Inst_SOPP__S_ENDPGM_SAVED::~Inst_SOPP__S_ENDPGM_SAVED()
    {
    } // ~Inst_SOPP__S_ENDPGM_SAVED

    // --- description from .arch file ---
    // End of program; signal that a wave has been saved by the context-switch
    // trap handler and terminate wavefront.
    // The hardware implicitly executes S_WAITCNT 0 before executing this
    // instruction.
    // Use S_ENDPGM in all cases unless you are executing the context-switch
    // save handler.
    void
    Inst_SOPP__S_ENDPGM_SAVED::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_SET_GPR_IDX_OFF class methods ---

    Inst_SOPP__S_SET_GPR_IDX_OFF::Inst_SOPP__S_SET_GPR_IDX_OFF(
          InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_set_gpr_idx_off")
    {
    } // Inst_SOPP__S_SET_GPR_IDX_OFF

    Inst_SOPP__S_SET_GPR_IDX_OFF::~Inst_SOPP__S_SET_GPR_IDX_OFF()
    {
    } // ~Inst_SOPP__S_SET_GPR_IDX_OFF

    // --- description from .arch file ---
    // MODE.gpr_idx_en = 0.
    // Clear GPR indexing mode. Vector operations after this will not perform
    // ---  relative GPR addressing regardless of the contents of M0. This
    // ---  instruction does not modify M0.
    void
    Inst_SOPP__S_SET_GPR_IDX_OFF::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SOPP__S_SET_GPR_IDX_MODE class methods ---

    Inst_SOPP__S_SET_GPR_IDX_MODE::Inst_SOPP__S_SET_GPR_IDX_MODE(
          InFmt_SOPP *iFmt)
        : Inst_SOPP(iFmt, "s_set_gpr_idx_mode")
    {
    } // Inst_SOPP__S_SET_GPR_IDX_MODE

    Inst_SOPP__S_SET_GPR_IDX_MODE::~Inst_SOPP__S_SET_GPR_IDX_MODE()
    {
    } // ~Inst_SOPP__S_SET_GPR_IDX_MODE

    // --- description from .arch file ---
    // M0[15:12] = SIMM4.
    // Modify the mode used for vector GPR indexing.
    // The raw contents of the source field are read and used to set the enable
    // bits. SIMM4[0] = VSRC0_REL, SIMM4[1] = VSRC1_REL, SIMM4[2] = VSRC2_REL
    // and SIMM4[3] = VDST_REL.
    void
    Inst_SOPP__S_SET_GPR_IDX_MODE::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
} // namespace VegaISA
} // namespace gem5
