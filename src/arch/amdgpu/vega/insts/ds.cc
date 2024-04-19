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

namespace gem5
{

namespace VegaISA
{
    // --- Inst_DS__DS_ADD_U32 class methods ---

    Inst_DS__DS_ADD_U32::Inst_DS__DS_ADD_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_add_u32")
    {
        setFlag(MemoryRef);
        setFlag(GroupSegment);
        setFlag(AtomicAdd);
        setFlag(AtomicNoReturn);
    } // Inst_DS__DS_ADD_U32

    Inst_DS__DS_ADD_U32::~Inst_DS__DS_ADD_U32()
    {
    } // ~Inst_DS__DS_ADD_U32

    // --- description from .arch file ---
    // 32b:
    // MEM[ADDR] += DATA;
    void
    Inst_DS__DS_ADD_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU32 data(gpuDynInst, extData.DATA0);

        addr.read();
        data.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->a_data))[lane]
                    = data[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_ADD_U32::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initAtomicAccess<VecElemU32>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_ADD_U32::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_DS__DS_SUB_U32 class methods ---

    Inst_DS__DS_SUB_U32::Inst_DS__DS_SUB_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_sub_u32")
    {
    } // Inst_DS__DS_SUB_U32

    Inst_DS__DS_SUB_U32::~Inst_DS__DS_SUB_U32()
    {
    } // ~Inst_DS__DS_SUB_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_SUB_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_RSUB_U32 class methods ---

    Inst_DS__DS_RSUB_U32::Inst_DS__DS_RSUB_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_rsub_u32")
    {
    } // Inst_DS__DS_RSUB_U32

    Inst_DS__DS_RSUB_U32::~Inst_DS__DS_RSUB_U32()
    {
    } // ~Inst_DS__DS_RSUB_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = DATA - MEM[ADDR];
    // RETURN_DATA = tmp.
    // Subtraction with reversed operands.
    void
    Inst_DS__DS_RSUB_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_INC_U32 class methods ---

    Inst_DS__DS_INC_U32::Inst_DS__DS_INC_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_inc_u32")
    {
    } // Inst_DS__DS_INC_U32

    Inst_DS__DS_INC_U32::~Inst_DS__DS_INC_U32()
    {
    } // ~Inst_DS__DS_INC_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp >= DATA) ? 0 : tmp + 1 (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_INC_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_DEC_U32 class methods ---

    Inst_DS__DS_DEC_U32::Inst_DS__DS_DEC_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_dec_u32")
    {
    } // Inst_DS__DS_DEC_U32

    Inst_DS__DS_DEC_U32::~Inst_DS__DS_DEC_U32()
    {
    } // ~Inst_DS__DS_DEC_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp == 0 || tmp > DATA) ? DATA : tmp - 1
    // (unsigned compare); RETURN_DATA = tmp.
    void
    Inst_DS__DS_DEC_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_I32 class methods ---

    Inst_DS__DS_MIN_I32::Inst_DS__DS_MIN_I32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_i32")
    {
    } // Inst_DS__DS_MIN_I32

    Inst_DS__DS_MIN_I32::~Inst_DS__DS_MIN_I32()
    {
    } // ~Inst_DS__DS_MIN_I32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA < tmp) ? DATA : tmp (signed compare);
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_MIN_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_I32 class methods ---

    Inst_DS__DS_MAX_I32::Inst_DS__DS_MAX_I32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_i32")
    {
    } // Inst_DS__DS_MAX_I32

    Inst_DS__DS_MAX_I32::~Inst_DS__DS_MAX_I32()
    {
    } // ~Inst_DS__DS_MAX_I32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA > tmp) ? DATA : tmp (signed compare);
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_MAX_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_U32 class methods ---

    Inst_DS__DS_MIN_U32::Inst_DS__DS_MIN_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_u32")
    {
    } // Inst_DS__DS_MIN_U32

    Inst_DS__DS_MIN_U32::~Inst_DS__DS_MIN_U32()
    {
    } // ~Inst_DS__DS_MIN_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA < tmp) ? DATA : tmp (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_MIN_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_U32 class methods ---

    Inst_DS__DS_MAX_U32::Inst_DS__DS_MAX_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_u32")
    {
    } // Inst_DS__DS_MAX_U32

    Inst_DS__DS_MAX_U32::~Inst_DS__DS_MAX_U32()
    {
    } // ~Inst_DS__DS_MAX_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA > tmp) ? DATA : tmp (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_MAX_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_AND_B32 class methods ---

    Inst_DS__DS_AND_B32::Inst_DS__DS_AND_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_and_b32")
    {
    } // Inst_DS__DS_AND_B32

    Inst_DS__DS_AND_B32::~Inst_DS__DS_AND_B32()
    {
    } // ~Inst_DS__DS_AND_B32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] &= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_AND_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_OR_B32 class methods ---

    Inst_DS__DS_OR_B32::Inst_DS__DS_OR_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_or_b32")
    {
        setFlag(MemoryRef);
        setFlag(GroupSegment);
        setFlag(AtomicOr);
        setFlag(AtomicNoReturn);
    } // Inst_DS__DS_OR_B32

    Inst_DS__DS_OR_B32::~Inst_DS__DS_OR_B32()
    {
    } // ~Inst_DS__DS_OR_B32

    // --- description from .arch file ---
    // 32b:
    // MEM[ADDR] |= DATA;
    void
    Inst_DS__DS_OR_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU32 data(gpuDynInst, extData.DATA0);

        addr.read();
        data.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->a_data))[lane]
                    = data[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_OR_B32::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initAtomicAccess<VecElemU32>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_OR_B32::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc

    // --- Inst_DS__DS_XOR_B32 class methods ---

    Inst_DS__DS_XOR_B32::Inst_DS__DS_XOR_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_xor_b32")
    {
    } // Inst_DS__DS_XOR_B32

    Inst_DS__DS_XOR_B32::~Inst_DS__DS_XOR_B32()
    {
    } // ~Inst_DS__DS_XOR_B32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] ^= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_XOR_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MSKOR_B32 class methods ---

    Inst_DS__DS_MSKOR_B32::Inst_DS__DS_MSKOR_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_mskor_b32")
    {
    } // Inst_DS__DS_MSKOR_B32

    Inst_DS__DS_MSKOR_B32::~Inst_DS__DS_MSKOR_B32()
    {
    } // ~Inst_DS__DS_MSKOR_B32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (MEM_ADDR[ADDR] & ~DATA) | DATA2;
    // RETURN_DATA = tmp.
    // Masked dword OR, D0 contains the mask and D1 contains the new value.
    void
    Inst_DS__DS_MSKOR_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRITE_B32 class methods ---

    Inst_DS__DS_WRITE_B32::Inst_DS__DS_WRITE_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write_b32")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE_B32

    Inst_DS__DS_WRITE_B32::~Inst_DS__DS_WRITE_B32()
    {
    } // ~Inst_DS__DS_WRITE_B32

    // --- description from .arch file ---
    // 32b:
    // MEM[ADDR] = DATA.
    // Write dword.
    void
    Inst_DS__DS_WRITE_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU32 data(gpuDynInst, extData.DATA0);

        addr.read();
        data.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE_B32::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemWrite<VecElemU32>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_WRITE_B32::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_DS__DS_WRITE2_B32 class methods ---

    Inst_DS__DS_WRITE2_B32::Inst_DS__DS_WRITE2_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write2_b32")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE2_B32

    Inst_DS__DS_WRITE2_B32::~Inst_DS__DS_WRITE2_B32()
    {
    } // ~Inst_DS__DS_WRITE2_B32

    // --- description from .arch file ---
    // 32b:
    // MEM[ADDR_BASE + OFFSET0 * 4] = DATA;
    // MEM[ADDR_BASE + OFFSET1 * 4] = DATA2.
    // Write 2 dwords.
    void
    Inst_DS__DS_WRITE2_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU32 data0(gpuDynInst, extData.DATA0);
        ConstVecOperandU32 data1(gpuDynInst, extData.DATA1);

        addr.read();
        data0.read();
        data1.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane * 2]
                    = data0[lane];
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 2 + 1] = data1[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE2_B32::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0 * 4;
        Addr offset1 = instData.OFFSET1 * 4;

        initDualMemWrite<VecElemU32>(gpuDynInst, offset0, offset1);
    }

    void
    Inst_DS__DS_WRITE2_B32::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    }
    // --- Inst_DS__DS_WRITE2ST64_B32 class methods ---

    Inst_DS__DS_WRITE2ST64_B32::Inst_DS__DS_WRITE2ST64_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write2st64_b32")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE2ST64_B32

    Inst_DS__DS_WRITE2ST64_B32::~Inst_DS__DS_WRITE2ST64_B32()
    {
    } // ~Inst_DS__DS_WRITE2ST64_B32

    // --- description from .arch file ---
    // 32b:
    // MEM[ADDR_BASE + OFFSET0 * 4 * 64] = DATA;
    // MEM[ADDR_BASE + OFFSET1 * 4 * 64] = DATA2;
    // Write 2 dwords.
    void
    Inst_DS__DS_WRITE2ST64_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU32 data0(gpuDynInst, extData.DATA0);
        ConstVecOperandU32 data1(gpuDynInst, extData.DATA1);

        addr.read();
        data0.read();
        data1.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane * 2]
                    = data0[lane];
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 2 + 1] = data1[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE2ST64_B32::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0 * 4 * 64;
        Addr offset1 = instData.OFFSET1 * 4 * 64;

        initDualMemWrite<VecElemU32>(gpuDynInst, offset0, offset1);
    }

    void
    Inst_DS__DS_WRITE2ST64_B32::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    }
    // --- Inst_DS__DS_CMPST_B32 class methods ---

    Inst_DS__DS_CMPST_B32::Inst_DS__DS_CMPST_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_cmpst_b32")
    {
    } // Inst_DS__DS_CMPST_B32

    Inst_DS__DS_CMPST_B32::~Inst_DS__DS_CMPST_B32()
    {
    } // ~Inst_DS__DS_CMPST_B32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // src = DATA2;
    // cmp = DATA;
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0] = tmp.
    // Compare and store.
    // Caution, the order of src and cmp are the *opposite* of the
    // ---  BUFFER_ATOMIC_CMPSWAP opcode.
    void
    Inst_DS__DS_CMPST_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_CMPST_F32 class methods ---

    Inst_DS__DS_CMPST_F32::Inst_DS__DS_CMPST_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_cmpst_f32")
    {
        setFlag(F32);
    } // Inst_DS__DS_CMPST_F32

    Inst_DS__DS_CMPST_F32::~Inst_DS__DS_CMPST_F32()
    {
    } // ~Inst_DS__DS_CMPST_F32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // src = DATA2;
    // cmp = DATA;
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0] = tmp.
    // Floating point compare and store that handles NaN/INF/denormal values.
    // Caution, the order of src and cmp are the *opposite* of the
    // ---  BUFFER_ATOMIC_FCMPSWAP opcode.
    void
    Inst_DS__DS_CMPST_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_F32 class methods ---

    Inst_DS__DS_MIN_F32::Inst_DS__DS_MIN_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_f32")
    {
        setFlag(F32);
    } // Inst_DS__DS_MIN_F32

    Inst_DS__DS_MIN_F32::~Inst_DS__DS_MIN_F32()
    {
    } // ~Inst_DS__DS_MIN_F32

    // --- description from .arch file ---
    // 32b.
    // tmp = MEM[ADDR];
    // src = DATA;
    // cmp = DATA2;
    // MEM[ADDR] = (cmp < tmp) ? src : tmp.
    // Floating point minimum that handles NaN/INF/denormal values.
    // Note that this opcode is slightly more general-purpose than
    // ---  BUFFER_ATOMIC_FMIN.
    void
    Inst_DS__DS_MIN_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_F32 class methods ---

    Inst_DS__DS_MAX_F32::Inst_DS__DS_MAX_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_f32")
    {
        setFlag(F32);
    } // Inst_DS__DS_MAX_F32

    Inst_DS__DS_MAX_F32::~Inst_DS__DS_MAX_F32()
    {
    } // ~Inst_DS__DS_MAX_F32

    // --- description from .arch file ---
    // 32b.
    // tmp = MEM[ADDR];
    // src = DATA;
    // cmp = DATA2;
    // MEM[ADDR] = (tmp > cmp) ? src : tmp.
    // Floating point maximum that handles NaN/INF/denormal values.
    // Note that this opcode is slightly more general-purpose than
    // ---  BUFFER_ATOMIC_FMAX.
    void
    Inst_DS__DS_MAX_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_NOP class methods ---

    Inst_DS__DS_NOP::Inst_DS__DS_NOP(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_nop")
    {
        setFlag(Nop);
    } // Inst_DS__DS_NOP

    Inst_DS__DS_NOP::~Inst_DS__DS_NOP()
    {
    } // ~Inst_DS__DS_NOP

    // --- description from .arch file ---
    // Do nothing.
    void
    Inst_DS__DS_NOP::execute(GPUDynInstPtr gpuDynInst)
    {
        gpuDynInst->wavefront()->decLGKMInstsIssued();
    } // execute
    // --- Inst_DS__DS_ADD_F32 class methods ---

    Inst_DS__DS_ADD_F32::Inst_DS__DS_ADD_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_add_f32")
    {
        setFlag(F32);
        setFlag(MemoryRef);
        setFlag(GroupSegment);
        setFlag(AtomicAdd);
        setFlag(AtomicNoReturn);
    } // Inst_DS__DS_ADD_F32

    Inst_DS__DS_ADD_F32::~Inst_DS__DS_ADD_F32()
    {
    } // ~Inst_DS__DS_ADD_F32

    // --- description from .arch file ---
    // 32b:
    // MEM[ADDR] += DATA;
    // Floating point add that handles NaN/INF/denormal values.
    void
    Inst_DS__DS_ADD_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandF32 data(gpuDynInst, extData.DATA0);

        addr.read();
        data.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemF32*>(gpuDynInst->a_data))[lane]
                    = data[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_ADD_F32::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initAtomicAccess<VecElemF32>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_ADD_F32::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_DS__DS_WRITE_B8 class methods ---

    Inst_DS__DS_WRITE_B8::Inst_DS__DS_WRITE_B8(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write_b8")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE_B8

    Inst_DS__DS_WRITE_B8::~Inst_DS__DS_WRITE_B8()
    {
    } // ~Inst_DS__DS_WRITE_B8

    // --- description from .arch file ---
    // MEM[ADDR] = DATA[7:0].
    // Byte write.
    void
    Inst_DS__DS_WRITE_B8::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU8 data(gpuDynInst, extData.DATA0);

        addr.read();
        data.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU8*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE_B8::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemWrite<VecElemU8>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_WRITE_B8::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_DS__DS_WRITE_B8_D16_HI class methods ---

    Inst_DS__DS_WRITE_B8_D16_HI::Inst_DS__DS_WRITE_B8_D16_HI(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write_b8_d16_hi")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE_B8_D16_HI

    Inst_DS__DS_WRITE_B8_D16_HI::~Inst_DS__DS_WRITE_B8_D16_HI()
    {
    } // ~Inst_DS__DS_WRITE_B8_D16_HI

    // --- description from .arch file ---
    // MEM[ADDR] = DATA[23:16].
    // Byte write in to high word.
    void
    Inst_DS__DS_WRITE_B8_D16_HI::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU8 data(gpuDynInst, extData.DATA0);

        addr.read();
        data.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU8*>(gpuDynInst->d_data))[lane]
                    = bits(data[lane], 23, 16);
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE_B8_D16_HI::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemWrite<VecElemU8>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_WRITE_B8_D16_HI::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_DS__DS_WRITE_B16 class methods ---

    Inst_DS__DS_WRITE_B16::Inst_DS__DS_WRITE_B16(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write_b16")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE_B16

    Inst_DS__DS_WRITE_B16::~Inst_DS__DS_WRITE_B16()
    {
    } // ~Inst_DS__DS_WRITE_B16

    // --- description from .arch file ---
    // MEM[ADDR] = DATA[15:0]
    // Short write.
    void
    Inst_DS__DS_WRITE_B16::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU16 data(gpuDynInst, extData.DATA0);

        addr.read();
        data.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU16*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE_B16::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemWrite<VecElemU16>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_WRITE_B16::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_DS__DS_ADD_RTN_U32 class methods ---

    Inst_DS__DS_ADD_RTN_U32::Inst_DS__DS_ADD_RTN_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_add_rtn_u32")
    {
    } // Inst_DS__DS_ADD_RTN_U32

    Inst_DS__DS_ADD_RTN_U32::~Inst_DS__DS_ADD_RTN_U32()
    {
    } // ~Inst_DS__DS_ADD_RTN_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] += DATA;
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_ADD_RTN_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_SUB_RTN_U32 class methods ---

    Inst_DS__DS_SUB_RTN_U32::Inst_DS__DS_SUB_RTN_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_sub_rtn_u32")
    {
    } // Inst_DS__DS_SUB_RTN_U32

    Inst_DS__DS_SUB_RTN_U32::~Inst_DS__DS_SUB_RTN_U32()
    {
    } // ~Inst_DS__DS_SUB_RTN_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_SUB_RTN_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_RSUB_RTN_U32 class methods ---

    Inst_DS__DS_RSUB_RTN_U32::Inst_DS__DS_RSUB_RTN_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_rsub_rtn_u32")
    {
    } // Inst_DS__DS_RSUB_RTN_U32

    Inst_DS__DS_RSUB_RTN_U32::~Inst_DS__DS_RSUB_RTN_U32()
    {
    } // ~Inst_DS__DS_RSUB_RTN_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = DATA - MEM[ADDR];
    // RETURN_DATA = tmp.
    // Subtraction with reversed operands.
    void
    Inst_DS__DS_RSUB_RTN_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_INC_RTN_U32 class methods ---

    Inst_DS__DS_INC_RTN_U32::Inst_DS__DS_INC_RTN_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_inc_rtn_u32")
    {
    } // Inst_DS__DS_INC_RTN_U32

    Inst_DS__DS_INC_RTN_U32::~Inst_DS__DS_INC_RTN_U32()
    {
    } // ~Inst_DS__DS_INC_RTN_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp >= DATA) ? 0 : tmp + 1 (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_INC_RTN_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_DEC_RTN_U32 class methods ---

    Inst_DS__DS_DEC_RTN_U32::Inst_DS__DS_DEC_RTN_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_dec_rtn_u32")
    {
    } // Inst_DS__DS_DEC_RTN_U32

    Inst_DS__DS_DEC_RTN_U32::~Inst_DS__DS_DEC_RTN_U32()
    {
    } // ~Inst_DS__DS_DEC_RTN_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp == 0 || tmp > DATA) ? DATA : tmp - 1
    // (unsigned compare); RETURN_DATA = tmp.
    void
    Inst_DS__DS_DEC_RTN_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_RTN_I32 class methods ---

    Inst_DS__DS_MIN_RTN_I32::Inst_DS__DS_MIN_RTN_I32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_rtn_i32")
    {
    } // Inst_DS__DS_MIN_RTN_I32

    Inst_DS__DS_MIN_RTN_I32::~Inst_DS__DS_MIN_RTN_I32()
    {
    } // ~Inst_DS__DS_MIN_RTN_I32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA < tmp) ? DATA : tmp (signed compare);
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_MIN_RTN_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_RTN_I32 class methods ---

    Inst_DS__DS_MAX_RTN_I32::Inst_DS__DS_MAX_RTN_I32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_rtn_i32")
    {
    } // Inst_DS__DS_MAX_RTN_I32

    Inst_DS__DS_MAX_RTN_I32::~Inst_DS__DS_MAX_RTN_I32()
    {
    } // ~Inst_DS__DS_MAX_RTN_I32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA > tmp) ? DATA : tmp (signed compare);
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_MAX_RTN_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_RTN_U32 class methods ---

    Inst_DS__DS_MIN_RTN_U32::Inst_DS__DS_MIN_RTN_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_rtn_u32")
    {
    } // Inst_DS__DS_MIN_RTN_U32

    Inst_DS__DS_MIN_RTN_U32::~Inst_DS__DS_MIN_RTN_U32()
    {
    } // ~Inst_DS__DS_MIN_RTN_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA < tmp) ? DATA : tmp (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_MIN_RTN_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_RTN_U32 class methods ---

    Inst_DS__DS_MAX_RTN_U32::Inst_DS__DS_MAX_RTN_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_rtn_u32")
    {
    } // Inst_DS__DS_MAX_RTN_U32

    Inst_DS__DS_MAX_RTN_U32::~Inst_DS__DS_MAX_RTN_U32()
    {
    } // ~Inst_DS__DS_MAX_RTN_U32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA > tmp) ? DATA : tmp (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_MAX_RTN_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_AND_RTN_B32 class methods ---

    Inst_DS__DS_AND_RTN_B32::Inst_DS__DS_AND_RTN_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_and_rtn_b32")
    {
    } // Inst_DS__DS_AND_RTN_B32

    Inst_DS__DS_AND_RTN_B32::~Inst_DS__DS_AND_RTN_B32()
    {
    } // ~Inst_DS__DS_AND_RTN_B32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] &= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_AND_RTN_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_OR_RTN_B32 class methods ---

    Inst_DS__DS_OR_RTN_B32::Inst_DS__DS_OR_RTN_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_or_rtn_b32")
    {
    } // Inst_DS__DS_OR_RTN_B32

    Inst_DS__DS_OR_RTN_B32::~Inst_DS__DS_OR_RTN_B32()
    {
    } // ~Inst_DS__DS_OR_RTN_B32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] |= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_OR_RTN_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_XOR_RTN_B32 class methods ---

    Inst_DS__DS_XOR_RTN_B32::Inst_DS__DS_XOR_RTN_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_xor_rtn_b32")
    {
    } // Inst_DS__DS_XOR_RTN_B32

    Inst_DS__DS_XOR_RTN_B32::~Inst_DS__DS_XOR_RTN_B32()
    {
    } // ~Inst_DS__DS_XOR_RTN_B32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] ^= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_XOR_RTN_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MSKOR_RTN_B32 class methods ---

    Inst_DS__DS_MSKOR_RTN_B32::Inst_DS__DS_MSKOR_RTN_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_mskor_rtn_b32")
    {
    } // Inst_DS__DS_MSKOR_RTN_B32

    Inst_DS__DS_MSKOR_RTN_B32::~Inst_DS__DS_MSKOR_RTN_B32()
    {
    } // ~Inst_DS__DS_MSKOR_RTN_B32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (MEM_ADDR[ADDR] & ~DATA) | DATA2;
    // RETURN_DATA = tmp.
    // Masked dword OR, D0 contains the mask and D1 contains the new value.
    void
    Inst_DS__DS_MSKOR_RTN_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRXCHG_RTN_B32 class methods ---

    Inst_DS__DS_WRXCHG_RTN_B32::Inst_DS__DS_WRXCHG_RTN_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_wrxchg_rtn_b32")
    {
    } // Inst_DS__DS_WRXCHG_RTN_B32

    Inst_DS__DS_WRXCHG_RTN_B32::~Inst_DS__DS_WRXCHG_RTN_B32()
    {
    } // ~Inst_DS__DS_WRXCHG_RTN_B32

    // --- description from .arch file ---
    // tmp = MEM[ADDR];
    // MEM[ADDR] = DATA;
    // RETURN_DATA = tmp.
    // Write-exchange operation.
    void
    Inst_DS__DS_WRXCHG_RTN_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRXCHG2_RTN_B32 class methods ---

    Inst_DS__DS_WRXCHG2_RTN_B32::Inst_DS__DS_WRXCHG2_RTN_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_wrxchg2_rtn_b32")
    {
    } // Inst_DS__DS_WRXCHG2_RTN_B32

    Inst_DS__DS_WRXCHG2_RTN_B32::~Inst_DS__DS_WRXCHG2_RTN_B32()
    {
    } // ~Inst_DS__DS_WRXCHG2_RTN_B32

    // --- description from .arch file ---
    // Write-exchange 2 separate dwords.
    void
    Inst_DS__DS_WRXCHG2_RTN_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRXCHG2ST64_RTN_B32 class methods ---

    Inst_DS__DS_WRXCHG2ST64_RTN_B32::Inst_DS__DS_WRXCHG2ST64_RTN_B32(
          InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_wrxchg2st64_rtn_b32")
    {
    } // Inst_DS__DS_WRXCHG2ST64_RTN_B32

    Inst_DS__DS_WRXCHG2ST64_RTN_B32::~Inst_DS__DS_WRXCHG2ST64_RTN_B32()
    {
    } // ~Inst_DS__DS_WRXCHG2ST64_RTN_B32

    // --- description from .arch file ---
    // Write-exchange 2 separate dwords with a stride of 64 dwords.
    void
    Inst_DS__DS_WRXCHG2ST64_RTN_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_CMPST_RTN_B32 class methods ---

    Inst_DS__DS_CMPST_RTN_B32::Inst_DS__DS_CMPST_RTN_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_cmpst_rtn_b32")
    {
    } // Inst_DS__DS_CMPST_RTN_B32

    Inst_DS__DS_CMPST_RTN_B32::~Inst_DS__DS_CMPST_RTN_B32()
    {
    } // ~Inst_DS__DS_CMPST_RTN_B32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // src = DATA2;
    // cmp = DATA;
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0] = tmp.
    // Compare and store.
    // Caution, the order of src and cmp are the *opposite* of the
    // ---  BUFFER_ATOMIC_CMPSWAP opcode.
    void
    Inst_DS__DS_CMPST_RTN_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_CMPST_RTN_F32 class methods ---

    Inst_DS__DS_CMPST_RTN_F32::Inst_DS__DS_CMPST_RTN_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_cmpst_rtn_f32")
    {
        setFlag(F32);
    } // Inst_DS__DS_CMPST_RTN_F32

    Inst_DS__DS_CMPST_RTN_F32::~Inst_DS__DS_CMPST_RTN_F32()
    {
    } // ~Inst_DS__DS_CMPST_RTN_F32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // src = DATA2;
    // cmp = DATA;
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0] = tmp.
    // Floating point compare and store that handles NaN/INF/denormal values.
    // Caution, the order of src and cmp are the *opposite* of the
    // ---  BUFFER_ATOMIC_FCMPSWAP opcode.
    void
    Inst_DS__DS_CMPST_RTN_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_RTN_F32 class methods ---

    Inst_DS__DS_MIN_RTN_F32::Inst_DS__DS_MIN_RTN_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_rtn_f32")
    {
        setFlag(F32);
    } // Inst_DS__DS_MIN_RTN_F32

    Inst_DS__DS_MIN_RTN_F32::~Inst_DS__DS_MIN_RTN_F32()
    {
    } // ~Inst_DS__DS_MIN_RTN_F32

    // --- description from .arch file ---
    // 32b.
    // tmp = MEM[ADDR];
    // src = DATA;
    // cmp = DATA2;
    // MEM[ADDR] = (cmp < tmp) ? src : tmp.
    // Floating point minimum that handles NaN/INF/denormal values.
    // Note that this opcode is slightly more general-purpose than
    // ---  BUFFER_ATOMIC_FMIN.
    void
    Inst_DS__DS_MIN_RTN_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_RTN_F32 class methods ---

    Inst_DS__DS_MAX_RTN_F32::Inst_DS__DS_MAX_RTN_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_rtn_f32")
    {
        setFlag(F32);
    } // Inst_DS__DS_MAX_RTN_F32

    Inst_DS__DS_MAX_RTN_F32::~Inst_DS__DS_MAX_RTN_F32()
    {
    } // ~Inst_DS__DS_MAX_RTN_F32

    // --- description from .arch file ---
    // 32b.
    // tmp = MEM[ADDR];
    // src = DATA;
    // cmp = DATA2;
    // MEM[ADDR] = (tmp > cmp) ? src : tmp.
    // Floating point maximum that handles NaN/INF/denormal values.
    // Note that this opcode is slightly more general-purpose than
    // ---  BUFFER_ATOMIC_FMAX.
    void
    Inst_DS__DS_MAX_RTN_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRAP_RTN_B32 class methods ---

    Inst_DS__DS_WRAP_RTN_B32::Inst_DS__DS_WRAP_RTN_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_wrap_rtn_b32")
    {
    } // Inst_DS__DS_WRAP_RTN_B32

    Inst_DS__DS_WRAP_RTN_B32::~Inst_DS__DS_WRAP_RTN_B32()
    {
    } // ~Inst_DS__DS_WRAP_RTN_B32

    // --- description from .arch file ---
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp >= DATA) ? tmp - DATA : tmp + DATA2;
    // RETURN_DATA = tmp.
    void
    Inst_DS__DS_WRAP_RTN_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_ADD_RTN_F32 class methods ---

    Inst_DS__DS_ADD_RTN_F32::Inst_DS__DS_ADD_RTN_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_add_rtn_f32")
    {
        setFlag(F32);
    } // Inst_DS__DS_ADD_RTN_F32

    Inst_DS__DS_ADD_RTN_F32::~Inst_DS__DS_ADD_RTN_F32()
    {
    } // ~Inst_DS__DS_ADD_RTN_F32

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] += DATA;
    // RETURN_DATA = tmp.
    // Floating point add that handles NaN/INF/denormal values.
    void
    Inst_DS__DS_ADD_RTN_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_READ_B32 class methods ---

    Inst_DS__DS_READ_B32::Inst_DS__DS_READ_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read_b32")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ_B32

    Inst_DS__DS_READ_B32::~Inst_DS__DS_READ_B32()
    {
    } // ~Inst_DS__DS_READ_B32

    // --- description from .arch file ---
    // RETURN_DATA = MEM[ADDR].
    // Dword read.
    void
    Inst_DS__DS_READ_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_READ_B32::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemRead<VecElemU32>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_READ_B32::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDST);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane];
            }
        }

        vdst.write();
    } // completeAcc
    // --- Inst_DS__DS_READ2_B32 class methods ---

    Inst_DS__DS_READ2_B32::Inst_DS__DS_READ2_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read2_b32")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ2_B32

    Inst_DS__DS_READ2_B32::~Inst_DS__DS_READ2_B32()
    {
    } // ~Inst_DS__DS_READ2_B32

    // --- description from .arch file ---
    // RETURN_DATA[0] = MEM[ADDR_BASE + OFFSET0 * 4];
    // RETURN_DATA[1] = MEM[ADDR_BASE + OFFSET1 * 4].
    // Read 2 dwords.
    void
    Inst_DS__DS_READ2_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_READ2_B32::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0 * 4;
        Addr offset1 = instData.OFFSET1 * 4;

        initDualMemRead<VecElemU32>(gpuDynInst, offset0, offset1);
    } // initiateAcc

    void
    Inst_DS__DS_READ2_B32::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst0(gpuDynInst, extData.VDST);
        VecOperandU32 vdst1(gpuDynInst, extData.VDST + 1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 2];
                vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 2 + 1];
            }
        }

        vdst0.write();
        vdst1.write();
    } // completeAcc
    // --- Inst_DS__DS_READ2ST64_B32 class methods ---

    Inst_DS__DS_READ2ST64_B32::Inst_DS__DS_READ2ST64_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read2st64_b32")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ2ST64_B32

    Inst_DS__DS_READ2ST64_B32::~Inst_DS__DS_READ2ST64_B32()
    {
    } // ~Inst_DS__DS_READ2ST64_B32

    // --- description from .arch file ---
    // RETURN_DATA[0] = MEM[ADDR_BASE + OFFSET0 * 4 * 64];
    // RETURN_DATA[1] = MEM[ADDR_BASE + OFFSET1 * 4 * 64].
    // Read 2 dwords.
    void
    Inst_DS__DS_READ2ST64_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_READ2ST64_B32::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = (instData.OFFSET0 * 4 * 64);
        Addr offset1 = (instData.OFFSET1 * 4 * 64);

        initDualMemRead<VecElemU32>(gpuDynInst, offset0, offset1);
    }

    void
    Inst_DS__DS_READ2ST64_B32::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst0(gpuDynInst, extData.VDST);
        VecOperandU32 vdst1(gpuDynInst, extData.VDST + 1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 2];
                vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 2 + 1];
            }
        }

        vdst0.write();
        vdst1.write();
    }
    // --- Inst_DS__DS_READ_I8 class methods ---

    Inst_DS__DS_READ_I8::Inst_DS__DS_READ_I8(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read_i8")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ_I8

    Inst_DS__DS_READ_I8::~Inst_DS__DS_READ_I8()
    {
    } // ~Inst_DS__DS_READ_I8

    // --- description from .arch file ---
    // RETURN_DATA = signext(MEM[ADDR][7:0]).
    // Signed byte read.
    void
    Inst_DS__DS_READ_I8::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_READ_I8::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemRead<VecElemI8>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_READ_I8::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDST);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst[lane] = (VecElemU32)sext<8>((reinterpret_cast<VecElemI8*>(
                    gpuDynInst->d_data))[lane]);
            }
        }

        vdst.write();
    } // completeAcc
    // --- Inst_DS__DS_READ_U8 class methods ---

    Inst_DS__DS_READ_U8::Inst_DS__DS_READ_U8(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read_u8")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ_U8

    Inst_DS__DS_READ_U8::~Inst_DS__DS_READ_U8()
    {
    } // ~Inst_DS__DS_READ_U8

    // --- description from .arch file ---
    // RETURN_DATA = {24'h0,MEM[ADDR][7:0]}.
    // Unsigned byte read.
    void
    Inst_DS__DS_READ_U8::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_READ_U8::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemRead<VecElemU8>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_READ_U8::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDST);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst[lane] = (VecElemU32)(reinterpret_cast<VecElemU8*>(
                    gpuDynInst->d_data))[lane];
            }
        }

        vdst.write();
    } // completeAcc
    // --- Inst_DS__DS_READ_I16 class methods ---

    Inst_DS__DS_READ_I16::Inst_DS__DS_READ_I16(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read_i16")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ_I16

    Inst_DS__DS_READ_I16::~Inst_DS__DS_READ_I16()
    {
    } // ~Inst_DS__DS_READ_I16

    // --- description from .arch file ---
    // RETURN_DATA = signext(MEM[ADDR][15:0]).
    // Signed short read.
    void
    Inst_DS__DS_READ_I16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_READ_U16 class methods ---

    Inst_DS__DS_READ_U16::Inst_DS__DS_READ_U16(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read_u16")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ_U16

    Inst_DS__DS_READ_U16::~Inst_DS__DS_READ_U16()
    {
    } // ~Inst_DS__DS_READ_U16

    // --- description from .arch file ---
    // RETURN_DATA = {16'h0,MEM[ADDR][15:0]}.
    // Unsigned short read.
    void
    Inst_DS__DS_READ_U16::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute
    void
    Inst_DS__DS_READ_U16::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemRead<VecElemU16>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_READ_U16::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDST);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst[lane] = (VecElemU32)(reinterpret_cast<VecElemU16*>(
                    gpuDynInst->d_data))[lane];
            }
        }

        vdst.write();
    } // completeAcc
    // --- Inst_DS__DS_SWIZZLE_B32 class methods ---

    Inst_DS__DS_SWIZZLE_B32::Inst_DS__DS_SWIZZLE_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_swizzle_b32")
    {
        /**
         * While this operation doesn't actually use DS storage we classify
         * it as a load here because it does a writeback to a VGPR, which
         * fits in better with the LDS pipeline logic.
         */
         setFlag(Load);
    } // Inst_DS__DS_SWIZZLE_B32

    Inst_DS__DS_SWIZZLE_B32::~Inst_DS__DS_SWIZZLE_B32()
    {
    } // ~Inst_DS__DS_SWIZZLE_B32

    // --- description from .arch file ---
    // RETURN_DATA = swizzle(vgpr_data, offset1:offset0).
    // Dword swizzle, no data is written to LDS memory; See ds_opcodes.docx for
    // ---  details.
    void
    Inst_DS__DS_SWIZZLE_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        wf->decLGKMInstsIssued();

        if (gpuDynInst->exec_mask.none()) {
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()
                                ->cyclesToTicks(Cycles(24)));

        ConstVecOperandU32 data(gpuDynInst, extData.DATA0);
        VecOperandU32 vdst(gpuDynInst, extData.VDST);
        /**
         * The "DS pattern" is comprised of both offset fields. That is, the
         * swizzle pattern between lanes. Bit 15 of the DS pattern dictates
         * which swizzle mode to use. There are two different swizzle
         * patterns: 1) QDMode and 2) Bit-masks mode. If bit 15 is set use
         * QDMode else use Bit-masks mode. The remaining bits dictate how to
         * swizzle the lanes.
         *
         * QDMode:      Chunks the lanes into 4s and swizzles among them.
         *              Bits 7:6 dictate where lane 3 (of the current chunk)
         *              gets its date, 5:4 lane 2, etc.
         *
         * Bit-mask:    This mode breaks bits 14:0 into 3 equal-sized chunks.
         *              14:10 is the xor_mask, 9:5 is the or_mask, and 4:0
         *              is the and_mask. Each lane is swizzled by performing
         *              the appropriate operation using these masks.
         */
        VecElemU16 ds_pattern = ((instData.OFFSET1 << 8) | instData.OFFSET0);

        data.read();

        if (bits(ds_pattern, 15)) {
            // QDMode
            for (int lane = 0; lane < NumVecElemPerVecReg; lane += 4) {
                /**
                 * This operation allows data sharing between groups
                 * of four consecutive threads. Note the increment by
                 * 4 in the for loop.
                 */
                if (gpuDynInst->exec_mask[lane]) {
                    int index0 = lane + bits(ds_pattern, 1, 0);
                    panic_if(index0 >= NumVecElemPerVecReg, "%s: index0 (%d) "
                             "is out of bounds.\n", gpuDynInst->disassemble(),
                             index0);
                    vdst[lane]
                        = gpuDynInst->exec_mask[index0] ? data[index0]: 0;
                }
                if (gpuDynInst->exec_mask[lane + 1]) {
                    int index1 = lane + bits(ds_pattern, 3, 2);
                    panic_if(index1 >= NumVecElemPerVecReg, "%s: index1 (%d) "
                             "is out of bounds.\n", gpuDynInst->disassemble(),
                             index1);
                    vdst[lane + 1]
                        = gpuDynInst->exec_mask[index1] ? data[index1]: 0;
                }
                if (gpuDynInst->exec_mask[lane + 2]) {
                    int index2 = lane + bits(ds_pattern, 5, 4);
                    panic_if(index2 >= NumVecElemPerVecReg, "%s: index2 (%d) "
                             "is out of bounds.\n", gpuDynInst->disassemble(),
                             index2);
                    vdst[lane + 2]
                        = gpuDynInst->exec_mask[index2] ? data[index2]: 0;
                }
                if (gpuDynInst->exec_mask[lane + 3]) {
                    int index3 = lane + bits(ds_pattern, 7, 6);
                    panic_if(index3 >= NumVecElemPerVecReg, "%s: index3 (%d) "
                             "is out of bounds.\n", gpuDynInst->disassemble(),
                             index3);
                    vdst[lane + 3]
                        = gpuDynInst->exec_mask[index3] ? data[index3]: 0;
                }
            }
        } else {
            // Bit Mode
            int and_mask = bits(ds_pattern, 4, 0);
            int or_mask = bits(ds_pattern, 9, 5);
            int xor_mask = bits(ds_pattern, 14, 10);
            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    int index = (((lane & and_mask) | or_mask) ^ xor_mask);
                    // Adjust for the next 32 lanes.
                    if (lane > 31) {
                        index += 32;
                    }
                    panic_if(index >= NumVecElemPerVecReg, "%s: index (%d) is "
                             "out of bounds.\n", gpuDynInst->disassemble(),
                             index);
                    vdst[lane]
                        = gpuDynInst->exec_mask[index] ? data[index] : 0;
                }
            }
        }

        vdst.write();

        /**
         * This is needed because we treat this instruction as a load
         * but it's not an actual memory request.
         * Without this, the destination register never gets marked as
         * free, leading to a  possible deadlock
         */
        wf->computeUnit->vrf[wf->simdId]->
            scheduleWriteOperandsFromLoad(wf, gpuDynInst);
        /**
         * Similarly, this counter could build up over time, even across
         * multiple wavefronts, and cause a deadlock.
         */
        wf->rdLmReqsInPipe--;
    } // execute
    // --- Inst_DS__DS_PERMUTE_B32 class methods ---

    Inst_DS__DS_PERMUTE_B32::Inst_DS__DS_PERMUTE_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_permute_b32")
    {
        setFlag(MemoryRef);
        /**
         * While this operation doesn't actually use DS storage we classify
         * it as a load here because it does a writeback to a VGPR, which
         * fits in better with the LDS pipeline logic.
         */
         setFlag(Load);
    } // Inst_DS__DS_PERMUTE_B32

    Inst_DS__DS_PERMUTE_B32::~Inst_DS__DS_PERMUTE_B32()
    {
    } // ~Inst_DS__DS_PERMUTE_B32

    // --- description from .arch file ---
    // Forward permute.
    void
    Inst_DS__DS_PERMUTE_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        wf->decLGKMInstsIssued();

        if (gpuDynInst->exec_mask.none()) {
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()
                                ->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU32 data(gpuDynInst, extData.DATA0);
        VecOperandU32 vdst(gpuDynInst, extData.VDST);

        addr.read();
        data.read();

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                /**
                 * One of the offset fields can be used for the index.
                 * It is assumed OFFSET0 would be used, as OFFSET1 is
                 * typically only used for DS ops that operate on two
                 * disparate pieces of data.
                 */
                assert(!instData.OFFSET1);
                /**
                 * The address provided is a byte address, but VGPRs are
                 * 4 bytes, so we must divide by 4 to get the actual VGPR
                 * index. Additionally, the index is calculated modulo the
                 * WF size, 64 in this case, so we simply extract bits 7-2.
                 */
                int index = bits(addr[lane] + instData.OFFSET0, 7, 2);
                panic_if(index >= NumVecElemPerVecReg, "%s: index (%d) is out "
                         "of bounds.\n", gpuDynInst->disassemble(), index);
                /**
                 * If the shuffled index corresponds to a lane that is
                 * inactive then this instruction writes a 0 to the active
                 * lane in VDST.
                 */
                if (wf->execMask(index)) {
                    vdst[index] = data[lane];
                } else {
                    vdst[index] = 0;
                }
            }
        }

        vdst.write();

        /**
         * This is needed because we treat this instruction as a load
         * but it's not an actual memory request.
         * Without this, the destination register never gets marked as
         * free, leading to a  possible deadlock
         */
        wf->computeUnit->vrf[wf->simdId]->
            scheduleWriteOperandsFromLoad(wf, gpuDynInst);
        /**
         * Similarly, this counter could build up over time, even across
         * multiple wavefronts, and cause a deadlock.
         */
        wf->rdLmReqsInPipe--;
    } // execute
    // --- Inst_DS__DS_BPERMUTE_B32 class methods ---

    Inst_DS__DS_BPERMUTE_B32::Inst_DS__DS_BPERMUTE_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_bpermute_b32")
    {
        setFlag(MemoryRef);
        /**
         * While this operation doesn't actually use DS storage we classify
         * it as a load here because it does a writeback to a VGPR, which
         * fits in better with the LDS pipeline logic.
         */
        setFlag(Load);
    } // Inst_DS__DS_BPERMUTE_B32

    Inst_DS__DS_BPERMUTE_B32::~Inst_DS__DS_BPERMUTE_B32()
    {
    } // ~Inst_DS__DS_BPERMUTE_B32

    // --- description from .arch file ---
    // Backward permute.
    void
    Inst_DS__DS_BPERMUTE_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        wf->decLGKMInstsIssued();

        if (gpuDynInst->exec_mask.none()) {
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()
                                ->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU32 data(gpuDynInst, extData.DATA0);
        VecOperandU32 vdst(gpuDynInst, extData.VDST);

        addr.read();
        data.read();

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                /**
                 * One of the offset fields can be used for the index.
                 * It is assumed OFFSET0 would be used, as OFFSET1 is
                 * typically only used for DS ops that operate on two
                 * disparate pieces of data.
                 */
                assert(!instData.OFFSET1);
                /**
                 * The address provided is a byte address, but VGPRs are
                 * 4 bytes, so we must divide by 4 to get the actual VGPR
                 * index. Additionally, the index is calculated modulo the
                 * WF size, 64 in this case, so we simply extract bits 7-2.
                 */
                int index = bits(addr[lane] + instData.OFFSET0, 7, 2);
                panic_if(index >= NumVecElemPerVecReg, "%s: index (%d) is out "
                         "of bounds.\n", gpuDynInst->disassemble(), index);
                /**
                 * If the shuffled index corresponds to a lane that is
                 * inactive then this instruction writes a 0 to the active
                 * lane in VDST.
                 */
                if (wf->execMask(index)) {
                    vdst[lane] = data[index];
                } else {
                    vdst[lane] = 0;
                }
            }
        }

        vdst.write();

        /**
         * This is needed because we treat this instruction as a load
         * but it's not an actual memory request.
         * Without this, the destination register never gets marked as
         * free, leading to a  possible deadlock
         */
        wf->computeUnit->vrf[wf->simdId]->
            scheduleWriteOperandsFromLoad(wf, gpuDynInst);
        /**
         * Similarly, this counter could build up over time, even across
         * multiple wavefronts, and cause a deadlock.
         */
        wf->rdLmReqsInPipe--;
    } // execute

    // --- Inst_DS__DS_ADD_U64 class methods ---

    Inst_DS__DS_ADD_U64::Inst_DS__DS_ADD_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_add_u64")
    {
        setFlag(MemoryRef);
        setFlag(GroupSegment);
        setFlag(AtomicAdd);
        setFlag(AtomicNoReturn);
    } // Inst_DS__DS_ADD_U64

    Inst_DS__DS_ADD_U64::~Inst_DS__DS_ADD_U64()
    {
    } // ~Inst_DS__DS_ADD_U64

    // --- description from .arch file ---
    // 64b:
    // MEM[ADDR] += DATA[0:1];
    void
    Inst_DS__DS_ADD_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU64 data(gpuDynInst, extData.DATA0);

        addr.read();
        data.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU64*>(gpuDynInst->a_data))[lane]
                    = data[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_ADD_U64::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initAtomicAccess<VecElemU64>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_ADD_U64::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_DS__DS_SUB_U64 class methods ---

    Inst_DS__DS_SUB_U64::Inst_DS__DS_SUB_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_sub_u64")
    {
    } // Inst_DS__DS_SUB_U64

    Inst_DS__DS_SUB_U64::~Inst_DS__DS_SUB_U64()
    {
    } // ~Inst_DS__DS_SUB_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_SUB_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_RSUB_U64 class methods ---

    Inst_DS__DS_RSUB_U64::Inst_DS__DS_RSUB_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_rsub_u64")
    {
    } // Inst_DS__DS_RSUB_U64

    Inst_DS__DS_RSUB_U64::~Inst_DS__DS_RSUB_U64()
    {
    } // ~Inst_DS__DS_RSUB_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = DATA - MEM[ADDR];
    // RETURN_DATA = tmp.
    // Subtraction with reversed operands.
    void
    Inst_DS__DS_RSUB_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_INC_U64 class methods ---

    Inst_DS__DS_INC_U64::Inst_DS__DS_INC_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_inc_u64")
    {
    } // Inst_DS__DS_INC_U64

    Inst_DS__DS_INC_U64::~Inst_DS__DS_INC_U64()
    {
    } // ~Inst_DS__DS_INC_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp >= DATA[0:1]) ? 0 : tmp + 1 (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_INC_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_DEC_U64 class methods ---

    Inst_DS__DS_DEC_U64::Inst_DS__DS_DEC_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_dec_u64")
    {
    } // Inst_DS__DS_DEC_U64

    Inst_DS__DS_DEC_U64::~Inst_DS__DS_DEC_U64()
    {
    } // ~Inst_DS__DS_DEC_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp == 0 || tmp > DATA[0:1]) ? DATA[0:1] : tmp - 1
    // (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_DEC_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_I64 class methods ---

    Inst_DS__DS_MIN_I64::Inst_DS__DS_MIN_I64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_i64")
    {
    } // Inst_DS__DS_MIN_I64

    Inst_DS__DS_MIN_I64::~Inst_DS__DS_MIN_I64()
    {
    } // ~Inst_DS__DS_MIN_I64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] < tmp) ? DATA[0:1] : tmp (signed compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_MIN_I64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_I64 class methods ---

    Inst_DS__DS_MAX_I64::Inst_DS__DS_MAX_I64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_i64")
    {
    } // Inst_DS__DS_MAX_I64

    Inst_DS__DS_MAX_I64::~Inst_DS__DS_MAX_I64()
    {
    } // ~Inst_DS__DS_MAX_I64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] > tmp) ? DATA[0:1] : tmp (signed compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_MAX_I64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_U64 class methods ---

    Inst_DS__DS_MIN_U64::Inst_DS__DS_MIN_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_u64")
    {
    } // Inst_DS__DS_MIN_U64

    Inst_DS__DS_MIN_U64::~Inst_DS__DS_MIN_U64()
    {
    } // ~Inst_DS__DS_MIN_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] < tmp) ? DATA[0:1] : tmp (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_MIN_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_U64 class methods ---

    Inst_DS__DS_MAX_U64::Inst_DS__DS_MAX_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_u64")
    {
    } // Inst_DS__DS_MAX_U64

    Inst_DS__DS_MAX_U64::~Inst_DS__DS_MAX_U64()
    {
    } // ~Inst_DS__DS_MAX_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] > tmp) ? DATA[0:1] : tmp (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_MAX_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_AND_B64 class methods ---

    Inst_DS__DS_AND_B64::Inst_DS__DS_AND_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_and_b64")
    {
    } // Inst_DS__DS_AND_B64

    Inst_DS__DS_AND_B64::~Inst_DS__DS_AND_B64()
    {
    } // ~Inst_DS__DS_AND_B64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] &= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_AND_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_OR_B64 class methods ---

    Inst_DS__DS_OR_B64::Inst_DS__DS_OR_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_or_b64")
    {
    } // Inst_DS__DS_OR_B64

    Inst_DS__DS_OR_B64::~Inst_DS__DS_OR_B64()
    {
    } // ~Inst_DS__DS_OR_B64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] |= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_OR_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_XOR_B64 class methods ---

    Inst_DS__DS_XOR_B64::Inst_DS__DS_XOR_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_xor_b64")
    {
    } // Inst_DS__DS_XOR_B64

    Inst_DS__DS_XOR_B64::~Inst_DS__DS_XOR_B64()
    {
    } // ~Inst_DS__DS_XOR_B64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] ^= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_XOR_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MSKOR_B64 class methods ---

    Inst_DS__DS_MSKOR_B64::Inst_DS__DS_MSKOR_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_mskor_b64")
    {
    } // Inst_DS__DS_MSKOR_B64

    Inst_DS__DS_MSKOR_B64::~Inst_DS__DS_MSKOR_B64()
    {
    } // ~Inst_DS__DS_MSKOR_B64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (MEM_ADDR[ADDR] & ~DATA) | DATA2;
    // RETURN_DATA = tmp.
    // Masked dword OR, D0 contains the mask and D1 contains the new value.
    void
    Inst_DS__DS_MSKOR_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRITE_B64 class methods ---

    Inst_DS__DS_WRITE_B64::Inst_DS__DS_WRITE_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write_b64")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE_B64

    Inst_DS__DS_WRITE_B64::~Inst_DS__DS_WRITE_B64()
    {
    } // ~Inst_DS__DS_WRITE_B64

    // --- description from .arch file ---
    // 64b:
    // MEM[ADDR] = DATA.
    // Write qword.
    void
    Inst_DS__DS_WRITE_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU64 data(gpuDynInst, extData.DATA0);

        addr.read();
        data.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU64*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE_B64::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemWrite<VecElemU64>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_WRITE_B64::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_DS__DS_WRITE2_B64 class methods ---

    Inst_DS__DS_WRITE2_B64::Inst_DS__DS_WRITE2_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write2_b64")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE2_B64

    Inst_DS__DS_WRITE2_B64::~Inst_DS__DS_WRITE2_B64()
    {
    } // ~Inst_DS__DS_WRITE2_B64

    // --- description from .arch file ---
    // 64b:
    // MEM[ADDR_BASE + OFFSET0 * 8] = DATA;
    // MEM[ADDR_BASE + OFFSET1 * 8] = DATA2.
    // Write 2 qwords.
    void
    Inst_DS__DS_WRITE2_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU64 data0(gpuDynInst, extData.DATA0);
        ConstVecOperandU64 data1(gpuDynInst, extData.DATA1);

        addr.read();
        data0.read();
        data1.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU64*>(
                    gpuDynInst->d_data))[lane * 2] = data0[lane];
                (reinterpret_cast<VecElemU64*>(
                    gpuDynInst->d_data))[lane * 2 + 1] = data1[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE2_B64::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0 * 8;
        Addr offset1 = instData.OFFSET1 * 8;

        initDualMemWrite<VecElemU64>(gpuDynInst, offset0, offset1);
    }

    void
    Inst_DS__DS_WRITE2_B64::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    }
    // --- Inst_DS__DS_WRITE2ST64_B64 class methods ---

    Inst_DS__DS_WRITE2ST64_B64::Inst_DS__DS_WRITE2ST64_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write2st64_b64")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE2ST64_B64

    Inst_DS__DS_WRITE2ST64_B64::~Inst_DS__DS_WRITE2ST64_B64()
    {
    } // ~Inst_DS__DS_WRITE2ST64_B64

    // --- description from .arch file ---
    // 64b:
    // MEM[ADDR_BASE + OFFSET0 * 8 * 64] = DATA;
    // MEM[ADDR_BASE + OFFSET1 * 8 * 64] = DATA2;
    // Write 2 qwords.
    void
    Inst_DS__DS_WRITE2ST64_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU64 data0(gpuDynInst, extData.DATA0);
        ConstVecOperandU64 data1(gpuDynInst, extData.DATA1);

        addr.read();
        data0.read();
        data1.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU64*>(
                    gpuDynInst->d_data))[lane * 2] = data0[lane];
                (reinterpret_cast<VecElemU64*>(
                    gpuDynInst->d_data))[lane * 2 + 1] = data1[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE2ST64_B64::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0 * 8 * 64;
        Addr offset1 = instData.OFFSET1 * 8 * 64;

        initDualMemWrite<VecElemU64>(gpuDynInst, offset0, offset1);
    }

    void
    Inst_DS__DS_WRITE2ST64_B64::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    }
    // --- Inst_DS__DS_CMPST_B64 class methods ---

    Inst_DS__DS_CMPST_B64::Inst_DS__DS_CMPST_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_cmpst_b64")
    {
    } // Inst_DS__DS_CMPST_B64

    Inst_DS__DS_CMPST_B64::~Inst_DS__DS_CMPST_B64()
    {
    } // ~Inst_DS__DS_CMPST_B64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // src = DATA2;
    // cmp = DATA;
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0] = tmp.
    // Compare and store.
    // Caution, the order of src and cmp are the *opposite* of the
    // ---  BUFFER_ATOMIC_CMPSWAP_X2 opcode.
    void
    Inst_DS__DS_CMPST_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_CMPST_F64 class methods ---

    Inst_DS__DS_CMPST_F64::Inst_DS__DS_CMPST_F64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_cmpst_f64")
    {
        setFlag(F64);
    } // Inst_DS__DS_CMPST_F64

    Inst_DS__DS_CMPST_F64::~Inst_DS__DS_CMPST_F64()
    {
    } // ~Inst_DS__DS_CMPST_F64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // src = DATA2;
    // cmp = DATA;
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0] = tmp.
    // Floating point compare and store that handles NaN/INF/denormal values.
    // Caution, the order of src and cmp are the *opposite* of the
    // ---  BUFFER_ATOMIC_FCMPSWAP_X2 opcode.
    void
    Inst_DS__DS_CMPST_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_F64 class methods ---

    Inst_DS__DS_MIN_F64::Inst_DS__DS_MIN_F64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_f64")
    {
        setFlag(F64);
    } // Inst_DS__DS_MIN_F64

    Inst_DS__DS_MIN_F64::~Inst_DS__DS_MIN_F64()
    {
    } // ~Inst_DS__DS_MIN_F64

    // --- description from .arch file ---
    // 64b.
    // tmp = MEM[ADDR];
    // src = DATA;
    // cmp = DATA2;
    // MEM[ADDR] = (cmp < tmp) ? src : tmp.
    // Floating point minimum that handles NaN/INF/denormal values.
    // Note that this opcode is slightly more general-purpose than
    // ---  BUFFER_ATOMIC_FMIN_X2.
    void
    Inst_DS__DS_MIN_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_F64 class methods ---

    Inst_DS__DS_MAX_F64::Inst_DS__DS_MAX_F64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_f64")
    {
        setFlag(F64);
    } // Inst_DS__DS_MAX_F64

    Inst_DS__DS_MAX_F64::~Inst_DS__DS_MAX_F64()
    {
    } // ~Inst_DS__DS_MAX_F64

    // --- description from .arch file ---
    // 64b.
    // tmp = MEM[ADDR];
    // src = DATA;
    // cmp = DATA2;
    // MEM[ADDR] = (tmp > cmp) ? src : tmp.
    // Floating point maximum that handles NaN/INF/denormal values.
    // Note that this opcode is slightly more general-purpose than
    // ---  BUFFER_ATOMIC_FMAX_X2.
    void
    Inst_DS__DS_MAX_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_ADD_RTN_U64 class methods ---

    Inst_DS__DS_ADD_RTN_U64::Inst_DS__DS_ADD_RTN_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_add_rtn_u64")
    {
    } // Inst_DS__DS_ADD_RTN_U64

    Inst_DS__DS_ADD_RTN_U64::~Inst_DS__DS_ADD_RTN_U64()
    {
    } // ~Inst_DS__DS_ADD_RTN_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] += DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_ADD_RTN_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_SUB_RTN_U64 class methods ---

    Inst_DS__DS_SUB_RTN_U64::Inst_DS__DS_SUB_RTN_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_sub_rtn_u64")
    {
    } // Inst_DS__DS_SUB_RTN_U64

    Inst_DS__DS_SUB_RTN_U64::~Inst_DS__DS_SUB_RTN_U64()
    {
    } // ~Inst_DS__DS_SUB_RTN_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_SUB_RTN_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_RSUB_RTN_U64 class methods ---

    Inst_DS__DS_RSUB_RTN_U64::Inst_DS__DS_RSUB_RTN_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_rsub_rtn_u64")
    {
    } // Inst_DS__DS_RSUB_RTN_U64

    Inst_DS__DS_RSUB_RTN_U64::~Inst_DS__DS_RSUB_RTN_U64()
    {
    } // ~Inst_DS__DS_RSUB_RTN_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = DATA - MEM[ADDR];
    // RETURN_DATA = tmp.
    // Subtraction with reversed operands.
    void
    Inst_DS__DS_RSUB_RTN_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_INC_RTN_U64 class methods ---

    Inst_DS__DS_INC_RTN_U64::Inst_DS__DS_INC_RTN_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_inc_rtn_u64")
    {
    } // Inst_DS__DS_INC_RTN_U64

    Inst_DS__DS_INC_RTN_U64::~Inst_DS__DS_INC_RTN_U64()
    {
    } // ~Inst_DS__DS_INC_RTN_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp >= DATA[0:1]) ? 0 : tmp + 1 (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_INC_RTN_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_DEC_RTN_U64 class methods ---

    Inst_DS__DS_DEC_RTN_U64::Inst_DS__DS_DEC_RTN_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_dec_rtn_u64")
    {
    } // Inst_DS__DS_DEC_RTN_U64

    Inst_DS__DS_DEC_RTN_U64::~Inst_DS__DS_DEC_RTN_U64()
    {
    } // ~Inst_DS__DS_DEC_RTN_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp == 0 || tmp > DATA[0:1]) ? DATA[0:1] : tmp - 1
    // (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_DEC_RTN_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_RTN_I64 class methods ---

    Inst_DS__DS_MIN_RTN_I64::Inst_DS__DS_MIN_RTN_I64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_rtn_i64")
    {
    } // Inst_DS__DS_MIN_RTN_I64

    Inst_DS__DS_MIN_RTN_I64::~Inst_DS__DS_MIN_RTN_I64()
    {
    } // ~Inst_DS__DS_MIN_RTN_I64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] < tmp) ? DATA[0:1] : tmp (signed compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_MIN_RTN_I64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_RTN_I64 class methods ---

    Inst_DS__DS_MAX_RTN_I64::Inst_DS__DS_MAX_RTN_I64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_rtn_i64")
    {
    } // Inst_DS__DS_MAX_RTN_I64

    Inst_DS__DS_MAX_RTN_I64::~Inst_DS__DS_MAX_RTN_I64()
    {
    } // ~Inst_DS__DS_MAX_RTN_I64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] > tmp) ? DATA[0:1] : tmp (signed compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_MAX_RTN_I64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_RTN_U64 class methods ---

    Inst_DS__DS_MIN_RTN_U64::Inst_DS__DS_MIN_RTN_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_rtn_u64")
    {
    } // Inst_DS__DS_MIN_RTN_U64

    Inst_DS__DS_MIN_RTN_U64::~Inst_DS__DS_MIN_RTN_U64()
    {
    } // ~Inst_DS__DS_MIN_RTN_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] < tmp) ? DATA[0:1] : tmp (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_MIN_RTN_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_RTN_U64 class methods ---

    Inst_DS__DS_MAX_RTN_U64::Inst_DS__DS_MAX_RTN_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_rtn_u64")
    {
    } // Inst_DS__DS_MAX_RTN_U64

    Inst_DS__DS_MAX_RTN_U64::~Inst_DS__DS_MAX_RTN_U64()
    {
    } // ~Inst_DS__DS_MAX_RTN_U64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] > tmp) ? DATA[0:1] : tmp (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_MAX_RTN_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_AND_RTN_B64 class methods ---

    Inst_DS__DS_AND_RTN_B64::Inst_DS__DS_AND_RTN_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_and_rtn_b64")
    {
    } // Inst_DS__DS_AND_RTN_B64

    Inst_DS__DS_AND_RTN_B64::~Inst_DS__DS_AND_RTN_B64()
    {
    } // ~Inst_DS__DS_AND_RTN_B64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] &= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_AND_RTN_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_OR_RTN_B64 class methods ---

    Inst_DS__DS_OR_RTN_B64::Inst_DS__DS_OR_RTN_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_or_rtn_b64")
    {
    } // Inst_DS__DS_OR_RTN_B64

    Inst_DS__DS_OR_RTN_B64::~Inst_DS__DS_OR_RTN_B64()
    {
    } // ~Inst_DS__DS_OR_RTN_B64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] |= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_OR_RTN_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_XOR_RTN_B64 class methods ---

    Inst_DS__DS_XOR_RTN_B64::Inst_DS__DS_XOR_RTN_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_xor_rtn_b64")
    {
    } // Inst_DS__DS_XOR_RTN_B64

    Inst_DS__DS_XOR_RTN_B64::~Inst_DS__DS_XOR_RTN_B64()
    {
    } // ~Inst_DS__DS_XOR_RTN_B64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] ^= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_DS__DS_XOR_RTN_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MSKOR_RTN_B64 class methods ---

    Inst_DS__DS_MSKOR_RTN_B64::Inst_DS__DS_MSKOR_RTN_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_mskor_rtn_b64")
    {
    } // Inst_DS__DS_MSKOR_RTN_B64

    Inst_DS__DS_MSKOR_RTN_B64::~Inst_DS__DS_MSKOR_RTN_B64()
    {
    } // ~Inst_DS__DS_MSKOR_RTN_B64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (MEM_ADDR[ADDR] & ~DATA) | DATA2;
    // RETURN_DATA = tmp.
    // Masked dword OR, D0 contains the mask and D1 contains the new value.
    void
    Inst_DS__DS_MSKOR_RTN_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRXCHG_RTN_B64 class methods ---

    Inst_DS__DS_WRXCHG_RTN_B64::Inst_DS__DS_WRXCHG_RTN_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_wrxchg_rtn_b64")
    {
    } // Inst_DS__DS_WRXCHG_RTN_B64

    Inst_DS__DS_WRXCHG_RTN_B64::~Inst_DS__DS_WRXCHG_RTN_B64()
    {
    } // ~Inst_DS__DS_WRXCHG_RTN_B64

    // --- description from .arch file ---
    // tmp = MEM[ADDR];
    // MEM[ADDR] = DATA;
    // RETURN_DATA = tmp.
    // Write-exchange operation.
    void
    Inst_DS__DS_WRXCHG_RTN_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRXCHG2_RTN_B64 class methods ---

    Inst_DS__DS_WRXCHG2_RTN_B64::Inst_DS__DS_WRXCHG2_RTN_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_wrxchg2_rtn_b64")
    {
    } // Inst_DS__DS_WRXCHG2_RTN_B64

    Inst_DS__DS_WRXCHG2_RTN_B64::~Inst_DS__DS_WRXCHG2_RTN_B64()
    {
    } // ~Inst_DS__DS_WRXCHG2_RTN_B64

    // --- description from .arch file ---
    // Write-exchange 2 separate qwords.
    void
    Inst_DS__DS_WRXCHG2_RTN_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRXCHG2ST64_RTN_B64 class methods ---

    Inst_DS__DS_WRXCHG2ST64_RTN_B64::Inst_DS__DS_WRXCHG2ST64_RTN_B64(
          InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_wrxchg2st64_rtn_b64")
    {
    } // Inst_DS__DS_WRXCHG2ST64_RTN_B64

    Inst_DS__DS_WRXCHG2ST64_RTN_B64::~Inst_DS__DS_WRXCHG2ST64_RTN_B64()
    {
    } // ~Inst_DS__DS_WRXCHG2ST64_RTN_B64

    // --- description from .arch file ---
    // Write-exchange 2 qwords with a stride of 64 qwords.
    void
    Inst_DS__DS_WRXCHG2ST64_RTN_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_CMPST_RTN_B64 class methods ---

    Inst_DS__DS_CMPST_RTN_B64::Inst_DS__DS_CMPST_RTN_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_cmpst_rtn_b64")
    {
    } // Inst_DS__DS_CMPST_RTN_B64

    Inst_DS__DS_CMPST_RTN_B64::~Inst_DS__DS_CMPST_RTN_B64()
    {
    } // ~Inst_DS__DS_CMPST_RTN_B64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // src = DATA2;
    // cmp = DATA;
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0] = tmp.
    // Compare and store.
    // Caution, the order of src and cmp are the *opposite* of the
    // ---  BUFFER_ATOMIC_CMPSWAP_X2 opcode.
    void
    Inst_DS__DS_CMPST_RTN_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_CMPST_RTN_F64 class methods ---

    Inst_DS__DS_CMPST_RTN_F64::Inst_DS__DS_CMPST_RTN_F64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_cmpst_rtn_f64")
    {
        setFlag(F64);
    } // Inst_DS__DS_CMPST_RTN_F64

    Inst_DS__DS_CMPST_RTN_F64::~Inst_DS__DS_CMPST_RTN_F64()
    {
    } // ~Inst_DS__DS_CMPST_RTN_F64

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // src = DATA2;
    // cmp = DATA;
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0] = tmp.
    // Floating point compare and store that handles NaN/INF/denormal values.
    // Caution, the order of src and cmp are the *opposite* of the
    // ---  BUFFER_ATOMIC_FCMPSWAP_X2 opcode.
    void
    Inst_DS__DS_CMPST_RTN_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_RTN_F64 class methods ---

    Inst_DS__DS_MIN_RTN_F64::Inst_DS__DS_MIN_RTN_F64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_rtn_f64")
    {
        setFlag(F64);
    } // Inst_DS__DS_MIN_RTN_F64

    Inst_DS__DS_MIN_RTN_F64::~Inst_DS__DS_MIN_RTN_F64()
    {
    } // ~Inst_DS__DS_MIN_RTN_F64

    // --- description from .arch file ---
    // 64b.
    // tmp = MEM[ADDR];
    // src = DATA;
    // cmp = DATA2;
    // MEM[ADDR] = (cmp < tmp) ? src : tmp.
    // Floating point minimum that handles NaN/INF/denormal values.
    // Note that this opcode is slightly more general-purpose than
    // ---  BUFFER_ATOMIC_FMIN_X2.
    void
    Inst_DS__DS_MIN_RTN_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_RTN_F64 class methods ---

    Inst_DS__DS_MAX_RTN_F64::Inst_DS__DS_MAX_RTN_F64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_rtn_f64")
    {
        setFlag(F64);
    } // Inst_DS__DS_MAX_RTN_F64

    Inst_DS__DS_MAX_RTN_F64::~Inst_DS__DS_MAX_RTN_F64()
    {
    } // ~Inst_DS__DS_MAX_RTN_F64

    // --- description from .arch file ---
    // 64b.
    // tmp = MEM[ADDR];
    // src = DATA;
    // cmp = DATA2;
    // MEM[ADDR] = (tmp > cmp) ? src : tmp.
    // Floating point maximum that handles NaN/INF/denormal values.
    // Note that this opcode is slightly more general-purpose than
    // ---  BUFFER_ATOMIC_FMAX_X2.
    void
    Inst_DS__DS_MAX_RTN_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_READ_B64 class methods ---

    Inst_DS__DS_READ_B64::Inst_DS__DS_READ_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read_b64")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ_B64

    Inst_DS__DS_READ_B64::~Inst_DS__DS_READ_B64()
    {
    } // ~Inst_DS__DS_READ_B64

    // --- description from .arch file ---
    // RETURN_DATA = MEM[ADDR].
    // Read 1 qword.
    void
    Inst_DS__DS_READ_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_READ_B64::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemRead<VecElemU64>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_READ_B64::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU64 vdst(gpuDynInst, extData.VDST);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst[lane] = (reinterpret_cast<VecElemU64*>(
                    gpuDynInst->d_data))[lane];
            }
        }

        vdst.write();
    } // completeAcc
    // --- Inst_DS__DS_READ2_B64 class methods ---

    Inst_DS__DS_READ2_B64::Inst_DS__DS_READ2_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read2_b64")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ2_B64

    Inst_DS__DS_READ2_B64::~Inst_DS__DS_READ2_B64()
    {
    } // ~Inst_DS__DS_READ2_B64

    // --- description from .arch file ---
    // RETURN_DATA[0] = MEM[ADDR_BASE + OFFSET0 * 8];
    // RETURN_DATA[1] = MEM[ADDR_BASE + OFFSET1 * 8].
    // Read 2 qwords.
    void
    Inst_DS__DS_READ2_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_READ2_B64::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0 * 8;
        Addr offset1 = instData.OFFSET1 * 8;

        initDualMemRead<VecElemU64>(gpuDynInst, offset0, offset1);
    } // initiateAcc

    void
    Inst_DS__DS_READ2_B64::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU64 vdst0(gpuDynInst, extData.VDST);
        VecOperandU64 vdst1(gpuDynInst, extData.VDST + 2);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst0[lane] = (reinterpret_cast<VecElemU64*>(
                    gpuDynInst->d_data))[lane * 2];
                vdst1[lane] = (reinterpret_cast<VecElemU64*>(
                    gpuDynInst->d_data))[lane * 2 + 1];
            }
        }

        vdst0.write();
        vdst1.write();
    } // completeAcc
    // --- Inst_DS__DS_READ2ST64_B64 class methods ---

    Inst_DS__DS_READ2ST64_B64::Inst_DS__DS_READ2ST64_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read2st64_b64")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ2ST64_B64

    Inst_DS__DS_READ2ST64_B64::~Inst_DS__DS_READ2ST64_B64()
    {
    } // ~Inst_DS__DS_READ2ST64_B64

    // --- description from .arch file ---
    // RETURN_DATA[0] = MEM[ADDR_BASE + OFFSET0 * 8 * 64];
    // RETURN_DATA[1] = MEM[ADDR_BASE + OFFSET1 * 8 * 64].
    // Read 2 qwords.
    void
    Inst_DS__DS_READ2ST64_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decLGKMInstsIssued();
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_READ2ST64_B64::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = (instData.OFFSET0 * 8 * 64);
        Addr offset1 = (instData.OFFSET1 * 8 * 64);

        initDualMemRead<VecElemU64>(gpuDynInst, offset0, offset1);
    }

    void
    Inst_DS__DS_READ2ST64_B64::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU64 vdst0(gpuDynInst, extData.VDST);
        VecOperandU64 vdst1(gpuDynInst, extData.VDST + 2);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst0[lane] = (reinterpret_cast<VecElemU64*>(
                    gpuDynInst->d_data))[lane * 2];
                vdst1[lane] = (reinterpret_cast<VecElemU64*>(
                    gpuDynInst->d_data))[lane * 2 + 1];
            }
        }

        vdst0.write();
        vdst1.write();
    }
    // --- Inst_DS__DS_CONDXCHG32_RTN_B64 class methods ---

    Inst_DS__DS_CONDXCHG32_RTN_B64::Inst_DS__DS_CONDXCHG32_RTN_B64(
          InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_condxchg32_rtn_b64")
    {
    } // Inst_DS__DS_CONDXCHG32_RTN_B64

    Inst_DS__DS_CONDXCHG32_RTN_B64::~Inst_DS__DS_CONDXCHG32_RTN_B64()
    {
    } // ~Inst_DS__DS_CONDXCHG32_RTN_B64

    // --- description from .arch file ---
    // Conditional write exchange.
    void
    Inst_DS__DS_CONDXCHG32_RTN_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_ADD_SRC2_U32 class methods ---

    Inst_DS__DS_ADD_SRC2_U32::Inst_DS__DS_ADD_SRC2_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_add_src2_u32")
    {
    } // Inst_DS__DS_ADD_SRC2_U32

    Inst_DS__DS_ADD_SRC2_U32::~Inst_DS__DS_ADD_SRC2_U32()
    {
    } // ~Inst_DS__DS_ADD_SRC2_U32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[A] + MEM[B].
    void
    Inst_DS__DS_ADD_SRC2_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_SUB_SRC2_U32 class methods ---

    Inst_DS__DS_SUB_SRC2_U32::Inst_DS__DS_SUB_SRC2_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_sub_src2_u32")
    {
    } // Inst_DS__DS_SUB_SRC2_U32

    Inst_DS__DS_SUB_SRC2_U32::~Inst_DS__DS_SUB_SRC2_U32()
    {
    } // ~Inst_DS__DS_SUB_SRC2_U32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[A] - MEM[B].
    void
    Inst_DS__DS_SUB_SRC2_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_RSUB_SRC2_U32 class methods ---

    Inst_DS__DS_RSUB_SRC2_U32::Inst_DS__DS_RSUB_SRC2_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_rsub_src2_u32")
    {
    } // Inst_DS__DS_RSUB_SRC2_U32

    Inst_DS__DS_RSUB_SRC2_U32::~Inst_DS__DS_RSUB_SRC2_U32()
    {
    } // ~Inst_DS__DS_RSUB_SRC2_U32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[B] - MEM[A].
    void
    Inst_DS__DS_RSUB_SRC2_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_INC_SRC2_U32 class methods ---

    Inst_DS__DS_INC_SRC2_U32::Inst_DS__DS_INC_SRC2_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_inc_src2_u32")
    {
    } // Inst_DS__DS_INC_SRC2_U32

    Inst_DS__DS_INC_SRC2_U32::~Inst_DS__DS_INC_SRC2_U32()
    {
    } // ~Inst_DS__DS_INC_SRC2_U32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = (MEM[A] >= MEM[B] ? 0 : MEM[A] + 1).
    void
    Inst_DS__DS_INC_SRC2_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_DEC_SRC2_U32 class methods ---

    Inst_DS__DS_DEC_SRC2_U32::Inst_DS__DS_DEC_SRC2_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_dec_src2_u32")
    {
    } // Inst_DS__DS_DEC_SRC2_U32

    Inst_DS__DS_DEC_SRC2_U32::~Inst_DS__DS_DEC_SRC2_U32()
    {
    } // ~Inst_DS__DS_DEC_SRC2_U32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = (MEM[A] == 0 || MEM[A] > MEM[B] ? MEM[B] : MEM[A] - 1).
    // Uint decrement.
    void
    Inst_DS__DS_DEC_SRC2_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_SRC2_I32 class methods ---

    Inst_DS__DS_MIN_SRC2_I32::Inst_DS__DS_MIN_SRC2_I32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_src2_i32")
    {
    } // Inst_DS__DS_MIN_SRC2_I32

    Inst_DS__DS_MIN_SRC2_I32::~Inst_DS__DS_MIN_SRC2_I32()
    {
    } // ~Inst_DS__DS_MIN_SRC2_I32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = min(MEM[A], MEM[B]).
    void
    Inst_DS__DS_MIN_SRC2_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_SRC2_I32 class methods ---

    Inst_DS__DS_MAX_SRC2_I32::Inst_DS__DS_MAX_SRC2_I32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_src2_i32")
    {
    } // Inst_DS__DS_MAX_SRC2_I32

    Inst_DS__DS_MAX_SRC2_I32::~Inst_DS__DS_MAX_SRC2_I32()
    {
    } // ~Inst_DS__DS_MAX_SRC2_I32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = max(MEM[A], MEM[B]).
    void
    Inst_DS__DS_MAX_SRC2_I32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_SRC2_U32 class methods ---

    Inst_DS__DS_MIN_SRC2_U32::Inst_DS__DS_MIN_SRC2_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_src2_u32")
    {
    } // Inst_DS__DS_MIN_SRC2_U32

    Inst_DS__DS_MIN_SRC2_U32::~Inst_DS__DS_MIN_SRC2_U32()
    {
    } // ~Inst_DS__DS_MIN_SRC2_U32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = min(MEM[A], MEM[B]).
    void
    Inst_DS__DS_MIN_SRC2_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_SRC2_U32 class methods ---

    Inst_DS__DS_MAX_SRC2_U32::Inst_DS__DS_MAX_SRC2_U32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_src2_u32")
    {
    } // Inst_DS__DS_MAX_SRC2_U32

    Inst_DS__DS_MAX_SRC2_U32::~Inst_DS__DS_MAX_SRC2_U32()
    {
    } // ~Inst_DS__DS_MAX_SRC2_U32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = max(MEM[A], MEM[B]).
    void
    Inst_DS__DS_MAX_SRC2_U32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_AND_SRC2_B32 class methods ---

    Inst_DS__DS_AND_SRC2_B32::Inst_DS__DS_AND_SRC2_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_and_src2_b32")
    {
    } // Inst_DS__DS_AND_SRC2_B32

    Inst_DS__DS_AND_SRC2_B32::~Inst_DS__DS_AND_SRC2_B32()
    {
    } // ~Inst_DS__DS_AND_SRC2_B32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[A] & MEM[B].
    void
    Inst_DS__DS_AND_SRC2_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_OR_SRC2_B32 class methods ---

    Inst_DS__DS_OR_SRC2_B32::Inst_DS__DS_OR_SRC2_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_or_src2_b32")
    {
    } // Inst_DS__DS_OR_SRC2_B32

    Inst_DS__DS_OR_SRC2_B32::~Inst_DS__DS_OR_SRC2_B32()
    {
    } // ~Inst_DS__DS_OR_SRC2_B32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[A] | MEM[B].
    void
    Inst_DS__DS_OR_SRC2_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_XOR_SRC2_B32 class methods ---

    Inst_DS__DS_XOR_SRC2_B32::Inst_DS__DS_XOR_SRC2_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_xor_src2_b32")
    {
    } // Inst_DS__DS_XOR_SRC2_B32

    Inst_DS__DS_XOR_SRC2_B32::~Inst_DS__DS_XOR_SRC2_B32()
    {
    } // ~Inst_DS__DS_XOR_SRC2_B32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[A] ^ MEM[B].
    void
    Inst_DS__DS_XOR_SRC2_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRITE_SRC2_B32 class methods ---

    Inst_DS__DS_WRITE_SRC2_B32::Inst_DS__DS_WRITE_SRC2_B32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write_src2_b32")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE_SRC2_B32

    Inst_DS__DS_WRITE_SRC2_B32::~Inst_DS__DS_WRITE_SRC2_B32()
    {
    } // ~Inst_DS__DS_WRITE_SRC2_B32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[B].
    // Write dword.
    void
    Inst_DS__DS_WRITE_SRC2_B32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_SRC2_F32 class methods ---

    Inst_DS__DS_MIN_SRC2_F32::Inst_DS__DS_MIN_SRC2_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_src2_f32")
    {
        setFlag(F32);
    } // Inst_DS__DS_MIN_SRC2_F32

    Inst_DS__DS_MIN_SRC2_F32::~Inst_DS__DS_MIN_SRC2_F32()
    {
    } // ~Inst_DS__DS_MIN_SRC2_F32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = (MEM[B] < MEM[A]) ? MEM[B] : MEM[A].
    // Float, handles NaN/INF/denorm.
    void
    Inst_DS__DS_MIN_SRC2_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_SRC2_F32 class methods ---

    Inst_DS__DS_MAX_SRC2_F32::Inst_DS__DS_MAX_SRC2_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_src2_f32")
    {
        setFlag(F32);
    } // Inst_DS__DS_MAX_SRC2_F32

    Inst_DS__DS_MAX_SRC2_F32::~Inst_DS__DS_MAX_SRC2_F32()
    {
    } // ~Inst_DS__DS_MAX_SRC2_F32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = (MEM[B] > MEM[A]) ? MEM[B] : MEM[A].
    // Float, handles NaN/INF/denorm.
    void
    Inst_DS__DS_MAX_SRC2_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_ADD_SRC2_F32 class methods ---

    Inst_DS__DS_ADD_SRC2_F32::Inst_DS__DS_ADD_SRC2_F32(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_add_src2_f32")
    {
        setFlag(F32);
    } // Inst_DS__DS_ADD_SRC2_F32

    Inst_DS__DS_ADD_SRC2_F32::~Inst_DS__DS_ADD_SRC2_F32()
    {
    } // ~Inst_DS__DS_ADD_SRC2_F32

    // --- description from .arch file ---
    // 32b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[B] + MEM[A].
    // Float, handles NaN/INF/denorm.
    void
    Inst_DS__DS_ADD_SRC2_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_GWS_SEMA_RELEASE_ALL class methods ---

    Inst_DS__DS_GWS_SEMA_RELEASE_ALL::Inst_DS__DS_GWS_SEMA_RELEASE_ALL(
          InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_gws_sema_release_all")
    {
    } // Inst_DS__DS_GWS_SEMA_RELEASE_ALL

    Inst_DS__DS_GWS_SEMA_RELEASE_ALL::~Inst_DS__DS_GWS_SEMA_RELEASE_ALL()
    {
    } // ~Inst_DS__DS_GWS_SEMA_RELEASE_ALL

    // --- description from .arch file ---
    // GDS Only: The GWS resource (rid) indicated will process this opcode by
    // updating the counter and labeling the specified resource as a semaphore.
    // //Determine the GWS resource to work on
    // rid[5:0] = SH_SX_EXPCMD.gds_base[5:0] + offset0[5:0];
    // //Incr the state counter of the resource
    // state.counter[rid] = state.wave_in_queue;
    // state.type = SEMAPHORE;
    // return rd_done; //release calling wave
    // This action will release ALL queued waves; it Will have no effect if no
    // ---  waves are present.
    void
    Inst_DS__DS_GWS_SEMA_RELEASE_ALL::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_GWS_INIT class methods ---

    Inst_DS__DS_GWS_INIT::Inst_DS__DS_GWS_INIT(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_gws_init")
    {
    } // Inst_DS__DS_GWS_INIT

    Inst_DS__DS_GWS_INIT::~Inst_DS__DS_GWS_INIT()
    {
    } // ~Inst_DS__DS_GWS_INIT

    // --- description from .arch file ---
    // GDS Only: Initialize a barrier or semaphore resource.
    // //Determine the GWS resource to work on
    // rid[5:0] = SH_SX_EXPCMD.gds_base[5:0] + offset0[5:0];
    // //Get the value to use in init
    // index = find_first_valid(vector mask)
    // value = DATA[thread: index]
    // //Set the state of the resource
    // state.counter[rid] = lsb(value); //limit #waves
    // state.flag[rid] = 0;
    // return rd_done; //release calling wave
    void
    Inst_DS__DS_GWS_INIT::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_GWS_SEMA_V class methods ---

    Inst_DS__DS_GWS_SEMA_V::Inst_DS__DS_GWS_SEMA_V(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_gws_sema_v")
    {
    } // Inst_DS__DS_GWS_SEMA_V

    Inst_DS__DS_GWS_SEMA_V::~Inst_DS__DS_GWS_SEMA_V()
    {
    } // ~Inst_DS__DS_GWS_SEMA_V

    // --- description from .arch file ---
    // GDS Only: The GWS resource indicated will process this opcode by
    // updating the counter and labeling the resource as a semaphore.
    // //Determine the GWS resource to work on
    // rid[5:0] = SH_SX_EXPCMD.gds_base[5:0] + offset0[5:0];
    // //Incr the state counter of the resource
    // state.counter[rid]++;
    // state.type = SEMAPHORE;
    // return rd_done; //release calling wave
    // This action will release one waved if any are queued in this resource.
    void
    Inst_DS__DS_GWS_SEMA_V::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_GWS_SEMA_BR class methods ---

    Inst_DS__DS_GWS_SEMA_BR::Inst_DS__DS_GWS_SEMA_BR(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_gws_sema_br")
    {
    } // Inst_DS__DS_GWS_SEMA_BR

    Inst_DS__DS_GWS_SEMA_BR::~Inst_DS__DS_GWS_SEMA_BR()
    {
    } // ~Inst_DS__DS_GWS_SEMA_BR

    // --- description from .arch file ---
    // GDS Only: The GWS resource indicated will process this opcode by
    // updating the counter by the bulk release delivered count and labeling
    // the resource as a semaphore.
    // //Determine the GWS resource to work on
    // rid[5:0] = SH_SX_EXPCMD.gds_base[5:0] + offset0[5:0];
    // index =  find first valid (vector mask)
    // count = DATA[thread: index];
    // //Add count to the resource state counter
    // state.counter[rid] += count;
    // state.type = SEMAPHORE;
    // return rd_done; //release calling wave
    // This action will release count number of waves, immediately if queued,
    // or as they arrive from the noted resource.
    void
    Inst_DS__DS_GWS_SEMA_BR::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_GWS_SEMA_P class methods ---

    Inst_DS__DS_GWS_SEMA_P::Inst_DS__DS_GWS_SEMA_P(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_gws_sema_p")
    {
    } // Inst_DS__DS_GWS_SEMA_P

    Inst_DS__DS_GWS_SEMA_P::~Inst_DS__DS_GWS_SEMA_P()
    {
    } // ~Inst_DS__DS_GWS_SEMA_P

    // --- description from .arch file ---
    // GDS Only: The GWS resource indicated will process this opcode by
    // queueing it until counter enables a release and then decrementing the
    // counter of the resource as a semaphore.
    // //Determine the GWS resource to work on
    // rid[5:0] = SH_SX_EXPCMD.gds_base[5:0] + offset0[5:0];
    // state.type = SEMAPHORE;
    // ENQUEUE until(state[rid].counter > 0)
    // state[rid].counter--;
    // return rd_done
    void
    Inst_DS__DS_GWS_SEMA_P::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_GWS_BARRIER class methods ---

    Inst_DS__DS_GWS_BARRIER::Inst_DS__DS_GWS_BARRIER(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_gws_barrier")
    {
    } // Inst_DS__DS_GWS_BARRIER

    Inst_DS__DS_GWS_BARRIER::~Inst_DS__DS_GWS_BARRIER()
    {
    } // ~Inst_DS__DS_GWS_BARRIER

    // --- description from .arch file ---
    // GDS Only: The GWS resource indicated will process this opcode by
    // queueing it until barrier is satisfied. The number of waves needed is
    // passed in as DATA of first valid thread.
    // //Determine the GWS resource to work on
    // rid[5:0] = SH_SX_EXPCMD.gds_base[5:0] + OFFSET0[5:0];
    // index =  find first valid (vector mask);
    // value = DATA[thread: index];
    // // Input Decision Machine
    // state.type[rid] = BARRIER;
    // if (state[rid].counter <= 0) {
    //     thread[rid].flag = state[rid].flag;
    //     ENQUEUE;
    //     state[rid].flag = !state.flag;
    //     state[rid].counter = value;
    //     return rd_done;
    // } else {
    //     state[rid].counter--;
    //     thread.flag = state[rid].flag;
    //     ENQUEUE;
    // }
    // Since the waves deliver the count for the next barrier, this function
    // can have a different size barrier for each occurrence.
    // // Release Machine
    // if (state.type == BARRIER) {
    //     if (state.flag != thread.flag) {
    //         return rd_done;
    //     }
    // }
    void
    Inst_DS__DS_GWS_BARRIER::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_CONSUME class methods ---

    Inst_DS__DS_CONSUME::Inst_DS__DS_CONSUME(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_consume")
    {
    } // Inst_DS__DS_CONSUME

    Inst_DS__DS_CONSUME::~Inst_DS__DS_CONSUME()
    {
    } // ~Inst_DS__DS_CONSUME

    // --- description from .arch file ---
    // LDS & GDS. Subtract (count_bits(exec_mask)) from the value stored in DS
    // memory at (M0.base + instr_offset). Return the pre-operation value to
    // VGPRs.
    void
    Inst_DS__DS_CONSUME::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_APPEND class methods ---

    Inst_DS__DS_APPEND::Inst_DS__DS_APPEND(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_append")
    {
    } // Inst_DS__DS_APPEND

    Inst_DS__DS_APPEND::~Inst_DS__DS_APPEND()
    {
    } // ~Inst_DS__DS_APPEND

    // --- description from .arch file ---
    // LDS & GDS. Add (count_bits(exec_mask)) to the value stored in DS memory
    // at (M0.base + instr_offset). Return the pre-operation value to VGPRs.
    void
    Inst_DS__DS_APPEND::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_ORDERED_COUNT class methods ---

    Inst_DS__DS_ORDERED_COUNT::Inst_DS__DS_ORDERED_COUNT(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_ordered_count")
    {
    } // Inst_DS__DS_ORDERED_COUNT

    Inst_DS__DS_ORDERED_COUNT::~Inst_DS__DS_ORDERED_COUNT()
    {
    } // ~Inst_DS__DS_ORDERED_COUNT

    // --- description from .arch file ---
    // GDS-only. Add (count_bits(exec_mask)) to one of 4 dedicated
    // ordered-count counters (aka 'packers'). Additional bits of instr.offset
    // field are overloaded to hold packer-id, 'last'.
    void
    Inst_DS__DS_ORDERED_COUNT::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_ADD_SRC2_U64 class methods ---

    Inst_DS__DS_ADD_SRC2_U64::Inst_DS__DS_ADD_SRC2_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_add_src2_u64")
    {
    } // Inst_DS__DS_ADD_SRC2_U64

    Inst_DS__DS_ADD_SRC2_U64::~Inst_DS__DS_ADD_SRC2_U64()
    {
    } // ~Inst_DS__DS_ADD_SRC2_U64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[A] + MEM[B].
    void
    Inst_DS__DS_ADD_SRC2_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_SUB_SRC2_U64 class methods ---

    Inst_DS__DS_SUB_SRC2_U64::Inst_DS__DS_SUB_SRC2_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_sub_src2_u64")
    {
    } // Inst_DS__DS_SUB_SRC2_U64

    Inst_DS__DS_SUB_SRC2_U64::~Inst_DS__DS_SUB_SRC2_U64()
    {
    } // ~Inst_DS__DS_SUB_SRC2_U64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[A] - MEM[B].
    void
    Inst_DS__DS_SUB_SRC2_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_RSUB_SRC2_U64 class methods ---

    Inst_DS__DS_RSUB_SRC2_U64::Inst_DS__DS_RSUB_SRC2_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_rsub_src2_u64")
    {
    } // Inst_DS__DS_RSUB_SRC2_U64

    Inst_DS__DS_RSUB_SRC2_U64::~Inst_DS__DS_RSUB_SRC2_U64()
    {
    } // ~Inst_DS__DS_RSUB_SRC2_U64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[B] - MEM[A].
    void
    Inst_DS__DS_RSUB_SRC2_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_INC_SRC2_U64 class methods ---

    Inst_DS__DS_INC_SRC2_U64::Inst_DS__DS_INC_SRC2_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_inc_src2_u64")
    {
    } // Inst_DS__DS_INC_SRC2_U64

    Inst_DS__DS_INC_SRC2_U64::~Inst_DS__DS_INC_SRC2_U64()
    {
    } // ~Inst_DS__DS_INC_SRC2_U64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = (MEM[A] >= MEM[B] ? 0 : MEM[A] + 1).
    void
    Inst_DS__DS_INC_SRC2_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_DEC_SRC2_U64 class methods ---

    Inst_DS__DS_DEC_SRC2_U64::Inst_DS__DS_DEC_SRC2_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_dec_src2_u64")
    {
    } // Inst_DS__DS_DEC_SRC2_U64

    Inst_DS__DS_DEC_SRC2_U64::~Inst_DS__DS_DEC_SRC2_U64()
    {
    } // ~Inst_DS__DS_DEC_SRC2_U64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = (MEM[A] == 0 || MEM[A] > MEM[B] ? MEM[B] : MEM[A] - 1).
    // Uint decrement.
    void
    Inst_DS__DS_DEC_SRC2_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_SRC2_I64 class methods ---

    Inst_DS__DS_MIN_SRC2_I64::Inst_DS__DS_MIN_SRC2_I64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_src2_i64")
    {
    } // Inst_DS__DS_MIN_SRC2_I64

    Inst_DS__DS_MIN_SRC2_I64::~Inst_DS__DS_MIN_SRC2_I64()
    {
    } // ~Inst_DS__DS_MIN_SRC2_I64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = min(MEM[A], MEM[B]).
    void
    Inst_DS__DS_MIN_SRC2_I64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_SRC2_I64 class methods ---

    Inst_DS__DS_MAX_SRC2_I64::Inst_DS__DS_MAX_SRC2_I64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_src2_i64")
    {
    } // Inst_DS__DS_MAX_SRC2_I64

    Inst_DS__DS_MAX_SRC2_I64::~Inst_DS__DS_MAX_SRC2_I64()
    {
    } // ~Inst_DS__DS_MAX_SRC2_I64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = max(MEM[A], MEM[B]).
    void
    Inst_DS__DS_MAX_SRC2_I64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_SRC2_U64 class methods ---

    Inst_DS__DS_MIN_SRC2_U64::Inst_DS__DS_MIN_SRC2_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_src2_u64")
    {
    } // Inst_DS__DS_MIN_SRC2_U64

    Inst_DS__DS_MIN_SRC2_U64::~Inst_DS__DS_MIN_SRC2_U64()
    {
    } // ~Inst_DS__DS_MIN_SRC2_U64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = min(MEM[A], MEM[B]).
    void
    Inst_DS__DS_MIN_SRC2_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_SRC2_U64 class methods ---

    Inst_DS__DS_MAX_SRC2_U64::Inst_DS__DS_MAX_SRC2_U64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_src2_u64")
    {
    } // Inst_DS__DS_MAX_SRC2_U64

    Inst_DS__DS_MAX_SRC2_U64::~Inst_DS__DS_MAX_SRC2_U64()
    {
    } // ~Inst_DS__DS_MAX_SRC2_U64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = max(MEM[A], MEM[B]).
    void
    Inst_DS__DS_MAX_SRC2_U64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_AND_SRC2_B64 class methods ---

    Inst_DS__DS_AND_SRC2_B64::Inst_DS__DS_AND_SRC2_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_and_src2_b64")
    {
    } // Inst_DS__DS_AND_SRC2_B64

    Inst_DS__DS_AND_SRC2_B64::~Inst_DS__DS_AND_SRC2_B64()
    {
    } // ~Inst_DS__DS_AND_SRC2_B64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[A] & MEM[B].
    void
    Inst_DS__DS_AND_SRC2_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_OR_SRC2_B64 class methods ---

    Inst_DS__DS_OR_SRC2_B64::Inst_DS__DS_OR_SRC2_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_or_src2_b64")
    {
    } // Inst_DS__DS_OR_SRC2_B64

    Inst_DS__DS_OR_SRC2_B64::~Inst_DS__DS_OR_SRC2_B64()
    {
    } // ~Inst_DS__DS_OR_SRC2_B64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[A] | MEM[B].
    void
    Inst_DS__DS_OR_SRC2_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_XOR_SRC2_B64 class methods ---

    Inst_DS__DS_XOR_SRC2_B64::Inst_DS__DS_XOR_SRC2_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_xor_src2_b64")
    {
    } // Inst_DS__DS_XOR_SRC2_B64

    Inst_DS__DS_XOR_SRC2_B64::~Inst_DS__DS_XOR_SRC2_B64()
    {
    } // ~Inst_DS__DS_XOR_SRC2_B64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[A] ^ MEM[B].
    void
    Inst_DS__DS_XOR_SRC2_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRITE_SRC2_B64 class methods ---

    Inst_DS__DS_WRITE_SRC2_B64::Inst_DS__DS_WRITE_SRC2_B64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write_src2_b64")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE_SRC2_B64

    Inst_DS__DS_WRITE_SRC2_B64::~Inst_DS__DS_WRITE_SRC2_B64()
    {
    } // ~Inst_DS__DS_WRITE_SRC2_B64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = MEM[B].
    // Write qword.
    void
    Inst_DS__DS_WRITE_SRC2_B64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MIN_SRC2_F64 class methods ---

    Inst_DS__DS_MIN_SRC2_F64::Inst_DS__DS_MIN_SRC2_F64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_min_src2_f64")
    {
        setFlag(F64);
    } // Inst_DS__DS_MIN_SRC2_F64

    Inst_DS__DS_MIN_SRC2_F64::~Inst_DS__DS_MIN_SRC2_F64()
    {
    } // ~Inst_DS__DS_MIN_SRC2_F64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = (MEM[B] < MEM[A]) ? MEM[B] : MEM[A].
    // Float, handles NaN/INF/denorm.
    void
    Inst_DS__DS_MIN_SRC2_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_MAX_SRC2_F64 class methods ---

    Inst_DS__DS_MAX_SRC2_F64::Inst_DS__DS_MAX_SRC2_F64(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_max_src2_f64")
    {
        setFlag(F64);
    } // Inst_DS__DS_MAX_SRC2_F64

    Inst_DS__DS_MAX_SRC2_F64::~Inst_DS__DS_MAX_SRC2_F64()
    {
    } // ~Inst_DS__DS_MAX_SRC2_F64

    // --- description from .arch file ---
    // 64b:
    // A = ADDR_BASE;
    // B = A + 4*(offset1[7] ? {A[31],A[31:17]} :
    // ---  {offset1[6],offset1[6:0],offset0});
    // MEM[A] = (MEM[B] > MEM[A]) ? MEM[B] : MEM[A].
    // Float, handles NaN/INF/denorm.
    void
    Inst_DS__DS_MAX_SRC2_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_DS__DS_WRITE_B96 class methods ---

    Inst_DS__DS_WRITE_B96::Inst_DS__DS_WRITE_B96(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write_b96")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE_B96

    Inst_DS__DS_WRITE_B96::~Inst_DS__DS_WRITE_B96()
    {
    } // ~Inst_DS__DS_WRITE_B96

    // --- description from .arch file ---
    // {MEM[ADDR + 8], MEM[ADDR + 4], MEM[ADDR]} = DATA[95:0].
    // Tri-dword write.
    void
    Inst_DS__DS_WRITE_B96::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU32 data0(gpuDynInst, extData.DATA0);
        ConstVecOperandU32 data1(gpuDynInst, extData.DATA0 + 1);
        ConstVecOperandU32 data2(gpuDynInst, extData.DATA0 + 2);

        addr.read();
        data0.read();
        data1.read();
        data2.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4] = data0[lane];
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 1] = data1[lane];
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 2] = data2[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE_B96::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemWrite<3>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_WRITE_B96::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_DS__DS_WRITE_B128 class methods ---

    Inst_DS__DS_WRITE_B128::Inst_DS__DS_WRITE_B128(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_write_b128")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_DS__DS_WRITE_B128

    Inst_DS__DS_WRITE_B128::~Inst_DS__DS_WRITE_B128()
    {
    } // ~Inst_DS__DS_WRITE_B128

    // --- description from .arch file ---
    // {MEM[ADDR + 12], MEM[ADDR + 8], MEM[ADDR + 4], MEM[ADDR]} = DATA[127:0].
    // Qword write.
    void
    Inst_DS__DS_WRITE_B128::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);
        ConstVecOperandU32 data0(gpuDynInst, extData.DATA0);
        ConstVecOperandU32 data1(gpuDynInst, extData.DATA0 + 1);
        ConstVecOperandU32 data2(gpuDynInst, extData.DATA0 + 2);
        ConstVecOperandU32 data3(gpuDynInst, extData.DATA0 + 3);

        addr.read();
        data0.read();
        data1.read();
        data2.read();
        data3.read();

        calcAddr(gpuDynInst, addr);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4] = data0[lane];
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 1] = data1[lane];
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 2] = data2[lane];
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 3] = data3[lane];
            }
        }

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_WRITE_B128::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemWrite<4>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_WRITE_B128::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_DS__DS_READ_B96 class methods ---

    Inst_DS__DS_READ_B96::Inst_DS__DS_READ_B96(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read_b96")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ_B96

    Inst_DS__DS_READ_B96::~Inst_DS__DS_READ_B96()
    {
    } // ~Inst_DS__DS_READ_B96

    // --- description from .arch file ---
    // Tri-dword read.
    void
    Inst_DS__DS_READ_B96::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_READ_B96::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemRead<3>(gpuDynInst, offset);
    }

    void
    Inst_DS__DS_READ_B96::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst0(gpuDynInst, extData.VDST);
        VecOperandU32 vdst1(gpuDynInst, extData.VDST + 1);
        VecOperandU32 vdst2(gpuDynInst, extData.VDST + 2);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4];
                vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 1];
                vdst2[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 2];
            }
        }

        vdst0.write();
        vdst1.write();
        vdst2.write();
    }
    // --- Inst_DS__DS_READ_B128 class methods ---

    Inst_DS__DS_READ_B128::Inst_DS__DS_READ_B128(InFmt_DS *iFmt)
        : Inst_DS(iFmt, "ds_read_b128")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_DS__DS_READ_B128

    Inst_DS__DS_READ_B128::~Inst_DS__DS_READ_B128()
    {
    } // ~Inst_DS__DS_READ_B128

    // --- description from .arch file ---
    // Qword read.
    void
    Inst_DS__DS_READ_B128::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(
                gpuDynInst->computeUnit()->cyclesToTicks(Cycles(24)));
        ConstVecOperandU32 addr(gpuDynInst, extData.ADDR);

        addr.read();

        calcAddr(gpuDynInst, addr);

        gpuDynInst->computeUnit()->localMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_DS__DS_READ_B128::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        Addr offset0 = instData.OFFSET0;
        Addr offset1 = instData.OFFSET1;
        Addr offset = (offset1 << 8) | offset0;

        initMemRead<4>(gpuDynInst, offset);
    } // initiateAcc

    void
    Inst_DS__DS_READ_B128::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst0(gpuDynInst, extData.VDST);
        VecOperandU32 vdst1(gpuDynInst, extData.VDST + 1);
        VecOperandU32 vdst2(gpuDynInst, extData.VDST + 2);
        VecOperandU32 vdst3(gpuDynInst, extData.VDST + 3);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4];
                vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 1];
                vdst2[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 2];
                vdst3[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 3];
            }
        }

        vdst0.write();
        vdst1.write();
        vdst2.write();
        vdst3.write();
    } // completeAcc
} // namespace VegaISA
} // namespace gem5
