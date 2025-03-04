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
    // --- Inst_FLAT__FLAT_LOAD_UBYTE class methods ---

    Inst_FLAT__FLAT_LOAD_UBYTE::Inst_FLAT__FLAT_LOAD_UBYTE(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_load_ubyte")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_FLAT__FLAT_LOAD_UBYTE

    Inst_FLAT__FLAT_LOAD_UBYTE::~Inst_FLAT__FLAT_LOAD_UBYTE()
    {
    } // ~Inst_FLAT__FLAT_LOAD_UBYTE

    // --- description from .arch file ---
    // Untyped buffer load unsigned byte (zero extend to VGPR destination).
    void
    Inst_FLAT__FLAT_LOAD_UBYTE::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_LOAD_UBYTE::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<VecElemU8>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_LOAD_UBYTE::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDST);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst[lane] = (VecElemU32)((reinterpret_cast<VecElemU8*>(
                    gpuDynInst->d_data))[lane]);
            }
        }
        vdst.write();
    } // execute
    // --- Inst_FLAT__FLAT_LOAD_SBYTE class methods ---

    Inst_FLAT__FLAT_LOAD_SBYTE::Inst_FLAT__FLAT_LOAD_SBYTE(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_load_sbyte")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_FLAT__FLAT_LOAD_SBYTE

    Inst_FLAT__FLAT_LOAD_SBYTE::~Inst_FLAT__FLAT_LOAD_SBYTE()
    {
    } // ~Inst_FLAT__FLAT_LOAD_SBYTE

    // --- description from .arch file ---
    // Untyped buffer load signed byte (sign extend to VGPR destination).
    void
    Inst_FLAT__FLAT_LOAD_SBYTE::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_LOAD_SBYTE::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<VecElemI8>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_LOAD_SBYTE::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDST);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst[lane] = (VecElemI32)((reinterpret_cast<VecElemI8*>(
                    gpuDynInst->d_data))[lane]);
            }
        }
        vdst.write();
    } // execute
    // --- Inst_FLAT__FLAT_LOAD_USHORT class methods ---

    Inst_FLAT__FLAT_LOAD_USHORT::Inst_FLAT__FLAT_LOAD_USHORT(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_load_ushort")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_FLAT__FLAT_LOAD_USHORT

    Inst_FLAT__FLAT_LOAD_USHORT::~Inst_FLAT__FLAT_LOAD_USHORT()
    {
    } // ~Inst_FLAT__FLAT_LOAD_USHORT

    // --- description from .arch file ---
    // Untyped buffer load unsigned short (zero extend to VGPR destination).
    void
    Inst_FLAT__FLAT_LOAD_USHORT::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_LOAD_USHORT::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<VecElemU16>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_LOAD_USHORT::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDST);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                vdst[lane] = (VecElemU32)((reinterpret_cast<VecElemU16*>(
                    gpuDynInst->d_data))[lane]);
            }
        }
        vdst.write();
    } // execute

    // --- Inst_FLAT__FLAT_LOAD_SSHORT class methods ---

    Inst_FLAT__FLAT_LOAD_SSHORT::Inst_FLAT__FLAT_LOAD_SSHORT(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_load_sshort")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_FLAT__FLAT_LOAD_SSHORT

    Inst_FLAT__FLAT_LOAD_SSHORT::~Inst_FLAT__FLAT_LOAD_SSHORT()
    {
    } // ~Inst_FLAT__FLAT_LOAD_SSHORT

    // --- description from .arch file ---
    // Untyped buffer load signed short (sign extend to VGPR destination).
    void
    Inst_FLAT__FLAT_LOAD_SSHORT::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_FLAT__FLAT_LOAD_SSHORT::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_FLAT__FLAT_LOAD_SSHORT::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_FLAT__FLAT_LOAD_DWORD class methods ---

    Inst_FLAT__FLAT_LOAD_DWORD::Inst_FLAT__FLAT_LOAD_DWORD(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_load_dword")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_FLAT__FLAT_LOAD_DWORD

    Inst_FLAT__FLAT_LOAD_DWORD::~Inst_FLAT__FLAT_LOAD_DWORD()
    {
    } // ~Inst_FLAT__FLAT_LOAD_DWORD

    // --- description from .arch file ---
    // Untyped buffer load dword.
    void
    Inst_FLAT__FLAT_LOAD_DWORD::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_LOAD_DWORD::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_LOAD_DWORD::completeAcc(GPUDynInstPtr gpuDynInst)
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
    // --- Inst_FLAT__FLAT_LOAD_DWORDX2 class methods ---

    Inst_FLAT__FLAT_LOAD_DWORDX2::Inst_FLAT__FLAT_LOAD_DWORDX2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_load_dwordx2")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_FLAT__FLAT_LOAD_DWORDX2

    Inst_FLAT__FLAT_LOAD_DWORDX2::~Inst_FLAT__FLAT_LOAD_DWORDX2()
    {
    } // ~Inst_FLAT__FLAT_LOAD_DWORDX2

    // --- description from .arch file ---
    // Untyped buffer load 2 dwords.
    void
    Inst_FLAT__FLAT_LOAD_DWORDX2::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_LOAD_DWORDX2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<2>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_LOAD_DWORDX2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst0(gpuDynInst, extData.VDST);
        VecOperandU32 vdst1(gpuDynInst, extData.VDST + 1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane] && !isFlatScratch()) {
                vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 2];
                vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 2 + 1];
            } else if (gpuDynInst->exec_mask[lane] && isFlatScratch()) {
                // Unswizzle the data opposite of swizzleData. See swizzleData
                // in src/arch/amdgpu/vega/insts/op_encodings.hh for details.
                vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane];
                vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane + NumVecElemPerVecReg];
            }
        }

        vdst0.write();
        vdst1.write();
    } // completeAcc
    // --- Inst_FLAT__FLAT_LOAD_DWORDX3 class methods ---

    Inst_FLAT__FLAT_LOAD_DWORDX3::Inst_FLAT__FLAT_LOAD_DWORDX3(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_load_dwordx3")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_FLAT__FLAT_LOAD_DWORDX3

    Inst_FLAT__FLAT_LOAD_DWORDX3::~Inst_FLAT__FLAT_LOAD_DWORDX3()
    {
    } // ~Inst_FLAT__FLAT_LOAD_DWORDX3

    // --- description from .arch file ---
    // Untyped buffer load 3 dwords.
    void
    Inst_FLAT__FLAT_LOAD_DWORDX3::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_LOAD_DWORDX3::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<3>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_LOAD_DWORDX3::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst0(gpuDynInst, extData.VDST);
        VecOperandU32 vdst1(gpuDynInst, extData.VDST + 1);
        VecOperandU32 vdst2(gpuDynInst, extData.VDST + 2);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane] && !isFlatScratch()) {
                vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 3];
                vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 3 + 1];
                vdst2[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 3 + 2];
            } else if (gpuDynInst->exec_mask[lane] && isFlatScratch()) {
                // Unswizzle the data opposite of swizzleData. See swizzleData
                // in src/arch/amdgpu/vega/insts/op_encodings.hh for details.
                vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane];
                vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane + NumVecElemPerVecReg];
                vdst2[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane + 2*NumVecElemPerVecReg];
            }
        }

        vdst0.write();
        vdst1.write();
        vdst2.write();
    } // completeAcc
    // --- Inst_FLAT__FLAT_LOAD_DWORDX4 class methods ---

    Inst_FLAT__FLAT_LOAD_DWORDX4::Inst_FLAT__FLAT_LOAD_DWORDX4(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_load_dwordx4")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_FLAT__FLAT_LOAD_DWORDX4

    Inst_FLAT__FLAT_LOAD_DWORDX4::~Inst_FLAT__FLAT_LOAD_DWORDX4()
    {
    } // ~Inst_FLAT__FLAT_LOAD_DWORDX4

    // --- description from .arch file ---
    // Untyped buffer load 4 dwords.
    void
    Inst_FLAT__FLAT_LOAD_DWORDX4::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_LOAD_DWORDX4::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<4>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_LOAD_DWORDX4::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst0(gpuDynInst, extData.VDST);
        VecOperandU32 vdst1(gpuDynInst, extData.VDST + 1);
        VecOperandU32 vdst2(gpuDynInst, extData.VDST + 2);
        VecOperandU32 vdst3(gpuDynInst, extData.VDST + 3);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane] && !isFlatScratch()) {
                vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4];
                vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 1];
                vdst2[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 2];
                vdst3[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 4 + 3];
            } else if (gpuDynInst->exec_mask[lane] && isFlatScratch()) {
                // Unswizzle the data opposite of swizzleData. See swizzleData
                // in src/arch/amdgpu/vega/insts/op_encodings.hh for details.
                vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane];
                vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane + NumVecElemPerVecReg];
                vdst2[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane + 2*NumVecElemPerVecReg];
                vdst3[lane] = (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane + 3*NumVecElemPerVecReg];
            }
        }

        vdst0.write();
        vdst1.write();
        vdst2.write();
        vdst3.write();
    } // completeAcc
    // --- Inst_FLAT__FLAT_STORE_BYTE class methods ---

    Inst_FLAT__FLAT_STORE_BYTE::Inst_FLAT__FLAT_STORE_BYTE(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_store_byte")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_FLAT__FLAT_STORE_BYTE

    Inst_FLAT__FLAT_STORE_BYTE::~Inst_FLAT__FLAT_STORE_BYTE()
    {
    } // ~Inst_FLAT__FLAT_STORE_BYTE

    // --- description from .arch file ---
    // Untyped buffer store byte.
    void
    Inst_FLAT__FLAT_STORE_BYTE::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU8 data(gpuDynInst, extData.DATA);

        data.read();

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU8*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_STORE_BYTE::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<VecElemU8>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_STORE_BYTE::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_FLAT__FLAT_STORE_SHORT class methods ---

    Inst_FLAT__FLAT_STORE_SHORT::Inst_FLAT__FLAT_STORE_SHORT(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_store_short")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_FLAT__FLAT_STORE_SHORT

    Inst_FLAT__FLAT_STORE_SHORT::~Inst_FLAT__FLAT_STORE_SHORT()
    {
    } // ~Inst_FLAT__FLAT_STORE_SHORT

    // --- description from .arch file ---
    // Untyped buffer store short.
    void
    Inst_FLAT__FLAT_STORE_SHORT::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU16 data(gpuDynInst, extData.DATA);

        data.read();

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU16*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_STORE_SHORT::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<VecElemU16>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_STORE_SHORT::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_FLAT__FLAT_STORE_SHORT_D16_HI class methods ---

    Inst_FLAT__FLAT_STORE_SHORT_D16_HI::
        Inst_FLAT__FLAT_STORE_SHORT_D16_HI(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_store_short_d16_hi")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_FLAT__FLAT_STORE_SHORT_D16_HI

    Inst_FLAT__FLAT_STORE_SHORT_D16_HI::~Inst_FLAT__FLAT_STORE_SHORT_D16_HI()
    {
    } // ~Inst_FLAT__FLAT_STORE_SHORT_D16_HI

    // --- description from .arch file ---
    // Untyped buffer store short.
    void
    Inst_FLAT__FLAT_STORE_SHORT_D16_HI::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 data(gpuDynInst, extData.DATA);

        data.read();

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU16*>(gpuDynInst->d_data))[lane]
                    = (data[lane] >> 16);
            }
        }

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_STORE_SHORT_D16_HI::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<VecElemU16>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_STORE_SHORT_D16_HI::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_FLAT__FLAT_STORE_DWORD class methods ---

    Inst_FLAT__FLAT_STORE_DWORD::Inst_FLAT__FLAT_STORE_DWORD(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_store_dword")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_FLAT__FLAT_STORE_DWORD

    Inst_FLAT__FLAT_STORE_DWORD::~Inst_FLAT__FLAT_STORE_DWORD()
    {
    } // ~Inst_FLAT__FLAT_STORE_DWORD

    // --- description from .arch file ---
    // Untyped buffer store dword.
    void
    Inst_FLAT__FLAT_STORE_DWORD::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 data(gpuDynInst, extData.DATA);

        data.read();

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_STORE_DWORD::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_STORE_DWORD::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_FLAT__FLAT_STORE_DWORDX2 class methods ---

    Inst_FLAT__FLAT_STORE_DWORDX2::Inst_FLAT__FLAT_STORE_DWORDX2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_store_dwordx2")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_FLAT__FLAT_STORE_DWORDX2

    Inst_FLAT__FLAT_STORE_DWORDX2::~Inst_FLAT__FLAT_STORE_DWORDX2()
    {
    } // ~Inst_FLAT__FLAT_STORE_DWORDX2

    // --- description from .arch file ---
    // Untyped buffer store 2 dwords.
    void
    Inst_FLAT__FLAT_STORE_DWORDX2::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU64 data(gpuDynInst, extData.DATA);

        data.read();

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU64*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_STORE_DWORDX2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<2>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_STORE_DWORDX2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_FLAT__FLAT_STORE_DWORDX3 class methods ---

    Inst_FLAT__FLAT_STORE_DWORDX3::Inst_FLAT__FLAT_STORE_DWORDX3(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_store_dwordx3")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_FLAT__FLAT_STORE_DWORDX3

    Inst_FLAT__FLAT_STORE_DWORDX3::~Inst_FLAT__FLAT_STORE_DWORDX3()
    {
    } // ~Inst_FLAT__FLAT_STORE_DWORDX3

    // --- description from .arch file ---
    // Untyped buffer store 3 dwords.
    void
    Inst_FLAT__FLAT_STORE_DWORDX3::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 data0(gpuDynInst, extData.DATA);
        ConstVecOperandU32 data1(gpuDynInst, extData.DATA + 1);
        ConstVecOperandU32 data2(gpuDynInst, extData.DATA + 2);

        data0.read();
        data1.read();
        data2.read();

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 3] = data0[lane];
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 3 + 1] = data1[lane];
                (reinterpret_cast<VecElemU32*>(
                    gpuDynInst->d_data))[lane * 3 + 2] = data2[lane];
            }
        }

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_STORE_DWORDX3::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<3>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_STORE_DWORDX3::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_FLAT__FLAT_STORE_DWORDX4 class methods ---

    Inst_FLAT__FLAT_STORE_DWORDX4::Inst_FLAT__FLAT_STORE_DWORDX4(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_store_dwordx4")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_FLAT__FLAT_STORE_DWORDX4

    Inst_FLAT__FLAT_STORE_DWORDX4::~Inst_FLAT__FLAT_STORE_DWORDX4()
    {
    } // ~Inst_FLAT__FLAT_STORE_DWORDX4

    // --- description from .arch file ---
    // Untyped buffer store 4 dwords.
    void
    Inst_FLAT__FLAT_STORE_DWORDX4::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            if (isFlat()) {
                wf->decLGKMInstsIssued();
                wf->untrackLGKMInst(gpuDynInst);
            }
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 data0(gpuDynInst, extData.DATA);
        ConstVecOperandU32 data1(gpuDynInst, extData.DATA + 1);
        ConstVecOperandU32 data2(gpuDynInst, extData.DATA + 2);
        ConstVecOperandU32 data3(gpuDynInst, extData.DATA + 3);

        data0.read();
        data1.read();
        data2.read();
        data3.read();

        calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

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

        issueRequestHelper(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_STORE_DWORDX4::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<4>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_STORE_DWORDX4::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_SWAP class methods ---

    Inst_FLAT__FLAT_ATOMIC_SWAP::Inst_FLAT__FLAT_ATOMIC_SWAP(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_swap")
    {
        setFlag(AtomicExch);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_SWAP

    Inst_FLAT__FLAT_ATOMIC_SWAP::~Inst_FLAT__FLAT_ATOMIC_SWAP()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_SWAP

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = DATA;
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_SWAP::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_SWAP::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_SWAP::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc

    // --- Inst_FLAT__FLAT_ATOMIC_CMPSWAP class methods ---

    Inst_FLAT__FLAT_ATOMIC_CMPSWAP
        ::Inst_FLAT__FLAT_ATOMIC_CMPSWAP(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_cmpswap")
    {
        setFlag(AtomicCAS);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_CMPSWAP

    Inst_FLAT__FLAT_ATOMIC_CMPSWAP::~Inst_FLAT__FLAT_ATOMIC_CMPSWAP()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_CMPSWAP

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // src = DATA[0];
    // cmp = DATA[1];
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_CMPSWAP::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32, 1>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_CMPSWAP::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_CMPSWAP::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_ADD class methods ---

    Inst_FLAT__FLAT_ATOMIC_ADD::Inst_FLAT__FLAT_ATOMIC_ADD(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_add")
    {
        setFlag(AtomicAdd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_ADD

    Inst_FLAT__FLAT_ATOMIC_ADD::~Inst_FLAT__FLAT_ATOMIC_ADD()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_ADD

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] += DATA;
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_ADD::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_ADD::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_ADD::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_SUB class methods ---

    Inst_FLAT__FLAT_ATOMIC_SUB::Inst_FLAT__FLAT_ATOMIC_SUB(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_sub")
    {
        setFlag(AtomicSub);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_SUB

    Inst_FLAT__FLAT_ATOMIC_SUB::~Inst_FLAT__FLAT_ATOMIC_SUB()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_SUB

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_SUB::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_SUB::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_SUB::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_SMIN class methods ---

    Inst_FLAT__FLAT_ATOMIC_SMIN::Inst_FLAT__FLAT_ATOMIC_SMIN(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_smin")
    {
        setFlag(AtomicMin);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_SMIN

    Inst_FLAT__FLAT_ATOMIC_SMIN::~Inst_FLAT__FLAT_ATOMIC_SMIN()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_SMIN

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA < tmp) ? DATA : tmp (signed compare);
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_SMIN::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandI32, VecElemI32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_SMIN::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemI32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_SMIN::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandI32, VecElemI32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_UMIN class methods ---

    Inst_FLAT__FLAT_ATOMIC_UMIN::Inst_FLAT__FLAT_ATOMIC_UMIN(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_umin")
    {
        setFlag(AtomicMin);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_UMIN

    Inst_FLAT__FLAT_ATOMIC_UMIN::~Inst_FLAT__FLAT_ATOMIC_UMIN()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_UMIN

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA < tmp) ? DATA : tmp (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_UMIN::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_UMIN::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_UMIN::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_SMAX class methods ---

    Inst_FLAT__FLAT_ATOMIC_SMAX::Inst_FLAT__FLAT_ATOMIC_SMAX(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_smax")
    {
        setFlag(AtomicMax);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_SMAX

    Inst_FLAT__FLAT_ATOMIC_SMAX::~Inst_FLAT__FLAT_ATOMIC_SMAX()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_SMAX

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA > tmp) ? DATA : tmp (signed compare);
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_SMAX::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandI32, VecElemI32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_SMAX::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemI32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_SMAX::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandI32, VecElemI32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_UMAX class methods ---

    Inst_FLAT__FLAT_ATOMIC_UMAX::Inst_FLAT__FLAT_ATOMIC_UMAX(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_umax")
    {
        setFlag(AtomicMax);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_UMAX

    Inst_FLAT__FLAT_ATOMIC_UMAX::~Inst_FLAT__FLAT_ATOMIC_UMAX()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_UMAX

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA > tmp) ? DATA : tmp (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_UMAX::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_UMAX::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_UMAX::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_AND class methods ---

    Inst_FLAT__FLAT_ATOMIC_AND::Inst_FLAT__FLAT_ATOMIC_AND(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_and")
    {
        setFlag(AtomicAnd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_AND

    Inst_FLAT__FLAT_ATOMIC_AND::~Inst_FLAT__FLAT_ATOMIC_AND()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_AND

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] &= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_AND::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_AND::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_AND::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_OR class methods ---

    Inst_FLAT__FLAT_ATOMIC_OR::Inst_FLAT__FLAT_ATOMIC_OR(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_or")
    {
        setFlag(AtomicOr);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_OR

    Inst_FLAT__FLAT_ATOMIC_OR::~Inst_FLAT__FLAT_ATOMIC_OR()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_OR

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] |= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_OR::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_OR::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_OR::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc

    // --- Inst_FLAT__FLAT_ATOMIC_XOR class methods ---

    Inst_FLAT__FLAT_ATOMIC_XOR::Inst_FLAT__FLAT_ATOMIC_XOR(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_xor")
    {
        setFlag(AtomicXor);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_XOR

    Inst_FLAT__FLAT_ATOMIC_XOR::~Inst_FLAT__FLAT_ATOMIC_XOR()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_XOR

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] ^= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_XOR::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_XOR::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_XOR::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_INC class methods ---

    Inst_FLAT__FLAT_ATOMIC_INC::Inst_FLAT__FLAT_ATOMIC_INC(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_inc")
    {
        setFlag(AtomicInc);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_INC

    Inst_FLAT__FLAT_ATOMIC_INC::~Inst_FLAT__FLAT_ATOMIC_INC()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_INC

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp >= DATA) ? 0 : tmp + 1 (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_INC::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_INC::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_INC::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_DEC class methods ---

    Inst_FLAT__FLAT_ATOMIC_DEC::Inst_FLAT__FLAT_ATOMIC_DEC(InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_dec")
    {
        setFlag(AtomicDec);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_DEC

    Inst_FLAT__FLAT_ATOMIC_DEC::~Inst_FLAT__FLAT_ATOMIC_DEC()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_DEC

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp == 0 || tmp > DATA) ? DATA : tmp - 1
    // (unsigned compare); RETURN_DATA = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_DEC::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU32, VecElemU32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_DEC::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_DEC::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU32, VecElemU32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_SWAP_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_SWAP_X2::Inst_FLAT__FLAT_ATOMIC_SWAP_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_swap_x2")
    {
        setFlag(AtomicExch);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_SWAP_X2

    Inst_FLAT__FLAT_ATOMIC_SWAP_X2::~Inst_FLAT__FLAT_ATOMIC_SWAP_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_SWAP_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_SWAP_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_SWAP_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_SWAP_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2::Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_cmpswap_x2")
    {
        setFlag(AtomicCAS);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2

    Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2::~Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // src = DATA[0:1];
    // cmp = DATA[2:3];
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64, 2>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_CMPSWAP_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_ADD_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_ADD_X2::Inst_FLAT__FLAT_ATOMIC_ADD_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_add_x2")
    {
        setFlag(AtomicAdd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_ADD_X2

    Inst_FLAT__FLAT_ATOMIC_ADD_X2::~Inst_FLAT__FLAT_ATOMIC_ADD_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_ADD_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] += DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_ADD_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_ADD_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_ADD_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_SUB_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_SUB_X2::Inst_FLAT__FLAT_ATOMIC_SUB_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_sub_x2")
    {
        setFlag(AtomicSub);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_SUB_X2

    Inst_FLAT__FLAT_ATOMIC_SUB_X2::~Inst_FLAT__FLAT_ATOMIC_SUB_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_SUB_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_SUB_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_SUB_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_SUB_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_SMIN_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_SMIN_X2::Inst_FLAT__FLAT_ATOMIC_SMIN_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_smin_x2")
    {
        setFlag(AtomicMin);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_SMIN_X2

    Inst_FLAT__FLAT_ATOMIC_SMIN_X2::~Inst_FLAT__FLAT_ATOMIC_SMIN_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_SMIN_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] < tmp) ? DATA[0:1] : tmp (signed compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_SMIN_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandI64, VecElemI64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_SMIN_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemI64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_SMIN_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandI64, VecElemI64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_UMIN_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_UMIN_X2::Inst_FLAT__FLAT_ATOMIC_UMIN_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_umin_x2")
    {
        setFlag(AtomicMin);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_UMIN_X2

    Inst_FLAT__FLAT_ATOMIC_UMIN_X2::~Inst_FLAT__FLAT_ATOMIC_UMIN_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_UMIN_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] < tmp) ? DATA[0:1] : tmp (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_UMIN_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_UMIN_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_UMIN_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_SMAX_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_SMAX_X2::Inst_FLAT__FLAT_ATOMIC_SMAX_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_smax_x2")
    {
        setFlag(AtomicMax);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_SMAX_X2

    Inst_FLAT__FLAT_ATOMIC_SMAX_X2::~Inst_FLAT__FLAT_ATOMIC_SMAX_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_SMAX_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] > tmp) ? DATA[0:1] : tmp (signed compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_SMAX_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandI64, VecElemI64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_SMAX_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemI64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_SMAX_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandI64, VecElemI64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_UMAX_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_UMAX_X2::Inst_FLAT__FLAT_ATOMIC_UMAX_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_umax_x2")
    {
        setFlag(AtomicMax);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_UMAX_X2

    Inst_FLAT__FLAT_ATOMIC_UMAX_X2::~Inst_FLAT__FLAT_ATOMIC_UMAX_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_UMAX_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] > tmp) ? DATA[0:1] : tmp (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_UMAX_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_UMAX_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_UMAX_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_AND_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_AND_X2::Inst_FLAT__FLAT_ATOMIC_AND_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_and_x2")
    {
        setFlag(AtomicAnd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_AND_X2

    Inst_FLAT__FLAT_ATOMIC_AND_X2::~Inst_FLAT__FLAT_ATOMIC_AND_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_AND_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] &= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_AND_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_AND_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_AND_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_OR_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_OR_X2::Inst_FLAT__FLAT_ATOMIC_OR_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_or_x2")
    {
        setFlag(AtomicOr);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_OR_X2

    Inst_FLAT__FLAT_ATOMIC_OR_X2::~Inst_FLAT__FLAT_ATOMIC_OR_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_OR_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] |= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_OR_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_OR_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_OR_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_XOR_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_XOR_X2::Inst_FLAT__FLAT_ATOMIC_XOR_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_xor_x2")
    {
        setFlag(AtomicXor);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_XOR_X2

    Inst_FLAT__FLAT_ATOMIC_XOR_X2::~Inst_FLAT__FLAT_ATOMIC_XOR_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_XOR_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] ^= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_XOR_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_XOR_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_XOR_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_INC_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_INC_X2::Inst_FLAT__FLAT_ATOMIC_INC_X2(
          InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_inc_x2")
    {
        setFlag(AtomicInc);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_INC_X2

    Inst_FLAT__FLAT_ATOMIC_INC_X2::~Inst_FLAT__FLAT_ATOMIC_INC_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_INC_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp >= DATA[0:1]) ? 0 : tmp + 1 (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_INC_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_INC_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_INC_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_DEC_X2 class methods ---

    Inst_FLAT__FLAT_ATOMIC_DEC_X2::Inst_FLAT__FLAT_ATOMIC_DEC_X2(
        InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_dec_x2")
    {
        setFlag(AtomicDec);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_DEC_X2

    Inst_FLAT__FLAT_ATOMIC_DEC_X2::~Inst_FLAT__FLAT_ATOMIC_DEC_X2()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_DEC_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp == 0 || tmp > DATA[0:1]) ? DATA[0:1] : tmp - 1
    // (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_FLAT__FLAT_ATOMIC_DEC_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandU64, VecElemU64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_DEC_X2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_DEC_X2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandU64, VecElemU64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_ADD_F32 class methods ---

    Inst_FLAT__FLAT_ATOMIC_ADD_F32::Inst_FLAT__FLAT_ATOMIC_ADD_F32(
        InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_add_f32")
    {
        setFlag(AtomicAdd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_ADD_F32

    Inst_FLAT__FLAT_ATOMIC_ADD_F32::~Inst_FLAT__FLAT_ATOMIC_ADD_F32()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_ADD_F32

    void
    Inst_FLAT__FLAT_ATOMIC_ADD_F32::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandF32, VecElemF32>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_ADD_F32::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemF32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_ADD_F32::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandF32, VecElemF32>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16 class methods ---

    Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16::Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16(
        InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_pk_add_f16")
    {
        setFlag(AtomicAdd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16

    Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16::~Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16

    void
    Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_PK_ADD_F16::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_ADD_F64 class methods ---

    Inst_FLAT__FLAT_ATOMIC_ADD_F64::Inst_FLAT__FLAT_ATOMIC_ADD_F64(
        InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_add_f64")
    {
        setFlag(AtomicAdd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_ADD_F64

    Inst_FLAT__FLAT_ATOMIC_ADD_F64::~Inst_FLAT__FLAT_ATOMIC_ADD_F64()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_ADD_F64

    void
    Inst_FLAT__FLAT_ATOMIC_ADD_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandF64, VecElemF64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_ADD_F64::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemF64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_ADD_F64::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandF64, VecElemF64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_MIN_F64 class methods ---

    Inst_FLAT__FLAT_ATOMIC_MIN_F64::Inst_FLAT__FLAT_ATOMIC_MIN_F64(
        InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_min_f64")
    {
        setFlag(AtomicMin);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_MIN_F64

    Inst_FLAT__FLAT_ATOMIC_MIN_F64::~Inst_FLAT__FLAT_ATOMIC_MIN_F64()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_MIN_F64

    void
    Inst_FLAT__FLAT_ATOMIC_MIN_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandF64, VecElemF64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_MIN_F64::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemF64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_MIN_F64::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandF64, VecElemF64>(gpuDynInst);
    } // completeAcc
    // --- Inst_FLAT__FLAT_ATOMIC_MAX_F64 class methods ---

    Inst_FLAT__FLAT_ATOMIC_MAX_F64::Inst_FLAT__FLAT_ATOMIC_MAX_F64(
        InFmt_FLAT *iFmt)
        : Inst_FLAT(iFmt, "flat_atomic_max_f64")
    {
        setFlag(AtomicMax);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
    } // Inst_FLAT__FLAT_ATOMIC_MAX_F64

    Inst_FLAT__FLAT_ATOMIC_MAX_F64::~Inst_FLAT__FLAT_ATOMIC_MAX_F64()
    {
    } // ~Inst_FLAT__FLAT_ATOMIC_MAX_F64

    void
    Inst_FLAT__FLAT_ATOMIC_MAX_F64::execute(GPUDynInstPtr gpuDynInst)
    {
        atomicExecute<ConstVecOperandF64, VecElemF64>(gpuDynInst);
    } // execute

    void
    Inst_FLAT__FLAT_ATOMIC_MAX_F64::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemF64>(gpuDynInst);
    } // initiateAcc

    void
    Inst_FLAT__FLAT_ATOMIC_MAX_F64::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        atomicComplete<VecOperandF64, VecElemF64>(gpuDynInst);
    } // completeAcc
} // namespace VegaISA
} // namespace gem5
