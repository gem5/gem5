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
    // --- Inst_MUBUF__BUFFER_LOAD_FORMAT_X class methods ---

    Inst_MUBUF__BUFFER_LOAD_FORMAT_X
        ::Inst_MUBUF__BUFFER_LOAD_FORMAT_X(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_format_x")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_LOAD_FORMAT_X

    Inst_MUBUF__BUFFER_LOAD_FORMAT_X::~Inst_MUBUF__BUFFER_LOAD_FORMAT_X()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_FORMAT_X

    // --- description from .arch file ---
    // Untyped buffer load 1 dword with format conversion.
    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_X::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_X::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_X::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_LOAD_FORMAT_XY class methods ---

    Inst_MUBUF__BUFFER_LOAD_FORMAT_XY
        ::Inst_MUBUF__BUFFER_LOAD_FORMAT_XY(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_format_xy")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_LOAD_FORMAT_XY

    Inst_MUBUF__BUFFER_LOAD_FORMAT_XY::~Inst_MUBUF__BUFFER_LOAD_FORMAT_XY()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_FORMAT_XY

    // --- description from .arch file ---
    // Untyped buffer load 2 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_XY::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_XY::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_XY::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ class methods ---

    Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ
        ::Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_format_xyz")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ

    Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ::~Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ

    // --- description from .arch file ---
    // Untyped buffer load 3 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZ::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW class methods ---

    Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW
        ::Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_format_xyzw")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW

    Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW::~Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW

    // --- description from .arch file ---
    // Untyped buffer load 4 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_XYZW::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_STORE_FORMAT_X class methods ---

    Inst_MUBUF__BUFFER_STORE_FORMAT_X
        ::Inst_MUBUF__BUFFER_STORE_FORMAT_X(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_format_x")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_STORE_FORMAT_X

    Inst_MUBUF__BUFFER_STORE_FORMAT_X::~Inst_MUBUF__BUFFER_STORE_FORMAT_X()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_FORMAT_X

    // --- description from .arch file ---
    // Untyped buffer store 1 dword with format conversion.
    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_X::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_X::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_X::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_STORE_FORMAT_XY class methods ---

    Inst_MUBUF__BUFFER_STORE_FORMAT_XY
        ::Inst_MUBUF__BUFFER_STORE_FORMAT_XY(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_format_xy")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_STORE_FORMAT_XY

    Inst_MUBUF__BUFFER_STORE_FORMAT_XY::~Inst_MUBUF__BUFFER_STORE_FORMAT_XY()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_FORMAT_XY

    // --- description from .arch file ---
    // Untyped buffer store 2 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_XY::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_XY::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_XY::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ class methods ---

    Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ
        ::Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_format_xyz")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ

    Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ::~Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ

    // --- description from .arch file ---
    // Untyped buffer store 3 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_XYZ::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW class methods ---

    Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW
        ::Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_format_xyzw")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW

    Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW
        ::~Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW

    // --- description from .arch file ---
    // Untyped buffer store 4 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_XYZW::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X class methods ---

    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X
        ::Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_format_d16_x")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X

    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X
        ::~Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X

    // --- description from .arch file ---
    // Untyped buffer load 1 dword with format conversion.
    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_X::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY class methods ---

    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY
        ::Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_format_d16_xy")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY

    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY
        ::~Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY

    // --- description from .arch file ---
    // Untyped buffer load 2 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XY::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ class methods ---

    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ
        ::Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_format_d16_xyz")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ

    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ
        ::~Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ

    // --- description from .arch file ---
    // Untyped buffer load 3 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZ::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW class methods ---

    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW
        ::Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_format_d16_xyzw")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW

    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW
        ::~Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW

    // --- description from .arch file ---
    // Untyped buffer load 4 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_FORMAT_D16_XYZW::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X class methods ---

    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X
        ::Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_format_d16_x")
    {
        setFlag(Store);
    } // Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X

    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X
        ::~Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X

    // --- description from .arch file ---
    // Untyped buffer store 1 dword with format conversion.
    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_X::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY class methods ---

    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY
        ::Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_format_d16_xy")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY

    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY
        ::~Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY

    // --- description from .arch file ---
    // Untyped buffer store 2 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XY::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ class methods ---

    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ
        ::Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_format_d16_xyz")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ

    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ
        ::~Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ

    // --- description from .arch file ---
    // Untyped buffer store 3 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZ::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW class methods ---

    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW
        ::Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_format_d16_xyzw")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW

    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW
        ::~Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW

    // --- description from .arch file ---
    // Untyped buffer store 4 dwords with format conversion.
    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_FORMAT_D16_XYZW::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_LOAD_UBYTE class methods ---

    Inst_MUBUF__BUFFER_LOAD_UBYTE
        ::Inst_MUBUF__BUFFER_LOAD_UBYTE(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_ubyte")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_LOAD_UBYTE

    Inst_MUBUF__BUFFER_LOAD_UBYTE::~Inst_MUBUF__BUFFER_LOAD_UBYTE()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_UBYTE

    // --- description from .arch file ---
    // Untyped buffer load unsigned byte (zero extend to VGPR destination).
    void
    Inst_MUBUF__BUFFER_LOAD_UBYTE::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);

        rsrcDesc.read();
        offset.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_UBYTE::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<VecElemU8>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_UBYTE::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDATA);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                if (!oobMask[lane]) {
                    vdst[lane] = (VecElemU32)((reinterpret_cast<VecElemU8*>(
                        gpuDynInst->d_data))[lane]);
                } else {
                    vdst[lane] = 0;
                }
            }
        }

        vdst.write();
    } // execute

    // --- Inst_MUBUF__BUFFER_LOAD_SBYTE class methods ---

    Inst_MUBUF__BUFFER_LOAD_SBYTE
        ::Inst_MUBUF__BUFFER_LOAD_SBYTE(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_sbyte")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_LOAD_SBYTE

    Inst_MUBUF__BUFFER_LOAD_SBYTE::~Inst_MUBUF__BUFFER_LOAD_SBYTE()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_SBYTE

    // --- description from .arch file ---
    // Untyped buffer load signed byte (sign extend to VGPR destination).
    void
    Inst_MUBUF__BUFFER_LOAD_SBYTE::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_SBYTE::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_SBYTE::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_LOAD_USHORT class methods ---

    Inst_MUBUF__BUFFER_LOAD_USHORT
        ::Inst_MUBUF__BUFFER_LOAD_USHORT(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_ushort")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_LOAD_USHORT

    Inst_MUBUF__BUFFER_LOAD_USHORT::~Inst_MUBUF__BUFFER_LOAD_USHORT()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_USHORT

    // --- description from .arch file ---
    // Untyped buffer load unsigned short (zero extend to VGPR destination).
    void
    Inst_MUBUF__BUFFER_LOAD_USHORT::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);

        rsrcDesc.read();
        offset.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_USHORT::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<VecElemU16>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_USHORT::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDATA);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                if (!oobMask[lane]) {
                    vdst[lane] = (VecElemU32)((reinterpret_cast<VecElemU16*>(
                        gpuDynInst->d_data))[lane]);
                } else {
                    vdst[lane] = 0;
                }
            }
        }

        vdst.write();
    } // execute

    // --- Inst_MUBUF__BUFFER_LOAD_SSHORT class methods ---

    Inst_MUBUF__BUFFER_LOAD_SSHORT
        ::Inst_MUBUF__BUFFER_LOAD_SSHORT(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_sshort")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_LOAD_SSHORT

    Inst_MUBUF__BUFFER_LOAD_SSHORT::~Inst_MUBUF__BUFFER_LOAD_SSHORT()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_SSHORT

    // --- description from .arch file ---
    // Untyped buffer load signed short (sign extend to VGPR destination).
    void
    Inst_MUBUF__BUFFER_LOAD_SSHORT::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_SSHORT::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_SSHORT::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_LOAD_SHORT_D16 class methods ---

    Inst_MUBUF__BUFFER_LOAD_SHORT_D16
        ::Inst_MUBUF__BUFFER_LOAD_SHORT_D16(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_short_d16")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        if (instData.LDS) {
            setFlag(GroupSegment);
            warn("BUFFER.LDS not implemented!");
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_LOAD_SHORT_D16

    Inst_MUBUF__BUFFER_LOAD_SHORT_D16::~Inst_MUBUF__BUFFER_LOAD_SHORT_D16()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_SHORT_D16

    // --- description from .arch file ---
    // RETURN_DATA[15 : 0].u16 = MEM[ADDR].u16;
    // // RETURN_DATA[31:16] is preserved.
    void
    Inst_MUBUF__BUFFER_LOAD_SHORT_D16::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);

        rsrcDesc.read();
        offset.read();

        int inst_offset = instData.OFFSET;

        // For explanation of buffer addressing, see section 9.1.5 in:
        // https://www.amd.com/content/dam/amd/en/documents/instinct-tech-docs/
        //    instruction-set-architectures/
        //    amd-instinct-mi300-cdna3-instruction-set-architecture.pdf
        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_SHORT_D16::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<VecElemU16>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_SHORT_D16::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDATA);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                if (!oobMask[lane]) {
                    VecElemU16 buf_val = (reinterpret_cast<VecElemU16*>(
                        gpuDynInst->d_data))[lane];
                    replaceBits(vdst[lane], 15, 0, buf_val);
                } else {
                    vdst[lane] = 0;
                }
            }
        }

        vdst.write();
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI class methods ---

    Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI
        ::Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_short_d16_hi")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        if (instData.LDS) {
            setFlag(GroupSegment);
            warn("BUFFER.LDS not implemented!");
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI

    Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI::
        ~Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI

    // --- description from .arch file ---
    // VDATA[31 : 16].b16 = MEM[ADDR].b16;
    // // VDATA[15:0] is preserved.
    void
    Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);

        rsrcDesc.read();
        offset.read();

        int inst_offset = instData.OFFSET;

        // For explanation of buffer addressing, see section 9.1.5 in:
        // https://www.amd.com/content/dam/amd/en/documents/instinct-tech-docs/
        //    instruction-set-architectures/
        //    amd-instinct-mi300-cdna3-instruction-set-architecture.pdf
        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<VecElemU16>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_SHORT_D16_HI::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDATA);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                if (!oobMask[lane]) {
                    VecElemU16 buf_val = (reinterpret_cast<VecElemU16*>(
                        gpuDynInst->d_data))[lane];
                    replaceBits(vdst[lane], 31, 16, buf_val);
                } else {
                    vdst[lane] = 0;
                }
            }
        }

        vdst.write();
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_LOAD_DWORD class methods ---

    Inst_MUBUF__BUFFER_LOAD_DWORD
        ::Inst_MUBUF__BUFFER_LOAD_DWORD(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_dword")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_LOAD_DWORD

    Inst_MUBUF__BUFFER_LOAD_DWORD::~Inst_MUBUF__BUFFER_LOAD_DWORD()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_DWORD

    // --- description from .arch file ---
    // Untyped buffer load dword.
    void
    Inst_MUBUF__BUFFER_LOAD_DWORD::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);

        rsrcDesc.read();
        offset.read();

        int inst_offset = instData.OFFSET;

        // For explanation of buffer addressing, see section 9.1.5 in:
        // https://www.amd.com/content/dam/amd/en/documents/instinct-tech-docs/
        //    instruction-set-architectures/
        //    amd-instinct-mi300-cdna3-instruction-set-architecture.pdf
        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_DWORD::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_DWORD::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst(gpuDynInst, extData.VDATA);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                if (!oobMask[lane]) {
                    vdst[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane];
                } else {
                    vdst[lane] = 0;
                }
            }
        }

        vdst.write();
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_LOAD_DWORDX2 class methods ---

    Inst_MUBUF__BUFFER_LOAD_DWORDX2
        ::Inst_MUBUF__BUFFER_LOAD_DWORDX2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_dwordx2")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_LOAD_DWORDX2

    Inst_MUBUF__BUFFER_LOAD_DWORDX2::~Inst_MUBUF__BUFFER_LOAD_DWORDX2()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_DWORDX2

    // --- description from .arch file ---
    // Untyped buffer load 2 dwords.
    void
    Inst_MUBUF__BUFFER_LOAD_DWORDX2::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);

        rsrcDesc.read();
        offset.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_DWORDX2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<2>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_DWORDX2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst0(gpuDynInst, extData.VDATA);
        VecOperandU32 vdst1(gpuDynInst, extData.VDATA + 1);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                if (!oobMask[lane]) {
                    vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * 2];
                    vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * 2 + 1];
                } else {
                    vdst0[lane] = 0;
                    vdst1[lane] = 0;
                }
            }
        }

        vdst0.write();
        vdst1.write();
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_LOAD_DWORDX3 class methods ---

    Inst_MUBUF__BUFFER_LOAD_DWORDX3
        ::Inst_MUBUF__BUFFER_LOAD_DWORDX3(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_dwordx3")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_LOAD_DWORDX3

    Inst_MUBUF__BUFFER_LOAD_DWORDX3::~Inst_MUBUF__BUFFER_LOAD_DWORDX3()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_DWORDX3

    // --- description from .arch file ---
    // Untyped buffer load 3 dwords.
    void
    Inst_MUBUF__BUFFER_LOAD_DWORDX3::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);

        rsrcDesc.read();
        offset.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_DWORDX3::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<3>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_DWORDX3::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst0(gpuDynInst, extData.VDATA);
        VecOperandU32 vdst1(gpuDynInst, extData.VDATA + 1);
        VecOperandU32 vdst2(gpuDynInst, extData.VDATA + 2);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                if (!oobMask[lane]) {
                    vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * 3];
                    vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * 3 + 1];
                    vdst2[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * 3 + 2];
                } else {
                    vdst0[lane] = 0;
                    vdst1[lane] = 0;
                    vdst2[lane] = 0;
                }
            }
        }

        vdst0.write();
        vdst1.write();
        vdst2.write();
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_LOAD_DWORDX4 class methods ---

    Inst_MUBUF__BUFFER_LOAD_DWORDX4
        ::Inst_MUBUF__BUFFER_LOAD_DWORDX4(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_load_dwordx4")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_LOAD_DWORDX4

    Inst_MUBUF__BUFFER_LOAD_DWORDX4::~Inst_MUBUF__BUFFER_LOAD_DWORDX4()
    {
    } // ~Inst_MUBUF__BUFFER_LOAD_DWORDX4

    // --- description from .arch file ---
    // Untyped buffer load 4 dwords.
    void
    Inst_MUBUF__BUFFER_LOAD_DWORDX4::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);

        rsrcDesc.read();
        offset.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_MUBUF__BUFFER_LOAD_DWORDX4::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<4>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_LOAD_DWORDX4::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        VecOperandU32 vdst0(gpuDynInst, extData.VDATA);
        VecOperandU32 vdst1(gpuDynInst, extData.VDATA + 1);
        VecOperandU32 vdst2(gpuDynInst, extData.VDATA + 2);
        VecOperandU32 vdst3(gpuDynInst, extData.VDATA + 3);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                if (!oobMask[lane]) {
                    vdst0[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * 4];
                    vdst1[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * 4 + 1];
                    vdst2[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * 4 + 2];
                    vdst3[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * 4 + 3];
                } else {
                    vdst0[lane] = 0;
                    vdst1[lane] = 0;
                    vdst2[lane] = 0;
                    vdst3[lane] = 0;
                }
            }
        }

        vdst0.write();
        vdst1.write();
        vdst2.write();
        vdst3.write();
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_STORE_BYTE class methods ---

    Inst_MUBUF__BUFFER_STORE_BYTE
        ::Inst_MUBUF__BUFFER_STORE_BYTE(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_byte")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_STORE_BYTE

    Inst_MUBUF__BUFFER_STORE_BYTE::~Inst_MUBUF__BUFFER_STORE_BYTE()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_BYTE

    // --- description from .arch file ---
    // Untyped buffer store byte.
    void
    Inst_MUBUF__BUFFER_STORE_BYTE::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);
        ConstVecOperandI8 data(gpuDynInst, extData.VDATA);

        rsrcDesc.read();
        offset.read();
        data.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

       gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemI8*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_BYTE::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<VecElemI8>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_BYTE::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_STORE_SHORT class methods ---

    Inst_MUBUF__BUFFER_STORE_SHORT
        ::Inst_MUBUF__BUFFER_STORE_SHORT(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_short")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_STORE_SHORT

    Inst_MUBUF__BUFFER_STORE_SHORT::~Inst_MUBUF__BUFFER_STORE_SHORT()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_SHORT

    // --- description from .arch file ---
    // Untyped buffer store short.
    void
    Inst_MUBUF__BUFFER_STORE_SHORT::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);
        ConstVecOperandI16 data(gpuDynInst, extData.VDATA);

        rsrcDesc.read();
        offset.read();
        data.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemI16*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_SHORT::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<VecElemI16>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_SHORT::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MUBUF__BUFFER_STORE_DWORD class methods ---

    Inst_MUBUF__BUFFER_STORE_DWORD::
        Inst_MUBUF__BUFFER_STORE_DWORD(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_dword")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_STORE_DWORD

    Inst_MUBUF__BUFFER_STORE_DWORD::~Inst_MUBUF__BUFFER_STORE_DWORD()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_DWORD

    // --- description from .arch file ---
    // Untyped buffer store dword.
    void
    Inst_MUBUF__BUFFER_STORE_DWORD::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);
        ConstVecOperandU32 data(gpuDynInst, extData.VDATA);

        rsrcDesc.read();
        offset.read();
        data.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane]
                    = data[lane];
            }
        }
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_DWORD::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_DWORD::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_STORE_DWORDX2 class methods ---

    Inst_MUBUF__BUFFER_STORE_DWORDX2
        ::Inst_MUBUF__BUFFER_STORE_DWORDX2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_dwordx2")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_STORE_DWORDX2

    Inst_MUBUF__BUFFER_STORE_DWORDX2::~Inst_MUBUF__BUFFER_STORE_DWORDX2()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_DWORDX2

    // --- description from .arch file ---
    // Untyped buffer store 2 dwords.
    void
    Inst_MUBUF__BUFFER_STORE_DWORDX2::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);
        ConstVecOperandU32 data0(gpuDynInst, extData.VDATA);
        ConstVecOperandU32 data1(gpuDynInst, extData.VDATA + 1);

        rsrcDesc.read();
        offset.read();
        data0.read();
        data1.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane * 2]
                    = data0[lane];
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane*2 + 1]
                    = data1[lane];
            }
        }
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_DWORDX2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<2>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_DWORDX2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_STORE_DWORDX3 class methods ---

    Inst_MUBUF__BUFFER_STORE_DWORDX3
        ::Inst_MUBUF__BUFFER_STORE_DWORDX3(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_dwordx3")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_STORE_DWORDX3

    Inst_MUBUF__BUFFER_STORE_DWORDX3::~Inst_MUBUF__BUFFER_STORE_DWORDX3()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_DWORDX3

    // --- description from .arch file ---
    // Untyped buffer store 3 dwords.
    void
    Inst_MUBUF__BUFFER_STORE_DWORDX3::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);
        ConstVecOperandU32 data0(gpuDynInst, extData.VDATA);
        ConstVecOperandU32 data1(gpuDynInst, extData.VDATA + 1);
        ConstVecOperandU32 data2(gpuDynInst, extData.VDATA + 2);

        rsrcDesc.read();
        offset.read();
        data0.read();
        data1.read();
        data2.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane * 3]
                    = data0[lane];
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane*3 + 1]
                    = data1[lane];
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane*3 + 2]
                    = data2[lane];
            }
        }
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_DWORDX3::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<3>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_DWORDX3::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_STORE_DWORDX4 class methods ---

    Inst_MUBUF__BUFFER_STORE_DWORDX4
        ::Inst_MUBUF__BUFFER_STORE_DWORDX4(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_dwordx4")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        if (instData.LDS) {
            setFlag(GroupSegment);
        } else {
            setFlag(GlobalSegment);
        }
    } // Inst_MUBUF__BUFFER_STORE_DWORDX4

    Inst_MUBUF__BUFFER_STORE_DWORDX4::~Inst_MUBUF__BUFFER_STORE_DWORDX4()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_DWORDX4

    // --- description from .arch file ---
    // Untyped buffer store 4 dwords.
    void
    Inst_MUBUF__BUFFER_STORE_DWORDX4::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            wf->decExpInstsIssued();
            wf->untrackExpInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);
        ConstVecOperandU32 data0(gpuDynInst, extData.VDATA);
        ConstVecOperandU32 data1(gpuDynInst, extData.VDATA + 1);
        ConstVecOperandU32 data2(gpuDynInst, extData.VDATA + 2);
        ConstVecOperandU32 data3(gpuDynInst, extData.VDATA + 3);

        rsrcDesc.read();
        offset.read();
        data0.read();
        data1.read();
        data2.read();
        data3.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane * 4]
                    = data0[lane];
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane*4 + 1]
                    = data1[lane];
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane*4 + 2]
                    = data2[lane];
                (reinterpret_cast<VecElemU32*>(gpuDynInst->d_data))[lane*4 + 3]
                    = data3[lane];
            }
        }
    } // execute

    void
    Inst_MUBUF__BUFFER_STORE_DWORDX4::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<4>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_STORE_DWORDX4::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_STORE_LDS_DWORD class methods ---

    Inst_MUBUF__BUFFER_STORE_LDS_DWORD
        ::Inst_MUBUF__BUFFER_STORE_LDS_DWORD(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_store_lds_dword")
    {
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_STORE_LDS_DWORD

    Inst_MUBUF__BUFFER_STORE_LDS_DWORD::~Inst_MUBUF__BUFFER_STORE_LDS_DWORD()
    {
    } // ~Inst_MUBUF__BUFFER_STORE_LDS_DWORD

    // --- description from .arch file ---
    // Store one DWORD from LDS memory to system memory without utilizing
    // VGPRs.
    void
    Inst_MUBUF__BUFFER_STORE_LDS_DWORD::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_WBINVL1 class methods ---

    Inst_MUBUF__BUFFER_WBINVL1::Inst_MUBUF__BUFFER_WBINVL1(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_wbinvl1")
    {
        setFlag(MemoryRef);
        setFlag(GPUStaticInst::MemSync);
        setFlag(GlobalSegment);
        setFlag(MemSync);
    } // Inst_MUBUF__BUFFER_WBINVL1

    Inst_MUBUF__BUFFER_WBINVL1::~Inst_MUBUF__BUFFER_WBINVL1()
    {
    } // ~Inst_MUBUF__BUFFER_WBINVL1

    // --- description from .arch file ---
    // Write back and invalidate the shader L1.
    // Always returns ACK to shader.
    void
    Inst_MUBUF__BUFFER_WBINVL1::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        if (gpuDynInst->executedAs() == enums::SC_GLOBAL) {
            gpuDynInst->computeUnit()->globalMemoryPipe.
                issueRequest(gpuDynInst);
        } else {
            fatal("Unsupported scope for flat instruction.\n");
        }
    } // execute

    void
    Inst_MUBUF__BUFFER_WBINVL1::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        // TODO: Fix it for gfx10. Once we have the new gfx10 cache model, we
        // need to precisely communicate the writeback-invalidate operation to
        // the new gfx10 coalescer rather than sending AcquireRelease markers.
        // The SICoalescer would need to be updated appropriately as well.
        injectGlobalMemFence(gpuDynInst);
    } // initiateAcc
    void
    Inst_MUBUF__BUFFER_WBINVL1::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_WBINVL1_VOL class methods ---

    Inst_MUBUF__BUFFER_WBINVL1_VOL
        ::Inst_MUBUF__BUFFER_WBINVL1_VOL(InFmt_MUBUF*iFmt)
        : Inst_MUBUF(iFmt, "buffer_wbinvl1_vol") {
        // This instruction is same as buffer_wbinvl1 instruction except this
        // instruction only invalidate L1 shader line with MTYPE SC and GC.
        // Since Hermes L1 (TCP) do not differentiate between its cache lines,
        // this instruction currently behaves (and implemented ) exactly like
        // buffer_wbinvl1 instruction.
        setFlag(MemoryRef);
        setFlag(GPUStaticInst::MemSync);
        setFlag(GlobalSegment);
        setFlag(MemSync);
    } // Inst_MUBUF__BUFFER_WBINVL1_VOL

    Inst_MUBUF__BUFFER_WBINVL1_VOL::~Inst_MUBUF__BUFFER_WBINVL1_VOL()
    {
    } // ~Inst_MUBUF__BUFFER_WBINVL1_VOL

    // --- description from .arch file ---
    // Write back and invalidate the shader L1 only for lines that are marked
    // ---  volatile.
    // Always returns ACK to shader.
    void
    Inst_MUBUF__BUFFER_WBINVL1_VOL::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        if (gpuDynInst->executedAs() == enums::SC_GLOBAL) {
            gpuDynInst->computeUnit()->globalMemoryPipe.
                issueRequest(gpuDynInst);
        } else {
            fatal("Unsupported scope for flat instruction.\n");
        }
    } // execute
    void
    Inst_MUBUF__BUFFER_WBINVL1_VOL::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        injectGlobalMemFence(gpuDynInst);
    } // initiateAcc
    void
    Inst_MUBUF__BUFFER_WBINVL1_VOL::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_ATOMIC_SWAP class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_SWAP
        ::Inst_MUBUF__BUFFER_ATOMIC_SWAP(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_swap")
    {
        setFlag(AtomicExch);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_SWAP

    Inst_MUBUF__BUFFER_ATOMIC_SWAP::~Inst_MUBUF__BUFFER_ATOMIC_SWAP()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_SWAP

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = DATA;
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_SWAP::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP
        ::Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_cmpswap")
    {
        setFlag(AtomicCAS);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP

    Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP::~Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // src = DATA[0];
    // cmp = DATA[1];
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();

        if (gpuDynInst->exec_mask.none()) {
            wf->decVMemInstsIssued();
            wf->untrackVMemInst(gpuDynInst);
            return;
        }

        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

        ConstVecOperandU32 addr0(gpuDynInst, extData.VADDR);
        ConstVecOperandU32 addr1(gpuDynInst, extData.VADDR + 1);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, extData.SRSRC * 4);
        ConstScalarOperandU32 offset(gpuDynInst, extData.SOFFSET);
        ConstVecOperandU32 src(gpuDynInst, extData.VDATA);
        ConstVecOperandU32 cmp(gpuDynInst, extData.VDATA + 1);

        rsrcDesc.read();
        offset.read();
        src.read();
        cmp.read();

        int inst_offset = instData.OFFSET;

        if (!instData.IDXEN && !instData.OFFEN) {
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (!instData.IDXEN && instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr0, addr1, rsrcDesc, offset, inst_offset);
        } else if (instData.IDXEN && !instData.OFFEN) {
            addr0.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        } else {
            addr0.read();
            addr1.read();
            calcAddr<ConstVecOperandU32, ConstVecOperandU32,
                ConstScalarOperandU128, ConstScalarOperandU32>(gpuDynInst,
                    addr1, addr0, rsrcDesc, offset, inst_offset);
        }

        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            if (gpuDynInst->exec_mask[lane]) {
                (reinterpret_cast<VecElemU32*>(gpuDynInst->x_data))[lane]
                    = src[lane];
                (reinterpret_cast<VecElemU32*>(gpuDynInst->a_data))[lane]
                    = cmp[lane];
            }
        }

        gpuDynInst->computeUnit()->globalMemoryPipe.issueRequest(gpuDynInst);
    } // execute

    void
    Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initAtomicAccess<VecElemU32>(gpuDynInst);
    } // initiateAcc

    void
    Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        if (isAtomicRet()) {
            VecOperandU32 vdst(gpuDynInst, extData.VDATA);

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    vdst[lane] = (reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane];
                }
            }

            vdst.write();
        }
    } // completeAcc
    // --- Inst_MUBUF__BUFFER_ATOMIC_ADD class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_ADD
        ::Inst_MUBUF__BUFFER_ATOMIC_ADD(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_add")
    {
        setFlag(AtomicAdd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_ADD

    Inst_MUBUF__BUFFER_ATOMIC_ADD::~Inst_MUBUF__BUFFER_ATOMIC_ADD()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_ADD

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] += DATA;
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_ADD::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_SUB class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_SUB
        ::Inst_MUBUF__BUFFER_ATOMIC_SUB(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_sub")
    {
        setFlag(AtomicSub);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_SUB

    Inst_MUBUF__BUFFER_ATOMIC_SUB::~Inst_MUBUF__BUFFER_ATOMIC_SUB()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_SUB

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_SUB::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_SMIN class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_SMIN
        ::Inst_MUBUF__BUFFER_ATOMIC_SMIN(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_smin")
    {
        setFlag(AtomicMin);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_SMIN

    Inst_MUBUF__BUFFER_ATOMIC_SMIN::~Inst_MUBUF__BUFFER_ATOMIC_SMIN()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_SMIN

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA < tmp) ? DATA : tmp (signed compare);
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_SMIN::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_UMIN class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_UMIN
        ::Inst_MUBUF__BUFFER_ATOMIC_UMIN(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_umin")
    {
        setFlag(AtomicMin);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_UMIN

    Inst_MUBUF__BUFFER_ATOMIC_UMIN::~Inst_MUBUF__BUFFER_ATOMIC_UMIN()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_UMIN

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA < tmp) ? DATA : tmp (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_UMIN::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_SMAX class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_SMAX
        ::Inst_MUBUF__BUFFER_ATOMIC_SMAX(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_smax")
    {
        setFlag(AtomicMax);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_SMAX

    Inst_MUBUF__BUFFER_ATOMIC_SMAX::~Inst_MUBUF__BUFFER_ATOMIC_SMAX()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_SMAX

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA > tmp) ? DATA : tmp (signed compare);
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_SMAX::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_UMAX class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_UMAX
        ::Inst_MUBUF__BUFFER_ATOMIC_UMAX(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_umax")
    {
        setFlag(AtomicMax);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_UMAX

    Inst_MUBUF__BUFFER_ATOMIC_UMAX::~Inst_MUBUF__BUFFER_ATOMIC_UMAX()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_UMAX

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (DATA > tmp) ? DATA : tmp (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_UMAX::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_AND class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_AND
        ::Inst_MUBUF__BUFFER_ATOMIC_AND(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_and")
    {
        setFlag(AtomicAnd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_AND

    Inst_MUBUF__BUFFER_ATOMIC_AND::~Inst_MUBUF__BUFFER_ATOMIC_AND()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_AND

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] &= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_AND::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_OR class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_OR
        ::Inst_MUBUF__BUFFER_ATOMIC_OR(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_or")
    {
        setFlag(AtomicOr);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_OR

    Inst_MUBUF__BUFFER_ATOMIC_OR::~Inst_MUBUF__BUFFER_ATOMIC_OR()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_OR

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] |= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_OR::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_XOR class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_XOR
        ::Inst_MUBUF__BUFFER_ATOMIC_XOR(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_xor")
    {
        setFlag(AtomicXor);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_XOR

    Inst_MUBUF__BUFFER_ATOMIC_XOR::~Inst_MUBUF__BUFFER_ATOMIC_XOR()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_XOR

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] ^= DATA;
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_XOR::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_INC class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_INC
        ::Inst_MUBUF__BUFFER_ATOMIC_INC(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_inc")
    {
        setFlag(AtomicInc);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_INC

    Inst_MUBUF__BUFFER_ATOMIC_INC::~Inst_MUBUF__BUFFER_ATOMIC_INC()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_INC

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp >= DATA) ? 0 : tmp + 1 (unsigned compare);
    // RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_INC::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_DEC class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_DEC
        ::Inst_MUBUF__BUFFER_ATOMIC_DEC(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_dec")
    {
        setFlag(AtomicDec);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_DEC

    Inst_MUBUF__BUFFER_ATOMIC_DEC::~Inst_MUBUF__BUFFER_ATOMIC_DEC()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_DEC

    // --- description from .arch file ---
    // 32b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp == 0 || tmp > DATA) ? DATA : tmp - 1
    // (unsigned compare); RETURN_DATA = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_DEC::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_SWAP_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_SWAP_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_SWAP_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_swap_x2")
    {
        setFlag(AtomicExch);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_SWAP_X2

    Inst_MUBUF__BUFFER_ATOMIC_SWAP_X2::~Inst_MUBUF__BUFFER_ATOMIC_SWAP_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_SWAP_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_SWAP_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_cmpswap_x2")
    {
        setFlag(AtomicCAS);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2

    Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2
        ::~Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // src = DATA[0:1];
    // cmp = DATA[2:3];
    // MEM[ADDR] = (tmp == cmp) ? src : tmp;
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_CMPSWAP_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_ADD_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_ADD_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_ADD_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_add_x2")
    {
        setFlag(AtomicAdd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_ADD_X2

    Inst_MUBUF__BUFFER_ATOMIC_ADD_X2::~Inst_MUBUF__BUFFER_ATOMIC_ADD_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_ADD_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] += DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_ADD_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_SUB_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_SUB_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_SUB_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_sub_x2")
    {
        setFlag(AtomicSub);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_SUB_X2

    Inst_MUBUF__BUFFER_ATOMIC_SUB_X2::~Inst_MUBUF__BUFFER_ATOMIC_SUB_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_SUB_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_SUB_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_SMIN_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_SMIN_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_SMIN_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_smin_x2")
    {
        setFlag(AtomicMin);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_SMIN_X2

    Inst_MUBUF__BUFFER_ATOMIC_SMIN_X2::~Inst_MUBUF__BUFFER_ATOMIC_SMIN_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_SMIN_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] < tmp) ? DATA[0:1] : tmp (signed compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_SMIN_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_UMIN_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_UMIN_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_UMIN_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_umin_x2")
    {
        setFlag(AtomicMin);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_UMIN_X2

    Inst_MUBUF__BUFFER_ATOMIC_UMIN_X2::~Inst_MUBUF__BUFFER_ATOMIC_UMIN_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_UMIN_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] < tmp) ? DATA[0:1] : tmp (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_UMIN_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_SMAX_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_SMAX_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_SMAX_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_smax_x2")
    {
        setFlag(AtomicMax);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_SMAX_X2

    Inst_MUBUF__BUFFER_ATOMIC_SMAX_X2::~Inst_MUBUF__BUFFER_ATOMIC_SMAX_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_SMAX_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] > tmp) ? DATA[0:1] : tmp (signed compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_SMAX_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_UMAX_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_UMAX_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_UMAX_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_umax_x2")
    {
        setFlag(AtomicMax);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_UMAX_X2

    Inst_MUBUF__BUFFER_ATOMIC_UMAX_X2::~Inst_MUBUF__BUFFER_ATOMIC_UMAX_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_UMAX_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] -= (DATA[0:1] > tmp) ? DATA[0:1] : tmp (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_UMAX_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_AND_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_AND_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_AND_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_and_x2")
    {
        setFlag(AtomicAnd);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_AND_X2

    Inst_MUBUF__BUFFER_ATOMIC_AND_X2::~Inst_MUBUF__BUFFER_ATOMIC_AND_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_AND_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] &= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_AND_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_OR_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_OR_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_OR_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_or_x2")
    {
        setFlag(AtomicOr);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
    } // Inst_MUBUF__BUFFER_ATOMIC_OR_X2

    Inst_MUBUF__BUFFER_ATOMIC_OR_X2::~Inst_MUBUF__BUFFER_ATOMIC_OR_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_OR_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] |= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_OR_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_XOR_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_XOR_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_XOR_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_xor_x2")
    {
        setFlag(AtomicXor);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_XOR_X2

    Inst_MUBUF__BUFFER_ATOMIC_XOR_X2::~Inst_MUBUF__BUFFER_ATOMIC_XOR_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_XOR_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] ^= DATA[0:1];
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_XOR_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_INC_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_INC_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_INC_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_inc_x2")
    {
        setFlag(AtomicInc);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_INC_X2

    Inst_MUBUF__BUFFER_ATOMIC_INC_X2::~Inst_MUBUF__BUFFER_ATOMIC_INC_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_INC_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp >= DATA[0:1]) ? 0 : tmp + 1 (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_INC_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_MUBUF__BUFFER_ATOMIC_DEC_X2 class methods ---

    Inst_MUBUF__BUFFER_ATOMIC_DEC_X2
        ::Inst_MUBUF__BUFFER_ATOMIC_DEC_X2(InFmt_MUBUF *iFmt)
        : Inst_MUBUF(iFmt, "buffer_atomic_dec_x2")
    {
        setFlag(AtomicDec);
        if (instData.GLC) {
            setFlag(AtomicReturn);
        } else {
            setFlag(AtomicNoReturn);
        }
        setFlag(MemoryRef);
        setFlag(GlobalSegment);
    } // Inst_MUBUF__BUFFER_ATOMIC_DEC_X2

    Inst_MUBUF__BUFFER_ATOMIC_DEC_X2::~Inst_MUBUF__BUFFER_ATOMIC_DEC_X2()
    {
    } // ~Inst_MUBUF__BUFFER_ATOMIC_DEC_X2

    // --- description from .arch file ---
    // 64b:
    // tmp = MEM[ADDR];
    // MEM[ADDR] = (tmp == 0 || tmp > DATA[0:1]) ? DATA[0:1] : tmp - 1
    // (unsigned compare);
    // RETURN_DATA[0:1] = tmp.
    void
    Inst_MUBUF__BUFFER_ATOMIC_DEC_X2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
} // namespace VegaISA
} // namespace gem5
