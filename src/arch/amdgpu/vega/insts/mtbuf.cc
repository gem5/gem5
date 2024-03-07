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
    // --- Inst_MTBUF__TBUFFER_LOAD_FORMAT_X class methods ---

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_X
        ::Inst_MTBUF__TBUFFER_LOAD_FORMAT_X(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_load_format_x")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_LOAD_FORMAT_X

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_X::~Inst_MTBUF__TBUFFER_LOAD_FORMAT_X()
    {
    } // ~Inst_MTBUF__TBUFFER_LOAD_FORMAT_X

    // --- description from .arch file ---
    // Typed buffer load 1 dword with format conversion.
    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_X::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_X::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_X::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY class methods ---

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY
        ::Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_load_format_xy")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY::~Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY()
    {
    } // ~Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY

    // --- description from .arch file ---
    // Typed buffer load 2 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XY::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ class methods ---

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ
        ::Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_load_format_xyz")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ::~Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ()
    {
    } // ~Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ

    // --- description from .arch file ---
    // Typed buffer load 3 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZ::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW class methods ---

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW
        ::Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_load_format_xyzw")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW
        ::~Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW()
    {
    } // ~Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW

    // --- description from .arch file ---
    // Typed buffer load 4 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_XYZW::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_STORE_FORMAT_X class methods ---

    Inst_MTBUF__TBUFFER_STORE_FORMAT_X
        ::Inst_MTBUF__TBUFFER_STORE_FORMAT_X(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_store_format_x")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_STORE_FORMAT_X

    Inst_MTBUF__TBUFFER_STORE_FORMAT_X::~Inst_MTBUF__TBUFFER_STORE_FORMAT_X()
    {
    } // ~Inst_MTBUF__TBUFFER_STORE_FORMAT_X

    // --- description from .arch file ---
    // Typed buffer store 1 dword with format conversion.
    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_X::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_X::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_X::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_STORE_FORMAT_XY class methods ---

    Inst_MTBUF__TBUFFER_STORE_FORMAT_XY
        ::Inst_MTBUF__TBUFFER_STORE_FORMAT_XY(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_store_format_xy")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_STORE_FORMAT_XY

    Inst_MTBUF__TBUFFER_STORE_FORMAT_XY::~Inst_MTBUF__TBUFFER_STORE_FORMAT_XY()
    {
    } // ~Inst_MTBUF__TBUFFER_STORE_FORMAT_XY

    // --- description from .arch file ---
    // Typed buffer store 2 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_XY::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_XY::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_XY::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ class methods ---

    Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ
        ::Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_store_format_xyz")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ

    Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ
        ::~Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ()
    {
    } // ~Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ

    // --- description from .arch file ---
    // Typed buffer store 3 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZ::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW class methods ---

    Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW
        ::Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_store_format_xyzw")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW

    Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW
        ::~Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW()
    {
    } // ~Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW

    // --- description from .arch file ---
    // Typed buffer store 4 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_XYZW::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X class methods ---

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X
        ::Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_load_format_d16_x")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X::
        ~Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X()
    {
    } // ~Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X

    // --- description from .arch file ---
    // Typed buffer load 1 dword with format conversion.
    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X::initiateAcc(
          GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_X::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY class methods ---

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY
        ::Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_load_format_d16_xy")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY
        ::~Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY()
    {
    } // ~Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY

    // --- description from .arch file ---
    // Typed buffer load 2 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XY::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ class methods ---

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ
        ::Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ(
          InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_load_format_d16_xyz")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ
        ::~Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ()
    {
    } // ~Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ

    // --- description from .arch file ---
    // Typed buffer load 3 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZ::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW class methods ---

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW
        ::Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW(
          InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_load_format_d16_xyzw")
    {
        setFlag(MemoryRef);
        setFlag(Load);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW

    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW
        ::~Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW()
    {
    } // ~Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW

    // --- description from .arch file ---
    // Typed buffer load 4 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_LOAD_FORMAT_D16_XYZW::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X class methods ---

    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X
        ::Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_store_format_d16_x")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X

    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X
        ::~Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X()
    {
    } // ~Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X

    // --- description from .arch file ---
    // Typed buffer store 1 dword with format conversion.
    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_X::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY class methods ---

    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY
        ::Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_store_format_d16_xy")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY

    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY
        ::~Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY()
    {
    } // ~Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY

    // --- description from .arch file ---
    // Typed buffer store 2 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XY::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ class methods ---

    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ
        ::Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_store_format_d16_xyz")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ

    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ
        ::~Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ()
    {
    } // ~Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ

    // --- description from .arch file ---
    // Typed buffer store 3 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ::initiateAcc(
          GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZ::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
    // --- Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW class methods ---

    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW
        ::Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW(InFmt_MTBUF *iFmt)
        : Inst_MTBUF(iFmt, "tbuffer_store_format_d16_xyzw")
    {
        setFlag(MemoryRef);
        setFlag(Store);
        setFlag(GlobalSegment);
    } // Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW

    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW
        ::~Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW()
    {
    } // ~Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW

    // --- description from .arch file ---
    // Typed buffer store 4 dwords with format conversion.
    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW::execute(
        GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW::initiateAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_MTBUF__TBUFFER_STORE_FORMAT_D16_XYZW::completeAcc(
        GPUDynInstPtr gpuDynInst)
    {
    } // execute
} // namespace VegaISA
} // namespace gem5
