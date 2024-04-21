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
#include "instructions.hh"

namespace gem5
{

namespace VegaISA
{
    // --- Inst_SMEM__S_LOAD_DWORD class methods ---

    Inst_SMEM__S_LOAD_DWORD::Inst_SMEM__S_LOAD_DWORD(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_load_dword")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_LOAD_DWORD

    Inst_SMEM__S_LOAD_DWORD::~Inst_SMEM__S_LOAD_DWORD()
    {
    } // ~Inst_SMEM__S_LOAD_DWORD

    /**
     * Read 1 dword from scalar data cache. If the offset is specified as an
     * sgpr, the sgpr contains an unsigned byte offset (the 2 LSBs are
     * ignored). If the offset is specified as an immediate 20-bit constant,
     * the constant is an unsigned byte offset.
     */
    void
    Inst_SMEM__S_LOAD_DWORD::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU64 addr(gpuDynInst, instData.SBASE << 1);

        addr.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, addr, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe
            .issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_LOAD_DWORD::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<1>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_LOAD_DWORD::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        ScalarOperandU32 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_LOAD_DWORDX2 class methods ---

    Inst_SMEM__S_LOAD_DWORDX2::Inst_SMEM__S_LOAD_DWORDX2(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_load_dwordx2")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_LOAD_DWORDX2

    Inst_SMEM__S_LOAD_DWORDX2::~Inst_SMEM__S_LOAD_DWORDX2()
    {
    } // ~Inst_SMEM__S_LOAD_DWORDX2

    /**
     * Read 2 dwords from scalar data cache. See s_load_dword for details on
     * the offset input.
     */
    void
    Inst_SMEM__S_LOAD_DWORDX2::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU64 addr(gpuDynInst, instData.SBASE << 1);

        addr.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, addr, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe.
            issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_LOAD_DWORDX2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<2>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_LOAD_DWORDX2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        ScalarOperandU64 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_LOAD_DWORDX4 class methods ---

    Inst_SMEM__S_LOAD_DWORDX4::Inst_SMEM__S_LOAD_DWORDX4(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_load_dwordx4")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_LOAD_DWORDX4

    Inst_SMEM__S_LOAD_DWORDX4::~Inst_SMEM__S_LOAD_DWORDX4()
    {
    } // ~Inst_SMEM__S_LOAD_DWORDX4

    // --- description from .arch file ---
    // Read 4 dwords from scalar data cache. See S_LOAD_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_LOAD_DWORDX4::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU64 addr(gpuDynInst, instData.SBASE << 1);

        addr.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, addr, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe.
            issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_LOAD_DWORDX4::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<4>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_LOAD_DWORDX4::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        ScalarOperandU128 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_LOAD_DWORDX8 class methods ---

    Inst_SMEM__S_LOAD_DWORDX8::Inst_SMEM__S_LOAD_DWORDX8(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_load_dwordx8")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_LOAD_DWORDX8

    Inst_SMEM__S_LOAD_DWORDX8::~Inst_SMEM__S_LOAD_DWORDX8()
    {
    } // ~Inst_SMEM__S_LOAD_DWORDX8

    // --- description from .arch file ---
    // Read 8 dwords from scalar data cache. See S_LOAD_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_LOAD_DWORDX8::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU64 addr(gpuDynInst, instData.SBASE << 1);

        addr.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, addr, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe.
            issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_LOAD_DWORDX8::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<8>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_LOAD_DWORDX8::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        ScalarOperandU256 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_LOAD_DWORDX16 class methods ---

    Inst_SMEM__S_LOAD_DWORDX16::Inst_SMEM__S_LOAD_DWORDX16(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_load_dwordx16")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_LOAD_DWORDX16

    Inst_SMEM__S_LOAD_DWORDX16::~Inst_SMEM__S_LOAD_DWORDX16()
    {
    } // ~Inst_SMEM__S_LOAD_DWORDX16

    // --- description from .arch file ---
    // Read 16 dwords from scalar data cache. See S_LOAD_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_LOAD_DWORDX16::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU64 addr(gpuDynInst, instData.SBASE << 1);

        addr.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, addr, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe.
            issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_LOAD_DWORDX16::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<16>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_LOAD_DWORDX16::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        ScalarOperandU512 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_BUFFER_LOAD_DWORD class methods ---

    Inst_SMEM__S_BUFFER_LOAD_DWORD::Inst_SMEM__S_BUFFER_LOAD_DWORD(
          InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_buffer_load_dword")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_BUFFER_LOAD_DWORD

    Inst_SMEM__S_BUFFER_LOAD_DWORD::~Inst_SMEM__S_BUFFER_LOAD_DWORD()
    {
    } // ~Inst_SMEM__S_BUFFER_LOAD_DWORD

    // --- description from .arch file ---
    // Read 1 dword from scalar data cache. See S_LOAD_DWORD for details on the
    // ---  offset input.
    void
    Inst_SMEM__S_BUFFER_LOAD_DWORD::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, instData.SBASE);

        rsrcDesc.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, rsrcDesc, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe
            .issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_BUFFER_LOAD_DWORD::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<1>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_BUFFER_LOAD_DWORD::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        // 1 request, size 32
        ScalarOperandU32 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_BUFFER_LOAD_DWORDX2 class methods ---

    Inst_SMEM__S_BUFFER_LOAD_DWORDX2::Inst_SMEM__S_BUFFER_LOAD_DWORDX2(
          InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_buffer_load_dwordx2")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_BUFFER_LOAD_DWORDX2

    Inst_SMEM__S_BUFFER_LOAD_DWORDX2::~Inst_SMEM__S_BUFFER_LOAD_DWORDX2()
    {
    } // ~Inst_SMEM__S_BUFFER_LOAD_DWORDX2

    // --- description from .arch file ---
    // Read 2 dwords from scalar data cache. See S_LOAD_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX2::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, instData.SBASE);

        rsrcDesc.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, rsrcDesc, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe
            .issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<2>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        // use U64 because 2 requests, each size 32
        ScalarOperandU64 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_BUFFER_LOAD_DWORDX4 class methods ---

    Inst_SMEM__S_BUFFER_LOAD_DWORDX4::Inst_SMEM__S_BUFFER_LOAD_DWORDX4(
          InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_buffer_load_dwordx4")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_BUFFER_LOAD_DWORDX4

    Inst_SMEM__S_BUFFER_LOAD_DWORDX4::~Inst_SMEM__S_BUFFER_LOAD_DWORDX4()
    {
    } // ~Inst_SMEM__S_BUFFER_LOAD_DWORDX4

    // --- description from .arch file ---
    // Read 4 dwords from scalar data cache. See S_LOAD_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX4::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, instData.SBASE);

        rsrcDesc.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, rsrcDesc, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe
            .issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX4::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<4>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX4::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        // 4 requests, each size 32
        ScalarOperandU128 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_BUFFER_LOAD_DWORDX8 class methods ---

    Inst_SMEM__S_BUFFER_LOAD_DWORDX8::Inst_SMEM__S_BUFFER_LOAD_DWORDX8(
          InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_buffer_load_dwordx8")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_BUFFER_LOAD_DWORDX8

    Inst_SMEM__S_BUFFER_LOAD_DWORDX8::~Inst_SMEM__S_BUFFER_LOAD_DWORDX8()
    {
    } // ~Inst_SMEM__S_BUFFER_LOAD_DWORDX8

    // --- description from .arch file ---
    // Read 8 dwords from scalar data cache. See S_LOAD_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX8::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, instData.SBASE);

        rsrcDesc.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, rsrcDesc, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe
            .issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX8::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<8>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX8::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        // 8 requests, each size 32
        ScalarOperandU256 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_BUFFER_LOAD_DWORDX16 class methods ---

    Inst_SMEM__S_BUFFER_LOAD_DWORDX16::Inst_SMEM__S_BUFFER_LOAD_DWORDX16(
          InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_buffer_load_dwordx16")
    {
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_BUFFER_LOAD_DWORDX16

    Inst_SMEM__S_BUFFER_LOAD_DWORDX16::~Inst_SMEM__S_BUFFER_LOAD_DWORDX16()
    {
    } // ~Inst_SMEM__S_BUFFER_LOAD_DWORDX16

    // --- description from .arch file ---
    // Read 16 dwords from scalar data cache. See S_LOAD_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX16::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, instData.SBASE);

        rsrcDesc.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, rsrcDesc, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe
            .issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX16::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<16>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_BUFFER_LOAD_DWORDX16::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        // 16 requests, each size 32
        ScalarOperandU512 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_STORE_DWORD class methods ---

    Inst_SMEM__S_STORE_DWORD::Inst_SMEM__S_STORE_DWORD(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_store_dword")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_SMEM__S_STORE_DWORD

    Inst_SMEM__S_STORE_DWORD::~Inst_SMEM__S_STORE_DWORD()
    {
    } // ~Inst_SMEM__S_STORE_DWORD

    // --- description from .arch file ---
    // Write 1 dword to scalar data cache.
    // If the offset is specified as an SGPR, the SGPR contains an unsigned
    // BYTE offset (the 2 LSBs are ignored).
    // If the offset is specified as an immediate 20-bit constant, the
    // constant is an unsigned BYTE offset.
    void
    Inst_SMEM__S_STORE_DWORD::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU64 addr(gpuDynInst, instData.SBASE << 1);
        ConstScalarOperandU32 sdata(gpuDynInst, instData.SDATA);

        addr.read();
        sdata.read();

        std::memcpy((void*)gpuDynInst->scalar_data, sdata.rawDataPtr(),
            sizeof(ScalarRegU32));

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, addr, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe.
            issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_STORE_DWORD::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<1>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_STORE_DWORD::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_SMEM__S_STORE_DWORDX2 class methods ---

    Inst_SMEM__S_STORE_DWORDX2::Inst_SMEM__S_STORE_DWORDX2(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_store_dwordx2")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_SMEM__S_STORE_DWORDX2

    Inst_SMEM__S_STORE_DWORDX2::~Inst_SMEM__S_STORE_DWORDX2()
    {
    } // ~Inst_SMEM__S_STORE_DWORDX2

    // --- description from .arch file ---
    // Write 2 dwords to scalar data cache. See S_STORE_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_STORE_DWORDX2::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU64 addr(gpuDynInst, instData.SBASE << 1);
        ConstScalarOperandU64 sdata(gpuDynInst, instData.SDATA);

        addr.read();
        sdata.read();

        std::memcpy((void*)gpuDynInst->scalar_data, sdata.rawDataPtr(),
            sizeof(ScalarRegU64));

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, addr, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe.
            issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_STORE_DWORDX2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<2>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_STORE_DWORDX2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_SMEM__S_STORE_DWORDX4 class methods ---

    Inst_SMEM__S_STORE_DWORDX4::Inst_SMEM__S_STORE_DWORDX4(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_store_dwordx4")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_SMEM__S_STORE_DWORDX4

    Inst_SMEM__S_STORE_DWORDX4::~Inst_SMEM__S_STORE_DWORDX4()
    {
    } // ~Inst_SMEM__S_STORE_DWORDX4

    // --- description from .arch file ---
    // Write 4 dwords to scalar data cache. See S_STORE_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_STORE_DWORDX4::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());
        ScalarRegU32 offset(0);
        ConstScalarOperandU64 addr(gpuDynInst, instData.SBASE << 1);
        ConstScalarOperandU64 sdata(gpuDynInst, instData.SDATA);

        addr.read();
        sdata.read();

        std::memcpy((void*)gpuDynInst->scalar_data, sdata.rawDataPtr(),
            sizeof(gpuDynInst->scalar_data));

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, addr, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe.
            issueRequest(gpuDynInst);
    } // execute

    void
    Inst_SMEM__S_STORE_DWORDX4::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemWrite<4>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_STORE_DWORDX4::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_SMEM__S_BUFFER_STORE_DWORD class methods ---

    Inst_SMEM__S_BUFFER_STORE_DWORD::Inst_SMEM__S_BUFFER_STORE_DWORD(
          InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_buffer_store_dword")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_SMEM__S_BUFFER_STORE_DWORD

    Inst_SMEM__S_BUFFER_STORE_DWORD::~Inst_SMEM__S_BUFFER_STORE_DWORD()
    {
    } // ~Inst_SMEM__S_BUFFER_STORE_DWORD

    // --- description from .arch file ---
    // Write 1 dword to scalar data cache. See S_STORE_DWORD for details on the
    // ---  offset input.
    void
    Inst_SMEM__S_BUFFER_STORE_DWORD::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_SMEM__S_BUFFER_STORE_DWORD::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_SMEM__S_BUFFER_STORE_DWORD::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_SMEM__S_BUFFER_STORE_DWORDX2 class methods ---

    Inst_SMEM__S_BUFFER_STORE_DWORDX2::Inst_SMEM__S_BUFFER_STORE_DWORDX2(
          InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_buffer_store_dwordx2")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_SMEM__S_BUFFER_STORE_DWORDX2

    Inst_SMEM__S_BUFFER_STORE_DWORDX2::~Inst_SMEM__S_BUFFER_STORE_DWORDX2()
    {
    } // ~Inst_SMEM__S_BUFFER_STORE_DWORDX2

    // --- description from .arch file ---
    // Write 2 dwords to scalar data cache. See S_STORE_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_BUFFER_STORE_DWORDX2::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_SMEM__S_BUFFER_STORE_DWORDX2::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_SMEM__S_BUFFER_STORE_DWORDX2::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_SMEM__S_BUFFER_STORE_DWORDX4 class methods ---

    Inst_SMEM__S_BUFFER_STORE_DWORDX4::Inst_SMEM__S_BUFFER_STORE_DWORDX4(
          InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_buffer_store_dwordx4")
    {
        setFlag(MemoryRef);
        setFlag(Store);
    } // Inst_SMEM__S_BUFFER_STORE_DWORDX4

    Inst_SMEM__S_BUFFER_STORE_DWORDX4::~Inst_SMEM__S_BUFFER_STORE_DWORDX4()
    {
    } // ~Inst_SMEM__S_BUFFER_STORE_DWORDX4

    // --- description from .arch file ---
    // Write 4 dwords to scalar data cache. See S_STORE_DWORD for details on
    // the offset input.
    void
    Inst_SMEM__S_BUFFER_STORE_DWORDX4::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute

    void
    Inst_SMEM__S_BUFFER_STORE_DWORDX4::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
    } // initiateAcc

    void
    Inst_SMEM__S_BUFFER_STORE_DWORDX4::completeAcc(GPUDynInstPtr gpuDynInst)
    {
    } // completeAcc
    // --- Inst_SMEM__S_DCACHE_INV class methods ---

    Inst_SMEM__S_DCACHE_INV::Inst_SMEM__S_DCACHE_INV(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_dcache_inv")
    {
    } // Inst_SMEM__S_DCACHE_INV

    Inst_SMEM__S_DCACHE_INV::~Inst_SMEM__S_DCACHE_INV()
    {
    } // ~Inst_SMEM__S_DCACHE_INV

    // --- description from .arch file ---
    // Invalidate the scalar data cache.
    void
    Inst_SMEM__S_DCACHE_INV::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SMEM__S_DCACHE_WB class methods ---

    Inst_SMEM__S_DCACHE_WB::Inst_SMEM__S_DCACHE_WB(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_dcache_wb")
    {
    } // Inst_SMEM__S_DCACHE_WB

    Inst_SMEM__S_DCACHE_WB::~Inst_SMEM__S_DCACHE_WB()
    {
    } // ~Inst_SMEM__S_DCACHE_WB

    // --- description from .arch file ---
    // Write back dirty data in the scalar data cache.
    void
    Inst_SMEM__S_DCACHE_WB::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SMEM__S_DCACHE_INV_VOL class methods ---

    Inst_SMEM__S_DCACHE_INV_VOL::Inst_SMEM__S_DCACHE_INV_VOL(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_dcache_inv_vol")
    {
    } // Inst_SMEM__S_DCACHE_INV_VOL

    Inst_SMEM__S_DCACHE_INV_VOL::~Inst_SMEM__S_DCACHE_INV_VOL()
    {
    } // ~Inst_SMEM__S_DCACHE_INV_VOL

    // --- description from .arch file ---
    // Invalidate the scalar data cache volatile lines.
    void
    Inst_SMEM__S_DCACHE_INV_VOL::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SMEM__S_DCACHE_WB_VOL class methods ---

    Inst_SMEM__S_DCACHE_WB_VOL::Inst_SMEM__S_DCACHE_WB_VOL(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_dcache_wb_vol")
    {
    } // Inst_SMEM__S_DCACHE_WB_VOL

    Inst_SMEM__S_DCACHE_WB_VOL::~Inst_SMEM__S_DCACHE_WB_VOL()
    {
    } // ~Inst_SMEM__S_DCACHE_WB_VOL

    // --- description from .arch file ---
    // Write back dirty data in the scalar data cache volatile lines.
    void
    Inst_SMEM__S_DCACHE_WB_VOL::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SMEM__S_MEMTIME class methods ---

    Inst_SMEM__S_MEMTIME::Inst_SMEM__S_MEMTIME(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_memtime")
    {
        // s_memtime does not issue a memory request
        setFlag(Memtime);
        setFlag(MemoryRef);
        setFlag(Load);
    } // Inst_SMEM__S_MEMTIME

    Inst_SMEM__S_MEMTIME::~Inst_SMEM__S_MEMTIME()
    {
    } // ~Inst_SMEM__S_MEMTIME

    // --- description from .arch file ---
    // Return current 64-bit timestamp.
    void
    Inst_SMEM__S_MEMTIME::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *wf = gpuDynInst->wavefront();
        gpuDynInst->execUnitId = wf->execUnitId;
        gpuDynInst->latency.init(gpuDynInst->computeUnit());
        gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod()*41);
        ScalarRegU32 offset(0);
        ConstScalarOperandU128 rsrcDesc(gpuDynInst, instData.SBASE);

        rsrcDesc.read();

        if (instData.IMM) {
            offset = extData.OFFSET;
        } else {
            ConstScalarOperandU32 off_sgpr(gpuDynInst, extData.OFFSET);
            off_sgpr.read();
            offset = off_sgpr.rawData();
        }

        calcAddr(gpuDynInst, rsrcDesc, offset);

        gpuDynInst->computeUnit()->scalarMemoryPipe
            .issueRequest(gpuDynInst);
    } // execute

    void Inst_SMEM__S_MEMTIME::initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        initMemRead<2>(gpuDynInst);
    } // initiateAcc

    void
    Inst_SMEM__S_MEMTIME::completeAcc(GPUDynInstPtr gpuDynInst)
    {
        // use U64 because 2 requests, each size 32
        ScalarOperandU64 sdst(gpuDynInst, instData.SDATA);
        sdst.write();
    } // completeAcc
    // --- Inst_SMEM__S_MEMREALTIME class methods ---

    Inst_SMEM__S_MEMREALTIME::Inst_SMEM__S_MEMREALTIME(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_memrealtime")
    {
    } // Inst_SMEM__S_MEMREALTIME

    Inst_SMEM__S_MEMREALTIME::~Inst_SMEM__S_MEMREALTIME()
    {
    } // ~Inst_SMEM__S_MEMREALTIME

    // --- description from .arch file ---
    // Return current 64-bit RTC.
    void
    Inst_SMEM__S_MEMREALTIME::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SMEM__S_ATC_PROBE class methods ---

    Inst_SMEM__S_ATC_PROBE::Inst_SMEM__S_ATC_PROBE(InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_atc_probe")
    {
    } // Inst_SMEM__S_ATC_PROBE

    Inst_SMEM__S_ATC_PROBE::~Inst_SMEM__S_ATC_PROBE()
    {
    } // ~Inst_SMEM__S_ATC_PROBE

    // --- description from .arch file ---
    // Probe or prefetch an address into the SQC data cache.
    void
    Inst_SMEM__S_ATC_PROBE::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
    // --- Inst_SMEM__S_ATC_PROBE_BUFFER class methods ---

    Inst_SMEM__S_ATC_PROBE_BUFFER::Inst_SMEM__S_ATC_PROBE_BUFFER(
          InFmt_SMEM *iFmt)
        : Inst_SMEM(iFmt, "s_atc_probe_buffer")
    {
    } // Inst_SMEM__S_ATC_PROBE_BUFFER

    Inst_SMEM__S_ATC_PROBE_BUFFER::~Inst_SMEM__S_ATC_PROBE_BUFFER()
    {
    } // ~Inst_SMEM__S_ATC_PROBE_BUFFER

    // --- description from .arch file ---
    // Probe or prefetch an address into the SQC data cache.
    void
    Inst_SMEM__S_ATC_PROBE_BUFFER::execute(GPUDynInstPtr gpuDynInst)
    {
        panicUnimplemented();
    } // execute
} // namespace VegaISA
} // namespace gem5
