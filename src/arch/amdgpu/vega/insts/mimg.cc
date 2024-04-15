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
// --- Inst_MIMG__IMAGE_LOAD class methods ---

Inst_MIMG__IMAGE_LOAD::Inst_MIMG__IMAGE_LOAD(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_load")
{
    setFlag(MemoryRef);
    setFlag(Load);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_LOAD

Inst_MIMG__IMAGE_LOAD::~Inst_MIMG__IMAGE_LOAD() {} // ~Inst_MIMG__IMAGE_LOAD

// --- description from .arch file ---
// Image memory load with format conversion specified in T#. No sampler.
void
Inst_MIMG__IMAGE_LOAD::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

void
Inst_MIMG__IMAGE_LOAD::initiateAcc(GPUDynInstPtr gpuDynInst)
{} // initiateAcc

void
Inst_MIMG__IMAGE_LOAD::completeAcc(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_MIMG__IMAGE_LOAD_MIP class methods ---

Inst_MIMG__IMAGE_LOAD_MIP::Inst_MIMG__IMAGE_LOAD_MIP(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_load_mip")
{
    setFlag(MemoryRef);
    setFlag(Load);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_LOAD_MIP

Inst_MIMG__IMAGE_LOAD_MIP::~Inst_MIMG__IMAGE_LOAD_MIP() {
} // ~Inst_MIMG__IMAGE_LOAD_MIP

// --- description from .arch file ---
// Image memory load with user-supplied mip level. No sampler.
void
Inst_MIMG__IMAGE_LOAD_MIP::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

void
Inst_MIMG__IMAGE_LOAD_MIP::initiateAcc(GPUDynInstPtr gpuDynInst)
{} // initiateAcc

void
Inst_MIMG__IMAGE_LOAD_MIP::completeAcc(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_MIMG__IMAGE_LOAD_PCK class methods ---

Inst_MIMG__IMAGE_LOAD_PCK::Inst_MIMG__IMAGE_LOAD_PCK(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_load_pck")
{
    setFlag(MemoryRef);
    setFlag(Load);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_LOAD_PCK

Inst_MIMG__IMAGE_LOAD_PCK::~Inst_MIMG__IMAGE_LOAD_PCK() {
} // ~Inst_MIMG__IMAGE_LOAD_PCK

// --- description from .arch file ---
// Image memory load with no format conversion. No sampler.
void
Inst_MIMG__IMAGE_LOAD_PCK::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

void
Inst_MIMG__IMAGE_LOAD_PCK::initiateAcc(GPUDynInstPtr gpuDynInst)
{} // initiateAcc

void
Inst_MIMG__IMAGE_LOAD_PCK::completeAcc(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_MIMG__IMAGE_LOAD_PCK_SGN class methods ---

Inst_MIMG__IMAGE_LOAD_PCK_SGN::Inst_MIMG__IMAGE_LOAD_PCK_SGN(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_load_pck_sgn")
{
    setFlag(MemoryRef);
    setFlag(Load);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_LOAD_PCK_SGN

Inst_MIMG__IMAGE_LOAD_PCK_SGN::~Inst_MIMG__IMAGE_LOAD_PCK_SGN() {
} // ~Inst_MIMG__IMAGE_LOAD_PCK_SGN

// --- description from .arch file ---
// Image memory load with with no format conversion and sign extension. No
// ---  sampler.
void
Inst_MIMG__IMAGE_LOAD_PCK_SGN::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

void
Inst_MIMG__IMAGE_LOAD_PCK_SGN::initiateAcc(GPUDynInstPtr gpuDynInst)
{} // initiateAcc

void
Inst_MIMG__IMAGE_LOAD_PCK_SGN::completeAcc(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_MIMG__IMAGE_LOAD_MIP_PCK class methods ---

Inst_MIMG__IMAGE_LOAD_MIP_PCK::Inst_MIMG__IMAGE_LOAD_MIP_PCK(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_load_mip_pck")
{
    setFlag(MemoryRef);
    setFlag(Load);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_LOAD_MIP_PCK

Inst_MIMG__IMAGE_LOAD_MIP_PCK::~Inst_MIMG__IMAGE_LOAD_MIP_PCK() {
} // ~Inst_MIMG__IMAGE_LOAD_MIP_PCK

// --- description from .arch file ---
// Image memory load with user-supplied mip level, no format conversion. No
// ---  sampler.
void
Inst_MIMG__IMAGE_LOAD_MIP_PCK::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

void
Inst_MIMG__IMAGE_LOAD_MIP_PCK::initiateAcc(GPUDynInstPtr gpuDynInst)
{} // initiateAcc

void
Inst_MIMG__IMAGE_LOAD_MIP_PCK::completeAcc(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN class methods ---

Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN::Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_load_mip_pck_sgn")
{
    setFlag(MemoryRef);
    setFlag(Load);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN

Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN::~Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN() {
} // ~Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN

// --- description from .arch file ---
// Image memory load with user-supplied mip level, no format conversion and
// ---  with sign extension. No sampler.
void
Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

void
Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN::initiateAcc(GPUDynInstPtr gpuDynInst)
{} // initiateAcc

void
Inst_MIMG__IMAGE_LOAD_MIP_PCK_SGN::completeAcc(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_MIMG__IMAGE_STORE class methods ---

Inst_MIMG__IMAGE_STORE::Inst_MIMG__IMAGE_STORE(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_store")
{
    setFlag(MemoryRef);
    setFlag(Store);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_STORE

Inst_MIMG__IMAGE_STORE::~Inst_MIMG__IMAGE_STORE() {} // ~Inst_MIMG__IMAGE_STORE

// --- description from .arch file ---
// Image memory store with format conversion specified in T#. No sampler.
void
Inst_MIMG__IMAGE_STORE::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

void
Inst_MIMG__IMAGE_STORE::initiateAcc(GPUDynInstPtr gpuDynInst)
{} // initiateAcc

void
Inst_MIMG__IMAGE_STORE::completeAcc(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_MIMG__IMAGE_STORE_MIP class methods ---

Inst_MIMG__IMAGE_STORE_MIP::Inst_MIMG__IMAGE_STORE_MIP(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_store_mip")
{
    setFlag(MemoryRef);
    setFlag(Store);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_STORE_MIP

Inst_MIMG__IMAGE_STORE_MIP::~Inst_MIMG__IMAGE_STORE_MIP() {
} // ~Inst_MIMG__IMAGE_STORE_MIP

// --- description from .arch file ---
// Image memory store with format conversion specified in T# to user
// specified mip level. No sampler.
void
Inst_MIMG__IMAGE_STORE_MIP::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

void
Inst_MIMG__IMAGE_STORE_MIP::initiateAcc(GPUDynInstPtr gpuDynInst)
{} // initiateAcc

void
Inst_MIMG__IMAGE_STORE_MIP::completeAcc(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_MIMG__IMAGE_STORE_PCK class methods ---

Inst_MIMG__IMAGE_STORE_PCK::Inst_MIMG__IMAGE_STORE_PCK(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_store_pck")
{
    setFlag(MemoryRef);
    setFlag(Store);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_STORE_PCK

Inst_MIMG__IMAGE_STORE_PCK::~Inst_MIMG__IMAGE_STORE_PCK() {
} // ~Inst_MIMG__IMAGE_STORE_PCK

// --- description from .arch file ---
// Image memory store of packed data without format conversion. No sampler.
void
Inst_MIMG__IMAGE_STORE_PCK::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

void
Inst_MIMG__IMAGE_STORE_PCK::initiateAcc(GPUDynInstPtr gpuDynInst)
{} // initiateAcc

void
Inst_MIMG__IMAGE_STORE_PCK::completeAcc(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_MIMG__IMAGE_STORE_MIP_PCK class methods ---

Inst_MIMG__IMAGE_STORE_MIP_PCK::Inst_MIMG__IMAGE_STORE_MIP_PCK(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_store_mip_pck")
{
    setFlag(MemoryRef);
    setFlag(Store);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_STORE_MIP_PCK

Inst_MIMG__IMAGE_STORE_MIP_PCK::~Inst_MIMG__IMAGE_STORE_MIP_PCK() {
} // ~Inst_MIMG__IMAGE_STORE_MIP_PCK

// --- description from .arch file ---
// Image memory store of packed data without format conversion to
// user-supplied mip level. No sampler.
void
Inst_MIMG__IMAGE_STORE_MIP_PCK::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

void
Inst_MIMG__IMAGE_STORE_MIP_PCK::initiateAcc(GPUDynInstPtr gpuDynInst)
{} // initiateAcc

void
Inst_MIMG__IMAGE_STORE_MIP_PCK::completeAcc(GPUDynInstPtr gpuDynInst)
{} // execute

// --- Inst_MIMG__IMAGE_GET_RESINFO class methods ---

Inst_MIMG__IMAGE_GET_RESINFO::Inst_MIMG__IMAGE_GET_RESINFO(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_get_resinfo")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GET_RESINFO

Inst_MIMG__IMAGE_GET_RESINFO::~Inst_MIMG__IMAGE_GET_RESINFO() {
} // ~Inst_MIMG__IMAGE_GET_RESINFO

// --- description from .arch file ---
// return resource info for a given mip level specified in the address
// vgpr. No sampler. Returns 4 integer values into VGPRs 3-0:
// {num_mip_levels, depth, height, width}.
void
Inst_MIMG__IMAGE_GET_RESINFO::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_SWAP class methods ---

Inst_MIMG__IMAGE_ATOMIC_SWAP::Inst_MIMG__IMAGE_ATOMIC_SWAP(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_swap")
{
    setFlag(AtomicExch);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_SWAP

Inst_MIMG__IMAGE_ATOMIC_SWAP::~Inst_MIMG__IMAGE_ATOMIC_SWAP() {
} // ~Inst_MIMG__IMAGE_ATOMIC_SWAP

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] = DATA;
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_SWAP::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_CMPSWAP class methods ---

Inst_MIMG__IMAGE_ATOMIC_CMPSWAP::Inst_MIMG__IMAGE_ATOMIC_CMPSWAP(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_cmpswap")
{
    setFlag(AtomicCAS);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_CMPSWAP

Inst_MIMG__IMAGE_ATOMIC_CMPSWAP::~Inst_MIMG__IMAGE_ATOMIC_CMPSWAP() {
} // ~Inst_MIMG__IMAGE_ATOMIC_CMPSWAP

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// src = DATA[0];
// cmp = DATA[1];
// MEM[ADDR] = (tmp == cmp) ? src : tmp;
// RETURN_DATA[0] = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_CMPSWAP::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_ADD class methods ---

Inst_MIMG__IMAGE_ATOMIC_ADD::Inst_MIMG__IMAGE_ATOMIC_ADD(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_add")
{
    setFlag(AtomicAdd);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_ADD

Inst_MIMG__IMAGE_ATOMIC_ADD::~Inst_MIMG__IMAGE_ATOMIC_ADD() {
} // ~Inst_MIMG__IMAGE_ATOMIC_ADD

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] += DATA;
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_ADD::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_SUB class methods ---

Inst_MIMG__IMAGE_ATOMIC_SUB::Inst_MIMG__IMAGE_ATOMIC_SUB(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_sub")
{
    setFlag(AtomicSub);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_SUB

Inst_MIMG__IMAGE_ATOMIC_SUB::~Inst_MIMG__IMAGE_ATOMIC_SUB() {
} // ~Inst_MIMG__IMAGE_ATOMIC_SUB

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] -= DATA;
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_SUB::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_SMIN class methods ---

Inst_MIMG__IMAGE_ATOMIC_SMIN::Inst_MIMG__IMAGE_ATOMIC_SMIN(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_smin")
{
    setFlag(AtomicMin);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_SMIN

Inst_MIMG__IMAGE_ATOMIC_SMIN::~Inst_MIMG__IMAGE_ATOMIC_SMIN() {
} // ~Inst_MIMG__IMAGE_ATOMIC_SMIN

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] = (DATA < tmp) ? DATA : tmp (signed compare);
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_SMIN::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_UMIN class methods ---

Inst_MIMG__IMAGE_ATOMIC_UMIN::Inst_MIMG__IMAGE_ATOMIC_UMIN(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_umin")
{
    setFlag(AtomicMin);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_UMIN

Inst_MIMG__IMAGE_ATOMIC_UMIN::~Inst_MIMG__IMAGE_ATOMIC_UMIN() {
} // ~Inst_MIMG__IMAGE_ATOMIC_UMIN

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] = (DATA < tmp) ? DATA : tmp (unsigned compare);
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_UMIN::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_SMAX class methods ---

Inst_MIMG__IMAGE_ATOMIC_SMAX::Inst_MIMG__IMAGE_ATOMIC_SMAX(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_smax")
{
    setFlag(AtomicMax);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_SMAX

Inst_MIMG__IMAGE_ATOMIC_SMAX::~Inst_MIMG__IMAGE_ATOMIC_SMAX() {
} // ~Inst_MIMG__IMAGE_ATOMIC_SMAX

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] = (DATA > tmp) ? DATA : tmp (signed compare);
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_SMAX::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_UMAX class methods ---

Inst_MIMG__IMAGE_ATOMIC_UMAX::Inst_MIMG__IMAGE_ATOMIC_UMAX(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_umax")
{
    setFlag(AtomicMax);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_UMAX

Inst_MIMG__IMAGE_ATOMIC_UMAX::~Inst_MIMG__IMAGE_ATOMIC_UMAX() {
} // ~Inst_MIMG__IMAGE_ATOMIC_UMAX

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] = (DATA > tmp) ? DATA : tmp (unsigned compare);
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_UMAX::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_AND class methods ---

Inst_MIMG__IMAGE_ATOMIC_AND::Inst_MIMG__IMAGE_ATOMIC_AND(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_and")
{
    setFlag(AtomicAnd);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_AND

Inst_MIMG__IMAGE_ATOMIC_AND::~Inst_MIMG__IMAGE_ATOMIC_AND() {
} // ~Inst_MIMG__IMAGE_ATOMIC_AND

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] &= DATA;
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_AND::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_OR class methods ---

Inst_MIMG__IMAGE_ATOMIC_OR::Inst_MIMG__IMAGE_ATOMIC_OR(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_or")
{
    setFlag(AtomicOr);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_OR

Inst_MIMG__IMAGE_ATOMIC_OR::~Inst_MIMG__IMAGE_ATOMIC_OR() {
} // ~Inst_MIMG__IMAGE_ATOMIC_OR

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] |= DATA;
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_OR::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_XOR class methods ---

Inst_MIMG__IMAGE_ATOMIC_XOR::Inst_MIMG__IMAGE_ATOMIC_XOR(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_xor")
{
    setFlag(AtomicXor);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_XOR

Inst_MIMG__IMAGE_ATOMIC_XOR::~Inst_MIMG__IMAGE_ATOMIC_XOR() {
} // ~Inst_MIMG__IMAGE_ATOMIC_XOR

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] ^= DATA;
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_XOR::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_INC class methods ---

Inst_MIMG__IMAGE_ATOMIC_INC::Inst_MIMG__IMAGE_ATOMIC_INC(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_inc")
{
    setFlag(AtomicInc);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_INC

Inst_MIMG__IMAGE_ATOMIC_INC::~Inst_MIMG__IMAGE_ATOMIC_INC() {
} // ~Inst_MIMG__IMAGE_ATOMIC_INC

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] = (tmp >= DATA) ? 0 : tmp + 1 (unsigned compare);
// RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_INC::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_ATOMIC_DEC class methods ---

Inst_MIMG__IMAGE_ATOMIC_DEC::Inst_MIMG__IMAGE_ATOMIC_DEC(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_atomic_dec")
{
    setFlag(AtomicDec);
    if (instData.GLC) {
        setFlag(AtomicReturn);
    } else {
        setFlag(AtomicNoReturn);
    }
    setFlag(MemoryRef);
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_ATOMIC_DEC

Inst_MIMG__IMAGE_ATOMIC_DEC::~Inst_MIMG__IMAGE_ATOMIC_DEC() {
} // ~Inst_MIMG__IMAGE_ATOMIC_DEC

// --- description from .arch file ---
// 32b:
// tmp = MEM[ADDR];
// MEM[ADDR] = (tmp == 0 || tmp > DATA) ? DATA : tmp - 1
// (unsigned compare); RETURN_DATA = tmp.
void
Inst_MIMG__IMAGE_ATOMIC_DEC::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE class methods ---

Inst_MIMG__IMAGE_SAMPLE::Inst_MIMG__IMAGE_SAMPLE(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample")
{} // Inst_MIMG__IMAGE_SAMPLE

Inst_MIMG__IMAGE_SAMPLE::~Inst_MIMG__IMAGE_SAMPLE() {
} // ~Inst_MIMG__IMAGE_SAMPLE

// --- description from .arch file ---
// sample texture map.
void
Inst_MIMG__IMAGE_SAMPLE::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_CL class methods ---

Inst_MIMG__IMAGE_SAMPLE_CL::Inst_MIMG__IMAGE_SAMPLE_CL(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_CL

Inst_MIMG__IMAGE_SAMPLE_CL::~Inst_MIMG__IMAGE_SAMPLE_CL() {
} // ~Inst_MIMG__IMAGE_SAMPLE_CL

// --- description from .arch file ---
// sample texture map, with LOD clamp specified in shader.
void
Inst_MIMG__IMAGE_SAMPLE_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_D class methods ---

Inst_MIMG__IMAGE_SAMPLE_D::Inst_MIMG__IMAGE_SAMPLE_D(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_d")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_D

Inst_MIMG__IMAGE_SAMPLE_D::~Inst_MIMG__IMAGE_SAMPLE_D() {
} // ~Inst_MIMG__IMAGE_SAMPLE_D

// --- description from .arch file ---
// sample texture map, with user derivatives
void
Inst_MIMG__IMAGE_SAMPLE_D::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_D_CL class methods ---

Inst_MIMG__IMAGE_SAMPLE_D_CL::Inst_MIMG__IMAGE_SAMPLE_D_CL(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_d_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_D_CL

Inst_MIMG__IMAGE_SAMPLE_D_CL::~Inst_MIMG__IMAGE_SAMPLE_D_CL() {
} // ~Inst_MIMG__IMAGE_SAMPLE_D_CL

// --- description from .arch file ---
// sample texture map, with LOD clamp specified in shader, with user
// ---  derivatives.
void
Inst_MIMG__IMAGE_SAMPLE_D_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_L class methods ---

Inst_MIMG__IMAGE_SAMPLE_L::Inst_MIMG__IMAGE_SAMPLE_L(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_l")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_L

Inst_MIMG__IMAGE_SAMPLE_L::~Inst_MIMG__IMAGE_SAMPLE_L() {
} // ~Inst_MIMG__IMAGE_SAMPLE_L

// --- description from .arch file ---
// sample texture map, with user LOD.
void
Inst_MIMG__IMAGE_SAMPLE_L::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_B class methods ---

Inst_MIMG__IMAGE_SAMPLE_B::Inst_MIMG__IMAGE_SAMPLE_B(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_b")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_B

Inst_MIMG__IMAGE_SAMPLE_B::~Inst_MIMG__IMAGE_SAMPLE_B() {
} // ~Inst_MIMG__IMAGE_SAMPLE_B

// --- description from .arch file ---
// sample texture map, with lod bias.
void
Inst_MIMG__IMAGE_SAMPLE_B::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_B_CL class methods ---

Inst_MIMG__IMAGE_SAMPLE_B_CL::Inst_MIMG__IMAGE_SAMPLE_B_CL(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_b_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_B_CL

Inst_MIMG__IMAGE_SAMPLE_B_CL::~Inst_MIMG__IMAGE_SAMPLE_B_CL() {
} // ~Inst_MIMG__IMAGE_SAMPLE_B_CL

// --- description from .arch file ---
// sample texture map, with LOD clamp specified in shader, with lod bias.
void
Inst_MIMG__IMAGE_SAMPLE_B_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_LZ class methods ---

Inst_MIMG__IMAGE_SAMPLE_LZ::Inst_MIMG__IMAGE_SAMPLE_LZ(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_lz")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_LZ

Inst_MIMG__IMAGE_SAMPLE_LZ::~Inst_MIMG__IMAGE_SAMPLE_LZ() {
} // ~Inst_MIMG__IMAGE_SAMPLE_LZ

// --- description from .arch file ---
// sample texture map, from level 0.
void
Inst_MIMG__IMAGE_SAMPLE_LZ::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C class methods ---

Inst_MIMG__IMAGE_SAMPLE_C::Inst_MIMG__IMAGE_SAMPLE_C(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C

Inst_MIMG__IMAGE_SAMPLE_C::~Inst_MIMG__IMAGE_SAMPLE_C() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C

// --- description from .arch file ---
// sample texture map, with PCF.
void
Inst_MIMG__IMAGE_SAMPLE_C::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_CL class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_CL::Inst_MIMG__IMAGE_SAMPLE_C_CL(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_CL

Inst_MIMG__IMAGE_SAMPLE_C_CL::~Inst_MIMG__IMAGE_SAMPLE_C_CL() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_CL

// --- description from .arch file ---
// SAMPLE_C, with LOD clamp specified in shader.
void
Inst_MIMG__IMAGE_SAMPLE_C_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_D class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_D::Inst_MIMG__IMAGE_SAMPLE_C_D(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_d")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_D

Inst_MIMG__IMAGE_SAMPLE_C_D::~Inst_MIMG__IMAGE_SAMPLE_C_D() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_D

// --- description from .arch file ---
// SAMPLE_C, with user derivatives.
void
Inst_MIMG__IMAGE_SAMPLE_C_D::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_D_CL class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_D_CL::Inst_MIMG__IMAGE_SAMPLE_C_D_CL(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_d_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_D_CL

Inst_MIMG__IMAGE_SAMPLE_C_D_CL::~Inst_MIMG__IMAGE_SAMPLE_C_D_CL() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_D_CL

// --- description from .arch file ---
// SAMPLE_C, with LOD clamp specified in shader, with user derivatives.
void
Inst_MIMG__IMAGE_SAMPLE_C_D_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_L class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_L::Inst_MIMG__IMAGE_SAMPLE_C_L(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_l")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_L

Inst_MIMG__IMAGE_SAMPLE_C_L::~Inst_MIMG__IMAGE_SAMPLE_C_L() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_L

// --- description from .arch file ---
// SAMPLE_C, with user LOD.
void
Inst_MIMG__IMAGE_SAMPLE_C_L::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_B class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_B::Inst_MIMG__IMAGE_SAMPLE_C_B(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_b")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_B

Inst_MIMG__IMAGE_SAMPLE_C_B::~Inst_MIMG__IMAGE_SAMPLE_C_B() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_B

// --- description from .arch file ---
// SAMPLE_C, with lod bias.
void
Inst_MIMG__IMAGE_SAMPLE_C_B::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_B_CL class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_B_CL::Inst_MIMG__IMAGE_SAMPLE_C_B_CL(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_b_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_B_CL

Inst_MIMG__IMAGE_SAMPLE_C_B_CL::~Inst_MIMG__IMAGE_SAMPLE_C_B_CL() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_B_CL

// --- description from .arch file ---
// SAMPLE_C, with LOD clamp specified in shader, with lod bias.
void
Inst_MIMG__IMAGE_SAMPLE_C_B_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_LZ class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_LZ::Inst_MIMG__IMAGE_SAMPLE_C_LZ(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_lz")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_LZ

Inst_MIMG__IMAGE_SAMPLE_C_LZ::~Inst_MIMG__IMAGE_SAMPLE_C_LZ() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_LZ

// --- description from .arch file ---
// SAMPLE_C, from level 0.
void
Inst_MIMG__IMAGE_SAMPLE_C_LZ::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_O::Inst_MIMG__IMAGE_SAMPLE_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_O

Inst_MIMG__IMAGE_SAMPLE_O::~Inst_MIMG__IMAGE_SAMPLE_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_O

// --- description from .arch file ---
// sample texture map, with user offsets.
void
Inst_MIMG__IMAGE_SAMPLE_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_CL_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_CL_O::Inst_MIMG__IMAGE_SAMPLE_CL_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_CL_O

Inst_MIMG__IMAGE_SAMPLE_CL_O::~Inst_MIMG__IMAGE_SAMPLE_CL_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_CL_O

// --- description from .arch file ---
// SAMPLE_O with LOD clamp specified in shader.
void
Inst_MIMG__IMAGE_SAMPLE_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_D_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_D_O::Inst_MIMG__IMAGE_SAMPLE_D_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_d_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_D_O

Inst_MIMG__IMAGE_SAMPLE_D_O::~Inst_MIMG__IMAGE_SAMPLE_D_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_D_O

// --- description from .arch file ---
// SAMPLE_O, with user derivatives.
void
Inst_MIMG__IMAGE_SAMPLE_D_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_D_CL_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_D_CL_O::Inst_MIMG__IMAGE_SAMPLE_D_CL_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_d_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_D_CL_O

Inst_MIMG__IMAGE_SAMPLE_D_CL_O::~Inst_MIMG__IMAGE_SAMPLE_D_CL_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_D_CL_O

// --- description from .arch file ---
// SAMPLE_O, with LOD clamp specified in shader, with user derivatives.
void
Inst_MIMG__IMAGE_SAMPLE_D_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_L_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_L_O::Inst_MIMG__IMAGE_SAMPLE_L_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_l_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_L_O

Inst_MIMG__IMAGE_SAMPLE_L_O::~Inst_MIMG__IMAGE_SAMPLE_L_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_L_O

// --- description from .arch file ---
// SAMPLE_O, with user LOD.
void
Inst_MIMG__IMAGE_SAMPLE_L_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_B_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_B_O::Inst_MIMG__IMAGE_SAMPLE_B_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_b_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_B_O

Inst_MIMG__IMAGE_SAMPLE_B_O::~Inst_MIMG__IMAGE_SAMPLE_B_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_B_O

// --- description from .arch file ---
// SAMPLE_O, with lod bias.
void
Inst_MIMG__IMAGE_SAMPLE_B_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_B_CL_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_B_CL_O::Inst_MIMG__IMAGE_SAMPLE_B_CL_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_b_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_B_CL_O

Inst_MIMG__IMAGE_SAMPLE_B_CL_O::~Inst_MIMG__IMAGE_SAMPLE_B_CL_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_B_CL_O

// --- description from .arch file ---
// SAMPLE_O, with LOD clamp specified in shader, with lod bias.
void
Inst_MIMG__IMAGE_SAMPLE_B_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_LZ_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_LZ_O::Inst_MIMG__IMAGE_SAMPLE_LZ_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_lz_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_LZ_O

Inst_MIMG__IMAGE_SAMPLE_LZ_O::~Inst_MIMG__IMAGE_SAMPLE_LZ_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_LZ_O

// --- description from .arch file ---
// SAMPLE_O, from level 0.
void
Inst_MIMG__IMAGE_SAMPLE_LZ_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_O::Inst_MIMG__IMAGE_SAMPLE_C_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_O

Inst_MIMG__IMAGE_SAMPLE_C_O::~Inst_MIMG__IMAGE_SAMPLE_C_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_O

// --- description from .arch file ---
// SAMPLE_C with user specified offsets.
void
Inst_MIMG__IMAGE_SAMPLE_C_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_CL_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_CL_O::Inst_MIMG__IMAGE_SAMPLE_C_CL_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_CL_O

Inst_MIMG__IMAGE_SAMPLE_C_CL_O::~Inst_MIMG__IMAGE_SAMPLE_C_CL_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_CL_O

// --- description from .arch file ---
// SAMPLE_C_O, with LOD clamp specified in shader.
void
Inst_MIMG__IMAGE_SAMPLE_C_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_D_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_D_O::Inst_MIMG__IMAGE_SAMPLE_C_D_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_d_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_D_O

Inst_MIMG__IMAGE_SAMPLE_C_D_O::~Inst_MIMG__IMAGE_SAMPLE_C_D_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_D_O

// --- description from .arch file ---
// SAMPLE_C_O, with user derivatives.
void
Inst_MIMG__IMAGE_SAMPLE_C_D_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_D_CL_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_D_CL_O::Inst_MIMG__IMAGE_SAMPLE_C_D_CL_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_d_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_D_CL_O

Inst_MIMG__IMAGE_SAMPLE_C_D_CL_O::~Inst_MIMG__IMAGE_SAMPLE_C_D_CL_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_D_CL_O

// --- description from .arch file ---
// SAMPLE_C_O, with LOD clamp specified in shader, with user derivatives.
void
Inst_MIMG__IMAGE_SAMPLE_C_D_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_L_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_L_O::Inst_MIMG__IMAGE_SAMPLE_C_L_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_l_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_L_O

Inst_MIMG__IMAGE_SAMPLE_C_L_O::~Inst_MIMG__IMAGE_SAMPLE_C_L_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_L_O

// --- description from .arch file ---
// SAMPLE_C_O, with user LOD.
void
Inst_MIMG__IMAGE_SAMPLE_C_L_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_B_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_B_O::Inst_MIMG__IMAGE_SAMPLE_C_B_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_b_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_B_O

Inst_MIMG__IMAGE_SAMPLE_C_B_O::~Inst_MIMG__IMAGE_SAMPLE_C_B_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_B_O

// --- description from .arch file ---
// SAMPLE_C_O, with lod bias.
void
Inst_MIMG__IMAGE_SAMPLE_C_B_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_B_CL_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_B_CL_O::Inst_MIMG__IMAGE_SAMPLE_C_B_CL_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_b_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_B_CL_O

Inst_MIMG__IMAGE_SAMPLE_C_B_CL_O::~Inst_MIMG__IMAGE_SAMPLE_C_B_CL_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_B_CL_O

// --- description from .arch file ---
// SAMPLE_C_O, with LOD clamp specified in shader, with lod bias.
void
Inst_MIMG__IMAGE_SAMPLE_C_B_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_LZ_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_LZ_O::Inst_MIMG__IMAGE_SAMPLE_C_LZ_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_lz_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_LZ_O

Inst_MIMG__IMAGE_SAMPLE_C_LZ_O::~Inst_MIMG__IMAGE_SAMPLE_C_LZ_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_LZ_O

// --- description from .arch file ---
// SAMPLE_C_O, from level 0.
void
Inst_MIMG__IMAGE_SAMPLE_C_LZ_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4 class methods ---

Inst_MIMG__IMAGE_GATHER4::Inst_MIMG__IMAGE_GATHER4(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4

Inst_MIMG__IMAGE_GATHER4::~Inst_MIMG__IMAGE_GATHER4() {
} // ~Inst_MIMG__IMAGE_GATHER4

// --- description from .arch file ---
// gather 4 single component elements (2x2).
void
Inst_MIMG__IMAGE_GATHER4::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_CL class methods ---

Inst_MIMG__IMAGE_GATHER4_CL::Inst_MIMG__IMAGE_GATHER4_CL(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_CL

Inst_MIMG__IMAGE_GATHER4_CL::~Inst_MIMG__IMAGE_GATHER4_CL() {
} // ~Inst_MIMG__IMAGE_GATHER4_CL

// --- description from .arch file ---
// gather 4 single component elements (2x2) with user LOD clamp.
void
Inst_MIMG__IMAGE_GATHER4_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_L class methods ---

Inst_MIMG__IMAGE_GATHER4_L::Inst_MIMG__IMAGE_GATHER4_L(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_l")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_L

Inst_MIMG__IMAGE_GATHER4_L::~Inst_MIMG__IMAGE_GATHER4_L() {
} // ~Inst_MIMG__IMAGE_GATHER4_L

// --- description from .arch file ---
// gather 4 single component elements (2x2) with user LOD.
void
Inst_MIMG__IMAGE_GATHER4_L::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_B class methods ---

Inst_MIMG__IMAGE_GATHER4_B::Inst_MIMG__IMAGE_GATHER4_B(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_b")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_B

Inst_MIMG__IMAGE_GATHER4_B::~Inst_MIMG__IMAGE_GATHER4_B() {
} // ~Inst_MIMG__IMAGE_GATHER4_B

// --- description from .arch file ---
// gather 4 single component elements (2x2) with user bias.
void
Inst_MIMG__IMAGE_GATHER4_B::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_B_CL class methods ---

Inst_MIMG__IMAGE_GATHER4_B_CL::Inst_MIMG__IMAGE_GATHER4_B_CL(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_b_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_B_CL

Inst_MIMG__IMAGE_GATHER4_B_CL::~Inst_MIMG__IMAGE_GATHER4_B_CL() {
} // ~Inst_MIMG__IMAGE_GATHER4_B_CL

// --- description from .arch file ---
// gather 4 single component elements (2x2) with user bias and clamp.
void
Inst_MIMG__IMAGE_GATHER4_B_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_LZ class methods ---

Inst_MIMG__IMAGE_GATHER4_LZ::Inst_MIMG__IMAGE_GATHER4_LZ(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_lz")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_LZ

Inst_MIMG__IMAGE_GATHER4_LZ::~Inst_MIMG__IMAGE_GATHER4_LZ() {
} // ~Inst_MIMG__IMAGE_GATHER4_LZ

// --- description from .arch file ---
// gather 4 single component elements (2x2) at level 0.
void
Inst_MIMG__IMAGE_GATHER4_LZ::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C class methods ---

Inst_MIMG__IMAGE_GATHER4_C::Inst_MIMG__IMAGE_GATHER4_C(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C

Inst_MIMG__IMAGE_GATHER4_C::~Inst_MIMG__IMAGE_GATHER4_C() {
} // ~Inst_MIMG__IMAGE_GATHER4_C

// --- description from .arch file ---
// gather 4 single component elements (2x2) with PCF.
void
Inst_MIMG__IMAGE_GATHER4_C::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_CL class methods ---

Inst_MIMG__IMAGE_GATHER4_C_CL::Inst_MIMG__IMAGE_GATHER4_C_CL(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_CL

Inst_MIMG__IMAGE_GATHER4_C_CL::~Inst_MIMG__IMAGE_GATHER4_C_CL() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_CL

// --- description from .arch file ---
// gather 4 single component elements (2x2) with user LOD clamp and PCF.
void
Inst_MIMG__IMAGE_GATHER4_C_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_L class methods ---

Inst_MIMG__IMAGE_GATHER4_C_L::Inst_MIMG__IMAGE_GATHER4_C_L(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_l")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_L

Inst_MIMG__IMAGE_GATHER4_C_L::~Inst_MIMG__IMAGE_GATHER4_C_L() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_L

// --- description from .arch file ---
// gather 4 single component elements (2x2) with user LOD and PCF.
void
Inst_MIMG__IMAGE_GATHER4_C_L::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_B class methods ---

Inst_MIMG__IMAGE_GATHER4_C_B::Inst_MIMG__IMAGE_GATHER4_C_B(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_b")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_B

Inst_MIMG__IMAGE_GATHER4_C_B::~Inst_MIMG__IMAGE_GATHER4_C_B() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_B

// --- description from .arch file ---
// gather 4 single component elements (2x2) with user bias and PCF.
void
Inst_MIMG__IMAGE_GATHER4_C_B::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_B_CL class methods ---

Inst_MIMG__IMAGE_GATHER4_C_B_CL::Inst_MIMG__IMAGE_GATHER4_C_B_CL(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_b_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_B_CL

Inst_MIMG__IMAGE_GATHER4_C_B_CL::~Inst_MIMG__IMAGE_GATHER4_C_B_CL() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_B_CL

// --- description from .arch file ---
// gather 4 single component elements (2x2) with user bias, clamp and PCF.
void
Inst_MIMG__IMAGE_GATHER4_C_B_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_LZ class methods ---

Inst_MIMG__IMAGE_GATHER4_C_LZ::Inst_MIMG__IMAGE_GATHER4_C_LZ(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_lz")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_LZ

Inst_MIMG__IMAGE_GATHER4_C_LZ::~Inst_MIMG__IMAGE_GATHER4_C_LZ() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_LZ

// --- description from .arch file ---
// gather 4 single component elements (2x2) at level 0, with PCF.
void
Inst_MIMG__IMAGE_GATHER4_C_LZ::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_O class methods ---

Inst_MIMG__IMAGE_GATHER4_O::Inst_MIMG__IMAGE_GATHER4_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_O

Inst_MIMG__IMAGE_GATHER4_O::~Inst_MIMG__IMAGE_GATHER4_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_O

// --- description from .arch file ---
// GATHER4, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_CL_O class methods ---

Inst_MIMG__IMAGE_GATHER4_CL_O::Inst_MIMG__IMAGE_GATHER4_CL_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_CL_O

Inst_MIMG__IMAGE_GATHER4_CL_O::~Inst_MIMG__IMAGE_GATHER4_CL_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_CL_O

// --- description from .arch file ---
// GATHER4_CL, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_L_O class methods ---

Inst_MIMG__IMAGE_GATHER4_L_O::Inst_MIMG__IMAGE_GATHER4_L_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_l_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_L_O

Inst_MIMG__IMAGE_GATHER4_L_O::~Inst_MIMG__IMAGE_GATHER4_L_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_L_O

// --- description from .arch file ---
// GATHER4_L, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_L_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_B_O class methods ---

Inst_MIMG__IMAGE_GATHER4_B_O::Inst_MIMG__IMAGE_GATHER4_B_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_b_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_B_O

Inst_MIMG__IMAGE_GATHER4_B_O::~Inst_MIMG__IMAGE_GATHER4_B_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_B_O

// --- description from .arch file ---
// GATHER4_B, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_B_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_B_CL_O class methods ---

Inst_MIMG__IMAGE_GATHER4_B_CL_O::Inst_MIMG__IMAGE_GATHER4_B_CL_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_b_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_B_CL_O

Inst_MIMG__IMAGE_GATHER4_B_CL_O::~Inst_MIMG__IMAGE_GATHER4_B_CL_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_B_CL_O

// --- description from .arch file ---
// GATHER4_B_CL, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_B_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_LZ_O class methods ---

Inst_MIMG__IMAGE_GATHER4_LZ_O::Inst_MIMG__IMAGE_GATHER4_LZ_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_lz_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_LZ_O

Inst_MIMG__IMAGE_GATHER4_LZ_O::~Inst_MIMG__IMAGE_GATHER4_LZ_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_LZ_O

// --- description from .arch file ---
// GATHER4_LZ, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_LZ_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_O class methods ---

Inst_MIMG__IMAGE_GATHER4_C_O::Inst_MIMG__IMAGE_GATHER4_C_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_O

Inst_MIMG__IMAGE_GATHER4_C_O::~Inst_MIMG__IMAGE_GATHER4_C_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_O

// --- description from .arch file ---
// GATHER4_C, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_C_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_CL_O class methods ---

Inst_MIMG__IMAGE_GATHER4_C_CL_O::Inst_MIMG__IMAGE_GATHER4_C_CL_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_CL_O

Inst_MIMG__IMAGE_GATHER4_C_CL_O::~Inst_MIMG__IMAGE_GATHER4_C_CL_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_CL_O

// --- description from .arch file ---
// GATHER4_C_CL, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_C_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_L_O class methods ---

Inst_MIMG__IMAGE_GATHER4_C_L_O::Inst_MIMG__IMAGE_GATHER4_C_L_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_l_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_L_O

Inst_MIMG__IMAGE_GATHER4_C_L_O::~Inst_MIMG__IMAGE_GATHER4_C_L_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_L_O

// --- description from .arch file ---
// GATHER4_C_L, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_C_L_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_B_O class methods ---

Inst_MIMG__IMAGE_GATHER4_C_B_O::Inst_MIMG__IMAGE_GATHER4_C_B_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_b_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_B_O

Inst_MIMG__IMAGE_GATHER4_C_B_O::~Inst_MIMG__IMAGE_GATHER4_C_B_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_B_O

// --- description from .arch file ---
// GATHER4_B, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_C_B_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_B_CL_O class methods ---

Inst_MIMG__IMAGE_GATHER4_C_B_CL_O::Inst_MIMG__IMAGE_GATHER4_C_B_CL_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_b_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_B_CL_O

Inst_MIMG__IMAGE_GATHER4_C_B_CL_O::~Inst_MIMG__IMAGE_GATHER4_C_B_CL_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_B_CL_O

// --- description from .arch file ---
// GATHER4_B_CL, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_C_B_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GATHER4_C_LZ_O class methods ---

Inst_MIMG__IMAGE_GATHER4_C_LZ_O::Inst_MIMG__IMAGE_GATHER4_C_LZ_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_gather4_c_lz_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GATHER4_C_LZ_O

Inst_MIMG__IMAGE_GATHER4_C_LZ_O::~Inst_MIMG__IMAGE_GATHER4_C_LZ_O() {
} // ~Inst_MIMG__IMAGE_GATHER4_C_LZ_O

// --- description from .arch file ---
// GATHER4_C_LZ, with user offsets.
void
Inst_MIMG__IMAGE_GATHER4_C_LZ_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_GET_LOD class methods ---

Inst_MIMG__IMAGE_GET_LOD::Inst_MIMG__IMAGE_GET_LOD(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_get_lod")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_GET_LOD

Inst_MIMG__IMAGE_GET_LOD::~Inst_MIMG__IMAGE_GET_LOD() {
} // ~Inst_MIMG__IMAGE_GET_LOD

// --- description from .arch file ---
// Return calculated LOD. Vdata gets 2 32bit integer values: { rawLOD,
// ---  clampedLOD }.
void
Inst_MIMG__IMAGE_GET_LOD::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_CD class methods ---

Inst_MIMG__IMAGE_SAMPLE_CD::Inst_MIMG__IMAGE_SAMPLE_CD(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_cd")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_CD

Inst_MIMG__IMAGE_SAMPLE_CD::~Inst_MIMG__IMAGE_SAMPLE_CD() {
} // ~Inst_MIMG__IMAGE_SAMPLE_CD

// --- description from .arch file ---
// sample texture map, with user derivatives (LOD per quad)
void
Inst_MIMG__IMAGE_SAMPLE_CD::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_CD_CL class methods ---

Inst_MIMG__IMAGE_SAMPLE_CD_CL::Inst_MIMG__IMAGE_SAMPLE_CD_CL(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_cd_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_CD_CL

Inst_MIMG__IMAGE_SAMPLE_CD_CL::~Inst_MIMG__IMAGE_SAMPLE_CD_CL() {
} // ~Inst_MIMG__IMAGE_SAMPLE_CD_CL

// --- description from .arch file ---
// sample texture map, with LOD clamp specified in shader, with user
// ---  derivatives (LOD per quad).
void
Inst_MIMG__IMAGE_SAMPLE_CD_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_CD class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_CD::Inst_MIMG__IMAGE_SAMPLE_C_CD(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_cd")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_CD

Inst_MIMG__IMAGE_SAMPLE_C_CD::~Inst_MIMG__IMAGE_SAMPLE_C_CD() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_CD

// --- description from .arch file ---
// SAMPLE_C, with user derivatives (LOD per quad).
void
Inst_MIMG__IMAGE_SAMPLE_C_CD::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_CD_CL class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_CD_CL::Inst_MIMG__IMAGE_SAMPLE_C_CD_CL(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_cd_cl")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_CD_CL

Inst_MIMG__IMAGE_SAMPLE_C_CD_CL::~Inst_MIMG__IMAGE_SAMPLE_C_CD_CL() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_CD_CL

// --- description from .arch file ---
// SAMPLE_C, with LOD clamp specified in shader, with user derivatives
// (LOD per quad).
void
Inst_MIMG__IMAGE_SAMPLE_C_CD_CL::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_CD_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_CD_O::Inst_MIMG__IMAGE_SAMPLE_CD_O(InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_cd_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_CD_O

Inst_MIMG__IMAGE_SAMPLE_CD_O::~Inst_MIMG__IMAGE_SAMPLE_CD_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_CD_O

// --- description from .arch file ---
// SAMPLE_O, with user derivatives (LOD per quad).
void
Inst_MIMG__IMAGE_SAMPLE_CD_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_CD_CL_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_CD_CL_O::Inst_MIMG__IMAGE_SAMPLE_CD_CL_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_cd_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_CD_CL_O

Inst_MIMG__IMAGE_SAMPLE_CD_CL_O::~Inst_MIMG__IMAGE_SAMPLE_CD_CL_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_CD_CL_O

// --- description from .arch file ---
// SAMPLE_O, with LOD clamp specified in shader, with user derivatives
// (LOD per quad).
void
Inst_MIMG__IMAGE_SAMPLE_CD_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_CD_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_CD_O::Inst_MIMG__IMAGE_SAMPLE_C_CD_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_cd_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_CD_O

Inst_MIMG__IMAGE_SAMPLE_C_CD_O::~Inst_MIMG__IMAGE_SAMPLE_C_CD_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_CD_O

// --- description from .arch file ---
// SAMPLE_C_O, with user derivatives (LOD per quad).
void
Inst_MIMG__IMAGE_SAMPLE_C_CD_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute

// --- Inst_MIMG__IMAGE_SAMPLE_C_CD_CL_O class methods ---

Inst_MIMG__IMAGE_SAMPLE_C_CD_CL_O::Inst_MIMG__IMAGE_SAMPLE_C_CD_CL_O(
    InFmt_MIMG *iFmt)
    : Inst_MIMG(iFmt, "image_sample_c_cd_cl_o")
{
    setFlag(GlobalSegment);
} // Inst_MIMG__IMAGE_SAMPLE_C_CD_CL_O

Inst_MIMG__IMAGE_SAMPLE_C_CD_CL_O::~Inst_MIMG__IMAGE_SAMPLE_C_CD_CL_O() {
} // ~Inst_MIMG__IMAGE_SAMPLE_C_CD_CL_O

// --- description from .arch file ---
// SAMPLE_C_O, with LOD clamp specified in shader, with user derivatives
// (LOD per quad).
void
Inst_MIMG__IMAGE_SAMPLE_C_CD_CL_O::execute(GPUDynInstPtr gpuDynInst)
{
    panicUnimplemented();
} // execute
} // namespace VegaISA
} // namespace gem5
