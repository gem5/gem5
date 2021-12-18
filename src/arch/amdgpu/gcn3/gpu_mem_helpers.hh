/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_GCN3_GPU_MEM_HELPERS_HH__
#define __ARCH_GCN3_GPU_MEM_HELPERS_HH__

#include "arch/amdgpu/gcn3/insts/gpu_static_inst.hh"
#include "arch/amdgpu/gcn3/insts/op_encodings.hh"
#include "debug/GPUMem.hh"
#include "gpu-compute/gpu_dyn_inst.hh"

namespace gem5
{

/**
 * Helper function for instructions declared in op_encodings.  This function
 * takes in all of the arguments for a given memory request we are trying to
 * initialize, then submits the request or requests depending on if the
 * original request is aligned or unaligned.
 */
template<typename T, int N>
inline void
initMemReqHelper(GPUDynInstPtr gpuDynInst, MemCmd mem_req_type,
                 bool is_atomic=false)
{
    // local variables
    int req_size = N * sizeof(T);
    int block_size = gpuDynInst->computeUnit()->cacheLineSize();
    Addr vaddr = 0, split_addr = 0;
    bool misaligned_acc = false;
    RequestPtr req = nullptr, req1 = nullptr, req2 = nullptr;
    PacketPtr pkt = nullptr, pkt1 = nullptr, pkt2 = nullptr;

    gpuDynInst->resetEntireStatusVector();
    for (int lane = 0; lane < Gcn3ISA::NumVecElemPerVecReg; ++lane) {
        if (gpuDynInst->exec_mask[lane]) {
            vaddr = gpuDynInst->addr[lane];

            /**
             * the base address of the cache line where the the last
             * byte of the request will be stored.
             */
            split_addr = roundDown(vaddr + req_size - 1, block_size);

            assert(split_addr <= vaddr || split_addr - vaddr < block_size);
            /**
             * if the base cache line address of the last byte is
             * greater than the address of the first byte then we have
             * a misaligned access.
             */
            misaligned_acc = split_addr > vaddr;

            if (is_atomic) {
                // make sure request is word aligned
                assert((vaddr & 0x3) == 0);

                // a given lane's atomic can't cross cache lines
                assert(!misaligned_acc);

                req = std::make_shared<Request>(vaddr, sizeof(T), 0,
                    gpuDynInst->computeUnit()->requestorId(), 0,
                    gpuDynInst->wfDynId,
                    gpuDynInst->makeAtomicOpFunctor<T>(
                        &(reinterpret_cast<T*>(gpuDynInst->a_data))[lane],
                        &(reinterpret_cast<T*>(gpuDynInst->x_data))[lane]));
            } else {
                req = std::make_shared<Request>(vaddr, req_size, 0,
                                  gpuDynInst->computeUnit()->requestorId(), 0,
                                  gpuDynInst->wfDynId);
            }

            if (misaligned_acc) {
                gpuDynInst->setStatusVector(lane, 2);
                req->splitOnVaddr(split_addr, req1, req2);
                gpuDynInst->setRequestFlags(req1);
                gpuDynInst->setRequestFlags(req2);
                pkt1 = new Packet(req1, mem_req_type);
                pkt2 = new Packet(req2, mem_req_type);
                pkt1->dataStatic(&(reinterpret_cast<T*>(
                    gpuDynInst->d_data))[lane * N]);
                pkt2->dataStatic(&(reinterpret_cast<T*>(
                    gpuDynInst->d_data))[lane * N +
                                         req1->getSize()/sizeof(T)]);
                DPRINTF(GPUMem, "CU%d: WF[%d][%d]: index: %d unaligned memory "
                        "request for %#x\n", gpuDynInst->cu_id,
                        gpuDynInst->simdId, gpuDynInst->wfSlotId, lane,
                        split_addr);
                gpuDynInst->computeUnit()->sendRequest(gpuDynInst, lane, pkt1);
                gpuDynInst->computeUnit()->sendRequest(gpuDynInst, lane, pkt2);
            } else {
                gpuDynInst->setStatusVector(lane, 1);
                gpuDynInst->setRequestFlags(req);
                pkt = new Packet(req, mem_req_type);
                pkt->dataStatic(&(reinterpret_cast<T*>(
                    gpuDynInst->d_data))[lane * N]);
                gpuDynInst->computeUnit()->sendRequest(gpuDynInst, lane, pkt);
            }
        } else { // if lane is not active, then no pending requests
            gpuDynInst->setStatusVector(lane, 0);
        }
    }
}

/**
 * Helper function for scalar instructions declared in op_encodings.  This
 * function takes in all of the arguments for a given memory request we are
 * trying to initialize, then submits the request or requests depending on if
 * the original request is aligned or unaligned.
 */
template<typename T, int N>
inline void
initMemReqScalarHelper(GPUDynInstPtr gpuDynInst, MemCmd mem_req_type)
{
    int req_size = N * sizeof(T);
    int block_size = gpuDynInst->computeUnit()->cacheLineSize();
    Addr vaddr = gpuDynInst->scalarAddr;

    /**
     * the base address of the cache line where the the last byte of
     * the request will be stored.
     */
    Addr split_addr = roundDown(vaddr + req_size - 1, block_size);

    assert(split_addr <= vaddr || split_addr - vaddr < block_size);
    /**
     * if the base cache line address of the last byte is greater
     * than the address of the first byte then we have a misaligned
     * access.
     */
    bool misaligned_acc = split_addr > vaddr;

    RequestPtr req = std::make_shared<Request>(vaddr, req_size, 0,
                                 gpuDynInst->computeUnit()->requestorId(), 0,
                                 gpuDynInst->wfDynId);

    if (misaligned_acc) {
        RequestPtr req1, req2;
        req->splitOnVaddr(split_addr, req1, req2);
        gpuDynInst->numScalarReqs = 2;
        gpuDynInst->setRequestFlags(req1);
        gpuDynInst->setRequestFlags(req2);
        PacketPtr pkt1 = new Packet(req1, mem_req_type);
        PacketPtr pkt2 = new Packet(req2, mem_req_type);
        pkt1->dataStatic(gpuDynInst->scalar_data);
        pkt2->dataStatic(gpuDynInst->scalar_data + req1->getSize());
        DPRINTF(GPUMem, "CU%d: WF[%d][%d]: unaligned scalar memory request for"
                " %#x\n", gpuDynInst->cu_id, gpuDynInst->simdId,
                gpuDynInst->wfSlotId, split_addr);
        gpuDynInst->computeUnit()->sendScalarRequest(gpuDynInst, pkt1);
        gpuDynInst->computeUnit()->sendScalarRequest(gpuDynInst, pkt2);
    } else {
        gpuDynInst->numScalarReqs = 1;
        gpuDynInst->setRequestFlags(req);
        PacketPtr pkt = new Packet(req, mem_req_type);
        pkt->dataStatic(gpuDynInst->scalar_data);
        gpuDynInst->computeUnit()->sendScalarRequest(gpuDynInst, pkt);
    }
}

} // namespace gem5

#endif // __ARCH_GCN3_GPU_MEM_HELPERS_HH__
