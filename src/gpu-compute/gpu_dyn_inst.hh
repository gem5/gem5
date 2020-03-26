/*
 * Copyright (c) 2015-2017 Advanced Micro Devices, Inc.
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

#ifndef __GPU_DYN_INST_HH__
#define __GPU_DYN_INST_HH__

#include <cstdint>
#include <string>

#include "base/amo.hh"
#include "base/logging.hh"
#include "enums/MemType.hh"
#include "enums/StorageClassType.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_exec_context.hh"

class GPUStaticInst;

template<typename T>
class AtomicOpCAS : public TypedAtomicOpFunctor<T>
{
  public:
    T c;
    T s;

    ComputeUnit *computeUnit;

    AtomicOpCAS(T _c, T _s, ComputeUnit *compute_unit)
      : c(_c), s(_s), computeUnit(compute_unit) { }

    void
    execute(T *b)
    {
        computeUnit->numCASOps++;

        if (*b == c) {
            *b = s;
        } else {
            computeUnit->numFailedCASOps++;
        }

        if (computeUnit->xact_cas_mode) {
            computeUnit->xactCasLoadMap.clear();
        }
    }
    AtomicOpFunctor* clone () { return new AtomicOpCAS(c, s, computeUnit); }
};

typedef enum
{
    VT_32,
    VT_64,
} vgpr_type;

class GPUDynInst : public GPUExecContext
{
  public:
    GPUDynInst(ComputeUnit *_cu, Wavefront *_wf, GPUStaticInst *static_inst,
               uint64_t instSeqNum);
    ~GPUDynInst();
    void execute(GPUDynInstPtr gpuDynInst);
    int numSrcRegOperands();
    int numDstRegOperands();
    int getNumOperands();
    bool isVectorRegister(int operandIdx);
    bool isScalarRegister(int operandIdx);
    bool isCondRegister(int operandIdx);
    int getRegisterIndex(int operandIdx, GPUDynInstPtr gpuDynInst);
    int getOperandSize(int operandIdx);
    bool isDstOperand(int operandIdx);
    bool isSrcOperand(int operandIdx);

    const std::string &disassemble() const;

    uint64_t seqNum() const;

    Enums::StorageClassType executedAs();

    // The address of the memory operation
    std::vector<Addr> addr;
    Addr pAddr;

    // The data to get written
    uint8_t *d_data;
    // Additional data (for atomics)
    uint8_t *a_data;
    // Additional data (for atomics)
    uint8_t *x_data;
    // The execution mask
    VectorMask exec_mask;

    // The memory type (M_U32, M_S32, ...)
    Enums::MemType m_type;

    // The equivalency class
    int equiv;
    // The return VGPR type (VT_32 or VT_64)
    vgpr_type v_type;
    // Number of VGPR's accessed (1, 2, or 4)
    int n_reg;
    // The return VGPR index
    int dst_reg;
    // There can be max 4 dest regs>
    int dst_reg_vec[4];
    // SIMD where the WF of the memory instruction has been mapped to
    int simdId;
    // unique id of the WF where the memory instruction belongs to
    int wfDynId;
    // The kernel id of the requesting wf
    int kern_id;
    // The CU id of the requesting wf
    int cu_id;
    // HW slot id where the WF is mapped to inside a SIMD unit
    int wfSlotId;
    // execution pipeline id where the memory instruction has been scheduled
    int pipeId;
    // The execution time of this operation
    Tick time;
    // The latency of this operation
    WaitClass latency;
    // A list of bank conflicts for the 4 cycles.
    uint32_t bc[4];

    // A pointer to ROM
    uint8_t *rom;
    // The size of the READONLY segment
    int sz_rom;

    // Initiate the specified memory operation, by creating a
    // memory request and sending it off to the memory system.
    void initiateAcc(GPUDynInstPtr gpuDynInst);
    // Complete the specified memory operation, by writing
    // value back to the RF in the case of a load or atomic
    // return or, in the case of a store, we do nothing
    void completeAcc(GPUDynInstPtr gpuDynInst);

    void updateStats();

    GPUStaticInst* staticInstruction() { return _staticInst; }

    bool isALU() const;
    bool isBranch() const;
    bool isNop() const;
    bool isReturn() const;
    bool isUnconditionalJump() const;
    bool isSpecialOp() const;
    bool isWaitcnt() const;

    bool isBarrier() const;
    bool isMemFence() const;
    bool isMemRef() const;
    bool isFlat() const;
    bool isLoad() const;
    bool isStore() const;

    bool isAtomic() const;
    bool isAtomicNoRet() const;
    bool isAtomicRet() const;

    bool isScalar() const;
    bool readsSCC() const;
    bool writesSCC() const;
    bool readsVCC() const;
    bool writesVCC() const;

    bool isAtomicAnd() const;
    bool isAtomicOr() const;
    bool isAtomicXor() const;
    bool isAtomicCAS() const;
    bool isAtomicExch() const;
    bool isAtomicAdd() const;
    bool isAtomicSub() const;
    bool isAtomicInc() const;
    bool isAtomicDec() const;
    bool isAtomicMax() const;
    bool isAtomicMin() const;

    bool isArgLoad() const;
    bool isGlobalMem() const;
    bool isLocalMem() const;

    bool isArgSeg() const;
    bool isGlobalSeg() const;
    bool isGroupSeg() const;
    bool isKernArgSeg() const;
    bool isPrivateSeg() const;
    bool isReadOnlySeg() const;
    bool isSpillSeg() const;

    bool isWorkitemScope() const;
    bool isWavefrontScope() const;
    bool isWorkgroupScope() const;
    bool isDeviceScope() const;
    bool isSystemScope() const;
    bool isNoScope() const;

    bool isRelaxedOrder() const;
    bool isAcquire() const;
    bool isRelease() const;
    bool isAcquireRelease() const;
    bool isNoOrder() const;

    bool isGloballyCoherent() const;
    bool isSystemCoherent() const;

    /*
     * Loads/stores/atomics may have acquire/release semantics associated
     * withthem. Some protocols want to see the acquire/release as separate
     * requests from the load/store/atomic. We implement that separation
     * using continuations (i.e., a function pointer with an object associated
     * with it). When, for example, the front-end generates a store with
     * release semantics, we will first issue a normal store and set the
     * continuation in the GPUDynInst to a function that generate a
     * release request. That continuation will be called when the normal
     * store completes (in ComputeUnit::DataPort::recvTimingResponse). The
     * continuation will be called in the context of the same GPUDynInst
     * that generated the initial store.
     */
    std::function<void(GPUStaticInst*, GPUDynInstPtr)> execContinuation;

    // when true, call execContinuation when response arrives
    bool useContinuation;

    template<typename c0> AtomicOpFunctorPtr
    makeAtomicOpFunctor(c0 *reg0, c0 *reg1)
    {
        if (isAtomicAnd()) {
            return m5::make_unique<AtomicOpAnd<c0>>(*reg0);
        } else if (isAtomicOr()) {
            return m5::make_unique<AtomicOpOr<c0>>(*reg0);
        } else if (isAtomicXor()) {
            return m5::make_unique<AtomicOpXor<c0>>(*reg0);
        } else if (isAtomicCAS()) {
            return m5::make_unique<AtomicOpCAS<c0>>(*reg0, *reg1, cu);
        } else if (isAtomicExch()) {
            return m5::make_unique<AtomicOpExch<c0>>(*reg0);
        } else if (isAtomicAdd()) {
            return m5::make_unique<AtomicOpAdd<c0>>(*reg0);
        } else if (isAtomicSub()) {
            return m5::make_unique<AtomicOpSub<c0>>(*reg0);
        } else if (isAtomicInc()) {
            return m5::make_unique<AtomicOpInc<c0>>();
        } else if (isAtomicDec()) {
            return m5::make_unique<AtomicOpDec<c0>>();
        } else if (isAtomicMax()) {
            return m5::make_unique<AtomicOpMax<c0>>(*reg0);
        } else if (isAtomicMin()) {
            return m5::make_unique<AtomicOpMin<c0>>(*reg0);
        } else {
            fatal("Unrecognized atomic operation");
        }
    }

    void
    setRequestFlags(RequestPtr req, bool setMemOrder=true)
    {
        // currently these are the easy scopes to deduce
        if (isPrivateSeg()) {
            req->setMemSpaceConfigFlags(Request::PRIVATE_SEGMENT);
        } else if (isSpillSeg()) {
            req->setMemSpaceConfigFlags(Request::SPILL_SEGMENT);
        } else if (isGlobalSeg()) {
            req->setMemSpaceConfigFlags(Request::GLOBAL_SEGMENT);
        } else if (isReadOnlySeg()) {
            req->setMemSpaceConfigFlags(Request::READONLY_SEGMENT);
        } else if (isGroupSeg()) {
            req->setMemSpaceConfigFlags(Request::GROUP_SEGMENT);
        } else if (isFlat()) {
            panic("TODO: translate to correct scope");
        } else {
            fatal("%s has bad segment type\n", disassemble());
        }

        if (isWavefrontScope()) {
            req->setMemSpaceConfigFlags(Request::SCOPE_VALID |
                                        Request::WAVEFRONT_SCOPE);
        } else if (isWorkgroupScope()) {
            req->setMemSpaceConfigFlags(Request::SCOPE_VALID |
                                        Request::WORKGROUP_SCOPE);
        } else if (isDeviceScope()) {
            req->setMemSpaceConfigFlags(Request::SCOPE_VALID |
                                        Request::DEVICE_SCOPE);
        } else if (isSystemScope()) {
            req->setMemSpaceConfigFlags(Request::SCOPE_VALID |
                                        Request::SYSTEM_SCOPE);
        } else if (!isNoScope() && !isWorkitemScope()) {
            fatal("%s has bad scope type\n", disassemble());
        }

        if (setMemOrder) {
            // set acquire and release flags
            if (isAcquire()) {
                req->setFlags(Request::ACQUIRE);
            } else if (isRelease()) {
                req->setFlags(Request::RELEASE);
            } else if (isAcquireRelease()) {
                req->setFlags(Request::ACQUIRE | Request::RELEASE);
            } else if (!isNoOrder()) {
                fatal("%s has bad memory order\n", disassemble());
            }
        }

        // set atomic type
        // currently, the instruction genenerator only produces atomic return
        // but a magic instruction can produce atomic no return
        if (isAtomicRet()) {
            req->setFlags(Request::ATOMIC_RETURN_OP);
        } else if (isAtomicNoRet()) {
            req->setFlags(Request::ATOMIC_NO_RETURN_OP);
        }
    }

    // Map returned packets and the addresses they satisfy with which lane they
    // were requested from
    typedef std::unordered_map<Addr, std::vector<int>> StatusVector;
    StatusVector memStatusVector;

    // Track the status of memory requests per lane, a bit per lane
    VectorMask statusBitVector;
    // for ld_v# or st_v#
    std::vector<int> statusVector;
    std::vector<int> tlbHitLevel;

  private:
    GPUStaticInst *_staticInst;
    uint64_t _seqNum;
};

#endif // __GPU_DYN_INST_HH__
