/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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
 * Author: Anthony Gutierrez
 */

#ifndef __GPU_DYN_INST_HH__
#define __GPU_DYN_INST_HH__

#include <cstdint>
#include <string>

#include "enums/GenericMemoryOrder.hh"
#include "enums/GenericMemoryScope.hh"
#include "enums/MemOpType.hh"
#include "enums/MemType.hh"
#include "enums/OpType.hh"
#include "enums/StorageClassType.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_exec_context.hh"

class GPUStaticInst;

template<typename T>
class AtomicOpAnd : public TypedAtomicOpFunctor<T>
{
  public:
    T a;

    AtomicOpAnd(T _a) : a(_a) { }
    void execute(T *b) { *b &= a; }
};

template<typename T>
class AtomicOpOr : public TypedAtomicOpFunctor<T>
{
  public:
    T a;
    AtomicOpOr(T _a) : a(_a) { }
    void execute(T *b) { *b |= a; }
};

template<typename T>
class AtomicOpXor : public TypedAtomicOpFunctor<T>
{
  public:
    T a;
    AtomicOpXor(T _a) : a(_a) {}
    void execute(T *b) { *b ^= a; }
};

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
};

template<typename T>
class AtomicOpExch : public TypedAtomicOpFunctor<T>
{
  public:
    T a;
    AtomicOpExch(T _a) : a(_a) { }
    void execute(T *b) { *b = a; }
};

template<typename T>
class AtomicOpAdd : public TypedAtomicOpFunctor<T>
{
  public:
    T a;
    AtomicOpAdd(T _a) : a(_a) { }
    void execute(T *b) { *b += a; }
};

template<typename T>
class AtomicOpSub : public TypedAtomicOpFunctor<T>
{
  public:
    T a;
    AtomicOpSub(T _a) : a(_a) { }
    void execute(T *b) { *b -= a; }
};

template<typename T>
class AtomicOpInc : public TypedAtomicOpFunctor<T>
{
  public:
    AtomicOpInc() { }
    void execute(T *b) { *b += 1; }
};

template<typename T>
class AtomicOpDec : public TypedAtomicOpFunctor<T>
{
  public:
    AtomicOpDec() {}
    void execute(T *b) { *b -= 1; }
};

template<typename T>
class AtomicOpMax : public TypedAtomicOpFunctor<T>
{
  public:
    T a;
    AtomicOpMax(T _a) : a(_a) { }

    void
    execute(T *b)
    {
        if (a > *b)
            *b = a;
    }
};

template<typename T>
class AtomicOpMin : public TypedAtomicOpFunctor<T>
{
  public:
    T a;
    AtomicOpMin(T _a) : a(_a) {}

    void
    execute(T *b)
    {
        if (a < *b)
            *b = a;
    }
};

#define MO_A(a) ((a)>=Enums::MO_AAND && (a)<=Enums::MO_AMIN)
#define MO_ANR(a) ((a)>=Enums::MO_ANRAND && (a)<=Enums::MO_ANRMIN)
#define MO_H(a) ((a)>=Enums::MO_HAND && (a)<=Enums::MO_HMIN)

typedef enum
{
    VT_32,
    VT_64,
} vgpr_type;

typedef enum
{
    SEG_PRIVATE,
    SEG_SPILL,
    SEG_GLOBAL,
    SEG_SHARED,
    SEG_READONLY,
    SEG_FLAT
} seg_type;

class GPUDynInst : public GPUExecContext
{
  public:
    GPUDynInst(ComputeUnit *_cu, Wavefront *_wf, GPUStaticInst *_staticInst,
               uint64_t instSeqNum);

    void execute();
    int numSrcRegOperands();
    int numDstRegOperands();
    int getNumOperands();
    bool isVectorRegister(int operandIdx);
    bool isScalarRegister(int operandIdx);
    int getRegisterIndex(int operandIdx);
    int getOperandSize(int operandIdx);
    bool isDstOperand(int operandIdx);
    bool isSrcOperand(int operandIdx);
    bool isArgLoad();

    const std::string &disassemble() const;

    uint64_t seqNum() const;

    Enums::OpType opType();
    Enums::StorageClassType executedAs();

    // The address of the memory operation
    Addr addr[VSZ];
    Addr pAddr;

    // The data to get written
    uint8_t d_data[VSZ * 16];
    // Additional data (for atomics)
    uint8_t a_data[VSZ * 8];
    // Additional data (for atomics)
    uint8_t x_data[VSZ * 8];
    // The execution mask
    VectorMask exec_mask;

    // The memory type (M_U32, M_S32, ...)
    Enums::MemType m_type;
    // The memory operation (MO_LD, MO_ST, ...)
    Enums::MemOpType m_op;
    Enums::GenericMemoryOrder memoryOrder;

    // Scope of the request
    Enums::GenericMemoryScope scope;
    // The memory segment (SEG_SHARED, SEG_GLOBAL, ...)
    seg_type s_type;
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

    void updateStats();

    GPUStaticInst* staticInstruction() { return staticInst; }

    // Is the instruction a scalar or vector op?
    bool scalarOp() const;

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

    template<typename c0> AtomicOpFunctor*
    makeAtomicOpFunctor(c0 *reg0, c0 *reg1, Enums::MemOpType op)
    {
        using namespace Enums;

        switch(op) {
          case MO_AAND:
          case MO_ANRAND:
            return new AtomicOpAnd<c0>(*reg0);
          case MO_AOR:
          case MO_ANROR:
            return new AtomicOpOr<c0>(*reg0);
          case MO_AXOR:
          case MO_ANRXOR:
            return new AtomicOpXor<c0>(*reg0);
          case MO_ACAS:
          case MO_ANRCAS:
            return new AtomicOpCAS<c0>(*reg0, *reg1, cu);
          case MO_AEXCH:
          case MO_ANREXCH:
            return new AtomicOpExch<c0>(*reg0);
          case MO_AADD:
          case MO_ANRADD:
            return new AtomicOpAdd<c0>(*reg0);
          case MO_ASUB:
          case MO_ANRSUB:
            return new AtomicOpSub<c0>(*reg0);
          case MO_AINC:
          case MO_ANRINC:
            return new AtomicOpInc<c0>();
          case MO_ADEC:
          case MO_ANRDEC:
            return new AtomicOpDec<c0>();
          case MO_AMAX:
          case MO_ANRMAX:
            return new AtomicOpMax<c0>(*reg0);
          case MO_AMIN:
          case MO_ANRMIN:
            return new AtomicOpMin<c0>(*reg0);
          default:
            panic("Unrecognized atomic operation");
        }
    }

    void
    setRequestFlags(Request *req, bool setMemOrder=true)
    {
        // currently these are the easy scopes to deduce
        switch (s_type) {
          case SEG_PRIVATE:
            req->setMemSpaceConfigFlags(Request::PRIVATE_SEGMENT);
            break;
          case SEG_SPILL:
            req->setMemSpaceConfigFlags(Request::SPILL_SEGMENT);
            break;
          case SEG_GLOBAL:
            req->setMemSpaceConfigFlags(Request::GLOBAL_SEGMENT);
            break;
          case SEG_READONLY:
            req->setMemSpaceConfigFlags(Request::READONLY_SEGMENT);
            break;
          case SEG_SHARED:
            req->setMemSpaceConfigFlags(Request::GROUP_SEGMENT);
            break;
          case SEG_FLAT:
            // TODO: translate to correct scope
            assert(false);
          default:
            panic("Bad segment type");
            break;
        }

        switch (scope) {
          case Enums::MEMORY_SCOPE_NONE:
          case Enums::MEMORY_SCOPE_WORKITEM:
            break;
          case Enums::MEMORY_SCOPE_WAVEFRONT:
            req->setMemSpaceConfigFlags(Request::SCOPE_VALID |
                                        Request::WAVEFRONT_SCOPE);
            break;
          case Enums::MEMORY_SCOPE_WORKGROUP:
            req->setMemSpaceConfigFlags(Request::SCOPE_VALID |
                                        Request::WORKGROUP_SCOPE);
            break;
          case Enums::MEMORY_SCOPE_DEVICE:
            req->setMemSpaceConfigFlags(Request::SCOPE_VALID |
                                        Request::DEVICE_SCOPE);
            break;
          case Enums::MEMORY_SCOPE_SYSTEM:
            req->setMemSpaceConfigFlags(Request::SCOPE_VALID |
                                        Request::SYSTEM_SCOPE);
            break;
          default:
            panic("Bad scope type");
            break;
        }

        if (setMemOrder) {
            // set acquire and release flags
            switch (memoryOrder){
              case Enums::MEMORY_ORDER_SC_ACQUIRE:
                req->setFlags(Request::ACQUIRE);
                break;
              case Enums::MEMORY_ORDER_SC_RELEASE:
                req->setFlags(Request::RELEASE);
                break;
              case Enums::MEMORY_ORDER_SC_ACQUIRE_RELEASE:
                req->setFlags(Request::ACQUIRE | Request::RELEASE);
                break;
              default:
                break;
            }
        }

        // set atomic type
        // currently, the instruction genenerator only produces atomic return
        // but a magic instruction can produce atomic no return
        if (m_op == Enums::MO_AADD || m_op == Enums::MO_ASUB ||
            m_op == Enums::MO_AAND || m_op == Enums::MO_AOR ||
            m_op == Enums::MO_AXOR || m_op == Enums::MO_AMAX ||
            m_op == Enums::MO_AMIN || m_op == Enums::MO_AINC ||
            m_op == Enums::MO_ADEC || m_op == Enums::MO_AEXCH ||
            m_op == Enums::MO_ACAS) {
            req->setFlags(Request::ATOMIC_RETURN_OP);
        } else if (m_op == Enums::MO_ANRADD || m_op == Enums::MO_ANRSUB ||
                   m_op == Enums::MO_ANRAND || m_op == Enums::MO_ANROR ||
                   m_op == Enums::MO_ANRXOR || m_op == Enums::MO_ANRMAX ||
                   m_op == Enums::MO_ANRMIN || m_op == Enums::MO_ANRINC ||
                   m_op == Enums::MO_ANRDEC || m_op == Enums::MO_ANREXCH ||
                   m_op == Enums::MO_ANRCAS) {
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
    GPUStaticInst *staticInst;
    uint64_t _seqNum;
};

#endif // __GPU_DYN_INST_HH__
