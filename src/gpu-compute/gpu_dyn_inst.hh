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
#include "base/trace.hh"
#include "debug/GPUMem.hh"
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
    }
    AtomicOpFunctor* clone () { return new AtomicOpCAS(c, s, computeUnit); }
};

class GPUDynInst : public GPUExecContext
{
  public:
    GPUDynInst(ComputeUnit *_cu, Wavefront *_wf, GPUStaticInst *static_inst,
               uint64_t instSeqNum);
    ~GPUDynInst();
    void execute(GPUDynInstPtr gpuDynInst);
    int numSrcRegOperands();
    int numDstRegOperands();
    int numDstVecOperands();
    int numSrcVecOperands();
    int numSrcVecDWORDs();
    int numDstVecDWORDs();
    int numOpdDWORDs(int operandIdx);
    int getNumOperands();
    bool isVectorRegister(int operandIdx);
    bool isScalarRegister(int operandIdx);
    int getRegisterIndex(int operandIdx, GPUDynInstPtr gpuDynInst);
    int getOperandSize(int operandIdx);
    bool isDstOperand(int operandIdx);
    bool isSrcOperand(int operandIdx);

    bool hasDestinationSgpr() const;
    bool hasSourceSgpr() const;
    bool hasDestinationVgpr() const;
    bool hasSourceVgpr() const;

    bool hasSgprRawDependence(GPUDynInstPtr s);
    bool hasVgprRawDependence(GPUDynInstPtr s);

    // returns true if the string "opcodeStr" is found in the
    // opcode of the instruction
    bool isOpcode(const std::string& opcodeStr) const;
    bool isOpcode(const std::string& opcodeStr,
                  const std::string& extStr) const;
    // returns true if source operand at "index" is a vector register
    bool srcIsVgpr(int index) const;

    const std::string &disassemble() const;

    InstSeqNum seqNum() const;

    Enums::StorageClassType executedAs();

    // virtual address for scalar memory operations
    Addr scalarAddr;
    // virtual addressies for vector memory operations
    std::vector<Addr> addr;
    Addr pAddr;

    // vector data to get written
    uint8_t *d_data;
    // scalar data to be transferred
    uint8_t *scalar_data;
    // Additional data (for atomics)
    uint8_t *a_data;
    // Additional data (for atomics)
    uint8_t *x_data;
    // The execution mask
    VectorMask exec_mask;

    // SIMD where the WF of the memory instruction has been mapped to
    int simdId;
    // unique id of the WF where the memory instruction belongs to
    int wfDynId;
    // The kernel id of the requesting wf
    int kern_id;
    // The CU id of the requesting wf
    int cu_id;
    // The workgroup id of the requesting wf
    int wg_id;
    // HW slot id where the WF is mapped to inside a SIMD unit
    int wfSlotId;
    // execution pipeline id where the memory instruction has been scheduled
    int execUnitId;
    // The execution time of this operation
    Tick time;
    // The latency of this operation
    WaitClass latency;

    // Initiate the specified memory operation, by creating a
    // memory request and sending it off to the memory system.
    void initiateAcc(GPUDynInstPtr gpuDynInst);
    // Complete the specified memory operation, by writing
    // value back to the RF in the case of a load or atomic
    // return or, in the case of a store, we do nothing
    void completeAcc(GPUDynInstPtr gpuDynInst);

    void updateStats();

    GPUStaticInst* staticInstruction() { return _staticInst; }

    TheGpuISA::ScalarRegU32 srcLiteral() const;

    bool isALU() const;
    bool isBranch() const;
    bool isCondBranch() const;
    bool isNop() const;
    bool isReturn() const;
    bool isEndOfKernel() const;
    bool isKernelLaunch() const;
    bool isSDWAInst() const;
    bool isDPPInst() const;
    bool isUnconditionalJump() const;
    bool isSpecialOp() const;
    bool isWaitcnt() const;

    bool isBarrier() const;
    bool isMemSync() const;
    bool isMemRef() const;
    bool isFlat() const;
    bool isLoad() const;
    bool isStore() const;

    bool isAtomic() const;
    bool isAtomicNoRet() const;
    bool isAtomicRet() const;

    bool isScalar() const;
    bool isVector() const;
    bool readsSCC() const;
    bool writesSCC() const;
    bool readsVCC() const;
    bool writesVCC() const;
    bool readsEXEC() const;
    bool writesEXEC() const;
    bool readsMode() const;
    bool writesMode() const;
    bool ignoreExec() const;
    bool readsFlatScratch() const;
    bool writesFlatScratch() const;
    bool readsExecMask() const;
    bool writesExecMask() const;

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

    bool isGloballyCoherent() const;
    bool isSystemCoherent() const;

    bool isF16() const;
    bool isF32() const;
    bool isF64() const;

    bool isFMA() const;
    bool isMAC() const;
    bool isMAD() const;

    // for FLAT memory ops. check the segment address
    // against the APE registers to see if it falls
    // within one of the APE ranges for LDS/SCRATCH/GPUVM.
    // if it does not fall into one of the three APEs, it
    // will be a regular global access.
    void doApertureCheck(const VectorMask &mask);
    // Function to resolve a flat accesses during execution stage.
    void resolveFlatSegment(const VectorMask &mask);

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
    setRequestFlags(RequestPtr req) const
    {
        if (isGloballyCoherent()) {
            req->setCacheCoherenceFlags(Request::GLC_BIT);
        }

        if (isSystemCoherent()) {
            req->setCacheCoherenceFlags(Request::SLC_BIT);
        }

        if (isAtomicRet()) {
            req->setFlags(Request::ATOMIC_RETURN_OP);
        } else if (isAtomicNoRet()) {
            req->setFlags(Request::ATOMIC_NO_RETURN_OP);
        }

        if (isMemSync()) {
            // the path for kernel launch and kernel end is different
            // from non-kernel mem sync.
            assert(!isKernelLaunch());
            assert(!isEndOfKernel());

            // must be wbinv inst if not kernel launch/end
            req->setCacheCoherenceFlags(Request::ACQUIRE);
        }
    }

    // reset the number of pending memory requests for all lanes
    void
    resetEntireStatusVector()
    {
        assert(statusVector.size() == TheGpuISA::NumVecElemPerVecReg);
        for (int lane = 0; lane < TheGpuISA::NumVecElemPerVecReg; ++lane) {
            resetStatusVector(lane);
        }
    }

    // reset the number of pending memory requests for the inputted lane
    void
    resetStatusVector(int lane)
    {
        setStatusVector(lane, 0);
    }

    // set the number of pending memory requests for the inputted lane
    void
    setStatusVector(int lane, int newVal)
    {
        // currently we can have up to 2 memory requests per lane (if the
        // lane's request goes across multiple cache lines)
        assert((newVal >= 0) && (newVal <= 2));
        statusVector[lane] = newVal;
    }

    // subtracts the number of pending memory requests for the inputted lane
    // by 1
    void
    decrementStatusVector(int lane)
    {
        // this lane may have multiple requests, so only subtract one for
        // this request
        assert(statusVector[lane] >= 1);
        statusVector[lane]--;
    }

    // return the current number of pending memory requests for the inputted
    // lane
    int
    getLaneStatus(int lane) const
    {
        return statusVector[lane];
    }

    // returns true if all memory requests from all lanes have been received,
    // else returns false
    bool
    allLanesZero() const
    {
        // local variables
        bool allZero = true;

        // iterate over all lanes, checking the number of pending memory
        // requests they have
        for (int lane = 0; lane < TheGpuISA::NumVecElemPerVecReg; ++lane) {
            // if any lane still has pending requests, return false
            if (statusVector[lane] > 0) {
                DPRINTF(GPUMem, "CU%d: WF[%d][%d]: lane: %d has %d pending "
                        "request(s) for %#x\n", cu_id, simdId, wfSlotId, lane,
                        statusVector[lane], addr[lane]);
                allZero = false;
            }
        }

        if (allZero) {
            DPRINTF(GPUMem, "CU%d: WF[%d][%d]: all lanes have no pending"
                    " requests for %#x\n", cu_id, simdId, wfSlotId, addr[0]);
        }
        return allZero;
    }

    // returns a string representing the current state of the statusVector
    std::string
    printStatusVector() const
    {
        std::string statusVec_str = "[";

        // iterate over all lanes, adding the current number of pending
        // requests for this lane to the string
        for (int lane = 0; lane < TheGpuISA::NumVecElemPerVecReg; ++lane) {
            statusVec_str += std::to_string(statusVector[lane]);
        }
        statusVec_str += "]";

        return statusVec_str;
    }

    // Map returned packets and the addresses they satisfy with which lane they
    // were requested from
    typedef std::unordered_map<Addr, std::vector<int>> StatusVector;
    StatusVector memStatusVector;

    // Track the status of memory requests per lane, an int per lane to allow
    // unaligned accesses
    std::vector<int> statusVector;
    // for ld_v# or st_v#
    std::vector<int> tlbHitLevel;

    // for misaligned scalar ops we track the number
    // of outstanding reqs here
    int numScalarReqs;

    Tick getAccessTime() const { return accessTime; }

    void setAccessTime(Tick currentTime) { accessTime = currentTime; }

    void profileRoundTripTime(Tick currentTime, int hopId);
    std::vector<Tick> getRoundTripTime() const { return roundTripTime; }

    void profileLineAddressTime(Addr addr, Tick currentTime, int hopId);
    const std::map<Addr, std::vector<Tick>>& getLineAddressTime() const
    { return lineAddressTime; }

    // inst used to save/restore a wavefront context
    bool isSaveRestore;
  private:
    GPUStaticInst *_staticInst;
    const InstSeqNum _seqNum;

    // the time the request was started
    Tick accessTime = -1;

    // hold the tick when the instruction arrives at certain hop points
    // on it's way to main memory
    std::vector<Tick> roundTripTime;

    // hold each cache block address for the instruction and a vector
    // to hold the tick when the block arrives at certain hop points
    std::map<Addr, std::vector<Tick>> lineAddressTime;
};

#endif // __GPU_DYN_INST_HH__
