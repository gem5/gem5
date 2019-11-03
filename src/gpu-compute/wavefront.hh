/*
 * Copyright (c) 2011-2017 Advanced Micro Devices, Inc.
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
 *
 * Authors: Lisa Hsu
 */

#ifndef __WAVEFRONT_HH__
#define __WAVEFRONT_HH__

#include <cassert>
#include <deque>
#include <memory>
#include <stack>
#include <vector>

#include "arch/gpu_isa.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "config/the_gpu_isa.hh"
#include "gpu-compute/condition_register_state.hh"
#include "gpu-compute/lds_state.hh"
#include "gpu-compute/misc.hh"
#include "gpu-compute/ndrange.hh"
#include "params/Wavefront.hh"
#include "sim/sim_object.hh"

static const int MAX_NUM_INSTS_PER_WF = 12;

/**
 * A reconvergence stack entry conveys the necessary state to implement
 * control flow divergence.
 */
struct ReconvergenceStackEntry {
    /**
     * PC of current instruction.
     */
    uint32_t pc;
    /**
     * PC of the immediate post-dominator instruction, i.e., the value of
     * @a pc for the first instruction that will be executed by the wavefront
     * when a reconvergence point is reached.
     */
    uint32_t rpc;
    /**
     * Execution mask.
     */
    VectorMask execMask;
};

/*
 * Arguments for the hsail opcode call, are user defined and variable length.
 * The hardware/finalizer can support arguments in hardware or use memory to
 * pass arguments. For now, let's assume that an unlimited number of arguments
 * are supported in hardware (the compiler inlines functions whenver it can
 * anyways, so unless someone is interested in the implications of linking/
 * library functions, I think this is a reasonable assumption given the typical
 * size of an OpenCL kernel).
 *
 * Note that call args are different than kernel arguments:
 *   * All work-items in a kernel refer the same set of kernel arguments
 *   * Each work-item has it's on set of call args. So a call argument at
 *     address 0x4 is different for work-item 0 and work-item 1.
 *
 * Ok, the table below shows an example of how we organize the call arguments in
 * the CallArgMem class.
 *
 * int foo(int arg1, double arg2)
 *  ___________________________________________________
 * | 0: return.0 | 4: return.1 | ... | 252: return.63  |
 * |---------------------------------------------------|
 * | 256: arg1.0 | 260: arg1.1 | ... | 508: arg1.63    |
 * |---------------------------------------------------|
 * | 512: arg2.0 | 520: arg2.1 | ... | 1016: arg2.63   |
 *  ___________________________________________________
 */
class CallArgMem
{
  public:
    // pointer to buffer for storing function arguments
    uint8_t *mem;
    int wfSize;
    // size of function args
    int funcArgsSizePerItem;

    template<typename CType>
    int
    getLaneOffset(int lane, int addr)
    {
        return addr * wfSize + sizeof(CType) * lane;
    }

    CallArgMem(int func_args_size_per_item, int wf_size)
        : wfSize(wf_size), funcArgsSizePerItem(func_args_size_per_item)
    {
        mem = (uint8_t*)malloc(funcArgsSizePerItem * wfSize);
    }

    ~CallArgMem()
    {
        free(mem);
    }

    template<typename CType>
    uint8_t*
    getLaneAddr(int lane, int addr)
    {
        return mem + getLaneOffset<CType>(lane, addr);
    }

    template<typename CType>
    void
    setLaneAddr(int lane, int addr, CType val)
    {
        *((CType*)(mem + getLaneOffset<CType>(lane, addr))) = val;
    }
};

class Wavefront : public SimObject
{
  public:
    enum itype_e {I_ALU,I_GLOBAL,I_SHARED,I_FLAT,I_PRIVATE};
    enum status_e {S_STOPPED,S_RETURNING,S_RUNNING};

    // Base pointer for array of instruction pointers
    uint64_t basePtr;

    uint32_t oldBarrierCnt;
    uint32_t barrierCnt;
    uint32_t barrierId;
    uint32_t barrierSlots;
    status_e status;
    // HW slot id where the WF is mapped to inside a SIMD unit
    int wfSlotId;
    int kernId;
    // SIMD unit where the WV has been scheduled
    int simdId;
    // pointer to parent CU
    ComputeUnit *computeUnit;

    std::deque<GPUDynInstPtr> instructionBuffer;

    bool pendingFetch;
    bool dropFetch;

    // Condition Register State (for HSAIL simulations only)
    class ConditionRegisterState *condRegState;
    // number of single precision VGPRs required by WF
    uint32_t maxSpVgprs;
    // number of double precision VGPRs required by WF
    uint32_t maxDpVgprs;
    // map virtual to physical vector register
    uint32_t remap(uint32_t vgprIndex, uint32_t size, uint8_t mode=0);
    void resizeRegFiles(int num_cregs, int num_sregs, int num_dregs);
    bool isGmInstruction(GPUDynInstPtr ii);
    bool isLmInstruction(GPUDynInstPtr ii);
    bool isOldestInstGMem();
    bool isOldestInstLMem();
    bool isOldestInstPrivMem();
    bool isOldestInstFlatMem();
    bool isOldestInstALU();
    bool isOldestInstBarrier();
    // used for passing spill address to DDInstGPU
    std::vector<Addr> lastAddr;
    std::vector<uint32_t> workItemId[3];
    std::vector<uint32_t> workItemFlatId;
    /* kernel launch parameters */
    uint32_t workGroupId[3];
    uint32_t workGroupSz[3];
    uint32_t gridSz[3];
    uint32_t wgId;
    uint32_t wgSz;
    /* the actual WG size can differ than the maximum size */
    uint32_t actualWgSz[3];
    uint32_t actualWgSzTotal;
    void computeActualWgSz(NDRange *ndr);
    // wavefront id within a workgroup
    uint32_t wfId;
    uint32_t maxDynWaveId;
    uint32_t dispatchId;
    // outstanding global+local memory requests
    uint32_t outstandingReqs;
    // memory requests between scoreboard
    // and execute stage not yet executed
    uint32_t memReqsInPipe;
    // outstanding global memory write requests
    uint32_t outstandingReqsWrGm;
    // outstanding local memory write requests
    uint32_t outstandingReqsWrLm;
    // outstanding global memory read requests
    uint32_t outstandingReqsRdGm;
    // outstanding local memory read requests
    uint32_t outstandingReqsRdLm;
    uint32_t rdLmReqsInPipe;
    uint32_t rdGmReqsInPipe;
    uint32_t wrLmReqsInPipe;
    uint32_t wrGmReqsInPipe;

    int memTraceBusy;
    uint64_t lastTrace;
    // number of vector registers reserved by WF
    int reservedVectorRegs;
    // Index into the Vector Register File's namespace where the WF's registers
    // will live while the WF is executed
    uint32_t startVgprIndex;

    // Old value of destination gpr (for trace)
    std::vector<uint32_t> oldVgpr;
    // Id of destination gpr (for trace)
    uint32_t oldVgprId;
    // Tick count of last old_vgpr copy
    uint64_t oldVgprTcnt;

    // Old value of destination gpr (for trace)
    std::vector<uint64_t> oldDgpr;
    // Id of destination gpr (for trace)
    uint32_t oldDgprId;
    // Tick count of last old_vgpr copy
    uint64_t oldDgprTcnt;

    // Execution mask at wavefront start
    VectorMask initMask;

    // number of barriers this WF has joined
    std::vector<int> barCnt;
    int maxBarCnt;
    // Flag to stall a wave on barrier
    bool stalledAtBarrier;

    // a pointer to the fraction of the LDS allocated
    // to this workgroup (thus this wavefront)
    LdsChunk *ldsChunk;

    // A pointer to the spill area
    Addr spillBase;
    // The size of the spill area
    uint32_t spillSizePerItem;
    // The vector width of the spill area
    uint32_t spillWidth;

    // A pointer to the private memory area
    Addr privBase;
    // The size of the private memory area
    uint32_t privSizePerItem;

    // A pointer ot the read-only memory area
    Addr roBase;
    // size of the read-only memory area
    uint32_t roSize;

    // pointer to buffer for storing kernel arguments
    uint8_t *kernelArgs;
    // unique WF id over all WFs executed across all CUs
    uint64_t wfDynId;

    // number of times instruction issue for this wavefront is blocked
    // due to VRF port availability
    Stats::Scalar numTimesBlockedDueVrfPortAvail;
    // number of times an instruction of a WF is blocked from being issued
    // due to WAR and WAW dependencies
    Stats::Scalar numTimesBlockedDueWAXDependencies;
    // number of times an instruction of a WF is blocked from being issued
    // due to WAR and WAW dependencies
    Stats::Scalar numTimesBlockedDueRAWDependencies;
    // distribution of executed instructions based on their register
    // operands; this is used to highlight the load on the VRF
    Stats::Distribution srcRegOpDist;
    Stats::Distribution dstRegOpDist;

    // Functions to operate on call argument memory
    // argument memory for hsail call instruction
    CallArgMem *callArgMem;
    void
    initCallArgMem(int func_args_size_per_item, int wf_size)
    {
        callArgMem = new CallArgMem(func_args_size_per_item, wf_size);
    }

    template<typename CType>
    CType
    readCallArgMem(int lane, int addr)
    {
        return *((CType*)(callArgMem->getLaneAddr<CType>(lane, addr)));
    }

    template<typename CType>
    void
    writeCallArgMem(int lane, int addr, CType val)
    {
        callArgMem->setLaneAddr<CType>(lane, addr, val);
    }

    typedef WavefrontParams Params;
    Wavefront(const Params *p);
    ~Wavefront();
    virtual void init();

    void
    setParent(ComputeUnit *cu)
    {
        computeUnit = cu;
    }

    void start(uint64_t _wfDynId, uint64_t _base_ptr);
    void exec();
    void updateResources();
    int ready(itype_e type);
    bool instructionBufferHasBranch();
    void regStats();
    VectorMask getPred() { return execMask() & initMask; }

    bool waitingAtBarrier(int lane);

    void pushToReconvergenceStack(uint32_t pc, uint32_t rpc,
                                  const VectorMask& exec_mask);

    void popFromReconvergenceStack();

    uint32_t pc() const;

    uint32_t rpc() const;

    VectorMask execMask() const;

    bool execMask(int lane) const;

    void pc(uint32_t new_pc);

    void discardFetch();

    /**
     * Returns the size of the static hardware context of a particular wavefront
     * This should be updated everytime the context is changed
     */
    uint32_t getStaticContextSize() const;

    /**
     * Returns the hardware context as a stream of bytes
     * This method is designed for HSAIL execution
     */
    void getContext(const void *out);

    /**
     * Sets the hardware context fromt a stream of bytes
     * This method is designed for HSAIL execution
     */
    void setContext(const void *in);

    TheGpuISA::GPUISA&
    gpuISA()
    {
        return _gpuISA;
    }

  private:
    TheGpuISA::GPUISA _gpuISA;
    /**
     * Stack containing Control Flow Graph nodes (i.e., kernel instructions)
     * to be visited by the wavefront, and the associated execution masks. The
     * reconvergence stack grows every time the wavefront reaches a divergence
     * point (branch instruction), and shrinks every time the wavefront
     * reaches a reconvergence point (immediate post-dominator instruction).
     */
    std::deque<std::unique_ptr<ReconvergenceStackEntry>> reconvergenceStack;
};

#endif // __WAVEFRONT_HH__
