/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
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
 * Author: Lisa Hsu
 */

#ifndef __WAVEFRONT_HH__
#define __WAVEFRONT_HH__

#include <cassert>
#include <deque>
#include <memory>
#include <stack>
#include <vector>

#include "base/misc.hh"
#include "base/types.hh"
#include "gpu-compute/condition_register_state.hh"
#include "gpu-compute/lds_state.hh"
#include "gpu-compute/misc.hh"
#include "params/Wavefront.hh"
#include "sim/sim_object.hh"

static const int MAX_NUM_INSTS_PER_WF = 12;

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
    // size of function args
    int funcArgsSizePerItem;

    template<typename CType>
    int
    getLaneOffset(int lane, int addr)
    {
        return addr * VSZ + sizeof(CType) * lane;
    }

    CallArgMem(int func_args_size_per_item)
      : funcArgsSizePerItem(func_args_size_per_item)
    {
        mem = (uint8_t*)malloc(funcArgsSizePerItem * VSZ);
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

/**
 * A reconvergence stack entry conveys the necessary state to implement
 * control flow divergence.
 */
class ReconvergenceStackEntry {

  public:
    ReconvergenceStackEntry(uint32_t new_pc, uint32_t new_rpc,
                            VectorMask new_mask) : pc(new_pc), rpc(new_rpc),
                            execMask(new_mask) {
    }

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

class Wavefront : public SimObject
{
  public:
    enum itype_e {I_ALU,I_GLOBAL,I_SHARED,I_FLAT,I_PRIVATE};
    enum status_e {S_STOPPED,S_RETURNING,S_RUNNING};

    // Base pointer for array of instruction pointers
    uint64_t base_ptr;

    uint32_t old_barrier_cnt;
    uint32_t barrier_cnt;
    uint32_t barrier_id;
    uint32_t barrier_slots;
    status_e status;
    // HW slot id where the WF is mapped to inside a SIMD unit
    int wfSlotId;
    int kern_id;
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
    uint64_t last_addr[VSZ];
    uint32_t workitemid[3][VSZ];
    uint32_t workitemFlatId[VSZ];
    uint32_t workgroupid[3];
    uint32_t workgroupsz[3];
    uint32_t gridsz[3];
    uint32_t wg_id;
    uint32_t wg_sz;
    uint32_t dynwaveid;
    uint32_t maxdynwaveid;
    uint32_t dispatchid;
    // outstanding global+local memory requests
    uint32_t outstanding_reqs;
    // memory requests between scoreboard
    // and execute stage not yet executed
    uint32_t mem_reqs_in_pipe;
    // outstanding global memory write requests
    uint32_t outstanding_reqs_wr_gm;
    // outstanding local memory write requests
    uint32_t outstanding_reqs_wr_lm;
    // outstanding global memory read requests
    uint32_t outstanding_reqs_rd_gm;
    // outstanding local memory read requests
    uint32_t outstanding_reqs_rd_lm;
    uint32_t rd_lm_reqs_in_pipe;
    uint32_t rd_gm_reqs_in_pipe;
    uint32_t wr_lm_reqs_in_pipe;
    uint32_t wr_gm_reqs_in_pipe;

    int mem_trace_busy;
    uint64_t last_trace;
    // number of vector registers reserved by WF
    int reservedVectorRegs;
    // Index into the Vector Register File's namespace where the WF's registers
    // will live while the WF is executed
    uint32_t startVgprIndex;

    // Old value of destination gpr (for trace)
    uint32_t old_vgpr[VSZ];
    // Id of destination gpr (for trace)
    uint32_t old_vgpr_id;
    // Tick count of last old_vgpr copy
    uint64_t old_vgpr_tcnt;

    // Old value of destination gpr (for trace)
    uint64_t old_dgpr[VSZ];
    // Id of destination gpr (for trace)
    uint32_t old_dgpr_id;
    // Tick count of last old_vgpr copy
    uint64_t old_dgpr_tcnt;

    // Execution mask at wavefront start
    VectorMask init_mask;

    // number of barriers this WF has joined
    int bar_cnt[VSZ];
    int max_bar_cnt;
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
    initCallArgMem(int func_args_size_per_item)
    {
        callArgMem = new CallArgMem(func_args_size_per_item);
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
    VectorMask get_pred() { return execMask() & init_mask; }

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

  private:
    /**
     * Stack containing Control Flow Graph nodes (i.e., kernel instructions)
     * to be visited by the wavefront, and the associated execution masks. The
     * reconvergence stack grows every time the wavefront reaches a divergence
     * point (branch instruction), and shrinks every time the wavefront
     * reaches a reconvergence point (immediate post-dominator instruction).
     */
    std::stack<std::unique_ptr<ReconvergenceStackEntry>> reconvergenceStack;
};

#endif // __WAVEFRONT_HH__
