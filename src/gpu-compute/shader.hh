/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
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

#ifndef __SHADER_HH__
#define __SHADER_HH__

#include <functional>
#include <string>

#include "arch/gpu_isa.hh"
#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/types.hh"
#include "cpu/simple/atomic.hh"
#include "cpu/simple/timing.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "dev/amdgpu/system_hub.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/hsa_queue_entry.hh"
#include "gpu-compute/lds_state.hh"
#include "mem/page_table.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "params/Shader.hh"
#include "sim/faults.hh"
#include "sim/process.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class BaseTLB;
class GPUCommandProcessor;
class GPUDispatcher;

static const int LDS_SIZE = 65536;

// aperture (APE) registers define the base/limit
// pair for the ATC mapped memory space. currently
// the only APEs we consider are for GPUVM/LDS/scratch.
// the APEs are registered with unique values based
// on a per-device basis
struct ApertureRegister
{
    Addr base;
    Addr limit;
};

// Class Shader: This describes a single shader instance. Most
// configurations will only have a single shader.

class Shader : public ClockedObject
{
  private:
    ApertureRegister _gpuVmApe;
    ApertureRegister _ldsApe;
    ApertureRegister _scratchApe;
    Addr shHiddenPrivateBaseVmid;

    // Hardware regs accessed by getreg/setreg instructions, set by queues
    std::unordered_map<int, uint32_t> hwRegs;

    // Number of active Cus attached to this shader
    int _activeCus;

    // Last tick that all CUs attached to this shader were inactive
    Tick _lastInactiveTick;

    // If a kernel-based exit event was requested, wait for all CUs in the
    // shader to complete before actually exiting so that stats are updated.
    bool kernelExitRequested = false;

  public:
    typedef ShaderParams Params;
    enum hsail_mode_e {SIMT,VECTOR_SCALAR};

    GPUDispatcher &dispatcher();
    void sampleLoad(const Tick accessTime);
    void sampleStore(const Tick accessTime);
    void sampleInstRoundTrip(std::vector<Tick> roundTripTime);
    void sampleLineRoundTrip(const std::map<Addr,
        std::vector<Tick>> &roundTripTime);

    SimpleThread *cpuThread;
    ThreadContext *gpuTc;
    BaseCPU *cpuPointer;

    void
    setHwReg(int regIdx, uint32_t val)
    {
        hwRegs[regIdx] = val;
    }

    uint32_t
    getHwReg(int regIdx)
    {
        return hwRegs[regIdx];
    }

    const ApertureRegister&
    gpuVmApe() const
    {
        return _gpuVmApe;
    }

    const ApertureRegister&
    ldsApe() const
    {
        return _ldsApe;
    }

    void
    setLdsApe(Addr base, Addr limit)
    {
        _ldsApe.base = base;
        _ldsApe.limit = limit;
    }

    const ApertureRegister&
    scratchApe() const
    {
        return _scratchApe;
    }

    void
    setScratchApe(Addr base, Addr limit)
    {
        _scratchApe.base = base;
        _scratchApe.limit = limit;
    }

    bool
    isGpuVmApe(Addr addr) const
    {
        bool is_gpu_vm = addr >= _gpuVmApe.base && addr <= _gpuVmApe.limit;

        return is_gpu_vm;
    }

    bool
    isLdsApe(Addr addr) const
    {
        bool is_lds = addr >= _ldsApe.base && addr <= _ldsApe.limit;

        return is_lds;
    }

    bool
    isScratchApe(Addr addr) const
    {
        bool is_scratch
            = addr >= _scratchApe.base && addr <= _scratchApe.limit;

        return is_scratch;
    }

    Addr
    getScratchBase()
    {
        return _scratchApe.base;
    }

    Addr
    getHiddenPrivateBase()
    {
        return shHiddenPrivateBaseVmid;
    }

    void
    initShHiddenPrivateBase(Addr queueBase, uint32_t offset)
    {
        Addr sh_hidden_base_new = queueBase - offset;

        // We are initializing sh_hidden_private_base_vmid from the
        // amd queue descriptor from the first queue.
        // The sh_hidden_private_base_vmid is supposed to be same for
        // all the queues from the same process
        if (shHiddenPrivateBaseVmid != sh_hidden_base_new) {
            // Do not panic if shHiddenPrivateBaseVmid == 0,
            // that is if it is uninitialized. Panic only
            // if the value is initilized and we get
            // a differnt base later.
            panic_if(shHiddenPrivateBaseVmid != 0,
                     "Currently we support only single process\n");
        }
        shHiddenPrivateBaseVmid = sh_hidden_base_new;
    }

    RequestorID vramRequestorId();

    EventFunctionWrapper tickEvent;

    // is this simulation going to be timing mode in the memory?
    bool timingSim;
    hsail_mode_e hsail_mode;

    // If set, issue acq packet @ kernel launch
    int impl_kern_launch_acq;
    // If set, issue rel packet @ kernel end
    int impl_kern_end_rel;
    // If set, fetch returns may be coissued with instructions
    int coissue_return;
    // If set, always dump all 64 gprs to trace
    int trace_vgpr_all;
    // Number of cu units in the shader
    int n_cu;
    // Number of wavefront slots per SIMD per CU
    int n_wf;

    // The size of global memory
    int globalMemSize;

    // Tracks CU that rr dispatcher should attempt scheduling
    int nextSchedCu;

    // Size of scheduled add queue
    uint32_t sa_n;

    // Pointer to value to be increments
    std::vector<int*> sa_val;
    // When to do the increment
    std::vector<uint64_t> sa_when;
    // Amount to increment by
    std::vector<int32_t> sa_x;

    // List of Compute Units (CU's)
    std::vector<ComputeUnit*> cuList;

    GPUCommandProcessor &gpuCmdProc;
    GPUDispatcher &_dispatcher;
    AMDGPUSystemHub *systemHub;

    int64_t max_valu_insts;
    int64_t total_valu_insts;

    Shader(const Params &p);
    ~Shader();
    virtual void init();

    // Run shader scheduled adds
    void execScheduledAdds();

    // Schedule a 32-bit value to be incremented some time in the future
    void ScheduleAdd(int *val, Tick when, int x);
    bool processTimingPacket(PacketPtr pkt);

    void AccessMem(uint64_t address, void *ptr, uint32_t size, int cu_id,
                   MemCmd cmd, bool suppress_func_errors);

    void ReadMem(uint64_t address, void *ptr, uint32_t sz, int cu_id);

    void ReadMem(uint64_t address, void *ptr, uint32_t sz, int cu_id,
                 bool suppress_func_errors);

    void WriteMem(uint64_t address, void *ptr, uint32_t sz, int cu_id);

    void WriteMem(uint64_t address, void *ptr, uint32_t sz, int cu_id,
                  bool suppress_func_errors);

    void doFunctionalAccess(const RequestPtr &req, MemCmd cmd, void *data,
                            bool suppress_func_errors, int cu_id);

    void
    registerCU(int cu_id, ComputeUnit *compute_unit)
    {
        cuList[cu_id] = compute_unit;
    }

    void prepareInvalidate(HSAQueueEntry *task);
    void prepareFlush(GPUDynInstPtr gpuDynInst);

    bool dispatchWorkgroups(HSAQueueEntry *task);
    Addr mmap(int length);
    void functionalTLBAccess(PacketPtr pkt, int cu_id, BaseMMU::Mode mode);
    void updateContext(int cid);
    void notifyCuSleep();

    void
    incVectorInstSrcOperand(int num_operands)
    {
        stats.vectorInstSrcOperand[num_operands]++;
    }

    void
    incVectorInstDstOperand(int num_operands)
    {
        stats.vectorInstDstOperand[num_operands]++;
    }

    void
    requestKernelExitEvent()
    {
        kernelExitRequested = true;
    }

  protected:
    struct ShaderStats : public statistics::Group
    {
        ShaderStats(statistics::Group *parent, int wf_size);

        // some stats for measuring latency
        statistics::Distribution allLatencyDist;
        statistics::Distribution loadLatencyDist;
        statistics::Distribution storeLatencyDist;

        // average ticks from vmem inst initiateAcc to coalescer issue,
        statistics::Distribution initToCoalesceLatency;

        // average ticks from coalescer issue to coalescer hit callback,
        statistics::Distribution rubyNetworkLatency;

        // average ticks from coalescer hit callback to GM pipe enqueue,
        statistics::Distribution gmEnqueueLatency;

        // average ticks spent in GM pipe's ordered resp buffer.
        statistics::Distribution gmToCompleteLatency;

        // average number of cache blocks requested by vmem inst
        statistics::Distribution coalsrLineAddresses;

        // average ticks for cache blocks to main memory for the Nth
        // cache block generated by a vmem inst.
        statistics::Distribution *cacheBlockRoundTrip;

        statistics::Scalar shaderActiveTicks;
        statistics::Vector vectorInstSrcOperand;
        statistics::Vector vectorInstDstOperand;
    } stats;
};

} // namespace gem5

#endif // __SHADER_HH__
