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
 * Author: Steve Reinhardt
 */

#ifndef __SHADER_HH__
#define __SHADER_HH__

#include <functional>
#include <string>

#include "arch/isa.hh"
#include "arch/isa_traits.hh"
#include "base/types.hh"
#include "cpu/simple/atomic.hh"
#include "cpu/simple/timing.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "enums/MemType.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_tlb.hh"
#include "gpu-compute/lds_state.hh"
#include "gpu-compute/qstruct.hh"
#include "mem/page_table.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "params/Shader.hh"
#include "sim/faults.hh"
#include "sim/process.hh"
#include "sim/sim_object.hh"

class BaseTLB;
class GpuDispatcher;

namespace TheISA
{
    class GpuTLB;
}

static const int LDS_SIZE = 65536;

// Class Shader: This describes a single shader instance. Most
// configurations will only have a single shader.

class Shader : public SimObject
{
  protected:
      // Shader's clock period in terms of number of ticks of curTime,
      // aka global simulation clock
      Tick clock;

  public:
    typedef ShaderParams Params;
    enum hsail_mode_e {SIMT,VECTOR_SCALAR};

    // clock related functions ; maps to-and-from
    // Simulation ticks and shader clocks.
    Tick frequency() const { return SimClock::Frequency / clock; }

    Tick ticks(int numCycles) const { return  (Tick)clock * numCycles; }

    Tick getClock() const { return clock; }
    Tick curCycle() const { return curTick() / clock; }
    Tick tickToCycles(Tick val) const { return val / clock;}


    SimpleThread *cpuThread;
    ThreadContext *gpuTc;
    BaseCPU *cpuPointer;

    class TickEvent : public Event
    {
      private:
        Shader *shader;

      public:
        TickEvent(Shader*);
        void process();
        const char* description() const;
    };

    TickEvent tickEvent;

    // is this simulation going to be timing mode in the memory?
    bool timingSim;
    hsail_mode_e hsail_mode;

    // If set, issue acq packet @ kernel launch
    int impl_kern_boundary_sync;
    // If set, generate a separate packet for acquire/release on
    // ld_acquire/st_release/atomic operations
    int separate_acquire_release;
    // If set, fetch returns may be coissued with instructions
    int coissue_return;
    // If set, always dump all 64 gprs to trace
    int trace_vgpr_all;
    // Number of cu units in the shader
    int n_cu;
    // Number of wavefront slots per cu
    int n_wf;
    // The size of global memory
    int globalMemSize;

    /*
     * Bytes/work-item for call instruction
     * The number of arguments for an hsail function will
     * vary. We simply determine the maximum # of arguments
     * required by any hsail function up front before the
     * simulation (during parsing of the Brig) and record
     * that number here.
     */
    int funcargs_size;

    // Tracks CU that rr dispatcher should attempt scheduling
    int nextSchedCu;

    // Size of scheduled add queue
    uint32_t sa_n;

    // Pointer to value to be increments
    std::vector<uint32_t*> sa_val;
    // When to do the increment
    std::vector<uint64_t> sa_when;
    // Amount to increment by
    std::vector<int32_t> sa_x;

    // List of Compute Units (CU's)
    std::vector<ComputeUnit*> cuList;

    uint64_t tick_cnt;
    uint64_t box_tick_cnt;
    uint64_t start_tick_cnt;

    GpuDispatcher *dispatcher;

    Shader(const Params *p);
    ~Shader();
    virtual void init();

    // Run shader
    void exec();

    // Check to see if shader is busy
    bool busy();

    // Schedule a 32-bit value to be incremented some time in the future
    void ScheduleAdd(uint32_t *val, Tick when, int x);
    bool processTimingPacket(PacketPtr pkt);

    void AccessMem(uint64_t address, void *ptr, uint32_t size, int cu_id,
                   MemCmd cmd, bool suppress_func_errors);

    void ReadMem(uint64_t address, void *ptr, uint32_t sz, int cu_id);

    void ReadMem(uint64_t address, void *ptr, uint32_t sz, int cu_id,
                 bool suppress_func_errors);

    void WriteMem(uint64_t address, void *ptr, uint32_t sz, int cu_id);

    void WriteMem(uint64_t address, void *ptr, uint32_t sz, int cu_id,
                  bool suppress_func_errors);

    void doFunctionalAccess(RequestPtr req, MemCmd cmd, void *data,
                            bool suppress_func_errors, int cu_id);

    void
    registerCU(int cu_id, ComputeUnit *compute_unit)
    {
        cuList[cu_id] = compute_unit;
    }

    void handshake(GpuDispatcher *dispatcher);
    bool dispatch_workgroups(NDRange *ndr);
    Addr mmap(int length);
    void functionalTLBAccess(PacketPtr pkt, int cu_id, BaseTLB::Mode mode);
    void updateContext(int cid);
    void hostWakeUp(BaseCPU *cpu);
};

#endif // __SHADER_HH__
