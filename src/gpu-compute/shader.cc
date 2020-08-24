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

#include "gpu-compute/shader.hh"

#include <limits>

#include "arch/x86/isa_traits.hh"
#include "arch/x86/linux/linux.hh"
#include "base/chunk_generator.hh"
#include "debug/GPUDisp.hh"
#include "debug/GPUMem.hh"
#include "debug/GPUShader.hh"
#include "debug/GPUWgLatency.hh"
#include "gpu-compute/dispatcher.hh"
#include "gpu-compute/gpu_command_processor.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/hsa_queue_entry.hh"
#include "gpu-compute/wavefront.hh"
#include "mem/packet.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "sim/sim_exit.hh"

Shader::Shader(const Params *p) : ClockedObject(p),
    _activeCus(0), _lastInactiveTick(0), cpuThread(nullptr),
    gpuTc(nullptr), cpuPointer(p->cpu_pointer),
    tickEvent([this]{ execScheduledAdds(); }, "Shader scheduled adds event",
          false, Event::CPU_Tick_Pri),
    timingSim(p->timing), hsail_mode(SIMT),
    impl_kern_launch_acq(p->impl_kern_launch_acq),
    impl_kern_end_rel(p->impl_kern_end_rel),
    coissue_return(1),
    trace_vgpr_all(1), n_cu((p->CUs).size()), n_wf(p->n_wf),
    globalMemSize(p->globalmem),
    nextSchedCu(0), sa_n(0), gpuCmdProc(*p->gpu_cmd_proc),
    _dispatcher(*p->dispatcher),
    max_valu_insts(p->max_valu_insts), total_valu_insts(0)
{
    gpuCmdProc.setShader(this);
    _dispatcher.setShader(this);

    _gpuVmApe.base = ((Addr)1 << 61) + 0x1000000000000L;
    _gpuVmApe.limit = (_gpuVmApe.base & 0xFFFFFF0000000000UL) | 0xFFFFFFFFFFL;

    _ldsApe.base = ((Addr)1 << 61) + 0x0;
    _ldsApe.limit =  (_ldsApe.base & 0xFFFFFFFF00000000UL) | 0xFFFFFFFF;

    _scratchApe.base = ((Addr)1 << 61) + 0x100000000L;
    _scratchApe.limit = (_scratchApe.base & 0xFFFFFFFF00000000UL) | 0xFFFFFFFF;

    shHiddenPrivateBaseVmid = 0;

    cuList.resize(n_cu);

    panic_if(n_wf <= 0, "Must have at least 1 WF Slot per SIMD");

    for (int i = 0; i < n_cu; ++i) {
        cuList[i] = p->CUs[i];
        assert(i == cuList[i]->cu_id);
        cuList[i]->shader = this;
        cuList[i]->idleCUTimeout = p->idlecu_timeout;
    }
}

GPUDispatcher&
Shader::dispatcher()
{
    return _dispatcher;
}

Addr
Shader::mmap(int length)
{

    Addr start;

    // round up length to the next page
    length = roundUp(length, TheISA::PageBytes);

    Process *proc = gpuTc->getProcessPtr();
    auto mem_state = proc->memState;

    if (proc->mmapGrowsDown()) {
        DPRINTF(GPUShader, "GROWS DOWN");
        start = mem_state->getMmapEnd() - length;
        mem_state->setMmapEnd(start);
    } else {
        DPRINTF(GPUShader, "GROWS UP");
        start = mem_state->getMmapEnd();
        mem_state->setMmapEnd(start + length);

        // assertion to make sure we don't overwrite the stack (it grows down)
        assert(mem_state->getStackBase() - mem_state->getMaxStackSize() >
               mem_state->getMmapEnd());
    }

    DPRINTF(GPUShader, "Shader::mmap start= %#x, %#x\n", start, length);

    proc->allocateMem(start, length);

    return start;
}

void
Shader::init()
{
    // grab the threadContext of the thread running on the CPU
    assert(cpuPointer);
    gpuTc = cpuPointer->getContext(0);
    assert(gpuTc);
}

Shader::~Shader()
{
    for (int j = 0; j < n_cu; ++j)
        delete cuList[j];
}

void
Shader::updateContext(int cid) {
    // context of the thread which dispatched work
    assert(cpuPointer);
    gpuTc = cpuPointer->getContext(cid);
    assert(gpuTc);
}

Shader*
ShaderParams::create()
{
    return new Shader(this);
}

void
Shader::execScheduledAdds()
{
    assert(!sa_when.empty());

    // apply any scheduled adds
    for (int i = 0; i < sa_n; ++i) {
        if (sa_when[i] <= curTick()) {
            *sa_val[i] += sa_x[i];
            panic_if(*sa_val[i] < 0, "Negative counter value\n");
            sa_val.erase(sa_val.begin() + i);
            sa_x.erase(sa_x.begin() + i);
            sa_when.erase(sa_when.begin() + i);
            --sa_n;
            --i;
        }
    }
    if (!sa_when.empty()) {
        Tick shader_wakeup = *std::max_element(sa_when.begin(),
                 sa_when.end());
        DPRINTF(GPUDisp, "Scheduling shader wakeup at %lu\n", shader_wakeup);
        schedule(tickEvent, shader_wakeup);
    } else {
        DPRINTF(GPUDisp, "sa_when empty, shader going to sleep!\n");
    }
}

/*
 * dispatcher/shader arranges invalidate requests to the CUs
 */
void
Shader::prepareInvalidate(HSAQueueEntry *task) {
    // if invalidate has already started/finished, then do nothing
    if (task->isInvStarted()) return;

    // invalidate has never started; it can only perform once at kernel launch
    assert(task->outstandingInvs() == -1);
    int kernId = task->dispatchId();
    // counter value is 0 now, indicating the inv is about to start
    _dispatcher.updateInvCounter(kernId, +1);

    // iterate all cus managed by the shader, to perform invalidate.
    for (int i_cu = 0; i_cu < n_cu; ++i_cu) {
        // create a request to hold INV info; the request's fields will
        // be updated in cu before use
        auto req = std::make_shared<Request>(0, 0, 0,
                                             cuList[i_cu]->requestorId(),
                                             0, -1);

        _dispatcher.updateInvCounter(kernId, +1);
        // all necessary INV flags are all set now, call cu to execute
        cuList[i_cu]->doInvalidate(req, task->dispatchId());
    }
}

/**
 * dispatcher/shader arranges flush requests to the CUs
 */
void
Shader::prepareFlush(GPUDynInstPtr gpuDynInst){
    int kernId = gpuDynInst->kern_id;
    // flush has never been started, performed only once at kernel end
    assert(_dispatcher.getOutstandingWbs(kernId) == 0);

    // the first cu, managed by the shader, performs flush operation,
    // assuming that L2 cache is shared by all cus in the shader
    int i_cu = 0;
    _dispatcher.updateWbCounter(kernId, +1);
    cuList[i_cu]->doFlush(gpuDynInst);
}

bool
Shader::dispatchWorkgroups(HSAQueueEntry *task)
{
    bool scheduledSomething = false;
    int cuCount = 0;
    int curCu = nextSchedCu;

    while (cuCount < n_cu) {
        //Every time we try a CU, update nextSchedCu
        nextSchedCu = (nextSchedCu + 1) % n_cu;

        // dispatch workgroup iff the following two conditions are met:
        // (a) wg_rem is true - there are unassigned workgroups in the grid
        // (b) there are enough free slots in cu cuList[i] for this wg
        int num_wfs_in_wg = 0;
        bool can_disp = cuList[curCu]->hasDispResources(task, num_wfs_in_wg);
        if (!task->dispComplete() && can_disp) {
            scheduledSomething = true;
            DPRINTF(GPUDisp, "Dispatching a workgroup to CU %d: WG %d\n",
                            curCu, task->globalWgId());
            DPRINTF(GPUWgLatency, "WG Begin cycle:%d wg:%d cu:%d\n",
                    curTick(), task->globalWgId(), curCu);

            if (!cuList[curCu]->tickEvent.scheduled()) {
                if (!_activeCus)
                    _lastInactiveTick = curTick();
                _activeCus++;
            }

            panic_if(_activeCus <= 0 || _activeCus > cuList.size(),
                     "Invalid activeCu size\n");
            cuList[curCu]->dispWorkgroup(task, num_wfs_in_wg);

            task->markWgDispatch();
        }

        ++cuCount;
        curCu = nextSchedCu;
    }

    return scheduledSomething;
}

void
Shader::regStats()
{
    ClockedObject::regStats();

    shaderActiveTicks
        .name(name() + ".shader_active_ticks")
        .desc("Total ticks that any CU attached to this shader is active")
        ;
    allLatencyDist
        .init(0, 1600000, 10000)
        .name(name() + ".allLatencyDist")
        .desc("delay distribution for all")
        .flags(Stats::pdf | Stats::oneline);

    loadLatencyDist
        .init(0, 1600000, 10000)
        .name(name() + ".loadLatencyDist")
        .desc("delay distribution for loads")
        .flags(Stats::pdf | Stats::oneline);

    storeLatencyDist
        .init(0, 1600000, 10000)
        .name(name() + ".storeLatencyDist")
        .desc("delay distribution for stores")
        .flags(Stats::pdf | Stats::oneline);

    vectorInstSrcOperand
        .init(4)
        .name(name() + ".vec_inst_src_operand")
        .desc("vector instruction source operand distribution");

    vectorInstDstOperand
        .init(4)
        .name(name() + ".vec_inst_dst_operand")
        .desc("vector instruction destination operand distribution");

    initToCoalesceLatency
        .init(0, 1600000, 10000)
        .name(name() + ".initToCoalesceLatency")
        .desc("Ticks from vmem inst initiateAcc to coalescer issue")
        .flags(Stats::pdf | Stats::oneline);

    rubyNetworkLatency
        .init(0, 1600000, 10000)
        .name(name() + ".rubyNetworkLatency")
        .desc("Ticks from coalescer issue to coalescer hit callback")
        .flags(Stats::pdf | Stats::oneline);

    gmEnqueueLatency
        .init(0, 1600000, 10000)
        .name(name() + ".gmEnqueueLatency")
        .desc("Ticks from coalescer hit callback to GM pipe enqueue")
        .flags(Stats::pdf | Stats::oneline);

    gmToCompleteLatency
        .init(0, 1600000, 10000)
        .name(name() + ".gmToCompleteLatency")
        .desc("Ticks queued in GM pipes ordered response buffer")
        .flags(Stats::pdf | Stats::oneline);

    coalsrLineAddresses
        .init(0, 20, 1)
        .name(name() + ".coalsrLineAddresses")
        .desc("Number of cache lines for coalesced request")
        .flags(Stats::pdf | Stats::oneline);

    int wfSize = cuList[0]->wfSize();
    cacheBlockRoundTrip = new Stats::Distribution[wfSize];
    for (int idx = 0; idx < wfSize; ++idx) {
        std::stringstream namestr;
        ccprintf(namestr, "%s.cacheBlockRoundTrip%d", name(), idx);
        cacheBlockRoundTrip[idx]
            .init(0, 1600000, 10000)
            .name(namestr.str())
            .desc("Coalsr-to-coalsr time for the Nth cache block in an inst")
            .flags(Stats::pdf | Stats::oneline);
    }
}

void
Shader::doFunctionalAccess(const RequestPtr &req, MemCmd cmd, void *data,
                           bool suppress_func_errors, int cu_id)
{
    int block_size = cuList.at(cu_id)->cacheLineSize();
    unsigned size = req->getSize();

    Addr tmp_addr;
    BaseTLB::Mode trans_mode;

    if (cmd == MemCmd::ReadReq) {
        trans_mode = BaseTLB::Read;
    } else if (cmd == MemCmd::WriteReq) {
        trans_mode = BaseTLB::Write;
    } else {
        fatal("unexcepted MemCmd\n");
    }

    tmp_addr = req->getVaddr();
    Addr split_addr = roundDown(tmp_addr + size - 1, block_size);

    assert(split_addr <= tmp_addr || split_addr - tmp_addr < block_size);

    // Misaligned access
    if (split_addr > tmp_addr) {
        RequestPtr req1, req2;
        req->splitOnVaddr(split_addr, req1, req2);

        PacketPtr pkt1 = new Packet(req2, cmd);
        PacketPtr pkt2 = new Packet(req1, cmd);

        functionalTLBAccess(pkt1, cu_id, trans_mode);
        functionalTLBAccess(pkt2, cu_id, trans_mode);

        PacketPtr new_pkt1 = new Packet(pkt1->req, cmd);
        PacketPtr new_pkt2 = new Packet(pkt2->req, cmd);

        new_pkt1->dataStatic(data);
        new_pkt2->dataStatic((uint8_t*)data + req1->getSize());

        if (suppress_func_errors) {
            new_pkt1->setSuppressFuncError();
            new_pkt2->setSuppressFuncError();
        }

        // fixme: this should be cuList[cu_id] if cu_id != n_cu
        // The latter requires a memPort in the dispatcher
        cuList[0]->memPort[0].sendFunctional(new_pkt1);
        cuList[0]->memPort[0].sendFunctional(new_pkt2);

        delete new_pkt1;
        delete new_pkt2;
        delete pkt1;
        delete pkt2;
    } else {
        PacketPtr pkt = new Packet(req, cmd);
        functionalTLBAccess(pkt, cu_id, trans_mode);
        PacketPtr new_pkt = new Packet(pkt->req, cmd);
        new_pkt->dataStatic(data);

        if (suppress_func_errors) {
            new_pkt->setSuppressFuncError();
        };

        // fixme: this should be cuList[cu_id] if cu_id != n_cu
        // The latter requires a memPort in the dispatcher
        cuList[0]->memPort[0].sendFunctional(new_pkt);

        delete new_pkt;
        delete pkt;
    }
}

void
Shader::ScheduleAdd(int *val,Tick when,int x)
{
    sa_val.push_back(val);
    when += curTick();
    sa_when.push_back(when);
    sa_x.push_back(x);
    ++sa_n;
    if (!tickEvent.scheduled() || (when < tickEvent.when())) {
        DPRINTF(GPUDisp, "New scheduled add; scheduling shader wakeup at "
                "%lu\n", when);
        reschedule(tickEvent, when, true);
    } else {
        assert(tickEvent.scheduled());
        DPRINTF(GPUDisp, "New scheduled add; wakeup already scheduled at "
                "%lu\n", when);
    }
}

void
Shader::AccessMem(uint64_t address, void *ptr, uint32_t size, int cu_id,
                  MemCmd cmd, bool suppress_func_errors)
{
    uint8_t *data_buf = (uint8_t*)ptr;

    for (ChunkGenerator gen(address, size, cuList.at(cu_id)->cacheLineSize());
         !gen.done(); gen.next()) {

        RequestPtr req = std::make_shared<Request>(
            gen.addr(), gen.size(), 0,
            cuList[0]->requestorId(), 0, 0, nullptr);

        doFunctionalAccess(req, cmd, data_buf, suppress_func_errors, cu_id);
        data_buf += gen.size();
    }
}

void
Shader::ReadMem(uint64_t address, void *ptr, uint32_t size, int cu_id)
{
    AccessMem(address, ptr, size, cu_id, MemCmd::ReadReq, false);
}

void
Shader::ReadMem(uint64_t address, void *ptr, uint32_t size, int cu_id,
                bool suppress_func_errors)
{
    AccessMem(address, ptr, size, cu_id, MemCmd::ReadReq,
        suppress_func_errors);
}

void
Shader::WriteMem(uint64_t address, void *ptr,uint32_t size, int cu_id)
{
    AccessMem(address, ptr, size, cu_id, MemCmd::WriteReq, false);
}

void
Shader::WriteMem(uint64_t address, void *ptr, uint32_t size, int cu_id,
                 bool suppress_func_errors)
{
    AccessMem(address, ptr, size, cu_id, MemCmd::WriteReq,
              suppress_func_errors);
}

/*
 * Send a packet through the appropriate TLB functional port.
 * If cu_id=n_cu, then this is the dispatcher's TLB.
 * Otherwise it's the TLB of the cu_id compute unit.
 */
void
Shader::functionalTLBAccess(PacketPtr pkt, int cu_id, BaseTLB::Mode mode)
{
    // update senderState. Need to know the gpuTc and the TLB mode
    pkt->senderState =
        new TheISA::GpuTLB::TranslationState(mode, gpuTc, false);

    // even when the perLaneTLB flag is turned on
    // it's ok tp send all accesses through lane 0
    // since the lane # is not known here,
    // This isn't important since these are functional accesses.
    cuList[cu_id]->tlbPort[0].sendFunctional(pkt);

    /* safe_cast the senderState */
    TheISA::GpuTLB::TranslationState *sender_state =
               safe_cast<TheISA::GpuTLB::TranslationState*>(pkt->senderState);

    delete sender_state->tlbEntry;
    delete pkt->senderState;
}

/*
 * allow the shader to sample stats from constituent devices
 */
void
Shader::sampleStore(const Tick accessTime)
{
    storeLatencyDist.sample(accessTime);
    allLatencyDist.sample(accessTime);
}

/*
 * allow the shader to sample stats from constituent devices
 */
void
Shader::sampleLoad(const Tick accessTime)
{
    loadLatencyDist.sample(accessTime);
    allLatencyDist.sample(accessTime);
}

void
Shader::sampleInstRoundTrip(std::vector<Tick> roundTripTime)
{
    // Only sample instructions that go all the way to main memory
    if (roundTripTime.size() != InstMemoryHop::InstMemoryHopMax) {
        return;
    }

    Tick t1 = roundTripTime[0];
    Tick t2 = roundTripTime[1];
    Tick t3 = roundTripTime[2];
    Tick t4 = roundTripTime[3];
    Tick t5 = roundTripTime[4];

    initToCoalesceLatency.sample(t2-t1);
    rubyNetworkLatency.sample(t3-t2);
    gmEnqueueLatency.sample(t4-t3);
    gmToCompleteLatency.sample(t5-t4);
}

void
Shader::sampleLineRoundTrip(const std::map<Addr, std::vector<Tick>>& lineMap)
{
    coalsrLineAddresses.sample(lineMap.size());
    std::vector<Tick> netTimes;

    // For each cache block address generated by a vmem inst, calculate
    // the round-trip time for that cache block.
    for (auto& it : lineMap) {
        const std::vector<Tick>& timeVec = it.second;
        if (timeVec.size() == 2) {
            netTimes.push_back(timeVec[1] - timeVec[0]);
        }
    }

    // Sort the cache block round trip times so that the first
    // distrubtion is always measuring the fastests and the last
    // distrubtion is always measuring the slowest cache block.
    std::sort(netTimes.begin(), netTimes.end());

    // Sample the round trip time for each N cache blocks into the
    // Nth distribution.
    int idx = 0;
    for (auto& time : netTimes) {
        cacheBlockRoundTrip[idx].sample(time);
        ++idx;
    }
}

void
Shader::notifyCuSleep() {
    // If all CUs attached to his shader are asleep, update shaderActiveTicks
    panic_if(_activeCus <= 0 || _activeCus > cuList.size(),
             "Invalid activeCu size\n");
    _activeCus--;
    if (!_activeCus)
        shaderActiveTicks += curTick() - _lastInactiveTick;
}
