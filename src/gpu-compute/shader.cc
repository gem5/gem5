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

#include "arch/x86/linux/linux.hh"
#include "base/chunk_generator.hh"
#include "debug/GPUDisp.hh"
#include "debug/GPUMem.hh"
#include "debug/HSAIL.hh"
#include "gpu-compute/dispatcher.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/qstruct.hh"
#include "gpu-compute/wavefront.hh"
#include "mem/packet.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "sim/sim_exit.hh"

Shader::Shader(const Params *p)
    : ClockedObject(p), clock(p->clk_domain->clockPeriod()),
      cpuThread(nullptr), gpuTc(nullptr), cpuPointer(p->cpu_pointer),
      tickEvent([this]{ processTick(); }, "Shader tick",
                false, Event::CPU_Tick_Pri),
      timingSim(p->timing), hsail_mode(SIMT),
      impl_kern_boundary_sync(p->impl_kern_boundary_sync),
      separate_acquire_release(p->separate_acquire_release), coissue_return(1),
      trace_vgpr_all(1), n_cu((p->CUs).size()), n_wf(p->n_wf),
      globalMemSize(p->globalmem), nextSchedCu(0), sa_n(0), tick_cnt(0),
      box_tick_cnt(0), start_tick_cnt(0)
{

    cuList.resize(n_cu);

    for (int i = 0; i < n_cu; ++i) {
        cuList[i] = p->CUs[i];
        assert(i == cuList[i]->cu_id);
        cuList[i]->shader = this;
    }
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
        DPRINTF(HSAIL, "GROWS DOWN");
        start = mem_state->getMmapEnd() - length;
        mem_state->setMmapEnd(start);
    } else {
        DPRINTF(HSAIL, "GROWS UP");
        start = mem_state->getMmapEnd();
        mem_state->setMmapEnd(start + length);

        // assertion to make sure we don't overwrite the stack (it grows down)
        assert(mem_state->getStackBase() - mem_state->getMaxStackSize() >
               mem_state->getMmapEnd());
    }

    DPRINTF(HSAIL,"Shader::mmap start= %#x, %#x\n", start, length);

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

void
Shader::hostWakeUp(BaseCPU *cpu) {
    if (cpuPointer == cpu) {
        if (gpuTc->status() == ThreadContext::Suspended)
            cpu->activateContext(gpuTc->threadId());
    } else {
        //Make sure both dispatcher and shader are trying to
        //wakeup same host. Hack here to enable kernel launch
        //from multiple CPUs
        panic("Dispatcher wants to wakeup a different host");
    }
}

Shader*
ShaderParams::create()
{
    return new Shader(this);
}

void
Shader::exec()
{
    tick_cnt = curTick();
    box_tick_cnt = curTick() - start_tick_cnt;

    // apply any scheduled adds
    for (int i = 0; i < sa_n; ++i) {
        if (sa_when[i] <= tick_cnt) {
            *sa_val[i] += sa_x[i];
            sa_val.erase(sa_val.begin() + i);
            sa_x.erase(sa_x.begin() + i);
            sa_when.erase(sa_when.begin() + i);
            --sa_n;
            --i;
        }
    }

    // clock all of the cu's
    for (int i = 0; i < n_cu; ++i)
        cuList[i]->exec();
}

bool
Shader::dispatch_workgroups(NDRange *ndr)
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
        if (ndr->wg_disp_rem && cuList[curCu]->ReadyWorkgroup(ndr)) {
            scheduledSomething = true;
            DPRINTF(GPUDisp, "Dispatching a workgroup to CU %d\n", curCu);

            // ticks() member function translates cycles to simulation ticks.
            if (!tickEvent.scheduled()) {
                schedule(tickEvent, curTick() + this->ticks(1));
            }

            cuList[curCu]->StartWorkgroup(ndr);
            ndr->wgId[0]++;
            ndr->globalWgId++;
            if (ndr->wgId[0] * ndr->q.wgSize[0] >= ndr->q.gdSize[0]) {
                ndr->wgId[0] = 0;
                ndr->wgId[1]++;

                if (ndr->wgId[1] * ndr->q.wgSize[1] >= ndr->q.gdSize[1]) {
                    ndr->wgId[1] = 0;
                    ndr->wgId[2]++;

                    if (ndr->wgId[2] * ndr->q.wgSize[2] >= ndr->q.gdSize[2]) {
                        ndr->wg_disp_rem = false;
                        break;
                    }
                }
            }
        }

        ++cuCount;
        curCu = nextSchedCu;
    }

    return scheduledSomething;
}

void
Shader::handshake(GpuDispatcher *_dispatcher)
{
    dispatcher = _dispatcher;
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
        cuList[0]->memPort[0]->sendFunctional(new_pkt1);
        cuList[0]->memPort[0]->sendFunctional(new_pkt2);

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
        cuList[0]->memPort[0]->sendFunctional(new_pkt);

        delete new_pkt;
        delete pkt;
    }
}

bool
Shader::busy()
{
    for (int i_cu = 0; i_cu < n_cu; ++i_cu) {
        if (!cuList[i_cu]->isDone()) {
            return true;
        }
    }

    return false;
}

void
Shader::ScheduleAdd(uint32_t *val,Tick when,int x)
{
    sa_val.push_back(val);
    sa_when.push_back(tick_cnt + when);
    sa_x.push_back(x);
    ++sa_n;
}


void
Shader::processTick()
{
    if (busy()) {
        exec();
        schedule(tickEvent, curTick() + ticks(1));
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
            cuList[0]->masterId(), 0, 0, nullptr);

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
    AccessMem(address, ptr, size, cu_id, MemCmd::ReadReq, suppress_func_errors);
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

    if (cu_id == n_cu) {
        dispatcher->tlbPort->sendFunctional(pkt);
    } else {
        // even when the perLaneTLB flag is turned on
        // it's ok tp send all accesses through lane 0
        // since the lane # is not known here,
        // This isn't important since these are functional accesses.
        cuList[cu_id]->tlbPort[0]->sendFunctional(pkt);
    }

    /* safe_cast the senderState */
    TheISA::GpuTLB::TranslationState *sender_state =
               safe_cast<TheISA::GpuTLB::TranslationState*>(pkt->senderState);

    delete sender_state->tlbEntry;
    delete pkt->senderState;
}
