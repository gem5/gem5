/*
 * Copyright (c) 2011-2015,2018 Advanced Micro Devices, Inc.
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


#include "gpu-compute/dispatcher.hh"

#include "cpu/base.hh"
#include "debug/GPUDisp.hh"
#include "gpu-compute/cl_driver.hh"
#include "gpu-compute/cl_event.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"
#include "mem/packet_access.hh"

GpuDispatcher *GpuDispatcher::instance = nullptr;

GpuDispatcher::GpuDispatcher(const Params *p)
    : DmaDevice(p), _masterId(p->system->getMasterId(this, "disp")),
      pioAddr(p->pio_addr), pioSize(4096), pioDelay(p->pio_latency),
      dispatchCount(0), dispatchActive(false), cpu(p->cpu),
      shader(p->shader_pointer), driver(p->cl_driver),
      tickEvent([this]{ exec(); }, "GPU Dispatcher tick",
                false, Event::CPU_Tick_Pri)
{
    shader->handshake(this);
    driver->handshake(this);

    ndRange.wg_disp_rem = false;
    ndRange.globalWgId = 0;

    schedule(&tickEvent, 0);

    // translation port for the dispatcher
    tlbPort = new TLBPort(csprintf("%s-port%d", name()), this);

    num_kernelLaunched
    .name(name() + ".num_kernel_launched")
    .desc("number of kernel launched")
    ;
}

GpuDispatcher *GpuDispatcherParams::create()
{
    GpuDispatcher *dispatcher = new GpuDispatcher(this);
    GpuDispatcher::setInstance(dispatcher);

    return GpuDispatcher::getInstance();
}

void
GpuDispatcher::serialize(CheckpointOut &cp) const
{
    Tick event_tick = 0;

    if (ndRange.wg_disp_rem)
        fatal("Checkpointing not supported during active workgroup execution");

    if (tickEvent.scheduled())
        event_tick = tickEvent.when();

    SERIALIZE_SCALAR(event_tick);

}

void
GpuDispatcher::unserialize(CheckpointIn &cp)
{
    Tick event_tick;

    if (tickEvent.scheduled())
        deschedule(&tickEvent);

    UNSERIALIZE_SCALAR(event_tick);

    if (event_tick)
        schedule(&tickEvent, event_tick);
}

AddrRangeList
GpuDispatcher::getAddrRanges() const
{
    AddrRangeList ranges;

    DPRINTF(GPUDisp, "dispatcher registering addr range at %#x size %#x\n",
            pioAddr, pioSize);

    ranges.push_back(RangeSize(pioAddr, pioSize));

    return ranges;
}

Tick
GpuDispatcher::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr);
    assert(pkt->getAddr() < pioAddr + pioSize);

    int offset = pkt->getAddr() - pioAddr;
    pkt->allocate();

    DPRINTF(GPUDisp, " read register %#x size=%d\n", offset, pkt->getSize());

    if (offset < 8) {
        assert(!offset);
        assert(pkt->getSize() == 8);

        uint64_t retval = dispatchActive;
        pkt->setLE(retval);
    } else {
        offset -= 8;
        assert(offset + pkt->getSize() < sizeof(HsaQueueEntry));
        char *curTaskPtr = (char*)&curTask;

        memcpy(pkt->getPtr<const void*>(), curTaskPtr + offset, pkt->getSize());
    }

    pkt->makeAtomicResponse();

    return pioDelay;
}

Tick
GpuDispatcher::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr);
    assert(pkt->getAddr() < pioAddr + pioSize);

    int offset = pkt->getAddr() - pioAddr;

#if TRACING_ON
    uint64_t data_val = 0;

    switch (pkt->getSize()) {
      case 1:
        data_val = pkt->getLE<uint8_t>();
        break;
      case 2:
        data_val = pkt->getLE<uint16_t>();
        break;
      case 4:
        data_val = pkt->getLE<uint32_t>();
        break;
      case 8:
        data_val = pkt->getLE<uint64_t>();
        break;
      default:
        DPRINTF(GPUDisp, "bad size %d\n", pkt->getSize());
    }

    DPRINTF(GPUDisp, "write register %#x value %#x size=%d\n", offset, data_val,
            pkt->getSize());
#endif
    if (!offset) {
        static int nextId = 0;

        // The depends field of the qstruct, which was previously unused, is
        // used to communicate with simulated application.
        if (curTask.depends) {
            HostState hs;
            shader->ReadMem((uint64_t)(curTask.depends), &hs,
                            sizeof(HostState), 0);

            // update event start time (in nano-seconds)
            uint64_t start = curTick() / 1000;

            shader->WriteMem((uint64_t)(&((_cl_event*)hs.event)->start),
                             &start, sizeof(uint64_t), 0);
        }

        // launch kernel
        ++num_kernelLaunched;

        NDRange *ndr = &(ndRangeMap[nextId]);
        // copy dispatch info
        ndr->q = curTask;

        // update the numDispTask polled by the runtime
        accessUserVar(cpu, (uint64_t)(curTask.numDispLeft), 0, 1);

        ndr->numWgTotal = 1;

        for (int i = 0; i < 3; ++i) {
            ndr->wgId[i] = 0;
            ndr->numWg[i] = divCeil(curTask.gdSize[i], curTask.wgSize[i]);
            ndr->numWgTotal *= ndr->numWg[i];
        }

        ndr->numWgCompleted = 0;
        ndr->globalWgId = 0;
        ndr->wg_disp_rem = true;
        ndr->execDone = false;
        ndr->addrToNotify = (volatile bool*)curTask.addrToNotify;
        ndr->numDispLeft = (volatile uint32_t*)curTask.numDispLeft;
        ndr->dispatchId = nextId;
        ndr->curCid = pkt->req->contextId();
        DPRINTF(GPUDisp, "launching kernel %d\n",nextId);
        execIds.push(nextId);
        ++nextId;

        dispatchActive = true;

        if (!tickEvent.scheduled()) {
            schedule(&tickEvent, curTick() + shader->ticks(1));
        }
    } else {
        // populate current task struct
        // first 64 bits are launch reg
        offset -= 8;
        assert(offset < sizeof(HsaQueueEntry));
        char *curTaskPtr = (char*)&curTask;
        memcpy(curTaskPtr + offset, pkt->getPtr<const void*>(), pkt->getSize());
    }

    pkt->makeAtomicResponse();

    return pioDelay;
}


Port &
GpuDispatcher::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "translation_port") {
        return *tlbPort;
    }

    return DmaDevice::getPort(if_name, idx);
}

void
GpuDispatcher::exec()
{
    int fail_count = 0;

    // There are potentially multiple outstanding kernel launches.
    // It is possible that the workgroups in a different kernel
    // can fit on the GPU even if another kernel's workgroups cannot
    DPRINTF(GPUDisp, "Launching %d Kernels\n", execIds.size());

    while (execIds.size() > fail_count) {
        int execId = execIds.front();

        while (ndRangeMap[execId].wg_disp_rem) {
            //update the thread context
            shader->updateContext(ndRangeMap[execId].curCid);

            // attempt to dispatch_workgroup
            if (!shader->dispatch_workgroups(&ndRangeMap[execId])) {
                // if we failed try the next kernel,
                // it may have smaller workgroups.
                // put it on the queue to rety latter
                DPRINTF(GPUDisp, "kernel %d failed to launch\n", execId);
                execIds.push(execId);
                ++fail_count;
                break;
            }
        }
        // let's try the next kernel_id
        execIds.pop();
    }

    DPRINTF(GPUDisp, "Returning %d Kernels\n", doneIds.size());

    if (doneIds.size() && cpu) {
        shader->hostWakeUp(cpu);
    }

    while (doneIds.size()) {
        // wakeup the CPU if any Kernels completed this cycle
        DPRINTF(GPUDisp, "WorkGroup %d completed\n", doneIds.front());
        doneIds.pop();
    }
}

void
GpuDispatcher::notifyWgCompl(Wavefront *w)
{
    int kern_id = w->kernId;
    DPRINTF(GPUDisp, "notify WgCompl %d\n",kern_id);
    assert(ndRangeMap[kern_id].dispatchId == kern_id);
    ndRangeMap[kern_id].numWgCompleted++;

    if (ndRangeMap[kern_id].numWgCompleted == ndRangeMap[kern_id].numWgTotal) {
        ndRangeMap[kern_id].execDone = true;
        doneIds.push(kern_id);

        if (ndRangeMap[kern_id].addrToNotify) {
            accessUserVar(cpu, (uint64_t)(ndRangeMap[kern_id].addrToNotify), 1,
                          0);
        }

        accessUserVar(cpu, (uint64_t)(ndRangeMap[kern_id].numDispLeft), 0, -1);

        // update event end time (in nano-seconds)
        if (ndRangeMap[kern_id].q.depends) {
            HostState *host_state = (HostState*)ndRangeMap[kern_id].q.depends;
            uint64_t event;
            shader->ReadMem((uint64_t)(&host_state->event), &event,
                            sizeof(uint64_t), 0);

            uint64_t end = curTick() / 1000;

            shader->WriteMem((uint64_t)(&((_cl_event*)event)->end), &end,
                             sizeof(uint64_t), 0);
        }
    }

    if (!tickEvent.scheduled()) {
        schedule(&tickEvent, curTick() + shader->ticks(1));
    }
}

void
GpuDispatcher::scheduleDispatch()
{
    if (!tickEvent.scheduled())
        schedule(&tickEvent, curTick() + shader->ticks(1));
}

void
GpuDispatcher::accessUserVar(BaseCPU *cpu, uint64_t addr, int val, int off)
{
    if (cpu) {
        if (off) {
            shader->AccessMem(addr, &val, sizeof(int), 0, MemCmd::ReadReq,
                              true);
            val += off;
        }

        shader->AccessMem(addr, &val, sizeof(int), 0, MemCmd::WriteReq, true);
    } else {
        panic("Cannot find host");
    }
}

// helper functions for driver to retrieve GPU attributes
int
GpuDispatcher::getNumCUs()
{
    return shader->cuList.size();
}

int
GpuDispatcher::wfSize() const
{
    return shader->cuList[0]->wfSize();
}

void
GpuDispatcher::setFuncargsSize(int funcargs_size)
{
    shader->funcargs_size = funcargs_size;
}

uint32_t
GpuDispatcher::getStaticContextSize() const
{
    return shader->cuList[0]->wfList[0][0]->getStaticContextSize();
}
