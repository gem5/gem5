/*
 * Copyright (c) 2015-2018 Advanced Micro Devices, Inc.
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
 * Authors: Sooraj Puthoor
 *          Michael LeBeane
 *          Eric van Tassell
 *          Anthony Gutierrez
 */

#include "dev/hsa/hsa_device.hh"

#include "base/chunk_generator.hh"
#include "sim/process.hh"

HSAPacketProcessor&
HSADevice::hsaPacketProc()
{
    return *hsaPP;
}

void
HSADevice::dmaReadVirt(Addr host_addr, unsigned size,
                             DmaCallback *cb, void *data, Tick delay)
{
    dmaVirt(&DmaDevice::dmaRead, host_addr, size, cb, data, delay);
}

void
HSADevice::dmaWriteVirt(Addr host_addr, unsigned size,
                              DmaCallback *cb, void *data, Tick delay)
{
    dmaVirt(&DmaDevice::dmaWrite, host_addr, size, cb, data, delay);
}

void
HSADevice::dmaVirt(DmaFnPtr dmaFn, Addr addr, unsigned size,
                           DmaCallback *cb, void *data, Tick delay)
{
    if (size == 0) {
        if (cb)
            schedule(cb->getChunkEvent(), curTick() + delay);
        return;
    }

    // move the buffer data pointer with the chunks
    uint8_t *loc_data = (uint8_t*)data;

    for (ChunkGenerator gen(addr, size, PAGE_SIZE); !gen.done(); gen.next()) {
        Addr phys;

        // translate pages into their corresponding frames
        translateOrDie(gen.addr(), phys);

        Event *event = cb ? cb->getChunkEvent() : nullptr;

        (this->*dmaFn)(phys, gen.size(), event, loc_data, delay);

        loc_data += gen.size();
    }
}

/**
 * HSADevices will perform DMA operations on VAs, and because
 * page faults are not currently supported for HSADevices, we
 * must be able to find the pages mapped for the process.
 */
void
HSADevice::translateOrDie(Addr vaddr, Addr &paddr)
{
    /**
     * Grab the process and try to translate the virtual address with it;
     * with new extensions, it will likely be wrong to just arbitrarily
     * grab context zero.
     */
    auto process = sys->threads[0]->getProcessPtr();

    if (!process->pTable->translate(vaddr, paddr)) {
        fatal("failed translation: vaddr 0x%x\n", vaddr);
    }
}
