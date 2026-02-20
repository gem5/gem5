/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#include "salam/dma_write_fifo.hh"

DmaWriteFifo::DmaWriteFifo(DmaPort &_port, size_t size, unsigned max_req_size,
                           unsigned max_pending, Request::Flags flags)
    : maxReqSize(max_req_size),
      fifoSize(size),
      reqFlags(flags),
      port(_port),
      buffer(size),
      nextAddr(0),
      endAddr(0)
{
    freeRequests.resize(max_pending);
    for (auto &e : freeRequests) {
        e.reset(new DmaDoneEvent(this, max_req_size));
    }
}

DmaWriteFifo::~DmaWriteFifo()
{
    for (auto &p : pendingRequests) {
        DmaDoneEvent *e(p.release());

        if (e->done()) {
            delete e;
        } else {
            // We can't kill in-flight DMAs, so we'll just transfer
            // ownership to the event queue so that they get freed
            // when they are done.
            e->kill();
        }
    }
}

void
DmaWriteFifo::serialize(CheckpointOut &cp) const
{
    assert(pendingRequests.empty());

    SERIALIZE_CONTAINER(buffer);
    SERIALIZE_SCALAR(endAddr);
    SERIALIZE_SCALAR(nextAddr);
}

void
DmaWriteFifo::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_CONTAINER(buffer);
    UNSERIALIZE_SCALAR(endAddr);
    UNSERIALIZE_SCALAR(nextAddr);
}

bool
DmaWriteFifo::canFill(size_t len)
{
    if ((buffer.size() + len) <= fifoSize) {
        return true;
    } else {
        return false;
    }
}

bool
DmaWriteFifo::tryFill(uint8_t *src, size_t len)
{
    if ((buffer.size() + len) <= fifoSize) {
        buffer.write(src, len);
        resumeEmpty();
        return true;
    } else {
        return false;
    }
}

void
DmaWriteFifo::fill(uint8_t *src, size_t len)
{
    const bool success(tryFill(src, len));
    panic_if(!success, "Buffer overrun in DmaWriteFifo::fill()\n");
}

void
DmaWriteFifo::startEmpty(Addr start, size_t size)
{
    assert(atEndOfBlock());

    nextAddr = start;
    endAddr = start + size;
    resumeEmpty();
}

void
DmaWriteFifo::stopEmpty()
{
    // Prevent new DMA requests by setting the next address to the end
    // address. Pending requests will still complete.
    nextAddr = endAddr;

    // Flag in-flight accesses as canceled. This prevents their data
    // from being written to the FIFO.
    for (auto &p : pendingRequests) {
        p->cancel();
    }
}

void
DmaWriteFifo::resumeEmpty()
{
    // Don't try to fetch more data if we are draining. This ensures
    // that the DMA engine settles down before we checkpoint it.
    if (drainState() == DrainState::Draining) {
        return;
    }

    const bool old_eob(atEndOfBlock());

    if (port.sys->bypassCaches()) {
        resumeEmptyFunctional();
    } else {
        resumeEmptyTiming();
    }

    if (!old_eob && atEndOfBlock()) {
        onEndOfBlock();
    }
}

void
DmaWriteFifo::resumeEmptyFunctional()
{
    const size_t fifo_space = buffer.capacity() - buffer.size();
    const size_t kvm_watermark = port.sys->cacheLineSize();
    if (fifo_space >= kvm_watermark || buffer.capacity() < kvm_watermark) {
        const size_t block_remaining = endAddr - nextAddr;
        const size_t xfer_size = std::min(fifo_space, block_remaining);
        std::vector<uint8_t> tmp_buffer(xfer_size);

        assert(pendingRequests.empty());
        DPRINTF(DMA,
                "KVM Bypassing startAddr=%#x xfer_size=%#x "
                "fifo_space=%#x block_remaining=%#x\n",
                nextAddr, xfer_size, fifo_space, block_remaining);

        port.sys->physProxy.readBlob(nextAddr, tmp_buffer.data(), xfer_size);
        buffer.write(tmp_buffer.begin(), xfer_size);
        nextAddr += xfer_size;
    }
}

void
DmaWriteFifo::resumeEmptyTiming()
{
    size_t size_pending(0);
    for (auto &e : pendingRequests) {
        size_pending += e->requestSize();
    }

    while (!freeRequests.empty() && !atEndOfBlock()) {
        const size_t req_size(std::min(maxReqSize, endAddr - nextAddr));
        if ((int64_t)(buffer.size() - size_pending - req_size) < 0) {
            break;
        }

        DmaDoneEventUPtr event(std::move(freeRequests.front()));
        freeRequests.pop_front();
        assert(event);

        event->reset(req_size);
        buffer.read(event->data(), req_size);
        port.dmaAction(MemCmd::WriteReq, nextAddr, req_size, event.get(),
                       event->data(), 0, reqFlags);
        nextAddr += req_size;
        size_pending += req_size;

        pendingRequests.emplace_back(std::move(event));
    }
}

void
DmaWriteFifo::dmaDone()
{
    const bool old_active(isActive());

    handlePending();
    resumeEmpty();

    if (old_active && !isActive()) {
        onIdle();
    }
}

void
DmaWriteFifo::handlePending()
{
    while (!pendingRequests.empty() && pendingRequests.front()->done()) {
        // Get the first finished pending request
        DmaDoneEventUPtr event(std::move(pendingRequests.front()));
        pendingRequests.pop_front();

        if (event->canceled()) {
            buffer.write(event->data(), event->requestSize());
        }

        // Move the event to the list of free requests
        freeRequests.emplace_back(std::move(event));
    }

    if (pendingRequests.empty()) {
        signalDrainDone();
    }
}

DrainState
DmaWriteFifo::drain()
{
    return pendingRequests.empty() ? DrainState::Drained
                                   : DrainState::Draining;
}

DmaWriteFifo::DmaDoneEvent::DmaDoneEvent(DmaWriteFifo *_parent,
                                         size_t max_size)
    : parent(_parent), _done(false), _canceled(false), _data(max_size, 0)
{}

void
DmaWriteFifo::DmaDoneEvent::kill()
{
    parent = nullptr;
    setFlags(AutoDelete);
}

void
DmaWriteFifo::DmaDoneEvent::cancel()
{
    _canceled = true;
}

void
DmaWriteFifo::DmaDoneEvent::reset(size_t size)
{
    assert(size <= _data.size());
    _done = false;
    _canceled = false;
    _requestSize = size;
}

void
DmaWriteFifo::DmaDoneEvent::process()
{
    if (!parent) {
        return;
    }

    assert(!_done);
    _done = true;
    parent->dmaDone();
}
