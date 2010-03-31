/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>

#include "mem/ruby/storebuffer/stb_interface.hh"

StoreBuffer *
createNewSTB(uint32 id, uint32 block_bits, int storebuffer_size)
{
    StoreBuffer *stb = new StoreBuffer(id, block_bits, storebuffer_size);
    return stb;
}

storebuffer_status_t
handleStore(StoreBuffer *storebuffer, const RubyRequest &request)
{
    assert(request.type == RubyRequestType_ST);
    if (storebuffer->storeBufferFull()){
        return WB_FULL;
    } else if (storebuffer->storeBufferFlushing()) {
        return WB_FLUSHING;
    } else {
        storebuffer->addToStoreBuffer(request);
        return WB_OK;
    }
}

uint64_t
handleLoad(StoreBuffer *storebuffer, const RubyRequest &request)
{
    assert(request.type == RubyRequestType_LD);
    return storebuffer->handleLoad(request);
}

#if 0
uint64_t
handleAtomic(StoreBuffer *storebuffer, const RubyRequest &request)
{
    // flush the store buffer
    storebuffer->flushStoreBuffer();
    // let storebuffer issue atomic
    // return storebuffer->issueAtomic(request);
}
#endif

void
flushSTB(StoreBuffer *storebuffer)
{
    // in in-order can't get a request to flushSTB if already flushing
    // on out of order, have to check if already flushing
    storebuffer->flushStoreBuffer();
}

void
registerHitCallback(StoreBuffer *storebuffer,
        void (*hit_callback)(int64_t access_id))
{
    storebuffer->registerHitCallback(hit_callback);
}
