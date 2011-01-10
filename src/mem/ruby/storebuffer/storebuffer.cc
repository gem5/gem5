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

#include <cassert>
#include <cstdio>
#include <map>

#include "base/trace.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/storebuffer/storebuffer.hh"

using namespace std;

#define SYSTEM_EXIT assert(0)

// global map of request id_s to map them back to storebuffer pointers
map<uint64_t, StoreBuffer *> request_map;

void
hit(int64_t id)
{
    if (request_map.find(id) == request_map.end()) {
        panic("Request ID %d not found in the map\n", id);
    } else {
        request_map[id]->complete(id);
        request_map.erase(id);
    }
}

StoreBuffer::StoreBuffer(uint32 id, uint32 block_bits, int storebuffer_size)
{
    iseq = 0;
    tso_iseq = 0;
    char name [] = "Sequencer_";
    char port_name [13];
    sprintf(port_name, "%s%d", name, id);
    m_port = libruby_get_port(port_name, hit);
    m_hit_callback = NULL;
    assert(storebuffer_size >= 0);
    m_storebuffer_size = storebuffer_size;
    m_id = id;
    m_block_size = 1 << block_bits;
    m_block_mask = ~(m_block_size - 1);
    m_buffer_size = 0;
    m_use_storebuffer = false;
    m_storebuffer_full = false;
    m_storebuffer_flushing = false;
    m_stalled_issue = true;
    if (m_storebuffer_size > 0){
        m_use_storebuffer = true;
    }
}

StoreBuffer::~StoreBuffer()
{
}

void
StoreBuffer::registerHitCallback(void (*hit_callback)(int64_t request_id))
{
    assert(m_hit_callback == NULL); // can't assign hit_callback twice
    m_hit_callback = hit_callback;
}


void
StoreBuffer::addToStoreBuffer(RubyRequest request)
{
    if (!m_use_storebuffer) {
        // make request to libruby
        uint64_t id = libruby_issue_request(m_port, request);
        if (request_map.find(id) != request_map.end()) {
            panic("Request ID: %d is already in the map\n", id);
        } else {
            request_map.insert(make_pair(id, this));
            outstanding_requests.insert(make_pair(id, request));
        }
        return;
    }


    buffer.push_front(SBEntry(request, NULL));

    m_buffer_size++;

    if (m_buffer_size >= m_storebuffer_size) {
        m_storebuffer_full = true;
    } else if (m_stalled_issue) {
        m_stalled_issue = false;
        issueNextStore();
    }

    iseq++;
}


// Return value of -2 indicates that the load request was satisfied by
// the store buffer
// Return value of -3 indicates a partial match, so the load has to
// retry until NO_MATCH
// Alternatively we could satisfy the partial match, but tso gets
// complicated and more races
int64_t
StoreBuffer::handleLoad(RubyRequest request)
{
    if (!m_use_storebuffer) {
        // make a request to ruby
        return libruby_issue_request(m_port, request);
    }

    load_match match = checkForLoadHit(request);
    if (match == FULL_MATCH) {
        // fill data
        returnMatchedData(request);
        iseq++;
        return -2;
    } else if (match == NO_MATCH) {
        // make request to libruby and return the id
        uint64_t id = libruby_issue_request(m_port, request);
        if (request_map.find(id) != request_map.end()) {
            panic("Request ID: %d is already in the map\n", id);
        } else {
            request_map.insert(make_pair(id, this));
            outstanding_requests.insert(make_pair(id, request));
        }
        iseq++;
        return id;
    } else { // partial match
        return -3;
    }
}

// This function will fill the data array if any match is found
load_match
StoreBuffer::checkForLoadHit(RubyRequest request)
{
    if (!m_use_storebuffer) {
        // this function should never be called if we are not using a
        // store buffer
        panic("checkForLoadHit called while write buffer is not in use\n");
    }

    physical_address_t physical_address = request.paddr;
    int len = request.len;

    uint8_t * data = new uint8_t[64];
    memset(data, 0, 64);
    for (int i = physical_address % 64; i < len; i++)
        data[i] = 1;

    bool found = false;
    physical_address_t lineaddr = physical_address & m_block_mask;

    // iterate over the buffer looking for hits
    deque<SBEntry>::iterator it = buffer.begin();
    for (; it != buffer.end(); it++) {
        RubyRequest &req = it->m_request;
        if ((req.paddr & m_block_mask) != lineaddr)
            continue;

        found = true;
        for (int i = req.paddr % 64; i < req.len; i++)
            data[i] = 0;
    }

    // if any matching entry is found, determine if all the
    // requested bytes have been matched
    if (found) {
        assert(m_buffer_size > 0);
        int unmatched_bytes = 0;
        for (int i = physical_address%64; i < len; i++) {
            unmatched_bytes = unmatched_bytes + data[i];
        }
        if (unmatched_bytes == 0) {
            delete data;
            return FULL_MATCH;
        } else {
            delete data;
            return PARTIAL_MATCH;
        }
    } else {
        delete data;
        return NO_MATCH;
    }
}

void
StoreBuffer::returnMatchedData(RubyRequest request)
{
    if (!m_use_storebuffer) {
        panic("returnMatchedData called while write buffer is not in use\n");
    }

    uint8_t * data = new uint8_t[64];
    memset(data, 0, 64);
    uint8_t * written = new uint8_t[64];
    memset(written, 0, 64);

    physical_address_t physical_address = request.paddr;
    int len = request.len;

    assert(checkForLoadHit(request) != NO_MATCH);
    physical_address_t lineaddr = physical_address & m_block_mask;
    bool found = false;
    deque<SBEntry>::iterator satisfying_store;
    deque<SBEntry>::iterator it = buffer.begin();
    for (; it != buffer.end(); it++) {
        if ((it->m_request.paddr & m_block_mask) == lineaddr) {
            if (!found) {
                found = true;
            }
            uint8_t * dataPtr = it->m_request.data;
            int offset = it->m_request.paddr%64;
            for (int i = offset; i < it->m_request.len; i++) {
                if (!written[i]) { // don't overwrite data with earlier data
                    data[i] = dataPtr[i-offset];
                    written[i] = 1;
                }
            }
        }
    }

    int i = physical_address%64;
    for (int j = 0; (i < physical_address%64 + len) && (j < len); i++, j++) {
        if (written[i]) {
            request.data[j] = data[i];
        }
    }

    delete data;
    delete written;
}

void
StoreBuffer::flushStoreBuffer()
{
    if (!m_use_storebuffer) {
        // do nothing
        return;
    }

    m_storebuffer_flushing = (m_buffer_size > 0);
}

void
StoreBuffer::issueNextStore()
{
    SBEntry request = buffer.back();
    uint64_t id = libruby_issue_request(m_port, request.m_request);
    if (request_map.find(id) != request_map.end()) {
        assert(0);
    } else {
        request_map.insert(make_pair(id, this));
        outstanding_requests.insert(make_pair(id, request.m_request));
    }
}

void
StoreBuffer::complete(uint64_t id)
{
    if (!m_use_storebuffer) {
        m_hit_callback(id);
        return;
    }

    assert(outstanding_requests.find(id) != outstanding_requests.end());
    physical_address_t physical_address =
        outstanding_requests.find(id)->second.paddr;
    RubyRequestType type = outstanding_requests.find(id)->second.type;

    if (type == RubyRequestType_ST) {
        physical_address_t lineaddr = physical_address & m_block_mask;

        // Note fastpath hits are handled like regular requests - they
        // must remove the WB entry!
        if (lineaddr != physical_address) {
            warn("error: StoreBuffer: ruby returns pa 0x%0llx "
                      "which is not a cache line: 0x%0llx\n",
                      physical_address, lineaddr);
        }

        SBEntry from_buffer = buffer.back();
        if ((from_buffer.m_request.paddr & m_block_mask) == lineaddr &&
                from_buffer.m_request.type == type) {
            buffer.pop_back();
            m_buffer_size--;
            assert(m_buffer_size >= 0);

            // schedule the next request
            if (m_buffer_size > 0) {
                issueNextStore();
            } else if (m_buffer_size == 0) {
                m_storebuffer_flushing = false;
                m_stalled_issue = true;
            }

            m_storebuffer_full = false;
        } else {
            panic("[%d] error: StoreBuffer: at complete, address 0x%0llx "
                 "not found.\n"
                 "StoreBuffer:: complete FAILS\n",
                 m_id, lineaddr);
        }

    } else if (type == RubyRequestType_LD) {
        m_hit_callback(id);
    }

    // LD, ST or FETCH hit callback
    outstanding_requests.erase(id);
}

void
StoreBuffer::print()
{
    DPRINTF(RubyStorebuffer, "[%d] StoreBuffer: Total entries: %d "
            "Outstanding: %d\n",
            m_id, m_storebuffer_size, m_buffer_size);

    if (!m_use_storebuffer)
        DPRINTF(RubyStorebuffer, "\t WRITE BUFFER NOT USED\n");
}
