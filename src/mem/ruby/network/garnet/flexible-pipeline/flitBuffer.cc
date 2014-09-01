/*
 * Copyright (c) 2008 Princeton University
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
 *
 * Authors: Niket Agarwal
 */

#include <algorithm>

#include "mem/ruby/network/garnet/flexible-pipeline/flitBuffer.hh"

using namespace std;

flitBuffer::flitBuffer()
{
    max_size = INFINITE_;
}

flitBuffer::flitBuffer(int maximum_size)
{
    max_size = maximum_size;
}

bool
flitBuffer::isEmpty()
{
    return (m_buffer.size() == 0);
}

bool
flitBuffer::isReady(Cycles curTime)
{
    if (m_buffer.size() != 0 ) {
        flit *t_flit = m_buffer.front();
        if (t_flit->get_time() <= curTime)
            return true;
    }
    return false;
}

bool
flitBuffer::isFull()
{
    return (m_buffer.size() >= max_size);
}

void
flitBuffer::setMaxSize(int maximum)
{
    max_size = maximum;
}

flit*
flitBuffer:: getTopFlit()
{
    flit *f = m_buffer.front();
    pop_heap(m_buffer.begin(), m_buffer.end(), flit::greater);
    m_buffer.pop_back();
    return f;
}

flit*
flitBuffer::peekTopFlit()
{
    return m_buffer.front();
}

void
flitBuffer::insert(flit *flt)
{
    m_buffer.push_back(flt);
    push_heap(m_buffer.begin(), m_buffer.end(), flit::greater);
}

void
flitBuffer::print(std::ostream& out) const
{
    out << "[flitBuffer: ";
    out << m_buffer.size() << "] " << std::endl;
}

bool
flitBuffer::functionalRead(Packet *pkt)
{
    for (unsigned int i = 0; i < m_buffer.size(); ++i) {
        if (m_buffer[i]->functionalRead(pkt)) {
            return true;
        }
    }
    return false;
}

uint32_t
flitBuffer::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;

    for (unsigned int i = 0; i < m_buffer.size(); ++i) {
        if (m_buffer[i]->functionalWrite(pkt)) {
            num_functional_writes++;
        }
    }
    return num_functional_writes;
}
