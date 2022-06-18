/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
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

#include "mem/ruby/network/garnet/Credit.hh"

#include "base/trace.hh"
#include "debug/RubyNetwork.hh"

namespace gem5
{

namespace ruby
{

namespace garnet
{

// Credit Signal for buffers inside VC
// Carries m_vc (inherits from flit.hh)
// and m_is_free_signal (whether VC is free or not)

Credit::Credit(int vc, bool is_free_signal, Tick curTime)
    : flit(0, 0, vc, 0, RouteInfo(), 0, nullptr, 0, 0, curTime)
{
    m_is_free_signal = is_free_signal;
    m_type = CREDIT_;
}

flit *
Credit::serialize(int ser_id, int parts, uint32_t bWidth)
{
    DPRINTF(RubyNetwork, "Serializing a credit\n");
    bool new_free = false;
    if ((ser_id+1 == parts) && m_is_free_signal) {
        new_free = true;
    }
    Credit *new_credit_flit = new Credit(m_vc, new_free, m_time);
    return new_credit_flit;
}

flit *
Credit::deserialize(int des_id, int num_flits, uint32_t bWidth)
{
    DPRINTF(RubyNetwork, "DeSerializing a credit vc:%d free:%d\n",
    m_vc, m_is_free_signal);
    if (m_is_free_signal) {
        // We are not going to get anymore credits for this vc
        // So send a credit in any case
        return new Credit(m_vc, true, m_time);
    }

    return new Credit(m_vc, false, m_time);
}

void
Credit::print(std::ostream& out) const
{
    out << "[Credit:: ";
    out << "Type=" << m_type << " ";
    out << "VC=" << m_vc << " ";
    out << "FreeVC=" << m_is_free_signal << " ";
    out << "Set Time=" << m_time << " ";
    out << "]";
}

} // namespace garnet
} // namespace ruby
} // namespace gem5
