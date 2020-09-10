/*
 * Copyright (c) 2020 Inria
 * Copyright (c) 2016 Georgia Institute of Technology
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
 */


#include "mem/ruby/network/garnet/CrossbarSwitch.hh"

#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/OutputUnit.hh"
#include "mem/ruby/network/garnet/Router.hh"

CrossbarSwitch::CrossbarSwitch(Router *router)
  : Consumer(router), m_router(router), m_num_vcs(m_router->get_num_vcs()),
    m_crossbar_activity(0), switchBuffers(0)
{
}

void
CrossbarSwitch::init()
{
    switchBuffers.resize(m_router->get_num_inports());
}

/*
 * The wakeup function of the CrossbarSwitch loops through all input ports,
 * and sends the winning flit (from SA) out of its output port on to the
 * output link. The output link is scheduled for wakeup in the next cycle.
 */

void
CrossbarSwitch::wakeup()
{
    DPRINTF(RubyNetwork, "CrossbarSwitch at Router %d woke up "
            "at time: %lld\n",
            m_router->get_id(), m_router->curCycle());

    for (auto& switch_buffer : switchBuffers) {
        if (!switch_buffer.isReady(curTick())) {
            continue;
        }

        flit *t_flit = switch_buffer.peekTopFlit();
        if (t_flit->is_stage(ST_, curTick())) {
            int outport = t_flit->get_outport();

            // flit performs LT_ in the next cycle
            t_flit->advance_stage(LT_, m_router->clockEdge(Cycles(1)));
            t_flit->set_time(m_router->clockEdge(Cycles(1)));

            // This will take care of waking up the Network Link
            // in the next cycle
            m_router->getOutputUnit(outport)->insert_flit(t_flit);
            switch_buffer.getTopFlit();
            m_crossbar_activity++;
        }
    }
}

uint32_t
CrossbarSwitch::functionalWrite(Packet *pkt)
{
   uint32_t num_functional_writes = 0;

   for (auto& switch_buffer : switchBuffers) {
       num_functional_writes += switch_buffer.functionalWrite(pkt);
   }

   return num_functional_writes;
}

void
CrossbarSwitch::resetStats()
{
    m_crossbar_activity = 0;
}
