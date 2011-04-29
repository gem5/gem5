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

/*
 * The actual modelled switch. It use the perfect switch and a
 * Throttle object to control and bandwidth and timing *only for the
 * output port*. So here we have un-realistic modelling, since the
 * order of PerfectSwitch and Throttle objects get woke up affect the
 * message timing. A more accurate model would be having two set of
 * system states, one for this cycle, one for next cycle. And on the
 * cycle boundary swap the two set of states.
 */

#ifndef __MEM_RUBY_NETWORK_SIMPLE_SWITCH_HH__
#define __MEM_RUBY_NETWORK_SIMPLE_SWITCH_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Global.hh"

class MessageBuffer;
class PerfectSwitch;
class NetDest;
class SimpleNetwork;
class Throttle;

class Switch
{
  public:
    Switch(SwitchID sid, SimpleNetwork* network_ptr);
    ~Switch();

    void addInPort(const std::vector<MessageBuffer*>& in);
    void addOutPort(const std::vector<MessageBuffer*>& out,
        const NetDest& routing_table_entry, int link_latency,
        int bw_multiplier);
    const Throttle* getThrottle(LinkID link_number) const;
    const std::vector<Throttle*>* getThrottles() const;
    void clearRoutingTables();
    void clearBuffers();
    void reconfigureOutPort(const NetDest& routing_table_entry);

    void printStats(std::ostream& out) const;
    void clearStats();
    void printConfig(std::ostream& out) const;

    void print(std::ostream& out) const;

  private:
    // Private copy constructor and assignment operator
    Switch(const Switch& obj);
    Switch& operator=(const Switch& obj);

    PerfectSwitch* m_perfect_switch_ptr;
    SimpleNetwork* m_network_ptr;
    std::vector<Throttle*> m_throttles;
    std::vector<MessageBuffer*> m_buffers_to_free;
    SwitchID m_switch_id;
};

inline std::ostream&
operator<<(std::ostream& out, const Switch& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_SIMPLE_SWITCH_HH__
