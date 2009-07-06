
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
 * $Id$
 *
 * Description: The class to implement bandwidth and latency throttle. An
 *              instance of consumer class that can be woke up. It is only used
 *              to control bandwidth at output port of a switch. And the
 *              throttle is added *after* the output port, means the message is
 *              put in the output port of the PerfectSwitch (a
 *              intermediateBuffers) first, then go through the Throttle.
 *
 */

#ifndef THROTTLE_H
#define THROTTLE_H

#include "mem/ruby/common/Global.hh"
#include "mem/gems_common/Vector.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/network/Network.hh"

class MessageBuffer;

class Throttle : public Consumer {
public:
  // Constructors
  Throttle(int sID, NodeID node, int link_latency, int link_bandwidth_multiplier);
  Throttle(NodeID node, int link_latency, int link_bandwidth_multiplier);

  // Destructor
  ~Throttle() {}

  // Public Methods
  void addLinks(const Vector<MessageBuffer*>& in_vec, const Vector<MessageBuffer*>& out_vec);
  void wakeup();
  bool broadcastBandwidthAvailable(int rand) const;

  void printStats(ostream& out) const;
  void clearStats();
  void printConfig(ostream& out) const;
  double getUtilization() const; // The average utilization (a percent) since last clearStats()
  int getLinkBandwidth() const { return RubySystem::getNetwork()->getEndpointBandwidth() * m_link_bandwidth_multiplier; }
  int getLatency() const { return m_link_latency; }

  const Vector<Vector<int> >& getCounters() const { return m_message_counters; }

  void clear();

  void print(ostream& out) const;

private:
  // Private Methods
  void init(NodeID node, int link_latency, int link_bandwidth_multiplier);
  void addVirtualNetwork(MessageBuffer* in_ptr, MessageBuffer* out_ptr);
  void linkUtilized(double ratio) { m_links_utilized += ratio; }

  // Private copy constructor and assignment operator
  Throttle(const Throttle& obj);
  Throttle& operator=(const Throttle& obj);

  // Data Members (m_ prefix)
  Vector<MessageBuffer*> m_in;
  Vector<MessageBuffer*> m_out;
  Vector<Vector<int> > m_message_counters;
  int m_vnets;
  Vector<int> m_units_remaining;
  int m_sID;
  NodeID m_node;
  int m_bash_counter;
  int m_bandwidth_since_sample;
  Time m_last_bandwidth_sample;
  int m_link_bandwidth_multiplier;
  int m_link_latency;
  int m_wakeups_wo_switch;

  // For tracking utilization
  Time m_ruby_start;
  double m_links_utilized;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Throttle& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Throttle& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //THROTTLE_H
