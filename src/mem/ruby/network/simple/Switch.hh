
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
 * Description: The actual modelled switch. It use the perfect switch and a
 *              Throttle object to control and bandwidth and timing *only for
 *              the output port*. So here we have un-realistic modelling,
 *              since the order of PerfectSwitch and Throttle objects get
 *              woke up affect the message timing. A more accurate model would
 *              be having two set of system states, one for this cycle, one for
 *              next cycle. And on the cycle boundary swap the two set of
 *              states.
 *
 */

#ifndef Switch_H
#define Switch_H

#include "Global.hh"
#include "Vector.hh"

class MessageBuffer;
class PerfectSwitch;
class NetDest;
class SimpleNetwork;
class Throttle;

class Switch {
public:
  // Constructors

  // constructor specifying the number of ports
  Switch(SwitchID sid, SimpleNetwork* network_ptr);
  void addInPort(const Vector<MessageBuffer*>& in);
  void addOutPort(const Vector<MessageBuffer*>& out, const NetDest& routing_table_entry, int link_latency, int bw_multiplier);
  const Throttle* getThrottle(LinkID link_number) const;
  const Vector<Throttle*>* getThrottles() const;
  void clearRoutingTables();
  void clearBuffers();
  void reconfigureOutPort(const NetDest& routing_table_entry);

  void printStats(ostream& out) const;
  void clearStats();
  void printConfig(ostream& out) const;

  // Destructor
  ~Switch();

  void print(ostream& out) const;
private:

  // Private copy constructor and assignment operator
  Switch(const Switch& obj);
  Switch& operator=(const Switch& obj);

  // Data Members (m_ prefix)
  PerfectSwitch* m_perfect_switch_ptr;
  Vector<Throttle*> m_throttles;
  Vector<MessageBuffer*> m_buffers_to_free;
  SwitchID m_switch_id;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Switch& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Switch& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //Switch_H
