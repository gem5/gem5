
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
 * Description: This is the virtual base class of all classes that can
 * be the targets of wakeup events.  There is only two methods,
 * wakeup() and print() and no data members.
 *
 */

#ifndef CONSUMER_H
#define CONSUMER_H

#include "Global.hh"
#include "EventQueue.hh"

class MessageBuffer;

class Consumer {
public:
  // Constructors
  Consumer() { m_last_scheduled_wakeup = 0; m_last_wakeup = 0; m_out_link_vec.setSize(0); }

  // Destructor
  virtual ~Consumer() { }

  // Public Methods - pure virtual methods
  void triggerWakeup() { Time time = g_eventQueue_ptr->getTime(); if (m_last_wakeup != time) { wakeup(); m_last_wakeup = time; }}
  virtual void wakeup() = 0;
  virtual void print(ostream& out) const = 0;
  const Time& getLastScheduledWakeup() const { return m_last_scheduled_wakeup; }
  void setLastScheduledWakeup(const Time& time) { m_last_scheduled_wakeup = time; }
  Vector< Vector<MessageBuffer*> > getOutBuffers() { return m_out_link_vec; }

protected:
  Vector< Vector<MessageBuffer*> > m_out_link_vec;

private:
  // Private Methods

  // Data Members (m_ prefix)
  Time m_last_scheduled_wakeup;
  Time m_last_wakeup;
};

// Output operator declaration
inline extern
ostream& operator<<(ostream& out, const Consumer& obj);

// ******************* Definitions *******************

// Output operator definition
inline extern
ostream& operator<<(ostream& out, const Consumer& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //CONSUMER_H
