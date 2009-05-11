
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
 */

#ifndef RUBYEVENTQUEUENODE_H
#define RUBYEVENTQUEUENODE_H

#include "Global.hh"
class Consumer;

class RubyEventQueueNode {
public:
  // Constructors
  RubyEventQueueNode() { m_time = 0; m_consumer_ptr = NULL; }

  // Destructor
  //~RubyEventQueueNode();

  // Public Methods
  void print(ostream& out) const;

  // Assignment operator and copy constructor since the default
  // constructors confuse purify when long longs are present.
  RubyEventQueueNode& operator=(const RubyEventQueueNode& obj) {
    m_time = obj.m_time;
    m_consumer_ptr = obj.m_consumer_ptr;
    return *this;
  }

  RubyEventQueueNode(const RubyEventQueueNode& obj) {
    m_time = obj.m_time;
    m_consumer_ptr = obj.m_consumer_ptr;
  }
private:
  // Private Methods

  // Default copy constructor and assignment operator
  // RubyEventQueueNode(const RubyEventQueueNode& obj);

  // Data Members (m_ prefix)
public:
  Time m_time;
  Consumer* m_consumer_ptr;
};

// Output operator declaration
ostream& operator<<(ostream& out, const RubyEventQueueNode& obj);

// ******************* Definitions *******************

inline extern bool node_less_then_eq(const RubyEventQueueNode& n1, const RubyEventQueueNode& n2);

inline extern
bool node_less_then_eq(const RubyEventQueueNode& n1, const RubyEventQueueNode& n2)
{
  return (n1.m_time <= n2.m_time);
}

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const RubyEventQueueNode& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //EVENTQUEUENODE_H
