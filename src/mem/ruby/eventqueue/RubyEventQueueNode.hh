
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

#include <iostream>

#include "mem/ruby/common/Global.hh"
#include "sim/eventq.hh"
#include "mem/ruby/common/Consumer.hh"
//class Consumer;

class RubyEventQueueNode : public Event {
public:
  // Constructors
  RubyEventQueueNode(Consumer* _consumer) 
    : m_consumer_ptr(_consumer)
  { 
    setFlags(AutoDelete); 
  }

  // Destructor
  //~RubyEventQueueNode();

  // Public Methods
  void print(std::ostream& out) const;
  virtual void process() { m_consumer_ptr->wakeup(); }
  virtual const char *description() const { return "Ruby Event"; }

private:
  // Private Methods

  // Default copy constructor and assignment operator
  // RubyEventQueueNode(const RubyEventQueueNode& obj);

  // Data Members (m_ prefix)
  Consumer* m_consumer_ptr;
};

// Output operator declaration
std::ostream& operator<<(std::ostream& out, const RubyEventQueueNode& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
std::ostream& operator<<(std::ostream& out, const RubyEventQueueNode& obj)
{
  obj.print(out);
  out << std::flush;
  return out;
}

#endif //EVENTQUEUENODE_H
