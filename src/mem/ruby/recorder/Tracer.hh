
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
 * Description: Controller class of the tracer. Can stop/start/playback
 *              the ruby cache requests trace.
 *
 */

#ifndef TRACER_H
#define TRACER_H

#include "mem/ruby/libruby_internal.hh"

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/protocol/CacheRequestType.hh"
#include "sim/sim_object.hh"

#include "params/RubyTracer.hh"

#include "gzstream.hh"

template <class TYPE> class PrioHeap;
class Address;
class TraceRecord;
class Sequencer;

class Tracer : public SimObject {
public:
  // Constructors
//  Tracer();
    typedef RubyTracerParams Params;
  Tracer(const Params *p);

  // Destructor
  ~Tracer();

  // Public Methods
  void startTrace(string filename);
  void stopTrace();
  bool traceEnabled() { return m_enabled; }
  void traceRequest(Sequencer* sequencer, 
                    const Address& data_addr, 
                    const Address& pc_addr, 
                    RubyRequestType type, 
                    Time time);

  void print(ostream& out) const;

  // Public Class Methods
  int playbackTrace(string filename);
  void init();
private:
  // Private Methods

  // Private copy constructor and assignment operator
  Tracer(const Tracer& obj);
  Tracer& operator=(const Tracer& obj);

  // Data Members (m_ prefix)
  ogzstream m_trace_file;
  bool m_enabled;

  //added by SS
  int m_warmup_length;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Tracer& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Tracer& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //TRACER_H
