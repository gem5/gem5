/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Daniel Sanchez
 */

#ifndef __RUBY_MEMORY_HH__
#define __RUBY_MEMORY_HH__

#include <map>
#include <vector>

#include "base/callback.hh"
#include "mem/packet.hh"
#include "mem/physical.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "params/RubyMemory.hh"

class RubyMemory : public PhysicalMemory
{
  public:
    std::vector<RubyPort *> ruby_ports;
    class Port : public MemoryPort
    {
        friend void ruby_hit_callback(int64_t req_id);

        RubyMemory *ruby_mem;

      public:
        Port(const std::string &_name, RubyMemory *_memory);
        void sendTiming(PacketPtr pkt);

      protected:
        virtual bool recvTiming(PacketPtr pkt);
    };

    class RubyEvent : public Event
    {
        RubyMemory *ruby_ptr;
      public:
        RubyEvent(RubyMemory *p)
            : Event(), ruby_ptr(p) {}

        virtual void process() { ruby_ptr->tick(); }

        virtual const char *description() const { return "ruby tick"; }
    };

    struct SenderState : public Packet::SenderState
    {
        Port *port;
        Packet::SenderState *saved;

        SenderState(Port *p, Packet::SenderState *s = NULL)
            : port(p), saved(s)
        {}
    };

  private:
    // prevent copying of a RubyMemory object
    RubyMemory(const RubyMemory &specmem);
    const RubyMemory &operator=(const RubyMemory &specmem);

    RubyEvent* rubyTickEvent;

  public:
    typedef RubyMemoryParams Params;
    RubyMemory(const Params *p);
    virtual ~RubyMemory();

    const Params *
    params() const
    {
        return safe_cast<const Params *>(_params);
    }

  public:
    virtual ::Port *getPort(const std::string &if_name, int idx = -1);
    void virtual init();

    //Ruby-related specifics
    void printConfigStats(); //dsm: Maybe this function should
                             //disappear once the configuration
                             //options change & M5 determines the
                             //stats file to use

    void hitCallback(PacketPtr pkt, Port *port);

    void printStats(std::ostream & out) const;
    void clearStats();
    void printConfig(std::ostream & out) const;

    void tick();

  private:
    Tick ruby_clock;
    Tick ruby_phase;

  public:
    static std::map<int64_t, PacketPtr> pending_requests;
};

void ruby_hit_callback(int64_t);

class RubyExitCallback : public Callback
{
  private:
   RubyMemory* ruby;

  public:
    /**
     * virtualize the destructor to make sure that the correct one
     * gets called.
     */

    virtual ~RubyExitCallback() {};

    RubyExitCallback(RubyMemory* rm) {ruby=rm;};

    /**
     * virtual process function that is invoked when the callback
     * queue is executed.
     */
    virtual void process() {ruby->printConfigStats();  /*delete ruby; was doing double delete...*/};
};


#endif //__RUBY_MEMORY_HH__

