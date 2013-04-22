/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __PC_EVENT_HH__
#define __PC_EVENT_HH__

#include <vector>

#include "base/misc.hh"
#include "base/types.hh"

class ThreadContext;
class PCEventQueue;
class System;

class PCEvent
{
  protected:
    std::string description;
    PCEventQueue *queue;
    Addr evpc;

  public:
    PCEvent(PCEventQueue *q, const std::string &desc, Addr pc);

    virtual ~PCEvent() { if (queue) remove(); }

    // for DPRINTF
    virtual const std::string name() const { return description; }

    std::string descr() const { return description; }
    Addr pc() const { return evpc; }

    bool remove();
    virtual void process(ThreadContext *tc) = 0;
};

class PCEventQueue
{
  protected:
    typedef PCEvent * record_t;
    class MapCompare {
      public:
        bool operator()(const record_t &l, const record_t &r) const {
            return l->pc() < r->pc();
        }
        bool operator()(const record_t &l, Addr pc) const {
            return l->pc() < pc;
        }
        bool operator()(Addr pc, const record_t &r) const {
            return pc < r->pc();
        }
    };
    typedef std::vector<record_t> map_t;

  public:
    typedef map_t::iterator iterator;
    typedef map_t::const_iterator const_iterator;

  protected:
    typedef std::pair<iterator, iterator> range_t;
    typedef std::pair<const_iterator, const_iterator> const_range_t;

  protected:
    map_t pc_map;

    bool doService(ThreadContext *tc);

  public:
    PCEventQueue();
    ~PCEventQueue();

    bool remove(PCEvent *event);
    bool schedule(PCEvent *event);
    bool service(ThreadContext *tc)
    {
        if (pc_map.empty())
            return false;

        return doService(tc);
    }

    range_t equal_range(Addr pc);
    range_t equal_range(PCEvent *event) { return equal_range(event->pc()); }

    void dump() const;
};


inline
PCEvent::PCEvent(PCEventQueue *q, const std::string &desc, Addr pc)
    : description(desc), queue(q), evpc(pc)
{
    queue->schedule(this);
}

inline bool
PCEvent::remove()
{
    if (!queue)
        panic("cannot remove an uninitialized event;");

    return queue->remove(this);
}

class BreakPCEvent : public PCEvent
{
  protected:
    bool remove;

  public:
    BreakPCEvent(PCEventQueue *q, const std::string &desc, Addr addr,
                 bool del = false);
    virtual void process(ThreadContext *tc);
};

void sched_break_pc_sys(System *sys, Addr addr);

void sched_break_pc(Addr addr);

class PanicPCEvent : public PCEvent
{
  public:
    PanicPCEvent(PCEventQueue *q, const std::string &desc, Addr pc);
    virtual void process(ThreadContext *tc);
};

#endif // __PC_EVENT_HH__
