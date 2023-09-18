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
 */

#ifndef __PC_EVENT_HH__
#define __PC_EVENT_HH__

#include <vector>

#include "base/logging.hh"
#include "base/types.hh"

namespace gem5
{

class ThreadContext;
class PCEventQueue;
class System;
class PCEventScope;

class PCEvent
{
  protected:
    std::string description;
    PCEventScope *scope;
    Addr evpc;

  public:
    PCEvent(PCEventScope *q, const std::string &desc, Addr pc);

    virtual ~PCEvent() { if (scope) remove(); }

    // for DPRINTF
    virtual const std::string name() const { return description; }

    std::string descr() const { return description; }
    Addr pc() const { return evpc; }

    bool remove();
    virtual void process(ThreadContext *tc) = 0;
};

class PCEventScope
{
  public:
    virtual bool remove(PCEvent *event) = 0;
    virtual bool schedule(PCEvent *event) = 0;
};

class PCEventQueue : public PCEventScope
{
  protected:
    class MapCompare
    {
      public:
        bool
        operator()(PCEvent * const &l, PCEvent * const &r) const
        {
            return l->pc() < r->pc();
        }
        bool
        operator()(PCEvent * const &l, Addr pc) const
        {
            return l->pc() < pc;
        }
        bool
        operator()(Addr pc, PCEvent * const &r) const
        {
            return pc < r->pc();
        }
    };
    typedef std::vector<PCEvent *> Map;

  public:
    typedef Map::iterator iterator;
    typedef Map::const_iterator const_iterator;

  protected:
    typedef std::pair<iterator, iterator> range_t;
    typedef std::pair<const_iterator, const_iterator> const_range_t;

  protected:
    Map pcMap;

    bool doService(Addr pc, ThreadContext *tc);

  public:
    PCEventQueue();
    ~PCEventQueue();

    bool remove(PCEvent *event) override;
    bool schedule(PCEvent *event) override;
    bool service(Addr pc, ThreadContext *tc)
    {
        if (pcMap.empty())
            return false;

        return doService(pc, tc);
    }
    bool empty() const { return pcMap.empty(); }

    range_t equal_range(Addr pc);
    range_t equal_range(PCEvent *event) { return equal_range(event->pc()); }

    void dump() const;
};


inline
PCEvent::PCEvent(PCEventScope *s, const std::string &desc, Addr pc)
    : description(desc), scope(s), evpc(pc)
{
    scope->schedule(this);
}

inline bool
PCEvent::remove()
{
    if (!scope)
        panic("cannot remove an uninitialized event;");

    return scope->remove(this);
}

class BreakPCEvent : public PCEvent
{
  protected:
    bool remove;

  public:
    BreakPCEvent(PCEventScope *s, const std::string &desc, Addr addr,
                 bool del = false);
    virtual void process(ThreadContext *tc);
};

class PanicPCEvent : public PCEvent
{
  public:
    PanicPCEvent(PCEventScope *s, const std::string &desc, Addr pc);
    virtual void process(ThreadContext *tc);
};

} // namespace gem5

#endif // __PC_EVENT_HH__
