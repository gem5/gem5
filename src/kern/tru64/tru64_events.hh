/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *          Lisa Hsu
 */

#ifndef __TRU64_EVENTS_HH__
#define __TRU64_EVENTS_HH__

#include <string>

#include "cpu/pc_event.hh"
#include "kern/system_events.hh"

class ThreadContext;

class BadAddrEvent : public SkipFuncEvent
{
  public:
    BadAddrEvent(PCEventQueue *q, const std::string &desc, Addr addr)
        : SkipFuncEvent(q, desc, addr) {}
    virtual void process(ThreadContext *tc);
};

class PrintfEvent : public PCEvent
{
  public:
    PrintfEvent(PCEventQueue *q, const std::string &desc, Addr addr)
        : PCEvent(q, desc, addr) {}
    virtual void process(ThreadContext *tc);
};

class DebugPrintfEvent : public PCEvent
{
  private:
    bool raw;

  public:
    DebugPrintfEvent(PCEventQueue *q, const std::string &desc, Addr addr,
                     bool r = false)
        : PCEvent(q, desc, addr), raw(r) {}
    virtual void process(ThreadContext *tc);
};

class DebugPrintfrEvent : public DebugPrintfEvent
{
  public:
    DebugPrintfrEvent(PCEventQueue *q, const std::string &desc, Addr addr)
        : DebugPrintfEvent(q, desc, addr, true)
    {}
};

class DumpMbufEvent : public PCEvent
{
  public:
    DumpMbufEvent(PCEventQueue *q, const std::string &desc, Addr addr)
        : PCEvent(q, desc, addr) {}
    virtual void process(ThreadContext *tc);
};

#endif // __TRU64_EVENTS_HH__
