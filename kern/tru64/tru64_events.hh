/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __TRU64_EVENTS_HH__
#define __TRU64_EVENTS_HH__

#include <string>

#include "cpu/pc_event.hh"

class ExecContext;

#ifdef FS_MEASURE
class System;
#endif

class SkipFuncEvent : public PCEvent
{
  public:
    SkipFuncEvent(PCEventQueue *q, const std::string &desc)
        : PCEvent(q, desc) {}
    virtual void process(ExecContext *xc);
};

class BadAddrEvent : public SkipFuncEvent
{
  public:
    BadAddrEvent(PCEventQueue *q, const std::string &desc)
        : SkipFuncEvent(q, desc) {}
    virtual void process(ExecContext *xc);
};

class PrintfEvent : public PCEvent
{
  public:
    PrintfEvent(PCEventQueue *q, const std::string &desc)
        : PCEvent(q, desc) {}
    virtual void process(ExecContext *xc);
};

class DebugPrintfEvent : public PCEvent
{
  private:
    bool raw;

  public:
    DebugPrintfEvent(PCEventQueue *q, const std::string &desc, bool r = false)
        : PCEvent(q, desc), raw(r) {}
    virtual void process(ExecContext *xc);
};

class DumpMbufEvent : public PCEvent
{
  public:
    DumpMbufEvent(PCEventQueue *q, const std::string &desc)
        : PCEvent(q, desc) {}
    virtual void process(ExecContext *xc);
};

#ifdef FS_MEASURE
class FnEvent : public PCEvent
{
  public:
    FnEvent(PCEventQueue *q, const std::string &desc, System *system);
    virtual void process(ExecContext *xc);
    std::string myname() const { return _name; }

  private:
    std::string _name;
    Statistics::GenBin *myBin;
};
#endif //FS_MEASURE
#endif // __TRU64_EVENTS_HH__
