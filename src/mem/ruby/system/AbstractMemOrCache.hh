/*
 * Copyright (c) 2008 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_SYSTEM_ABSTRACTMEMORCACHE_HH__
#define __MEM_RUBY_SYSTEM_ABSTRACTMEMORCACHE_HH__

#include <iosfwd>

#include "mem/ruby/slicc_interface/Message.hh"

class Consumer;
class MemoryNode;
class Message;

class AbstractMemOrCache
{
  public:
    virtual ~AbstractMemOrCache() {};
    virtual void setConsumer(Consumer* consumer_ptr) = 0;
    virtual Consumer* getConsumer() = 0;

    virtual void enqueue (const MsgPtr& message, int latency) = 0;
    virtual void enqueueMemRef (MemoryNode& memRef) = 0;
    virtual void dequeue () = 0;
    virtual const Message* peek () = 0;
    virtual bool isReady () = 0;
    virtual MemoryNode peekNode () = 0;
    virtual bool areNSlotsAvailable (int n) = 0;
    virtual void printConfig (std::ostream& out) = 0;
    virtual void print (std::ostream& out) const = 0;
};

#endif // __MEM_RUBY_SYSTEM_ABSTRACTMEMORCACHE_HH__
