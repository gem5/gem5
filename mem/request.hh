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

/**
 * @file Decleration of a request, the overall memory request consisting of
 the parts of the request that are persistent throughout the transaction.
 */

#ifndef __MEM_REQUEST_HH__
#define __MEM_REQUEST_HH__

#include "targetarch/isa_traits.hh"

class Request;
class CpuRequest;

typedef Request* RequestPtr;
typedef CpuRequest* CpuRequestPtr;

class Request
{
    //@todo Make Accesor functions, make these private.
  public:
    /** The physical address of the request. */
    Addr paddr;

    /** whether this req came from the CPU or not  **DO we need this??***/
    bool nicReq;

    /** The size of the request. */
    int size;

    /** The time this request was started. Used to calculate latencies. */
    Tick time;

    /** Destination address if this is a block copy. */
    Addr copyDest;
};

class CpuRequest : public Request
{
    //@todo Make Accesor functions, make these private.
  public:
    /** The virtual address of the request. */
    Addr vaddr;

    /** The address space ID. */
    int asid;

    /** The return value of store conditional. */
    uint64_t scResult;

    /** The cpu number for statistics. */
    int cpuNum;

    /** The requesting  thread id. */
    int  threadNum;

    /** program counter of initiating access; for tracing/debugging */
    Addr pc;
};

#endif // __MEM_REQUEST_HH__
