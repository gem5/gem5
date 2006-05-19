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

#include "arch/isa_traits.hh"

class Request;

typedef Request* RequestPtr;

/** The request is a Load locked/store conditional. */
const unsigned LOCKED		= 0x001;
/** The virtual address is also the physical address. */
const unsigned PHYSICAL		= 0x002;
/** The request is an ALPHA VPTE pal access (hw_ld). */
const unsigned VPTE		= 0x004;
/** Use the alternate mode bits in ALPHA. */
const unsigned ALTMODE		= 0x008;
/** The request is to an uncacheable address. */
const unsigned UNCACHEABLE	= 0x010;
/** The request should not cause a page fault. */
const unsigned NO_FAULT         = 0x020;
/** The request should be prefetched into the exclusive state. */
const unsigned PF_EXCLUSIVE	= 0x100;
/** The request should be marked as LRU. */
const unsigned EVICT_NEXT	= 0x200;
/** The request should ignore unaligned access faults */
const unsigned NO_ALIGN_FAULT   = 0x400;

class Request
{
    //@todo Make Accesor functions, make these private.
  public:
    /** Constructor, needs a bool to signify if it is/isn't Cpu Request. */
    Request(bool isCpu);

    /** reset the request to it's initial state so it can be reused.*/
    void resetAll(bool isCpu);

    /** reset the request's addrs times, etc, so but not everything to same
     * time. */
    void resetMin();

//First non-cpu request fields
  private:
    /** The physical address of the request. */
    Addr paddr;
    /** Wether or not paddr is valid (has been written yet). */
    bool validPaddr;

    /** The size of the request. */
    int size;
    /** Wether or not size is valid (has been written yet). */
    bool validSize;

    /** The time this request was started. Used to calculate latencies. */
    Tick time;
    /** Wether or not time is valid (has been written yet). */
    bool validTime;

    /** Destination address if this is a block copy. */
    Addr copyDest;
    /** Wether or not copyDest is valid (has been written yet). */
    bool validCopyDest;

    /** Flag structure for the request. */
    uint32_t flags;

//Accsesors for non-cpu request fields
  public:
    /** Accesor for paddr. */
    Addr getPaddr();
    /** Accesor for paddr. */
    void setPaddr(Addr _paddr);

    /** Accesor for size. */
    int getSize();
    /** Accesor for size. */
    void setSize(int _size);

    /** Accesor for time. */
    Tick getTime();
    /** Accesor for time. */
    void setTime(Tick _time);

    /** Accesor for copy dest. */
    Addr getCopyDest();
    /** Accesor for copy dest. */
    void setCopyDest(Addr _copyDest);

    /** Accesor for flags. */
    uint32_t getFlags();
    /** Accesor for paddr. */
    void setFlags(uint32_t _flags);

//Now cpu-request fields
  private:
    /** Bool to signify if this is a cpuRequest. */
    bool cpuReq;

    /** The virtual address of the request. */
    Addr vaddr;
    /** Wether or not the vaddr is valid. */
    bool validVaddr;

    /** The address space ID. */
    int asid;
    /** Wether or not the asid is valid. */
    bool validAsid;

    /** The return value of store conditional. */
    uint64_t scResult;
    /** Wether or not the sc result is valid. */
    bool validScResult;

    /** The cpu number for statistics. */
    int cpuNum;
    /** Wether or not the cpu number is valid. */
    bool validCpuNum;

    /** The requesting  thread id. */
    int  threadNum;
    /** Wether or not the thread id is valid. */
    bool validThreadNum;

    /** program counter of initiating access; for tracing/debugging */
    Addr pc;
    /** Wether or not the pc is valid. */
    bool validPC;

//Accessor Functions for cpu request fields
  public:
    /** Accesor function to determine if this is a cpu request or not.*/
    bool isCpuRequest();

    /** Accesor function for vaddr.*/
    Addr getVaddr();
    /** Accesor function for vaddr.*/
    void setVaddr(Addr _vaddr);

    /** Accesor function for asid.*/
    int getAsid();
    /** Accesor function for asid.*/
    void setAsid(int _asid);

    /** Accesor function for store conditional return value.*/
    uint64_t getScResult();
    /** Accesor function for store conditional return value.*/
    void setScResult(uint64_t _scResult);

    /** Accesor function for cpu number.*/
    int getCpuNum();
    /** Accesor function for cpu number.*/
    void setCpuNum(int _cpuNum);

    /** Accesor function for thread number.*/
    int getThreadNum();
    /** Accesor function for thread number.*/
    void setThreadNum(int _threadNum);

    /** Accesor function for pc.*/
    Addr getPC();
    /** Accesor function for pc.*/
    void setPC(Addr _pc);

};

#endif // __MEM_REQUEST_HH__
