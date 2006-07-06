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
 * Authors: Ron Dreslinski
 *          Steve Reinhardt
 *          Ali Saidi
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
/** The request was an instruction read. */
const unsigned INST_READ        = 0x800;

class Request
{
  private:
    /**
     * The physical address of the request. Valid only if validPaddr
     * is set. */
    Addr paddr;

    /**
     * The size of the request. This field must be set when vaddr or
     * paddr is written via setVirt() or setPhys(), so it is always
     * valid as long as one of the address fields is valid.  */
    int size;

    /** Flag structure for the request. */
    uint32_t flags;

    /**
     * The time this request was started. Used to calculate
     * latencies. This field is set to curTick any time paddr or vaddr
     * is written.  */
    Tick time;

    /** The address space ID. */
    int asid;
    /** The virtual address of the request. */
    Addr vaddr;

    /** The return value of store conditional. */
    uint64_t scResult;

    /** The cpu number (for statistics, typically). */
    int cpuNum;
    /** The requesting thread id (for statistics, typically). */
    int  threadNum;

    /** program counter of initiating access; for tracing/debugging */
    Addr pc;

    /** Whether or not paddr is valid (has been written yet). */
    bool validPaddr;
    /** Whether or not the asid & vaddr are valid. */
    bool validAsidVaddr;
    /** Whether or not the sc result is valid. */
    bool validScResult;
    /** Whether or not the cpu number & thread ID are valid. */
    bool validCpuAndThreadNums;
    /** Whether or not the pc is valid. */
    bool validPC;

  public:
    /** Minimal constructor.  No fields are initialized. */
    Request()
        : validPaddr(false), validAsidVaddr(false),
          validScResult(false), validCpuAndThreadNums(false), validPC(false)
    {}

    /**
     * Constructor for physical (e.g. device) requests.  Initializes
     * just physical address, size, flags, and timestamp (to curTick).
     * These fields are adequate to perform a request.  */
    Request(Addr _paddr, int _size, int _flags)
        : validCpuAndThreadNums(false)
    { setPhys(_paddr, _size, _flags); }

    Request(int _asid, Addr _vaddr, int _size, int _flags, Addr _pc,
            int _cpuNum, int _threadNum)
    {
        setThreadContext(_cpuNum, _threadNum);
        setVirt(_asid, _vaddr, _size, _flags, _pc);
    }

    /**
     * Set up CPU and thread numbers. */
    void setThreadContext(int _cpuNum, int _threadNum)
    {
        cpuNum = _cpuNum;
        threadNum = _threadNum;
        validCpuAndThreadNums = true;
    }

    /**
     * Set up a physical (e.g. device) request in a previously
     * allocated Request object. */
    void setPhys(Addr _paddr, int _size, int _flags)
    {
        paddr = _paddr;
        size = _size;
        flags = _flags;
        time = curTick;
        validPaddr = true;
        validAsidVaddr = false;
        validPC = false;
        validScResult = false;
    }

    /**
     * Set up a virtual (e.g., CPU) request in a previously
     * allocated Request object. */
    void setVirt(int _asid, Addr _vaddr, int _size, int _flags, Addr _pc)
    {
        asid = _asid;
        vaddr = _vaddr;
        size = _size;
        flags = _flags;
        pc = _pc;
        time = curTick;
        validPaddr = false;
        validAsidVaddr = true;
        validPC = true;
        validScResult = false;
    }

    /** Set just the physical address.  This should only be used to
     * record the result of a translation, and thus the vaddr must be
     * valid before this method is called.  Otherwise, use setPhys()
     * to guarantee that the size and flags are also set.
     */
    void setPaddr(Addr _paddr)
    {
        assert(validAsidVaddr);
        paddr = _paddr;
        validPaddr = true;
    }

    /** Accessor for paddr. */
    Addr getPaddr() { assert(validPaddr); return paddr; }

    /** Accessor for size. */
    int getSize() { assert(validPaddr || validAsidVaddr); return size; }
    /** Accessor for time. */
    Tick getTime() { assert(validPaddr || validAsidVaddr); return time; }

    /** Accessor for flags. */
    uint32_t getFlags() { assert(validPaddr || validAsidVaddr); return flags; }
    /** Accessor for paddr. */
    void setFlags(uint32_t _flags)
    { assert(validPaddr || validAsidVaddr); flags = _flags; }

    /** Accessor function for vaddr.*/
    Addr getVaddr() { assert(validAsidVaddr); return vaddr; }

    /** Accessor function for asid.*/
    int getAsid() { assert(validAsidVaddr); return asid; }

    /** Accessor function to check if sc result is valid. */
    bool scResultValid() { return validScResult; }
    /** Accessor function for store conditional return value.*/
    uint64_t getScResult() { assert(validScResult); return scResult; }
    /** Accessor function for store conditional return value.*/
    void setScResult(uint64_t _scResult)
    { scResult = _scResult; validScResult = true; }

    /** Accessor function for cpu number.*/
    int getCpuNum() { assert(validCpuAndThreadNums); return cpuNum; }
    /** Accessor function for thread number.*/
    int getThreadNum()  { assert(validCpuAndThreadNums); return threadNum; }

    /** Accessor function for pc.*/
    Addr getPC() { assert(validPC); return pc; }

    /** Accessor Function to Check Cacheability. */
    bool isUncacheable() { return getFlags() & UNCACHEABLE; }

    bool isInstRead() { return getFlags() & INST_READ; }

    friend class Packet;
};

#endif // __MEM_REQUEST_HH__
