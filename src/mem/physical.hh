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
 * Authors: Ron Dreslinski
 */

/* @file
 */

#ifndef __PHYSICAL_MEMORY_HH__
#define __PHYSICAL_MEMORY_HH__

#include <map>
#include <string>

#include "base/range.hh"
#include "base/statistics.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/tport.hh"
#include "params/PhysicalMemory.hh"
#include "sim/eventq.hh"
#include "sim/stats.hh"

//
// Functional model for a contiguous block of physical memory. (i.e. RAM)
//
class PhysicalMemory : public MemObject
{
  protected:

    class MemoryPort : public SimpleTimingPort
    {
        PhysicalMemory *memory;

      public:

        MemoryPort(const std::string &_name, PhysicalMemory *_memory);

      protected:

        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);

        virtual void recvRangeChange();

        virtual AddrRangeList getAddrRanges();

        virtual unsigned deviceBlockSize() const;
    };

    int numPorts;


  private:
    // prevent copying of a MainMemory object
    PhysicalMemory(const PhysicalMemory &specmem);
    const PhysicalMemory &operator=(const PhysicalMemory &specmem);

  protected:

    class LockedAddr {
      public:
        // on alpha, minimum LL/SC granularity is 16 bytes, so lower
        // bits need to masked off.
        static const Addr Addr_Mask = 0xf;

        static Addr mask(Addr paddr) { return (paddr & ~Addr_Mask); }

        Addr addr;      // locked address
        int contextId;     // locking hw context

        // check for matching execution context
        bool matchesContext(Request *req)
        {
            return (contextId == req->contextId());
        }

        LockedAddr(Request *req)
            : addr(mask(req->getPaddr())),
              contextId(req->contextId())
        {
        }
        // constructor for unserialization use
        LockedAddr(Addr _addr, int _cid)
            : addr(_addr), contextId(_cid)
        {
        }
    };

    std::list<LockedAddr> lockedAddrList;

    // helper function for checkLockedAddrs(): we really want to
    // inline a quick check for an empty locked addr list (hopefully
    // the common case), and do the full list search (if necessary) in
    // this out-of-line function
    bool checkLockedAddrList(PacketPtr pkt);

    // Record the address of a load-locked operation so that we can
    // clear the execution context's lock flag if a matching store is
    // performed
    void trackLoadLocked(PacketPtr pkt);

    // Compare a store address with any locked addresses so we can
    // clear the lock flag appropriately.  Return value set to 'false'
    // if store operation should be suppressed (because it was a
    // conditional store and the address was no longer locked by the
    // requesting execution context), 'true' otherwise.  Note that
    // this method must be called on *all* stores since even
    // non-conditional stores must clear any matching lock addresses.
    bool writeOK(PacketPtr pkt) {
        Request *req = pkt->req;
        if (lockedAddrList.empty()) {
            // no locked addrs: nothing to check, store_conditional fails
            bool isLLSC = pkt->isLLSC();
            if (isLLSC) {
                req->setExtraData(0);
            }
            return !isLLSC; // only do write if not an sc
        } else {
            // iterate over list...
            return checkLockedAddrList(pkt);
        }
    }

    uint8_t *pmemAddr;
    Tick lat;
    Tick lat_var;
    std::vector<MemoryPort*> ports;
    typedef std::vector<MemoryPort*>::iterator PortIterator;

    uint64_t _size;
    uint64_t _start;

    /** Number of total bytes read from this memory */
    Stats::Scalar bytesRead;
    /** Number of instruction bytes read from this memory */
    Stats::Scalar bytesInstRead;
    /** Number of bytes written to this memory */
    Stats::Scalar bytesWritten;
    /** Number of read requests */
    Stats::Scalar numReads;
    /** Number of write requests */
    Stats::Scalar numWrites;
    /** Number of other requests */
    Stats::Scalar numOther;
    /** Read bandwidth from this memory */
    Stats::Formula bwRead;
    /** Read bandwidth from this memory */
    Stats::Formula bwInstRead;
    /** Write bandwidth from this memory */
    Stats::Formula bwWrite;
    /** Total bandwidth from this memory */
    Stats::Formula bwTotal;

  public:
    uint64_t size() { return _size; }
    uint64_t start() { return _start; }

  public:
    typedef PhysicalMemoryParams Params;
    PhysicalMemory(const Params *p);
    virtual ~PhysicalMemory();

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

  public:
    unsigned deviceBlockSize() const;
    AddrRangeList getAddrRanges();
    virtual Port *getPort(const std::string &if_name, int idx = -1);
    void virtual init();
    unsigned int drain(Event *de);

  protected:
    Tick doAtomicAccess(PacketPtr pkt);
    void doFunctionalAccess(PacketPtr pkt);
    virtual Tick calculateLatency(PacketPtr pkt);

  public:
     /**
     * Register Statistics
     */
    void regStats();

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

};

#endif //__PHYSICAL_MEMORY_HH__
