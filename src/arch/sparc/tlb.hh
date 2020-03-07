/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __ARCH_SPARC_TLB_HH__
#define __ARCH_SPARC_TLB_HH__

#include "arch/generic/tlb.hh"
#include "arch/sparc/asi.hh"
#include "arch/sparc/tlb_map.hh"
#include "base/logging.hh"
#include "mem/request.hh"
#include "params/SparcTLB.hh"

class ThreadContext;
class Packet;

namespace SparcISA
{

const Addr StartVAddrHole = ULL(0x0000800000000000);
const Addr EndVAddrHole = ULL(0xFFFF7FFFFFFFFFFF);
const Addr VAddrAMask = ULL(0xFFFFFFFF);
const Addr PAddrImplMask = ULL(0x000000FFFFFFFFFF);

class TLB : public BaseTLB
{
    // These faults need to be able to populate the tlb in SE mode.
    friend class FastInstructionAccessMMUMiss;
    friend class FastDataAccessMMUMiss;

    // TLB state
  protected:
    // Only used when this is the data TLB.
    uint64_t sfar;
    uint64_t c0_tsb_ps0;
    uint64_t c0_tsb_ps1;
    uint64_t c0_config;
    uint64_t cx_tsb_ps0;
    uint64_t cx_tsb_ps1;
    uint64_t cx_config;
    uint64_t sfsr;
    uint64_t tag_access;

  protected:
    TlbMap lookupTable;;
    typedef TlbMap::iterator MapIter;

    TlbEntry *tlb;

    int size;
    int usedEntries;
    int lastReplaced;

    uint64_t cacheState;
    bool cacheValid;

    std::list<TlbEntry*> freeList;

    enum FaultTypes {
        OtherFault = 0,
        PrivViolation = 0x1,
        SideEffect = 0x2,
        AtomicToIo = 0x4,
        IllegalAsi = 0x8,
        LoadFromNfo = 0x10,
        VaOutOfRange = 0x20,
        VaOutOfRangeJmp = 0x40
    };

    enum ContextType {
        Primary = 0,
        Secondary = 1,
        Nucleus = 2
    };

    enum TsbPageSize {
        Ps0,
        Ps1
    };
  public:
    /** lookup an entry in the TLB based on the partition id, and real bit if
     * real is true or the partition id, and context id if real is false.
     * @param va the virtual address not shifted (e.g. bottom 13 bits are 0)
     * @param paritition_id partition this entry is for
     * @param real is this a real->phys or virt->phys translation
     * @param context_id if this is virt->phys what context
     * @param update_used should ew update the used bits in the
     * entries on not useful if we are trying to do a va->pa without
     * mucking with any state for a debug read for example.
     * @return A pointer to a tlb entry
     */
    TlbEntry *lookup(Addr va, int partition_id, bool real, int context_id = 0,
            bool update_used = true);

    /** Remove all entries from the TLB */
    void flushAll() override;

  protected:
    /** Insert a PTE into the TLB. */
    void insert(Addr vpn, int partition_id, int context_id, bool real,
            const PageTableEntry& PTE, int entry = -1);

    /** Given an entry id, read that tlb entries' tag. */
    uint64_t TagRead(int entry);

    /** Remove all non-locked entries from the tlb that match partition id. */
    void demapAll(int partition_id);

    /** Remove all entries that match a given context/partition id. */
    void demapContext(int partition_id, int context_id);

    /** Remve all entries that match a certain partition id, (contextid), and
     * va). */
    void demapPage(Addr va, int partition_id, bool real, int context_id);

    /** Checks if the virtual address provided is a valid one. */
    bool validVirtualAddress(Addr va, bool am);

    void writeSfsr(bool write, ContextType ct,
            bool se, FaultTypes ft, int asi);

    void clearUsedBits();


    void writeTagAccess(Addr va, int context);

    Fault translateInst(const RequestPtr &req, ThreadContext *tc);
    Fault translateData(const RequestPtr &req, ThreadContext *tc, bool write);

  public:
    typedef SparcTLBParams Params;
    TLB(const Params *p);

    void takeOverFrom(BaseTLB *otlb) override {}

    void
    demapPage(Addr vaddr, uint64_t asn) override
    {
        panic("demapPage(Addr) is not implemented.\n");
    }

    void dumpAll();

    Fault translateAtomic(
            const RequestPtr &req, ThreadContext *tc, Mode mode) override;
    Fault translateFunctional(
            const RequestPtr &req, ThreadContext *tc, Mode mode) override;
    void translateTiming(
            const RequestPtr &req, ThreadContext *tc,
            Translation *translation, Mode mode) override;
    Fault finalizePhysical(
            const RequestPtr &req,
            ThreadContext *tc, Mode mode) const override;
    Cycles doMmuRegRead(ThreadContext *tc, Packet *pkt);
    Cycles doMmuRegWrite(ThreadContext *tc, Packet *pkt);
    void GetTsbPtr(ThreadContext *tc, Addr addr, int ctx, Addr *ptrs);

    // Checkpointing
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /** Give an entry id, read that tlb entries' tte */
    uint64_t TteRead(int entry);

  private:
    void writeSfsr(Addr a, bool write, ContextType ct,
            bool se, FaultTypes ft, int asi);

    uint64_t MakeTsbPtr(TsbPageSize ps, uint64_t tag_access, uint64_t c0_tsb,
        uint64_t c0_config, uint64_t cX_tsb, uint64_t cX_config);


    TlbEntry *cacheEntry[2];
    ASI cacheAsi[2];
};

}

#endif // __ARCH_SPARC_TLB_HH__
