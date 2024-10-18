/*
 * Copyright (c) 2010-2013, 2016, 2019-2022, 2024 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 */

#ifndef __ARCH_ARM_TLB_HH__
#define __ARCH_ARM_TLB_HH__


#include "arch/arm/faults.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/utility.hh"
#include "arch/generic/tlb.hh"
#include "base/statistics.hh"
#include "enums/TypeTLB.hh"
#include "mem/request.hh"
#include "params/ArmTLB.hh"
#include "sim/probe/pmu.hh"

namespace gem5
{

class ThreadContext;

namespace ArmISA {

class TableWalker;
class TLB;
class TLBIOp;

class TlbTestInterface
{
  public:
    TlbTestInterface() {}
    virtual ~TlbTestInterface() {}

    /**
     * Check if a TLB translation should be forced to fail.
     *
     * @param req Request requiring a translation.
     * @param is_priv Access from a privileged mode (i.e., not EL0)
     * @param mode Access type
     * @param domain Domain type
     */
    virtual Fault translationCheck(const RequestPtr &req, bool is_priv,
                                   BaseMMU::Mode mode,
                                   DomainType domain) = 0;

    /**
     * Check if a page table walker access should be forced to fail.
     *
     * @param req walk request bearing a valid phys address
     * @param va Virtual address that initiated the walk
     * @param is_secure Access from secure state
     * @param is_priv Access from a privileged mode (i.e., not EL0)
     * @param mode Access type
     * @param domain Domain type
     * @param lookup_level Page table walker level
     */
    virtual Fault walkCheck(const RequestPtr &walk_req,
                            Addr va, bool is_secure,
                            Addr is_priv, BaseMMU::Mode mode,
                            DomainType domain,
                            enums::ArmLookupLevel lookup_level) = 0;
};

class TLB : public BaseTLB
{
  protected:
    TlbEntry* table;

    /** TLB Size */
    int size;

    /** Indicates this TLB caches IPA->PA translations */
    bool isStage2;

    /**
     * Hash map containing one entry per lookup level
     * The TLB is caching partial translations from the key lookup level
     * if the matching value is true.
     */
    std::unordered_map<enums::ArmLookupLevel, bool> partialLevels;

    /**
     * True if the TLB caches partial translations
     */
    bool _walkCache;

    TableWalker *tableWalker;

    struct TlbStats : public statistics::Group
    {
        TlbStats(TLB &parent);

        const TLB &tlb;

        // Access Stats
        mutable statistics::Scalar partialHits;
        mutable statistics::Scalar instHits;
        mutable statistics::Scalar instMisses;
        mutable statistics::Scalar readHits;
        mutable statistics::Scalar readMisses;
        mutable statistics::Scalar writeHits;
        mutable statistics::Scalar writeMisses;
        mutable statistics::Scalar inserts;
        mutable statistics::Scalar flushTlb;
        mutable statistics::Scalar flushedEntries;

        statistics::Formula readAccesses;
        statistics::Formula writeAccesses;
        statistics::Formula instAccesses;
        statistics::Formula hits;
        statistics::Formula misses;
        statistics::Formula accesses;
    } stats;

    /** PMU probe for TLB refills */
    probing::PMUUPtr ppRefills;

    int rangeMRU; //On lookup, only move entries ahead when outside rangeMRU
    vmid_t vmid;

  public:
    using Params = ArmTLBParams;
    using Lookup = TlbEntry::Lookup;
    using LookupLevel = enums::ArmLookupLevel;

    TLB(const Params &p);
    TLB(const Params &p, int _size, TableWalker *_walker);

    /** Lookup an entry in the TLB
     * @return pointer to TLB entry if it exists
     */
    TlbEntry *lookup(const Lookup &lookup_data);

    /** Lookup an entry in the TLB and in the next levels by
     * following the nextLevel pointer
     *
     * @param mode to differentiate between read/writes/fetches.
     * @return pointer to TLB entry if it exists
     */
    TlbEntry *multiLookup(const Lookup &lookup_data);

    virtual ~TLB();

    void takeOverFrom(BaseTLB *otlb) override;

    void setTableWalker(TableWalker *table_walker);

    TableWalker *getTableWalker() { return tableWalker; }

    int getsize() const { return size; }

    bool walkCache() const { return _walkCache; }

    void setVMID(vmid_t _vmid) { vmid = _vmid; }

    /** Insert a PTE in the current TLB */
    void insert(TlbEntry &pte);

    /** Insert a PTE in the current TLB and in the higher levels */
    void multiInsert(TlbEntry &pte);

    /** Reset the entire TLB. Used for CPU switching to prevent stale
     * translations after multiple switches
     */
    void flushAll() override;


    /** Flush TLB entries
     */
    void flush(const TLBIOp &tlbi_op);

    void printTlb() const;

    void demapPage(Addr vaddr, uint64_t asn) override
    {
        // needed for x86 only
        panic("demapPage() is not implemented.\n");
    }

    Fault
    translateAtomic(const RequestPtr &req, ThreadContext *tc,
                    BaseMMU::Mode mode) override
    {
        panic("unimplemented");
    }

    void
    translateTiming(const RequestPtr &req, ThreadContext *tc,
                    BaseMMU::Translation *translation,
                    BaseMMU::Mode mode) override
    {
        panic("unimplemented");
    }

    Fault
    finalizePhysical(const RequestPtr &req, ThreadContext *tc,
                     BaseMMU::Mode mode) const override
    {
        panic("unimplemented");
    }

    void regProbePoints() override;

    /**
     * Get the table walker port. This is used for migrating
     * port connections during a CPU takeOverFrom() call. For
     * architectures that do not have a table walker, NULL is
     * returned, hence the use of a pointer rather than a
     * reference. For ARM this method will always return a valid port
     * pointer.
     *
     * @return A pointer to the walker request port
     */
    Port *getTableWalkerPort() override;

    // Caching misc register values here.
    // Writing to misc registers needs to invalidate them.
    // translateFunctional/translateSe/translateFs checks if they are
    // invalid and call updateMiscReg if necessary.

  private:
    /** Remove any entries that match both a va and asn
     * @param mva virtual address to flush
     * @param asn contextid/asn to flush on match
     * @param secure_lookup if the operation affects the secure world
     * @param ignore_asn if the flush should ignore the asn
     * @param in_host if hcr.e2h == 1 and hcr.tge == 1 for VHE.
     * @param entry_type type of entry to flush (instruction/data/unified)
     */
    void _flushMva(Addr mva, uint64_t asn, bool secure_lookup,
                   bool ignore_asn, ExceptionLevel target_el,
                   bool in_host, TypeTLB entry_type);

    /** Check if the tlb entry passed as an argument needs to
     * be "promoted" as a unified entry:
     * this should happen if we are hitting an instruction TLB entry on a
     * data access or a data TLB entry on an instruction access:
     */
    void checkPromotion(TlbEntry *entry, BaseMMU::Mode mode);

    /** Helper function looking up for a matching TLB entry
     * Does not update stats; see lookup method instead */
    TlbEntry *match(const Lookup &lookup_data);
};

} // namespace ArmISA
} // namespace gem5

#endif // __ARCH_ARM_TLB_HH__
