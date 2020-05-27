/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Lisa Hsu
 */

#include "gpu-compute/gpu_tlb.hh"

#include <cmath>
#include <cstring>

#include "arch/x86/faults.hh"
#include "arch/x86/insts/microldstop.hh"
#include "arch/x86/pagetable.hh"
#include "arch/x86/pagetable_walker.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/regs/msr.hh"
#include "arch/x86/x86_traits.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/GPUPrefetch.hh"
#include "debug/GPUTLB.hh"
#include "mem/packet_access.hh"
#include "mem/page_table.hh"
#include "mem/request.hh"
#include "sim/process.hh"
#include "sim/pseudo_inst.hh"

namespace X86ISA
{

    GpuTLB::GpuTLB(const Params *p)
        : ClockedObject(p), configAddress(0), size(p->size),
          cleanupEvent([this]{ cleanup(); }, name(), false,
                       Event::Maximum_Pri),
          exitEvent([this]{ exitCallback(); }, name())
    {
        assoc = p->assoc;
        assert(assoc <= size);
        numSets = size/assoc;
        allocationPolicy = p->allocationPolicy;
        hasMemSidePort = false;
        accessDistance = p->accessDistance;
        clock = p->clk_domain->clockPeriod();

        tlb.assign(size, TlbEntry());

        freeList.resize(numSets);
        entryList.resize(numSets);

        for (int set = 0; set < numSets; ++set) {
            for (int way = 0; way < assoc; ++way) {
                int x = set * assoc + way;
                freeList[set].push_back(&tlb.at(x));
            }
        }

        FA = (size == assoc);

        /**
         * @warning: the set-associative version assumes you have a
         * fixed page size of 4KB.
         * If the page size is greather than 4KB (as defined in the
         * TheISA::PageBytes), then there are various issues w/ the current
         * implementation (you'd have the same 8KB page being replicated in
         * different sets etc)
         */
        setMask = numSets - 1;

        maxCoalescedReqs = p->maxOutstandingReqs;

        // Do not allow maxCoalescedReqs to be more than the TLB associativity
        if (maxCoalescedReqs > assoc) {
            maxCoalescedReqs = assoc;
            cprintf("Forcing maxCoalescedReqs to %d (TLB assoc.) \n", assoc);
        }

        outstandingReqs = 0;
        hitLatency = p->hitLatency;
        missLatency1 = p->missLatency1;
        missLatency2 = p->missLatency2;

        // create the slave ports based on the number of connected ports
        for (size_t i = 0; i < p->port_slave_connection_count; ++i) {
            cpuSidePort.push_back(new CpuSidePort(csprintf("%s-port%d",
                                  name(), i), this, i));
        }

        // create the master ports based on the number of connected ports
        for (size_t i = 0; i < p->port_master_connection_count; ++i) {
            memSidePort.push_back(new MemSidePort(csprintf("%s-port%d",
                                  name(), i), this, i));
        }
    }

    // fixme: this is never called?
    GpuTLB::~GpuTLB()
    {
        // make sure all the hash-maps are empty
        assert(translationReturnEvent.empty());
    }

    Port &
    GpuTLB::getPort(const std::string &if_name, PortID idx)
    {
        if (if_name == "slave") {
            if (idx >= static_cast<PortID>(cpuSidePort.size())) {
                panic("TLBCoalescer::getPort: unknown index %d\n", idx);
            }

            return *cpuSidePort[idx];
        } else if (if_name == "master") {
            if (idx >= static_cast<PortID>(memSidePort.size())) {
                panic("TLBCoalescer::getPort: unknown index %d\n", idx);
            }

            hasMemSidePort = true;

            return *memSidePort[idx];
        } else {
            panic("TLBCoalescer::getPort: unknown port %s\n", if_name);
        }
    }

    TlbEntry*
    GpuTLB::insert(Addr vpn, TlbEntry &entry)
    {
        TlbEntry *newEntry = nullptr;

        /**
         * vpn holds the virtual page address
         * The least significant bits are simply masked
         */
        int set = (vpn >> TheISA::PageShift) & setMask;

        if (!freeList[set].empty()) {
            newEntry = freeList[set].front();
            freeList[set].pop_front();
        } else {
            newEntry = entryList[set].back();
            entryList[set].pop_back();
        }

        *newEntry = entry;
        newEntry->vaddr = vpn;
        entryList[set].push_front(newEntry);

        return newEntry;
    }

    GpuTLB::EntryList::iterator
    GpuTLB::lookupIt(Addr va, bool update_lru)
    {
        int set = (va >> TheISA::PageShift) & setMask;

        if (FA) {
            assert(!set);
        }

        auto entry = entryList[set].begin();
        for (; entry != entryList[set].end(); ++entry) {
            int page_size = (*entry)->size();

            if ((*entry)->vaddr <= va && (*entry)->vaddr + page_size > va) {
                DPRINTF(GPUTLB, "Matched vaddr %#x to entry starting at %#x "
                        "with size %#x.\n", va, (*entry)->vaddr, page_size);

                if (update_lru) {
                    entryList[set].push_front(*entry);
                    entryList[set].erase(entry);
                    entry = entryList[set].begin();
                }

                break;
            }
        }

        return entry;
    }

    TlbEntry*
    GpuTLB::lookup(Addr va, bool update_lru)
    {
        int set = (va >> TheISA::PageShift) & setMask;

        auto entry = lookupIt(va, update_lru);

        if (entry == entryList[set].end())
            return nullptr;
        else
            return *entry;
    }

    void
    GpuTLB::invalidateAll()
    {
        DPRINTF(GPUTLB, "Invalidating all entries.\n");

        for (int i = 0; i < numSets; ++i) {
            while (!entryList[i].empty()) {
                TlbEntry *entry = entryList[i].front();
                entryList[i].pop_front();
                freeList[i].push_back(entry);
            }
        }
    }

    void
    GpuTLB::setConfigAddress(uint32_t addr)
    {
        configAddress = addr;
    }

    void
    GpuTLB::invalidateNonGlobal()
    {
        DPRINTF(GPUTLB, "Invalidating all non global entries.\n");

        for (int i = 0; i < numSets; ++i) {
            for (auto entryIt = entryList[i].begin();
                 entryIt != entryList[i].end();) {
                if (!(*entryIt)->global) {
                    freeList[i].push_back(*entryIt);
                    entryList[i].erase(entryIt++);
                } else {
                    ++entryIt;
                }
            }
        }
    }

    void
    GpuTLB::demapPage(Addr va, uint64_t asn)
    {

        int set = (va >> TheISA::PageShift) & setMask;
        auto entry = lookupIt(va, false);

        if (entry != entryList[set].end()) {
            freeList[set].push_back(*entry);
            entryList[set].erase(entry);
        }
    }



    namespace
    {

    Cycles
    localMiscRegAccess(bool read, MiscRegIndex regNum,
                       ThreadContext *tc, PacketPtr pkt)
    {
        if (read) {
            RegVal data = htole(tc->readMiscReg(regNum));
            // Make sure we don't trot off the end of data.
            pkt->setData((uint8_t *)&data);
        } else {
            RegVal data = htole(tc->readMiscRegNoEffect(regNum));
            tc->setMiscReg(regNum, letoh(data));
        }
        return Cycles(1);
    }

    } // anonymous namespace

    Fault
    GpuTLB::translateInt(bool read, const RequestPtr &req, ThreadContext *tc)
    {
        DPRINTF(GPUTLB, "Addresses references internal memory.\n");
        Addr vaddr = req->getVaddr();
        Addr prefix = (vaddr >> 3) & IntAddrPrefixMask;

        if (prefix == IntAddrPrefixCPUID) {
            panic("CPUID memory space not yet implemented!\n");
        } else if (prefix == IntAddrPrefixMSR) {
            vaddr = (vaddr >> 3) & ~IntAddrPrefixMask;

            MiscRegIndex regNum;
            if (!msrAddrToIndex(regNum, vaddr))
                return std::make_shared<GeneralProtection>(0);

            req->setLocalAccessor(
                [read,regNum](ThreadContext *tc, PacketPtr pkt)
                {
                    return localMiscRegAccess(read, regNum, tc, pkt);
                }
            );

            return NoFault;
        } else if (prefix == IntAddrPrefixIO) {
            // TODO If CPL > IOPL or in virtual mode, check the I/O permission
            // bitmap in the TSS.

            Addr IOPort = vaddr & ~IntAddrPrefixMask;
            // Make sure the address fits in the expected 16 bit IO address
            // space.
            assert(!(IOPort & ~0xFFFF));
            if (IOPort == 0xCF8 && req->getSize() == 4) {
                req->setLocalAccessor(
                    [read](ThreadContext *tc, PacketPtr pkt)
                    {
                        return localMiscRegAccess(
                                read, MISCREG_PCI_CONFIG_ADDRESS, tc, pkt);
                    }
                );
            } else if ((IOPort & ~mask(2)) == 0xCFC) {
                req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
                Addr configAddress =
                    tc->readMiscRegNoEffect(MISCREG_PCI_CONFIG_ADDRESS);
                if (bits(configAddress, 31, 31)) {
                    req->setPaddr(PhysAddrPrefixPciConfig |
                            mbits(configAddress, 30, 2) |
                            (IOPort & mask(2)));
                } else {
                    req->setPaddr(PhysAddrPrefixIO | IOPort);
                }
            } else {
                req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
                req->setPaddr(PhysAddrPrefixIO | IOPort);
            }
            return NoFault;
        } else {
            panic("Access to unrecognized internal address space %#x.\n",
                  prefix);
        }
    }

    /**
     * TLB_lookup will only perform a TLB lookup returning true on a TLB hit
     * and false on a TLB miss.
     * Many of the checks about different modes have been converted to
     * assertions, since these parts of the code are not really used.
     * On a hit it will update the LRU stack.
     */
    bool
    GpuTLB::tlbLookup(const RequestPtr &req,
                      ThreadContext *tc, bool update_stats)
    {
        bool tlb_hit = false;
    #ifndef NDEBUG
        uint32_t flags = req->getFlags();
        int seg = flags & SegmentFlagMask;
    #endif

        assert(seg != SEGMENT_REG_MS);
        Addr vaddr = req->getVaddr();
        DPRINTF(GPUTLB, "TLB Lookup for vaddr %#x.\n", vaddr);
        HandyM5Reg m5Reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);

        if (m5Reg.prot) {
            DPRINTF(GPUTLB, "In protected mode.\n");
            // make sure we are in 64-bit mode
            assert(m5Reg.mode == LongMode);

            // If paging is enabled, do the translation.
            if (m5Reg.paging) {
                DPRINTF(GPUTLB, "Paging enabled.\n");
                //update LRU stack on a hit
                TlbEntry *entry = lookup(vaddr, true);

                if (entry)
                    tlb_hit = true;

                if (!update_stats) {
                    // functional tlb access for memory initialization
                    // i.e., memory seeding or instr. seeding -> don't update
                    // TLB and stats
                    return tlb_hit;
                }

                localNumTLBAccesses++;

                if (!entry) {
                    localNumTLBMisses++;
                } else {
                    localNumTLBHits++;
                }
            }
        }

        return tlb_hit;
    }

    Fault
    GpuTLB::translate(const RequestPtr &req, ThreadContext *tc,
                      Translation *translation, Mode mode,
                      bool &delayedResponse, bool timing, int &latency)
    {
        uint32_t flags = req->getFlags();
        int seg = flags & SegmentFlagMask;
        bool storeCheck = flags & (StoreCheck << FlagShift);

        // If this is true, we're dealing with a request
        // to a non-memory address space.
        if (seg == SEGMENT_REG_MS) {
            return translateInt(mode == Mode::Read, req, tc);
        }

        delayedResponse = false;
        Addr vaddr = req->getVaddr();
        DPRINTF(GPUTLB, "Translating vaddr %#x.\n", vaddr);

        HandyM5Reg m5Reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);

        // If protected mode has been enabled...
        if (m5Reg.prot) {
            DPRINTF(GPUTLB, "In protected mode.\n");
            // If we're not in 64-bit mode, do protection/limit checks
            if (m5Reg.mode != LongMode) {
                DPRINTF(GPUTLB, "Not in long mode. Checking segment "
                        "protection.\n");

                // Check for a null segment selector.
                if (!(seg == SEGMENT_REG_TSG || seg == SYS_SEGMENT_REG_IDTR ||
                    seg == SEGMENT_REG_HS || seg == SEGMENT_REG_LS)
                    && !tc->readMiscRegNoEffect(MISCREG_SEG_SEL(seg))) {
                    return std::make_shared<GeneralProtection>(0);
                }

                bool expandDown = false;
                SegAttr attr = tc->readMiscRegNoEffect(MISCREG_SEG_ATTR(seg));

                if (seg >= SEGMENT_REG_ES && seg <= SEGMENT_REG_HS) {
                    if (!attr.writable && (mode == BaseTLB::Write ||
                        storeCheck))
                        return std::make_shared<GeneralProtection>(0);

                    if (!attr.readable && mode == BaseTLB::Read)
                        return std::make_shared<GeneralProtection>(0);

                    expandDown = attr.expandDown;

                }

                Addr base = tc->readMiscRegNoEffect(MISCREG_SEG_BASE(seg));
                Addr limit = tc->readMiscRegNoEffect(MISCREG_SEG_LIMIT(seg));
                // This assumes we're not in 64 bit mode. If we were, the
                // default address size is 64 bits, overridable to 32.
                int size = 32;
                bool sizeOverride = (flags & (AddrSizeFlagBit << FlagShift));
                SegAttr csAttr = tc->readMiscRegNoEffect(MISCREG_CS_ATTR);

                if ((csAttr.defaultSize && sizeOverride) ||
                    (!csAttr.defaultSize && !sizeOverride)) {
                    size = 16;
                }

                Addr offset = bits(vaddr - base, size - 1, 0);
                Addr endOffset = offset + req->getSize() - 1;

                if (expandDown) {
                    DPRINTF(GPUTLB, "Checking an expand down segment.\n");
                    warn_once("Expand down segments are untested.\n");

                    if (offset <= limit || endOffset <= limit)
                        return std::make_shared<GeneralProtection>(0);
                } else {
                    if (offset > limit || endOffset > limit)
                        return std::make_shared<GeneralProtection>(0);
                }
            }

            // If paging is enabled, do the translation.
            if (m5Reg.paging) {
                DPRINTF(GPUTLB, "Paging enabled.\n");
                // The vaddr already has the segment base applied.
                TlbEntry *entry = lookup(vaddr);
                localNumTLBAccesses++;

                if (!entry) {
                    localNumTLBMisses++;
                    if (timing) {
                        latency = missLatency1;
                    }

                    if (FullSystem) {
                        fatal("GpuTLB doesn't support full-system mode\n");
                    } else {
                        DPRINTF(GPUTLB, "Handling a TLB miss for address %#x "
                                "at pc %#x.\n", vaddr, tc->instAddr());

                        Process *p = tc->getProcessPtr();
                        const EmulationPageTable::Entry *pte =
                            p->pTable->lookup(vaddr);

                        if (!pte && mode != BaseTLB::Execute) {
                            // penalize a "page fault" more
                            if (timing)
                                latency += missLatency2;

                            if (p->fixupFault(vaddr))
                                pte = p->pTable->lookup(vaddr);
                        }

                        if (!pte) {
                            return std::make_shared<PageFault>(vaddr, true,
                                                               mode, true,
                                                               false);
                        } else {
                            Addr alignedVaddr = p->pTable->pageAlign(vaddr);

                            DPRINTF(GPUTLB, "Mapping %#x to %#x\n",
                                    alignedVaddr, pte->paddr);

                            TlbEntry gpuEntry(p->pid(), alignedVaddr,
                                              pte->paddr, false, false);
                            entry = insert(alignedVaddr, gpuEntry);
                        }

                        DPRINTF(GPUTLB, "Miss was serviced.\n");
                    }
                } else {
                    localNumTLBHits++;

                    if (timing) {
                        latency = hitLatency;
                    }
                }

                // Do paging protection checks.
                bool inUser = (m5Reg.cpl == 3 &&
                               !(flags & (CPL0FlagBit << FlagShift)));

                CR0 cr0 = tc->readMiscRegNoEffect(MISCREG_CR0);
                bool badWrite = (!entry->writable && (inUser || cr0.wp));

                if ((inUser && !entry->user) || (mode == BaseTLB::Write &&
                     badWrite)) {
                    // The page must have been present to get into the TLB in
                    // the first place. We'll assume the reserved bits are
                    // fine even though we're not checking them.
                    return std::make_shared<PageFault>(vaddr, true, mode,
                                                       inUser, false);
                }

                if (storeCheck && badWrite) {
                    // This would fault if this were a write, so return a page
                    // fault that reflects that happening.
                    return std::make_shared<PageFault>(vaddr, true,
                                                       BaseTLB::Write,
                                                       inUser, false);
                }


                DPRINTF(GPUTLB, "Entry found with paddr %#x, doing protection "
                        "checks.\n", entry->paddr);

                int page_size = entry->size();
                Addr paddr = entry->paddr | (vaddr & (page_size - 1));
                DPRINTF(GPUTLB, "Translated %#x -> %#x.\n", vaddr, paddr);
                req->setPaddr(paddr);

                if (entry->uncacheable)
                    req->setFlags(Request::UNCACHEABLE);
            } else {
                //Use the address which already has segmentation applied.
                DPRINTF(GPUTLB, "Paging disabled.\n");
                DPRINTF(GPUTLB, "Translated %#x -> %#x.\n", vaddr, vaddr);
                req->setPaddr(vaddr);
            }
        } else {
            // Real mode
            DPRINTF(GPUTLB, "In real mode.\n");
            DPRINTF(GPUTLB, "Translated %#x -> %#x.\n", vaddr, vaddr);
            req->setPaddr(vaddr);
        }

        // Check for an access to the local APIC
        if (FullSystem) {
            LocalApicBase localApicBase =
                tc->readMiscRegNoEffect(MISCREG_APIC_BASE);

            Addr baseAddr = localApicBase.base * PageBytes;
            Addr paddr = req->getPaddr();

            if (baseAddr <= paddr && baseAddr + PageBytes > paddr) {
                // Force the access to be uncacheable.
                req->setFlags(Request::UNCACHEABLE);
                req->setPaddr(x86LocalAPICAddress(tc->contextId(),
                                                  paddr - baseAddr));
            }
        }

        return NoFault;
    };

    Fault
    GpuTLB::translateAtomic(const RequestPtr &req, ThreadContext *tc,
                            Mode mode, int &latency)
    {
        bool delayedResponse;

        return GpuTLB::translate(req, tc, nullptr, mode, delayedResponse, false,
                                 latency);
    }

    void
    GpuTLB::translateTiming(const RequestPtr &req, ThreadContext *tc,
            Translation *translation, Mode mode, int &latency)
    {
        bool delayedResponse;
        assert(translation);

        Fault fault = GpuTLB::translate(req, tc, translation, mode,
                                        delayedResponse, true, latency);

        if (!delayedResponse)
            translation->finish(fault, req, tc, mode);
    }

    Walker*
    GpuTLB::getWalker()
    {
        return walker;
    }


    void
    GpuTLB::serialize(CheckpointOut &cp) const
    {
    }

    void
    GpuTLB::unserialize(CheckpointIn &cp)
    {
    }

    void
    GpuTLB::regStats()
    {
        ClockedObject::regStats();

        localNumTLBAccesses
            .name(name() + ".local_TLB_accesses")
            .desc("Number of TLB accesses")
            ;

        localNumTLBHits
            .name(name() + ".local_TLB_hits")
            .desc("Number of TLB hits")
            ;

        localNumTLBMisses
            .name(name() + ".local_TLB_misses")
            .desc("Number of TLB misses")
            ;

        localTLBMissRate
            .name(name() + ".local_TLB_miss_rate")
            .desc("TLB miss rate")
            ;

        accessCycles
            .name(name() + ".access_cycles")
            .desc("Cycles spent accessing this TLB level")
            ;

        pageTableCycles
            .name(name() + ".page_table_cycles")
            .desc("Cycles spent accessing the page table")
            ;

        localTLBMissRate = 100 * localNumTLBMisses / localNumTLBAccesses;

        numUniquePages
            .name(name() + ".unique_pages")
            .desc("Number of unique pages touched")
            ;

        localCycles
            .name(name() + ".local_cycles")
            .desc("Number of cycles spent in queue for all incoming reqs")
            ;

        localLatency
            .name(name() + ".local_latency")
            .desc("Avg. latency over incoming coalesced reqs")
            ;

        localLatency = localCycles / localNumTLBAccesses;

        globalNumTLBAccesses
            .name(name() + ".global_TLB_accesses")
            .desc("Number of TLB accesses")
            ;

        globalNumTLBHits
            .name(name() + ".global_TLB_hits")
            .desc("Number of TLB hits")
            ;

        globalNumTLBMisses
            .name(name() + ".global_TLB_misses")
            .desc("Number of TLB misses")
            ;

        globalTLBMissRate
            .name(name() + ".global_TLB_miss_rate")
            .desc("TLB miss rate")
            ;

        globalTLBMissRate = 100 * globalNumTLBMisses / globalNumTLBAccesses;

        avgReuseDistance
            .name(name() + ".avg_reuse_distance")
            .desc("avg. reuse distance over all pages (in ticks)")
            ;

    }

    /**
     * Do the TLB lookup for this coalesced request and schedule
     * another event <TLB access latency> cycles later.
     */

    void
    GpuTLB::issueTLBLookup(PacketPtr pkt)
    {
        assert(pkt);
        assert(pkt->senderState);

        Addr virt_page_addr = roundDown(pkt->req->getVaddr(),
                                        TheISA::PageBytes);

        TranslationState *sender_state =
                safe_cast<TranslationState*>(pkt->senderState);

        bool update_stats = !sender_state->prefetch;
        ThreadContext * tmp_tc = sender_state->tc;

        DPRINTF(GPUTLB, "Translation req. for virt. page addr %#x\n",
                virt_page_addr);

        int req_cnt = sender_state->reqCnt.back();

        if (update_stats) {
            accessCycles -= (curTick() * req_cnt);
            localCycles -= curTick();
            updatePageFootprint(virt_page_addr);
            globalNumTLBAccesses += req_cnt;
        }

        tlbOutcome lookup_outcome = TLB_MISS;
        const RequestPtr &tmp_req = pkt->req;

        // Access the TLB and figure out if it's a hit or a miss.
        bool success = tlbLookup(tmp_req, tmp_tc, update_stats);

        if (success) {
            lookup_outcome = TLB_HIT;
            // Put the entry in SenderState
            TlbEntry *entry = lookup(tmp_req->getVaddr(), false);
            assert(entry);

            auto p = sender_state->tc->getProcessPtr();
            sender_state->tlbEntry =
                new TlbEntry(p->pid(), entry->vaddr, entry->paddr,
                             false, false);

            if (update_stats) {
                // the reqCnt has an entry per level, so its size tells us
                // which level we are in
                sender_state->hitLevel = sender_state->reqCnt.size();
                globalNumTLBHits += req_cnt;
            }
        } else {
            if (update_stats)
                globalNumTLBMisses += req_cnt;
        }

        /*
         * We now know the TLB lookup outcome (if it's a hit or a miss), as well
         * as the TLB access latency.
         *
         * We create and schedule a new TLBEvent which will help us take the
         * appropriate actions (e.g., update TLB on a hit, send request to lower
         * level TLB on a miss, or start a page walk if this was the last-level
         * TLB)
         */
        TLBEvent *tlb_event =
            new TLBEvent(this, virt_page_addr, lookup_outcome, pkt);

        if (translationReturnEvent.count(virt_page_addr)) {
            panic("Virtual Page Address %#x already has a return event\n",
                  virt_page_addr);
        }

        translationReturnEvent[virt_page_addr] = tlb_event;
        assert(tlb_event);

        DPRINTF(GPUTLB, "schedule translationReturnEvent @ curTick %d\n",
                curTick() + this->ticks(hitLatency));

        schedule(tlb_event, curTick() + this->ticks(hitLatency));
    }

    GpuTLB::TLBEvent::TLBEvent(GpuTLB* _tlb, Addr _addr, tlbOutcome tlb_outcome,
                               PacketPtr _pkt)
        : Event(CPU_Tick_Pri), tlb(_tlb), virtPageAddr(_addr),
        outcome(tlb_outcome), pkt(_pkt)
    {
    }

    /**
     * Do Paging protection checks. If we encounter a page fault, then
     * an assertion is fired.
     */
    void
    GpuTLB::pagingProtectionChecks(ThreadContext *tc, PacketPtr pkt,
            TlbEntry * tlb_entry, Mode mode)
    {
        HandyM5Reg m5Reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);
        uint32_t flags = pkt->req->getFlags();
        bool storeCheck = flags & (StoreCheck << FlagShift);

        // Do paging protection checks.
        bool inUser = (m5Reg.cpl == 3 && !(flags & (CPL0FlagBit << FlagShift)));
        CR0 cr0 = tc->readMiscRegNoEffect(MISCREG_CR0);

        bool badWrite = (!tlb_entry->writable && (inUser || cr0.wp));

        if ((inUser && !tlb_entry->user) ||
            (mode == BaseTLB::Write && badWrite)) {
            // The page must have been present to get into the TLB in
            // the first place. We'll assume the reserved bits are
            // fine even though we're not checking them.
            panic("Page fault detected");
        }

        if (storeCheck && badWrite) {
            // This would fault if this were a write, so return a page
            // fault that reflects that happening.
            panic("Page fault detected");
        }
    }

    /**
     * handleTranslationReturn is called on a TLB hit,
     * when a TLB miss returns or when a page fault returns.
     * The latter calls handelHit with TLB miss as tlbOutcome.
     */
    void
    GpuTLB::handleTranslationReturn(Addr virt_page_addr, tlbOutcome tlb_outcome,
            PacketPtr pkt)
    {

        assert(pkt);
        Addr vaddr = pkt->req->getVaddr();

        TranslationState *sender_state =
            safe_cast<TranslationState*>(pkt->senderState);

        ThreadContext *tc = sender_state->tc;
        Mode mode = sender_state->tlbMode;

        TlbEntry *local_entry, *new_entry;

        if (tlb_outcome == TLB_HIT) {
            DPRINTF(GPUTLB, "Translation Done - TLB Hit for addr %#x\n", vaddr);
            local_entry = sender_state->tlbEntry;
        } else {
            DPRINTF(GPUTLB, "Translation Done - TLB Miss for addr %#x\n",
                    vaddr);

            // We are returning either from a page walk or from a hit at a lower
            // TLB level. The senderState should be "carrying" a pointer to the
            // correct TLBEntry.
            new_entry = sender_state->tlbEntry;
            assert(new_entry);
            local_entry = new_entry;

            if (allocationPolicy) {
                DPRINTF(GPUTLB, "allocating entry w/ addr %#x\n",
                        virt_page_addr);

                local_entry = insert(virt_page_addr, *new_entry);
            }

            assert(local_entry);
        }

        /**
         * At this point the packet carries an up-to-date tlbEntry pointer
         * in its senderState.
         * Next step is to do the paging protection checks.
         */
        DPRINTF(GPUTLB, "Entry found with vaddr %#x,  doing protection checks "
                "while paddr was %#x.\n", local_entry->vaddr,
                local_entry->paddr);

        pagingProtectionChecks(tc, pkt, local_entry, mode);
        int page_size = local_entry->size();
        Addr paddr = local_entry->paddr | (vaddr & (page_size - 1));
        DPRINTF(GPUTLB, "Translated %#x -> %#x.\n", vaddr, paddr);

        // Since this packet will be sent through the cpu side slave port,
        // it must be converted to a response pkt if it is not one already
        if (pkt->isRequest()) {
            pkt->makeTimingResponse();
        }

        pkt->req->setPaddr(paddr);

        if (local_entry->uncacheable) {
             pkt->req->setFlags(Request::UNCACHEABLE);
        }

        //send packet back to coalescer
        cpuSidePort[0]->sendTimingResp(pkt);
        //schedule cleanup event
        cleanupQueue.push(virt_page_addr);

        // schedule this only once per cycle.
        // The check is required because we might have multiple translations
        // returning the same cycle
        // this is a maximum priority event and must be on the same cycle
        // as the cleanup event in TLBCoalescer to avoid a race with
        // IssueProbeEvent caused by TLBCoalescer::MemSidePort::recvReqRetry
        if (!cleanupEvent.scheduled())
            schedule(cleanupEvent, curTick());
    }

    /**
     * Here we take the appropriate actions based on the result of the
     * TLB lookup.
     */
    void
    GpuTLB::translationReturn(Addr virtPageAddr, tlbOutcome outcome,
                              PacketPtr pkt)
    {
        DPRINTF(GPUTLB, "Triggered TLBEvent for addr %#x\n", virtPageAddr);

        assert(translationReturnEvent[virtPageAddr]);
        assert(pkt);

        TranslationState *tmp_sender_state =
            safe_cast<TranslationState*>(pkt->senderState);

        int req_cnt = tmp_sender_state->reqCnt.back();
        bool update_stats = !tmp_sender_state->prefetch;


        if (outcome == TLB_HIT) {
            handleTranslationReturn(virtPageAddr, TLB_HIT, pkt);

            if (update_stats) {
                accessCycles += (req_cnt * curTick());
                localCycles += curTick();
            }

        } else if (outcome == TLB_MISS) {

            DPRINTF(GPUTLB, "This is a TLB miss\n");
            if (update_stats) {
                accessCycles += (req_cnt*curTick());
                localCycles += curTick();
            }

            if (hasMemSidePort) {
                // the one cyle added here represent the delay from when we get
                // the reply back till when we propagate it to the coalescer
                // above.
                if (update_stats) {
                    accessCycles += (req_cnt * 1);
                    localCycles += 1;
                }

                /**
                 * There is a TLB below. Send the coalesced request.
                 * We actually send the very first packet of all the
                 * pending packets for this virtual page address.
                 */
                if (!memSidePort[0]->sendTimingReq(pkt)) {
                    DPRINTF(GPUTLB, "Failed sending translation request to "
                            "lower level TLB for addr %#x\n", virtPageAddr);

                    memSidePort[0]->retries.push_back(pkt);
                } else {
                    DPRINTF(GPUTLB, "Sent translation request to lower level "
                            "TLB for addr %#x\n", virtPageAddr);
                }
            } else {
                //this is the last level TLB. Start a page walk
                DPRINTF(GPUTLB, "Last level TLB - start a page walk for "
                        "addr %#x\n", virtPageAddr);

                if (update_stats)
                    pageTableCycles -= (req_cnt*curTick());

                TLBEvent *tlb_event = translationReturnEvent[virtPageAddr];
                assert(tlb_event);
                tlb_event->updateOutcome(PAGE_WALK);
                schedule(tlb_event, curTick() + ticks(missLatency2));
            }
        } else if (outcome == PAGE_WALK) {
            if (update_stats)
                pageTableCycles += (req_cnt*curTick());

            // Need to access the page table and update the TLB
            DPRINTF(GPUTLB, "Doing a page walk for address %#x\n",
                    virtPageAddr);

            TranslationState *sender_state =
                safe_cast<TranslationState*>(pkt->senderState);

            Process *p = sender_state->tc->getProcessPtr();
            Addr vaddr = pkt->req->getVaddr();
    #ifndef NDEBUG
            Addr alignedVaddr = p->pTable->pageAlign(vaddr);
            assert(alignedVaddr == virtPageAddr);
    #endif
            const EmulationPageTable::Entry *pte = p->pTable->lookup(vaddr);
            if (!pte && sender_state->tlbMode != BaseTLB::Execute &&
                    p->fixupFault(vaddr)) {
                pte = p->pTable->lookup(vaddr);
            }

            if (pte) {
                DPRINTF(GPUTLB, "Mapping %#x to %#x\n", alignedVaddr,
                        pte->paddr);

                sender_state->tlbEntry =
                    new TlbEntry(p->pid(), virtPageAddr, pte->paddr, false,
                                 false);
            } else {
                sender_state->tlbEntry = nullptr;
            }

            handleTranslationReturn(virtPageAddr, TLB_MISS, pkt);
        } else if (outcome == MISS_RETURN) {
            /** we add an extra cycle in the return path of the translation
             * requests in between the various TLB levels.
             */
            handleTranslationReturn(virtPageAddr, TLB_MISS, pkt);
        } else {
            panic("Unexpected TLB outcome %d", outcome);
        }
    }

    void
    GpuTLB::TLBEvent::process()
    {
        tlb->translationReturn(virtPageAddr, outcome, pkt);
    }

    const char*
    GpuTLB::TLBEvent::description() const
    {
        return "trigger translationDoneEvent";
    }

    void
    GpuTLB::TLBEvent::updateOutcome(tlbOutcome _outcome)
    {
        outcome = _outcome;
    }

    Addr
    GpuTLB::TLBEvent::getTLBEventVaddr()
    {
        return virtPageAddr;
    }

    /*
     * recvTiming receives a coalesced timing request from a TLBCoalescer
     * and it calls issueTLBLookup()
     * It only rejects the packet if we have exceeded the max
     * outstanding number of requests for the TLB
     */
    bool
    GpuTLB::CpuSidePort::recvTimingReq(PacketPtr pkt)
    {
        if (tlb->outstandingReqs < tlb->maxCoalescedReqs) {
            tlb->issueTLBLookup(pkt);
            // update number of outstanding translation requests
            tlb->outstandingReqs++;
            return true;
         } else {
            DPRINTF(GPUTLB, "Reached maxCoalescedReqs number %d\n",
                    tlb->outstandingReqs);
            return false;
         }
    }

    /**
     * handleFuncTranslationReturn is called on a TLB hit,
     * when a TLB miss returns or when a page fault returns.
     * It updates LRU, inserts the TLB entry on a miss
     * depending on the allocation policy and does the required
     * protection checks. It does NOT create a new packet to
     * update the packet's addr; this is done in hsail-gpu code.
     */
    void
    GpuTLB::handleFuncTranslationReturn(PacketPtr pkt, tlbOutcome tlb_outcome)
    {
        TranslationState *sender_state =
            safe_cast<TranslationState*>(pkt->senderState);

        ThreadContext *tc = sender_state->tc;
        Mode mode = sender_state->tlbMode;
        Addr vaddr = pkt->req->getVaddr();

        TlbEntry *local_entry, *new_entry;

        if (tlb_outcome == TLB_HIT) {
            DPRINTF(GPUTLB, "Functional Translation Done - TLB hit for addr "
                    "%#x\n", vaddr);

            local_entry = sender_state->tlbEntry;
        } else {
            DPRINTF(GPUTLB, "Functional Translation Done - TLB miss for addr "
                    "%#x\n", vaddr);

            // We are returning either from a page walk or from a hit at a lower
            // TLB level. The senderState should be "carrying" a pointer to the
            // correct TLBEntry.
            new_entry = sender_state->tlbEntry;
            assert(new_entry);
            local_entry = new_entry;

            if (allocationPolicy) {
                Addr virt_page_addr = roundDown(vaddr, TheISA::PageBytes);

                DPRINTF(GPUTLB, "allocating entry w/ addr %#x\n",
                        virt_page_addr);

                local_entry = insert(virt_page_addr, *new_entry);
            }

            assert(local_entry);
        }

        DPRINTF(GPUTLB, "Entry found with vaddr %#x, doing protection checks "
                "while paddr was %#x.\n", local_entry->vaddr,
                local_entry->paddr);

        /**
         * Do paging checks if it's a normal functional access.  If it's for a
         * prefetch, then sometimes you can try to prefetch something that
         * won't pass protection. We don't actually want to fault becuase there
         * is no demand access to deem this a violation.  Just put it in the
         * TLB and it will fault if indeed a future demand access touches it in
         * violation.
         *
         * This feature could be used to explore security issues around
         * speculative memory accesses.
         */
        if (!sender_state->prefetch && sender_state->tlbEntry)
            pagingProtectionChecks(tc, pkt, local_entry, mode);

        int page_size = local_entry->size();
        Addr paddr = local_entry->paddr | (vaddr & (page_size - 1));
        DPRINTF(GPUTLB, "Translated %#x -> %#x.\n", vaddr, paddr);

        pkt->req->setPaddr(paddr);

        if (local_entry->uncacheable)
             pkt->req->setFlags(Request::UNCACHEABLE);
    }

    // This is used for atomic translations. Need to
    // make it all happen during the same cycle.
    void
    GpuTLB::CpuSidePort::recvFunctional(PacketPtr pkt)
    {
        TranslationState *sender_state =
            safe_cast<TranslationState*>(pkt->senderState);

        ThreadContext *tc = sender_state->tc;
        bool update_stats = !sender_state->prefetch;

        Addr virt_page_addr = roundDown(pkt->req->getVaddr(),
                                        TheISA::PageBytes);

        if (update_stats)
            tlb->updatePageFootprint(virt_page_addr);

        // do the TLB lookup without updating the stats
        bool success = tlb->tlbLookup(pkt->req, tc, update_stats);
        tlbOutcome tlb_outcome = success ? TLB_HIT : TLB_MISS;

        // functional mode means no coalescing
        // global metrics are the same as the local metrics
        if (update_stats) {
            tlb->globalNumTLBAccesses++;

            if (success) {
                sender_state->hitLevel = sender_state->reqCnt.size();
                tlb->globalNumTLBHits++;
            }
        }

        if (!success) {
            if (update_stats)
                tlb->globalNumTLBMisses++;
            if (tlb->hasMemSidePort) {
                // there is a TLB below -> propagate down the TLB hierarchy
                tlb->memSidePort[0]->sendFunctional(pkt);
                // If no valid translation from a prefetch, then just return
                if (sender_state->prefetch && !pkt->req->hasPaddr())
                    return;
            } else {
                // Need to access the page table and update the TLB
                DPRINTF(GPUTLB, "Doing a page walk for address %#x\n",
                        virt_page_addr);

                Process *p = tc->getProcessPtr();

                Addr vaddr = pkt->req->getVaddr();
    #ifndef NDEBUG
                Addr alignedVaddr = p->pTable->pageAlign(vaddr);
                assert(alignedVaddr == virt_page_addr);
    #endif

                const EmulationPageTable::Entry *pte =
                        p->pTable->lookup(vaddr);
                if (!pte && sender_state->tlbMode != BaseTLB::Execute &&
                        p->fixupFault(vaddr)) {
                    pte = p->pTable->lookup(vaddr);
                }

                if (!sender_state->prefetch) {
                    // no PageFaults are permitted after
                    // the second page table lookup
                    assert(pte);

                    DPRINTF(GPUTLB, "Mapping %#x to %#x\n", alignedVaddr,
                            pte->paddr);

                    sender_state->tlbEntry =
                        new TlbEntry(p->pid(), virt_page_addr,
                                     pte->paddr, false, false);
                } else {
                    // If this was a prefetch, then do the normal thing if it
                    // was a successful translation.  Otherwise, send an empty
                    // TLB entry back so that it can be figured out as empty and
                    // handled accordingly.
                    if (pte) {
                        DPRINTF(GPUTLB, "Mapping %#x to %#x\n", alignedVaddr,
                                pte->paddr);

                        sender_state->tlbEntry =
                            new TlbEntry(p->pid(), virt_page_addr,
                                         pte->paddr, false, false);
                    } else {
                        DPRINTF(GPUPrefetch, "Prefetch failed %#x\n",
                                alignedVaddr);

                        sender_state->tlbEntry = nullptr;

                        return;
                    }
                }
            }
        } else {
            DPRINTF(GPUPrefetch, "Functional Hit for vaddr %#x\n",
                    tlb->lookup(pkt->req->getVaddr()));

            TlbEntry *entry = tlb->lookup(pkt->req->getVaddr(),
                                             update_stats);

            assert(entry);

            auto p = sender_state->tc->getProcessPtr();
            sender_state->tlbEntry =
                new TlbEntry(p->pid(), entry->vaddr, entry->paddr,
                             false, false);
        }
        // This is the function that would populate pkt->req with the paddr of
        // the translation. But if no translation happens (i.e Prefetch fails)
        // then the early returns in the above code wiill keep this function
        // from executing.
        tlb->handleFuncTranslationReturn(pkt, tlb_outcome);
    }

    void
    GpuTLB::CpuSidePort::recvReqRetry()
    {
        // The CPUSidePort never sends anything but replies. No retries
        // expected.
        panic("recvReqRetry called");
    }

    AddrRangeList
    GpuTLB::CpuSidePort::getAddrRanges() const
    {
        // currently not checked by the master
        AddrRangeList ranges;

        return ranges;
    }

    /**
     * MemSidePort receives the packet back.
     * We need to call the handleTranslationReturn
     * and propagate up the hierarchy.
     */
    bool
    GpuTLB::MemSidePort::recvTimingResp(PacketPtr pkt)
    {
        Addr virt_page_addr = roundDown(pkt->req->getVaddr(),
                                        TheISA::PageBytes);

        DPRINTF(GPUTLB, "MemSidePort recvTiming for virt_page_addr %#x\n",
                virt_page_addr);

        TLBEvent *tlb_event = tlb->translationReturnEvent[virt_page_addr];
        assert(tlb_event);
        assert(virt_page_addr == tlb_event->getTLBEventVaddr());

        tlb_event->updateOutcome(MISS_RETURN);
        tlb->schedule(tlb_event, curTick()+tlb->ticks(1));

        return true;
    }

    void
    GpuTLB::MemSidePort::recvReqRetry()
    {
        // No retries should reach the TLB. The retries
        // should only reach the TLBCoalescer.
        panic("recvReqRetry called");
    }

    void
    GpuTLB::cleanup()
    {
        while (!cleanupQueue.empty()) {
            Addr cleanup_addr = cleanupQueue.front();
            cleanupQueue.pop();

            // delete TLBEvent
            TLBEvent * old_tlb_event = translationReturnEvent[cleanup_addr];
            delete old_tlb_event;
            translationReturnEvent.erase(cleanup_addr);

            // update number of outstanding requests
            outstandingReqs--;
        }

        /** the higher level coalescer should retry if it has
         * any pending requests.
         */
        for (int i = 0; i < cpuSidePort.size(); ++i) {
            cpuSidePort[i]->sendRetryReq();
        }
    }

    void
    GpuTLB::updatePageFootprint(Addr virt_page_addr)
    {

        std::pair<AccessPatternTable::iterator, bool> ret;

        AccessInfo tmp_access_info;
        tmp_access_info.lastTimeAccessed = 0;
        tmp_access_info.accessesPerPage = 0;
        tmp_access_info.totalReuseDistance = 0;
        tmp_access_info.sumDistance = 0;
        tmp_access_info.meanDistance = 0;

        ret = TLBFootprint.insert(AccessPatternTable::value_type(virt_page_addr,
                                  tmp_access_info));

        bool first_page_access = ret.second;

        if (first_page_access) {
            numUniquePages++;
        } else  {
            int accessed_before;
            accessed_before  = curTick() - ret.first->second.lastTimeAccessed;
            ret.first->second.totalReuseDistance += accessed_before;
        }

        ret.first->second.accessesPerPage++;
        ret.first->second.lastTimeAccessed = curTick();

        if (accessDistance) {
            ret.first->second.localTLBAccesses
                .push_back(localNumTLBAccesses.value());
        }
    }

    void
    GpuTLB::exitCallback()
    {
        std::ostream *page_stat_file = nullptr;

        if (accessDistance) {

            // print per page statistics to a separate file (.csv format)
            // simout is the gem5 output directory (default is m5out or the one
            // specified with -d
            page_stat_file = simout.create(name().c_str())->stream();

            // print header
            *page_stat_file << "page,max_access_distance,mean_access_distance, "
                            << "stddev_distance" << std::endl;
        }

        // update avg. reuse distance footprint
        AccessPatternTable::iterator iter, iter_begin, iter_end;
        unsigned int sum_avg_reuse_distance_per_page = 0;

        // iterate through all pages seen by this TLB
        for (iter = TLBFootprint.begin(); iter != TLBFootprint.end(); iter++) {
            sum_avg_reuse_distance_per_page += iter->second.totalReuseDistance /
                                               iter->second.accessesPerPage;

            if (accessDistance) {
                unsigned int tmp = iter->second.localTLBAccesses[0];
                unsigned int prev = tmp;

                for (int i = 0; i < iter->second.localTLBAccesses.size(); ++i) {
                    if (i) {
                        tmp = prev + 1;
                    }

                    prev = iter->second.localTLBAccesses[i];
                    // update the localTLBAccesses value
                    // with the actual differece
                    iter->second.localTLBAccesses[i] -= tmp;
                    // compute the sum of AccessDistance per page
                    // used later for mean
                    iter->second.sumDistance +=
                        iter->second.localTLBAccesses[i];
                }

                iter->second.meanDistance =
                    iter->second.sumDistance / iter->second.accessesPerPage;

                // compute std_dev and max  (we need a second round because we
                // need to know the mean value
                unsigned int max_distance = 0;
                unsigned int stddev_distance = 0;

                for (int i = 0; i < iter->second.localTLBAccesses.size(); ++i) {
                    unsigned int tmp_access_distance =
                        iter->second.localTLBAccesses[i];

                    if (tmp_access_distance > max_distance) {
                        max_distance = tmp_access_distance;
                    }

                    unsigned int diff =
                        tmp_access_distance - iter->second.meanDistance;
                    stddev_distance += pow(diff, 2);

                }

                stddev_distance =
                    sqrt(stddev_distance/iter->second.accessesPerPage);

                if (page_stat_file) {
                    *page_stat_file << std::hex << iter->first << ",";
                    *page_stat_file << std::dec << max_distance << ",";
                    *page_stat_file << std::dec << iter->second.meanDistance
                                    << ",";
                    *page_stat_file << std::dec << stddev_distance;
                    *page_stat_file << std::endl;
                }

                // erase the localTLBAccesses array
                iter->second.localTLBAccesses.clear();
            }
        }

        if (!TLBFootprint.empty()) {
            avgReuseDistance =
                sum_avg_reuse_distance_per_page / TLBFootprint.size();
        }

        //clear the TLBFootprint map
        TLBFootprint.clear();
    }
} // namespace X86ISA

X86ISA::GpuTLB*
X86GPUTLBParams::create()
{
    return new X86ISA::GpuTLB(this);
}

