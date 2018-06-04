/*
 * Copyright (c) 2010-2016 ARM Limited
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
 * Authors: Ali Saidi
 *          Giacomo Gabrielli
 */

#ifndef __ARCH_ARM_TABLE_WALKER_HH__
#define __ARCH_ARM_TABLE_WALKER_HH__

#include <list>

#include "arch/arm/miscregs.hh"
#include "arch/arm/system.hh"
#include "arch/arm/tlb.hh"
#include "mem/request.hh"
#include "params/ArmTableWalker.hh"
#include "sim/eventq.hh"

class ThreadContext;

class DmaPort;

namespace ArmISA {
class Translation;
class TLB;
class Stage2MMU;

class TableWalker : public MemObject
{
  public:
    class WalkerState;

    class DescriptorBase {
      public:
        /** Current lookup level for this descriptor */
        LookupLevel lookupLevel;

        virtual Addr pfn() const = 0;
        virtual TlbEntry::DomainType domain() const = 0;
        virtual bool xn() const = 0;
        virtual uint8_t ap() const = 0;
        virtual bool global(WalkerState *currState) const = 0;
        virtual uint8_t offsetBits() const = 0;
        virtual bool secure(bool have_security, WalkerState *currState) const = 0;
        virtual std::string dbgHeader() const = 0;
        virtual uint64_t getRawData() const = 0;
        virtual uint8_t texcb() const
        {
            panic("texcb() not implemented for this class\n");
        }
        virtual bool shareable() const
        {
            panic("shareable() not implemented for this class\n");
        }
    };

    class L1Descriptor : public DescriptorBase {
      public:
        /** Type of page table entry ARM DDI 0406B: B3-8*/
        enum EntryType {
            Ignore,
            PageTable,
            Section,
            Reserved
        };

        /** The raw bits of the entry */
        uint32_t data;

        /** This entry has been modified (access flag set) and needs to be
         * written back to memory */
        bool _dirty;

        /** Default ctor */
        L1Descriptor() : data(0), _dirty(false)
        {
            lookupLevel = L1;
        }

        virtual uint64_t getRawData() const
        {
            return (data);
        }

        virtual std::string dbgHeader() const
        {
            return "Inserting Section Descriptor into TLB\n";
        }

        virtual uint8_t offsetBits() const
        {
            return 20;
        }

        EntryType type() const
        {
            return (EntryType)(data & 0x3);
        }

        /** Is the page a Supersection (16MB)?*/
        bool supersection() const
        {
            return bits(data, 18);
        }

        /** Return the physcal address of the entry, bits in position*/
        Addr paddr() const
        {
            if (supersection())
                panic("Super sections not implemented\n");
            return mbits(data, 31, 20);
        }
        /** Return the physcal address of the entry, bits in position*/
        Addr paddr(Addr va) const
        {
            if (supersection())
                panic("Super sections not implemented\n");
            return mbits(data, 31, 20) | mbits(va, 19, 0);
        }


        /** Return the physical frame, bits shifted right */
        Addr pfn() const
        {
            if (supersection())
                panic("Super sections not implemented\n");
            return bits(data, 31, 20);
        }

        /** Is the translation global (no asid used)? */
        bool global(WalkerState *currState) const
        {
            return !bits(data, 17);
        }

        /** Is the translation not allow execution? */
        bool xn() const
        {
            return bits(data, 4);
        }

        /** Three bit access protection flags */
        uint8_t ap() const
        {
            return (bits(data, 15) << 2) | bits(data, 11, 10);
        }

        /** Domain Client/Manager: ARM DDI 0406B: B3-31 */
        TlbEntry::DomainType domain() const
        {
            return static_cast<TlbEntry::DomainType>(bits(data, 8, 5));
        }

        /** Address of L2 descriptor if it exists */
        Addr l2Addr() const
        {
            return mbits(data, 31, 10);
        }

        /** Memory region attributes: ARM DDI 0406B: B3-32.
         * These bits are largly ignored by M5 and only used to
         * provide the illusion that the memory system cares about
         * anything but cachable vs. uncachable.
         */
        uint8_t texcb() const
        {
            return bits(data, 2) | bits(data, 3) << 1 | bits(data, 14, 12) << 2;
        }

        /** If the section is shareable. See texcb() comment. */
        bool shareable() const
        {
            return bits(data, 16);
        }

        /** Set access flag that this entry has been touched. Mark
         * the entry as requiring a writeback, in the future.
         */
        void setAp0()
        {
            data |= 1 << 10;
            _dirty = true;
        }

        /** This entry needs to be written back to memory */
        bool dirty() const
        {
            return _dirty;
        }

        /**
         * Returns true if this entry targets the secure physical address
         * map.
         */
        bool secure(bool have_security, WalkerState *currState) const
        {
            if (have_security) {
                if (type() == PageTable)
                    return !bits(data, 3);
                else
                    return !bits(data, 19);
            }
            return false;
        }
    };

    /** Level 2 page table descriptor */
    class L2Descriptor : public DescriptorBase {
      public:
        /** The raw bits of the entry. */
        uint32_t     data;
        L1Descriptor *l1Parent;

        /** This entry has been modified (access flag set) and needs to be
         * written back to memory */
        bool _dirty;

        /** Default ctor */
        L2Descriptor() : data(0), l1Parent(nullptr), _dirty(false)
        {
            lookupLevel = L2;
        }

        L2Descriptor(L1Descriptor &parent) : data(0), l1Parent(&parent),
                                             _dirty(false)
        {
            lookupLevel = L2;
        }

        virtual uint64_t getRawData() const
        {
            return (data);
        }

        virtual std::string dbgHeader() const
        {
            return "Inserting L2 Descriptor into TLB\n";
        }

        virtual TlbEntry::DomainType domain() const
        {
            return l1Parent->domain();
        }

        bool secure(bool have_security, WalkerState *currState) const
        {
            return l1Parent->secure(have_security, currState);
        }

        virtual uint8_t offsetBits() const
        {
            return large() ? 16 : 12;
        }

        /** Is the entry invalid */
        bool invalid() const
        {
            return bits(data, 1, 0) == 0;
        }

        /** What is the size of the mapping? */
        bool large() const
        {
            return bits(data, 1) == 0;
        }

        /** Is execution allowed on this mapping? */
        bool xn() const
        {
            return large() ? bits(data, 15) : bits(data, 0);
        }

        /** Is the translation global (no asid used)? */
        bool global(WalkerState *currState) const
        {
            return !bits(data, 11);
        }

        /** Three bit access protection flags */
        uint8_t ap() const
        {
           return bits(data, 5, 4) | (bits(data, 9) << 2);
        }

        /** Memory region attributes: ARM DDI 0406B: B3-32 */
        uint8_t texcb() const
        {
            return large() ?
                (bits(data, 2) | (bits(data, 3) << 1) | (bits(data, 14, 12) << 2)) :
                (bits(data, 2) | (bits(data, 3) << 1) | (bits(data, 8, 6) << 2));
        }

        /** Return the physical frame, bits shifted right */
        Addr pfn() const
        {
            return large() ? bits(data, 31, 16) : bits(data, 31, 12);
        }

        /** Return complete physical address given a VA */
        Addr paddr(Addr va) const
        {
            if (large())
                return mbits(data, 31, 16) | mbits(va, 15, 0);
            else
                return mbits(data, 31, 12) | mbits(va, 11, 0);
        }

        /** If the section is shareable. See texcb() comment. */
        bool shareable() const
        {
            return bits(data, 10);
        }

        /** Set access flag that this entry has been touched. Mark
         * the entry as requiring a writeback, in the future.
         */
        void setAp0()
        {
            data |= 1 << 4;
            _dirty = true;
        }

        /** This entry needs to be written back to memory */
        bool dirty() const
        {
            return _dirty;
        }

    };

    // Granule sizes for AArch64 long descriptors
    enum GrainSize {
        Grain4KB  = 12,
        Grain16KB = 14,
        Grain64KB = 16,
        ReservedGrain = 0
    };

    /** Long-descriptor format (LPAE) */
    class LongDescriptor : public DescriptorBase {
      public:
        /** Descriptor type */
        enum EntryType {
            Invalid,
            Table,
            Block,
            Page
        };

        /** The raw bits of the entry */
        uint64_t data;

        /** This entry has been modified (access flag set) and needs to be
         * written back to memory */
        bool _dirty;

        virtual uint64_t getRawData() const
        {
            return (data);
        }

        virtual std::string dbgHeader() const
        {
            if (type() == LongDescriptor::Page) {
                assert(lookupLevel == L3);
                return "Inserting Page descriptor into TLB\n";
            } else {
                assert(lookupLevel < L3);
                return "Inserting Block descriptor into TLB\n";
            }
        }

        /**
         * Returns true if this entry targets the secure physical address
         * map.
         */
        bool secure(bool have_security, WalkerState *currState) const
        {
            assert(type() == Block || type() == Page);
            return have_security && (currState->secureLookup && !bits(data, 5));
        }

        /** True if the current lookup is performed in AArch64 state */
        bool aarch64;

        /** Width of the granule size in bits */
        GrainSize grainSize;

        /** Return the descriptor type */
        EntryType type() const
        {
            switch (bits(data, 1, 0)) {
              case 0x1:
                // In AArch64 blocks are not allowed at L0 for the 4 KB granule
                // and at L1 for 16/64 KB granules
                if (grainSize > Grain4KB)
                    return lookupLevel == L2 ? Block : Invalid;
                return lookupLevel == L0 || lookupLevel == L3 ? Invalid : Block;
              case 0x3:
                return lookupLevel == L3 ? Page : Table;
              default:
                return Invalid;
            }
        }

        /** Return the bit width of the page/block offset */
        uint8_t offsetBits() const
        {
            if (type() == Block) {
                switch (grainSize) {
                    case Grain4KB:
                        return lookupLevel == L1 ? 30 /* 1 GB */
                                                 : 21 /* 2 MB */;
                    case Grain16KB:
                        return 25  /* 32 MB */;
                    case Grain64KB:
                        return 29 /* 512 MB */;
                    default:
                        panic("Invalid AArch64 VM granule size\n");
                }
            } else if (type() == Page) {
                switch (grainSize) {
                    case Grain4KB:
                    case Grain16KB:
                    case Grain64KB:
                        return grainSize; /* enum -> uint okay */
                    default:
                        panic("Invalid AArch64 VM granule size\n");
                }
            } else {
                panic("AArch64 page table entry must be block or page\n");
            }
        }

        /** Return the physical frame, bits shifted right */
        Addr pfn() const
        {
            if (aarch64)
                return bits(data, 47, offsetBits());
            return bits(data, 39, offsetBits());
        }

        /** Return the complete physical address given a VA */
        Addr paddr(Addr va) const
        {
            int n = offsetBits();
            if (aarch64)
                return mbits(data, 47, n) | mbits(va, n - 1, 0);
            return mbits(data, 39, n) | mbits(va, n - 1, 0);
        }

        /** Return the physical address of the entry */
        Addr paddr() const
        {
            if (aarch64)
                return mbits(data, 47, offsetBits());
            return mbits(data, 39, offsetBits());
        }

        /** Return the address of the next page table */
        Addr nextTableAddr() const
        {
            assert(type() == Table);
            if (aarch64)
                return mbits(data, 47, grainSize);
            else
                return mbits(data, 39, 12);
        }

        /** Return the address of the next descriptor */
        Addr nextDescAddr(Addr va) const
        {
            assert(type() == Table);
            Addr pa = 0;
            if (aarch64) {
                int stride = grainSize - 3;
                int va_lo = stride * (3 - (lookupLevel + 1)) + grainSize;
                int va_hi = va_lo + stride - 1;
                pa = nextTableAddr() | (bits(va, va_hi, va_lo) << 3);
            } else {
                if (lookupLevel == L1)
                    pa = nextTableAddr() | (bits(va, 29, 21) << 3);
                else  // lookupLevel == L2
                    pa = nextTableAddr() | (bits(va, 20, 12) << 3);
            }
            return pa;
        }

        /** Is execution allowed on this mapping? */
        bool xn() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 54);
        }

        /** Is privileged execution allowed on this mapping? (LPAE only) */
        bool pxn() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 53);
        }

        /** Contiguous hint bit. */
        bool contiguousHint() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 52);
        }

        /** Is the translation global (no asid used)? */
        bool global(WalkerState *currState) const
        {
            assert(currState && (type() == Block || type() == Page));
            if (!currState->aarch64 && (currState->isSecure &&
                                        !currState->secureLookup)) {
                return false;  // ARM ARM issue C B3.6.3
            } else if (currState->aarch64) {
                if (currState->el == EL2 || currState->el == EL3) {
                    return true;  // By default translations are treated as global
                                  // in AArch64 EL2 and EL3
                } else if (currState->isSecure && !currState->secureLookup) {
                    return false;
                }
            }
            return !bits(data, 11);
        }

        /** Returns true if the access flag (AF) is set. */
        bool af() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 10);
        }

        /** 2-bit shareability field */
        uint8_t sh() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 9, 8);
        }

        /** 2-bit access protection flags */
        uint8_t ap() const
        {
            assert(type() == Block || type() == Page);
            // Long descriptors only support the AP[2:1] scheme
            return bits(data, 7, 6);
        }

        /** Read/write access protection flag */
        bool rw() const
        {
            assert(type() == Block || type() == Page);
            return !bits(data, 7);
        }

        /** User/privileged level access protection flag */
        bool user() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 6);
        }

        /** Return the AP bits as compatible with the AP[2:0] format.  Utility
         * function used to simplify the code in the TLB for performing
         * permission checks. */
        static uint8_t ap(bool rw, bool user)
        {
            return ((!rw) << 2) | (user << 1);
        }

        TlbEntry::DomainType domain() const
        {
            // Long-desc. format only supports Client domain
            assert(type() == Block || type() == Page);
            return TlbEntry::DomainType::Client;
        }

        /** Attribute index */
        uint8_t attrIndx() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 4, 2);
        }

        /** Memory attributes, only used by stage 2 translations */
        uint8_t memAttr() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 5, 2);
        }

        /** Set access flag that this entry has been touched.  Mark the entry as
         * requiring a writeback, in the future. */
        void setAf()
        {
            data |= 1 << 10;
            _dirty = true;
        }

        /** This entry needs to be written back to memory */
        bool dirty() const
        {
            return _dirty;
        }

        /** Whether the subsequent levels of lookup are secure */
        bool secureTable() const
        {
            assert(type() == Table);
            return !bits(data, 63);
        }

        /** Two bit access protection flags for subsequent levels of lookup */
        uint8_t apTable() const
        {
            assert(type() == Table);
            return bits(data, 62, 61);
        }

        /** R/W protection flag for subsequent levels of lookup */
        uint8_t rwTable() const
        {
            assert(type() == Table);
            return !bits(data, 62);
        }

        /** User/privileged mode protection flag for subsequent levels of
         * lookup */
        uint8_t userTable() const
        {
            assert(type() == Table);
            return !bits(data, 61);
        }

        /** Is execution allowed on subsequent lookup levels? */
        bool xnTable() const
        {
            assert(type() == Table);
            return bits(data, 60);
        }

        /** Is privileged execution allowed on subsequent lookup levels? */
        bool pxnTable() const
        {
            assert(type() == Table);
            return bits(data, 59);
        }
    };

    class WalkerState
    {
      public:
        /** Thread context that we're doing the walk for */
        ThreadContext *tc;

        /** If the access is performed in AArch64 state */
        bool aarch64;

        /** Current exception level */
        ExceptionLevel el;

        /** Current physical address range in bits */
        int physAddrRange;

        /** Request that is currently being serviced */
        RequestPtr req;

        /** ASID that we're servicing the request under */
        uint16_t asid;
        uint8_t vmid;
        bool    isHyp;

        /** Translation state for delayed requests */
        TLB::Translation *transState;

        /** The fault that we are going to return */
        Fault fault;

        /** The virtual address that is being translated with tagging removed.*/
        Addr vaddr;

        /** The virtual address that is being translated */
        Addr vaddr_tainted;

        /** Cached copy of the sctlr as it existed when translation began */
        SCTLR sctlr;

        /** Cached copy of the scr as it existed when translation began */
        SCR scr;

        /** Cached copy of the cpsr as it existed when translation began */
        CPSR cpsr;

        /** Cached copy of ttbcr/tcr as it existed when translation began */
        union {
            TTBCR ttbcr; // AArch32 translations
            TCR tcr;     // AArch64 translations
        };

        /** Cached copy of the htcr as it existed when translation began. */
        HTCR htcr;

        /** Cached copy of the htcr as it existed when translation began. */
        HCR  hcr;

        /** Cached copy of the vtcr as it existed when translation began. */
        VTCR_t vtcr;

        /** If the access is a write */
        bool isWrite;

        /** If the access is a fetch (for execution, and no-exec) must be checked?*/
        bool isFetch;

        /** If the access comes from the secure state. */
        bool isSecure;

        /** Helper variables used to implement hierarchical access permissions
         * when the long-desc. format is used (LPAE only) */
        bool secureLookup;
        bool rwTable;
        bool userTable;
        bool xnTable;
        bool pxnTable;

        /** Flag indicating if a second stage of lookup is required */
        bool stage2Req;

        /** A pointer to the stage 2 translation that's in progress */
        TLB::Translation *stage2Tran;

        /** If the mode is timing or atomic */
        bool timing;

        /** If the atomic mode should be functional */
        bool functional;

        /** Save mode for use in delayed response */
        BaseTLB::Mode mode;

        /** The translation type that has been requested */
        TLB::ArmTranslationType tranType;

        /** Short-format descriptors */
        L1Descriptor l1Desc;
        L2Descriptor l2Desc;

        /** Long-format descriptor (LPAE and AArch64) */
        LongDescriptor longDesc;

        /** Whether the response is delayed in timing mode due to additional
         * lookups */
        bool delayed;

        TableWalker *tableWalker;

        /** Timestamp for calculating elapsed time in service (for stats) */
        Tick startTime;

        /** Page entries walked during service (for stats) */
        unsigned levels;

        void doL1Descriptor();
        void doL2Descriptor();

        void doLongDescriptor();

        WalkerState();

        std::string name() const { return tableWalker->name(); }
    };

  protected:

    /** Queues of requests for all the different lookup levels */
    std::list<WalkerState *> stateQueues[MAX_LOOKUP_LEVELS];

    /** Queue of requests that have passed are waiting because the walker is
     * currently busy. */
    std::list<WalkerState *> pendingQueue;

    /** The MMU to forward second stage look upts to */
    Stage2MMU *stage2Mmu;

    /** Port shared by the two table walkers. */
    DmaPort* port;

    /** Master id assigned by the MMU. */
    MasterID masterId;

    /** Indicates whether this table walker is part of the stage 2 mmu */
    const bool isStage2;

    /** TLB that is initiating these table walks */
    TLB *tlb;

    /** Cached copy of the sctlr as it existed when translation began */
    SCTLR sctlr;

    WalkerState *currState;

    /** If a timing translation is currently in progress */
    bool pending;

    /** The number of walks belonging to squashed instructions that can be
     * removed from the pendingQueue per cycle. */
    unsigned numSquashable;

    /** Cached copies of system-level properties */
    bool haveSecurity;
    bool _haveLPAE;
    bool _haveVirtualization;
    uint8_t physAddrRange;
    bool _haveLargeAsid64;

    /** Statistics */
    Stats::Scalar statWalks;
    Stats::Scalar statWalksShortDescriptor;
    Stats::Scalar statWalksLongDescriptor;
    Stats::Vector statWalksShortTerminatedAtLevel;
    Stats::Vector statWalksLongTerminatedAtLevel;
    Stats::Scalar statSquashedBefore;
    Stats::Scalar statSquashedAfter;
    Stats::Histogram statWalkWaitTime;
    Stats::Histogram statWalkServiceTime;
    Stats::Histogram statPendingWalks; // essentially "L" of queueing theory
    Stats::Vector statPageSizes;
    Stats::Vector2d statRequestOrigin;

    mutable unsigned pendingReqs;
    mutable Tick pendingChangeTick;

    static const unsigned REQUESTED = 0;
    static const unsigned COMPLETED = 1;

  public:
   typedef ArmTableWalkerParams Params;
    TableWalker(const Params *p);
    virtual ~TableWalker();

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    void init() override;

    bool haveLPAE() const { return _haveLPAE; }
    bool haveVirtualization() const { return _haveVirtualization; }
    bool haveLargeAsid64() const { return _haveLargeAsid64; }
    /** Checks if all state is cleared and if so, completes drain */
    void completeDrain();
    DrainState drain() override;
    void drainResume() override;

    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    void regStats() override;

    Fault walk(const RequestPtr &req, ThreadContext *tc,
               uint16_t asid, uint8_t _vmid,
               bool _isHyp, TLB::Mode mode, TLB::Translation *_trans,
               bool timing, bool functional, bool secure,
               TLB::ArmTranslationType tranType, bool _stage2Req);

    void setTlb(TLB *_tlb) { tlb = _tlb; }
    TLB* getTlb() { return tlb; }
    void setMMU(Stage2MMU *m, MasterID master_id);
    void memAttrs(ThreadContext *tc, TlbEntry &te, SCTLR sctlr,
                  uint8_t texcb, bool s);
    void memAttrsLPAE(ThreadContext *tc, TlbEntry &te,
                      LongDescriptor &lDescriptor);
    void memAttrsAArch64(ThreadContext *tc, TlbEntry &te,
                         LongDescriptor &lDescriptor);

    static LookupLevel toLookupLevel(uint8_t lookup_level_as_int);

  private:

    void doL1Descriptor();
    void doL1DescriptorWrapper();
    EventFunctionWrapper doL1DescEvent;

    void doL2Descriptor();
    void doL2DescriptorWrapper();
    EventFunctionWrapper doL2DescEvent;

    void doLongDescriptor();

    void doL0LongDescriptorWrapper();
    EventFunctionWrapper doL0LongDescEvent;
    void doL1LongDescriptorWrapper();
    EventFunctionWrapper doL1LongDescEvent;
    void doL2LongDescriptorWrapper();
    EventFunctionWrapper doL2LongDescEvent;
    void doL3LongDescriptorWrapper();
    EventFunctionWrapper doL3LongDescEvent;

    void doLongDescriptorWrapper(LookupLevel curr_lookup_level);
    Event* LongDescEventByLevel[4];

    bool fetchDescriptor(Addr descAddr, uint8_t *data, int numBytes,
        Request::Flags flags, int queueIndex, Event *event,
        void (TableWalker::*doDescriptor)());

    void insertTableEntry(DescriptorBase &descriptor, bool longDescriptor);

    Fault processWalk();
    Fault processWalkLPAE();
    static unsigned adjustTableSizeAArch64(unsigned tsz);
    /// Returns true if the address exceeds the range permitted by the
    /// system-wide setting or by the TCR_ELx IPS/PS setting
    static bool checkAddrSizeFaultAArch64(Addr addr, int currPhysAddrRange);
    Fault processWalkAArch64();
    void processWalkWrapper();
    EventFunctionWrapper doProcessEvent;

    void nextWalk(ThreadContext *tc);

    void pendingChange();

    static uint8_t pageSizeNtoStatBin(uint8_t N);

    Fault testWalk(Addr pa, Addr size, TlbEntry::DomainType domain,
                   LookupLevel lookup_level);
};

} // namespace ArmISA

#endif //__ARCH_ARM_TABLE_WALKER_HH__

