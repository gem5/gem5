/*
 * Copyright (c) 2010-2016, 2019, 2021-2024 Arm Limited
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
 */

#ifndef __ARCH_ARM_TABLE_WALKER_HH__
#define __ARCH_ARM_TABLE_WALKER_HH__

#include <list>

#include "arch/arm/faults.hh"
#include "arch/arm/mmu.hh"
#include "arch/arm/regs/misc.hh"
#include "arch/arm/system.hh"
#include "arch/arm/tlb.hh"
#include "arch/arm/types.hh"
#include "arch/generic/mmu.hh"
#include "mem/packet_queue.hh"
#include "mem/qport.hh"
#include "mem/request.hh"
#include "params/ArmTableWalker.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

class ThreadContext;

namespace ArmISA {
class Translation;
class TLB;

class TableWalker : public ClockedObject
{
    using LookupLevel = enums::ArmLookupLevel;

  public:
    class WalkerState;

    class DescriptorBase
    {
      public:
        DescriptorBase() : lookupLevel(LookupLevel::L0) {}

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
        virtual uint8_t* getRawPtr() = 0;
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

    class L1Descriptor : public DescriptorBase
    {
      public:
        /** Type of page table entry ARM DDI 0406B: B3-8*/
        enum EntryType
        {
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
            lookupLevel = LookupLevel::L1;
        }

        uint8_t*
        getRawPtr() override
        {
            return reinterpret_cast<uint8_t*>(&data);
        }

        uint64_t
        getRawData() const override
        {
            return (data);
        }

        std::string
        dbgHeader() const override
        {
            return "Inserting Section Descriptor into TLB\n";
        }

        uint8_t
        offsetBits() const override
        {
            return 20;
        }

        EntryType
        type() const
        {
            return (EntryType)(data & 0x3);
        }

        /** Is the page a Supersection (16 MiB)?*/
        bool
        supersection() const
        {
            return bits(data, 18);
        }

        /** Return the physcal address of the entry, bits in position*/
        Addr
        paddr() const
        {
            if (supersection())
                panic("Super sections not implemented\n");
            return mbits(data, 31, 20);
        }

        /** Return the physcal address of the entry, bits in position*/
        Addr
        paddr(Addr va) const
        {
            if (supersection())
                panic("Super sections not implemented\n");
            return mbits(data, 31, 20) | mbits(va, 19, 0);
        }

        /** Return the physical frame, bits shifted right */
        Addr
        pfn() const override
        {
            if (supersection())
                panic("Super sections not implemented\n");
            return bits(data, 31, 20);
        }

        /** Is the translation global (no asid used)? */
        bool
        global(WalkerState *currState) const override
        {
            return !bits(data, 17);
        }

        /** Is the translation not allow execution? */
        bool
        xn() const override
        {
            return bits(data, 4);
        }

        /** Three bit access protection flags */
        uint8_t
        ap() const override
        {
            return (bits(data, 15) << 2) | bits(data, 11, 10);
        }

        /** Domain Client/Manager: ARM DDI 0406B: B3-31 */
        TlbEntry::DomainType
        domain() const override
        {
            return static_cast<TlbEntry::DomainType>(bits(data, 8, 5));
        }

        /** Address of L2 descriptor if it exists */
        Addr
        l2Addr() const
        {
            return mbits(data, 31, 10);
        }

        /** Memory region attributes: ARM DDI 0406B: B3-32.
         * These bits are largly ignored by M5 and only used to
         * provide the illusion that the memory system cares about
         * anything but cachable vs. uncachable.
         */
        uint8_t
        texcb() const override
        {
            return bits(data, 2) | bits(data, 3) << 1 | bits(data, 14, 12) << 2;
        }

        /** If the section is shareable. See texcb() comment. */
        bool
        shareable() const override
        {
            return bits(data, 16);
        }

        /** Set access flag that this entry has been touched. Mark
         * the entry as requiring a writeback, in the future.
         */
        void
        setAp0()
        {
            data |= 1 << 10;
            _dirty = true;
        }

        /** This entry needs to be written back to memory */
        bool
        dirty() const
        {
            return _dirty;
        }

        /**
         * Returns true if this entry targets the secure physical address
         * map.
         */
        bool
        secure(bool have_security, WalkerState *currState) const override
        {
            if (have_security && currState->secureLookup) {
                if (type() == PageTable)
                    return !bits(data, 3);
                else
                    return !bits(data, 19);
            }
            return false;
        }
    };

    /** Level 2 page table descriptor */
    class L2Descriptor : public DescriptorBase
    {
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
            lookupLevel = LookupLevel::L2;
        }

        L2Descriptor(L1Descriptor &parent) : data(0), l1Parent(&parent),
                                             _dirty(false)
        {
            lookupLevel = LookupLevel::L2;
        }

        uint8_t*
        getRawPtr() override
        {
            return reinterpret_cast<uint8_t*>(&data);
        }

        uint64_t
        getRawData() const override
        {
            return (data);
        }

        std::string
        dbgHeader() const override
        {
            return "Inserting L2 Descriptor into TLB\n";
        }

        TlbEntry::DomainType
        domain() const override
        {
            return l1Parent->domain();
        }

        bool
        secure(bool have_security, WalkerState *currState) const override
        {
            return l1Parent->secure(have_security, currState);
        }

        uint8_t
        offsetBits() const override
        {
            return large() ? 16 : 12;
        }

        /** Is the entry invalid */
        bool
        invalid() const
        {
            return bits(data, 1, 0) == 0;
        }

        /** What is the size of the mapping? */
        bool
        large() const
        {
            return bits(data, 1) == 0;
        }

        /** Is execution allowed on this mapping? */
        bool
        xn() const override
        {
            return large() ? bits(data, 15) : bits(data, 0);
        }

        /** Is the translation global (no asid used)? */
        bool
        global(WalkerState *currState) const override
        {
            return !bits(data, 11);
        }

        /** Three bit access protection flags */
        uint8_t
        ap() const override
        {
           return bits(data, 5, 4) | (bits(data, 9) << 2);
        }

        /** Memory region attributes: ARM DDI 0406B: B3-32 */
        uint8_t
        texcb() const override
        {
            return large() ?
                (bits(data, 2) | (bits(data, 3) << 1) | (bits(data, 14, 12) << 2)) :
                (bits(data, 2) | (bits(data, 3) << 1) | (bits(data, 8, 6) << 2));
        }

        /** Return the physical frame, bits shifted right */
        Addr
        pfn() const override
        {
            return large() ? bits(data, 31, 16) : bits(data, 31, 12);
        }

        /** Return complete physical address given a VA */
        Addr
        paddr(Addr va) const
        {
            if (large())
                return mbits(data, 31, 16) | mbits(va, 15, 0);
            else
                return mbits(data, 31, 12) | mbits(va, 11, 0);
        }

        /** If the section is shareable. See texcb() comment. */
        bool
        shareable() const override
        {
            return bits(data, 10);
        }

        /** Set access flag that this entry has been touched. Mark
         * the entry as requiring a writeback, in the future.
         */
        void
        setAp0()
        {
            data |= 1 << 4;
            _dirty = true;
        }

        /** This entry needs to be written back to memory */
        bool
        dirty() const
        {
            return _dirty;
        }

    };

    /** Long-descriptor format (LPAE) */
    class LongDescriptor : public DescriptorBase
    {
      public:
        /** Descriptor type */
        enum EntryType
        {
            Invalid,
            Table,
            Block,
            Page
        };

        LongDescriptor()
          : data(0), _dirty(false), aarch64(false), grainSize(Grain4KB),
            physAddrRange(0)
        {}

        /** The raw bits of the entry */
        uint64_t data;

        /** This entry has been modified (access flag set) and needs to be
         * written back to memory */
        bool _dirty;

        /** True if the current lookup is performed in AArch64 state */
        bool aarch64;

        /** Width of the granule size in bits */
        GrainSize grainSize;

        uint8_t physAddrRange;

        uint8_t*
        getRawPtr() override
        {
            return reinterpret_cast<uint8_t*>(&data);
        }

        uint64_t
        getRawData() const override
        {
            return (data);
        }

        std::string
        dbgHeader() const override
        {
            switch (type()) {
              case LongDescriptor::Page:
                assert(lookupLevel == LookupLevel::L3);
                return "Inserting Page descriptor into TLB\n";
              case LongDescriptor::Block:
                assert(lookupLevel < LookupLevel::L3);
                return "Inserting Block descriptor into TLB\n";
              case LongDescriptor::Table:
                return "Inserting Table descriptor into TLB\n";
              default:
                panic("Trying to insert and invalid descriptor\n");
            }
        }

        /**
         * Returns true if this entry targets the secure physical address
         * map.
         */
        bool
        secure(bool have_security, WalkerState *currState) const override
        {
            if (type() == Block || type() == Page) {
                return have_security &&
                    (currState->secureLookup && !bits(data, 5));
            } else {
                return have_security && currState->secureLookup;
            }
        }

        /** Return the descriptor type */
        EntryType
        type() const
        {
            switch (bits(data, 1, 0)) {
              case 0x1:
                // In AArch64 blocks are not allowed at L0 for the
                // 4 KiB granule and at L1 for 16/64 KiB granules
                switch (grainSize) {
                  case Grain4KB:
                    if (lookupLevel == LookupLevel::L0 ||
                        lookupLevel == LookupLevel::L3)
                        return Invalid;
                    else
                        return Block;

                  case Grain16KB:
                    if (lookupLevel == LookupLevel::L2)
                        return Block;
                    else
                        return Invalid;

                  case Grain64KB:
                    // With Armv8.2-LPA (52bit PA) L1 Block descriptors
                    // are allowed for 64KiB granule
                    if ((lookupLevel == LookupLevel::L1 && physAddrRange == 52) ||
                        lookupLevel == LookupLevel::L2)
                        return Block;
                    else
                        return Invalid;

                  default:
                    return Invalid;
                }
              case 0x3:
                return lookupLevel == LookupLevel::L3 ? Page : Table;
              default:
                return Invalid;
            }
        }

        /** Return the bit width of the page/block offset */
        uint8_t
        offsetBits() const override
        {
            if (type() == Block) {
                switch (grainSize) {
                    case Grain4KB:
                        return lookupLevel == LookupLevel::L1 ?
                            30 /* 1 GiB */ : 21 /* 2 MiB */;
                    case Grain16KB:
                        return 25  /* 32 MiB */;
                    case Grain64KB:
                        return lookupLevel == LookupLevel::L1 ?
                            42 /* 4 TiB */ : 29 /* 512 MiB */;
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
            } else if (type() == Table) {
                const auto* ptops = getPageTableOps(grainSize);
                return ptops->walkBits(lookupLevel);
            }
            panic("AArch64 page table entry must be block or page\n");
        }

        /** Return the physical frame, bits shifted right */
        Addr
        pfn() const override
        {
            return paddr() >> offsetBits();
        }

        /** Return the physical address of the entry */
        Addr
        paddr() const
        {
            Addr addr = 0;
            if (aarch64) {
                addr = mbits(data, 47, offsetBits());
                if (physAddrRange == 52 && grainSize == Grain64KB) {
                    addr |= bits(data, 15, 12) << 48;
                }
            } else {
                addr = mbits(data, 39, offsetBits());
            }
            return addr;
        }

        /** Return the address of the next page table */
        Addr
        nextTableAddr() const
        {
            assert(type() == Table);
            Addr table_address = 0;
            if (aarch64) {
                table_address = mbits(data, 47, grainSize);
                // Using 52bit if Armv8.2-LPA is implemented
                if (physAddrRange == 52 && grainSize == Grain64KB)
                    table_address |= bits(data, 15, 12) << 48;
            } else {
                table_address = mbits(data, 39, 12);
            }

            return table_address;
        }

        /** Return the address of the next descriptor */
        Addr
        nextDescAddr(Addr va) const
        {
            assert(type() == Table);
            Addr pa = 0;
            if (aarch64) {
                int stride = grainSize - 3;
                int va_lo = stride * (3 - (lookupLevel + 1)) + grainSize;
                int va_hi = va_lo + stride - 1;
                pa = nextTableAddr() | (bits(va, va_hi, va_lo) << 3);
            } else {
                if (lookupLevel == LookupLevel::L1)
                    pa = nextTableAddr() | (bits(va, 29, 21) << 3);
                else  // lookupLevel == L2
                    pa = nextTableAddr() | (bits(va, 20, 12) << 3);
            }
            return pa;
        }

        /** Is execution allowed on this mapping? */
        bool
        xn() const override
        {
            assert(type() == Block || type() == Page);
            return bits(data, 54);
        }

        /** Is privileged execution allowed on this mapping? (LPAE only) */
        bool
        pxn() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 53);
        }

        /** Contiguous hint bit. */
        bool
        contiguousHint() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 52);
        }

        /** Is the translation global (no asid used)? */
        bool
        global(WalkerState *currState) const override
        {
            assert(currState && (type() == Block || type() == Page));
            if (!currState->aarch64 && (currState->isSecure &&
                                        !currState->secureLookup)) {
                return false;  // ARM ARM issue C B3.6.3
            } else if (currState->aarch64) {
                if (!MMU::hasUnprivRegime(currState->regime)) {
                    // By default translations are treated as global
                    // in AArch64 for regimes without an unpriviledged
                    // component
                    return true;
                } else if (currState->isSecure && !currState->secureLookup) {
                    return false;
                }
            }
            return !bits(data, 11);
        }

        /** Returns true if the access flag (AF) is set. */
        bool
        af() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 10);
        }

        /** 2-bit shareability field */
        uint8_t
        sh() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 9, 8);
        }

        /** 2-bit access protection flags */
        uint8_t
        ap() const override
        {
            assert(type() == Block || type() == Page);
            // Long descriptors only support the AP[2:1] scheme
            return bits(data, 7, 6);
        }

        /** Read/write access protection flag */
        bool
        rw() const
        {
            assert(type() == Block || type() == Page);
            return !bits(data, 7);
        }

        /** User/privileged level access protection flag */
        bool
        user() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 6);
        }

        /** Return the AP bits as compatible with the AP[2:0] format.  Utility
         * function used to simplify the code in the TLB for performing
         * permission checks. */
        static uint8_t
        ap(bool rw, bool user)
        {
            return ((!rw) << 2) | (user << 1);
        }

        TlbEntry::DomainType
        domain() const override
        {
            // Long-desc. format only supports Client domain
            return TlbEntry::DomainType::Client;
        }

        /** Attribute index */
        uint8_t
        attrIndx() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 4, 2);
        }

        /** Memory attributes, only used by stage 2 translations */
        uint8_t
        memAttr() const
        {
            assert(type() == Block || type() == Page);
            return bits(data, 5, 2);
        }

        /** Set access flag that this entry has been touched.  Mark the entry as
         * requiring a writeback, in the future. */
        void
        setAf()
        {
            data |= 1 << 10;
            _dirty = true;
        }

        /** This entry needs to be written back to memory */
        bool
        dirty() const
        {
            return _dirty;
        }

        /** Whether the subsequent levels of lookup are secure */
        bool
        secureTable() const
        {
            assert(type() == Table);
            return !bits(data, 63);
        }

        /** Two bit access protection flags for subsequent levels of lookup */
        uint8_t
        apTable() const
        {
            assert(type() == Table);
            return bits(data, 62, 61);
        }

        /** R/W protection flag for subsequent levels of lookup */
        uint8_t
        rwTable() const
        {
            assert(type() == Table);
            return !bits(data, 62);
        }

        /** User/privileged mode protection flag for subsequent levels of
         * lookup */
        uint8_t
        userTable() const
        {
            assert(type() == Table);
            return !bits(data, 61);
        }

        /** Is execution allowed on subsequent lookup levels? */
        bool
        xnTable() const
        {
            assert(type() == Table);
            return bits(data, 60);
        }

        /** Is privileged execution allowed on subsequent lookup levels? */
        bool
        pxnTable() const
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

        /** Current translation regime */
        TranslationRegime regime;

        /** Current physical address range in bits */
        int physAddrRange;

        /** Request that is currently being serviced */
        RequestPtr req;

        /** Initial walk entry allowing to skip lookup levels */
        TlbEntry walkEntry;

        /** ASID that we're servicing the request under */
        uint16_t asid;
        vmid_t vmid;

        /** Translation state for delayed requests */
        BaseMMU::Translation *transState;

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
        union
        {
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
        /** Whether lookups should be treated as using the secure state.
         * This is usually the same as isSecure, but can be set to false by the
         * long descriptor table attributes. */
        bool secureLookup = false;

        /** True if table walks are uncacheable (for table descriptors) */
        bool isUncacheable;

        /** Helper variables used to implement hierarchical access permissions
         * when the long-desc. format is used. */
        struct LongDescData
        {
            bool rwTable = false;
            bool userTable = false;
            bool xnTable = false;
            bool pxnTable = false;
        };
        std::optional<LongDescData> longDescData;

        /** Hierarchical access permission disable */
        bool hpd;

        uint8_t sh;
        uint8_t irgn;
        uint8_t orgn;

        /** Flag indicating if a second stage of lookup is required */
        bool stage2Req;

        /** A pointer to the stage 2 translation that's in progress */
        BaseMMU::Translation *stage2Tran;

        /** If the mode is timing or atomic */
        bool timing;

        /** If the atomic mode should be functional */
        bool functional;

        /** Save mode for use in delayed response */
        BaseMMU::Mode mode;

        /** The translation type that has been requested */
        MMU::ArmTranslationType tranType;

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

    class TableWalkerState : public Packet::SenderState
    {
      public:
        Tick delay = 0;
        Event *event = nullptr;
    };

    class Port : public QueuedRequestPort
    {
      public:
        Port(TableWalker& _walker);

        void sendFunctionalReq(const RequestPtr &req, uint8_t *data);
        void sendAtomicReq(const RequestPtr &req, uint8_t *data, Tick delay);
        void sendTimingReq(const RequestPtr &req, uint8_t *data, Tick delay,
            Event *event);

        bool recvTimingResp(PacketPtr pkt) override;

      private:
        void handleRespPacket(PacketPtr pkt, Tick delay=0);
        void handleResp(TableWalkerState *state, Addr addr,
                        Addr size, Tick delay=0);

        PacketPtr createPacket(const RequestPtr &req, uint8_t *data,
                               Tick delay, Event *event);

      private:
        TableWalker& owner;

        /** Packet queue used to store outgoing requests. */
        ReqPacketQueue reqQueue;

        /** Packet queue used to store outgoing snoop responses. */
        SnoopRespPacketQueue snoopRespQueue;
    };

    /** This translation class is used to trigger the data fetch once a timing
        translation returns the translated physical address */
    class Stage2Walk : public BaseMMU::Translation
    {
      private:
        uint8_t      *data;
        int          numBytes;
        RequestPtr   req;
        Event        *event;
        TableWalker  &parent;
        Addr         oVAddr;
        BaseMMU::Mode mode;
        MMU::ArmTranslationType tranType;

      public:
        Fault fault;

        Stage2Walk(TableWalker &_parent, uint8_t *_data, Event *_event,
                   Addr vaddr, BaseMMU::Mode mode,
                   MMU::ArmTranslationType tran_type);

        void markDelayed() {}

        void finish(const Fault &fault, const RequestPtr &req,
            ThreadContext *tc, BaseMMU::Mode mode);

        void
        setVirt(Addr vaddr, int size, Request::Flags flags,
                int requestorId)
        {
            numBytes = size;
            req->setVirt(vaddr, size, flags, requestorId, 0);
        }

        void translateTiming(ThreadContext *tc);
    };

    Fault readDataUntimed(ThreadContext *tc, Addr vaddr, Addr desc_addr,
                          uint8_t *data, int num_bytes, Request::Flags flags,
                          BaseMMU::Mode mode, MMU::ArmTranslationType tran_type,
                          bool functional);
    void readDataTimed(ThreadContext *tc, Addr desc_addr,
                       Stage2Walk *translation, int num_bytes,
                       Request::Flags flags);

  protected:

    /** Queues of requests for all the different lookup levels */
    std::list<WalkerState *> stateQueues[LookupLevel::Num_ArmLookupLevel];

    /** Queue of requests that have passed are waiting because the walker is
     * currently busy. */
    std::list<WalkerState *> pendingQueue;

    /** The MMU to forward second stage look upts to */
    MMU *mmu;

    /** Requestor id assigned by the MMU. */
    RequestorID requestorId;

    /** Port shared by the two table walkers. */
    Port* port;

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
    const ArmRelease *release;
    uint8_t _physAddrRange;
    bool _haveLargeAsid64;

    /** Statistics */
    struct TableWalkerStats : public statistics::Group
    {
        TableWalkerStats(statistics::Group *parent);
        statistics::Scalar walks;
        statistics::Scalar walksShortDescriptor;
        statistics::Scalar walksLongDescriptor;
        statistics::Vector walksShortTerminatedAtLevel;
        statistics::Vector walksLongTerminatedAtLevel;
        statistics::Scalar squashedBefore;
        statistics::Scalar squashedAfter;
        statistics::Histogram walkWaitTime;
        statistics::Histogram walkServiceTime;
        // Essentially "L" of queueing theory
        statistics::Histogram pendingWalks;
        statistics::Vector pageSizes;
        statistics::Vector2d requestOrigin;
    } stats;

    mutable unsigned pendingReqs;
    mutable Tick pendingChangeTick;

    static const unsigned REQUESTED = 0;
    static const unsigned COMPLETED = 1;

  public:
    PARAMS(ArmTableWalker);
    TableWalker(const Params &p);
    virtual ~TableWalker();

    bool haveLargeAsid64() const { return _haveLargeAsid64; }
    uint8_t physAddrRange() const { return _physAddrRange; }
    /** Checks if all state is cleared and if so, completes drain */
    void completeDrain();
    DrainState drain() override;
    void drainResume() override;

    gem5::Port &getPort(const std::string &if_name,
                    PortID idx=InvalidPortID) override;

    Port &getTableWalkerPort();

    Fault walk(const RequestPtr &req, ThreadContext *tc,
               uint16_t asid, vmid_t _vmid,
               BaseMMU::Mode mode, BaseMMU::Translation *_trans,
               bool timing, bool functional, bool secure,
               MMU::ArmTranslationType tran_type, bool stage2,
               const TlbEntry *walk_entry);

    void setMmu(MMU *_mmu);
    void setTlb(TLB *_tlb) { tlb = _tlb; }
    TLB* getTlb() { return tlb; }
    void memAttrs(ThreadContext *tc, TlbEntry &te, SCTLR sctlr,
                  uint8_t texcb, bool s);
    void memAttrsLPAE(ThreadContext *tc, TlbEntry &te,
                      LongDescriptor &lDescriptor);
    void memAttrsAArch64(ThreadContext *tc, TlbEntry &te,
                         LongDescriptor &lDescriptor);
    void memAttrsWalkAArch64(TlbEntry &te);

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

    void fetchDescriptor(Addr desc_addr,
        DescriptorBase &descriptor, int num_bytes,
        Request::Flags flags, LookupLevel lookup_lvl, Event *event,
        void (TableWalker::*doDescriptor)());

    Fault generateLongDescFault(ArmFault::FaultSource src);

    void insertTableEntry(DescriptorBase &descriptor, bool longDescriptor);
    void insertPartialTableEntry(LongDescriptor &descriptor);

    /** Returns a tuple made of:
     * 1) The address of the first page table
     * 2) The address of the first descriptor within the table
     * 3) The page table level
     */
    std::tuple<Addr, Addr, LookupLevel> walkAddresses(
        Addr ttbr, GrainSize tg, int tsz, int pa_range);

    Fault processWalk();
    Fault processWalkLPAE();

    bool checkVAddrSizeFaultAArch64(Addr addr, int top_bit,
        GrainSize granule, int tsz, bool low_range);

    /// Returns true if the address exceeds the range permitted by the
    /// system-wide setting or by the TCR_ELx IPS/PS setting
    bool checkAddrSizeFaultAArch64(Addr addr, int pa_range);

    /// Returns true if the table walk should be uncacheable
    bool uncacheableWalk() const;

    Fault processWalkAArch64();
    void processWalkWrapper();
    EventFunctionWrapper doProcessEvent;

    void nextWalk(ThreadContext *tc);

    void pendingChange();

    /** Timing mode: saves the currState into the stateQueues */
    void stashCurrState(int queue_idx);

    static uint8_t pageSizeNtoStatBin(uint8_t N);

    void mpamTagTableWalk(RequestPtr &req) const;

  public: /* Testing */
    TlbTestInterface *test;

    void setTestInterface(TlbTestInterface *ti);

    Fault testWalk(const RequestPtr &walk_req, TlbEntry::DomainType domain,
                   LookupLevel lookup_level);
};

} // namespace ArmISA
} // namespace gem5

#endif //__ARCH_ARM_TABLE_WALKER_HH__
