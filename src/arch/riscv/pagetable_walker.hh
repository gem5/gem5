/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * Copyright (c) 2020 Barkhausen Institut
 * All rights reserved.
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

#ifndef __ARCH_RISCV_TABLE_WALKER_HH__
#define __ARCH_RISCV_TABLE_WALKER_HH__

#include <vector>

#include "arch/generic/mmu.hh"
#include "arch/riscv/page_size.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/pma_checker.hh"
#include "arch/riscv/pmp.hh"
#include "arch/riscv/tlb.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "params/RiscvPagetableWalker.hh"
#include "sim/clocked_object.hh"
#include "sim/faults.hh"
#include "sim/system.hh"

#include "debug/PageTableWalker.hh"

namespace gem5
{

class ThreadContext;

namespace RiscvISA
{

class Walker : public ClockedObject
{
  protected:
    // Port for accessing memory
    class WalkerPort : public RequestPort
    {
      public:
        WalkerPort(const std::string &_name, Walker *_walker)
            : RequestPort(_name), walker(_walker)
        {}

      protected:
        Walker *walker;

        bool recvTimingResp(PacketPtr pkt);
        void recvReqRetry();
    };

    friend class WalkerPort;
    WalkerPort port;

    // State to track each walk of the page table
    class WalkerState
    {
        friend class Walker;

      private:
        struct WalkFlags
        {
            bool doEndWalk = false;
            bool doTLBInsert = false;
            bool doWrite = false;
            bool pteIsLeaf = false;
        };

        enum WalkType

        {
            OneStage,
            TwoStage,
            GstageOnly,
        };

        enum State
        {
            Ready,
            Waiting,
            Translate,
        };

      protected:
        Walker *walker;
        WalkType walkType;
        ThreadContext *tc;
        RequestPtr req;
        State state;
        State nextState;
        State gstate;
        State nextgState;
        int level;
        int glevel;
        unsigned inflight;
        TlbEntry gresult;
        TlbEntry entry;
        PacketPtr read;
        std::vector<PacketPtr> writes;
        Fault timingFault;
        BaseMMU::Translation *translation;
        BaseMMU::Mode mode;
        MemAccessInfo memaccess;
        XlateStage curstage;
        SATP satp;
        SATP hgatp;
        STATUS status;
        PrivilegeMode pmode;
        bool functional;
        bool timing;
        bool retrying;
        bool started;
        bool squashed;

      public:
        WalkerState(Walker *_walker, BaseMMU::Translation *_translation,
                    const RequestPtr &_req, bool _isFunctional = false)
            : walker(_walker),
              req(_req),
              state(Ready),
              nextState(Ready),
              level(0),
              glevel(0),
              inflight(0),
              translation(_translation),
              functional(_isFunctional),
              timing(false),
              retrying(false),
              started(false),
              squashed(false)
        {}
        void initState(ThreadContext *_tc, BaseMMU::Mode _mode,
                       bool _isTiming = false);

        Fault walk();
        Fault startFunctional(Addr &addr, unsigned &logBytes);
        bool recvPacket(PacketPtr pkt);
        unsigned numInflight() const;
        bool isRetrying();
        bool wasStarted();
        bool isTiming();
        void retry();
        void squash();
        std::string
        name() const
        {
            return walker->name();
        }

      private:
        template <class T>
        Fault
        templateCheckPTEPermissions(T pte, WalkFlags &stepWalkFlags, int level)
        {

            if (!pte.v || (!pte.r && pte.w)) {
                stepWalkFlags.doEndWalk = true;
                return pageFault();
            }

            // If read-bit or exec-bit, then PTE is a leaf
            if (pte.r || pte.x) {
                stepWalkFlags.pteIsLeaf = true;
                stepWalkFlags.doEndWalk = true;

                Fault fault = walker->tlb->checkPermissions(
                    tc, memaccess, entry.vaddr, mode, pte, gresult.vaddr,
                    curstage);

                if (fault != NoFault) {
                    return fault;
                }

                // ppn fragments that correspond to unused
                // vpn fragments have to be all zeroes
                // Otherwise, throw a pagefault
                if (level >= 1 && pte.ppn0 != 0) {
                    return pageFault();
                } else if (level == 2 && pte.ppn1 != 0) {
                    return pageFault();
                }

                if (pte.n && (pte.ppn0 & mask(NapotShift)) != 8) {
                    DPRINTF(PageTableWalker, "SVNAPOT PTE has wrong encoding, \
                    raising PF\n");
                    fault = pageFault();
                }

                // Check if we need to write
                if (!pte.a) {
                    pte.a = 1;
                    stepWalkFlags.doWrite = true;
                }
                if (!pte.d && mode == BaseMMU::Write) {
                    pte.d = 1;
                    stepWalkFlags.doWrite = true;
                }
            }

            return NoFault;
        }

        Fault checkPTEPermissions(PTES pte, WalkFlags &stepWalkFlags,
                                  int level);

        Addr setupWalk(Addr vaddr);

        template <class T>

        Fault
        templateStepWalk(PacketPtr &write)
        {
            assert(state != Ready && state != Waiting);

            Fault fault = NoFault;
            write = NULL;
            T pte = read->getLE<uint64_t>();
            Addr nextRead = 0;

            // walk flags are initialized to false
            WalkFlags stepWalkFlags;

            DPRINTF(PageTableWalker, "Got level%d PTE: %#x\n", level, pte);

            // step 2:
            // Performing PMA/PMP checks on physical address of PTE

            // Effective privilege mode for pmp checks for page table
            // walks is S mode according to specs
            fault = walker->pmp->pmpCheck(read->req, BaseMMU::Read,
                                          RiscvISA::PrivilegeMode::PRV_S, tc,
                                          entry.vaddr);

            if (fault == NoFault) {
                fault =
                    walker->pma->check(read->req, BaseMMU::Read, entry.vaddr);
            }

            if (fault == NoFault) {

                fault = checkPTEPermissions(pte, stepWalkFlags, level);

                if (fault == NoFault && stepWalkFlags.pteIsLeaf) {

                    if (stepWalkFlags.doWrite) {

                        // this read will eventually become write
                        // if doWrite is True

                        fault = walker->pmp->pmpCheck(
                            read->req, BaseMMU::Write, pmode, tc, entry.vaddr);

                        if (fault == NoFault) {
                            fault = walker->pma->check(
                                read->req, BaseMMU::Write, entry.vaddr);
                        }
                    }

                    // perform next step only if pmp checks pass
                    if (fault == NoFault) {
                        // TLB inserts are OK for single stage walks
                        // For two-stage, FIRST_STAGE will reach here just once
                        // but the TLB insertion is done in walkTwoStage()
                        if (walkType == OneStage ||
                            (walkType == TwoStage &&
                             curstage == FIRST_STAGE)) {
                            // Fill in TLB entry
                            // Check if N (contig bit) is set, if yes we have
                            // a 64K page mapping (SVNAPOT Extension)
                            assert(!(pte.n) || level == 0);
                            entry.pte = pte;
                            entry.paddr = (pte.n) ? pte.ppn & ~mask(NapotShift)
                                                  : pte.ppn;

                            entry.logBytes =
                                (pte.n)
                                    ? PageShift + NapotShift
                                    : PageShift +
                                          (level * getLEVEL_BITS(satp.mode));

                            // Only truncate the address in non-two stage walks
                            // The truncation for two-stage is done in
                            // walkTwoStage()
                            if (walkType != TwoStage) {
                                entry.logBytes =
                                    PageShift +
                                    (level * getLEVEL_BITS(satp.mode));
                                entry.vaddr &= ~((1 << entry.logBytes) - 1);
                            }

                            // put it non-writable into the TLB to detect
                            // writes and redo the page table walk in order
                            // to update the dirty flag
                            if (!pte.d && mode != BaseMMU::Write) {
                                entry.pte.w = 0;
                            }

                            // Don't do TLB insert here when ending TwoStage.
                            // An additional GStage is done in walkTwoStage()
                            // and then we insert.
                            // Also don't insert on special_access
                            if (walkType != TwoStage &&
                                !memaccess.bypassTLB()) {
                                stepWalkFlags.doTLBInsert = true;
                            }
                        }

                        // Update statistics for completed page walks
                        if (level == 1) {
                            walker->pagewalkerstats.num_2mb_walks++;
                        }
                        if (level == 0) {
                            walker->pagewalkerstats.num_4kb_walks++;
                        }
                        DPRINTF(PageTableWalker,
                                "#1 leaf node at level %d, with vpn %#x\n",
                                level, entry.vaddr);
                    }
                }
                // PTE is not a leaf and there was no fault, decrement level
                else if (fault == NoFault) {
                    Addr shift, idx;
                    level--;
                    if (level < 0) {
                        stepWalkFlags.doEndWalk = true;
                        fault = pageFault();
                    } else {
                        shift = (PageShift + getLEVEL_BITS(satp.mode) * level);
                        idx = (entry.vaddr >> shift) &
                              mask(getLEVEL_BITS(satp.mode));
                        nextRead =
                            (pte.ppn << PageShift) + (idx * sizeof(pte));
                        nextState = Translate;
                    }
                }
            } else {
                stepWalkFlags.doEndWalk = true;
            }

            PacketPtr oldRead = read;
            Request::Flags flags = oldRead->req->getFlags();

            if (stepWalkFlags.doEndWalk) {
                // If we need to write, adjust the read packet to write the
                // modified value back to memory. Use a fresh packet so any
                // responder flags set during the read do not leak into the
                // write request.
                PacketPtr new_write = nullptr;
                if (!functional && stepWalkFlags.doWrite &&
                    !(walkType == TwoStage && curstage == FIRST_STAGE)) {
                    new_write = new Packet(oldRead, true, true);
                    if (oldRead->hasSharers()) {
                        new_write->setHasSharers();
                    }
                    new_write->setLE<uint64_t>(pte);
                    new_write->cmd = MemCmd::WriteReq;
                    read = NULL;
                    delete oldRead;
                    oldRead = nullptr;
                }
                write = new_write;

                if (stepWalkFlags.doTLBInsert) {
                    if (!functional && !memaccess.bypassTLB()) {
                        Addr vpn = getVPNFromVAddr(entry.vaddr, satp.mode);
                        walker->tlb->insert(vpn, entry);
                    }
                }
                endWalk();
            } else {
                // If we didn't return, we're setting up another read.
                RequestPtr request = std::make_shared<Request>(
                    nextRead, oldRead->getSize(), flags, walker->requestorId);

                delete oldRead;
                oldRead = nullptr;

                read = new Packet(request, MemCmd::ReadReq);
                read->allocate();
            }

            return fault;
        }

        Fault stepWalk(PacketPtr &write);

        template <class T>
        Fault
        templateStepWalkGStage(PacketPtr &write)
        {
            assert(gstate != Ready && gstate != Waiting);

            Fault fault = NoFault;
            write = NULL;
            T pte = read->getLE<uint64_t>();
            Addr nextRead = 0;

            // walk flags are initialized to false
            WalkFlags stepWalkFlags;

            DPRINTF(PageTableWalker, "[GSTAGE]: Got level%d PTE: %#x\n",
                    glevel, pte);

            // step 2:
            // Performing PMA/PMP checks on physical address of PTE

            // Effective privilege mode for pmp checks for page table
            // walks is S mode according to specs
            fault = walker->pmp->pmpCheck(read->req, BaseMMU::Read,
                                          RiscvISA::PrivilegeMode::PRV_S, tc,
                                          entry.vaddr);

            if (fault == NoFault) {
                fault =
                    walker->pma->check(read->req, BaseMMU::Read, entry.vaddr);
            }

            if (fault == NoFault) {

                fault = checkPTEPermissions(pte, stepWalkFlags, glevel);

                if (fault == NoFault && stepWalkFlags.pteIsLeaf) {

                    if (stepWalkFlags.doWrite) {

                        // this read will eventually become write
                        // if doWrite is True

                        fault = walker->pmp->pmpCheck(
                            read->req, BaseMMU::Write, pmode, tc, entry.vaddr);

                        if (fault == NoFault) {
                            fault = walker->pma->check(
                                read->req, BaseMMU::Write, entry.vaddr);
                        }
                    }

                    // perform next step only if pmp checks pass
                    if (fault == NoFault) {
                        // Only change TLB entry if walk is
                        // GStageOnly. Otherwise the entry is produced
                        // at the end of the two-stage walk.
                        // (we do not currently store intermediate GStage
                        // results)
                        if (walkType == GstageOnly) {
                            // Check if N (contig bit) is set, if yes we have
                            // a 64K page mapping (SVNAPOT Extension)
                            assert(!(pte.n) || glevel == 0);
                            entry.pte = pte;
                            entry.paddr = (pte.n) ? pte.ppn & ~mask(NapotShift)
                                                  : pte.ppn;

                            entry.logBytes =
                                (pte.n)
                                    ? PageShift + NapotShift
                                    : PageShift +
                                          (glevel * getLEVEL_BITS(satp.mode));

                            entry.vaddr &= ~((1 << entry.logBytes) - 1);

                            // put it non-writable into the TLB to detect
                            // writes and redo the page table walk in order
                            // to update the dirty flag.
                            if (!pte.d && mode != BaseMMU::Write) {
                                entry.pte.w = 0;
                            }

                            // Also don't do TLB inserts on special_access
                            if (!memaccess.bypassTLB()) {
                                stepWalkFlags.doTLBInsert = true;
                            }
                        } else {
                            gresult.logBytes =
                                PageShift +
                                (glevel * getLEVEL_BITS(satp.mode));
                            gresult.paddr = pte.ppn;
                            gresult.vaddr &= ~((1 << entry.logBytes) - 1);
                            gresult.pte = pte;
                        }

                        // Update statistics for completed page walks
                        if (glevel == 1) {
                            walker->pagewalkerstats.num_2mb_walks++;
                        }
                        if (glevel == 0) {
                            if (pte.n) {
                                walker->pagewalkerstats.num_64kb_walks++;
                            } else {
                                walker->pagewalkerstats.num_4kb_walks++;
                            }
                        }
                        DPRINTF(PageTableWalker,
                                "[GSTAGE] #1 leaf node at level %d, with vpn "
                                "%#x\n",
                                glevel, gresult.vaddr);
                    }
                } else if (fault == NoFault) {
                    Addr shift, idx;
                    glevel--;
                    if (glevel < 0) {
                        stepWalkFlags.doEndWalk = true;
                        fault = pageFault();
                    } else {
                        shift =
                            (PageShift + getLEVEL_BITS(satp.mode) * glevel);
                        idx = (gresult.vaddr >> shift) &
                              mask(getLEVEL_BITS(satp.mode));
                        nextRead =
                            (pte.ppn << PageShift) + (idx * sizeof(pte));
                        nextgState = Translate;
                    }
                }
            } else {
                stepWalkFlags.doEndWalk = true;
            }

            PacketPtr oldRead = read;
            Request::Flags flags = oldRead->req->getFlags();

            if (stepWalkFlags.doEndWalk) {
                // If we need to write, adjust the read packet to write the
                // modified value back to memory. Use a fresh packet so
                // responder state from the read does not carry into the write
                // request.
                PacketPtr new_write = nullptr;
                if (!functional && stepWalkFlags.doWrite) {
                    new_write = new Packet(oldRead, true, true);
                    if (oldRead->hasSharers()) {
                        new_write->setHasSharers();
                    }
                    new_write->setLE<uint64_t>(pte);
                    new_write->cmd = MemCmd::WriteReq;
                    read = NULL;
                    delete oldRead;
                    oldRead = nullptr;
                }
                write = new_write;

                if (stepWalkFlags.doTLBInsert) {
                    if (!functional && !memaccess.bypassTLB()) {
                        // This TLB insertion should only be reachable
                        // for GstageOnly walks. Two stage walks insert
                        // in walkTwoStage.
                        assert(walkType == GstageOnly);
                        Addr vpn = getVPNFromVAddr(entry.vaddr, satp.mode);
                        walker->tlb->insert(vpn, entry);
                    }
                }
                endWalk();
            } else {
                // If we didn't return, we're setting up another read.
                RequestPtr request = std::make_shared<Request>(
                    nextRead, oldRead->getSize(), flags, walker->requestorId);

                delete oldRead;
                oldRead = nullptr;

                read = new Packet(request, MemCmd::ReadReq);
                read->allocate();
            }

            return fault;
        }

        Fault stepWalkGStage(PacketPtr &write);
        Fault walkGStage(Addr guest_paddr, Addr &host_paddr);
        Fault walkOneStage(Addr vaddr);
        Fault walkTwoStage(Addr vaddr);
        void sendPackets();
        void endWalk();
        Fault pageFault();
        Fault guestToHostPage(Addr vaddr);
        PacketPtr createReqPacket(Addr paddr, MemCmd cmd, size_t bytes);
    };

    friend class WalkerState;
    // State for timing and atomic accesses (need multiple per walker in
    // the case of multiple outstanding requests in timing mode)
    std::list<WalkerState *> currStates;
    // State for functional accesses (only need one of these per walker)
    WalkerState funcState;

    struct WalkerSenderState : public Packet::SenderState
    {
        WalkerState *senderWalk;
        WalkerSenderState(WalkerState *_senderWalk) : senderWalk(_senderWalk)
        {}
    };

  public:
    // Kick off the state machine.
    Fault start(ThreadContext *_tc, BaseMMU::Translation *translation,
                const RequestPtr &req, BaseMMU::Mode mode,
                TlbEntry *result_entry = nullptr);
    Fault startFunctional(ThreadContext *_tc, Addr &addr, unsigned &logBytes,
                          BaseMMU::Mode mode);
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  protected:
    // The TLB we're supposed to load.
    TLB *tlb;
    System *sys;
    BasePMAChecker *pma;
    PMP *pmp;
    RequestorID requestorId;

    // The number of outstanding walks that can be squashed per cycle.
    unsigned numSquashable;

    // Wrapper for checking for squashes before starting a translation.
    void startWalkWrapper();

    /**
     * Event used to call startWalkWrapper.
     **/
    EventFunctionWrapper startWalkWrapperEvent;

    // Functions for dealing with packets.
    bool recvTimingResp(PacketPtr pkt);
    void recvReqRetry();
    bool sendTiming(WalkerState *sendingState, PacketPtr pkt);

    struct PagewalkerStats : public statistics::Group
    {
        PagewalkerStats(statistics::Group *parent);

        statistics::Scalar num_4kb_walks;
        statistics::Scalar num_64kb_walks;
        statistics::Scalar num_2mb_walks;
        statistics::Scalar num_1gb_walks;
        statistics::Scalar num_512gb_walks;

    } pagewalkerstats;

  public:
    void
    setTLB(TLB *_tlb)
    {
        tlb = _tlb;
    }

    using Params = RiscvPagetableWalkerParams;

    Walker(const Params &params)
        : ClockedObject(params),
          port(name() + ".port", this),
          funcState(this, NULL, NULL, true),
          tlb(NULL),
          sys(params.system),
          pma(params.pma_checker),
          pmp(params.pmp),
          requestorId(sys->getRequestorId(this)),
          numSquashable(params.num_squash_per_cycle),
          startWalkWrapperEvent([this] { startWalkWrapper(); }, name()),
          pagewalkerstats(this)
    {}
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_PAGE_TABLE_WALKER_HH__
