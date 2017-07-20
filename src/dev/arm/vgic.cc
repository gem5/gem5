/*
 * Copyright (c) 2013 ARM Limited
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
 * Authors: Matt Evans
 */

#include "dev/arm/vgic.hh"

#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "debug/VGIC.hh"
#include "dev/arm/base_gic.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

VGic::VGic(const Params *p)
    : PioDevice(p), platform(p->platform), gic(p->gic), vcpuAddr(p->vcpu_addr),
      hvAddr(p->hv_addr), pioDelay(p->pio_delay),
      maintInt(p->ppint)
{
    for (int x = 0; x < VGIC_CPU_MAX; x++) {
        postVIntEvent[x] = new EventFunctionWrapper(
            [this, x]{ processPostVIntEvent(x); },
            "Post VInterrupt to CPU");
        maintIntPosted[x] = false;
        vIntPosted[x] = false;
    }
    assert(sys->numRunningContexts() <= VGIC_CPU_MAX);
}

VGic::~VGic()
{
    for (int x = 0; x < VGIC_CPU_MAX; x++)
        delete postVIntEvent[x];
}

Tick
VGic::read(PacketPtr pkt)
{
    Addr addr = pkt->getAddr();

    if (addr >= vcpuAddr && addr < vcpuAddr + GICV_SIZE)
        return readVCpu(pkt);
    else if (addr >= hvAddr && addr < hvAddr + GICH_REG_SIZE)
        return readCtrl(pkt);
    else
        panic("Read to unknown address %#x\n", pkt->getAddr());
}

Tick
VGic::write(PacketPtr pkt)
{
    Addr addr = pkt->getAddr();

    if (addr >= vcpuAddr && addr < vcpuAddr + GICV_SIZE)
        return writeVCpu(pkt);
    else if (addr >= hvAddr && addr < hvAddr + GICH_REG_SIZE)
        return writeCtrl(pkt);
    else
        panic("Write to unknown address %#x\n", pkt->getAddr());
}

Tick
VGic::readVCpu(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - vcpuAddr;

    ContextID ctx_id = pkt->req->contextId();
    assert(ctx_id < VGIC_CPU_MAX);
    struct vcpuIntData *vid = &vcpuData[ctx_id];

    DPRINTF(VGIC, "VGIC VCPU read register %#x\n", daddr);

    switch (daddr) {
      case GICV_CTLR:
        pkt->set<uint32_t>(vid->vctrl);
        break;
      case GICV_IAR: {
          int i = findHighestPendingLR(vid);
          if (i < 0 || !vid->vctrl.En) {
              pkt->set<uint32_t>(1023); // "No int" marker
          } else {
              ListReg *lr = &vid->LR[i];

              pkt->set<uint32_t>(lr->VirtualID |
                                 (((int)lr->CpuID) << 10));
              // We don't support auto-EOI of HW interrupts via real GIC!
              // Fortunately, KVM doesn't use this.  How about Xen...? Ulp!
              if (lr->HW)
                  panic("VGIC does not support 'HW' List Register feature (LR %#x)!\n",
                        *lr);
              lr->State = LR_ACTIVE;
              DPRINTF(VGIC, "Consumed interrupt %d (cpu%d) from LR%d (EOI%d)\n",
                      lr->VirtualID, lr->CpuID, i, lr->EOI);
          }
      } break;
      default:
        panic("VGIC VCPU read of bad address %#x\n", daddr);
    }

    updateIntState(ctx_id);

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
VGic::readCtrl(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - hvAddr;

    ContextID ctx_id = pkt->req->contextId();

    DPRINTF(VGIC, "VGIC HVCtrl read register %#x\n", daddr);

    /* Munge the address: 0-0xfff is the usual space banked by requester CPU.
     * Anything > that is 0x200-sized slices of 'per CPU' regs.
     */
    if (daddr & ~0x1ff) {
        ctx_id = (daddr >> 9);
        if (ctx_id > 8)
            panic("VGIC: Weird unbanked hv ctrl address %#x!\n", daddr);
        daddr &= ~0x1ff;
    }
    assert(ctx_id < VGIC_CPU_MAX);
    struct vcpuIntData *vid = &vcpuData[ctx_id];

    switch (daddr) {
      case GICH_HCR:
        pkt->set<uint32_t>(vid->hcr);
        break;

      case GICH_VTR:
        pkt->set<uint32_t>(0x44000000 | (NUM_LR - 1));
        break;

      case GICH_VMCR:
        pkt->set<uint32_t>(
            ((uint32_t)vid->VMPriMask << 27) |
            ((uint32_t)vid->VMBP << 21) |
            ((uint32_t)vid->VMABP << 18) |
            ((uint32_t)vid->VEM << 9) |
            ((uint32_t)vid->VMCBPR << 4) |
            ((uint32_t)vid->VMFiqEn << 3) |
            ((uint32_t)vid->VMAckCtl << 2) |
            ((uint32_t)vid->VMGrp1En << 1) |
            ((uint32_t)vid->VMGrp0En << 0)
            );
        break;

      case GICH_MISR:
        pkt->set<uint32_t>(getMISR(vid));
        break;

      case GICH_EISR0:
        pkt->set<uint32_t>(vid->eisr & 0xffffffff);
        break;

      case GICH_EISR1:
        pkt->set<uint32_t>(vid->eisr >> 32);
        break;

      case GICH_ELSR0: {
          uint32_t bm = 0;
          for (int i = 0; i < ((NUM_LR < 32) ? NUM_LR : 32); i++) {
              if (!vid->LR[i].State)
                  bm |= 1 << i;
          }
          pkt->set<uint32_t>(bm);
      } break;

      case GICH_ELSR1: {
          uint32_t bm = 0;
          for (int i = 32; i < NUM_LR; i++) {
              if (!vid->LR[i].State)
                  bm |= 1 << (i-32);
          }
          pkt->set<uint32_t>(bm);
      } break;

      case GICH_APR0:
        warn_once("VGIC GICH_APR read!\n");
        pkt->set<uint32_t>(0);
        break;

      case GICH_LR0:
      case GICH_LR1:
      case GICH_LR2:
      case GICH_LR3:
        pkt->set<uint32_t>(vid->LR[(daddr - GICH_LR0) >> 2]);
        break;

      default:
        panic("VGIC HVCtrl read of bad address %#x\n", daddr);
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
VGic::writeVCpu(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - vcpuAddr;

    ContextID ctx_id = pkt->req->contextId();
    assert(ctx_id < VGIC_CPU_MAX);
    struct vcpuIntData *vid = &vcpuData[ctx_id];

    DPRINTF(VGIC, "VGIC VCPU write register %#x <= %#x\n", daddr, pkt->get<uint32_t>());

    switch (daddr) {
      case GICV_CTLR:
        vid->vctrl = pkt->get<uint32_t>();
        break;
      case GICV_PMR:
        vid->VMPriMask = pkt->get<uint32_t>();
        break;
      case GICV_EOIR: {
          // We don't handle the split EOI-then-DIR mode.  Linux (guest)
          // doesn't need it though.
          assert(!vid->vctrl.EOImode);
          uint32_t w = pkt->get<uint32_t>();
          unsigned int virq = w & 0x3ff;
          unsigned int vcpu = (w >> 10) & 7;
          int i = findLRForVIRQ(vid, virq, vcpu);
          if (i < 0) {
              DPRINTF(VGIC, "EOIR: No LR for irq %d(cpu%d)\n", virq, vcpu);
          } else {
              DPRINTF(VGIC, "EOIR: Found LR%d for irq %d(cpu%d)\n", i, virq, vcpu);
              ListReg *lr = &vid->LR[i];
              lr->State = 0;
              // Maintenance interrupt -- via eisr -- is flagged when
              // LRs have EOI=1 and State=INVALID!
          }
      } break;
      default:
        panic("VGIC VCPU write %#x to unk address %#x\n", pkt->get<uint32_t>(), daddr);
    }

    // This updates the EISRs and flags IRQs:
    updateIntState(ctx_id);

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
VGic::writeCtrl(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - hvAddr;

    ContextID ctx_id = pkt->req->contextId();

    DPRINTF(VGIC, "VGIC HVCtrl write register %#x <= %#x\n", daddr, pkt->get<uint32_t>());

    /* Munge the address: 0-0xfff is the usual space banked by requester CPU.
     * Anything > that is 0x200-sized slices of 'per CPU' regs.
     */
    if (daddr & ~0x1ff) {
        ctx_id = (daddr >> 9);
        if (ctx_id > 8)
            panic("VGIC: Weird unbanked hv ctrl address %#x!\n", daddr);
        daddr &= ~0x1ff;
    }
    assert(ctx_id < VGIC_CPU_MAX);
    struct vcpuIntData *vid = &vcpuData[ctx_id];

    switch (daddr) {
      case GICH_HCR:
        vid->hcr = pkt->get<uint32_t>();
        // update int state
        break;

      case GICH_VMCR: {
          uint32_t d = pkt->get<uint32_t>();
          vid->VMPriMask = d >> 27;
          vid->VMBP = (d >> 21) & 7;
          vid->VMABP = (d >> 18) & 7;
          vid->VEM = (d >> 9) & 1;
          vid->VMCBPR = (d >> 4) & 1;
          vid->VMFiqEn = (d >> 3) & 1;
          vid->VMAckCtl = (d >> 2) & 1;
          vid->VMGrp1En = (d >> 1) & 1;
          vid->VMGrp0En = d & 1;
      } break;

      case GICH_APR0:
        warn_once("VGIC GICH_APR0 written, ignored\n");
        break;

      case GICH_LR0:
      case GICH_LR1:
      case GICH_LR2:
      case GICH_LR3:
        vid->LR[(daddr - GICH_LR0) >> 2] = pkt->get<uint32_t>();
        // update int state
        break;

      default:
        panic("VGIC HVCtrl write to bad address %#x\n", daddr);
    }

    updateIntState(ctx_id);

    pkt->makeAtomicResponse();
    return pioDelay;
}


uint32_t
VGic::getMISR(struct vcpuIntData *vid)
{
    return (!!vid->hcr.VGrp1DIE && !vid->VMGrp1En ? 0x80 : 0) |
        (!!vid->hcr.VGrp1EIE &&  vid->VMGrp1En ? 0x40 : 0) |
        (!!vid->hcr.VGrp0DIE && !vid->VMGrp0En ? 0x20 : 0) |
        (!!vid->hcr.VGrp0EIE &&  vid->VMGrp0En ? 0x10 : 0) |
        (!!vid->hcr.NPIE && !lrPending(vid) ? 0x08 : 0) |
        (!!vid->hcr.LRENPIE && vid->hcr.EOICount ? 0x04 : 0) |
        (!!vid->hcr.UIE && lrValid(vid) <= 1 ? 0x02 : 0) |
        (vid->eisr ? 0x01 : 0);
}

void
VGic::postVInt(uint32_t cpu, Tick when)
{
    DPRINTF(VGIC, "Posting VIRQ to %d\n", cpu);
    if (!(postVIntEvent[cpu]->scheduled()))
        eventq->schedule(postVIntEvent[cpu], when);
}

void
VGic::unPostVInt(uint32_t cpu)
{
    DPRINTF(VGIC, "Unposting VIRQ to %d\n", cpu);
    platform->intrctrl->clear(cpu, ArmISA::INT_VIRT_IRQ, 0);
}

void
VGic::processPostVIntEvent(uint32_t cpu)
{
     platform->intrctrl->post(cpu, ArmISA::INT_VIRT_IRQ, 0);
}


void
VGic::postMaintInt(uint32_t cpu)
{
    DPRINTF(VGIC, "Posting maintenance PPI to GIC/cpu%d\n", cpu);
    // Linux DT configures this as Level.
    gic->sendPPInt(maintInt, cpu);
}

void
VGic::unPostMaintInt(uint32_t cpu)
{
    DPRINTF(VGIC, "Unposting maintenance PPI to GIC/cpu%d\n", cpu);
    gic->clearPPInt(maintInt, cpu);
}

/* Update state (in general); something concerned with ctx_id has changed.
 * This may raise a maintenance interrupt.
 */
void
VGic::updateIntState(ContextID ctx_id)
{
    // @todo This should update APRs!

    // Build EISR contents:
    // (Cached so that regs can read them without messing about again)
    struct vcpuIntData *tvid = &vcpuData[ctx_id];

    tvid->eisr = 0;
    for (int i = 0; i < NUM_LR; i++) {
        if (!tvid->LR[i].State && tvid->LR[i].EOI) {
            tvid->eisr |= 1 << i;
        }
    }

    assert(sys->numRunningContexts() <= VGIC_CPU_MAX);
    for (int i = 0; i < sys->numRunningContexts(); i++) {
        struct vcpuIntData *vid = &vcpuData[i];
        // Are any LRs active that weren't before?
        if (!vIntPosted[i]) {
            if (lrPending(vid) && vid->vctrl.En) {
                vIntPosted[i] = true;
                postVInt(i, curTick() + 1);
            }
        } else if (!lrPending(vid)) {
            vIntPosted[i] = false;
            unPostVInt(i);
        }

        // Any maintenance ints to send?
        if (!maintIntPosted[i]) {
            if (vid->hcr.En && getMISR(vid)) {
                maintIntPosted[i] = true;
                postMaintInt(i);
            }
        } else {
            if (!vid->hcr.En || !getMISR(vid)) {
                unPostMaintInt(i);
                maintIntPosted[i] = false;
            }
        }
    }
}

AddrRangeList
VGic::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(hvAddr, GICH_REG_SIZE));
    ranges.push_back(RangeSize(vcpuAddr, GICV_SIZE));
    return ranges;
}

void
VGic::serialize(CheckpointOut &cp) const
{
    Tick interrupt_time[VGIC_CPU_MAX];
    for (uint32_t cpu = 0; cpu < VGIC_CPU_MAX; cpu++) {
        interrupt_time[cpu] = 0;
        if (postVIntEvent[cpu]->scheduled()) {
            interrupt_time[cpu] = postVIntEvent[cpu]->when();
        }
    }

    DPRINTF(Checkpoint, "Serializing VGIC\n");

    SERIALIZE_ARRAY(interrupt_time, VGIC_CPU_MAX);
    SERIALIZE_ARRAY(maintIntPosted, VGIC_CPU_MAX);
    SERIALIZE_ARRAY(vIntPosted, VGIC_CPU_MAX);
    SERIALIZE_SCALAR(vcpuAddr);
    SERIALIZE_SCALAR(hvAddr);
    SERIALIZE_SCALAR(pioDelay);
    SERIALIZE_SCALAR(maintInt);

    for (uint32_t cpu = 0; cpu < VGIC_CPU_MAX; cpu++)
        vcpuData[cpu].serializeSection(cp, csprintf("vcpuData%d", cpu));
}

void
VGic::vcpuIntData::serialize(CheckpointOut &cp) const
{
    uint32_t vctrl_val = vctrl;
    SERIALIZE_SCALAR(vctrl_val);
    uint32_t hcr_val = hcr;
    SERIALIZE_SCALAR(hcr_val);
    uint64_t eisr_val = eisr;
    SERIALIZE_SCALAR(eisr_val);
    uint8_t VMGrp0En_val = VMGrp0En;
    SERIALIZE_SCALAR(VMGrp0En_val);
    uint8_t VMGrp1En_val = VMGrp1En;
    SERIALIZE_SCALAR(VMGrp1En_val);
    uint8_t VMAckCtl_val = VMAckCtl;
    SERIALIZE_SCALAR(VMAckCtl_val);
    uint8_t VMFiqEn_val = VMFiqEn;
    SERIALIZE_SCALAR(VMFiqEn_val);
    uint8_t VMCBPR_val = VMCBPR;
    SERIALIZE_SCALAR(VMCBPR_val);
    uint8_t VEM_val = VEM;
    SERIALIZE_SCALAR(VEM_val);
    uint8_t VMABP_val = VMABP;
    SERIALIZE_SCALAR(VMABP_val);
    uint8_t VMBP_val = VMBP;
    SERIALIZE_SCALAR(VMBP_val);
    uint8_t VMPriMask_val = VMPriMask;
    SERIALIZE_SCALAR(VMPriMask_val);

    for (int i = 0; i < NUM_LR; i++) {
        ScopedCheckpointSection sec_lr(cp, csprintf("LR%d", i));
        paramOut(cp, "lr", LR[i]);
    }
}

void VGic::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Arm GIC\n");

    Tick interrupt_time[VGIC_CPU_MAX];
    UNSERIALIZE_ARRAY(interrupt_time, VGIC_CPU_MAX);
    for (uint32_t cpu = 0; cpu < VGIC_CPU_MAX; cpu++) {
        if (interrupt_time[cpu])
            schedule(postVIntEvent[cpu], interrupt_time[cpu]);

        vcpuData[cpu].unserializeSection(cp, csprintf("vcpuData%d", cpu));
    }
    UNSERIALIZE_ARRAY(maintIntPosted, VGIC_CPU_MAX);
    UNSERIALIZE_ARRAY(vIntPosted, VGIC_CPU_MAX);
    UNSERIALIZE_SCALAR(vcpuAddr);
    UNSERIALIZE_SCALAR(hvAddr);
    UNSERIALIZE_SCALAR(pioDelay);
    UNSERIALIZE_SCALAR(maintInt);
}

void
VGic::vcpuIntData::unserialize(CheckpointIn &cp)
{
    paramIn(cp, "vctrl_val", vctrl);
    paramIn(cp, "hcr_val", hcr);
    paramIn(cp, "eisr_val", eisr);
    paramIn(cp, "VMGrp0En_val", VMGrp0En);
    paramIn(cp, "VMGrp1En_val", VMGrp1En);
    paramIn(cp, "VMAckCtl_val", VMAckCtl);
    paramIn(cp, "VMFiqEn_val", VMFiqEn);
    paramIn(cp, "VMCBPR_val", VMCBPR);
    paramIn(cp, "VEM_val", VEM);
    paramIn(cp, "VMABP_val", VMABP);
    paramIn(cp, "VMPriMask_val", VMPriMask);

    for (int i = 0; i < NUM_LR; i++) {
        ScopedCheckpointSection sec_lr(cp, csprintf("LR%d", i));
        paramIn(cp, "lr", LR[i]);
    }
}

VGic *
VGicParams::create()
{
    return new VGic(this);
}
