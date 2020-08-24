/*
 * Copyright (c) 2013, 2018-2019 ARM Limited
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

#include "dev/arm/smmu_v3.hh"

#include <cstddef>
#include <cstring>

#include "base/bitfield.hh"
#include "base/cast.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/Checkpoint.hh"
#include "debug/SMMUv3.hh"
#include "dev/arm/smmu_v3_transl.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

SMMUv3::SMMUv3(SMMUv3Params *params) :
    ClockedObject(params),
    system(*params->system),
    requestorId(params->system->getRequestorId(this)),
    requestPort(name() + ".request", *this),
    tableWalkPort(name() + ".walker", *this),
    controlPort(name() + ".control", *this, params->reg_map),
    tlb(params->tlb_entries, params->tlb_assoc, params->tlb_policy),
    configCache(params->cfg_entries, params->cfg_assoc, params->cfg_policy),
    ipaCache(params->ipa_entries, params->ipa_assoc, params->ipa_policy),
    walkCache({ { params->walk_S1L0, params->walk_S1L1,
                  params->walk_S1L2, params->walk_S1L3,
                  params->walk_S2L0, params->walk_S2L1,
                  params->walk_S2L2, params->walk_S2L3 } },
              params->walk_assoc, params->walk_policy),
    tlbEnable(params->tlb_enable),
    configCacheEnable(params->cfg_enable),
    ipaCacheEnable(params->ipa_enable),
    walkCacheEnable(params->walk_enable),
    tableWalkPortEnable(false),
    walkCacheNonfinalEnable(params->wc_nonfinal_enable),
    walkCacheS1Levels(params->wc_s1_levels),
    walkCacheS2Levels(params->wc_s2_levels),
    requestPortWidth(params->request_port_width),
    tlbSem(params->tlb_slots),
    ifcSmmuSem(1),
    smmuIfcSem(1),
    configSem(params->cfg_slots),
    ipaSem(params->ipa_slots),
    walkSem(params->walk_slots),
    requestPortSem(1),
    transSem(params->xlate_slots),
    ptwSem(params->ptw_slots),
    cycleSem(1),
    tlbLat(params->tlb_lat),
    ifcSmmuLat(params->ifc_smmu_lat),
    smmuIfcLat(params->smmu_ifc_lat),
    configLat(params->cfg_lat),
    ipaLat(params->ipa_lat),
    walkLat(params->walk_lat),
    deviceInterfaces(params->device_interfaces),
    commandExecutor(name() + ".cmd_exec", *this),
    regsMap(params->reg_map),
    processCommandsEvent(this)
{
    fatal_if(regsMap.size() != SMMU_REG_SIZE,
        "Invalid register map size: %#x different than SMMU_REG_SIZE = %#x\n",
        regsMap.size(), SMMU_REG_SIZE);

    // Init smmu registers to 0
    memset(&regs, 0, sizeof(regs));

    // Setup RO ID registers
    regs.idr0 = params->smmu_idr0;
    regs.idr1 = params->smmu_idr1;
    regs.idr2 = params->smmu_idr2;
    regs.idr3 = params->smmu_idr3;
    regs.idr4 = params->smmu_idr4;
    regs.idr5 = params->smmu_idr5;
    regs.iidr = params->smmu_iidr;
    regs.aidr = params->smmu_aidr;

    // TODO: At the moment it possible to set the ID registers to hold
    // any possible value. It would be nice to have a sanity check here
    // at construction time in case some idx registers are programmed to
    // store an unallowed values or if the are configuration conflicts.
    warn("SMMUv3 IDx register values unchecked\n");

    for (auto ifc : deviceInterfaces)
        ifc->setSMMU(this);
}

bool
SMMUv3::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(SMMUv3, "[t] requestor resp addr=%#x size=%#x\n",
        pkt->getAddr(), pkt->getSize());

    // @todo: We need to pay for this and not just zero it out
    pkt->headerDelay = pkt->payloadDelay = 0;

    SMMUProcess *proc =
        safe_cast<SMMUProcess *>(pkt->popSenderState());

    runProcessTiming(proc, pkt);

    return true;
}

void
SMMUv3::recvReqRetry()
{
    assert(!packetsToRetry.empty());

    while (!packetsToRetry.empty()) {
        SMMUAction a = packetsToRetry.front();

        assert(a.type==ACTION_SEND_REQ || a.type==ACTION_SEND_REQ_FINAL);

        DPRINTF(SMMUv3, "[t] requestor retr addr=%#x size=%#x\n",
            a.pkt->getAddr(), a.pkt->getSize());

        if (!requestPort.sendTimingReq(a.pkt))
            break;

        packetsToRetry.pop();

        /*
         * ACTION_SEND_REQ_FINAL means that we have just forwarded the packet
         * on the requestor interface; this means that we no longer hold on to
         * that transaction and therefore can accept a new one.
         * If the response port was stalled then unstall it (send retry).
         */
        if (a.type == ACTION_SEND_REQ_FINAL)
            scheduleDeviceRetries();
    }
}

bool
SMMUv3::tableWalkRecvTimingResp(PacketPtr pkt)
{
    DPRINTF(SMMUv3, "[t] requestor HWTW resp addr=%#x size=%#x\n",
        pkt->getAddr(), pkt->getSize());

    // @todo: We need to pay for this and not just zero it out
    pkt->headerDelay = pkt->payloadDelay = 0;

    SMMUProcess *proc =
        safe_cast<SMMUProcess *>(pkt->popSenderState());

    runProcessTiming(proc, pkt);

    return true;
}

void
SMMUv3::tableWalkRecvReqRetry()
{
    assert(tableWalkPortEnable);
    assert(!packetsTableWalkToRetry.empty());

    while (!packetsTableWalkToRetry.empty()) {
        SMMUAction a = packetsTableWalkToRetry.front();

        assert(a.type==ACTION_SEND_REQ);

        DPRINTF(SMMUv3, "[t] requestor HWTW retr addr=%#x size=%#x\n",
            a.pkt->getAddr(), a.pkt->getSize());

        if (!tableWalkPort.sendTimingReq(a.pkt))
            break;

        packetsTableWalkToRetry.pop();
    }
}

void
SMMUv3::scheduleDeviceRetries()
{
    for (auto ifc : deviceInterfaces) {
        ifc->scheduleDeviceRetry();
    }
}

SMMUAction
SMMUv3::runProcess(SMMUProcess *proc, PacketPtr pkt)
{
    if (system.isAtomicMode()) {
        return runProcessAtomic(proc, pkt);
    } else if (system.isTimingMode()) {
        return runProcessTiming(proc, pkt);
    } else {
        panic("Not in timing or atomic mode!");
    }
}

SMMUAction
SMMUv3::runProcessAtomic(SMMUProcess *proc, PacketPtr pkt)
{
    SMMUAction action;
    Tick delay = 0;
    bool finished = false;

    do {
        action = proc->run(pkt);

        switch (action.type) {
            case ACTION_SEND_REQ:
                // Send an MMU initiated request on the table walk port if
                // it is enabled. Otherwise, fall through and handle same
                // as the final ACTION_SEND_REQ_FINAL request.
                if (tableWalkPortEnable) {
                    delay += tableWalkPort.sendAtomic(action.pkt);
                    pkt = action.pkt;
                    break;
                }
                M5_FALLTHROUGH;
            case ACTION_SEND_REQ_FINAL:
                delay += requestPort.sendAtomic(action.pkt);
                pkt = action.pkt;
                break;

            case ACTION_SEND_RESP:
            case ACTION_SEND_RESP_ATS:
            case ACTION_SLEEP:
                finished = true;
                break;

            case ACTION_DELAY:
                delay += action.delay;
                break;

            case ACTION_TERMINATE:
                panic("ACTION_TERMINATE in atomic mode\n");

            default:
                panic("Unknown action\n");
        }
    } while (!finished);

    action.delay = delay;

    return action;
}

SMMUAction
SMMUv3::runProcessTiming(SMMUProcess *proc, PacketPtr pkt)
{
    SMMUAction action = proc->run(pkt);

    switch (action.type) {
        case ACTION_SEND_REQ:
            // Send an MMU initiated request on the table walk port if it is
            // enabled. Otherwise, fall through and handle same as the final
            // ACTION_SEND_REQ_FINAL request.
            if (tableWalkPortEnable) {
                action.pkt->pushSenderState(proc);

                DPRINTF(SMMUv3, "[t] requestor HWTW req  addr=%#x size=%#x\n",
                        action.pkt->getAddr(), action.pkt->getSize());

                if (packetsTableWalkToRetry.empty()
                        && tableWalkPort.sendTimingReq(action.pkt)) {
                    scheduleDeviceRetries();
                } else {
                    DPRINTF(SMMUv3, "[t] requestor HWTW req  needs retry,"
                            " qlen=%d\n", packetsTableWalkToRetry.size());
                    packetsTableWalkToRetry.push(action);
                }

                break;
            }
            M5_FALLTHROUGH;
        case ACTION_SEND_REQ_FINAL:
            action.pkt->pushSenderState(proc);

            DPRINTF(SMMUv3, "[t] requestor req  addr=%#x size=%#x\n",
                    action.pkt->getAddr(), action.pkt->getSize());

            if (packetsToRetry.empty() &&
                requestPort.sendTimingReq(action.pkt)) {
                scheduleDeviceRetries();
            } else {
                DPRINTF(SMMUv3, "[t] requestor req  needs retry, qlen=%d\n",
                        packetsToRetry.size());
                packetsToRetry.push(action);
            }

            break;

        case ACTION_SEND_RESP:
            // @todo: We need to pay for this and not just zero it out
            action.pkt->headerDelay = action.pkt->payloadDelay = 0;

            DPRINTF(SMMUv3, "[t] responder resp addr=%#x size=%#x\n",
                    action.pkt->getAddr(),
                    action.pkt->getSize());

            assert(action.ifc);
            action.ifc->schedTimingResp(action.pkt);

            delete proc;
            break;

        case ACTION_SEND_RESP_ATS:
            // @todo: We need to pay for this and not just zero it out
            action.pkt->headerDelay = action.pkt->payloadDelay = 0;

            DPRINTF(SMMUv3, "[t] ATS responder resp addr=%#x size=%#x\n",
                    action.pkt->getAddr(), action.pkt->getSize());

            assert(action.ifc);
            action.ifc->schedAtsTimingResp(action.pkt);

            delete proc;
            break;

        case ACTION_DELAY:
        case ACTION_SLEEP:
            break;

        case ACTION_TERMINATE:
            delete proc;
            break;

        default:
            panic("Unknown action\n");
    }

    return action;
}

void
SMMUv3::processCommands()
{
    DPRINTF(SMMUv3, "processCommands()\n");

    if (system.isAtomicMode()) {
        SMMUAction a = runProcessAtomic(&commandExecutor, NULL);
        (void) a;
    } else if (system.isTimingMode()) {
        if (!commandExecutor.isBusy())
            runProcessTiming(&commandExecutor, NULL);
    } else {
        panic("Not in timing or atomic mode!");
    }
}

void
SMMUv3::processCommand(const SMMUCommand &cmd)
{
    switch (cmd.dw0.type) {
        case CMD_PRF_CONFIG:
            DPRINTF(SMMUv3, "CMD_PREFETCH_CONFIG - ignored\n");
            break;

        case CMD_PRF_ADDR:
            DPRINTF(SMMUv3, "CMD_PREFETCH_ADDR - ignored\n");
            break;

        case CMD_CFGI_STE: {
            DPRINTF(SMMUv3, "CMD_CFGI_STE sid=%#x\n", cmd.dw0.sid);
            configCache.invalidateSID(cmd.dw0.sid);

            for (auto dev_interface : deviceInterfaces) {
                dev_interface->microTLB->invalidateSID(cmd.dw0.sid);
                dev_interface->mainTLB->invalidateSID(cmd.dw0.sid);
            }
            break;
        }

        case CMD_CFGI_STE_RANGE: {
            const auto range = cmd.dw1.range;
            if (range == 31) {
                // CMD_CFGI_ALL is an alias of CMD_CFGI_STE_RANGE with
                // range = 31
                DPRINTF(SMMUv3, "CMD_CFGI_ALL\n");
                configCache.invalidateAll();

                for (auto dev_interface : deviceInterfaces) {
                    dev_interface->microTLB->invalidateAll();
                    dev_interface->mainTLB->invalidateAll();
                }
            } else {
                DPRINTF(SMMUv3, "CMD_CFGI_STE_RANGE\n");
                const auto start_sid = cmd.dw0.sid & ~((1 << (range + 1)) - 1);
                const auto end_sid = start_sid + (1 << (range + 1)) - 1;
                for (auto sid = start_sid; sid <= end_sid; sid++) {
                    configCache.invalidateSID(sid);

                    for (auto dev_interface : deviceInterfaces) {
                        dev_interface->microTLB->invalidateSID(sid);
                        dev_interface->mainTLB->invalidateSID(sid);
                    }
                }
            }
            break;
        }

        case CMD_CFGI_CD: {
            DPRINTF(SMMUv3, "CMD_CFGI_CD sid=%#x ssid=%#x\n",
                    cmd.dw0.sid, cmd.dw0.ssid);
            configCache.invalidateSSID(cmd.dw0.sid, cmd.dw0.ssid);

            for (auto dev_interface : deviceInterfaces) {
                dev_interface->microTLB->invalidateSSID(
                    cmd.dw0.sid, cmd.dw0.ssid);
                dev_interface->mainTLB->invalidateSSID(
                    cmd.dw0.sid, cmd.dw0.ssid);
            }
            break;
        }

        case CMD_CFGI_CD_ALL: {
            DPRINTF(SMMUv3, "CMD_CFGI_CD_ALL sid=%#x\n", cmd.dw0.sid);
            configCache.invalidateSID(cmd.dw0.sid);

            for (auto dev_interface : deviceInterfaces) {
                dev_interface->microTLB->invalidateSID(cmd.dw0.sid);
                dev_interface->mainTLB->invalidateSID(cmd.dw0.sid);
            }
            break;
        }

        case CMD_TLBI_NH_ALL: {
            DPRINTF(SMMUv3, "CMD_TLBI_NH_ALL vmid=%#x\n", cmd.dw0.vmid);
            for (auto dev_interface : deviceInterfaces) {
                dev_interface->microTLB->invalidateVMID(cmd.dw0.vmid);
                dev_interface->mainTLB->invalidateVMID(cmd.dw0.vmid);
            }
            tlb.invalidateVMID(cmd.dw0.vmid);
            walkCache.invalidateVMID(cmd.dw0.vmid);
            break;
        }

        case CMD_TLBI_NH_ASID: {
            DPRINTF(SMMUv3, "CMD_TLBI_NH_ASID asid=%#x vmid=%#x\n",
                    cmd.dw0.asid, cmd.dw0.vmid);
            for (auto dev_interface : deviceInterfaces) {
                dev_interface->microTLB->invalidateASID(
                    cmd.dw0.asid, cmd.dw0.vmid);
                dev_interface->mainTLB->invalidateASID(
                    cmd.dw0.asid, cmd.dw0.vmid);
            }
            tlb.invalidateASID(cmd.dw0.asid, cmd.dw0.vmid);
            walkCache.invalidateASID(cmd.dw0.asid, cmd.dw0.vmid);
            break;
        }

        case CMD_TLBI_NH_VAA: {
            const Addr addr = cmd.addr();
            DPRINTF(SMMUv3, "CMD_TLBI_NH_VAA va=%#08x vmid=%#x\n",
                    addr, cmd.dw0.vmid);
            for (auto dev_interface : deviceInterfaces) {
                dev_interface->microTLB->invalidateVAA(
                    addr, cmd.dw0.vmid);
                dev_interface->mainTLB->invalidateVAA(
                    addr, cmd.dw0.vmid);
            }
            tlb.invalidateVAA(addr, cmd.dw0.vmid);
            const bool leaf_only = cmd.dw1.leaf ? true : false;
            walkCache.invalidateVAA(addr, cmd.dw0.vmid, leaf_only);
            break;
        }

        case CMD_TLBI_NH_VA: {
            const Addr addr = cmd.addr();
            DPRINTF(SMMUv3, "CMD_TLBI_NH_VA va=%#08x asid=%#x vmid=%#x\n",
                    addr, cmd.dw0.asid, cmd.dw0.vmid);
            for (auto dev_interface : deviceInterfaces) {
                dev_interface->microTLB->invalidateVA(
                    addr, cmd.dw0.asid, cmd.dw0.vmid);
                dev_interface->mainTLB->invalidateVA(
                    addr, cmd.dw0.asid, cmd.dw0.vmid);
            }
            tlb.invalidateVA(addr, cmd.dw0.asid, cmd.dw0.vmid);
            const bool leaf_only = cmd.dw1.leaf ? true : false;
            walkCache.invalidateVA(addr, cmd.dw0.asid, cmd.dw0.vmid,
                                   leaf_only);
            break;
        }

        case CMD_TLBI_S2_IPA: {
            const Addr addr = cmd.addr();
            DPRINTF(SMMUv3, "CMD_TLBI_S2_IPA ipa=%#08x vmid=%#x\n",
                    addr, cmd.dw0.vmid);
            // This does not invalidate TLBs containing
            // combined Stage1 + Stage2 translations, as per the spec.
            ipaCache.invalidateIPA(addr, cmd.dw0.vmid);

            if (!cmd.dw1.leaf)
                walkCache.invalidateVMID(cmd.dw0.vmid);
            break;
        }

        case CMD_TLBI_S12_VMALL: {
            DPRINTF(SMMUv3, "CMD_TLBI_S12_VMALL vmid=%#x\n", cmd.dw0.vmid);
            for (auto dev_interface : deviceInterfaces) {
                dev_interface->microTLB->invalidateVMID(cmd.dw0.vmid);
                dev_interface->mainTLB->invalidateVMID(cmd.dw0.vmid);
            }
            tlb.invalidateVMID(cmd.dw0.vmid);
            ipaCache.invalidateVMID(cmd.dw0.vmid);
            walkCache.invalidateVMID(cmd.dw0.vmid);
            break;
        }

        case CMD_TLBI_NSNH_ALL: {
            DPRINTF(SMMUv3, "CMD_TLBI_NSNH_ALL\n");
            for (auto dev_interface : deviceInterfaces) {
                dev_interface->microTLB->invalidateAll();
                dev_interface->mainTLB->invalidateAll();
            }
            tlb.invalidateAll();
            ipaCache.invalidateAll();
            walkCache.invalidateAll();
            break;
        }

        case CMD_RESUME:
            DPRINTF(SMMUv3, "CMD_RESUME\n");
            panic("resume unimplemented");
            break;

        default:
            warn("Unimplemented command %#x\n", cmd.dw0.type);
            break;
    }
}

const PageTableOps*
SMMUv3::getPageTableOps(uint8_t trans_granule)
{
    static V8PageTableOps4k  ptOps4k;
    static V8PageTableOps16k ptOps16k;
    static V8PageTableOps64k ptOps64k;

    switch (trans_granule) {
    case TRANS_GRANULE_4K:  return &ptOps4k;
    case TRANS_GRANULE_16K: return &ptOps16k;
    case TRANS_GRANULE_64K: return &ptOps64k;
    default:
        panic("Unknown translation granule size %d", trans_granule);
    }
}

Tick
SMMUv3::readControl(PacketPtr pkt)
{
    DPRINTF(SMMUv3, "readControl:  addr=%08x size=%d\n",
            pkt->getAddr(), pkt->getSize());

    int offset = pkt->getAddr() - regsMap.start();
    assert(offset >= 0 && offset < SMMU_REG_SIZE);

    if (inSecureBlock(offset)) {
        warn("smmu: secure registers (0x%x) are not implemented\n",
             offset);
    }

    auto reg_ptr = regs.data + offset;

    switch (pkt->getSize()) {
      case sizeof(uint32_t):
        pkt->setLE<uint32_t>(*reinterpret_cast<uint32_t *>(reg_ptr));
        break;
      case sizeof(uint64_t):
        pkt->setLE<uint64_t>(*reinterpret_cast<uint64_t *>(reg_ptr));
        break;
      default:
        panic("smmu: unallowed access size: %d bytes\n", pkt->getSize());
        break;
    }

    pkt->makeAtomicResponse();

    return 0;
}

Tick
SMMUv3::writeControl(PacketPtr pkt)
{
    int offset = pkt->getAddr() - regsMap.start();
    assert(offset >= 0 && offset < SMMU_REG_SIZE);

    DPRINTF(SMMUv3, "writeControl: addr=%08x size=%d data=%16x\n",
            pkt->getAddr(), pkt->getSize(),
            pkt->getSize() == sizeof(uint64_t) ?
            pkt->getLE<uint64_t>() : pkt->getLE<uint32_t>());

    switch (offset) {
        case offsetof(SMMURegs, cr0):
            assert(pkt->getSize() == sizeof(uint32_t));
            regs.cr0 = regs.cr0ack = pkt->getLE<uint32_t>();
            break;

        case offsetof(SMMURegs, cr1):
        case offsetof(SMMURegs, cr2):
        case offsetof(SMMURegs, strtab_base_cfg):
        case offsetof(SMMURegs, eventq_cons):
        case offsetof(SMMURegs, eventq_irq_cfg1):
        case offsetof(SMMURegs, priq_cons):
            assert(pkt->getSize() == sizeof(uint32_t));
            *reinterpret_cast<uint32_t *>(regs.data + offset) =
                pkt->getLE<uint32_t>();
            break;

        case offsetof(SMMURegs, cmdq_cons):
            assert(pkt->getSize() == sizeof(uint32_t));
            if (regs.cr0 & CR0_CMDQEN_MASK) {
                warn("CMDQ is enabled: ignoring write to CMDQ_CONS\n");
            } else {
                *reinterpret_cast<uint32_t *>(regs.data + offset) =
                    pkt->getLE<uint32_t>();
            }
            break;

        case offsetof(SMMURegs, cmdq_prod):
            assert(pkt->getSize() == sizeof(uint32_t));
            *reinterpret_cast<uint32_t *>(regs.data + offset) =
                pkt->getLE<uint32_t>();
            schedule(processCommandsEvent, nextCycle());
            break;

        case offsetof(SMMURegs, strtab_base):
        case offsetof(SMMURegs, eventq_irq_cfg0):
            assert(pkt->getSize() == sizeof(uint64_t));
            *reinterpret_cast<uint64_t *>(regs.data + offset) =
                pkt->getLE<uint64_t>();
            break;

        case offsetof(SMMURegs, cmdq_base):
            assert(pkt->getSize() == sizeof(uint64_t));
            if (regs.cr0 & CR0_CMDQEN_MASK) {
                warn("CMDQ is enabled: ignoring write to CMDQ_BASE\n");
            } else {
                *reinterpret_cast<uint64_t *>(regs.data + offset) =
                    pkt->getLE<uint64_t>();
                regs.cmdq_cons = 0;
                regs.cmdq_prod = 0;
            }
            break;

        case offsetof(SMMURegs, eventq_base):
            assert(pkt->getSize() == sizeof(uint64_t));
            *reinterpret_cast<uint64_t *>(regs.data + offset) =
                pkt->getLE<uint64_t>();
            regs.eventq_cons = 0;
            regs.eventq_prod = 0;
            break;

        case offsetof(SMMURegs, priq_base):
            assert(pkt->getSize() == sizeof(uint64_t));
            *reinterpret_cast<uint64_t *>(regs.data + offset) =
                pkt->getLE<uint64_t>();
            regs.priq_cons = 0;
            regs.priq_prod = 0;
            break;

        default:
            if (inSecureBlock(offset)) {
                warn("smmu: secure registers (0x%x) are not implemented\n",
                     offset);
            } else {
                warn("smmu: write to read-only/undefined register at 0x%x\n",
                     offset);
            }
    }

    pkt->makeAtomicResponse();

    return 0;
}

bool
SMMUv3::inSecureBlock(uint32_t offs) const
{
    if (offs >= offsetof(SMMURegs, _secure_regs) && offs < SMMU_SECURE_SZ)
        return true;
    else
        return false;
}

void
SMMUv3::init()
{
    // make sure both sides are connected and have the same block size
    if (!requestPort.isConnected())
        fatal("Request port is not connected.\n");

    // If the second request port is connected for the table walks, enable
    // the mode to send table walks through this port instead
    if (tableWalkPort.isConnected())
        tableWalkPortEnable = true;

    // notify the request side  of our address ranges
    for (auto ifc : deviceInterfaces) {
        ifc->sendRange();
    }

    if (controlPort.isConnected())
        controlPort.sendRangeChange();
}

void
SMMUv3::regStats()
{
    ClockedObject::regStats();

    using namespace Stats;

    for (size_t i = 0; i < deviceInterfaces.size(); i++) {
        deviceInterfaces[i]->microTLB->regStats(
            csprintf("%s.utlb%d", name(), i));
        deviceInterfaces[i]->mainTLB->regStats(
            csprintf("%s.maintlb%d", name(), i));
    }

    tlb.regStats(name() + ".tlb");
    configCache.regStats(name() + ".cfg");
    ipaCache.regStats(name() + ".ipa");
    walkCache.regStats(name() + ".walk");

    steL1Fetches
        .name(name() + ".steL1Fetches")
        .desc("STE L1 fetches")
        .flags(pdf);

    steFetches
        .name(name() + ".steFetches")
        .desc("STE fetches")
        .flags(pdf);

    cdL1Fetches
        .name(name() + ".cdL1Fetches")
        .desc("CD L1 fetches")
        .flags(pdf);

    cdFetches
        .name(name() + ".cdFetches")
        .desc("CD fetches")
        .flags(pdf);

    translationTimeDist
        .init(0, 2000000, 2000)
        .name(name() + ".translationTimeDist")
        .desc("Time to translate address")
        .flags(pdf);

    ptwTimeDist
        .init(0, 2000000, 2000)
        .name(name() + ".ptwTimeDist")
        .desc("Time to walk page tables")
        .flags(pdf);
}

DrainState
SMMUv3::drain()
{
    // Wait until the Command Executor is not busy
    if (commandExecutor.isBusy()) {
        return DrainState::Draining;
    }
    return DrainState::Drained;
}

void
SMMUv3::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing SMMUv3\n");

    SERIALIZE_ARRAY(regs.data, sizeof(regs.data) / sizeof(regs.data[0]));
}

void
SMMUv3::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing SMMUv3\n");

    UNSERIALIZE_ARRAY(regs.data, sizeof(regs.data) / sizeof(regs.data[0]));
}

Port&
SMMUv3::getPort(const std::string &name, PortID id)
{
    if (name == "request") {
        return requestPort;
    } else if (name == "walker") {
        return tableWalkPort;
    } else if (name == "control") {
        return controlPort;
    } else {
        return ClockedObject::getPort(name, id);
    }
}

SMMUv3*
SMMUv3Params::create()
{
    return new SMMUv3(this);
}
