/*
 * Copyright (c) 2024 Arm Limited
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

#include <optional>

#include "arch/arm/mpam.hh"

#include "arch/arm/regs/misc.hh"
#include "arch/arm/regs/misc_accessors.hh"
#include "arch/arm/regs/misc_types.hh"
#include "arch/arm/system.hh"
#include "arch/arm/types.hh"
#include "arch/arm/utility.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/MPAM.hh"

namespace gem5::ArmISA::mpam
{

std::unique_ptr<ExtensionBase>
PartitionFieldExtension::clone() const
{
    return std::make_unique<PartitionFieldExtension>(*this);
}

uint64_t
PartitionFieldExtension::getPartitionID() const
{
    return this->_partitionID;
}

uint64_t
PartitionFieldExtension::getPartitionMonitoringID() const
{
    return this->_partitionMonitoringID;
}

bool
PartitionFieldExtension::getMpamNS() const
{
    return this->_ns;
}

void
PartitionFieldExtension::setPartitionID(uint64_t id)
{
    this->_partitionID = id;
}

void
PartitionFieldExtension::setPartitionMonitoringID(uint64_t id)
{
    this->_partitionMonitoringID = id;
}

void
PartitionFieldExtension::setMpamNS(bool ns)
{
    this->_ns = ns;
}


namespace
{

using namespace misc_regs;

static PartID
getPARTID(ThreadContext *tc, ExceptionLevel el, bool ind)
{
    MPAM reg = readRegister<MpamAccessor>(tc, el);
    return ind ? reg.partidI : reg.partidD;
}

static PMG
getPMG(ThreadContext *tc, ExceptionLevel el, bool ind)
{
    MPAM reg = readRegister<MpamAccessor>(tc, el);
    return ind ? reg.pmgI : reg.pmgD;
}

static bool
useVirtualPartitions(ThreadContext *tc, ExceptionLevel el, MPAMIDR mpamidr)
{
    const MPAMHCR mpamhcr = tc->readMiscReg(MISCREG_MPAMHCR_EL2);
     return mpamidr.hasHcr && EL2Enabled(tc) &&
        ((el == EL0 && !ELIsInHost(tc, EL0) && mpamhcr.el0Vpmen) || // EL0 case
         (el == EL1 && mpamhcr.el1Vpmen));                          // EL1 case
}

static PartID
mapVpmv(ThreadContext *tc, PartID vpartid)
{
    uint8_t reg_index = vpartid / 4;
    uint8_t reg_field = vpartid % 4;

    // Register field size in bits
    size_t reg_field_size = sizeof(PartID) * 8;

    // LSB of the register field (Every field is 16bits)
    uint8_t lsb = reg_field * reg_field_size;
    uint8_t msb = lsb + reg_field_size - 1;

    const RegVal vpmv = tc->readMiscReg(MISCREG_MPAMVPM0_EL2 + reg_index);
    return bits(vpmv, msb ,lsb);
}

static std::optional<PartID>
virtToPhysPart(ThreadContext *tc, PartID vpartid, MPAMIDR mpamidr)
{
    // vpmrMax refers to the register index. Extract vpartid max
    const uint8_t vpartid_max = (mpamidr.vpmrMax << 2) + 3;
    const RegVal mpam_vpmv = tc->readMiscReg(MISCREG_MPAMVPMV_EL2);

    if (vpartid > vpartid_max) {
        vpartid = vpartid % (vpartid_max + 1);
    }

    PartID phys_partid = 0;
    if (bits(mpam_vpmv, vpartid)) {
        // Valid mapping entry for virtual partition vpartid
        phys_partid =  mapVpmv(tc, vpartid);
    } else if (bits(mpam_vpmv, 0)) {
        // Default virtual partition valid
        phys_partid = mapVpmv(tc, 0);
    } else {
        // Error
        return std::nullopt;
    }

    return phys_partid > mpamidr.partidMax ?
        std::nullopt : std::make_optional(phys_partid);
}

static std::optional<PartID>
genPARTID(ThreadContext *tc, ExceptionLevel el, bool ind)
{
    const MPAMIDR mpamidr = tc->readMiscReg(MISCREG_MPAMIDR_EL1);
    auto partid = getPARTID(tc, el, ind);

    if (partid > mpamidr.partidMax) {
        return std::nullopt;
    } else if (useVirtualPartitions(tc, el, mpamidr)) {
        return virtToPhysPart(tc, partid, mpamidr);
    } else {
        return partid;
    }
}

static std::optional<PMG>
genPMG(ThreadContext *tc, ExceptionLevel el, bool ind)
{
    const MPAMIDR mpamidr = tc->readMiscReg(MISCREG_MPAMIDR_EL1);
    PMG pgroup = getPMG(tc, el, ind);
    return pgroup > mpamidr.pmgMax ? std::nullopt : std::make_optional(pgroup);
}

static bool
isEnabled(ThreadContext *tc)
{
    MPAM reg = readRegister<MpamAccessor>(tc, ArmSystem::highestEL(tc));
    return reg.mpamEn;
}

static std::shared_ptr<PartitionFieldExtension>
genExtensionDefault()
{
    // tag with partID data
    auto ext = std::make_shared<PartitionFieldExtension>();
    ext->setPartitionID(DEFAULT_PARTITION_ID);
    ext->setPartitionMonitoringID(DEFAULT_PARTITION_MONITORING_ID);
    return ext;
}

static std::shared_ptr<PartitionFieldExtension>
genExtension(ThreadContext *tc, bool ind)
{
    ExceptionLevel curr_el = currEL(tc);
    const MPAMHCR mpamhcr = tc->readMiscReg(MISCREG_MPAMHCR_EL2);
    const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);

    bool gstplk = curr_el == EL0 && EL2Enabled(tc) &&
        mpamhcr.gstappPlk && !hcr.tge;
    if (gstplk) {
        curr_el = EL1;
    }

    // tag with partID data
    auto ext = std::make_shared<
        PartitionFieldExtension>();

    auto part_id = genPARTID(tc, curr_el, ind).value_or(
        DEFAULT_PARTITION_ID);
    auto part_mon_id = genPMG(tc, curr_el, ind).value_or(
        DEFAULT_PARTITION_MONITORING_ID);

    ext->setPartitionID(part_id);
    ext->setPartitionMonitoringID(part_mon_id);
    return ext;
}

} // namespace

void
tagRequest(ThreadContext *tc, const RequestPtr &req, bool ind)
{
    if (!HaveExt(tc, ArmExtension::FEAT_MPAM) || !isEnabled(tc))
        return;

    const MPAMIDR mpamidr = tc->readMiscReg(MISCREG_MPAMIDR_EL1);
    const MPAM mpam3 = tc->readMiscReg(MISCREG_MPAM3_EL3);
    auto ext = mpamidr.hasSdeflt && mpam3.el3.sdeflt && isSecure(tc) ?
        genExtensionDefault() :
        genExtension(tc, ind);

    DPRINTFS(MPAM, tc->getCpuPtr(),
             "MPAM Tagging req %#x => PART_ID: %d, PART_MON_ID: %d\n",
             req->getPaddr(),
             ext->getPartitionID(),
             ext->getPartitionMonitoringID());

    bool mpam_ns = !isSecure(tc);
    if (!mpam_ns && mpam3.el3.forceNs) {
        mpam_ns = true;
    }

    ext->setMpamNS(mpam_ns);

    req->setExtension(ext);
}

} // namespace gem5::ArmISA::mpam
