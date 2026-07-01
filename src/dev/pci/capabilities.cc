/*
 * Copyright (c) 2025 REDS institute of the HEIG-VD
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

#include "dev/pci/capabilities.hh"

namespace gem5
{

PciCapabilityBase::PciCapabilityBase(const Params &p)
    : RegisterBankLE(p.name, p.BaseOffset),
      SimObject(p),
      capId("cap_id", p.CapId),
      nextCap("next_cap", 0)
{
    addRegisters({{capId}, {nextCap}});
    capId.readonly();
    nextCap.readonly();
}

void
PciCapabilityBase::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(capId);
    SERIALIZE_SCALAR(nextCap);
}

void
PciCapabilityBase::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(capId);
    UNSERIALIZE_SCALAR(nextCap);
}

PciPowerManagementCap::PciPowerManagementCap(const Params &p)
    : PciCapabilityBase(p),
      pmc("pmc", p.Capabilities),
      pmcs("pmcs", p.CtrlStatus)
{
    addRegisters({{pmc}, {pmcs}});
}

void
PciPowerManagementCap::serialize(CheckpointOut &cp) const
{
    PciCapabilityBase::serialize(cp);
    SERIALIZE_SCALAR(pmc);
    SERIALIZE_SCALAR(pmcs);
}

void
PciPowerManagementCap::unserialize(CheckpointIn &cp)
{
    PciCapabilityBase::unserialize(cp);
    UNSERIALIZE_SCALAR(pmc);
    UNSERIALIZE_SCALAR(pmcs);
}

PciMsiCap::PciMsiCap(const Params &p)
    : PciCapabilityBase(p),
      mc("mc", p.MsgCtrl),
      ma("ma", p.MsgAddr),
      mua("mua", p.MsgUpperAddr),
      md("md", p.MsgData),
      emd("emd", p.ExtendedMsgData),
      mmask("mmask", p.MaskBits),
      mpend("mpend", p.PendingBits)
{
    addRegisters({mc, ma});

    // Upper address, for 64 bits, is optionnal
    if (bits(p.MsgCtrl, 7)) {
        addRegister(mua);
    }

    addRegisters({md, emd});

    // Per-vector masking is optionnal
    if (bits(p.MsgCtrl, 8)) {
        addRegisters({mmask, mpend});
    }
}

void
PciMsiCap::serialize(CheckpointOut &cp) const
{
    PciCapabilityBase::serialize(cp);
    SERIALIZE_SCALAR(mc);
    SERIALIZE_SCALAR(ma);
    SERIALIZE_SCALAR(mua);
    SERIALIZE_SCALAR(md);
    SERIALIZE_SCALAR(emd);
    SERIALIZE_SCALAR(mmask);
    SERIALIZE_SCALAR(mpend);
}

void
PciMsiCap::unserialize(CheckpointIn &cp)
{
    PciCapabilityBase::unserialize(cp);
    UNSERIALIZE_SCALAR(mc);
    UNSERIALIZE_SCALAR(ma);
    UNSERIALIZE_SCALAR(mua);
    UNSERIALIZE_SCALAR(md);
    UNSERIALIZE_SCALAR(emd);
    UNSERIALIZE_SCALAR(mmask);
    UNSERIALIZE_SCALAR(mpend);
}

PciMsiXCap::PciMsiXCap(const Params &p)
    : PciCapabilityBase(p),
      mxc("mxc", p.MsgCtrl),
      mtab("mtab", p.TableOffset),
      mpba("mpba", p.PbaOffset)
{
    addRegisters({{mxc}, {mtab}, {mpba}});

    // Allocate MSIX structures
    uint16_t msixcap_mxc_ts = mxc.get() & 0x07ff;
    int msix_vecs = msixcap_mxc_ts + 1;
    msix_table.resize(msix_vecs, {{0UL, 0UL, 0UL, 0UL}});

    int pba_size = msix_vecs / MSIXVECS_PER_PBA;
    if ((msix_vecs % MSIXVECS_PER_PBA) > 0) {
        pba_size++;
    }
    msix_pba.resize(pba_size, {0});
}

void
PciMsiXCap::serialize(CheckpointOut &cp) const
{
    PciCapabilityBase::serialize(cp);
    SERIALIZE_SCALAR(mxc);
    SERIALIZE_SCALAR(mtab);
    SERIALIZE_SCALAR(mpba);
}

void
PciMsiXCap::unserialize(CheckpointIn &cp)
{
    PciCapabilityBase::unserialize(cp);
    UNSERIALIZE_SCALAR(mxc);
    UNSERIALIZE_SCALAR(mtab);
    UNSERIALIZE_SCALAR(mpba);
}

PciExpressCap::PciExpressCap(const Params &p)
    : PciCapabilityBase(p),
      pxcap("pxcap", p.Capabilities),
      pxdcap("pxdcap", p.DevCapabilities),
      pxdc("pxdc", p.DevCtrl),
      pxds("pxds", p.DevStatus),
      pxlcap("pxlcap", p.LinkCap),
      pxlc("pxlc", p.LinkCtrl),
      pxls("pxls", p.LinkStatus),
      pxscap("pxscap", p.SlotCap),
      pxsc("pxsc", p.SlotCtrl),
      pxss("pxss", p.SlotStatus),
      pxrc("pxrc", p.RootCap),
      pxrcap("pxrcap", p.RootCtrl),
      pxrs("pxrs", p.RootStatus),
      pxdcap2("pxdcap2", p.DevCap2),
      pxdc2("pxdc2", p.DevCtrl2),
      pxds2("pxds2", p.DevStatus2),
      pxlcap2("pxlcap2", p.LinkCap2),
      pxlc2("pxlc2", p.LinkCtrl2),
      pxls2("pxls2", p.LinkStatus2),
      pxscap2("pxscap2", p.SlotCap2),
      pxsc2("pxsc2", p.SlotCtrl2),
      pxss2("pxss2", p.SlotStatus2)
{
    addRegisters({
        {pxcap}, {pxdcap},  {pxdc},  {pxds},  {pxlcap},  {pxlc},
        {pxls},  {pxscap},  {pxsc},  {pxss},  {pxrc},    {pxrcap},
        {pxrs},  {pxdcap2}, {pxdc2}, {pxds2}, {pxlcap2}, {pxlc2},
        {pxls2}, {pxscap2}, {pxsc2}, {pxss2},
    });
}

void
PciExpressCap::serialize(CheckpointOut &cp) const
{
    PciCapabilityBase::serialize(cp);
    SERIALIZE_SCALAR(pxcap);
    SERIALIZE_SCALAR(pxdcap);
    SERIALIZE_SCALAR(pxdc);
    SERIALIZE_SCALAR(pxds);
    SERIALIZE_SCALAR(pxlcap);
    SERIALIZE_SCALAR(pxlc);
    SERIALIZE_SCALAR(pxls);
    SERIALIZE_SCALAR(pxscap);
    SERIALIZE_SCALAR(pxsc);
    SERIALIZE_SCALAR(pxss);
    SERIALIZE_SCALAR(pxrc);
    SERIALIZE_SCALAR(pxrcap);
    SERIALIZE_SCALAR(pxrs);
    SERIALIZE_SCALAR(pxdcap2);
    SERIALIZE_SCALAR(pxdc2);
    SERIALIZE_SCALAR(pxds2);
    SERIALIZE_SCALAR(pxlcap2);
    SERIALIZE_SCALAR(pxlc2);
    SERIALIZE_SCALAR(pxls2);
    SERIALIZE_SCALAR(pxscap2);
    SERIALIZE_SCALAR(pxsc2);
    SERIALIZE_SCALAR(pxss2);
}

void
PciExpressCap::unserialize(CheckpointIn &cp)
{
    PciCapabilityBase::unserialize(cp);
    UNSERIALIZE_SCALAR(pxcap);
    UNSERIALIZE_SCALAR(pxdcap);
    UNSERIALIZE_SCALAR(pxdc);
    UNSERIALIZE_SCALAR(pxds);
    UNSERIALIZE_SCALAR(pxlcap);
    UNSERIALIZE_SCALAR(pxlc);
    UNSERIALIZE_SCALAR(pxls);
    UNSERIALIZE_SCALAR(pxscap);
    UNSERIALIZE_SCALAR(pxsc);
    UNSERIALIZE_SCALAR(pxss);
    UNSERIALIZE_SCALAR(pxrc);
    UNSERIALIZE_SCALAR(pxrcap);
    UNSERIALIZE_SCALAR(pxrs);
    UNSERIALIZE_SCALAR(pxdcap2);
    UNSERIALIZE_SCALAR(pxdc2);
    UNSERIALIZE_SCALAR(pxds2);
    UNSERIALIZE_SCALAR(pxlcap2);
    UNSERIALIZE_SCALAR(pxlc2);
    UNSERIALIZE_SCALAR(pxls2);
    UNSERIALIZE_SCALAR(pxscap2);
    UNSERIALIZE_SCALAR(pxsc2);
    UNSERIALIZE_SCALAR(pxss2);
}

} // namespace gem5
