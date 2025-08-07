/*
 * Copyright (c) 2025 REDS institute of the HEIG-VD
 * All rights reserved
 *
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
 */

#ifndef __DEV_PCI_CAPABILITIES_HH__
#define __DEV_PCI_CAPABILITIES_HH__

#include <vector>

#include "dev/reg_bank.hh"
#include "params/PciCapabilityBase.hh"
#include "params/PciExpressCap.hh"
#include "params/PciMsiCap.hh"
#include "params/PciMsiXCap.hh"
#include "params/PciPowerManagementCap.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class PciCapabilityBase : public RegisterBankLE, public SimObject
{
  public:
    Register8 capId;
    Register8 nextCap;

    PARAMS(PciCapabilityBase);
    PciCapabilityBase(const Params &p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

class PciPowerManagementCap : public PciCapabilityBase
{
  public:
    Register16 pmc;
    Register16 pmcs;

    PARAMS(PciPowerManagementCap);
    PciPowerManagementCap(const Params &p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

class PciMsiCap : public PciCapabilityBase
{
  public:
    // Depending on impl, not all register used
    Register16 mc;
    Register32 ma;
    Register32 mua;
    Register16 md;
    Register16 emd;
    Register32 mmask;
    Register32 mpend;

    PARAMS(PciMsiCap);
    PciMsiCap(const Params &p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

union MSIXTable
{
    struct
    {
        uint32_t addr_lo;
        uint32_t addr_hi;
        uint32_t msg_data;
        uint32_t vec_ctrl;
    } fields;

    uint32_t data[4];
};

#define MSIXVECS_PER_PBA 64

struct MSIXPbaEntry
{
    uint64_t bits;
};

class PciMsiXCap : public PciCapabilityBase
{
  public:
    Register16 mxc;
    Register32 mtab;
    Register32 mpba;

    std::vector<MSIXTable> msix_table;
    std::vector<MSIXPbaEntry> msix_pba;

    PARAMS(PciMsiXCap);
    PciMsiXCap(const Params &p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

class PciExpressCap : public PciCapabilityBase
{
  public:
    Register16 pxcap;
    Register32 pxdcap;
    Register16 pxdc;
    Register16 pxds;
    Register32 pxlcap;
    Register16 pxlc;
    Register16 pxls;
    Register32 pxscap;
    Register16 pxsc;
    Register16 pxss;
    Register16 pxrc;
    Register16 pxrcap;
    Register32 pxrs;
    Register32 pxdcap2;
    Register16 pxdc2;
    Register16 pxds2;
    Register32 pxlcap2;
    Register16 pxlc2;
    Register16 pxls2;
    Register32 pxscap2;
    Register16 pxsc2;
    Register16 pxss2;

    PARAMS(PciExpressCap);
    PciExpressCap(const Params &p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif //__DEV_PCI_CAPABILITIES_HH__
