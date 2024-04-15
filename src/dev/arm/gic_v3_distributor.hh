/*
 * Copyright (c) 2019-2022 ARM Limited
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
 * Copyright (c) 2018 Metempsy Technology Consulting
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

#ifndef __DEV_ARM_GICV3_DISTRIBUTOR_H__
#define __DEV_ARM_GICV3_DISTRIBUTOR_H__

#include "base/addr_range.hh"
#include "dev/arm/gic_v3.hh"
#include "sim/serialize.hh"

namespace gem5
{

class Gicv3Distributor : public Serializable
{
  private:
    friend class Gicv3Redistributor;
    friend class Gicv3CPUInterface;
    friend class Gicv3Its;

  protected:
    Gicv3 *gic;
    const uint32_t itLines;

    enum
    {
        // Control Register
        GICD_CTLR = 0x0000,
        // Interrupt Controller Type Register
        GICD_TYPER = 0x0004,
        // Implementer Identification Register
        GICD_IIDR = 0x0008,
        // Interrupt Controller Type Register 2
        GICD_TYPER2 = 0x000C,
        // Error Reporting Status Register
        GICD_STATUSR = 0x0010,
        // Set Non-secure SPI Pending Register
        GICD_SETSPI_NSR = 0x0040,
        // Clear Non-secure SPI Pending Register
        GICD_CLRSPI_NSR = 0x0048,
        // Set Secure SPI Pending Register
        GICD_SETSPI_SR = 0x0050,
        // Clear Secure SPI Pending Register
        GICD_CLRSPI_SR = 0x0058,
        // Software Generated Interrupt Register
        GICD_SGIR = 0x0f00,
        // Peripheral ID0 Register
        GICD_PIDR0 = 0xffe0,
        // Peripheral ID1 Register
        GICD_PIDR1 = 0xffe4,
        // Peripheral ID2 Register
        GICD_PIDR2 = 0xffe8,
        // Peripheral ID3 Register
        GICD_PIDR3 = 0xffec,
        // Peripheral ID4 Register
        GICD_PIDR4 = 0xffd0,
        // Peripheral ID5 Register
        GICD_PIDR5 = 0xffd4,
        // Peripheral ID6 Register
        GICD_PIDR6 = 0xffd8,
        // Peripheral ID7 Register
        GICD_PIDR7 = 0xffdc,
    };

    // Interrupt Group Registers
    static const AddrRange GICD_IGROUPR;
    // Interrupt Set-Enable Registers
    static const AddrRange GICD_ISENABLER;
    // Interrupt Clear-Enable Registers
    static const AddrRange GICD_ICENABLER;
    // Interrupt Set-Pending Registers
    static const AddrRange GICD_ISPENDR;
    // Interrupt Clear-Pending Registers
    static const AddrRange GICD_ICPENDR;
    // Interrupt Set-Active Registers
    static const AddrRange GICD_ISACTIVER;
    // Interrupt Clear-Active Registers
    static const AddrRange GICD_ICACTIVER;
    // Interrupt Priority Registers
    static const AddrRange GICD_IPRIORITYR;
    // Interrupt Processor Targets Registers
    static const AddrRange GICD_ITARGETSR; // GICv2 legacy
    // Interrupt Configuration Registers
    static const AddrRange GICD_ICFGR;
    // Interrupt Group Modifier Registers
    static const AddrRange GICD_IGRPMODR;
    // Non-secure Access Control Registers
    static const AddrRange GICD_NSACR;
    // SGI Clear-Pending Registers
    static const AddrRange GICD_CPENDSGIR; // GICv2 legacy
    // SGI Set-Pending Registers
    static const AddrRange GICD_SPENDSGIR; // GICv2 legacy
    // Interrupt Routing Registers
    static const AddrRange GICD_IROUTER;

    BitUnion64(IROUTER)
        Bitfield<63, 40> res0_1;
        Bitfield<39, 32> Aff3;
        Bitfield<31> IRM;
        Bitfield<30, 24> res0_2;
        Bitfield<23, 16> Aff2;
        Bitfield<15, 8> Aff1;
        Bitfield<7, 0> Aff0;
    EndBitUnion(IROUTER)

    static const uint32_t GICD_CTLR_ENABLEGRP0 = 1 << 0;
    static const uint32_t GICD_CTLR_ENABLEGRP1 = 1 << 0;
    static const uint32_t GICD_CTLR_ENABLEGRP1NS = 1 << 1;
    static const uint32_t GICD_CTLR_ENABLEGRP1A = 1 << 1;
    static const uint32_t GICD_CTLR_ENABLEGRP1S = 1 << 2;
    static const uint32_t GICD_CTLR_DS = 1 << 6;

    bool ARE;
    bool DS;
    bool EnableGrp1S;
    bool EnableGrp1NS;
    bool EnableGrp0;
    std::vector<uint8_t> irqGroup;
    std::vector<bool> irqEnabled;
    std::vector<bool> irqPending;
    std::vector<bool> irqPendingIspendr;
    std::vector<bool> irqActive;
    std::vector<uint8_t> irqPriority;
    std::vector<Gicv3::IntTriggerType> irqConfig;
    std::vector<uint8_t> irqGrpmod;
    std::vector<uint8_t> irqNsacr;
    std::vector<IROUTER> irqAffinityRouting;

    uint32_t gicdTyper;
    uint32_t gicdPidr0;
    uint32_t gicdPidr1;
    uint32_t gicdPidr2;
    uint32_t gicdPidr3;
    uint32_t gicdPidr4;

  public:
    static const uint32_t ADDR_RANGE_SIZE = 0x10000;
    static const uint32_t IDBITS = 0xf;

  protected:
    void activateIRQ(uint32_t int_id);
    void deactivateIRQ(uint32_t int_id);
    void fullUpdate();
    Gicv3::GroupId getIntGroup(int int_id) const;

    inline bool
    groupEnabled(Gicv3::GroupId group) const
    {
        if (DS == 0) {
            switch (group) {
            case Gicv3::G0S:
                return EnableGrp0;

            case Gicv3::G1S:
                return EnableGrp1S;

            case Gicv3::G1NS:
                return EnableGrp1NS;

            default:
                panic("Gicv3Distributor::groupEnabled(): "
                      "invalid group!\n");
            }
        } else {
            switch (group) {
            case Gicv3::G0S:
                return EnableGrp0;

            case Gicv3::G1S:
            case Gicv3::G1NS:
                return EnableGrp1NS;

            default:
                panic("Gicv3Distributor::groupEnabled(): "
                      "invalid group!\n");
            }
        }
    }

    Gicv3::IntStatus intStatus(uint32_t int_id) const;

    inline bool
    isNotSPI(uint32_t int_id) const
    {
        if (int_id < (Gicv3::SGI_MAX + Gicv3::PPI_MAX) || int_id >= itLines) {
            return true;
        } else {
            return false;
        }
    }

    bool
    isLevelSensitive(uint32_t int_id) const
    {
        return irqConfig[int_id] == Gicv3::INT_LEVEL_SENSITIVE;
    }

    /**
     * This helper is used to check if an interrupt should be treated as
     * edge triggered in the following scenarios:
     *
     * a) While activating the interrupt
     * b) While clearing an interrupt via ICPENDR
     *
     * In fact, in these two situations, a level sensitive interrupt
     * which had been made pending via a write to ISPENDR, will be
     * treated as it if was edge triggered.
     */
    bool
    treatAsEdgeTriggered(uint32_t int_id) const
    {
        return !isLevelSensitive(int_id) || irqPendingIspendr[int_id];
    }

    inline bool
    nsAccessToSecInt(uint32_t int_id, bool is_secure_access) const
    {
        return !DS && !is_secure_access && getIntGroup(int_id) != Gicv3::G1NS;
    }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    Gicv3CPUInterface *route(uint32_t int_id);

  public:
    Gicv3Distributor(Gicv3 *gic, uint32_t it_lines);

    void sendInt(uint32_t int_id);
    void clearInt(uint32_t int_id);
    void deassertSPI(uint32_t int_id);
    void clearIrqCpuInterface(uint32_t int_id);
    void init();
    uint64_t read(Addr addr, size_t size, bool is_secure_access);
    void write(Addr addr, uint64_t data, size_t size, bool is_secure_access);

    void copy(Gicv3Registers *from, Gicv3Registers *to);
    void update();
};

} // namespace gem5

#endif //__DEV_ARM_GICV3_DISTRIBUTOR_H__
