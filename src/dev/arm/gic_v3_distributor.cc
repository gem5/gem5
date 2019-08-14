/*
 * Copyright (c) 2019 ARM Limited
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
 *
 * Authors: Jairo Balart
 */

#include "dev/arm/gic_v3_distributor.hh"

#include <algorithm>

#include "debug/GIC.hh"
#include "dev/arm/gic_v3.hh"
#include "dev/arm/gic_v3_cpu_interface.hh"
#include "dev/arm/gic_v3_redistributor.hh"

const AddrRange Gicv3Distributor::GICD_IGROUPR   (0x0080, 0x00ff);
const AddrRange Gicv3Distributor::GICD_ISENABLER (0x0100, 0x017f);
const AddrRange Gicv3Distributor::GICD_ICENABLER (0x0180, 0x01ff);
const AddrRange Gicv3Distributor::GICD_ISPENDR   (0x0200, 0x027f);
const AddrRange Gicv3Distributor::GICD_ICPENDR   (0x0280, 0x02ff);
const AddrRange Gicv3Distributor::GICD_ISACTIVER (0x0300, 0x037f);
const AddrRange Gicv3Distributor::GICD_ICACTIVER (0x0380, 0x03ff);
const AddrRange Gicv3Distributor::GICD_IPRIORITYR(0x0400, 0x07ff);
const AddrRange Gicv3Distributor::GICD_ITARGETSR (0x0800, 0x08ff);
const AddrRange Gicv3Distributor::GICD_ICFGR     (0x0c00, 0x0cff);
const AddrRange Gicv3Distributor::GICD_IGRPMODR  (0x0d00, 0x0d7f);
const AddrRange Gicv3Distributor::GICD_NSACR     (0x0e00, 0x0eff);
const AddrRange Gicv3Distributor::GICD_CPENDSGIR (0x0f10, 0x0f1f);
const AddrRange Gicv3Distributor::GICD_SPENDSGIR (0x0f20, 0x0f2f);
const AddrRange Gicv3Distributor::GICD_IROUTER   (0x6000, 0x7fe0);

Gicv3Distributor::Gicv3Distributor(Gicv3 * gic, uint32_t it_lines)
    : gic(gic),
      itLines(it_lines),
      irqGroup(it_lines),
      irqEnabled(it_lines),
      irqPending(it_lines),
      irqActive(it_lines),
      irqPriority(it_lines),
      irqConfig(it_lines),
      irqGrpmod(it_lines),
      irqNsacr(it_lines),
      irqAffinityRouting(it_lines),
      gicdPidr0(0x92),
      gicdPidr1(0xb4),
      gicdPidr2(0x3b),
      gicdPidr3(0),
      gicdPidr4(0x44)
{
    panic_if(it_lines > Gicv3::INTID_SECURE, "Invalid value for it_lines!");
}

void
Gicv3Distributor::init()
{
}

void
Gicv3Distributor::initState()
{
    reset();
}

void
Gicv3Distributor::reset()
{
    std::fill(irqGroup.begin(), irqGroup.end(), 0);
    // Imp. defined reset value
    std::fill(irqEnabled.begin(), irqEnabled.end(), false);
    std::fill(irqPending.begin(), irqPending.end(), false);
    std::fill(irqActive.begin(), irqActive.end(), false);
    // Imp. defined reset value
    std::fill(irqPriority.begin(), irqPriority.end(), 0xAAAAAAAA);
    std::fill(irqConfig.begin(), irqConfig.end(),
              Gicv3::INT_LEVEL_SENSITIVE); // Imp. defined reset value
    std::fill(irqGrpmod.begin(), irqGrpmod.end(), 0);
    std::fill(irqNsacr.begin(), irqNsacr.end(), 0);
    /*
     * For our implementation affinity routing is always enabled,
     * no GICv2 legacy
     */
    ARE = true;

    if (gic->getSystem()->haveSecurity()) {
        DS = false;
    } else {
        DS = true;
    }

    EnableGrp0 = 0;
    EnableGrp1NS = 0;
    EnableGrp1S = 0;
}

uint64_t
Gicv3Distributor::read(Addr addr, size_t size, bool is_secure_access)
{
    if (GICD_IGROUPR.contains(addr)) { // Interrupt Group Registers
        uint64_t val = 0x0;

        if (!DS && !is_secure_access) {
            // RAZ/WI for non-secure accesses
            return 0;
        }

        int first_intid = (addr - GICD_IGROUPR.start()) * 8;

        if (isNotSPI(first_intid)) {
            return 0;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {
            val |= irqGroup[int_id] << i;
        }

        return val;
    } else if (GICD_ISENABLER.contains(addr)) {
        // Interrupt Set-Enable Registers
        uint64_t val = 0x0;
        int first_intid = (addr - GICD_ISENABLER.start()) * 8;

        if (isNotSPI(first_intid)) {
            return 0;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                continue;
            }

            val |= irqEnabled[int_id] << i;
        }

        return val;
    } else if (GICD_ICENABLER.contains(addr)) {
        // Interrupt Clear-Enable Registers
        uint64_t val = 0x0;
        int first_intid = (addr - GICD_ICENABLER.start()) * 8;

        if (isNotSPI(first_intid)) {
            return 0;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                continue;
            }

            val |= (irqEnabled[int_id] << i);
        }

        return val;
    } else if (GICD_ISPENDR.contains(addr)) {
        // Interrupt Set-Pending Registers
        uint64_t val = 0x0;
        int first_intid = (addr - GICD_ISPENDR.start()) * 8;

        if (isNotSPI(first_intid)) {
            return 0;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                if (irqNsacr[int_id] == 0) {
                    // Group 0 or Secure Group 1 interrupts are RAZ/WI
                    continue;
                }
            }

            val |= (irqPending[int_id] << i);
        }

        return val;
    } else if (GICD_ICPENDR.contains(addr)) {
        // Interrupt Clear-Pending Registers
        uint64_t val = 0x0;
        int first_intid = (addr - GICD_ICPENDR.start()) * 8;

        if (isNotSPI(first_intid)) {
            return 0;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                if (irqNsacr[int_id] < 2) {
                    // Group 0 or Secure Group 1 interrupts are RAZ/WI
                    continue;
                }
            }

            val |= (irqPending[int_id] << i);
        }

        return val;
    } else if (GICD_ISACTIVER.contains(addr)) {
        // Interrupt Set-Active Registers
        int first_intid = (addr - GICD_ISACTIVER.start()) * 8;

        if (isNotSPI(first_intid)) {
            return 0;
        }

        uint64_t val = 0x0;

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                // Group 0 or Secure Group 1 interrupts are RAZ/WI
                if (irqNsacr[int_id] < 2) {
                    continue;
                }
            }

            val |= (irqActive[int_id] << i);
        }

        return val;
    } else if (GICD_ICACTIVER.contains(addr)) {
        // Interrupt Clear-Active Registers
        int first_intid = (addr - GICD_ICACTIVER.start()) * 8;

        if (isNotSPI(first_intid)) {
            return 0;
        }

        uint64_t val = 0x0;

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                if (irqNsacr[int_id] < 2) {
                    continue;
                }
            }

            val |= (irqActive[int_id] << i);
        }

        return val;
    } else if (GICD_IPRIORITYR.contains(addr)) {
        // Interrupt Priority Registers
        uint64_t val = 0x0;
        int first_intid = addr - GICD_IPRIORITYR.start();

        if (isNotSPI(first_intid)) {
            return 0;
        }

        for (int i = 0, int_id = first_intid; i < size && int_id < itLines;
             i++, int_id++) {

            uint8_t prio = irqPriority[int_id];

            if (!DS && !is_secure_access) {
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    // RAZ/WI for non-secure accesses for secure interrupts
                    continue;
                } else {
                    // NS view
                    prio = (prio << 1) & 0xff;
                }
            }

            val |= prio << (i * 8);
        }

        return val;
    } else if (GICD_ITARGETSR.contains(addr)) {
        // Interrupt Processor Targets Registers
        // ARE always on, RAZ/WI
        warn("Gicv3Distributor::read(): "
             "GICD_ITARGETSR is RAZ/WI, legacy not supported!\n");
        return 0;
    } else if (GICD_ICFGR.contains(addr)) {
        // Interrupt Configuration Registers
        int first_intid = (addr - GICD_ICFGR.start()) * 4;

        if (isNotSPI(first_intid)) {
            return 0;
        }

        uint64_t val = 0x0;

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i = i + 2, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                continue;
            }

            if (irqConfig[int_id] == Gicv3::INT_EDGE_TRIGGERED) {
                val |= (0x2 << i);
            }
        }

        return val;
    } else if (GICD_IGRPMODR.contains(addr)) {
        // Interrupt Group Modifier Registers
        if (DS) {
            // RAZ/WI if security disabled
            return 0;
        } else {
            if (!is_secure_access) {
                // RAZ/WI for non-secure accesses
                return 0;
            } else {
                int first_intid = (addr - GICD_IGRPMODR.start()) * 8;

                if (isNotSPI(first_intid)) {
                    return 0;
                }

                uint64_t val = 0x0;

                for (int i = 0, int_id = first_intid;
                     i < 8 * size && int_id < itLines; i++, int_id++) {
                    val |= irqGrpmod[int_id] << i;
                }

                return val;
            }
        }
    } else if (GICD_NSACR.contains(addr)) {
        // Non-secure Access Control Registers
        // 2 bits per interrupt
        int first_intid = (addr - GICD_NSACR.start()) * 4;

        if (isNotSPI(first_intid)) {
            return 0;
        }

        if (DS || (!DS && !is_secure_access)) {
            return 0;
        }

        uint64_t val = 0x0;

        for (int i = 0, int_id = first_intid;
             i < 8 * size && int_id < itLines; i = i + 2, int_id++) {
            val |= irqNsacr[int_id] << i;
        }

        return val;
    } else if (GICD_CPENDSGIR.contains(addr)) { // SGI Clear-Pending Registers
        // ARE always on, RAZ/WI
        warn("Gicv3Distributor::read(): "
             "GICD_CPENDSGIR is RAZ/WI, legacy not supported!\n");
        return 0x0;
    } else if (GICD_SPENDSGIR.contains(addr)) { // SGI Set-Pending Registers
        // ARE always on, RAZ/WI
        warn("Gicv3Distributor::read(): "
             "GICD_SPENDSGIR is RAZ/WI, legacy not supported!\n");
        return 0x0;
    } else if (GICD_IROUTER.contains(addr)) { // Interrupt Routing Registers
        // 64 bit registers. 2 or 1 access.
        int int_id = (addr - GICD_IROUTER.start()) / 8;

        if (isNotSPI(int_id)) {
            return 0;
        }

        if (nsAccessToSecInt(int_id, is_secure_access))
        {
            if (irqNsacr[int_id] < 3) {
                return 0;
            }
        }

        if (size == 4) {
            if (addr & 7) { // high half of 64 bit register
                return irqAffinityRouting[int_id] >> 32;
            } else { // high low of 64 bit register
                return irqAffinityRouting[int_id] & 0xFFFFFFFF;
            }
        } else {
            return irqAffinityRouting[int_id];
        }
    }

    switch (addr) {
      case GICD_CTLR: // Control Register
        if (!DS) {
            if (is_secure_access) {
                // E1NWF [7] RAZ/WI
                // DS [6] - Disable Security
                // ARE_NS [5] RAO/WI
                // ARE_S [4] RAO/WI
                // EnableGrp1S [2]
                // EnableGrp1NS [1]
                // EnableGrp0 [0]
                return (EnableGrp0 << 0) |
                    (EnableGrp1NS << 1) |
                    (EnableGrp1S << 2) |
                    (1 << 4) |
                    (1 << 5) |
                    (DS << 6);
            } else {
                // ARE_NS [4] RAO/WI;
                // EnableGrp1A [1] is a read-write alias of the Secure
                // GICD_CTLR.EnableGrp1NS
                // EnableGrp1 [0] RES0
                return (1 << 4) | (EnableGrp1NS << 1);
            }
        } else {
            return (DS << 6) | (ARE << 4) |
                (EnableGrp1NS << 1) | (EnableGrp0 << 0);
        }

      case GICD_TYPER: // Interrupt Controller Type Register
        /*
         * RSS           [26]    == 1
         * (The implementation does supports targeted SGIs with affinity
         * level 0 values of 0 - 255)
         * No1N          [25]    == 1
         * (1 of N SPI interrupts are not supported)
         * A3V           [24]    == 1
         * (Supports nonzero values of Affinity level 3)
         * IDbits        [23:19] == 0xf
         * (The number of interrupt identifier bits supported, minus one)
         * DVIS          [18]    == 0
         * (The implementation does not support Direct Virtual LPI
         * injection)
         * LPIS          [17]    == 1
         * (The implementation does not support LPIs)
         * MBIS          [16]    == 0
         * (The implementation does not support message-based interrupts
         * by writing to Distributor registers)
         * SecurityExtn  [10]    == X
         * (The GIC implementation supports two Security states)
         * CPUNumber     [7:5]   == 0
         * (since for us ARE is always 1 [(ARE = 0) == Gicv2 legacy])
         * ITLinesNumber [4:0]   == N
         * (MaxSPIIntId = 32 (N + 1) - 1)
         */
        {
            int max_spi_int_id = itLines - 1;
            int it_lines_number = ceil((max_spi_int_id + 1) / 32.0) - 1;
            return (1 << 26) | (1 << 25) | (1 << 24) | (IDBITS << 19) |
                (1 << 17) | (gic->getSystem()->haveSecurity() << 10) |
                (it_lines_number << 0);
        }

      case GICD_IIDR: // Implementer Identification Register
        //return 0x43b; // ARM JEP106 code (r0p0 GIC-500)
        return 0;

      case GICD_STATUSR: // Error Reporting Status Register
        // Optional register, RAZ/WI
        return 0x0;

      case GICD_PIDR0: // Peripheral ID0 Register
        return gicdPidr0;

      case GICD_PIDR1: // Peripheral ID1 Register
        return gicdPidr1;

      case GICD_PIDR2: // Peripheral ID2 Register
        return gicdPidr2;

      case GICD_PIDR3: // Peripheral ID3 Register
        return gicdPidr3;

      case GICD_PIDR4: // Peripheral ID4 Register
        return gicdPidr4;

      case GICD_PIDR5: // Peripheral ID5 Register
      case GICD_PIDR6: // Peripheral ID6 Register
      case GICD_PIDR7: // Peripheral ID7 Register
        return 0; // RES0

      default:
        panic("Gicv3Distributor::read(): invalid offset %#x\n", addr);
        break;
    }
}

void
Gicv3Distributor::write(Addr addr, uint64_t data, size_t size,
                        bool is_secure_access)
{
    if (GICD_IGROUPR.contains(addr)) { // Interrupt Group Registers
        if (!DS && !is_secure_access) {
            // RAZ/WI for non-secure accesses
            return;
        }

        int first_intid = (addr - GICD_IGROUPR.start()) * 8;

        if (isNotSPI(first_intid)) {
            return;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {
            irqGroup[int_id] = data & (1 << i) ? 1 : 0;
            DPRINTF(GIC, "Gicv3Distributor::write(): int_id %d group %d\n",
                    int_id, irqGroup[int_id]);
        }

        return;
    } else if (GICD_ISENABLER.contains(addr)) {
        // Interrupt Set-Enable Registers
        int first_intid = (addr - GICD_ISENABLER.start()) * 8;

        if (isNotSPI(first_intid)) {
            return;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                continue;
            }

            bool enable = data & (1 << i) ? 1 : 0;

            if (enable) {
                if (!irqEnabled[int_id]) {
                    DPRINTF(GIC, "Gicv3Distributor::write(): "
                            "int_id %d enabled\n", int_id);
                }

                irqEnabled[int_id] = true;
            }
        }

        return;
    } else if (GICD_ICENABLER.contains(addr)) {
        // Interrupt Clear-Enable Registers
        int first_intid = (addr - GICD_ICENABLER.start()) * 8;

        if (isNotSPI(first_intid)) {
            return;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                continue;
            }

            bool disable = data & (1 << i) ? 1 : 0;

            if (disable) {
                if (irqEnabled[int_id]) {
                    DPRINTF(GIC, "Gicv3Distributor::write(): "
                            "int_id %d disabled\n", int_id);
                }

                irqEnabled[int_id] = false;
            }
        }

        return;
    } else if (GICD_ISPENDR.contains(addr)) {
        // Interrupt Set-Pending Registers
        int first_intid = (addr - GICD_ISPENDR.start()) * 8;

        if (isNotSPI(first_intid)) {
            return;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                if (irqNsacr[int_id] == 0) {
                    // Group 0 or Secure Group 1 interrupts are RAZ/WI
                    continue;
                }
            }

            bool pending = data & (1 << i) ? 1 : 0;

            if (pending) {
                DPRINTF(GIC, "Gicv3Distributor::write() (GICD_ISPENDR): "
                        "int_id %d (SPI) pending bit set\n", int_id);
                irqPending[int_id] = true;
            }
        }

        updateAndInformCPUInterfaces();
        return;
    } else if (GICD_ICPENDR.contains(addr)) {
        // Interrupt Clear-Pending Registers
        int first_intid = (addr - GICD_ICPENDR.start()) * 8;

        if (isNotSPI(first_intid)) {
            return;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                if (irqNsacr[int_id] < 2) {
                    // Group 0 or Secure Group 1 interrupts are RAZ/WI
                    continue;
                }
            }

            bool clear = data & (1 << i) ? 1 : 0;

            if (clear) {
                irqPending[int_id] = false;
            }
        }

        updateAndInformCPUInterfaces();
        return;
    } else if (GICD_ISACTIVER.contains(addr)) {
        // Interrupt Set-Active Registers
        int first_intid = (addr - GICD_ISACTIVER.start()) * 8;

        if (isNotSPI(first_intid)) {
            return;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                continue;
            }

            bool active = data & (1 << i) ? 1 : 0;

            if (active) {
                irqActive[int_id] = 1;
            }
        }

        return;
    } else if (GICD_ICACTIVER.contains(addr)) {
        // Interrupt Clear-Active Registers
        int first_intid = (addr - GICD_ICACTIVER.start()) * 8;

        if (isNotSPI(first_intid)) {
            return;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i++, int_id++) {

            if (nsAccessToSecInt(int_id, is_secure_access))
            {
                continue;
            }

            bool clear = data & (1 << i) ? 1 : 0;

            if (clear) {
                if (irqActive[int_id]) {
                    DPRINTF(GIC, "Gicv3Distributor::write(): "
                            "int_id %d active cleared\n", int_id);
                }

                irqActive[int_id] = false;
            }
        }

        return;
    } else if (GICD_IPRIORITYR.contains(addr)) {
        // Interrupt Priority Registers
        int first_intid = addr - GICD_IPRIORITYR.start();

        if (isNotSPI(first_intid)) {
            return;
        }

        for (int i = 0, int_id = first_intid; i < size && int_id < itLines;
                i++, int_id++) {
            uint8_t prio = bits(data, (i + 1) * 8 - 1, (i * 8));

            if (!DS && !is_secure_access) {
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    // RAZ/WI for non-secure accesses to secure interrupts
                    continue;
                } else {
                    prio = 0x80 | (prio >> 1);
                }
            }

            irqPriority[int_id] = prio;
            DPRINTF(GIC, "Gicv3Distributor::write(): int_id %d priority %d\n",
                    int_id, irqPriority[int_id]);
        }

        return;
    } else if (GICD_ITARGETSR.contains(addr)) {
        // Interrupt Processor Targets Registers
        // ARE always on, RAZ/WI
        warn("Gicv3Distributor::write(): "
             "GICD_ITARGETSR is RAZ/WI, legacy not supported!\n");
        return;
    } else if (GICD_ICFGR.contains(addr)) {
        // Interrupt Configuration Registers
        // for x = 0 to 15:
        //   GICD_ICFGR[2x] = RES0
        //   GICD_ICFGR[2x + 1] =
        //     0 level-sensitive
        //     1 edge-triggered
        int first_intid = (addr - GICD_ICFGR.start()) * 4;

        if (isNotSPI(first_intid)) {
            return;
        }

        for (int i = 0, int_id = first_intid; i < 8 * size && int_id < itLines;
             i = i + 2, int_id++) {
            irqConfig[int_id] = data & (0x2 << i) ?
                                Gicv3::INT_EDGE_TRIGGERED :
                                Gicv3::INT_LEVEL_SENSITIVE;
            DPRINTF(GIC, "Gicv3Distributor::write(): int_id %d config %d\n",
                    int_id, irqConfig[int_id]);
        }

        return;
    } else if (GICD_IGRPMODR.contains(addr)) {
        // Interrupt Group Modifier Registers
        if (DS) {
            return;
        } else {
            if (!is_secure_access) {
                // RAZ/WI for non-secure accesses
                return;
            } else {
                int first_intid = (addr - GICD_IGRPMODR.start()) * 8;

                if (isNotSPI(first_intid)) {
                    return;
                }

                for (int i = 0, int_id = first_intid;
                     i < 8 * size && int_id < itLines; i++, int_id++) {
                    irqGrpmod[int_id] = data & (0x1 << i);
                }

                return ;
            }
        }

    } else if (GICD_NSACR.contains(addr)) {
        // Non-secure Access Control Registers
        // 2 bits per interrupt
        int first_intid = (addr - GICD_NSACR.start()) * 4;

        if (isNotSPI(first_intid)) {
            return;
        }

        if (DS || (!DS && !is_secure_access)) {
            return;
        }

        for (int i = 0, int_id = first_intid;
             i < 8 * size && int_id < itLines; i = i + 2, int_id++) {
            irqNsacr[int_id] = (data >> (2 * int_id)) & 0x3;
        }

        return;
    } else if (GICD_IROUTER.contains(addr)) { // Interrupt Routing Registers
        // 64 bit registers. 2 accesses.
        int int_id = (addr - GICD_IROUTER.start()) / 8;

        if (isNotSPI(int_id)) {
            return;
        }

        if (nsAccessToSecInt(int_id, is_secure_access))
        {
            if (irqNsacr[int_id] < 3) {
                // Group 0 or Secure Group 1 interrupts are RAZ/WI
                return;
            }
        }

        if (size == 4) {
            if (addr & 7) { // high half of 64 bit register
                irqAffinityRouting[int_id] =
                    (irqAffinityRouting[int_id] & 0xffffffff) | (data << 32);
            } else { // low half of 64 bit register
                irqAffinityRouting[int_id] =
                    (irqAffinityRouting[int_id] & 0xffffffff00000000) |
                    (data & 0xffffffff);
            }
        } else {
            irqAffinityRouting[int_id] = data;
        }

        DPRINTF(GIC, "Gicv3Distributor::write(): "
                "int_id %d GICD_IROUTER %#llx\n",
                int_id, irqAffinityRouting[int_id]);
        return;
    }

    switch (addr) {
      case GICD_CTLR: // Control Register
        if (DS) {
            /*
             * E1NWF [7]
             * 1 of N wakeup functionality not supported, RAZ/WI
             * DS [6] - RAO/WI
             * ARE [4]
             * affinity routing always on, no GICv2 legacy, RAO/WI
             * EnableGrp1 [1]
             * EnableGrp0 [0]
             */
            if ((data & (1 << 4)) == 0) {
                warn("Gicv3Distributor::write(): "
                        "setting ARE to 0 is not supported!\n");
            }

            EnableGrp1NS = data & GICD_CTLR_ENABLEGRP1NS;
            EnableGrp0 = data & GICD_CTLR_ENABLEGRP0;
            DPRINTF(GIC, "Gicv3Distributor::write(): (DS 1)"
                    "EnableGrp1NS %d EnableGrp0 %d\n",
                    EnableGrp1NS, EnableGrp0);
        } else {
            if (is_secure_access) {
                /*
                 * E1NWF [7]
                 * 1 of N wakeup functionality not supported, RAZ/WI
                 * DS [6]
                 * ARE_NS [5]
                 * affinity routing always on, no GICv2 legacy, RAO/WI
                 * ARE_S [4]
                 * affinity routing always on, no GICv2 legacy, RAO/WI
                 * EnableGrp1S [2]
                 * EnableGrp1NS [1]
                 * EnableGrp0 [0]
                 */
                if ((data & (1 << 5)) == 0) {
                    warn("Gicv3Distributor::write(): "
                            "setting ARE_NS to 0 is not supported!\n");
                }

                if ((data & (1 << 4)) == 0) {
                    warn("Gicv3Distributor::write(): "
                            "setting ARE_S to 0 is not supported!\n");
                }

                DS = data & GICD_CTLR_DS;
                EnableGrp1S = data & GICD_CTLR_ENABLEGRP1S;
                EnableGrp1NS = data & GICD_CTLR_ENABLEGRP1NS;
                EnableGrp0 = data & GICD_CTLR_ENABLEGRP0;
                DPRINTF(GIC, "Gicv3Distributor::write(): (DS 0 secure)"
                        "DS %d "
                        "EnableGrp1S %d EnableGrp1NS %d EnableGrp0 %d\n",
                        DS, EnableGrp1S, EnableGrp1NS, EnableGrp0);

                if (data & GICD_CTLR_DS) {
                    EnableGrp1S = 0;
                }
            } else {
                /*
                 * ARE_NS [4] RAO/WI;
                 * EnableGrp1A [1] is a read-write alias of the Secure
                 * GICD_CTLR.EnableGrp1NS
                 * EnableGrp1 [0] RES0
                 */
                if ((data & (1 << 4)) == 0) {
                    warn("Gicv3Distributor::write(): "
                            "setting ARE_NS to 0 is not supported!\n");
                }

                EnableGrp1NS = data & GICD_CTLR_ENABLEGRP1A;
                DPRINTF(GIC, "Gicv3Distributor::write(): (DS 0 non-secure)"
                        "EnableGrp1NS %d\n", EnableGrp1NS);
            }
        }

        break;

      default:
        panic("Gicv3Distributor::write(): invalid offset %#x\n", addr);
        break;
    }
}

void
Gicv3Distributor::sendInt(uint32_t int_id)
{
    panic_if(int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX, "Invalid SPI!");
    panic_if(int_id > itLines, "Invalid SPI!");
    irqPending[int_id] = true;
    DPRINTF(GIC, "Gicv3Distributor::sendInt(): "
            "int_id %d (SPI) pending bit set\n", int_id);
    updateAndInformCPUInterfaces();
}

void
Gicv3Distributor::deassertSPI(uint32_t int_id)
{
    panic_if(int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX, "Invalid SPI!");
    panic_if(int_id > itLines, "Invalid SPI!");
    irqPending[int_id] = false;
    updateAndInformCPUInterfaces();
}

void
Gicv3Distributor::updateAndInformCPUInterfaces()
{
    update();

    for (int i = 0; i < gic->getSystem()->numContexts(); i++) {
        gic->getCPUInterface(i)->update();
    }
}

void
Gicv3Distributor::fullUpdate()
{
    for (int i = 0; i < gic->getSystem()->numContexts(); i++) {
        Gicv3CPUInterface * cpu_interface_i = gic->getCPUInterface(i);
        cpu_interface_i->hppi.prio = 0xff;
    }

    update();

    for (int i = 0; i < gic->getSystem()->numContexts(); i++) {
        Gicv3Redistributor * redistributor_i = gic->getRedistributor(i);
        redistributor_i->update();
    }
}

void
Gicv3Distributor::update()
{
    std::vector<bool> new_hppi(gic->getSystem()->numContexts(), false);

    // Find the highest priority pending SPI
    for (int int_id = Gicv3::SGI_MAX + Gicv3::PPI_MAX; int_id < itLines;
         int_id++) {
        Gicv3::GroupId int_group = getIntGroup(int_id);
        bool group_enabled = groupEnabled(int_group);

        if (irqPending[int_id] && irqEnabled[int_id] &&
            !irqActive[int_id] && group_enabled) {
            IROUTER affinity_routing = irqAffinityRouting[int_id];
            Gicv3Redistributor * target_redistributor = nullptr;

            if (affinity_routing.IRM) {
                // Interrupts routed to any PE defined as a participating node
                for (int i = 0; i < gic->getSystem()->numContexts(); i++) {
                    Gicv3Redistributor * redistributor_i =
                        gic->getRedistributor(i);

                    if (redistributor_i->
                            canBeSelectedFor1toNInterrupt(int_group)) {
                        target_redistributor = redistributor_i;
                        break;
                    }
                }
            } else {
                uint32_t affinity = (affinity_routing.Aff3 << 24) |
                                    (affinity_routing.Aff3 << 16) |
                                    (affinity_routing.Aff1 << 8) |
                                    (affinity_routing.Aff0 << 0);
                target_redistributor =
                    gic->getRedistributorByAffinity(affinity);
            }

            if (!target_redistributor) {
                // Interrrupts targeting not present cpus must remain pending
                return;
            }

            Gicv3CPUInterface * target_cpu_interface =
                target_redistributor->getCPUInterface();
            uint32_t target_cpu = target_redistributor->cpuId;

            if ((irqPriority[int_id] < target_cpu_interface->hppi.prio) ||
                /*
                 * Multiple pending ints with same priority.
                 * Implementation choice which one to signal.
                 * Our implementation selects the one with the lower id.
                 */
                (irqPriority[int_id] == target_cpu_interface->hppi.prio &&
                int_id < target_cpu_interface->hppi.intid)) {
                target_cpu_interface->hppi.intid = int_id;
                target_cpu_interface->hppi.prio = irqPriority[int_id];
                target_cpu_interface->hppi.group = int_group;
                new_hppi[target_cpu] = true;
            }
        }
    }

    for (int i = 0; i < gic->getSystem()->numContexts(); i++) {
        Gicv3Redistributor * redistributor_i = gic->getRedistributor(i);
        Gicv3CPUInterface * cpu_interface_i =
            redistributor_i->getCPUInterface();

        if (!new_hppi[i] && cpu_interface_i->hppi.prio != 0xff &&
            cpu_interface_i->hppi.intid >= (Gicv3::SGI_MAX + Gicv3::PPI_MAX) &&
            cpu_interface_i->hppi.intid < Gicv3::INTID_SECURE) {
            fullUpdate();
        }
    }
}

Gicv3::IntStatus
Gicv3Distributor::intStatus(uint32_t int_id) const
{
    panic_if(int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX, "Invalid SPI!");
    panic_if(int_id > itLines, "Invalid SPI!");

    if (irqPending[int_id]) {
        if (irqActive[int_id]) {
            return Gicv3::INT_ACTIVE_PENDING;
        }

        return Gicv3::INT_PENDING;
    } else if (irqActive[int_id]) {
        return Gicv3::INT_ACTIVE;
    } else {
        return Gicv3::INT_INACTIVE;
    }
}

Gicv3::GroupId
Gicv3Distributor::getIntGroup(int int_id) const
{
    panic_if(int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX, "Invalid SPI!");
    panic_if(int_id > itLines, "Invalid SPI!");

    if (DS) {
        if (irqGroup[int_id] == 1) {
            return Gicv3::G1NS;
        } else {
            return Gicv3::G0S;
        }
    } else {
        if (irqGrpmod[int_id] == 0 && irqGroup[int_id] == 0) {
            return Gicv3::G0S;
        } else if (irqGrpmod[int_id] == 0 && irqGroup[int_id] == 1) {
            return Gicv3::G1NS;
        } else if (irqGrpmod[int_id] == 1 && irqGroup[int_id] == 0) {
            return Gicv3::G1S;
        } else if (irqGrpmod[int_id] == 1 && irqGroup[int_id] == 1) {
            return Gicv3::G1NS;
        }
    }

    M5_UNREACHABLE;
}

void
Gicv3Distributor::activateIRQ(uint32_t int_id)
{
    irqPending[int_id] = false;
    irqActive[int_id] = true;
}

void
Gicv3Distributor::deactivateIRQ(uint32_t int_id)
{
    irqActive[int_id] = false;
}

void
Gicv3Distributor::serialize(CheckpointOut & cp) const
{
    SERIALIZE_SCALAR(ARE);
    SERIALIZE_SCALAR(DS);
    SERIALIZE_SCALAR(EnableGrp1S);
    SERIALIZE_SCALAR(EnableGrp1NS);
    SERIALIZE_SCALAR(EnableGrp0);
    SERIALIZE_CONTAINER(irqGroup);
    SERIALIZE_CONTAINER(irqEnabled);
    SERIALIZE_CONTAINER(irqPending);
    SERIALIZE_CONTAINER(irqActive);
    SERIALIZE_CONTAINER(irqPriority);
    SERIALIZE_CONTAINER(irqConfig);
    SERIALIZE_CONTAINER(irqGrpmod);
    SERIALIZE_CONTAINER(irqNsacr);
    SERIALIZE_CONTAINER(irqAffinityRouting);
}

void
Gicv3Distributor::unserialize(CheckpointIn & cp)
{
    UNSERIALIZE_SCALAR(ARE);
    UNSERIALIZE_SCALAR(DS);
    UNSERIALIZE_SCALAR(EnableGrp1S);
    UNSERIALIZE_SCALAR(EnableGrp1NS);
    UNSERIALIZE_SCALAR(EnableGrp0);
    UNSERIALIZE_CONTAINER(irqGroup);
    UNSERIALIZE_CONTAINER(irqEnabled);
    UNSERIALIZE_CONTAINER(irqPending);
    UNSERIALIZE_CONTAINER(irqActive);
    UNSERIALIZE_CONTAINER(irqPriority);
    UNSERIALIZE_CONTAINER(irqConfig);
    UNSERIALIZE_CONTAINER(irqGrpmod);
    UNSERIALIZE_CONTAINER(irqNsacr);
    UNSERIALIZE_CONTAINER(irqAffinityRouting);
}
