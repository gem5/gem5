/*
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

#include "dev/arm/gic_v3_redistributor.hh"

#include "arch/arm/utility.hh"
#include "debug/GIC.hh"
#include "dev/arm/gic_v3_cpu_interface.hh"
#include "dev/arm/gic_v3_distributor.hh"

const AddrRange Gicv3Redistributor::GICR_IPRIORITYR(SGI_base + 0x0400,
        SGI_base + 0x041f);

Gicv3Redistributor::Gicv3Redistributor(Gicv3 * gic, uint32_t cpu_id)
    : gic(gic),
      distributor(nullptr),
      cpuInterface(nullptr),
      cpuId(cpu_id),
      irqGroup(Gicv3::SGI_MAX + Gicv3::PPI_MAX),
      irqEnabled(Gicv3::SGI_MAX + Gicv3::PPI_MAX),
      irqPending(Gicv3::SGI_MAX + Gicv3::PPI_MAX),
      irqActive(Gicv3::SGI_MAX + Gicv3::PPI_MAX),
      irqPriority(Gicv3::SGI_MAX + Gicv3::PPI_MAX),
      irqConfig(Gicv3::SGI_MAX + Gicv3::PPI_MAX),
      irqGrpmod(Gicv3::SGI_MAX + Gicv3::PPI_MAX),
      irqNsacr(Gicv3::SGI_MAX + Gicv3::PPI_MAX)
{
}

Gicv3Redistributor::~Gicv3Redistributor()
{
}

void
Gicv3Redistributor::init()
{
    distributor = gic->getDistributor();
    cpuInterface = gic->getCPUInterface(cpuId);
}

void
Gicv3Redistributor::initState()
{
    reset();
}

void
Gicv3Redistributor::reset()
{
    peInLowPowerState = true;
    std::fill(irqGroup.begin(), irqGroup.end(), 0);
    std::fill(irqEnabled.begin(), irqEnabled.end(), false);
    std::fill(irqPending.begin(), irqPending.end(), false);
    std::fill(irqActive.begin(), irqActive.end(), false);
    std::fill(irqPriority.begin(), irqPriority.end(), 0);

    // SGIs have edge-triggered behavior
    for (uint32_t int_id = 0; int_id < Gicv3::SGI_MAX; int_id++) {
        irqConfig[int_id] = Gicv3::INT_EDGE_TRIGGERED;
    }

    std::fill(irqGrpmod.begin(), irqGrpmod.end(), 0);
    std::fill(irqNsacr.begin(), irqNsacr.end(), 0);
    DPG1S = false;
    DPG1NS = false;
    DPG0 = false;
}

uint64_t
Gicv3Redistributor::read(Addr addr, size_t size, bool is_secure_access)
{
    if (GICR_IPRIORITYR.contains(addr)) { // Interrupt Priority Registers
        uint64_t value = 0;
        int first_intid = addr - GICR_IPRIORITYR.start();

        for (int i = 0, int_id = first_intid; i < size; i++, int_id++) {
            uint8_t prio = irqPriority[int_id];

            if (!distributor->DS && !is_secure_access) {
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    // RAZ/WI for non-secure accesses for secure interrupts
                    continue;
                } else {
                    // NS view
                    prio = (prio << 1) & 0xff;
                }
            }

            value |= prio << (i * 8);
        }

        return value;
    }

    switch (addr) {
      case GICR_CTLR: { // Control Register
          uint64_t value = 0;

          if (DPG1S) {
              value |= GICR_CTLR_DPG1S;
          }

          if (DPG1NS) {
              value |= GICR_CTLR_DPG1NS;
          }

          if (DPG0) {
              value |= GICR_CTLR_DPG0;
          }

          return value;
      }

      case GICR_IIDR: // Implementer Identification Register
        //return 0x43b; // r0p0 GIC-500
        return 0;

      case GICR_TYPER: { // Type Register
          /*
           * Affinity_Value   [63:32] == X
           * (The identity of the PE associated with this Redistributor)
           * CommonLPIAff     [25:24] == 01
           * (All Redistributors with the same Aff3 value must share an
           * LPI Configuration table)
           * Processor_Number [23:8]  == X
           * (A unique identifier for the PE)
           * DPGS             [5]     == 1
           * (GICR_CTLR.DPG* bits are supported)
           * Last             [4]     == X
           * (This Redistributor is the highest-numbered Redistributor in
           * a series of contiguous Redistributor pages)
           * DirectLPI        [3]     == 0
           * (direct injection of LPIs not supported)
           * VLPIS            [1]     == 0
           * (virtual LPIs not supported)
           * PLPIS            [0]     == 0
           * (physical LPIs not supported)
           */
          uint64_t affinity = getAffinity();
          int last = cpuId == (gic->getSystem()->numContexts() - 1);
          return (affinity << 32) | (1 << 24) | (cpuId << 8) |
              (1 << 5) | (last << 4);
      }

      case GICR_WAKER: // Wake Register
        if (!distributor->DS && !is_secure_access) {
            // RAZ/WI for non-secure accesses
            return 0;
        }

        if (peInLowPowerState) {
            return GICR_WAKER_ChildrenAsleep | GICR_WAKER_ProcessorSleep;
        } else {
            return 0;
        }

      case GICR_PIDR0: { // Peripheral ID0 Register
          uint8_t part_0 = 0x92; // Part number, bits[7:0]
          return part_0;
      }

      case GICR_PIDR1: { // Peripheral ID1 Register
          uint8_t des_0 = 0xB; // JEP106 identification code, bits[3:0]
          uint8_t part_1 = 0x4; // Part number, bits[11:8]
          return (des_0 << 4) | (part_1 << 0);
      }

      case GICR_PIDR2: { // Peripheral ID2 Register
          uint8_t arch_rev = 0x3; // 0x3 GICv3
          uint8_t jedec = 0x1; // JEP code
          uint8_t des_1 = 0x3; // JEP106 identification code, bits[6:4]
          return (arch_rev << 4) | (jedec << 3) | (des_1 << 0);
      }

      case GICR_PIDR3: // Peripheral ID3 Register
        return 0x0; // Implementation defined

      case GICR_PIDR4: { // Peripheral ID4 Register
          uint8_t size = 0x4; // 64 KB software visible page
          uint8_t des_2 = 0x4; // ARM implementation
          return (size << 4) | (des_2 << 0);
      }

      case GICR_PIDR5: // Peripheral ID5 Register
      case GICR_PIDR6: // Peripheral ID6 Register
      case GICR_PIDR7: // Peripheral ID7 Register
        return 0; // RES0

      case GICR_IGROUPR0: { // Interrupt Group Register 0
          uint64_t value = 0;

          if (!distributor->DS && !is_secure_access) {
              // RAZ/WI for non-secure accesses
              return 0;
          }

          for (int int_id = 0; int_id < 8 * size; int_id++) {
              value |= (irqGroup[int_id] << int_id);
          }

          return value;
      }

      case GICR_ISENABLER0: // Interrupt Set-Enable Register 0
      case GICR_ICENABLER0: { // Interrupt Clear-Enable Register 0
          uint64_t value = 0;

          for (int int_id = 0; int_id < 8 * size; int_id++) {
              if (!distributor->DS && !is_secure_access) {
                  // RAZ/WI for non-secure accesses for secure interrupts
                  if (getIntGroup(int_id) != Gicv3::G1NS) {
                      continue;
                  }
              }

              if (irqEnabled[int_id]) {
                  value |= (1 << int_id);
              }
          }

          return value;
      }

      case GICR_ISPENDR0: // Interrupt Set-Pending Register 0
      case GICR_ICPENDR0: { // Interrupt Clear-Pending Register 0
          uint64_t value = 0;

          for (int int_id = 0; int_id < 8 * size; int_id++) {
              if (!distributor->DS && !is_secure_access) {
                  // RAZ/WI for non-secure accesses for secure interrupts
                  if (getIntGroup(int_id) != Gicv3::G1NS) {
                      continue;
                  }
              }

              value |= (irqPending[int_id] << int_id);
          }

          return value;
      }

      case GICR_ISACTIVER0: // Interrupt Set-Active Register 0
      case GICR_ICACTIVER0: { // Interrupt Clear-Active Register 0
          uint64_t value = 0;

          for (int int_id = 0; int_id < 8 * size; int_id++) {
              if (!distributor->DS && !is_secure_access) {
                  // RAZ/WI for non-secure accesses for secure interrupts
                  if (getIntGroup(int_id) != Gicv3::G1NS) {
                      continue;
                  }
              }

              value |=  irqActive[int_id] << int_id;
          }

          return value;
      }

      case GICR_ICFGR0: // SGI Configuration Register
      case GICR_ICFGR1: { // PPI Configuration Register
          uint64_t value = 0;
          uint32_t first_int_id = addr == GICR_ICFGR0 ? 0 : Gicv3::SGI_MAX;

          for (int i = 0, int_id = first_int_id; i < 32;
                  i = i + 2, int_id++) {
              if (!distributor->DS && !is_secure_access) {
                  // RAZ/WI for non-secure accesses for secure interrupts
                  if (getIntGroup(int_id) != Gicv3::G1NS) {
                      continue;
                  }
              }

              if (irqConfig[int_id] == Gicv3::INT_EDGE_TRIGGERED) {
                  value |= (0x2) << i;
              }
          }

          return value;
      }

      case GICR_IGRPMODR0: { // Interrupt Group Modifier Register 0
          uint64_t value = 0;

          if (distributor->DS) {
              value = 0;
          } else {
              if (!is_secure_access) {
                  // RAZ/WI for non-secure accesses
                  value = 0;
              } else {
                  for (int int_id = 0; int_id < 8 * size; int_id++) {
                      value |= irqGrpmod[int_id] << int_id;
                  }
              }
          }

          return value;
      }

      case GICR_NSACR: { // Non-secure Access Control Register
          uint64_t value = 0;

          if (distributor->DS) {
              // RAZ/WI
              value = 0;
          } else {
              if (!is_secure_access) {
                  // RAZ/WI
                  value = 0;
              } else {
                  for (int i = 0, int_id = 0; i < 8 * size;
                          i = i + 2, int_id++) {
                      value |= irqNsacr[int_id] << i;
                  }
              }
          }

          return value;
      }

      default:
        panic("Gicv3Redistributor::read(): invalid offset %#x\n", addr);
        break;
    }
}

void
Gicv3Redistributor::write(Addr addr, uint64_t data, size_t size,
                          bool is_secure_access)
{
    if (GICR_IPRIORITYR.contains(addr)) { // Interrupt Priority Registers
        int first_intid = addr - GICR_IPRIORITYR.start();

        for (int i = 0, int_id = first_intid; i < size; i++, int_id++) {
            uint8_t prio = bits(data, (i + 1) * 8 - 1, (i * 8));

            if (!distributor->DS && !is_secure_access) {
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    // RAZ/WI for non-secure accesses for secure interrupts
                    continue;
                } else {
                    // NS view
                    prio = 0x80 | (prio >> 1);
                }
            }

            irqPriority[int_id] = prio;
            DPRINTF(GIC, "Gicv3Redistributor::write(): "
                    "int_id %d priority %d\n", int_id, irqPriority[int_id]);
        }

        return;
    }

    switch (addr) {
      case GICR_CTLR: {
          // GICR_TYPER.LPIS is 0 so Enable_LPIs is RES0
          DPG1S = data & GICR_CTLR_DPG1S;
          DPG1NS = data & GICR_CTLR_DPG1NS;
          DPG0 = data & GICR_CTLR_DPG0;
          break;
      }

      case GICR_WAKER: // Wake Register
        if (!distributor->DS && !is_secure_access) {
            // RAZ/WI for non-secure accesses
            return;
        }

        if (not peInLowPowerState and
                (data & GICR_WAKER_ProcessorSleep)) {
            DPRINTF(GIC, "Gicv3Redistributor::write(): "
                    "PE entering in low power state\n");
        } else if (peInLowPowerState and
                not(data & GICR_WAKER_ProcessorSleep)) {
            DPRINTF(GIC, "Gicv3Redistributor::write(): powering up PE\n");
        }

        peInLowPowerState = data & GICR_WAKER_ProcessorSleep;
        break;

      case GICR_IGROUPR0: // Interrupt Group Register 0
        if (!distributor->DS && !is_secure_access) {
            // RAZ/WI for non-secure accesses
            return;
        }

        for (int int_id = 0; int_id < 8 * size; int_id++) {
            irqGroup[int_id] = data & (1 << int_id) ? 1 : 0;
            DPRINTF(GIC, "Gicv3Redistributor::write(): "
                    "int_id %d group %d\n", int_id, irqGroup[int_id]);
        }

        break;

      case GICR_ISENABLER0: // Interrupt Set-Enable Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool enable = data & (1 << int_id) ? 1 : 0;

            if (enable) {
                irqEnabled[int_id] = true;
            }

            DPRINTF(GIC, "Gicv3Redistributor::write(): "
                    "int_id %d enable %i\n", int_id, irqEnabled[int_id]);
        }

        break;

      case GICR_ICENABLER0: // Interrupt Clear-Enable Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool disable = data & (1 << int_id) ? 1 : 0;

            if (disable) {
                irqEnabled[int_id] = false;
            }

            DPRINTF(GIC, "Gicv3Redistributor::write(): "
                    "int_id %d enable %i\n", int_id, irqEnabled[int_id]);
        }

        break;

      case GICR_ISPENDR0: // Interrupt Set-Pending Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool pending = data & (1 << int_id) ? 1 : 0;

            if (pending) {
                DPRINTF(GIC, "Gicv3Redistributor::write() "
                        "(GICR_ISPENDR0): int_id %d (PPI) "
                        "pending bit set\n", int_id);
                irqPending[int_id] = true;
            }
        }

        updateAndInformCPUInterface();
        break;

      case GICR_ICPENDR0:// Interrupt Clear-Pending Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool clear = data & (1 << int_id) ? 1 : 0;

            if (clear) {
                irqPending[int_id] = false;
            }
        }

        break;

      case GICR_ISACTIVER0: // Interrupt Set-Active Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool activate = data & (1 << int_id) ? 1 : 0;

            if (activate) {
                if (!irqActive[int_id]) {
                    DPRINTF(GIC, "Gicv3Redistributor::write(): "
                            "int_id %d active set\n", int_id);
                }

                irqActive[int_id] = true;
            }
        }

        break;

      case GICR_ICACTIVER0: // Interrupt Clear-Active Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool clear = data & (1 << int_id) ? 1 : 0;

            if (clear) {
                if (irqActive[int_id]) {
                    DPRINTF(GIC, "Gicv3Redistributor::write(): "
                            "int_id %d active cleared\n", int_id);
                }

                irqActive[int_id] = false;
            }
        }

        break;

      case GICR_ICFGR1: { // PPI Configuration Register
          int first_intid = Gicv3::SGI_MAX;

          for (int i = 0, int_id = first_intid; i < 8 * size;
                  i = i + 2, int_id++) {
              if (!distributor->DS && !is_secure_access) {
                  // RAZ/WI for non-secure accesses for secure interrupts
                  if (getIntGroup(int_id) != Gicv3::G1NS) {
                      continue;
                  }
              }

              irqConfig[int_id] = data & (0x2 << i)
                  ? Gicv3::INT_EDGE_TRIGGERED :
                  Gicv3::INT_LEVEL_SENSITIVE;
              DPRINTF(GIC, "Gicv3Redistributor::write(): "
                      "int_id %d (PPI) config %d\n",
                      int_id, irqConfig[int_id]);
          }

          break;
      }

      case GICR_IGRPMODR0: { // Interrupt Group Modifier Register 0
          if (distributor->DS) {
              // RAZ/WI if secutiry disabled
          } else {
              for (int int_id = 0; int_id < 8 * size; int_id++) {
                  if (!is_secure_access) {
                      // RAZ/WI for non-secure accesses
                      continue;
                  }

                  irqGrpmod[int_id] = data & (1 << int_id);
              }
          }

          break;
      }

      case GICR_NSACR: { // Non-secure Access Control Register
          if (distributor->DS) {
              // RAZ/WI
          } else {
              if (!is_secure_access) {
                  // RAZ/WI
              } else {
                  for (int i = 0, int_id = 0; i < 8 * size;
                          i = i + 2, int_id++) {
                      irqNsacr[int_id] = (data >> i) & 0x3;
                  }
              }
          }

          break;
      }

      default:
        panic("Gicv3Redistributor::write(): invalid offset %#x\n", addr);
        break;
    }
}

void
Gicv3Redistributor::sendPPInt(uint32_t int_id)
{
    assert((int_id >= Gicv3::SGI_MAX) &&
           (int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX));
    irqPending[int_id] = true;
    DPRINTF(GIC, "Gicv3Redistributor::sendPPInt(): "
            "int_id %d (PPI) pending bit set\n", int_id);
    updateAndInformCPUInterface();
}

void
Gicv3Redistributor::sendSGI(uint32_t int_id, Gicv3::GroupId group, bool ns)
{
    assert(int_id < Gicv3::SGI_MAX);
    Gicv3::GroupId int_group = getIntGroup(int_id);

    // asked for secure group 1
    // configured as group 0
    // send group 0
    if (int_group == Gicv3::G0S && group == Gicv3::G1S) {
        group = Gicv3::G0S;
    }

    if (group == Gicv3::G0S and int_group != Gicv3::G0S) {
        return;
    }

    if (ns && distributor->DS == 0) {
        int nsaccess = irqNsacr[int_id];

        if ((int_group == Gicv3::G0S && nsaccess < 1) ||
                (int_group == Gicv3::G1S && nsaccess < 2)) {
            return;
        }
    }

    irqPending[int_id] = true;
    DPRINTF(GIC, "Gicv3ReDistributor::sendSGI(): "
            "int_id %d (SGI) pending bit set\n", int_id);
    updateAndInformCPUInterface();
}

Gicv3::IntStatus
Gicv3Redistributor::intStatus(uint32_t int_id)
{
    assert(int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX);

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

/*
 * Recalculate the highest priority pending interrupt after a
 * change to redistributor state.
 */
void
Gicv3Redistributor::update()
{
    bool new_hppi = false;

    for (int int_id = 0; int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX; int_id++) {
        Gicv3::GroupId int_group = getIntGroup(int_id);
        bool group_enabled = distributor->groupEnabled(int_group);

        if (irqPending[int_id] && irqEnabled[int_id] &&
                !irqActive[int_id] && group_enabled) {
            if ((irqPriority[int_id] < cpuInterface->hppi.prio) ||
                    /*
                     * Multiple pending ints with same priority.
                     * Implementation choice which one to signal.
                     * Our implementation selects the one with the lower id.
                     */
                    (irqPriority[int_id] == cpuInterface->hppi.prio &&
                     int_id < cpuInterface->hppi.intid)) {
                cpuInterface->hppi.intid = int_id;
                cpuInterface->hppi.prio = irqPriority[int_id];
                cpuInterface->hppi.group = int_group;
                new_hppi = true;
            }
        }
    }

    if (!new_hppi && cpuInterface->hppi.prio != 0xff &&
            cpuInterface->hppi.intid < Gicv3::SGI_MAX + Gicv3::PPI_MAX) {
        distributor->fullUpdate();
    }
}

void
Gicv3Redistributor::updateAndInformCPUInterface()
{
    update();
    cpuInterface->update();
}

Gicv3::GroupId
Gicv3Redistributor::getIntGroup(int int_id)
{
    assert(int_id < (Gicv3::SGI_MAX + Gicv3::PPI_MAX));

    if (distributor->DS) {
        if (irqGroup[int_id] == 0) {
            return Gicv3::G0S;
        } else {
            return Gicv3::G1NS;
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
Gicv3Redistributor::activateIRQ(uint32_t int_id)
{
    irqPending[int_id] = false;
    irqActive[int_id] = true;
}

void
Gicv3Redistributor::deactivateIRQ(uint32_t int_id)
{
    irqActive[int_id] = false;
}

uint32_t
Gicv3Redistributor::getAffinity()
{
    ThreadContext * tc = gic->getSystem()->getThreadContext(cpuId);
    uint64_t mpidr = getMPIDR(gic->getSystem(), tc);
    /*
     * Aff3 = MPIDR[39:32]
     * (Note getMPIDR() returns uint32_t so Aff3 is always 0...)
     * Aff2 = MPIDR[23:16]
     * Aff1 = MPIDR[15:8]
     * Aff0 = MPIDR[7:0]
     * affinity = Aff3.Aff2.Aff1.Aff0
     */
    uint64_t affinity = ((mpidr & 0xff00000000) >> 8) | (mpidr & (0xffffff));
    return affinity;
}

bool
Gicv3Redistributor::canBeSelectedFor1toNInterrupt(Gicv3::GroupId group)
{
    if (peInLowPowerState) {
        return false;
    }

    if (!distributor->groupEnabled(group)) {
        return false;
    }

    if ((group == Gicv3::G1S) && DPG1S) {
        return false;
    }

    if ((group == Gicv3::G1NS) && DPG1NS) {
        return false;
    }

    if ((group == Gicv3::G0S) && DPG0) {
        return false;
    }

    return true;
}

void
Gicv3Redistributor::serialize(CheckpointOut & cp) const
{
    SERIALIZE_SCALAR(peInLowPowerState);
    SERIALIZE_CONTAINER(irqGroup);
    SERIALIZE_CONTAINER(irqEnabled);
    SERIALIZE_CONTAINER(irqPending);
    SERIALIZE_CONTAINER(irqActive);
    SERIALIZE_CONTAINER(irqPriority);
    SERIALIZE_CONTAINER(irqConfig);
    SERIALIZE_CONTAINER(irqGrpmod);
    SERIALIZE_CONTAINER(irqNsacr);
    SERIALIZE_SCALAR(DPG1S);
    SERIALIZE_SCALAR(DPG1NS);
    SERIALIZE_SCALAR(DPG0);
}

void
Gicv3Redistributor::unserialize(CheckpointIn & cp)
{
    UNSERIALIZE_SCALAR(peInLowPowerState);
    UNSERIALIZE_CONTAINER(irqGroup);
    UNSERIALIZE_CONTAINER(irqEnabled);
    UNSERIALIZE_CONTAINER(irqPending);
    UNSERIALIZE_CONTAINER(irqActive);
    UNSERIALIZE_CONTAINER(irqPriority);
    UNSERIALIZE_CONTAINER(irqConfig);
    UNSERIALIZE_CONTAINER(irqGrpmod);
    UNSERIALIZE_CONTAINER(irqNsacr);
    UNSERIALIZE_SCALAR(DPG1S);
    UNSERIALIZE_SCALAR(DPG1NS);
    UNSERIALIZE_SCALAR(DPG0);
}
