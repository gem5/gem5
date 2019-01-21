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

#include "dev/arm/gic_v3_cpu_interface.hh"

#include "arch/arm/isa.hh"
#include "debug/GIC.hh"
#include "dev/arm/gic_v3.hh"
#include "dev/arm/gic_v3_distributor.hh"
#include "dev/arm/gic_v3_redistributor.hh"

Gicv3CPUInterface::Gicv3CPUInterface(Gicv3 * gic, uint32_t cpu_id)
    : BaseISADevice(),
      gic(gic),
      redistributor(nullptr),
      distributor(nullptr),
      cpuId(cpu_id)
{
}

Gicv3CPUInterface::~Gicv3CPUInterface()
{
}

void
Gicv3CPUInterface::init()
{
    redistributor = gic->getRedistributor(cpuId);
    distributor = gic->getDistributor();
}

void
Gicv3CPUInterface::initState()
{
    reset();
}

void
Gicv3CPUInterface::reset()
{
    hppi.prio = 0xff;
}

bool
Gicv3CPUInterface::getHCREL2FMO()
{
    HCR hcr = isa->readMiscRegNoEffect(MISCREG_HCR_EL2);

    if (hcr.tge && hcr.e2h) {
        return false;
    } else if (hcr.tge) {
        return true;
    } else {
        return hcr.fmo;
    }
}

bool
Gicv3CPUInterface::getHCREL2IMO()
{
    HCR hcr = isa->readMiscRegNoEffect(MISCREG_HCR_EL2);

    if (hcr.tge && hcr.e2h) {
        return false;
    } else if (hcr.tge) {
        return true;
    } else {
        return hcr.imo;
    }
}

RegVal
Gicv3CPUInterface::readMiscReg(int misc_reg)
{
    RegVal value = isa->readMiscRegNoEffect(misc_reg);
    bool hcr_fmo = getHCREL2FMO();
    bool hcr_imo = getHCREL2IMO();

    switch (misc_reg) {
      case MISCREG_ICC_AP1R0:
      case MISCREG_ICC_AP1R0_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
              return isa->readMiscRegNoEffect(MISCREG_ICV_AP1R0_EL1);
          }

          break;
      }

      case MISCREG_ICC_AP1R1:
      case MISCREG_ICC_AP1R1_EL1:

        // only implemented if supporting 6 or more bits of priority
      case MISCREG_ICC_AP1R2:
      case MISCREG_ICC_AP1R2_EL1:

        // only implemented if supporting 7 or more bits of priority
      case MISCREG_ICC_AP1R3:
      case MISCREG_ICC_AP1R3_EL1:
        // only implemented if supporting 7 or more bits of priority
        return 0;

      case MISCREG_ICC_AP0R0:
      case MISCREG_ICC_AP0R0_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
              return isa->readMiscRegNoEffect(MISCREG_ICV_AP0R0_EL1);
          }

          break;
      }

      case MISCREG_ICC_AP0R1:
      case MISCREG_ICC_AP0R1_EL1:

        // only implemented if supporting 6 or more bits of priority
      case MISCREG_ICC_AP0R2:
      case MISCREG_ICC_AP0R2_EL1:

        // only implemented if supporting 7 or more bits of priority
      case MISCREG_ICC_AP0R3:
      case MISCREG_ICC_AP0R3_EL1:
        // only implemented if supporting 7 or more bits of priority
        return 0;

      case MISCREG_ICC_IGRPEN0:
      case MISCREG_ICC_IGRPEN0_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
              return isa->readMiscRegNoEffect(MISCREG_ICV_IGRPEN0_EL1);
          }

          break;
      }

      case MISCREG_ICC_IGRPEN1:
      case MISCREG_ICC_IGRPEN1_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
              return isa->readMiscRegNoEffect(MISCREG_ICV_IGRPEN1_EL1);
          }

          break;
      }

      case MISCREG_ICC_MGRPEN1:
      case MISCREG_ICC_IGRPEN1_EL3: {
          // EnableGrp1S and EnableGrp1NS are aliased with
          // ICC_IGRPEN1_EL1_S.Enable and ICC_IGRPEN1_EL1_NS.Enable
          bool enable_grp_1s =
              isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1_S) &
              ICC_IGRPEN1_EL1_ENABLE;
          bool enable_grp_1ns =
              isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1_NS) &
              ICC_IGRPEN1_EL1_ENABLE;
          value = 0;

          if (enable_grp_1s) {
              value |= ICC_IGRPEN1_EL3_ENABLEGRP1S;
          }

          if (enable_grp_1ns) {
              value |= ICC_IGRPEN1_EL3_ENABLEGRP1NS;
          }

          break;
      }

      case MISCREG_ICC_RPR:
      case MISCREG_ICC_RPR_EL1: {
          if ((currEL() == EL1) && !inSecureState() &&
                  (hcr_imo || hcr_fmo)) {
              return readMiscReg(MISCREG_ICV_RPR_EL1);
          }

          uint8_t rprio = highestActivePriority();

          if (haveEL(EL3) && !inSecureState() &&
                  (isa->readMiscRegNoEffect(MISCREG_SCR_EL3) & (1U << 2))) {
              /* NS GIC access and Group 0 is inaccessible to NS */
              if ((rprio & 0x80) == 0) {
                  /* NS should not see priorities in the Secure half of the
                   * range */
                  rprio = 0;
              } else if (rprio != 0xff) {
                  /* Non-idle priority: show the Non-secure view of it */
                  rprio = (rprio << 1) & 0xff;
              }
          }

          value = rprio;
          break;
      }

      case MISCREG_ICV_RPR_EL1: {
          value = virtualHighestActivePriority();
          break;
      }

      case MISCREG_ICC_HPPIR0:
      case MISCREG_ICC_HPPIR0_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
              return readMiscReg(MISCREG_ICV_HPPIR0_EL1);
          }

          value = getHPPIR0();
          break;
      }

      case MISCREG_ICV_HPPIR0_EL1: {
          value = Gicv3::INTID_SPURIOUS;
          int lr_idx = getHPPVILR();

          if (lr_idx >= 0) {
              RegVal lr =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);
              Gicv3::GroupId group =
                  lr & ICH_LR_EL2_GROUP ? Gicv3::G1NS : Gicv3::G0S;

              if (group == Gicv3::G0S) {
                  value = bits(lr, 31, 0);
              }
          }

          break;
      }

      case MISCREG_ICC_HPPIR1:
      case MISCREG_ICC_HPPIR1_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
              return readMiscReg(MISCREG_ICV_HPPIR1_EL1);
          }

          value = getHPPIR1();
          break;
      }

      case MISCREG_ICV_HPPIR1_EL1: {
          value = Gicv3::INTID_SPURIOUS;
          int lr_idx = getHPPVILR();

          if (lr_idx >= 0) {
              RegVal lr =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);
              Gicv3::GroupId group =
                  lr & ICH_LR_EL2_GROUP ? Gicv3::G1NS : Gicv3::G0S;

              if (group == Gicv3::G1NS) {
                  value = bits(lr, 31, 0);
              }
          }

          break;
      }

      case MISCREG_ICC_BPR0:
      case MISCREG_ICC_BPR0_EL1:
        if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
            return readMiscReg(MISCREG_ICV_BPR0_EL1);
        }

        M5_FALLTHROUGH;

      case MISCREG_ICC_BPR1:
      case MISCREG_ICC_BPR1_EL1:
        if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
            return readMiscReg(MISCREG_ICV_BPR1_EL1);
        }

        {
            Gicv3::GroupId group =
                misc_reg == MISCREG_ICC_BPR0_EL1 ? Gicv3::G0S : Gicv3::G1S;

            if (group == Gicv3::G1S && !inSecureState()) {
                group = Gicv3::G1NS;
            }

            if ((group == Gicv3::G1S) &&
                    !isEL3OrMon() &&
                    (isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S)
                     & ICC_CTLR_EL1_CBPR)) {
                group = Gicv3::G0S;
            }

            bool sat_inc = false;

            if ((group == Gicv3::G1NS) &&
                    (currEL() < EL3) &&
                    (isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS)
                     & ICC_CTLR_EL1_CBPR)) {
                // Reads return BPR0 + 1 saturated to 7, WI
                group = Gicv3::G0S;
                sat_inc = true;
            }

            uint8_t bpr;

            if (group == Gicv3::G0S) {
                bpr = isa->readMiscRegNoEffect(MISCREG_ICC_BPR0_EL1);
            } else {
                bpr = isa->readMiscRegNoEffect(MISCREG_ICC_BPR1_EL1);
            }

            if (sat_inc) {
                bpr++;

                if (bpr > 7) {
                    bpr = 7;
                }
            }

            value = bpr;
            break;
        }

      case MISCREG_ICV_BPR0_EL1:
      case MISCREG_ICV_BPR1_EL1: {
          Gicv3::GroupId group =
              misc_reg == MISCREG_ICV_BPR0_EL1 ? Gicv3::G0S : Gicv3::G1NS;
          RegVal ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
          bool sat_inc = false;

          if (group == Gicv3::G1NS && (ich_vmcr_el2 & ICH_VMCR_EL2_VCBPR)) {
              // reads return bpr0 + 1 saturated to 7, writes ignored
              group = Gicv3::G0S;
              sat_inc = true;
          }

          uint8_t vbpr;

          if (group == Gicv3::G0S) {
              vbpr = bits(ich_vmcr_el2, 23, 21);
          } else {
              vbpr = bits(ich_vmcr_el2, 20, 18);
          }

          if (sat_inc) {
              vbpr++;

              if (vbpr > 7) {
                  vbpr = 7;
              }
          }

          value = vbpr;
          break;
      }

      case MISCREG_ICC_PMR:
      case MISCREG_ICC_PMR_EL1: // Priority Mask Register
        if ((currEL() == EL1) && !inSecureState() &&
                (hcr_imo || hcr_fmo)) {
            return isa->readMiscRegNoEffect(MISCREG_ICV_PMR_EL1);
        }

        if (haveEL(EL3) && !inSecureState() &&
                (isa->readMiscRegNoEffect(MISCREG_SCR_EL3) & (1U << 2))) {
            /* NS GIC access and Group 0 is inaccessible to NS */
            if ((value & 0x80) == 0) {
                /* NS should not see priorities in the Secure half of the
                 * range */
                value = 0;
            } else if (value != 0xff) {
                /* Non-idle priority: show the Non-secure view of it */
                value = (value << 1) & 0xff;
            }
        }

        break;

      case MISCREG_ICC_IAR0:
      case MISCREG_ICC_IAR0_EL1: { // Interrupt Acknowledge Register 0
          if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
              return readMiscReg(MISCREG_ICV_IAR0_EL1);
          }

          uint32_t int_id;

          if (hppiCanPreempt()) {
              int_id = getHPPIR0();

              // avoid activation for special interrupts
              if (int_id < Gicv3::INTID_SECURE) {
                  activateIRQ(int_id, hppi.group);
              }
          } else {
              int_id = Gicv3::INTID_SPURIOUS;
          }

          value = int_id;
          break;
      }

      case MISCREG_ICV_IAR0_EL1: {
          int lr_idx = getHPPVILR();
          uint32_t int_id = Gicv3::INTID_SPURIOUS;

          if (lr_idx >= 0) {
              RegVal lr =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

              if (!(lr & ICH_LR_EL2_GROUP) && hppviCanPreempt(lr_idx)) {
                  int_id = value = bits(lr, 31, 0);

                  if (int_id < Gicv3::INTID_SECURE ||
                          int_id > Gicv3::INTID_SPURIOUS) {
                      virtualActivateIRQ(lr_idx);
                  } else {
                      // Bogus... Pseudocode says:
                      // - Move from pending to invalid...
                      // - Return de bogus id...
                      lr &= ~ICH_LR_EL2_STATE_PENDING_BIT;
                      isa->setMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx,
                              lr);
                  }
              }
          }

          value = int_id;
          virtualUpdate();
          break;
      }

      case MISCREG_ICC_IAR1:
      case MISCREG_ICC_IAR1_EL1: { // Interrupt Acknowledge Register 1
          if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
              return readMiscReg(MISCREG_ICV_IAR1_EL1);
          }

          uint32_t int_id;

          if (hppiCanPreempt()) {
              int_id = getHPPIR1();

              // avoid activation for special interrupts
              if (int_id < Gicv3::INTID_SECURE) {
                  activateIRQ(int_id, hppi.group);
              }
          } else {
              int_id = Gicv3::INTID_SPURIOUS;
          }

          value = int_id;
          break;
      }

      case MISCREG_ICV_IAR1_EL1: {
          int lr_idx = getHPPVILR();
          uint32_t int_id = Gicv3::INTID_SPURIOUS;

          if (lr_idx >= 0) {
              RegVal lr =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

              if (lr & ICH_LR_EL2_GROUP && hppviCanPreempt(lr_idx)) {
                  int_id = value = bits(lr, 31, 0);

                  if (int_id < Gicv3::INTID_SECURE ||
                          int_id > Gicv3::INTID_SPURIOUS) {
                      virtualActivateIRQ(lr_idx);
                  } else {
                      // Bogus... Pseudocode says:
                      // - Move from pending to invalid...
                      // - Return de bogus id...
                      lr &= ~ICH_LR_EL2_STATE_PENDING_BIT;
                      isa->setMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx,
                              lr);
                  }
              }
          }

          value = int_id;
          virtualUpdate();
          break;
      }

      case MISCREG_ICC_SRE:
      case MISCREG_ICC_SRE_EL1: { // System Register Enable Register
          bool dfb;
          bool dib;

          if (haveEL(EL3) && !distributor->DS) {
              // DIB is RO alias of ICC_SRE_EL3.DIB
              // DFB is RO alias of ICC_SRE_EL3.DFB
              RegVal icc_sre_el3 =
                  isa->readMiscRegNoEffect(MISCREG_ICC_SRE_EL3);
              dfb = icc_sre_el3 & ICC_SRE_EL3_DFB;
              dib = icc_sre_el3 & ICC_SRE_EL3_DIB;
          } else if (haveEL(EL3) && distributor->DS) {
              // DIB is RW alias of ICC_SRE_EL3.DIB
              // DFB is RW alias of ICC_SRE_EL3.DFB
              RegVal icc_sre_el3 =
                  isa->readMiscRegNoEffect(MISCREG_ICC_SRE_EL3);
              dfb = icc_sre_el3 & ICC_SRE_EL3_DFB;
              dib = icc_sre_el3 & ICC_SRE_EL3_DIB;
          } else if ((!haveEL(EL3) || distributor->DS) and haveEL(EL2)) {
              // DIB is RO alias of ICC_SRE_EL2.DIB
              // DFB is RO alias of ICC_SRE_EL2.DFB
              RegVal icc_sre_el2 =
                  isa->readMiscRegNoEffect(MISCREG_ICC_SRE_EL2);
              dfb = icc_sre_el2 & ICC_SRE_EL2_DFB;
              dib = icc_sre_el2 & ICC_SRE_EL2_DIB;
          } else {
              dfb = value & ICC_SRE_EL1_DFB;
              dib = value & ICC_SRE_EL1_DIB;
          }

          value = ICC_SRE_EL1_SRE;

          if (dfb) {
              value |= ICC_SRE_EL1_DFB;
          }

          if (dib) {
              value |= ICC_SRE_EL1_DIB;
          }

          break;
      }

      case MISCREG_ICC_HSRE:
      case MISCREG_ICC_SRE_EL2: // System Register Enable Register
        /*
         * Enable [3] == 1
         * (Secure EL1 accesses to Secure ICC_SRE_EL1 do not trap to EL2,
         * RAO/WI)
         * DIB [2] == 1 (IRQ bypass not supported, RAO/WI)
         * DFB [1] == 1 (FIQ bypass not supported, RAO/WI)
         * SRE [0] == 1 (Only system register interface supported, RAO/WI)
         */
        value = ICC_SRE_EL2_ENABLE | ICC_SRE_EL2_DIB | ICC_SRE_EL2_DFB |
            ICC_SRE_EL2_SRE;
        break;

      case MISCREG_ICC_MSRE:
      case MISCREG_ICC_SRE_EL3: // System Register Enable Register
        /*
         * Enable [3] == 1
         * (Secure EL1 accesses to Secure ICC_SRE_EL1 do not trap to EL3,
         * RAO/WI)
         * DIB [2] == 1 (IRQ bypass not supported, RAO/WI)
         * DFB [1] == 1 (FIQ bypass not supported, RAO/WI)
         * SRE [0] == 1 (Only system register interface supported, RAO/WI)
         */
        value = ICC_SRE_EL3_ENABLE | ICC_SRE_EL3_DIB | ICC_SRE_EL3_DFB |
            ICC_SRE_EL3_SRE;
        break;

      case MISCREG_ICC_CTLR:
      case MISCREG_ICC_CTLR_EL1: { // Control Register
          if ((currEL() == EL1) && !inSecureState() &&
                  (hcr_imo || hcr_fmo)) {
              return readMiscReg(MISCREG_ICV_CTLR_EL1);
          }

          // Add value for RO bits
          // IDbits [13:11], 001 = 24 bits | 000 = 16 bits
          // PRIbits [10:8], number of priority bits implemented, minus one
          value |= ICC_CTLR_EL1_RSS | ICC_CTLR_EL1_A3V |
              (1 << 11) | ((PRIORITY_BITS - 1) << 8);
          break;
      }

      case MISCREG_ICV_CTLR_EL1: {
          value = ICC_CTLR_EL1_A3V | (1 << ICC_CTLR_EL1_IDBITS_SHIFT) |
              (7 << ICC_CTLR_EL1_PRIBITS_SHIFT);
          RegVal ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);

          if (ich_vmcr_el2 & ICH_VMCR_EL2_VEOIM) {
              value |= ICC_CTLR_EL1_EOIMODE;
          }

          if (ich_vmcr_el2 & ICH_VMCR_EL2_VCBPR) {
              value |= ICC_CTLR_EL1_CBPR;
          }

          break;
      }

      case MISCREG_ICC_MCTLR:
      case MISCREG_ICC_CTLR_EL3: {
          // Add value for RO bits
          // RSS [18]
          // A3V [15]
          // IDbits [13:11], 001 = 24 bits | 000 = 16 bits
          // PRIbits [10:8], number of priority bits implemented, minus one
          value |= ICC_CTLR_EL3_RSS | ICC_CTLR_EL3_A3V | (0 << 11) |
              ((PRIORITY_BITS - 1) << 8);
          // Aliased bits...
          RegVal icc_ctlr_el1_ns =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS);
          RegVal icc_ctlr_el1_s =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S);

          if (icc_ctlr_el1_ns & ICC_CTLR_EL1_EOIMODE) {
              value |= ICC_CTLR_EL3_EOIMODE_EL1NS;
          }

          if (icc_ctlr_el1_ns & ICC_CTLR_EL1_CBPR) {
              value |= ICC_CTLR_EL3_CBPR_EL1NS;
          }

          if (icc_ctlr_el1_s & ICC_CTLR_EL1_EOIMODE) {
              value |= ICC_CTLR_EL3_EOIMODE_EL1S;
          }

          if (icc_ctlr_el1_s & ICC_CTLR_EL1_CBPR) {
              value |= ICC_CTLR_EL3_CBPR_EL1S;
          }

          break;
      }

      case MISCREG_ICH_HCR:
      case MISCREG_ICH_HCR_EL2:
        break;

      case MISCREG_ICH_AP0R0:
      case MISCREG_ICH_AP0R0_EL2:
        break;

      case MISCREG_ICH_AP1R0:
      case MISCREG_ICH_AP1R0_EL2:
        break;

      case MISCREG_ICH_MISR:
      case MISCREG_ICH_MISR_EL2: {
          value = 0;
          // Scan list registers and fill in the U, NP and EOI bits
          eoiMaintenanceInterruptStatus((uint32_t *) &value);
          RegVal ich_hcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_HCR_EL2);
          RegVal ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);

          if (ich_hcr_el2 &
                  (ICH_HCR_EL2_LRENPIE | ICH_HCR_EL2_EOICOUNT_MASK)) {
              value |= ICH_MISR_EL2_LRENP;
          }

          if ((ich_hcr_el2 & ICH_HCR_EL2_VGRP0EIE) &&
                  (ich_vmcr_el2 & ICH_VMCR_EL2_VENG0)) {
              value |= ICH_MISR_EL2_VGRP0E;
          }

          if ((ich_hcr_el2 & ICH_HCR_EL2_VGRP0DIE) &&
                  !(ich_vmcr_el2 & ICH_VMCR_EL2_VENG1)) {
              value |= ICH_MISR_EL2_VGRP0D;
          }

          if ((ich_hcr_el2 & ICH_HCR_EL2_VGRP1EIE) &&
                  (ich_vmcr_el2 & ICH_VMCR_EL2_VENG1)) {
              value |= ICH_MISR_EL2_VGRP1E;
          }

          if ((ich_hcr_el2 & ICH_HCR_EL2_VGRP1DIE) &&
                  !(ich_vmcr_el2 & ICH_VMCR_EL2_VENG1)) {
              value |= ICH_MISR_EL2_VGRP1D;
          }

          break;
      }

      case MISCREG_ICH_VTR:
      case MISCREG_ICH_VTR_EL2:
        /*
         * PRIbits [31:29]
         * PREbits [28:26]
         * IDbits [25:23]
         * SEIS [22] == 0 (SEI Support)
         * A3V [21] == 1
         * (Non-zero values supported for Affinity 3 in SGI genearion)
         * nV4 [20] == 0
         * (Support for direct injection of virtual interrupts)
         * TDS [19] == 0 (Implementation supports ICH_HCR_EL2.TDIR)
         * ListRegs [4:0]
         */
        value = (16 - 1) << 0 |
            (5 - 1) << 26 |
            (5 - 1) << 29;
        value =
            ((VIRTUAL_NUM_LIST_REGS - 1) << ICH_VTR_EL2_LISTREGS_SHIFT) |
            // ICH_VTR_EL2_TDS |
            // ICH_VTR_EL2_NV4 |
            ICH_VTR_EL2_A3V |
            (1 << ICH_VTR_EL2_IDBITS_SHIFT) |
            ((VIRTUAL_PREEMPTION_BITS - 1) << ICH_VTR_EL2_PREBITS_SHIFT) |
            ((VIRTUAL_PRIORITY_BITS - 1) << ICH_VTR_EL2_PRIBITS_SHIFT);
        break;

      case MISCREG_ICH_EISR:
      case MISCREG_ICH_EISR_EL2:
        value = eoiMaintenanceInterruptStatus(nullptr);
        break;

      case MISCREG_ICH_ELRSR:
      case MISCREG_ICH_ELRSR_EL2:
        value = 0;

        for (int lr_idx = 0; lr_idx < VIRTUAL_NUM_LIST_REGS; lr_idx++) {
            RegVal lr =
                isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

            if ((lr & ICH_LR_EL2_STATE_MASK) == 0 &&
                    ((lr & ICH_LR_EL2_HW) != 0 ||
                     (lr & ICH_LR_EL2_EOI) == 0)) {
                value |= (1 << lr_idx);
            }
        }

        break;

      case MISCREG_ICH_LRC0 ... MISCREG_ICH_LRC15:
        // AArch32 (maps to AArch64 MISCREG_ICH_LR<n>_EL2 high half part)
        value = value >> 32;
        break;

      case MISCREG_ICH_LR0 ... MISCREG_ICH_LR15:
        // AArch32 (maps to AArch64 MISCREG_ICH_LR<n>_EL2 low half part)
        value = value & 0xffffffff;
        break;

      case MISCREG_ICH_LR0_EL2 ... MISCREG_ICH_LR15_EL2:
        break;

      case MISCREG_ICH_VMCR:
      case MISCREG_ICH_VMCR_EL2:
        break;

      default:
        panic("Gicv3CPUInterface::readMiscReg(): "
                "unknown register %d (%s)",
                misc_reg, miscRegName[misc_reg]);
    }

    DPRINTF(GIC, "Gicv3CPUInterface::readMiscReg(): "
            "register %s value %#x\n", miscRegName[misc_reg], value);
    return value;
}

void
Gicv3CPUInterface::setMiscReg(int misc_reg, RegVal val)
{
    bool do_virtual_update = false;
    DPRINTF(GIC, "Gicv3CPUInterface::setMiscReg(): "
            "register %s value %#x\n", miscRegName[misc_reg], val);
    bool hcr_fmo = getHCREL2FMO();
    bool hcr_imo = getHCREL2IMO();

    switch (misc_reg) {
      case MISCREG_ICC_AP1R0:
      case MISCREG_ICC_AP1R0_EL1:
        if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
            return isa->setMiscRegNoEffect(MISCREG_ICV_AP1R0_EL1, val);
        }

        break;

      case MISCREG_ICC_AP1R1:
      case MISCREG_ICC_AP1R1_EL1:

        // only implemented if supporting 6 or more bits of priority
      case MISCREG_ICC_AP1R2:
      case MISCREG_ICC_AP1R2_EL1:

        // only implemented if supporting 7 or more bits of priority
      case MISCREG_ICC_AP1R3:
      case MISCREG_ICC_AP1R3_EL1:
        // only implemented if supporting 7 or more bits of priority
        break;

      case MISCREG_ICC_AP0R0:
      case MISCREG_ICC_AP0R0_EL1:
        if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
            return isa->setMiscRegNoEffect(MISCREG_ICV_AP0R0_EL1, val);
        }

        break;

      case MISCREG_ICC_AP0R1:
      case MISCREG_ICC_AP0R1_EL1:

        // only implemented if supporting 6 or more bits of priority
      case MISCREG_ICC_AP0R2:
      case MISCREG_ICC_AP0R2_EL1:

        // only implemented if supporting 7 or more bits of priority
      case MISCREG_ICC_AP0R3:
      case MISCREG_ICC_AP0R3_EL1:
        // only implemented if supporting 7 or more bits of priority
        break;

      case MISCREG_ICC_EOIR0:
      case MISCREG_ICC_EOIR0_EL1: { // End Of Interrupt Register 0
          if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
              return setMiscReg(MISCREG_ICV_EOIR0_EL1, val);
          }

          int int_id = val & 0xffffff;

          // avoid activation for special interrupts
          if (int_id >= Gicv3::INTID_SECURE) {
              return;
          }

          Gicv3::GroupId group = Gicv3::G0S;

          if (highestActiveGroup() != group) {
              return;
          }

          dropPriority(group);

          if (!isEOISplitMode()) {
              deactivateIRQ(int_id, group);
          }

          break;
      }

      case MISCREG_ICV_EOIR0_EL1: {
          int int_id = val & 0xffffff;

          // avoid deactivation for special interrupts
          if (int_id >= Gicv3::INTID_SECURE &&
                  int_id <= Gicv3::INTID_SPURIOUS) {
              return;
          }

          uint8_t drop_prio = virtualDropPriority();

          if (drop_prio == 0xff) {
              return;
          }

          int lr_idx = virtualFindActive(int_id);

          if (lr_idx < 0) {
              // No LR found matching
              virtualIncrementEOICount();
          } else {
              RegVal lr =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);
              Gicv3::GroupId lr_group =
                  lr & ICH_LR_EL2_GROUP ? Gicv3::G1NS : Gicv3::G0S;
              uint8_t lr_group_prio = bits(lr, 55, 48) & 0xf8;

              if (lr_group == Gicv3::G0S && lr_group_prio == drop_prio) {
                  //JAIRO if (!virtualIsEOISplitMode())
                  {
                      virtualDeactivateIRQ(lr_idx);
                  }
              }
          }

          virtualUpdate();
          break;
      }

      case MISCREG_ICC_EOIR1:
      case MISCREG_ICC_EOIR1_EL1: { // End Of Interrupt Register 1
          if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
              return setMiscReg(MISCREG_ICV_EOIR1_EL1, val);
          }

          int int_id = val & 0xffffff;

          // avoid deactivation for special interrupts
          if (int_id >= Gicv3::INTID_SECURE) {
              return;
          }

          Gicv3::GroupId group =
              inSecureState() ? Gicv3::G1S : Gicv3::G1NS;

          if (highestActiveGroup() == Gicv3::G0S) {
              return;
          }

          if (distributor->DS == 0) {
              if (highestActiveGroup() == Gicv3::G1S && !inSecureState()) {
                  return;
              } else if (highestActiveGroup() == Gicv3::G1NS &&
                      !(!inSecureState() or (currEL() == EL3))) {
                  return;
              }
          }

          dropPriority(group);

          if (!isEOISplitMode()) {
              deactivateIRQ(int_id, group);
          }

          break;
      }

      case MISCREG_ICV_EOIR1_EL1: {
          int int_id = val & 0xffffff;

          // avoid deactivation for special interrupts
          if (int_id >= Gicv3::INTID_SECURE &&
                  int_id <= Gicv3::INTID_SPURIOUS) {
              return;
          }

          uint8_t drop_prio = virtualDropPriority();

          if (drop_prio == 0xff) {
              return;
          }

          int lr_idx = virtualFindActive(int_id);

          if (lr_idx < 0) {
              // No LR found matching
              virtualIncrementEOICount();
          } else {
              RegVal lr =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);
              Gicv3::GroupId lr_group =
                  lr & ICH_LR_EL2_GROUP ? Gicv3::G1NS : Gicv3::G0S;
              uint8_t lr_group_prio = bits(lr, 55, 48) & 0xf8;

              if (lr_group == Gicv3::G1NS && lr_group_prio == drop_prio) {
                  if (!virtualIsEOISplitMode()) {
                      virtualDeactivateIRQ(lr_idx);
                  }
              }
          }

          virtualUpdate();
          break;
      }

      case MISCREG_ICC_DIR:
      case MISCREG_ICC_DIR_EL1: { // Deactivate Interrupt Register
          if ((currEL() == EL1) && !inSecureState() &&
                  (hcr_imo || hcr_fmo)) {
              return setMiscReg(MISCREG_ICV_DIR_EL1, val);
          }

          int int_id = val & 0xffffff;

          // avoid deactivation for special interrupts
          if (int_id >= Gicv3::INTID_SECURE) {
              return;
          }

          if (!isEOISplitMode()) {
              return;
          }

          /*
           * Check whether we're allowed to deactivate.
           * These checks are correspond to the spec's pseudocode.
           */
          Gicv3::GroupId group =
              int_id >= 32 ? distributor->getIntGroup(int_id) :
              redistributor->getIntGroup(int_id);
          bool irq_is_grp0 = group == Gicv3::G0S;
          bool single_sec_state = distributor->DS;
          bool irq_is_secure = !single_sec_state && (group != Gicv3::G1NS);
          SCR scr_el3 = isa->readMiscRegNoEffect(MISCREG_SCR_EL3);
          bool route_fiq_to_el3 = scr_el3.fiq;
          bool route_irq_to_el3 = scr_el3.irq;
          bool route_fiq_to_el2 = hcr_fmo;
          bool route_irq_to_el2 = hcr_imo;

          switch (currEL()) {
            case EL3:
              break;

            case EL2:
              if (single_sec_state && irq_is_grp0 && !route_fiq_to_el3) {
                  break;
              }

              if (!irq_is_secure && !irq_is_grp0 && !route_irq_to_el3) {
                  break;
              }

              return;

            case EL1:
              if (!isSecureBelowEL3()) {
                  if (single_sec_state && irq_is_grp0 &&
                          !route_fiq_to_el3 && !route_fiq_to_el2) {
                      break;
                  }

                  if (!irq_is_secure && !irq_is_grp0 &&
                          !route_irq_to_el3 && !route_irq_to_el2) {
                      break;
                  }
              } else {
                  if (irq_is_grp0 && !route_fiq_to_el3) {
                      break;
                  }

                  if (!irq_is_grp0 &&
                          (!irq_is_secure || !single_sec_state) &&
                          !route_irq_to_el3) {
                      break;
                  }
              }

              return;

            default:
              break;
          }

          deactivateIRQ(int_id, group);
          break;
      }

      case MISCREG_ICV_DIR_EL1: {
          int int_id = val & 0xffffff;

          // avoid deactivation for special interrupts
          if (int_id >= Gicv3::INTID_SECURE &&
                  int_id <= Gicv3::INTID_SPURIOUS) {
              return;
          }

          if (!virtualIsEOISplitMode()) {
              return;
          }

          int lr_idx = virtualFindActive(int_id);

          if (lr_idx < 0) {
              // No LR found matching
              virtualIncrementEOICount();
          } else {
              virtualDeactivateIRQ(lr_idx);
          }

          virtualUpdate();
          break;
      }

      case MISCREG_ICC_BPR0:
      case MISCREG_ICC_BPR0_EL1: // Binary Point Register 0
      case MISCREG_ICC_BPR1:
      case MISCREG_ICC_BPR1_EL1: { // Binary Point Register 1
          if ((currEL() == EL1) && !inSecureState()) {
              if (misc_reg == MISCREG_ICC_BPR0_EL1 && hcr_fmo) {
                  return setMiscReg(MISCREG_ICV_BPR0_EL1, val);
              } else if (misc_reg == MISCREG_ICC_BPR1_EL1 && hcr_imo) {
                  return setMiscReg(MISCREG_ICV_BPR1_EL1, val);
              }
          }

          Gicv3::GroupId group =
              misc_reg == MISCREG_ICC_BPR0_EL1 ? Gicv3::G0S : Gicv3::G1S;

          if (group == Gicv3::G1S && !inSecureState()) {
              group = Gicv3::G1NS;
          }

          if ((group == Gicv3::G1S) &&
                  !isEL3OrMon() &&
                  (isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S) &
                   ICC_CTLR_EL1_CBPR)) {
              group = Gicv3::G0S;
          }

          if ((group == Gicv3::G1NS) &&
                  (currEL() < EL3) &&
                  (isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS) &
                   ICC_CTLR_EL1_CBPR)) {
              // Reads return BPR0 + 1 saturated to 7, WI
              return;
          }

          uint8_t min_val = (group == Gicv3::G1NS) ?
              GIC_MIN_BPR_NS : GIC_MIN_BPR;
          val &= 0x7;

          if (val < min_val) {
              val = min_val;
          }

          break;
      }

      case MISCREG_ICV_BPR0_EL1:
      case MISCREG_ICV_BPR1_EL1: {
          Gicv3::GroupId group =
              misc_reg == MISCREG_ICV_BPR0_EL1 ? Gicv3::G0S : Gicv3::G1NS;
          RegVal ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);

          if (group == Gicv3::G1NS && (ich_vmcr_el2 & ICH_VMCR_EL2_VCBPR)) {
              // reads return bpr0 + 1 saturated to 7, writes ignored
              return;
          }

          uint8_t min_VPBR = 7 - VIRTUAL_PREEMPTION_BITS;

          if (group != Gicv3::G0S) {
              min_VPBR++;
          }

          if (val < min_VPBR) {
              val = min_VPBR;
          }

          if (group == Gicv3::G0S) {
              ich_vmcr_el2 = insertBits(ich_vmcr_el2,
                      ICH_VMCR_EL2_VBPR0_SHIFT + 2, ICH_VMCR_EL2_VBPR0_SHIFT,
                      val);
          } else {
              ich_vmcr_el2 = insertBits(ich_vmcr_el2,
                      ICH_VMCR_EL2_VBPR1_SHIFT + 2, ICH_VMCR_EL2_VBPR1_SHIFT,
                      val);
          }

          isa->setMiscRegNoEffect(MISCREG_ICH_VMCR_EL2, ich_vmcr_el2);
          do_virtual_update = true;
          break;
      }

      case MISCREG_ICC_CTLR:
      case MISCREG_ICC_CTLR_EL1: { // Control Register
          if ((currEL() == EL1) && !inSecureState() &&
                  (hcr_imo || hcr_fmo)) {
              return setMiscReg(MISCREG_ICV_CTLR_EL1, val);
          }

          /*
           * RSS is RO.
           * A3V is RO.
           * SEIS is RO.
           * IDbits is RO.
           * PRIbits is RO.
           * If EL3 is implemented and GICD_CTLR.DS == 0, then PMHE is RO.
           * So, only CBPR[0] and EOIMODE[1] are RW.
           * If EL3 is implemented and GICD_CTLR.DS == 0, then CBPR is RO.
           */
          uint64_t mask;

          if (haveEL(EL3) and distributor->DS == 0) {
              mask = ICC_CTLR_EL1_EOIMODE;
          } else if (haveEL(EL3) and distributor->DS == 1) {
              mask = ICC_CTLR_EL1_PMHE | ICC_CTLR_EL1_CBPR |
                  ICC_CTLR_EL1_EOIMODE;
          } else {
              mask = ICC_CTLR_EL1_CBPR | ICC_CTLR_EL1_EOIMODE;
          }

          RegVal old_val =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1);
          old_val &= ~mask;
          val = old_val | (val & mask);
          break;
      }

      case MISCREG_ICV_CTLR_EL1: {
          RegVal ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
          ich_vmcr_el2 = insertBits(ich_vmcr_el2, ICH_VMCR_EL2_VCBPR_SHIFT,
                  val & ICC_CTLR_EL1_CBPR ? 1 : 0);
          ich_vmcr_el2 = insertBits(ich_vmcr_el2, ICH_VMCR_EL2_VEOIM_SHIFT,
                  val & ICC_CTLR_EL1_EOIMODE ? 1 : 0);
          isa->setMiscRegNoEffect(MISCREG_ICH_VMCR_EL2, ich_vmcr_el2);
          do_virtual_update = true;
          break;
      }

      case MISCREG_ICC_MCTLR:
      case MISCREG_ICC_CTLR_EL3: {
          RegVal icc_ctlr_el1_s =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S);
          RegVal icc_ctlr_el1_ns =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS);

          // ICC_CTLR_EL1(NS).EOImode is an alias of
          // ICC_CTLR_EL3.EOImode_EL1NS
          if (val & ICC_CTLR_EL3_EOIMODE_EL1NS) {
              icc_ctlr_el1_ns |= ICC_CTLR_EL1_EOIMODE;
          } else {
              icc_ctlr_el1_ns &= ~ICC_CTLR_EL1_EOIMODE;
          }

          // ICC_CTLR_EL1(NS).CBPR is an alias of ICC_CTLR_EL3.CBPR_EL1NS
          if (val & ICC_CTLR_EL3_CBPR_EL1NS) {
              icc_ctlr_el1_ns |= ICC_CTLR_EL1_CBPR;
          } else {
              icc_ctlr_el1_ns &= ~ICC_CTLR_EL1_CBPR;
          }

          // ICC_CTLR_EL1(S).EOImode is an alias of ICC_CTLR_EL3.EOImode_EL1S
          if (val & ICC_CTLR_EL3_EOIMODE_EL1S) {
              icc_ctlr_el1_s |= ICC_CTLR_EL1_EOIMODE;
          } else {
              icc_ctlr_el1_s &= ~ICC_CTLR_EL1_EOIMODE;
          }

          // ICC_CTLR_EL1(S).CBPR is an alias of ICC_CTLR_EL3.CBPR_EL1S
          if (val & ICC_CTLR_EL3_CBPR_EL1S) {
              icc_ctlr_el1_s |= ICC_CTLR_EL1_CBPR;
          } else {
              icc_ctlr_el1_s &= ~ICC_CTLR_EL1_CBPR;
          }

          isa->setMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S, icc_ctlr_el1_s);
          isa->setMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS, icc_ctlr_el1_ns);
          // Only ICC_CTLR_EL3_EOIMODE_EL3 is writable
          RegVal old_icc_ctlr_el3 =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL3);
          old_icc_ctlr_el3 &= ~(ICC_CTLR_EL3_EOIMODE_EL3 | ICC_CTLR_EL3_RM);
          val = old_icc_ctlr_el3 |
              (val & (ICC_CTLR_EL3_EOIMODE_EL3 | ICC_CTLR_EL3_RM));
          break;
      }

      case MISCREG_ICC_PMR:
      case MISCREG_ICC_PMR_EL1: { // Priority Mask Register
          if ((currEL() == EL1) && !inSecureState() &&
                  (hcr_imo || hcr_fmo)) {
              return isa->setMiscRegNoEffect(MISCREG_ICV_PMR_EL1, val);
          }

          val &= 0xff;
          SCR scr_el3 = isa->readMiscRegNoEffect(MISCREG_SCR_EL3);

          if (haveEL(EL3) && !inSecureState() && (scr_el3.fiq)) {
              /*
               * NS access and Group 0 is inaccessible to NS: return the
               * NS view of the current priority
               */
              RegVal old_icc_pmr_el1 =
                  isa->readMiscRegNoEffect(MISCREG_ICC_PMR_EL1);

              if (!(old_icc_pmr_el1 & 0x80)) {
                  /* Current PMR in the secure range, don't allow NS to
                   * change it */
                  return;
              }

              val = (val >> 1) | 0x80;
          }

          val &= ~0U << (8 - PRIORITY_BITS);
          break;
      }

      case MISCREG_ICC_IGRPEN0:
      case MISCREG_ICC_IGRPEN0_EL1: { // Interrupt Group 0 Enable Register
          if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
              return setMiscReg(MISCREG_ICV_IGRPEN0_EL1, val);
          }

          break;
      }

      case MISCREG_ICV_IGRPEN0_EL1: {
          bool enable = val & 0x1;
          RegVal ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
          ich_vmcr_el2 = insertBits(ich_vmcr_el2,
                  ICH_VMCR_EL2_VENG0_SHIFT, enable);
          isa->setMiscRegNoEffect(MISCREG_ICH_VMCR_EL2, ich_vmcr_el2);
          virtualUpdate();
          return;
      }

      case MISCREG_ICC_IGRPEN1:
      case MISCREG_ICC_IGRPEN1_EL1: { // Interrupt Group 1 Enable Register
          if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
              return setMiscReg(MISCREG_ICV_IGRPEN1_EL1, val);
          }

          break;
      }

      case MISCREG_ICV_IGRPEN1_EL1: {
          bool enable = val & 0x1;
          RegVal ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
          ich_vmcr_el2 = insertBits(ich_vmcr_el2,
                  ICH_VMCR_EL2_VENG1_SHIFT, enable);
          isa->setMiscRegNoEffect(MISCREG_ICH_VMCR_EL2, ich_vmcr_el2);
          virtualUpdate();
          return;
      }

      case MISCREG_ICC_MGRPEN1:
      case MISCREG_ICC_IGRPEN1_EL3: {
          // EnableGrp1S and EnableGrp1NS are aliased with
          // ICC_IGRPEN1_EL1_S.Enable and ICC_IGRPEN1_EL1_NS.Enable
          bool enable_grp_1s = val & ICC_IGRPEN1_EL3_ENABLEGRP1S;
          bool enable_grp_1ns = val & ICC_IGRPEN1_EL3_ENABLEGRP1NS;
          isa->setMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1_S, enable_grp_1s);
          isa->setMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1_NS, enable_grp_1ns);
          return;
      }

        // Software Generated Interrupt Group 0 Register
      case MISCREG_ICC_SGI0R:
      case MISCREG_ICC_SGI0R_EL1:

        // Software Generated Interrupt Group 1 Register
      case MISCREG_ICC_SGI1R:
      case MISCREG_ICC_SGI1R_EL1:

        // Alias Software Generated Interrupt Group 1 Register
      case MISCREG_ICC_ASGI1R:
      case MISCREG_ICC_ASGI1R_EL1: {
          bool ns = !inSecureState();
          Gicv3::GroupId group;

          if (misc_reg == MISCREG_ICC_SGI1R_EL1) {
              group = ns ? Gicv3::G1NS : Gicv3::G1S;
          } else if (misc_reg == MISCREG_ICC_ASGI1R_EL1) {
              group = ns ? Gicv3::G1S : Gicv3::G1NS;
          } else {
              group = Gicv3::G0S;
          }

          if (distributor->DS && group == Gicv3::G1S) {
              group = Gicv3::G0S;
          }

          uint8_t aff3 = bits(val, 55, 48);
          uint8_t aff2 = bits(val, 39, 32);
          uint8_t aff1 = bits(val, 23, 16);;
          uint16_t target_list = bits(val, 15, 0);
          uint32_t int_id = bits(val, 27, 24);
          bool irm = bits(val, 40, 40);
          uint8_t rs = bits(val, 47, 44);

          for (int i = 0; i < gic->getSystem()->numContexts(); i++) {
              Gicv3Redistributor * redistributor_i =
                  gic->getRedistributor(i);
              uint32_t affinity_i = redistributor_i->getAffinity();

              if (irm) {
                  // Interrupts routed to all PEs in the system,
                  // excluding "self"
                  if (affinity_i == redistributor->getAffinity()) {
                      continue;
                  }
              } else {
                  // Interrupts routed to the PEs specified by
                  // Aff3.Aff2.Aff1.<target list>
                  if ((affinity_i >> 8) !=
                          ((aff3 << 16) | (aff2 << 8) | (aff1 << 0))) {
                      continue;
                  }

                  uint8_t aff0_i = bits(affinity_i, 7, 0);

                  if (!(aff0_i >= rs * 16 && aff0_i < (rs + 1) * 16 &&
                              ((0x1 << (aff0_i - rs * 16)) & target_list))) {
                      continue;
                  }
              }

              redistributor_i->sendSGI(int_id, group, ns);
          }

          break;
      }

      case MISCREG_ICC_SRE:
      case MISCREG_ICC_SRE_EL1: { // System Register Enable Register EL1
          if (!(val & ICC_SRE_EL1_SRE)) {
              warn("Gicv3CPUInterface::setMiscReg(): "
                      "ICC_SRE_EL*.SRE is RAO/WI, legacy not supported!\n");
          }

          bool dfb = val & ICC_SRE_EL1_DFB;
          bool dib = val & ICC_SRE_EL1_DIB;

          if (haveEL(EL3) && !distributor->DS) {
              // DIB is RO alias of ICC_SRE_EL3.DIB
              // DFB is RO alias of ICC_SRE_EL3.DFB
          } else if (haveEL(EL3) && distributor->DS) {
              // DIB is RW alias of ICC_SRE_EL3.DIB
              // DFB is RW alias of ICC_SRE_EL3.DFB
              RegVal icc_sre_el3 =
                  isa->readMiscRegNoEffect(MISCREG_ICC_SRE_EL3);
              icc_sre_el3 = insertBits(icc_sre_el3, ICC_SRE_EL3_DFB, dfb);
              icc_sre_el3 = insertBits(icc_sre_el3, ICC_SRE_EL3_DIB, dib);
              isa->setMiscRegNoEffect(MISCREG_ICC_SRE_EL3, icc_sre_el3);
          } else if ((!haveEL(EL3) || distributor->DS) and haveEL(EL2)) {
              // DIB is RO alias of ICC_SRE_EL2.DIB
              // DFB is RO alias of ICC_SRE_EL2.DFB
          } else {
              isa->setMiscRegNoEffect(misc_reg, val);
          }

          return;
      }

      case MISCREG_ICC_HSRE:
      case MISCREG_ICC_SRE_EL2: // System Register Enable Register EL2
      case MISCREG_ICC_MSRE:
      case MISCREG_ICC_SRE_EL3: // System Register Enable Register EL3
        if (!(val & (1 << 0))) {
            warn("Gicv3CPUInterface::setMiscReg(): "
                    "ICC_SRE_EL*.SRE is RAO/WI, legacy not supported!\n");
        }

        // All bits are RAO/WI
        break;

      case MISCREG_ICH_HCR:
      case MISCREG_ICH_HCR_EL2:
        val &= ICH_HCR_EL2_EN | ICH_HCR_EL2_UIE | ICH_HCR_EL2_LRENPIE |
            ICH_HCR_EL2_NPIE | ICH_HCR_EL2_VGRP0EIE |
            ICH_HCR_EL2_VGRP0DIE | ICH_HCR_EL2_VGRP1EIE |
            ICH_HCR_EL2_VGRP1DIE | ICH_HCR_EL2_TC | ICH_HCR_EL2_TALL0 |
            ICH_HCR_EL2_TALL1 | ICH_HCR_EL2_TDIR |
            ICH_HCR_EL2_EOICOUNT_MASK;
        do_virtual_update = true;
        break;

      case MISCREG_ICH_LRC0 ... MISCREG_ICH_LRC15:
        // AArch32 (maps to AArch64 MISCREG_ICH_LR<n>_EL2 high half part)
        {
            // Enforce RES0 bits in priority field, 5 of 8 bits used
            val = insertBits(val, ICH_LRC_PRIORITY_SHIFT + 2,
                    ICH_LRC_PRIORITY_SHIFT, 0);
            RegVal old_val = isa->readMiscRegNoEffect(misc_reg);
            val = (old_val & 0xffffffff) | (val << 32);
            do_virtual_update = true;
            break;
        }

      case MISCREG_ICH_LR0 ... MISCREG_ICH_LR15: {
          // AArch32 (maps to AArch64 MISCREG_ICH_LR<n>_EL2 low half part)
          RegVal old_val = isa->readMiscRegNoEffect(misc_reg);
          val = (old_val & 0xffffffff00000000) | (val & 0xffffffff);
          do_virtual_update = true;
          break;
      }

      case MISCREG_ICH_LR0_EL2 ... MISCREG_ICH_LR15_EL2: { // AArch64
          // Enforce RES0 bits in priority field, 5 of 8 bits used
          val = insertBits(val, ICH_LR_EL2_PRIORITY_SHIFT + 2,
                  ICH_LR_EL2_PRIORITY_SHIFT, 0);
          do_virtual_update = true;
          break;
      }

      case MISCREG_ICH_VMCR:
      case MISCREG_ICH_VMCR_EL2: {
          val &= ICH_VMCR_EL2_VENG0 | ICH_VMCR_EL2_VENG1 |
              ICH_VMCR_EL2_VCBPR | ICH_VMCR_EL2_VEOIM |
              ICH_VMCR_EL2_VBPR1_MASK | ICH_VMCR_EL2_VBPR0_MASK |
              ICH_VMCR_EL2_VPMR_MASK;
          val |= ICH_VMCR_EL2_VFIQEN; // RES1
          // Check VBPRs against minimun allowed value
          uint8_t vbpr0 = bits(val, 23, 21);
          uint8_t vbpr1 = bits(val, 20, 18);
          uint8_t min_vpr0 = 7 - VIRTUAL_PREEMPTION_BITS;
          uint8_t min_vpr1 = min_vpr0 + 1;
          vbpr0 = vbpr0 < min_vpr0 ? min_vpr0 : vbpr0;
          vbpr1 = vbpr1 < min_vpr1 ? min_vpr1 : vbpr1;
          val = insertBits(val, ICH_VMCR_EL2_VBPR0_SHIFT + 2,
                  ICH_VMCR_EL2_VBPR0_SHIFT, vbpr0);
          val = insertBits(val, ICH_VMCR_EL2_VBPR1_SHIFT + 2,
                  ICH_VMCR_EL2_VBPR1_SHIFT, vbpr1);
          break;
      }

      case MISCREG_ICH_AP0R0 ... MISCREG_ICH_AP0R3:
      case MISCREG_ICH_AP0R0_EL2 ... MISCREG_ICH_AP0R3_EL2:
      case MISCREG_ICH_AP1R0 ... MISCREG_ICH_AP1R3:
      case MISCREG_ICH_AP1R0_EL2 ... MISCREG_ICH_AP1R3_EL2:
        break;

      default:
        panic("Gicv3CPUInterface::setMiscReg(): "
                "unknown register %d (%s)",
                misc_reg, miscRegName[misc_reg]);
    }

    isa->setMiscRegNoEffect(misc_reg, val);

    if (do_virtual_update) {
        virtualUpdate();
    }
}

int
Gicv3CPUInterface::virtualFindActive(uint32_t int_id)
{
    for (uint32_t lr_idx = 0; lr_idx < VIRTUAL_NUM_LIST_REGS; lr_idx++) {
        RegVal lr =
            isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);
        uint32_t lr_intid = bits(lr, 31, 0);

        if ((lr & ICH_LR_EL2_STATE_ACTIVE_BIT) && lr_intid == int_id) {
            return lr_idx;
        }
    }

    return -1;
}

uint32_t
Gicv3CPUInterface::getHPPIR0()
{
    if (hppi.prio == 0xff) {
        return Gicv3::INTID_SPURIOUS;
    }

    bool irq_is_secure = !distributor->DS && hppi.group != Gicv3::G1NS;

    if ((hppi.group != Gicv3::G0S) && isEL3OrMon()) {
        /* Indicate to EL3 that there's a Group 1 interrupt for the
         * other state pending.
         */
        return irq_is_secure ? Gicv3::INTID_SECURE : Gicv3::INTID_NONSECURE;
    }

    if ((hppi.group != Gicv3::G0S)) { // && !isEL3OrMon())
        return Gicv3::INTID_SPURIOUS;
    }

    if (irq_is_secure && !inSecureState()) {
        // Secure interrupts not visible in Non-secure
        return Gicv3::INTID_SPURIOUS;
    }

    return hppi.intid;
}

uint32_t
Gicv3CPUInterface::getHPPIR1()
{
    if (hppi.prio == 0xff) {
        return Gicv3::INTID_SPURIOUS;
    }

    //if ((currEL() == EL3) && ICC_CTLR_EL3_RM)
    if ((currEL() == EL3) &&
            isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL3) & ICC_CTLR_EL3_RM) {
        if (hppi.group == Gicv3::G0S) {
            return Gicv3::INTID_SECURE;
        } else if (hppi.group == Gicv3::G1NS) {
            return Gicv3::INTID_NONSECURE;
        }
    }

    if (hppi.group == Gicv3::G0S) {
        return Gicv3::INTID_SPURIOUS;
    }

    bool irq_is_secure = (distributor->DS == 0) && (hppi.group != Gicv3::G1NS);

    if (irq_is_secure) {
        if (!inSecureState()) {
            // Secure interrupts not visible in Non-secure
            return Gicv3::INTID_SPURIOUS;
        }
    } else if (!isEL3OrMon() && inSecureState()) {
        // Group 1 non-secure interrupts not visible in Secure EL1
        return Gicv3::INTID_SPURIOUS;
    }

    return hppi.intid;
}

void
Gicv3CPUInterface::dropPriority(Gicv3::GroupId group)
{
    int apr_misc_reg;
    RegVal apr;
    apr_misc_reg = group == Gicv3::G0S ?
                   MISCREG_ICC_AP0R0_EL1 : MISCREG_ICC_AP1R0_EL1;
    apr = isa->readMiscRegNoEffect(apr_misc_reg);

    if (apr) {
        /* Clear the lowest set bit */
        apr &= apr - 1;
        isa->setMiscRegNoEffect(apr_misc_reg, apr);
    }

    update();
}

uint8_t
Gicv3CPUInterface::virtualDropPriority()
{
    /* Drop the priority of the currently active virtual interrupt
     * (favouring group 0 if there is a set active bit at
     * the same priority for both group 0 and group 1).
     * Return the priority value for the bit we just cleared,
     * or 0xff if no bits were set in the AP registers at all.
     * Note that though the ich_apr[] are uint64_t only the low
     * 32 bits are actually relevant.
     */
    int apr_max = 1 << (VIRTUAL_PREEMPTION_BITS - 5);

    for (int i = 0; i < apr_max; i++) {
        RegVal vapr0 = isa->readMiscRegNoEffect(MISCREG_ICH_AP0R0_EL2 + i);
        RegVal vapr1 = isa->readMiscRegNoEffect(MISCREG_ICH_AP1R0_EL2 + i);

        if (!vapr0 && !vapr1) {
            continue;
        }

        int vapr0_count = ctz32(vapr0);
        int vapr1_count = ctz32(vapr1);

        if (vapr0_count <= vapr1_count) {
            /* Clear the lowest set bit */
            vapr0 &= vapr0 - 1;
            isa->setMiscRegNoEffect(MISCREG_ICH_AP0R0_EL2 + i, vapr0);
            return (vapr0_count + i * 32) << (GIC_MIN_VBPR + 1);
        } else {
            /* Clear the lowest set bit */
            vapr1 &= vapr1 - 1;
            isa->setMiscRegNoEffect(MISCREG_ICH_AP1R0_EL2 + i, vapr1);
            return (vapr1_count + i * 32) << (GIC_MIN_VBPR + 1);
        }
    }

    return 0xff;
}

void
Gicv3CPUInterface::activateIRQ(uint32_t int_id, Gicv3::GroupId group)
{
    // Update active priority registers.
    uint32_t prio = hppi.prio & 0xf8;
    int apr_bit = prio >> (8 - PRIORITY_BITS);
    int reg_bit = apr_bit % 32;
    int apr_idx = group == Gicv3::G0S ?
                 MISCREG_ICC_AP0R0_EL1 : MISCREG_ICC_AP1R0_EL1;
    RegVal apr = isa->readMiscRegNoEffect(apr_idx);
    apr |= (1 << reg_bit);
    isa->setMiscRegNoEffect(apr_idx, apr);

    // Move interrupt state from pending to active.
    if (int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX) {
        // SGI or PPI, redistributor
        redistributor->activateIRQ(int_id);
        redistributor->updateAndInformCPUInterface();
    } else if (int_id < Gicv3::INTID_SECURE) {
        // SPI, distributor
        distributor->activateIRQ(int_id);
        distributor->updateAndInformCPUInterfaces();
    }
}

void
Gicv3CPUInterface::virtualActivateIRQ(uint32_t lr_idx)
{
    // Update active priority registers.
    RegVal lr = isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 +
            lr_idx);
    Gicv3::GroupId group = lr & ICH_LR_EL2_GROUP ? Gicv3::G1NS : Gicv3::G0S;
    uint8_t prio = bits(lr, 55, 48) & 0xf8;
    int apr_bit = prio >> (8 - VIRTUAL_PREEMPTION_BITS);
    int reg_no = apr_bit / 32;
    int reg_bit = apr_bit % 32;
    int apr_idx = group == Gicv3::G0S ?
        MISCREG_ICH_AP0R0_EL2 + reg_no : MISCREG_ICH_AP1R0_EL2 + reg_no;
    RegVal apr = isa->readMiscRegNoEffect(apr_idx);
    apr |= (1 << reg_bit);
    isa->setMiscRegNoEffect(apr_idx, apr);
    // Move interrupt state from pending to active.
    lr &= ~ICH_LR_EL2_STATE_PENDING_BIT;
    lr |= ICH_LR_EL2_STATE_ACTIVE_BIT;
    isa->setMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx, lr);
}

void
Gicv3CPUInterface::deactivateIRQ(uint32_t int_id, Gicv3::GroupId group)
{
    if (int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX) {
        // SGI or PPI, redistributor
        redistributor->deactivateIRQ(int_id);
        redistributor->updateAndInformCPUInterface();
    } else if (int_id < Gicv3::INTID_SECURE) {
        // SPI, distributor
        distributor->deactivateIRQ(int_id);
        distributor->updateAndInformCPUInterfaces();
    } else {
        return;
    }
}

void
Gicv3CPUInterface::virtualDeactivateIRQ(int lr_idx)
{
    RegVal lr = isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 +
            lr_idx);

    if (lr & ICH_LR_EL2_HW) {
        // Deactivate the associated physical interrupt
        int pintid = bits(lr, 41, 32);

        if (pintid < Gicv3::INTID_SECURE) {
            Gicv3::GroupId group =
                pintid >= 32 ? distributor->getIntGroup(pintid) :
                redistributor->getIntGroup(pintid);
            deactivateIRQ(pintid, group);
        }
    }

    //  Remove the active bit
    lr &= ~ICH_LR_EL2_STATE_ACTIVE_BIT;
    isa->setMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx, lr);
}

/*
 * Return a mask word which clears the subpriority bits from
 * a priority value for an interrupt in the specified group.
 * This depends on the BPR value. For CBPR0 (S or NS):
 *  a BPR of 0 means the group priority bits are [7:1];
 *  a BPR of 1 means they are [7:2], and so on down to
 *  ...
 *  a BPR of 7 meaning no group priority bits at all.
 * For CBPR1 NS:
 *  a BPR of 0 is impossible (the minimum value is 1)
 *  a BPR of 1 means the group priority bits are [7:1];
 *  a BPR of 2 means they are [7:2], and so on down to
 *  ...
 *  a BPR of 7 meaning the group priority is [7].
 *
 * Which BPR to use depends on the group of the interrupt and
 * the current ICC_CTLR.CBPR settings.
 *
 * This corresponds to the GroupBits() pseudocode from 4.8.2.
 */
uint32_t
Gicv3CPUInterface::groupPriorityMask(Gicv3::GroupId group)
{
    if ((group == Gicv3::G1S &&
            isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S)
            & ICC_CTLR_EL1_CBPR) ||
            (group == Gicv3::G1NS &&
             isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS)
             & ICC_CTLR_EL1_CBPR)) {
        group = Gicv3::G0S;
    }

    int bpr;

    if (group == Gicv3::G0S) {
        bpr = isa->readMiscRegNoEffect(MISCREG_ICC_BPR0_EL1) & 0x7;
    } else {
        bpr = isa->readMiscRegNoEffect(MISCREG_ICC_BPR1_EL1) & 0x7;
    }

    if (group == Gicv3::G1NS) {
        assert(bpr > 0);
        bpr--;
    }

    return ~0U << (bpr + 1);
}

uint32_t
Gicv3CPUInterface::virtualGroupPriorityMask(Gicv3::GroupId group)
{
    RegVal ich_vmcr_el2 =
        isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);

    if (group == Gicv3::G1NS && (ich_vmcr_el2 & ICH_VMCR_EL2_VCBPR)) {
        group = Gicv3::G0S;
    }

    int bpr;

    if (group == Gicv3::G0S) {
        bpr = bits(ich_vmcr_el2, 23, 21);
    } else {
        bpr = bits(ich_vmcr_el2, 20, 18);
    }

    if (group == Gicv3::G1NS) {
        assert(bpr > 0);
        bpr--;
    }

    return ~0U << (bpr + 1);
}

bool
Gicv3CPUInterface::isEOISplitMode()
{
    if (isEL3OrMon()) {
        return isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL3) &
               ICC_CTLR_EL3_EOIMODE_EL3;
    } else {
        return isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1) &
               ICC_CTLR_EL1_EOIMODE;
    }
}

bool
Gicv3CPUInterface::virtualIsEOISplitMode()
{
    RegVal ich_vmcr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
    return ich_vmcr_el2 & ICH_VMCR_EL2_VEOIM;
}

int
Gicv3CPUInterface::highestActiveGroup()
{
    int g0_ctz = ctz32(isa->readMiscRegNoEffect(MISCREG_ICC_AP0R0_EL1));
    int gq_ctz = ctz32(isa->readMiscRegNoEffect(MISCREG_ICC_AP1R0_EL1_S));
    int g1nz_ctz = ctz32(isa->readMiscRegNoEffect(MISCREG_ICC_AP1R0_EL1_NS));

    if (g1nz_ctz < g0_ctz && g1nz_ctz < gq_ctz) {
        return Gicv3::G1NS;
    }

    if (gq_ctz < g0_ctz) {
        return Gicv3::G1S;
    }

    if (g0_ctz < 32) {
        return Gicv3::G0S;
    }

    return -1;
}

void
Gicv3CPUInterface::update()
{
    bool signal_IRQ = false;
    bool signal_FIQ = false;

    if (hppi.group == Gicv3::G1S && !haveEL(EL3)) {
        /*
         * Secure enabled GIC sending a G1S IRQ to a secure disabled
         * CPU -> send G0 IRQ
         */
        hppi.group = Gicv3::G0S;
    }

    if (hppiCanPreempt()) {
        ArmISA::InterruptTypes int_type = intSignalType(hppi.group);
        DPRINTF(GIC, "Gicv3CPUInterface::update(): "
                "posting int as %d!\n", int_type);
        int_type == ArmISA::INT_IRQ ? signal_IRQ = true : signal_FIQ = true;
    }

    if (signal_IRQ) {
        gic->postInt(cpuId, ArmISA::INT_IRQ);
    } else {
        gic->deassertInt(cpuId, ArmISA::INT_IRQ);
    }

    if (signal_FIQ) {
        gic->postInt(cpuId, ArmISA::INT_FIQ);
    } else {
        gic->deassertInt(cpuId, ArmISA::INT_FIQ);
    }
}

void
Gicv3CPUInterface::virtualUpdate()
{
    bool signal_IRQ = false;
    bool signal_FIQ = false;
    int lr_idx = getHPPVILR();

    if (lr_idx >= 0) {
        RegVal ich_lr_el2 =
            isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

        if (hppviCanPreempt(lr_idx)) {
            if (ich_lr_el2 & ICH_LR_EL2_GROUP) {
                signal_IRQ = true;
            } else {
                signal_FIQ = true;
            }
        }
    }

    RegVal ich_hcr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_HCR_EL2);

    if (ich_hcr_el2 & ICH_HCR_EL2_EN) {
        if (maintenanceInterruptStatus()) {
            redistributor->sendPPInt(25);
        }
    }

    if (signal_IRQ) {
        DPRINTF(GIC, "Gicv3CPUInterface::virtualUpdate(): "
                "posting int as %d!\n", ArmISA::INT_VIRT_IRQ);
        gic->postInt(cpuId, ArmISA::INT_VIRT_IRQ);
    } else {
        gic->deassertInt(cpuId, ArmISA::INT_VIRT_IRQ);
    }

    if (signal_FIQ) {
        DPRINTF(GIC, "Gicv3CPUInterface::virtualUpdate(): "
                "posting int as %d!\n", ArmISA::INT_VIRT_FIQ);
        gic->postInt(cpuId, ArmISA::INT_VIRT_FIQ);
    } else {
        gic->deassertInt(cpuId, ArmISA::INT_VIRT_FIQ);
    }
}

// Returns the intex of the LR with the HPPI
int
Gicv3CPUInterface::getHPPVILR()
{
    int idx = -1;
    RegVal ich_vmcr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);

    if (!(ich_vmcr_el2 & (ICH_VMCR_EL2_VENG0 | ICH_VMCR_EL2_VENG1))) {
        // VG0 and VG1 disabled...
        return idx;
    }

    uint8_t highest_prio = 0xff;

    for (int i = 0; i < 16; i++) {
        RegVal ich_lri_el2 =
            isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + i);
        uint8_t state = bits(ich_lri_el2, 63, 62);

        if (state != Gicv3::INT_PENDING) {
            continue;
        }

        if (ich_lri_el2 & ICH_LR_EL2_GROUP) {
            // VG1
            if (!(ich_vmcr_el2 & ICH_VMCR_EL2_VENG1)) {
                continue;
            }
        } else {
            // VG0
            if (!(ich_vmcr_el2 & ICH_VMCR_EL2_VENG0)) {
                continue;
            }
        }

        uint8_t prio = bits(ich_lri_el2, 55, 48);

        if (prio < highest_prio) {
            highest_prio = prio;
            idx = i;
        }
    }

    return idx;
}

bool
Gicv3CPUInterface::hppviCanPreempt(int lr_idx)
{
    RegVal lr = isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

    if (!(isa->readMiscRegNoEffect(MISCREG_ICH_HCR_EL2) & ICH_HCR_EL2_EN)) {
        // virtual interface is disabled
        return false;
    }

    uint8_t prio = bits(lr, 55, 48);
    uint8_t vpmr =
        bits(isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2), 31, 24);

    if (prio >= vpmr) {
        // prioriry masked
        return false;
    }

    uint8_t rprio = virtualHighestActivePriority();

    if (rprio == 0xff) {
        return true;
    }

    Gicv3::GroupId group = lr & ICH_LR_EL2_GROUP ? Gicv3::G1NS : Gicv3::G0S;
    uint32_t prio_mask = virtualGroupPriorityMask(group);

    if ((prio & prio_mask) < (rprio & prio_mask)) {
        return true;
    }

    return false;
}

uint8_t
Gicv3CPUInterface::virtualHighestActivePriority()
{
    uint8_t num_aprs = 1 << (VIRTUAL_PRIORITY_BITS - 5);

    for (int i = 0; i < num_aprs; i++) {
        RegVal vapr =
            isa->readMiscRegNoEffect(MISCREG_ICH_AP0R0_EL2 + i) |
            isa->readMiscRegNoEffect(MISCREG_ICH_AP1R0_EL2 + i);

        if (!vapr) {
            continue;
        }

        return (i * 32 + ctz32(vapr)) << (GIC_MIN_VBPR + 1);
    }

    // no active interrups, return idle priority
    return 0xff;
}

void
Gicv3CPUInterface::virtualIncrementEOICount()
{
    // Increment the EOICOUNT field in ICH_HCR_EL2
    RegVal ich_hcr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_HCR_EL2);
    uint32_t EOI_cout = bits(ich_hcr_el2, 31, 27);
    EOI_cout++;
    ich_hcr_el2 = insertBits(ich_hcr_el2, 31, 27, EOI_cout);
    isa->setMiscRegNoEffect(MISCREG_ICH_HCR_EL2, ich_hcr_el2);
}

/*
 * Should we signal the interrupt as IRQ or FIQ?
 * see spec section 4.6.2
 */
ArmISA::InterruptTypes
Gicv3CPUInterface::intSignalType(Gicv3::GroupId group)
{
    bool is_fiq = false;

    switch (group) {
      case Gicv3::G0S:
        is_fiq = true;
        break;

      case Gicv3::G1S:
        is_fiq = (distributor->DS == 0) &&
            (!inSecureState() || ((currEL() == EL3) && isAA64()));
        break;

      case Gicv3::G1NS:
        is_fiq = (distributor->DS == 0) && inSecureState();
        break;

      default:
        panic("Gicv3CPUInterface::intSignalType(): invalid group!");
    }

    if (is_fiq) {
        return ArmISA::INT_FIQ;
    } else {
        return ArmISA::INT_IRQ;
    }
}

bool
Gicv3CPUInterface::hppiCanPreempt()
{
    if (hppi.prio == 0xff) {
        // there is no pending interrupt
        return false;
    }

    if (!groupEnabled(hppi.group)) {
        // group disabled at CPU interface
        return false;
    }

    if (hppi.prio >= isa->readMiscRegNoEffect(MISCREG_ICC_PMR_EL1)) {
        // priority masked
        return false;
    }

    uint8_t rprio = highestActivePriority();

    if (rprio == 0xff) {
        return true;
    }

    uint32_t prio_mask = groupPriorityMask(hppi.group);

    if ((hppi.prio & prio_mask) < (rprio & prio_mask)) {
        return true;
    }

    return false;
}

uint8_t
Gicv3CPUInterface::highestActivePriority()
{
    uint32_t apr = isa->readMiscRegNoEffect(MISCREG_ICC_AP0R0_EL1) |
                   isa->readMiscRegNoEffect(MISCREG_ICC_AP1R0_EL1_NS) |
                   isa->readMiscRegNoEffect(MISCREG_ICC_AP1R0_EL1_S);

    if (apr) {
        return ctz32(apr) << (GIC_MIN_BPR + 1);
    }

    // no active interrups, return idle priority
    return 0xff;
}

bool
Gicv3CPUInterface::groupEnabled(Gicv3::GroupId group)
{
    switch (group) {
      case Gicv3::G0S:
        return isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN0_EL1) &
            ICC_IGRPEN0_EL1_ENABLE;

      case Gicv3::G1S:
        //if (distributor->DS)
        //{
        //    return isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1_NS) &
        //           ICC_IGRPEN1_EL1_ENABLE;
        //}
        //else
        //{
        return isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1_S) &
            ICC_IGRPEN1_EL1_ENABLE;

        //}

      case Gicv3::G1NS:
        return isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1_NS) &
            ICC_IGRPEN1_EL1_ENABLE;

      default:
        panic("Gicv3CPUInterface::groupEnable(): invalid group!\n");
    }
}

bool
Gicv3CPUInterface::inSecureState()
{
    if (!gic->getSystem()->haveSecurity()) {
        return false;
    }

    CPSR cpsr = isa->readMiscRegNoEffect(MISCREG_CPSR);
    SCR scr = isa->readMiscRegNoEffect(MISCREG_SCR);
    return ArmISA::inSecureState(scr, cpsr);
}

int
Gicv3CPUInterface::currEL()
{
    CPSR cpsr = isa->readMiscRegNoEffect(MISCREG_CPSR);
    bool is_64 = opModeIs64((OperatingMode)(uint8_t) cpsr.mode);

    if (is_64) {
        return (ExceptionLevel)(uint8_t) cpsr.el;
    } else {
        switch (cpsr.mode) {
          case MODE_USER:
            return 0;

          case MODE_HYP:
            return 2;

          case MODE_MON:
            return 3;

          default:
            return 1;
        }
    }
}

bool
Gicv3CPUInterface::haveEL(ExceptionLevel el)
{
    switch (el) {
      case EL0:
      case EL1:
        return true;

      case EL2:
        return gic->getSystem()->haveVirtualization();

      case EL3:
        return gic->getSystem()->haveSecurity();

      default:
        warn("Unimplemented Exception Level\n");
        return false;
    }
}

bool
Gicv3CPUInterface::isSecureBelowEL3()
{
    SCR scr = isa->readMiscRegNoEffect(MISCREG_SCR_EL3);
    return haveEL(EL3) && scr.ns == 0;
}

bool
Gicv3CPUInterface::isAA64()
{
    CPSR cpsr = isa->readMiscRegNoEffect(MISCREG_CPSR);
    return opModeIs64((OperatingMode)(uint8_t) cpsr.mode);
}

bool
Gicv3CPUInterface::isEL3OrMon()
{
    if (haveEL(EL3)) {
        CPSR cpsr = isa->readMiscRegNoEffect(MISCREG_CPSR);
        bool is_64 = opModeIs64((OperatingMode)(uint8_t) cpsr.mode);

        if (is_64 && (cpsr.el == EL3)) {
            return true;
        } else if (!is_64 && (cpsr.mode == MODE_MON)) {
            return true;
        }
    }

    return false;
}

uint32_t
Gicv3CPUInterface::eoiMaintenanceInterruptStatus(uint32_t * misr)
{
    /* Return a set of bits indicating the EOI maintenance interrupt status
     * for each list register. The EOI maintenance interrupt status is
     * 1 if LR.State == 0 && LR.HW == 0 && LR.EOI == 1
     * (see the GICv3 spec for the ICH_EISR_EL2 register).
     * If misr is not NULL then we should also collect the information
     * about the MISR.EOI, MISR.NP and MISR.U bits.
     */
    uint32_t value = 0;
    int valid_count = 0;
    bool seen_pending = false;

    for (int lr_idx = 0; lr_idx < VIRTUAL_NUM_LIST_REGS; lr_idx++) {
        RegVal lr = isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

        if ((lr & (ICH_LR_EL2_STATE_MASK | ICH_LR_EL2_HW | ICH_LR_EL2_EOI)) ==
                ICH_LR_EL2_EOI) {
            value |= (1 << lr_idx);
        }

        if ((lr & ICH_LR_EL2_STATE_MASK)) {
            valid_count++;
        }

        if (bits(lr, ICH_LR_EL2_STATE_SHIFT + ICH_LR_EL2_STATE_LENGTH,
                 ICH_LR_EL2_STATE_SHIFT) == ICH_LR_EL2_STATE_PENDING) {
            seen_pending = true;
        }
    }

    if (misr) {
        RegVal ich_hcr_el2 =
            isa->readMiscRegNoEffect(MISCREG_ICH_HCR_EL2);

        if (valid_count < 2 && (ich_hcr_el2 & ICH_HCR_EL2_UIE)) {
            *misr |= ICH_MISR_EL2_U;
        }

        if (!seen_pending && (ich_hcr_el2 & ICH_HCR_EL2_NPIE)) {
            *misr |= ICH_MISR_EL2_NP;
        }

        if (value) {
            *misr |= ICH_MISR_EL2_EOI;
        }
    }

    return value;
}

uint32_t
Gicv3CPUInterface::maintenanceInterruptStatus()
{
    /* Return a set of bits indicating the maintenance interrupt status
     * (as seen in the ICH_MISR_EL2 register).
     */
    uint32_t value = 0;
    /* Scan list registers and fill in the U, NP and EOI bits */
    eoiMaintenanceInterruptStatus(&value);
    RegVal ich_hcr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_HCR_EL2);
    RegVal ich_vmcr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);

    if (ich_hcr_el2 & (ICH_HCR_EL2_LRENPIE | ICH_HCR_EL2_EOICOUNT_MASK)) {
        value |= ICH_MISR_EL2_LRENP;
    }

    if ((ich_hcr_el2 & ICH_HCR_EL2_VGRP0EIE) &&
            (ich_vmcr_el2 & ICH_VMCR_EL2_VENG0)) {
        value |= ICH_MISR_EL2_VGRP0E;
    }

    if ((ich_hcr_el2 & ICH_HCR_EL2_VGRP0DIE) &&
            !(ich_vmcr_el2 & ICH_VMCR_EL2_VENG1)) {
        value |= ICH_MISR_EL2_VGRP0D;
    }

    if ((ich_hcr_el2 & ICH_HCR_EL2_VGRP1EIE) &&
            (ich_vmcr_el2 & ICH_VMCR_EL2_VENG1)) {
        value |= ICH_MISR_EL2_VGRP1E;
    }

    if ((ich_hcr_el2 & ICH_HCR_EL2_VGRP1DIE) &&
            !(ich_vmcr_el2 & ICH_VMCR_EL2_VENG1)) {
        value |= ICH_MISR_EL2_VGRP1D;
    }

    return value;
}

void
Gicv3CPUInterface::serialize(CheckpointOut & cp) const
{
    SERIALIZE_SCALAR(hppi.intid);
    SERIALIZE_SCALAR(hppi.prio);
    SERIALIZE_ENUM(hppi.group);
}

void
Gicv3CPUInterface::unserialize(CheckpointIn & cp)
{
    UNSERIALIZE_SCALAR(hppi.intid);
    UNSERIALIZE_SCALAR(hppi.prio);
    UNSERIALIZE_ENUM(hppi.group);
}
