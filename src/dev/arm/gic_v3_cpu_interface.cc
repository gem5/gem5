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
Gicv3CPUInterface::getHCREL2FMO() const
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
Gicv3CPUInterface::getHCREL2IMO() const
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
      // Active Priorities Group 1 Registers
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

      // Active Priorities Group 0 Registers
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

      // Interrupt Group 0 Enable register EL1
      case MISCREG_ICC_IGRPEN0:
      case MISCREG_ICC_IGRPEN0_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
              return isa->readMiscRegNoEffect(MISCREG_ICV_IGRPEN0_EL1);
          }

          break;
      }

      // Interrupt Group 1 Enable register EL1
      case MISCREG_ICC_IGRPEN1:
      case MISCREG_ICC_IGRPEN1_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
              return isa->readMiscRegNoEffect(MISCREG_ICV_IGRPEN1_EL1);
          }

          break;
      }

      // Interrupt Group 1 Enable register EL3
      case MISCREG_ICC_MGRPEN1:
      case MISCREG_ICC_IGRPEN1_EL3:
          break;

      // Running Priority Register
      case MISCREG_ICC_RPR:
      case MISCREG_ICC_RPR_EL1: {
          if ((currEL() == EL1) && !inSecureState() &&
              (hcr_imo || hcr_fmo)) {
              return readMiscReg(MISCREG_ICV_RPR_EL1);
          }

          uint8_t rprio = highestActivePriority();

          if (haveEL(EL3) && !inSecureState() &&
              (isa->readMiscRegNoEffect(MISCREG_SCR_EL3) & (1U << 2))) {
              // Spec section 4.8.1
              // For Non-secure access to ICC_RPR_EL1 when SCR_EL3.FIQ == 1
              if ((rprio & 0x80) == 0) {
                  // If the current priority mask value is in the range of
                  // 0x00-0x7F a read access returns the value 0x0
                  rprio = 0;
              } else if (rprio != 0xff) {
                  // If the current priority mask value is in the range of
                  // 0x80-0xFF a read access returns the Non-secure read of
                  // the current value
                  rprio = (rprio << 1) & 0xff;
              }
          }

          value = rprio;
          break;
      }

      // Virtual Running Priority Register
      case MISCREG_ICV_RPR_EL1: {
          value = virtualHighestActivePriority();
          break;
      }

      // Highest Priority Pending Interrupt Register 0
      case MISCREG_ICC_HPPIR0:
      case MISCREG_ICC_HPPIR0_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
              return readMiscReg(MISCREG_ICV_HPPIR0_EL1);
          }

          value = getHPPIR0();
          break;
      }

      // Virtual Highest Priority Pending Interrupt Register 0
      case MISCREG_ICV_HPPIR0_EL1: {
          value = Gicv3::INTID_SPURIOUS;
          int lr_idx = getHPPVILR();

          if (lr_idx >= 0) {
              ICH_LR_EL2 ich_lr_el2 =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);
              Gicv3::GroupId group =
                  ich_lr_el2.Group ? Gicv3::G1NS : Gicv3::G0S;

              if (group == Gicv3::G0S) {
                  value = ich_lr_el2.vINTID;
              }
          }

          break;
      }

      // Highest Priority Pending Interrupt Register 1
      case MISCREG_ICC_HPPIR1:
      case MISCREG_ICC_HPPIR1_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
              return readMiscReg(MISCREG_ICV_HPPIR1_EL1);
          }

          value = getHPPIR1();
          break;
      }

      // Virtual Highest Priority Pending Interrupt Register 1
      case MISCREG_ICV_HPPIR1_EL1: {
          value = Gicv3::INTID_SPURIOUS;
          int lr_idx = getHPPVILR();

          if (lr_idx >= 0) {
              ICH_LR_EL2 ich_lr_el2 =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);
              Gicv3::GroupId group =
                  ich_lr_el2.Group ? Gicv3::G1NS : Gicv3::G0S;

              if (group == Gicv3::G1NS) {
                  value = ich_lr_el2.vINTID;
              }
          }

          break;
      }

      // Binary Point Register 0
      case MISCREG_ICC_BPR0:
      case MISCREG_ICC_BPR0_EL1:
        if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
            return readMiscReg(MISCREG_ICV_BPR0_EL1);
        }

        M5_FALLTHROUGH;

      // Binary Point Register 1
      case MISCREG_ICC_BPR1:
      case MISCREG_ICC_BPR1_EL1: {
            if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
                return readMiscReg(MISCREG_ICV_BPR1_EL1);
            }

            Gicv3::GroupId group =
                misc_reg == MISCREG_ICC_BPR0_EL1 ? Gicv3::G0S : Gicv3::G1S;

            if (group == Gicv3::G1S && !inSecureState()) {
                group = Gicv3::G1NS;
            }

            ICC_CTLR_EL1 icc_ctlr_el1_s =
                isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S);

            if ((group == Gicv3::G1S) && !isEL3OrMon() &&
                icc_ctlr_el1_s.CBPR) {
                group = Gicv3::G0S;
            }

            bool sat_inc = false;

            ICC_CTLR_EL1 icc_ctlr_el1_ns =
                isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS);

            if ((group == Gicv3::G1NS) && (currEL() < EL3) &&
                icc_ctlr_el1_ns.CBPR) {
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

      // Virtual Binary Point Register 1
      case MISCREG_ICV_BPR0_EL1:
      case MISCREG_ICV_BPR1_EL1: {
          Gicv3::GroupId group =
              misc_reg == MISCREG_ICV_BPR0_EL1 ? Gicv3::G0S : Gicv3::G1NS;
          ICH_VMCR_EL2 ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
          bool sat_inc = false;

          if ((group == Gicv3::G1NS) && ich_vmcr_el2.VCBPR) {
              // bpr0 + 1 saturated to 7, WI
              group = Gicv3::G0S;
              sat_inc = true;
          }

          uint8_t vbpr;

          if (group == Gicv3::G0S) {
              vbpr = ich_vmcr_el2.VBPR0;
          } else {
              vbpr = ich_vmcr_el2.VBPR1;
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

      // Interrupt Priority Mask Register
      case MISCREG_ICC_PMR:
      case MISCREG_ICC_PMR_EL1:
        if ((currEL() == EL1) && !inSecureState() && (hcr_imo || hcr_fmo)) {
            return isa->readMiscRegNoEffect(MISCREG_ICV_PMR_EL1);
        }

        if (haveEL(EL3) && !inSecureState() &&
            (isa->readMiscRegNoEffect(MISCREG_SCR_EL3) & (1U << 2))) {
            // Spec section 4.8.1
            // For Non-secure access to ICC_PMR_EL1 when SCR_EL3.FIQ == 1:
            if ((value & 0x80) == 0) {
                // If the current priority mask value is in the range of
                // 0x00-0x7F a read access returns the value 0x00.
                value = 0;
            } else if (value != 0xff) {
                // If the current priority mask value is in the range of
                // 0x80-0xFF a read access returns the Non-secure read of the
                // current value.
                value = (value << 1) & 0xff;
            }
        }

        break;

      // Interrupt Acknowledge Register 0
      case MISCREG_ICC_IAR0:
      case MISCREG_ICC_IAR0_EL1: {
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

      // Virtual Interrupt Acknowledge Register 0
      case MISCREG_ICV_IAR0_EL1: {
          int lr_idx = getHPPVILR();
          uint32_t int_id = Gicv3::INTID_SPURIOUS;

          if (lr_idx >= 0) {
              ICH_LR_EL2 ich_lr_el2 =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

              if (!ich_lr_el2.Group && hppviCanPreempt(lr_idx)) {
                  int_id = ich_lr_el2.vINTID;

                  if (int_id < Gicv3::INTID_SECURE ||
                      int_id > Gicv3::INTID_SPURIOUS) {
                      virtualActivateIRQ(lr_idx);
                  } else {
                      // Bogus... Pseudocode says:
                      // - Move from pending to invalid...
                      // - Return de bogus id...
                      ich_lr_el2.State = ICH_LR_EL2_STATE_INVALID;
                      isa->setMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx,
                                              ich_lr_el2);
                  }
              }
          }

          value = int_id;
          virtualUpdate();
          break;
      }

      // Interrupt Acknowledge Register 1
      case MISCREG_ICC_IAR1:
      case MISCREG_ICC_IAR1_EL1: {
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

      // Virtual Interrupt Acknowledge Register 1
      case MISCREG_ICV_IAR1_EL1: {
          int lr_idx = getHPPVILR();
          uint32_t int_id = Gicv3::INTID_SPURIOUS;

          if (lr_idx >= 0) {
              ICH_LR_EL2 ich_lr_el2 =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

              if (ich_lr_el2.Group && hppviCanPreempt(lr_idx)) {
                  int_id = ich_lr_el2.vINTID;

                  if (int_id < Gicv3::INTID_SECURE ||
                      int_id > Gicv3::INTID_SPURIOUS) {
                      virtualActivateIRQ(lr_idx);
                  } else {
                      // Bogus... Pseudocode says:
                      // - Move from pending to invalid...
                      // - Return de bogus id...
                      ich_lr_el2.State = ICH_LR_EL2_STATE_INVALID;
                      isa->setMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx,
                                              ich_lr_el2);
                  }
              }
          }

          value = int_id;
          virtualUpdate();
          break;
      }

      // System Register Enable Register EL1
      case MISCREG_ICC_SRE:
      case MISCREG_ICC_SRE_EL1: {
        /*
         * DIB [2] == 1 (IRQ bypass not supported, RAO/WI)
         * DFB [1] == 1 (FIQ bypass not supported, RAO/WI)
         * SRE [0] == 1 (Only system register interface supported, RAO/WI)
         */
          ICC_SRE_EL1 icc_sre_el1 = 0;
          icc_sre_el1.SRE = 1;
          icc_sre_el1.DIB = 1;
          icc_sre_el1.DFB = 1;
          value = icc_sre_el1;
          break;
      }

      // System Register Enable Register EL2
      case MISCREG_ICC_HSRE:
      case MISCREG_ICC_SRE_EL2: {
        /*
         * Enable [3] == 1
         * (EL1 accesses to ICC_SRE_EL1 do not trap to EL2, RAO/WI)
         * DIB [2] == 1 (IRQ bypass not supported, RAO/WI)
         * DFB [1] == 1 (FIQ bypass not supported, RAO/WI)
         * SRE [0] == 1 (Only system register interface supported, RAO/WI)
         */
        ICC_SRE_EL2 icc_sre_el2 = 0;
        icc_sre_el2.SRE = 1;
        icc_sre_el2.DIB = 1;
        icc_sre_el2.DFB = 1;
        icc_sre_el2.Enable = 1;
        value = icc_sre_el2;
        break;
      }

      // System Register Enable Register EL3
      case MISCREG_ICC_MSRE:
      case MISCREG_ICC_SRE_EL3: {
        /*
         * Enable [3] == 1
         * (EL1 accesses to ICC_SRE_EL1 do not trap to EL3.
         *  EL2 accesses to ICC_SRE_EL1 and ICC_SRE_EL2 do not trap to EL3.
         *  RAO/WI)
         * DIB [2] == 1 (IRQ bypass not supported, RAO/WI)
         * DFB [1] == 1 (FIQ bypass not supported, RAO/WI)
         * SRE [0] == 1 (Only system register interface supported, RAO/WI)
         */
        ICC_SRE_EL3 icc_sre_el3 = 0;
        icc_sre_el3.SRE = 1;
        icc_sre_el3.DIB = 1;
        icc_sre_el3.DFB = 1;
        icc_sre_el3.Enable = 1;
        value = icc_sre_el3;
        break;
      }

      // Control Register
      case MISCREG_ICC_CTLR:
      case MISCREG_ICC_CTLR_EL1: {
          if ((currEL() == EL1) && !inSecureState() && (hcr_imo || hcr_fmo)) {
              return readMiscReg(MISCREG_ICV_CTLR_EL1);
          }

          // Enforce value for RO bits
          // ExtRange [19], INTIDs in the range 1024..8191 not supported
          // RSS [18], SGIs with affinity level 0 values of 0-255 are supported
          // A3V [15], supports non-zero values of the Aff3 field in SGI
          //           generation System registers
          // SEIS [14], does not support generation of SEIs (deprecated)
          // IDbits [13:11], 001 = 24 bits | 000 = 16 bits
          // PRIbits [10:8], number of priority bits implemented, minus one
          ICC_CTLR_EL1 icc_ctlr_el1 = value;
          icc_ctlr_el1.ExtRange = 0;
          icc_ctlr_el1.RSS = 1;
          icc_ctlr_el1.A3V = 1;
          icc_ctlr_el1.SEIS = 0;
          icc_ctlr_el1.IDbits = 1;
          icc_ctlr_el1.PRIbits = PRIORITY_BITS - 1;
          value = icc_ctlr_el1;
          break;
      }

      // Virtual Control Register
      case MISCREG_ICV_CTLR_EL1: {
          ICV_CTLR_EL1 icv_ctlr_el1 = value;
          icv_ctlr_el1.RSS = 0;
          icv_ctlr_el1.A3V = 1;
          icv_ctlr_el1.SEIS = 0;
          icv_ctlr_el1.IDbits = 1;
          icv_ctlr_el1.PRIbits = 7;
          value = icv_ctlr_el1;
          break;
      }

      // Control Register
      case MISCREG_ICC_MCTLR:
      case MISCREG_ICC_CTLR_EL3: {
          // Enforce value for RO bits
          // ExtRange [19], INTIDs in the range 1024..8191 not supported
          // RSS [18], SGIs with affinity level 0 values of 0-255 are supported
          // nDS [17], supports disabling of security
          // A3V [15], supports non-zero values of the Aff3 field in SGI
          //           generation System registers
          // SEIS [14], does not support generation of SEIs (deprecated)
          // IDbits [13:11], 001 = 24 bits | 000 = 16 bits
          // PRIbits [10:8], number of priority bits implemented, minus one
          ICC_CTLR_EL3 icc_ctlr_el3 = value;
          icc_ctlr_el3.ExtRange = 0;
          icc_ctlr_el3.RSS = 1;
          icc_ctlr_el3.nDS = 0;
          icc_ctlr_el3.A3V = 1;
          icc_ctlr_el3.SEIS = 0;
          icc_ctlr_el3.IDbits = 0;
          icc_ctlr_el3.PRIbits = PRIORITY_BITS - 1;
          value = icc_ctlr_el3;
          break;
      }

      // Hyp Control Register
      case MISCREG_ICH_HCR:
      case MISCREG_ICH_HCR_EL2:
        break;

      // Hyp Active Priorities Group 0 Registers
      case MISCREG_ICH_AP0R0:
      case MISCREG_ICH_AP0R0_EL2:
        break;

      // Hyp Active Priorities Group 1 Registers
      case MISCREG_ICH_AP1R0:
      case MISCREG_ICH_AP1R0_EL2:
        break;

      // Maintenance Interrupt State Register
      case MISCREG_ICH_MISR:
      case MISCREG_ICH_MISR_EL2:
        value = maintenanceInterruptStatus();
        break;

      // VGIC Type Register
      case MISCREG_ICH_VTR:
      case MISCREG_ICH_VTR_EL2: {
        ICH_VTR_EL2 ich_vtr_el2 = value;

        ich_vtr_el2.ListRegs = VIRTUAL_NUM_LIST_REGS - 1;
        ich_vtr_el2.A3V = 1;
        ich_vtr_el2.IDbits = 1;
        ich_vtr_el2.PREbits = VIRTUAL_PREEMPTION_BITS - 1;
        ich_vtr_el2.PRIbits = VIRTUAL_PRIORITY_BITS - 1;

        value = ich_vtr_el2;
        break;
      }

      // End of Interrupt Status Register
      case MISCREG_ICH_EISR:
      case MISCREG_ICH_EISR_EL2:
        value = eoiMaintenanceInterruptStatus();
        break;

      // Empty List Register Status Register
      case MISCREG_ICH_ELRSR:
      case MISCREG_ICH_ELRSR_EL2:
        value = 0;

        for (int lr_idx = 0; lr_idx < VIRTUAL_NUM_LIST_REGS; lr_idx++) {
            ICH_LR_EL2 ich_lr_el2 =
                isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

            if ((ich_lr_el2.State  == ICH_LR_EL2_STATE_INVALID) &&
                (ich_lr_el2.HW || !ich_lr_el2.EOI)) {
                value |= (1 << lr_idx);
            }
        }

        break;

      // List Registers
      case MISCREG_ICH_LRC0 ... MISCREG_ICH_LRC15:
        // AArch32 (maps to AArch64 MISCREG_ICH_LR<n>_EL2 high half part)
        value = value >> 32;
        break;

      // List Registers
      case MISCREG_ICH_LR0 ... MISCREG_ICH_LR15:
        // AArch32 (maps to AArch64 MISCREG_ICH_LR<n>_EL2 low half part)
        value = value & 0xffffffff;
        break;

      // List Registers
      case MISCREG_ICH_LR0_EL2 ... MISCREG_ICH_LR15_EL2:
        break;

      // Virtual Machine Control Register
      case MISCREG_ICH_VMCR:
      case MISCREG_ICH_VMCR_EL2:
        break;

      default:
        panic("Gicv3CPUInterface::readMiscReg(): unknown register %d (%s)",
              misc_reg, miscRegName[misc_reg]);
    }

    DPRINTF(GIC, "Gicv3CPUInterface::readMiscReg(): register %s value %#x\n",
            miscRegName[misc_reg], value);
    return value;
}

void
Gicv3CPUInterface::setMiscReg(int misc_reg, RegVal val)
{
    bool do_virtual_update = false;
    DPRINTF(GIC, "Gicv3CPUInterface::setMiscReg(): register %s value %#x\n",
            miscRegName[misc_reg], val);
    bool hcr_fmo = getHCREL2FMO();
    bool hcr_imo = getHCREL2IMO();

    switch (misc_reg) {
      // Active Priorities Group 1 Registers
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

      // Active Priorities Group 0 Registers
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

      // End Of Interrupt Register 0
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

      // Virtual End Of Interrupt Register 0
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
              ICH_LR_EL2 ich_lr_el2 =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);
              Gicv3::GroupId lr_group =
                  ich_lr_el2.Group ? Gicv3::G1NS : Gicv3::G0S;
              uint8_t lr_group_prio = ich_lr_el2.Priority & 0xf8;

              if (lr_group == Gicv3::G0S && lr_group_prio == drop_prio) {
                  //if (!virtualIsEOISplitMode())
                  {
                      virtualDeactivateIRQ(lr_idx);
                  }
              }
          }

          virtualUpdate();
          break;
      }

      // End Of Interrupt Register 1
      case MISCREG_ICC_EOIR1:
      case MISCREG_ICC_EOIR1_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
              return setMiscReg(MISCREG_ICV_EOIR1_EL1, val);
          }

          int int_id = val & 0xffffff;

          // avoid deactivation for special interrupts
          if (int_id >= Gicv3::INTID_SECURE) {
              return;
          }

          Gicv3::GroupId group = inSecureState() ? Gicv3::G1S : Gicv3::G1NS;

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

      // Virtual End Of Interrupt Register 1
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
              // No matching LR found
              virtualIncrementEOICount();
          } else {
              ICH_LR_EL2 ich_lr_el2 =
                  isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);
              Gicv3::GroupId lr_group =
                  ich_lr_el2.Group ? Gicv3::G1NS : Gicv3::G0S;
              uint8_t lr_group_prio = ich_lr_el2.Priority & 0xf8;

              if (lr_group == Gicv3::G1NS && lr_group_prio == drop_prio) {
                  if (!virtualIsEOISplitMode()) {
                      virtualDeactivateIRQ(lr_idx);
                  }
              }
          }

          virtualUpdate();
          break;
      }

      // Deactivate Interrupt Register
      case MISCREG_ICC_DIR:
      case MISCREG_ICC_DIR_EL1: {
          if ((currEL() == EL1) && !inSecureState() &&
              (hcr_imo || hcr_fmo)) {
              return setMiscReg(MISCREG_ICV_DIR_EL1, val);
          }

          int int_id = val & 0xffffff;

          // The following checks are as per spec pseudocode
          // aarch64/support/ICC_DIR_EL1

          // Check for spurious ID
          if (int_id >= Gicv3::INTID_SECURE) {
              return;
          }

          // EOI mode is not set, so don't deactivate
          if (!isEOISplitMode()) {
              return;
          }

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

      // Deactivate Virtual Interrupt Register
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
              // No matching LR found
              virtualIncrementEOICount();
          } else {
              virtualDeactivateIRQ(lr_idx);
          }

          virtualUpdate();
          break;
      }

      // Binary Point Register 0
      case MISCREG_ICC_BPR0:
      case MISCREG_ICC_BPR0_EL1:
      // Binary Point Register 1
      case MISCREG_ICC_BPR1:
      case MISCREG_ICC_BPR1_EL1: {
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

          ICC_CTLR_EL1 icc_ctlr_el1_s =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S);

          if ((group == Gicv3::G1S) && !isEL3OrMon() &&
              icc_ctlr_el1_s.CBPR) {
              group = Gicv3::G0S;
          }

          ICC_CTLR_EL1 icc_ctlr_el1_ns =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS);

          if ((group == Gicv3::G1NS) && (currEL() < EL3) &&
              icc_ctlr_el1_ns.CBPR) {
              // BPR0 + 1 saturated to 7, WI
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

      // Virtual Binary Point Register 0
      case MISCREG_ICV_BPR0_EL1:
      // Virtual Binary Point Register 1
      case MISCREG_ICV_BPR1_EL1: {
          Gicv3::GroupId group =
              misc_reg == MISCREG_ICV_BPR0_EL1 ? Gicv3::G0S : Gicv3::G1NS;
          ICH_VMCR_EL2 ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);

          if ((group == Gicv3::G1NS) && ich_vmcr_el2.VCBPR) {
              // BPR0 + 1 saturated to 7, WI
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
              ich_vmcr_el2.VBPR0 = val;
          } else {
              ich_vmcr_el2.VBPR1 = val;
          }

          isa->setMiscRegNoEffect(MISCREG_ICH_VMCR_EL2, ich_vmcr_el2);
          do_virtual_update = true;
          break;
      }

      // Control Register EL1
      case MISCREG_ICC_CTLR:
      case MISCREG_ICC_CTLR_EL1: {
          if ((currEL() == EL1) && !inSecureState() && (hcr_imo || hcr_fmo)) {
              return setMiscReg(MISCREG_ICV_CTLR_EL1, val);
          }

          /*
           * ExtRange is RO.
           * RSS is RO.
           * A3V is RO.
           * SEIS is RO.
           * IDbits is RO.
           * PRIbits is RO.
           */
          ICC_CTLR_EL1 requested_icc_ctlr_el1 = val;
          ICC_CTLR_EL1 icc_ctlr_el1 =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1);

          ICC_CTLR_EL3 icc_ctlr_el3 =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL3);

          // The following could be refactored but it is following
          // spec description section 9.2.6 point by point.

          // PMHE
          if (haveEL(EL3)) {
              // PMHE is alias of ICC_CTLR_EL3.PMHE

              if (distributor->DS == 0) {
                  // PMHE is RO
              } else if (distributor->DS == 1) {
                  // PMHE is RW
                  icc_ctlr_el1.PMHE = requested_icc_ctlr_el1.PMHE;
                  icc_ctlr_el3.PMHE = icc_ctlr_el1.PMHE;
              }
          } else {
              // PMHE is RW (by implementation choice)
              icc_ctlr_el1.PMHE = requested_icc_ctlr_el1.PMHE;
          }

          // EOImode
          icc_ctlr_el1.EOImode = requested_icc_ctlr_el1.EOImode;

          if (inSecureState()) {
              // EOIMode is alias of ICC_CTLR_EL3.EOImode_EL1S
              icc_ctlr_el3.EOImode_EL1S = icc_ctlr_el1.EOImode;
          } else {
              // EOIMode is alias of ICC_CTLR_EL3.EOImode_EL1NS
              icc_ctlr_el3.EOImode_EL1NS = icc_ctlr_el1.EOImode;
          }

          // CBPR
          if (haveEL(EL3)) {
              // CBPR is alias of ICC_CTLR_EL3.CBPR_EL1{S,NS}

              if (distributor->DS == 0) {
                  // CBPR is RO
              } else {
                  // CBPR is RW
                  icc_ctlr_el1.CBPR = requested_icc_ctlr_el1.CBPR;

                  if (inSecureState()) {
                      icc_ctlr_el3.CBPR_EL1S = icc_ctlr_el1.CBPR;
                  } else {
                      icc_ctlr_el3.CBPR_EL1NS = icc_ctlr_el1.CBPR;
                  }
              }
          } else {
              // CBPR is RW
              icc_ctlr_el1.CBPR = requested_icc_ctlr_el1.CBPR;
          }

          isa->setMiscRegNoEffect(MISCREG_ICC_CTLR_EL3, icc_ctlr_el3);

          val = icc_ctlr_el1;
          break;
      }

      // Virtual Control Register
      case MISCREG_ICV_CTLR_EL1: {
         ICV_CTLR_EL1 requested_icv_ctlr_el1 = val;
         ICV_CTLR_EL1 icv_ctlr_el1 =
             isa->readMiscRegNoEffect(MISCREG_ICV_CTLR_EL1);
         icv_ctlr_el1.EOImode = requested_icv_ctlr_el1.EOImode;
         icv_ctlr_el1.CBPR = requested_icv_ctlr_el1.CBPR;
         val = icv_ctlr_el1;

         // Aliases
         // ICV_CTLR_EL1.CBPR aliases ICH_VMCR_EL2.VCBPR.
         // ICV_CTLR_EL1.EOImode aliases ICH_VMCR_EL2.VEOIM.
         ICH_VMCR_EL2 ich_vmcr_el2 =
             isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
         ich_vmcr_el2.VCBPR = icv_ctlr_el1.CBPR;
         ich_vmcr_el2.VEOIM = icv_ctlr_el1.EOImode;
         isa->setMiscRegNoEffect(MISCREG_ICH_VMCR_EL2, ich_vmcr_el2);
         break;
      }

      // Control Register EL3
      case MISCREG_ICC_MCTLR:
      case MISCREG_ICC_CTLR_EL3: {
          /*
           * ExtRange is RO.
           * RSS is RO.
           * nDS is RO.
           * A3V is RO.
           * SEIS is RO.
           * IDbits is RO.
           * PRIbits is RO.
           * PMHE is RAO/WI, priority-based routing is always used.
           */
          ICC_CTLR_EL3 requested_icc_ctlr_el3 = val;

          // Aliases
          if (haveEL(EL3))
          {
              ICC_CTLR_EL1 icc_ctlr_el1_s =
                  isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S);
              ICC_CTLR_EL1 icc_ctlr_el1_ns =
                  isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS);

              // ICC_CTLR_EL1(NS).EOImode is an alias of
              // ICC_CTLR_EL3.EOImode_EL1NS
              icc_ctlr_el1_ns.EOImode = requested_icc_ctlr_el3.EOImode_EL1NS;
              // ICC_CTLR_EL1(S).EOImode is an alias of
              // ICC_CTLR_EL3.EOImode_EL1S
              icc_ctlr_el1_s.EOImode = requested_icc_ctlr_el3.EOImode_EL1S;
              // ICC_CTLR_EL1(NS).CBPR is an alias of ICC_CTLR_EL3.CBPR_EL1NS
              icc_ctlr_el1_ns.CBPR = requested_icc_ctlr_el3.CBPR_EL1NS;
              // ICC_CTLR_EL1(S).CBPR is an alias of ICC_CTLR_EL3.CBPR_EL1S
              icc_ctlr_el1_s.CBPR = requested_icc_ctlr_el3.CBPR_EL1S;

              isa->setMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S, icc_ctlr_el1_s);
              isa->setMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS,
                                      icc_ctlr_el1_ns);
          }

          ICC_CTLR_EL3 icc_ctlr_el3 =
              isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL3);

          icc_ctlr_el3.RM = requested_icc_ctlr_el3.RM;
          icc_ctlr_el3.EOImode_EL1NS = requested_icc_ctlr_el3.EOImode_EL1NS;
          icc_ctlr_el3.EOImode_EL1S = requested_icc_ctlr_el3.EOImode_EL1S;
          icc_ctlr_el3.EOImode_EL3 = requested_icc_ctlr_el3.EOImode_EL3;
          icc_ctlr_el3.CBPR_EL1NS = requested_icc_ctlr_el3.CBPR_EL1NS;
          icc_ctlr_el3.CBPR_EL1S = requested_icc_ctlr_el3.CBPR_EL1S;

          val = icc_ctlr_el3;
          break;
      }

      // Priority Mask Register
      case MISCREG_ICC_PMR:
      case MISCREG_ICC_PMR_EL1: {
          if ((currEL() == EL1) && !inSecureState() && (hcr_imo || hcr_fmo)) {
              return isa->setMiscRegNoEffect(MISCREG_ICV_PMR_EL1, val);
          }

          val &= 0xff;
          SCR scr_el3 = isa->readMiscRegNoEffect(MISCREG_SCR_EL3);

          if (haveEL(EL3) && !inSecureState() && (scr_el3.fiq)) {
              // Spec section 4.8.1
              // For Non-secure access to ICC_PMR_EL1 SCR_EL3.FIQ == 1:
              RegVal old_icc_pmr_el1 =
                  isa->readMiscRegNoEffect(MISCREG_ICC_PMR_EL1);

              if (!(old_icc_pmr_el1 & 0x80)) {
                  // If the current priority mask value is in the range of
                  // 0x00-0x7F then WI
                  return;
              }

              // If the current priority mask value is in the range of
              // 0x80-0xFF then a write access to ICC_PMR_EL1 succeeds,
              // based on the Non-secure read of the priority mask value
              // written to the register.

              val = (val >> 1) | 0x80;
          }

          val &= ~0U << (8 - PRIORITY_BITS);
          break;
      }

      // Interrupt Group 0 Enable Register EL1
      case MISCREG_ICC_IGRPEN0:
      case MISCREG_ICC_IGRPEN0_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_fmo) {
              return setMiscReg(MISCREG_ICV_IGRPEN0_EL1, val);
          }

          break;
      }

      // Virtual Interrupt Group 0 Enable register
      case MISCREG_ICV_IGRPEN0_EL1: {
          bool enable = val & 0x1;
          ICH_VMCR_EL2 ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
          ich_vmcr_el2.VENG0 = enable;
          isa->setMiscRegNoEffect(MISCREG_ICH_VMCR_EL2, ich_vmcr_el2);
          virtualUpdate();
          return;
      }

      // Interrupt Group 1 Enable register EL1
      case MISCREG_ICC_IGRPEN1:
      case MISCREG_ICC_IGRPEN1_EL1: {
          if ((currEL() == EL1) && !inSecureState() && hcr_imo) {
              return setMiscReg(MISCREG_ICV_IGRPEN1_EL1, val);
          }

          if (haveEL(EL3)) {
              ICC_IGRPEN1_EL1 icc_igrpen1_el1 = val;
              ICC_IGRPEN1_EL3 icc_igrpen1_el3 =
                  isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL3);

              if (inSecureState()) {
                  // Enable is RW alias of ICC_IGRPEN1_EL3.EnableGrp1S
                  icc_igrpen1_el3.EnableGrp1S = icc_igrpen1_el1.Enable;
              } else {
                  // Enable is RW alias of ICC_IGRPEN1_EL3.EnableGrp1NS
                  icc_igrpen1_el3.EnableGrp1NS = icc_igrpen1_el1.Enable;
              }

              isa->setMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL3,
                                      icc_igrpen1_el3);
          }

          break;
      }

      // Virtual Interrupt Group 1 Enable register
      case MISCREG_ICV_IGRPEN1_EL1: {
          bool enable = val & 0x1;
          ICH_VMCR_EL2 ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
          ich_vmcr_el2.VENG1 = enable;
          isa->setMiscRegNoEffect(MISCREG_ICH_VMCR_EL2, ich_vmcr_el2);
          virtualUpdate();
          return;
      }

      // Interrupt Group 1 Enable register
      case MISCREG_ICC_MGRPEN1:
      case MISCREG_ICC_IGRPEN1_EL3: {
          ICC_IGRPEN1_EL3 icc_igrpen1_el3 = val;
          ICC_IGRPEN1_EL1 icc_igrpen1_el1 =
              isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1);

          if (inSecureState()) {
              // ICC_IGRPEN1_EL1.Enable is RW alias of EnableGrp1S
              icc_igrpen1_el1.Enable = icc_igrpen1_el3.EnableGrp1S;
          } else {
              // ICC_IGRPEN1_EL1.Enable is RW alias of EnableGrp1NS
              icc_igrpen1_el1.Enable = icc_igrpen1_el3.EnableGrp1NS;
          }

          isa->setMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1, icc_igrpen1_el1);
          break;
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

      // System Register Enable Register EL1
      case MISCREG_ICC_SRE:
      case MISCREG_ICC_SRE_EL1:
      // System Register Enable Register EL2
      case MISCREG_ICC_HSRE:
      case MISCREG_ICC_SRE_EL2:
      // System Register Enable Register EL3
      case MISCREG_ICC_MSRE:
      case MISCREG_ICC_SRE_EL3:
        // All bits are RAO/WI
        return;

      // Hyp Control Register
      case MISCREG_ICH_HCR:
      case MISCREG_ICH_HCR_EL2: {
        ICH_HCR_EL2 requested_ich_hcr_el2 = val;
        ICH_HCR_EL2 ich_hcr_el2 =
            isa->readMiscRegNoEffect(MISCREG_ICH_HCR_EL2);

        if (requested_ich_hcr_el2.EOIcount >= ich_hcr_el2.EOIcount)
        {
            // EOIcount - Permitted behaviors are:
            // - Increment EOIcount.
            // - Leave EOIcount unchanged.
            ich_hcr_el2.EOIcount = requested_ich_hcr_el2.EOIcount;
        }

        ich_hcr_el2.TDIR = requested_ich_hcr_el2.TDIR;
        ich_hcr_el2.TSEI = requested_ich_hcr_el2.TSEI;
        ich_hcr_el2.TALL1 = requested_ich_hcr_el2.TALL1;;
        ich_hcr_el2.TALL0 = requested_ich_hcr_el2.TALL0;;
        ich_hcr_el2.TC = requested_ich_hcr_el2.TC;
        ich_hcr_el2.VGrp1DIE = requested_ich_hcr_el2.VGrp1DIE;
        ich_hcr_el2.VGrp1EIE = requested_ich_hcr_el2.VGrp1EIE;
        ich_hcr_el2.VGrp0DIE = requested_ich_hcr_el2.VGrp0DIE;
        ich_hcr_el2.VGrp0EIE = requested_ich_hcr_el2.VGrp0EIE;
        ich_hcr_el2.NPIE = requested_ich_hcr_el2.NPIE;
        ich_hcr_el2.LRENPIE = requested_ich_hcr_el2.LRENPIE;
        ich_hcr_el2.UIE = requested_ich_hcr_el2.UIE;
        ich_hcr_el2.En = requested_ich_hcr_el2.En;
        val = ich_hcr_el2;
        do_virtual_update = true;
        break;
      }

      // List Registers
      case MISCREG_ICH_LRC0 ... MISCREG_ICH_LRC15: {
        // AArch32 (maps to AArch64 MISCREG_ICH_LR<n>_EL2 high half part)
        ICH_LRC requested_ich_lrc = val;
        ICH_LRC ich_lrc = isa->readMiscRegNoEffect(misc_reg);

        ich_lrc.State = requested_ich_lrc.State;
        ich_lrc.HW = requested_ich_lrc.HW;
        ich_lrc.Group = requested_ich_lrc.Group;

        // Priority, bits [23:16]
        // At least five bits must be implemented.
        // Unimplemented bits are RES0 and start from bit[16] up to bit[18].
        // We implement 5 bits.
        ich_lrc.Priority = (requested_ich_lrc.Priority & 0xf8) |
                           (ich_lrc.Priority & 0x07);

        // pINTID, bits [12:0]
        // When ICH_LR<n>.HW is 0 this field has the following meaning:
        // - Bits[12:10] : RES0.
        // - Bit[9] : EOI.
        // - Bits[8:0] : RES0.
        // When ICH_LR<n>.HW is 1:
        // - This field is only required to implement enough bits to hold a
        // valid value for the implemented INTID size. Any unused higher
        // order bits are RES0.
        if (requested_ich_lrc.HW == 0) {
            ich_lrc.EOI = requested_ich_lrc.EOI;
        } else {
            ich_lrc.pINTID = requested_ich_lrc.pINTID;
        }

        val = ich_lrc;
        do_virtual_update = true;
        break;
      }

      // List Registers
      case MISCREG_ICH_LR0 ... MISCREG_ICH_LR15: {
          // AArch32 (maps to AArch64 MISCREG_ICH_LR<n>_EL2 low half part)
          RegVal old_val = isa->readMiscRegNoEffect(misc_reg);
          val = (old_val & 0xffffffff00000000) | (val & 0xffffffff);
          do_virtual_update = true;
          break;
      }

      // List Registers
      case MISCREG_ICH_LR0_EL2 ... MISCREG_ICH_LR15_EL2: { // AArch64
          ICH_LR_EL2 requested_ich_lr_el2 = val;
          ICH_LR_EL2 ich_lr_el2 = isa->readMiscRegNoEffect(misc_reg);

          ich_lr_el2.State = requested_ich_lr_el2.State;
          ich_lr_el2.HW = requested_ich_lr_el2.HW;
          ich_lr_el2.Group = requested_ich_lr_el2.Group;

          // Priority, bits [55:48]
          // At least five bits must be implemented.
          // Unimplemented bits are RES0 and start from bit[48] up to bit[50].
          // We implement 5 bits.
          ich_lr_el2.Priority = (requested_ich_lr_el2.Priority & 0xf8) |
                                (ich_lr_el2.Priority & 0x07);

          // pINTID, bits [44:32]
          // When ICH_LR<n>_EL2.HW is 0 this field has the following meaning:
          // - Bits[44:42] : RES0.
          // - Bit[41] : EOI.
          // - Bits[40:32] : RES0.
          // When ICH_LR<n>_EL2.HW is 1:
          // - This field is only required to implement enough bits to hold a
          // valid value for the implemented INTID size. Any unused higher
          // order bits are RES0.
          if (requested_ich_lr_el2.HW == 0) {
              ich_lr_el2.EOI = requested_ich_lr_el2.EOI;
          } else {
              ich_lr_el2.pINTID = requested_ich_lr_el2.pINTID;
          }

          // vINTID, bits [31:0]
          // It is IMPLEMENTATION DEFINED how many bits are implemented,
          // though at least 16 bits must be implemented.
          // Unimplemented bits are RES0.
          ich_lr_el2.vINTID = requested_ich_lr_el2.vINTID;

          val = ich_lr_el2;
          do_virtual_update = true;
          break;
      }

      // Virtual Machine Control Register
      case MISCREG_ICH_VMCR:
      case MISCREG_ICH_VMCR_EL2: {
          ICH_VMCR_EL2 requested_ich_vmcr_el2 = val;
          ICH_VMCR_EL2 ich_vmcr_el2 =
              isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
          ich_vmcr_el2.VPMR = requested_ich_vmcr_el2.VPMR;
          uint8_t min_vpr0 = 7 - VIRTUAL_PREEMPTION_BITS;

          if (requested_ich_vmcr_el2.VBPR0 < min_vpr0) {
              ich_vmcr_el2.VBPR0 = min_vpr0;
          } else {
              ich_vmcr_el2.VBPR0 = requested_ich_vmcr_el2.VBPR0;
          }

          uint8_t min_vpr1 = min_vpr0 + 1;

          if (requested_ich_vmcr_el2.VBPR1 < min_vpr1) {
              ich_vmcr_el2.VBPR1 = min_vpr1;
          } else {
              ich_vmcr_el2.VBPR1 = requested_ich_vmcr_el2.VBPR1;
          }

          ich_vmcr_el2.VEOIM = requested_ich_vmcr_el2.VEOIM;
          ich_vmcr_el2.VCBPR = requested_ich_vmcr_el2.VCBPR;
          ich_vmcr_el2.VENG1 = requested_ich_vmcr_el2.VENG1;
          ich_vmcr_el2.VENG0 = requested_ich_vmcr_el2.VENG0;
          val = ich_vmcr_el2;
          break;
      }

      // Hyp Active Priorities Group 0 Registers
      case MISCREG_ICH_AP0R0 ... MISCREG_ICH_AP0R3:
      case MISCREG_ICH_AP0R0_EL2 ... MISCREG_ICH_AP0R3_EL2:
      // Hyp Active Priorities Group 1 Registers
      case MISCREG_ICH_AP1R0 ... MISCREG_ICH_AP1R3:
      case MISCREG_ICH_AP1R0_EL2 ... MISCREG_ICH_AP1R3_EL2:
        break;

      default:
        panic("Gicv3CPUInterface::setMiscReg(): unknown register %d (%s)",
              misc_reg, miscRegName[misc_reg]);
    }

    isa->setMiscRegNoEffect(misc_reg, val);

    if (do_virtual_update) {
        virtualUpdate();
    }
}

int
Gicv3CPUInterface::virtualFindActive(uint32_t int_id) const
{
    for (uint32_t lr_idx = 0; lr_idx < VIRTUAL_NUM_LIST_REGS; lr_idx++) {
        ICH_LR_EL2 ich_lr_el2 =
            isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

        if (((ich_lr_el2.State == ICH_LR_EL2_STATE_ACTIVE) ||
             (ich_lr_el2.State == ICH_LR_EL2_STATE_ACTIVE_PENDING)) &&
            (ich_lr_el2.vINTID == int_id)) {
            return lr_idx;
        }
    }

    return -1;
}

uint32_t
Gicv3CPUInterface::getHPPIR0() const
{
    if (hppi.prio == 0xff) {
        return Gicv3::INTID_SPURIOUS;
    }

    bool irq_is_secure = !distributor->DS && hppi.group != Gicv3::G1NS;

    if ((hppi.group != Gicv3::G0S) && isEL3OrMon()) {
        // interrupt for the other state pending
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
Gicv3CPUInterface::getHPPIR1() const
{
    if (hppi.prio == 0xff) {
        return Gicv3::INTID_SPURIOUS;
    }

    ICC_CTLR_EL3 icc_ctlr_el3 = isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL3);
    if ((currEL() == EL3) && icc_ctlr_el3.RM) {
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
        apr &= apr - 1;
        isa->setMiscRegNoEffect(apr_misc_reg, apr);
    }

    update();
}

uint8_t
Gicv3CPUInterface::virtualDropPriority()
{
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
            vapr0 &= vapr0 - 1;
            isa->setMiscRegNoEffect(MISCREG_ICH_AP0R0_EL2 + i, vapr0);
            return (vapr0_count + i * 32) << (GIC_MIN_VBPR + 1);
        } else {
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
    ICH_LR_EL2 ich_lr_el = isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 +
            lr_idx);
    Gicv3::GroupId group = ich_lr_el.Group ? Gicv3::G1NS : Gicv3::G0S;
    uint8_t prio = ich_lr_el.Priority & 0xf8;
    int apr_bit = prio >> (8 - VIRTUAL_PREEMPTION_BITS);
    int reg_no = apr_bit / 32;
    int reg_bit = apr_bit % 32;
    int apr_idx = group == Gicv3::G0S ?
        MISCREG_ICH_AP0R0_EL2 + reg_no : MISCREG_ICH_AP1R0_EL2 + reg_no;
    RegVal apr = isa->readMiscRegNoEffect(apr_idx);
    apr |= (1 << reg_bit);
    isa->setMiscRegNoEffect(apr_idx, apr);
    // Move interrupt state from pending to active.
    ich_lr_el.State = ICH_LR_EL2_STATE_ACTIVE;
    isa->setMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx, ich_lr_el);
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
    ICH_LR_EL2 ich_lr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 +
            lr_idx);

    if (ich_lr_el2.HW) {
        // Deactivate the associated physical interrupt
        if (ich_lr_el2.pINTID < Gicv3::INTID_SECURE) {
            Gicv3::GroupId group = ich_lr_el2.pINTID >= 32 ?
                distributor->getIntGroup(ich_lr_el2.pINTID) :
                redistributor->getIntGroup(ich_lr_el2.pINTID);
            deactivateIRQ(ich_lr_el2.pINTID, group);
        }
    }

    //  Remove the active bit
    ich_lr_el2.State = ich_lr_el2.State & ~ICH_LR_EL2_STATE_ACTIVE;
    isa->setMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx, ich_lr_el2);
}

/*
 * Returns the priority group field for the current BPR value for the group.
 * GroupBits() Pseudocode from spec.
 */
uint32_t
Gicv3CPUInterface::groupPriorityMask(Gicv3::GroupId group) const
{
    ICC_CTLR_EL1 icc_ctlr_el1_s =
        isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_S);
    ICC_CTLR_EL1 icc_ctlr_el1_ns =
        isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1_NS);

    if ((group == Gicv3::G1S && icc_ctlr_el1_s.CBPR) ||
        (group == Gicv3::G1NS && icc_ctlr_el1_ns.CBPR)) {
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
Gicv3CPUInterface::virtualGroupPriorityMask(Gicv3::GroupId group) const
{
    ICH_VMCR_EL2 ich_vmcr_el2 =
        isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);

    if ((group == Gicv3::G1NS) && ich_vmcr_el2.VCBPR) {
        group = Gicv3::G0S;
    }

    int bpr;

    if (group == Gicv3::G0S) {
        bpr = ich_vmcr_el2.VBPR0;
    } else {
        bpr = ich_vmcr_el2.VBPR1;
    }

    if (group == Gicv3::G1NS) {
        assert(bpr > 0);
        bpr--;
    }

    return ~0U << (bpr + 1);
}

bool
Gicv3CPUInterface::isEOISplitMode() const
{
    if (isEL3OrMon()) {
        ICC_CTLR_EL3 icc_ctlr_el3 =
            isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL3);
        return icc_ctlr_el3.EOImode_EL3;
    } else {
        ICC_CTLR_EL1 icc_ctlr_el1 =
            isa->readMiscRegNoEffect(MISCREG_ICC_CTLR_EL1);
        return icc_ctlr_el1.EOImode;
    }
}

bool
Gicv3CPUInterface::virtualIsEOISplitMode() const
{
    ICH_VMCR_EL2 ich_vmcr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);
    return ich_vmcr_el2.VEOIM;
}

int
Gicv3CPUInterface::highestActiveGroup() const
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
        ICH_LR_EL2 ich_lr_el2 =
            isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

        if (hppviCanPreempt(lr_idx)) {
            if (ich_lr_el2.Group) {
                signal_IRQ = true;
            } else {
                signal_FIQ = true;
            }
        }
    }

    ICH_HCR_EL2 ich_hcr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_HCR_EL2);

    if (ich_hcr_el2.En) {
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

// Returns the index of the LR with the HPPI
int
Gicv3CPUInterface::getHPPVILR() const
{
    int idx = -1;
    ICH_VMCR_EL2 ich_vmcr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);

    if (!ich_vmcr_el2.VENG0 && !ich_vmcr_el2.VENG1) {
        // VG0 and VG1 disabled...
        return idx;
    }

    uint8_t highest_prio = 0xff;

    for (int i = 0; i < 16; i++) {
        ICH_LR_EL2 ich_lr_el2 =
            isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + i);

        if (ich_lr_el2.State != Gicv3::INT_PENDING) {
            continue;
        }

        if (ich_lr_el2.Group) {
            // VG1
            if (!ich_vmcr_el2.VENG1) {
                continue;
            }
        } else {
            // VG0
            if (!ich_vmcr_el2.VENG0) {
                continue;
            }
        }

        uint8_t prio = ich_lr_el2.Priority;

        if (prio < highest_prio) {
            highest_prio = prio;
            idx = i;
        }
    }

    return idx;
}

bool
Gicv3CPUInterface::hppviCanPreempt(int lr_idx) const
{
    ICH_HCR_EL2 ich_hcr_el2 = isa->readMiscRegNoEffect(MISCREG_ICH_HCR_EL2);
    if (!ich_hcr_el2.En) {
        // virtual interface is disabled
        return false;
    }

    ICH_LR_EL2 ich_lr_el2 =
        isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);
    uint8_t prio = ich_lr_el2.Priority;
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

    Gicv3::GroupId group = ich_lr_el2.Group ? Gicv3::G1NS : Gicv3::G0S;
    uint32_t prio_mask = virtualGroupPriorityMask(group);

    if ((prio & prio_mask) < (rprio & prio_mask)) {
        return true;
    }

    return false;
}

uint8_t
Gicv3CPUInterface::virtualHighestActivePriority() const
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

// spec section 4.6.2
ArmISA::InterruptTypes
Gicv3CPUInterface::intSignalType(Gicv3::GroupId group) const
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
Gicv3CPUInterface::hppiCanPreempt() const
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
Gicv3CPUInterface::highestActivePriority() const
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
Gicv3CPUInterface::groupEnabled(Gicv3::GroupId group) const
{
    switch (group) {
      case Gicv3::G0S: {
        ICC_IGRPEN0_EL1 icc_igrpen0_el1 =
            isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN0_EL1);
        return icc_igrpen0_el1.Enable;
      }

      case Gicv3::G1S: {
        ICC_IGRPEN1_EL1 icc_igrpen1_el1_s =
            isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1_S);
        return icc_igrpen1_el1_s.Enable;
      }

      case Gicv3::G1NS: {
        ICC_IGRPEN1_EL1 icc_igrpen1_el1_ns =
            isa->readMiscRegNoEffect(MISCREG_ICC_IGRPEN1_EL1_NS);
        return icc_igrpen1_el1_ns.Enable;
      }

      default:
        panic("Gicv3CPUInterface::groupEnable(): invalid group!\n");
    }
}

bool
Gicv3CPUInterface::inSecureState() const
{
    if (!gic->getSystem()->haveSecurity()) {
        return false;
    }

    CPSR cpsr = isa->readMiscRegNoEffect(MISCREG_CPSR);
    SCR scr = isa->readMiscRegNoEffect(MISCREG_SCR);
    return ArmISA::inSecureState(scr, cpsr);
}

int
Gicv3CPUInterface::currEL() const
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
Gicv3CPUInterface::haveEL(ExceptionLevel el) const
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
Gicv3CPUInterface::isSecureBelowEL3() const
{
    SCR scr = isa->readMiscRegNoEffect(MISCREG_SCR_EL3);
    return haveEL(EL3) && scr.ns == 0;
}

bool
Gicv3CPUInterface::isAA64() const
{
    CPSR cpsr = isa->readMiscRegNoEffect(MISCREG_CPSR);
    return opModeIs64((OperatingMode)(uint8_t) cpsr.mode);
}

bool
Gicv3CPUInterface::isEL3OrMon() const
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

// Computes ICH_EISR_EL2
uint64_t
Gicv3CPUInterface::eoiMaintenanceInterruptStatus() const
{
    // ICH_EISR_EL2
    // Bits [63:16] - RES0
    // Status<n>, bit [n], for n = 0 to 15
    //   EOI maintenance interrupt status bit for List register <n>:
    //     0 if List register <n>, ICH_LR<n>_EL2, does not have an EOI
    //     maintenance interrupt.
    //     1 if List register <n>, ICH_LR<n>_EL2, has an EOI maintenance
    //     interrupt that has not been handled.
    //
    // For any ICH_LR<n>_EL2, the corresponding status bit is set to 1 if all
    // of the following are true:
    // - ICH_LR<n>_EL2.State is 0b00 (ICH_LR_EL2_STATE_INVALID).
    // - ICH_LR<n>_EL2.HW is 0.
    // - ICH_LR<n>_EL2.EOI (bit [41]) is 1.

    uint64_t value = 0;

    for (int lr_idx = 0; lr_idx < VIRTUAL_NUM_LIST_REGS; lr_idx++) {
        ICH_LR_EL2 ich_lr_el2 =
            isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

        if ((ich_lr_el2.State == ICH_LR_EL2_STATE_INVALID) &&
            !ich_lr_el2.HW && ich_lr_el2.EOI) {
            value |= (1 << lr_idx);
        }
    }

    return value;
}

Gicv3CPUInterface::ICH_MISR_EL2
Gicv3CPUInterface::maintenanceInterruptStatus() const
{
    // Comments are copied from SPEC section 9.4.7 (ID012119)
    ICH_MISR_EL2 ich_misr_el2 = 0;
    ICH_HCR_EL2 ich_hcr_el2 =
        isa->readMiscRegNoEffect(MISCREG_ICH_HCR_EL2);
    ICH_VMCR_EL2 ich_vmcr_el2 =
        isa->readMiscRegNoEffect(MISCREG_ICH_VMCR_EL2);

    // End Of Interrupt. [bit 0]
    // This maintenance interrupt is asserted when at least one bit in
    // ICH_EISR_EL2 is 1.

    if (eoiMaintenanceInterruptStatus()) {
        ich_misr_el2.EOI = 1;
    }

    // Underflow. [bit 1]
    // This maintenance interrupt is asserted when ICH_HCR_EL2.UIE==1 and
    // zero or one of the List register entries are marked as a valid
    // interrupt, that is, if the corresponding ICH_LR<n>_EL2.State bits
    // do not equal 0x0.
    uint32_t num_valid_interrupts = 0;
    uint32_t num_pending_interrupts = 0;

    for (int lr_idx = 0; lr_idx < VIRTUAL_NUM_LIST_REGS; lr_idx++) {
        ICH_LR_EL2 ich_lr_el2 =
            isa->readMiscRegNoEffect(MISCREG_ICH_LR0_EL2 + lr_idx);

        if (ich_lr_el2.State != ICH_LR_EL2_STATE_INVALID) {
            num_valid_interrupts++;
        }

        if (ich_lr_el2.State == ICH_LR_EL2_STATE_PENDING) {
            num_pending_interrupts++;
        }
    }

    if (ich_hcr_el2.UIE && (num_valid_interrupts < 2)) {
        ich_misr_el2.U = 1;
    }

    // List Register Entry Not Present. [bit 2]
    // This maintenance interrupt is asserted when ICH_HCR_EL2.LRENPIE==1
    // and ICH_HCR_EL2.EOIcount is non-zero.
    if (ich_hcr_el2.LRENPIE && ich_hcr_el2.EOIcount) {
        ich_misr_el2.LRENP = 1;
    }

    // No Pending. [bit 3]
    // This maintenance interrupt is asserted when ICH_HCR_EL2.NPIE==1 and
    // no List register is in pending state.
    if (ich_hcr_el2.NPIE && (num_pending_interrupts == 0)) {
        ich_misr_el2.NP = 1;
    }

    // vPE Group 0 Enabled. [bit 4]
    // This maintenance interrupt is asserted when
    // ICH_HCR_EL2.VGrp0EIE==1 and ICH_VMCR_EL2.VENG0==1.
    if (ich_hcr_el2.VGrp0EIE && ich_vmcr_el2.VENG0) {
        ich_misr_el2.VGrp0E = 1;
    }

    // vPE Group 0 Disabled. [bit 5]
    // This maintenance interrupt is asserted when
    // ICH_HCR_EL2.VGrp0DIE==1 and ICH_VMCR_EL2.VENG0==0.
    if (ich_hcr_el2.VGrp0DIE && !ich_vmcr_el2.VENG0) {
        ich_misr_el2.VGrp0D = 1;
    }

    // vPE Group 1 Enabled. [bit 6]
    // This maintenance interrupt is asserted when
    // ICH_HCR_EL2.VGrp1EIE==1 and ICH_VMCR_EL2.VENG1==is 1.
    if (ich_hcr_el2.VGrp1EIE && ich_vmcr_el2.VENG1) {
        ich_misr_el2.VGrp1E = 1;
    }

    // vPE Group 1 Disabled. [bit 7]
    // This maintenance interrupt is asserted when
    // ICH_HCR_EL2.VGrp1DIE==1 and ICH_VMCR_EL2.VENG1==is 0.
    if (ich_hcr_el2.VGrp1DIE && !ich_vmcr_el2.VENG1) {
        ich_misr_el2.VGrp1D = 1;
    }

    return ich_misr_el2;
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
