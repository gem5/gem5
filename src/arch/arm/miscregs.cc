/*
 * Copyright (c) 2010-2013, 2015-2017 ARM Limited
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
 * Authors: Gabe Black
 *          Ali Saidi
 *          Giacomo Gabrielli
 */

#include "arch/arm/miscregs.hh"

#include <tuple>

#include "arch/arm/isa.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "sim/full_system.hh"

namespace ArmISA
{

MiscRegIndex
decodeCP14Reg(unsigned crn, unsigned opc1, unsigned crm, unsigned opc2)
{
    switch(crn) {
      case 0:
        switch (opc1) {
          case 0:
            switch (opc2) {
              case 0:
                switch (crm) {
                  case 0:
                    return MISCREG_DBGDIDR;
                  case 1:
                    return MISCREG_DBGDSCRint;
                }
                break;
            }
            break;
          case 7:
            switch (opc2) {
              case 0:
                switch (crm) {
                  case 0:
                    return MISCREG_JIDR;
                }
              break;
            }
            break;
        }
        break;
      case 1:
        switch (opc1) {
          case 6:
            switch (crm) {
              case 0:
                switch (opc2) {
                  case 0:
                    return MISCREG_TEEHBR;
                }
                break;
            }
            break;
          case 7:
            switch (crm) {
              case 0:
                switch (opc2) {
                  case 0:
                    return MISCREG_JOSCR;
                }
                break;
            }
            break;
        }
        break;
      case 2:
        switch (opc1) {
          case 7:
            switch (crm) {
              case 0:
                switch (opc2) {
                  case 0:
                    return MISCREG_JMCR;
                }
                break;
            }
            break;
        }
        break;
    }
    // If we get here then it must be a register that we haven't implemented
    warn("CP14 unimplemented crn[%d], opc1[%d], crm[%d], opc2[%d]",
         crn, opc1, crm, opc2);
    return MISCREG_CP14_UNIMPL;
}

using namespace std;

MiscRegIndex
decodeCP15Reg(unsigned crn, unsigned opc1, unsigned crm, unsigned opc2)
{
    switch (crn) {
      case 0:
        switch (opc1) {
          case 0:
            switch (crm) {
              case 0:
                switch (opc2) {
                  case 1:
                    return MISCREG_CTR;
                  case 2:
                    return MISCREG_TCMTR;
                  case 3:
                    return MISCREG_TLBTR;
                  case 5:
                    return MISCREG_MPIDR;
                  case 6:
                    return MISCREG_REVIDR;
                  default:
                    return MISCREG_MIDR;
                }
                break;
              case 1:
                switch (opc2) {
                  case 0:
                    return MISCREG_ID_PFR0;
                  case 1:
                    return MISCREG_ID_PFR1;
                  case 2:
                    return MISCREG_ID_DFR0;
                  case 3:
                    return MISCREG_ID_AFR0;
                  case 4:
                    return MISCREG_ID_MMFR0;
                  case 5:
                    return MISCREG_ID_MMFR1;
                  case 6:
                    return MISCREG_ID_MMFR2;
                  case 7:
                    return MISCREG_ID_MMFR3;
                }
                break;
              case 2:
                switch (opc2) {
                  case 0:
                    return MISCREG_ID_ISAR0;
                  case 1:
                    return MISCREG_ID_ISAR1;
                  case 2:
                    return MISCREG_ID_ISAR2;
                  case 3:
                    return MISCREG_ID_ISAR3;
                  case 4:
                    return MISCREG_ID_ISAR4;
                  case 5:
                    return MISCREG_ID_ISAR5;
                  case 6:
                  case 7:
                    return MISCREG_RAZ; // read as zero
                }
                break;
              default:
                return MISCREG_RAZ; // read as zero
            }
            break;
          case 1:
            if (crm == 0) {
                switch (opc2) {
                  case 0:
                    return MISCREG_CCSIDR;
                  case 1:
                    return MISCREG_CLIDR;
                  case 7:
                    return MISCREG_AIDR;
                }
            }
            break;
          case 2:
            if (crm == 0 && opc2 == 0) {
                return MISCREG_CSSELR;
            }
            break;
          case 4:
            if (crm == 0) {
                if (opc2 == 0)
                    return MISCREG_VPIDR;
                else if (opc2 == 5)
                    return MISCREG_VMPIDR;
            }
            break;
        }
        break;
      case 1:
        if (opc1 == 0) {
            if (crm == 0) {
                switch (opc2) {
                  case 0:
                    return MISCREG_SCTLR;
                  case 1:
                    return MISCREG_ACTLR;
                  case 0x2:
                    return MISCREG_CPACR;
                }
            } else if (crm == 1) {
                switch (opc2) {
                  case 0:
                    return MISCREG_SCR;
                  case 1:
                    return MISCREG_SDER;
                  case 2:
                    return MISCREG_NSACR;
                }
            }
        } else if (opc1 == 4) {
            if (crm == 0) {
                if (opc2 == 0)
                    return MISCREG_HSCTLR;
                else if (opc2 == 1)
                    return MISCREG_HACTLR;
            } else if (crm == 1) {
                switch (opc2) {
                  case 0:
                    return MISCREG_HCR;
                  case 1:
                    return MISCREG_HDCR;
                  case 2:
                    return MISCREG_HCPTR;
                  case 3:
                    return MISCREG_HSTR;
                  case 7:
                    return MISCREG_HACR;
                }
            }
        }
        break;
      case 2:
        if (opc1 == 0 && crm == 0) {
            switch (opc2) {
              case 0:
                return MISCREG_TTBR0;
              case 1:
                return MISCREG_TTBR1;
              case 2:
                return MISCREG_TTBCR;
            }
        } else if (opc1 == 4) {
            if (crm == 0 && opc2 == 2)
                return MISCREG_HTCR;
            else if (crm == 1 && opc2 == 2)
                return MISCREG_VTCR;
        }
        break;
      case 3:
        if (opc1 == 0 && crm == 0 && opc2 == 0) {
            return MISCREG_DACR;
        }
        break;
      case 5:
        if (opc1 == 0) {
            if (crm == 0) {
                if (opc2 == 0) {
                    return MISCREG_DFSR;
                } else if (opc2 == 1) {
                    return MISCREG_IFSR;
                }
            } else if (crm == 1) {
                if (opc2 == 0) {
                    return MISCREG_ADFSR;
                } else if (opc2 == 1) {
                    return MISCREG_AIFSR;
                }
            }
        } else if (opc1 == 4) {
            if (crm == 1) {
                if (opc2 == 0)
                    return MISCREG_HADFSR;
                else if (opc2 == 1)
                    return MISCREG_HAIFSR;
            } else if (crm == 2 && opc2 == 0) {
                return MISCREG_HSR;
            }
        }
        break;
      case 6:
        if (opc1 == 0 && crm == 0) {
            switch (opc2) {
              case 0:
                return MISCREG_DFAR;
              case 2:
                return MISCREG_IFAR;
            }
        } else if (opc1 == 4 && crm == 0) {
            switch (opc2) {
              case 0:
                return MISCREG_HDFAR;
              case 2:
                return MISCREG_HIFAR;
              case 4:
                return MISCREG_HPFAR;
            }
        }
        break;
      case 7:
        if (opc1 == 0) {
            switch (crm) {
              case 0:
                if (opc2 == 4) {
                    return MISCREG_NOP;
                }
                break;
              case 1:
                switch (opc2) {
                  case 0:
                    return MISCREG_ICIALLUIS;
                  case 6:
                    return MISCREG_BPIALLIS;
                }
                break;
              case 4:
                if (opc2 == 0) {
                    return MISCREG_PAR;
                }
                break;
              case 5:
                switch (opc2) {
                  case 0:
                    return MISCREG_ICIALLU;
                  case 1:
                    return MISCREG_ICIMVAU;
                  case 4:
                    return MISCREG_CP15ISB;
                  case 6:
                    return MISCREG_BPIALL;
                  case 7:
                    return MISCREG_BPIMVA;
                }
                break;
              case 6:
                if (opc2 == 1) {
                    return MISCREG_DCIMVAC;
                } else if (opc2 == 2) {
                    return MISCREG_DCISW;
                }
                break;
              case 8:
                switch (opc2) {
                  case 0:
                    return MISCREG_ATS1CPR;
                  case 1:
                    return MISCREG_ATS1CPW;
                  case 2:
                    return MISCREG_ATS1CUR;
                  case 3:
                    return MISCREG_ATS1CUW;
                  case 4:
                    return MISCREG_ATS12NSOPR;
                  case 5:
                    return MISCREG_ATS12NSOPW;
                  case 6:
                    return MISCREG_ATS12NSOUR;
                  case 7:
                    return MISCREG_ATS12NSOUW;
                }
                break;
              case 10:
                switch (opc2) {
                  case 1:
                    return MISCREG_DCCMVAC;
                  case 2:
                    return MISCREG_DCCSW;
                  case 4:
                    return MISCREG_CP15DSB;
                  case 5:
                    return MISCREG_CP15DMB;
                }
                break;
              case 11:
                if (opc2 == 1) {
                    return MISCREG_DCCMVAU;
                }
                break;
              case 13:
                if (opc2 == 1) {
                    return MISCREG_NOP;
                }
                break;
              case 14:
                if (opc2 == 1) {
                    return MISCREG_DCCIMVAC;
                } else if (opc2 == 2) {
                    return MISCREG_DCCISW;
                }
                break;
            }
        } else if (opc1 == 4 && crm == 8) {
            if (opc2 == 0)
                return MISCREG_ATS1HR;
            else if (opc2 == 1)
                return MISCREG_ATS1HW;
        }
        break;
      case 8:
        if (opc1 == 0) {
            switch (crm) {
              case 3:
                switch (opc2) {
                  case 0:
                    return MISCREG_TLBIALLIS;
                  case 1:
                    return MISCREG_TLBIMVAIS;
                  case 2:
                    return MISCREG_TLBIASIDIS;
                  case 3:
                    return MISCREG_TLBIMVAAIS;
                  case 5:
                    return MISCREG_TLBIMVALIS;
                  case 7:
                    return MISCREG_TLBIMVAALIS;
                }
                break;
              case 5:
                switch (opc2) {
                  case 0:
                    return MISCREG_ITLBIALL;
                  case 1:
                    return MISCREG_ITLBIMVA;
                  case 2:
                    return MISCREG_ITLBIASID;
                }
                break;
              case 6:
                switch (opc2) {
                  case 0:
                    return MISCREG_DTLBIALL;
                  case 1:
                    return MISCREG_DTLBIMVA;
                  case 2:
                    return MISCREG_DTLBIASID;
                }
                break;
              case 7:
                switch (opc2) {
                  case 0:
                    return MISCREG_TLBIALL;
                  case 1:
                    return MISCREG_TLBIMVA;
                  case 2:
                    return MISCREG_TLBIASID;
                  case 3:
                    return MISCREG_TLBIMVAA;
                  case 5:
                    return MISCREG_TLBIMVAL;
                  case 7:
                    return MISCREG_TLBIMVAAL;
                }
                break;
            }
        } else if (opc1 == 4) {
            if (crm == 0) {
                switch (opc2) {
                  case 1:
                    return MISCREG_TLBIIPAS2IS;
                  case 5:
                    return MISCREG_TLBIIPAS2LIS;
                }
            } else if (crm == 3) {
                switch (opc2) {
                  case 0:
                    return MISCREG_TLBIALLHIS;
                  case 1:
                    return MISCREG_TLBIMVAHIS;
                  case 4:
                    return MISCREG_TLBIALLNSNHIS;
                  case 5:
                    return MISCREG_TLBIMVALHIS;
                }
            } else if (crm == 4) {
                switch (opc2) {
                  case 1:
                    return MISCREG_TLBIIPAS2;
                  case 5:
                    return MISCREG_TLBIIPAS2L;
                }
            } else if (crm == 7) {
                switch (opc2) {
                  case 0:
                    return MISCREG_TLBIALLH;
                  case 1:
                    return MISCREG_TLBIMVAH;
                  case 4:
                    return MISCREG_TLBIALLNSNH;
                  case 5:
                    return MISCREG_TLBIMVALH;
                }
            }
        }
        break;
      case 9:
        // Every cop register with CRn = 9 and CRm in
        // {0-2}, {5-8} is implementation defined regardless
        // of opc1 and opc2.
        switch (crm) {
          case 0:
          case 1:
          case 2:
          case 5:
          case 6:
          case 7:
          case 8:
            return MISCREG_IMPDEF_UNIMPL;
        }
        if (opc1 == 0) {
            switch (crm) {
              case 12:
                switch (opc2) {
                  case 0:
                    return MISCREG_PMCR;
                  case 1:
                    return MISCREG_PMCNTENSET;
                  case 2:
                    return MISCREG_PMCNTENCLR;
                  case 3:
                    return MISCREG_PMOVSR;
                  case 4:
                    return MISCREG_PMSWINC;
                  case 5:
                    return MISCREG_PMSELR;
                  case 6:
                    return MISCREG_PMCEID0;
                  case 7:
                    return MISCREG_PMCEID1;
                }
                break;
              case 13:
                switch (opc2) {
                  case 0:
                    return MISCREG_PMCCNTR;
                  case 1:
                    // Selector is PMSELR.SEL
                    return MISCREG_PMXEVTYPER_PMCCFILTR;
                  case 2:
                    return MISCREG_PMXEVCNTR;
                }
                break;
              case 14:
                switch (opc2) {
                  case 0:
                    return MISCREG_PMUSERENR;
                  case 1:
                    return MISCREG_PMINTENSET;
                  case 2:
                    return MISCREG_PMINTENCLR;
                  case 3:
                    return MISCREG_PMOVSSET;
                }
                break;
            }
        } else if (opc1 == 1) {
            switch (crm) {
              case 0:
                switch (opc2) {
                  case 2: // L2CTLR, L2 Control Register
                    return MISCREG_L2CTLR;
                  case 3:
                    return MISCREG_L2ECTLR;
                }
                break;
                break;
            }
        }
        break;
      case 10:
        if (opc1 == 0) {
            // crm 0, 1, 4, and 8, with op2 0 - 7, reserved for TLB lockdown
            if (crm < 2) {
                return MISCREG_IMPDEF_UNIMPL;
            } else if (crm == 2) { // TEX Remap Registers
                if (opc2 == 0) {
                    // Selector is TTBCR.EAE
                    return MISCREG_PRRR_MAIR0;
                } else if (opc2 == 1) {
                    // Selector is TTBCR.EAE
                    return MISCREG_NMRR_MAIR1;
                }
            } else if (crm == 3) {
                if (opc2 == 0) {
                    return MISCREG_AMAIR0;
                } else if (opc2 == 1) {
                    return MISCREG_AMAIR1;
                }
            }
        } else if (opc1 == 4) {
            // crm 0, 1, 4, and 8, with op2 0 - 7, reserved for TLB lockdown
            if (crm == 2) {
                if (opc2 == 0)
                    return MISCREG_HMAIR0;
                else if (opc2 == 1)
                    return MISCREG_HMAIR1;
            } else if (crm == 3) {
                if (opc2 == 0)
                    return MISCREG_HAMAIR0;
                else if (opc2 == 1)
                    return MISCREG_HAMAIR1;
            }
        }
        break;
      case 11:
        if (opc1 <=7) {
            switch (crm) {
              case 0:
              case 1:
              case 2:
              case 3:
              case 4:
              case 5:
              case 6:
              case 7:
              case 8:
              case 15:
                // Reserved for DMA operations for TCM access
                return MISCREG_IMPDEF_UNIMPL;
              default:
                break;
            }
        }
        break;
      case 12:
        if (opc1 == 0) {
            if (crm == 0) {
                if (opc2 == 0) {
                    return MISCREG_VBAR;
                } else if (opc2 == 1) {
                    return MISCREG_MVBAR;
                }
            } else if (crm == 1) {
                if (opc2 == 0) {
                    return MISCREG_ISR;
                }
            }
        } else if (opc1 == 4) {
            if (crm == 0 && opc2 == 0)
                return MISCREG_HVBAR;
        }
        break;
      case 13:
        if (opc1 == 0) {
            if (crm == 0) {
                switch (opc2) {
                  case 0:
                    return MISCREG_FCSEIDR;
                  case 1:
                    return MISCREG_CONTEXTIDR;
                  case 2:
                    return MISCREG_TPIDRURW;
                  case 3:
                    return MISCREG_TPIDRURO;
                  case 4:
                    return MISCREG_TPIDRPRW;
                }
            }
        } else if (opc1 == 4) {
            if (crm == 0 && opc2 == 2)
                return MISCREG_HTPIDR;
        }
        break;
      case 14:
        if (opc1 == 0) {
            switch (crm) {
              case 0:
                if (opc2 == 0)
                    return MISCREG_CNTFRQ;
                break;
              case 1:
                if (opc2 == 0)
                    return MISCREG_CNTKCTL;
                break;
              case 2:
                if (opc2 == 0)
                    return MISCREG_CNTP_TVAL;
                else if (opc2 == 1)
                    return MISCREG_CNTP_CTL;
                break;
              case 3:
                if (opc2 == 0)
                    return MISCREG_CNTV_TVAL;
                else if (opc2 == 1)
                    return MISCREG_CNTV_CTL;
                break;
            }
        } else if (opc1 == 4) {
            if (crm == 1 && opc2 == 0) {
                return MISCREG_CNTHCTL;
            } else if (crm == 2) {
                if (opc2 == 0)
                    return MISCREG_CNTHP_TVAL;
                else if (opc2 == 1)
                    return MISCREG_CNTHP_CTL;
            }
        }
        break;
      case 15:
        // Implementation defined
        return MISCREG_IMPDEF_UNIMPL;
    }
    // Unrecognized register
    return MISCREG_CP15_UNIMPL;
}

MiscRegIndex
decodeCP15Reg64(unsigned crm, unsigned opc1)
{
    switch (crm) {
      case 2:
        switch (opc1) {
          case 0:
            return MISCREG_TTBR0;
          case 1:
            return MISCREG_TTBR1;
          case 4:
            return MISCREG_HTTBR;
          case 6:
            return MISCREG_VTTBR;
        }
        break;
      case 7:
        if (opc1 == 0)
            return MISCREG_PAR;
        break;
      case 14:
        switch (opc1) {
          case 0:
            return MISCREG_CNTPCT;
          case 1:
            return MISCREG_CNTVCT;
          case 2:
            return MISCREG_CNTP_CVAL;
          case 3:
            return MISCREG_CNTV_CVAL;
          case 4:
            return MISCREG_CNTVOFF;
          case 6:
            return MISCREG_CNTHP_CVAL;
        }
        break;
      case 15:
        if (opc1 == 0)
            return MISCREG_CPUMERRSR;
        else if (opc1 == 1)
            return MISCREG_L2MERRSR;
        break;
    }
    // Unrecognized register
    return MISCREG_CP15_UNIMPL;
}

std::tuple<bool, bool>
canReadCoprocReg(MiscRegIndex reg, SCR scr, CPSR cpsr)
{
    bool secure = !scr.ns;
    bool canRead = false;
    bool undefined = false;

    switch (cpsr.mode) {
      case MODE_USER:
        canRead = secure ? miscRegInfo[reg][MISCREG_USR_S_RD] :
                           miscRegInfo[reg][MISCREG_USR_NS_RD];
        break;
      case MODE_FIQ:
      case MODE_IRQ:
      case MODE_SVC:
      case MODE_ABORT:
      case MODE_UNDEFINED:
      case MODE_SYSTEM:
        canRead = secure ? miscRegInfo[reg][MISCREG_PRI_S_RD] :
                           miscRegInfo[reg][MISCREG_PRI_NS_RD];
        break;
      case MODE_MON:
        canRead = secure ? miscRegInfo[reg][MISCREG_MON_NS0_RD] :
                           miscRegInfo[reg][MISCREG_MON_NS1_RD];
        break;
      case MODE_HYP:
        canRead = miscRegInfo[reg][MISCREG_HYP_RD];
        break;
      default:
        undefined = true;
    }
    // can't do permissions checkes on the root of a banked pair of regs
    assert(!miscRegInfo[reg][MISCREG_BANKED]);
    return std::make_tuple(canRead, undefined);
}

std::tuple<bool, bool>
canWriteCoprocReg(MiscRegIndex reg, SCR scr, CPSR cpsr)
{
    bool secure = !scr.ns;
    bool canWrite = false;
    bool undefined = false;

    switch (cpsr.mode) {
      case MODE_USER:
        canWrite = secure ? miscRegInfo[reg][MISCREG_USR_S_WR] :
                            miscRegInfo[reg][MISCREG_USR_NS_WR];
        break;
      case MODE_FIQ:
      case MODE_IRQ:
      case MODE_SVC:
      case MODE_ABORT:
      case MODE_UNDEFINED:
      case MODE_SYSTEM:
        canWrite = secure ? miscRegInfo[reg][MISCREG_PRI_S_WR] :
                            miscRegInfo[reg][MISCREG_PRI_NS_WR];
        break;
      case MODE_MON:
        canWrite = secure ? miscRegInfo[reg][MISCREG_MON_NS0_WR] :
                            miscRegInfo[reg][MISCREG_MON_NS1_WR];
        break;
      case MODE_HYP:
        canWrite =  miscRegInfo[reg][MISCREG_HYP_WR];
        break;
      default:
        undefined = true;
    }
    // can't do permissions checkes on the root of a banked pair of regs
    assert(!miscRegInfo[reg][MISCREG_BANKED]);
    return std::make_tuple(canWrite, undefined);
}

int
snsBankedIndex(MiscRegIndex reg, ThreadContext *tc)
{
    SCR scr = tc->readMiscReg(MISCREG_SCR);
    return snsBankedIndex(reg, tc, scr.ns);
}

int
snsBankedIndex(MiscRegIndex reg, ThreadContext *tc, bool ns)
{
    int reg_as_int = static_cast<int>(reg);
    if (miscRegInfo[reg][MISCREG_BANKED]) {
        reg_as_int += (ArmSystem::haveSecurity(tc) &&
                      !ArmSystem::highestELIs64(tc) && !ns) ? 2 : 1;
    }
    return reg_as_int;
}


/**
 * If the reg is a child reg of a banked set, then the parent is the last
 * banked one in the list. This is messy, and the wish is to eventually have
 * the bitmap replaced with a better data structure. the preUnflatten function
 * initializes a lookup table to speed up the search for these banked
 * registers.
 */

int unflattenResultMiscReg[NUM_MISCREGS];

void
preUnflattenMiscReg()
{
    int reg = -1;
    for (int i = 0 ; i < NUM_MISCREGS; i++){
        if (miscRegInfo[i][MISCREG_BANKED])
            reg = i;
        if (miscRegInfo[i][MISCREG_BANKED_CHILD])
            unflattenResultMiscReg[i] = reg;
        else
            unflattenResultMiscReg[i] = i;
        // if this assert fails, no parent was found, and something is broken
        assert(unflattenResultMiscReg[i] > -1);
    }
}

int
unflattenMiscReg(int reg)
{
    return unflattenResultMiscReg[reg];
}

bool
canReadAArch64SysReg(MiscRegIndex reg, SCR scr, CPSR cpsr, ThreadContext *tc)
{
    // Check for SP_EL0 access while SPSEL == 0
    if ((reg == MISCREG_SP_EL0) && (tc->readMiscReg(MISCREG_SPSEL) == 0))
        return false;

    // Check for RVBAR access
    if (reg == MISCREG_RVBAR_EL1) {
        ExceptionLevel highest_el = ArmSystem::highestEL(tc);
        if (highest_el == EL2 || highest_el == EL3)
            return false;
    }
    if (reg == MISCREG_RVBAR_EL2) {
        ExceptionLevel highest_el = ArmSystem::highestEL(tc);
        if (highest_el == EL3)
            return false;
    }

    bool secure = ArmSystem::haveSecurity(tc) && !scr.ns;

    switch (opModeToEL((OperatingMode) (uint8_t) cpsr.mode)) {
      case EL0:
        return secure ? miscRegInfo[reg][MISCREG_USR_S_RD] :
            miscRegInfo[reg][MISCREG_USR_NS_RD];
      case EL1:
        return secure ? miscRegInfo[reg][MISCREG_PRI_S_RD] :
            miscRegInfo[reg][MISCREG_PRI_NS_RD];
      case EL2:
        return miscRegInfo[reg][MISCREG_HYP_RD];
      case EL3:
        return secure ? miscRegInfo[reg][MISCREG_MON_NS0_RD] :
            miscRegInfo[reg][MISCREG_MON_NS1_RD];
      default:
        panic("Invalid exception level");
    }
}

bool
canWriteAArch64SysReg(MiscRegIndex reg, SCR scr, CPSR cpsr, ThreadContext *tc)
{
    // Check for SP_EL0 access while SPSEL == 0
    if ((reg == MISCREG_SP_EL0) && (tc->readMiscReg(MISCREG_SPSEL) == 0))
        return false;
    ExceptionLevel el = opModeToEL((OperatingMode) (uint8_t) cpsr.mode);
    if (reg == MISCREG_DAIF) {
        SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
        if (el == EL0 && !sctlr.uma)
            return false;
    }
    if (FullSystem && reg == MISCREG_DC_ZVA_Xt) {
        // In syscall-emulation mode, this test is skipped and DCZVA is always
        // allowed at EL0
        SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
        if (el == EL0 && !sctlr.dze)
            return false;
    }
    if (reg == MISCREG_DC_CVAC_Xt || reg == MISCREG_DC_CIVAC_Xt) {
        SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR_EL1);
        if (el == EL0 && !sctlr.uci)
            return false;
    }

    bool secure = ArmSystem::haveSecurity(tc) && !scr.ns;

    switch (el) {
      case EL0:
        return secure ? miscRegInfo[reg][MISCREG_USR_S_WR] :
            miscRegInfo[reg][MISCREG_USR_NS_WR];
      case EL1:
        return secure ? miscRegInfo[reg][MISCREG_PRI_S_WR] :
            miscRegInfo[reg][MISCREG_PRI_NS_WR];
      case EL2:
        return miscRegInfo[reg][MISCREG_HYP_WR];
      case EL3:
        return secure ? miscRegInfo[reg][MISCREG_MON_NS0_WR] :
            miscRegInfo[reg][MISCREG_MON_NS1_WR];
      default:
        panic("Invalid exception level");
    }
}

MiscRegIndex
decodeAArch64SysReg(unsigned op0, unsigned op1,
                    unsigned crn, unsigned crm,
                    unsigned op2)
{
    switch (op0) {
      case 1:
        switch (crn) {
          case 7:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_IC_IALLUIS;
                    }
                    break;
                  case 5:
                    switch (op2) {
                      case 0:
                        return MISCREG_IC_IALLU;
                    }
                    break;
                  case 6:
                    switch (op2) {
                      case 1:
                        return MISCREG_DC_IVAC_Xt;
                      case 2:
                        return MISCREG_DC_ISW_Xt;
                    }
                    break;
                  case 8:
                    switch (op2) {
                      case 0:
                        return MISCREG_AT_S1E1R_Xt;
                      case 1:
                        return MISCREG_AT_S1E1W_Xt;
                      case 2:
                        return MISCREG_AT_S1E0R_Xt;
                      case 3:
                        return MISCREG_AT_S1E0W_Xt;
                    }
                    break;
                  case 10:
                    switch (op2) {
                      case 2:
                        return MISCREG_DC_CSW_Xt;
                    }
                    break;
                  case 14:
                    switch (op2) {
                      case 2:
                        return MISCREG_DC_CISW_Xt;
                    }
                    break;
                }
                break;
              case 3:
                switch (crm) {
                  case 4:
                    switch (op2) {
                      case 1:
                        return MISCREG_DC_ZVA_Xt;
                    }
                    break;
                  case 5:
                    switch (op2) {
                      case 1:
                        return MISCREG_IC_IVAU_Xt;
                    }
                    break;
                  case 10:
                    switch (op2) {
                      case 1:
                        return MISCREG_DC_CVAC_Xt;
                    }
                    break;
                  case 11:
                    switch (op2) {
                      case 1:
                        return MISCREG_DC_CVAU_Xt;
                    }
                    break;
                  case 14:
                    switch (op2) {
                      case 1:
                        return MISCREG_DC_CIVAC_Xt;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 8:
                    switch (op2) {
                      case 0:
                        return MISCREG_AT_S1E2R_Xt;
                      case 1:
                        return MISCREG_AT_S1E2W_Xt;
                      case 4:
                        return MISCREG_AT_S12E1R_Xt;
                      case 5:
                        return MISCREG_AT_S12E1W_Xt;
                      case 6:
                        return MISCREG_AT_S12E0R_Xt;
                      case 7:
                        return MISCREG_AT_S12E0W_Xt;
                    }
                    break;
                }
                break;
              case 6:
                switch (crm) {
                  case 8:
                    switch (op2) {
                      case 0:
                        return MISCREG_AT_S1E3R_Xt;
                      case 1:
                        return MISCREG_AT_S1E3W_Xt;
                    }
                    break;
                }
                break;
            }
            break;
          case 8:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_TLBI_VMALLE1IS;
                      case 1:
                        return MISCREG_TLBI_VAE1IS_Xt;
                      case 2:
                        return MISCREG_TLBI_ASIDE1IS_Xt;
                      case 3:
                        return MISCREG_TLBI_VAAE1IS_Xt;
                      case 5:
                        return MISCREG_TLBI_VALE1IS_Xt;
                      case 7:
                        return MISCREG_TLBI_VAALE1IS_Xt;
                    }
                    break;
                  case 7:
                    switch (op2) {
                      case 0:
                        return MISCREG_TLBI_VMALLE1;
                      case 1:
                        return MISCREG_TLBI_VAE1_Xt;
                      case 2:
                        return MISCREG_TLBI_ASIDE1_Xt;
                      case 3:
                        return MISCREG_TLBI_VAAE1_Xt;
                      case 5:
                        return MISCREG_TLBI_VALE1_Xt;
                      case 7:
                        return MISCREG_TLBI_VAALE1_Xt;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 1:
                        return MISCREG_TLBI_IPAS2E1IS_Xt;
                      case 5:
                        return MISCREG_TLBI_IPAS2LE1IS_Xt;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_TLBI_ALLE2IS;
                      case 1:
                        return MISCREG_TLBI_VAE2IS_Xt;
                      case 4:
                        return MISCREG_TLBI_ALLE1IS;
                      case 5:
                        return MISCREG_TLBI_VALE2IS_Xt;
                      case 6:
                        return MISCREG_TLBI_VMALLS12E1IS;
                    }
                    break;
                  case 4:
                    switch (op2) {
                      case 1:
                        return MISCREG_TLBI_IPAS2E1_Xt;
                      case 5:
                        return MISCREG_TLBI_IPAS2LE1_Xt;
                    }
                    break;
                  case 7:
                    switch (op2) {
                      case 0:
                        return MISCREG_TLBI_ALLE2;
                      case 1:
                        return MISCREG_TLBI_VAE2_Xt;
                      case 4:
                        return MISCREG_TLBI_ALLE1;
                      case 5:
                        return MISCREG_TLBI_VALE2_Xt;
                      case 6:
                        return MISCREG_TLBI_VMALLS12E1;
                    }
                    break;
                }
                break;
              case 6:
                switch (crm) {
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_TLBI_ALLE3IS;
                      case 1:
                        return MISCREG_TLBI_VAE3IS_Xt;
                      case 5:
                        return MISCREG_TLBI_VALE3IS_Xt;
                    }
                    break;
                  case 7:
                    switch (op2) {
                      case 0:
                        return MISCREG_TLBI_ALLE3;
                      case 1:
                        return MISCREG_TLBI_VAE3_Xt;
                      case 5:
                        return MISCREG_TLBI_VALE3_Xt;
                    }
                    break;
                }
                break;
            }
            break;
        }
        break;
      case 2:
        switch (crn) {
          case 0:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 2:
                        return MISCREG_OSDTRRX_EL1;
                      case 4:
                        return MISCREG_DBGBVR0_EL1;
                      case 5:
                        return MISCREG_DBGBCR0_EL1;
                      case 6:
                        return MISCREG_DBGWVR0_EL1;
                      case 7:
                        return MISCREG_DBGWCR0_EL1;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR1_EL1;
                      case 5:
                        return MISCREG_DBGBCR1_EL1;
                      case 6:
                        return MISCREG_DBGWVR1_EL1;
                      case 7:
                        return MISCREG_DBGWCR1_EL1;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_MDCCINT_EL1;
                      case 2:
                        return MISCREG_MDSCR_EL1;
                      case 4:
                        return MISCREG_DBGBVR2_EL1;
                      case 5:
                        return MISCREG_DBGBCR2_EL1;
                      case 6:
                        return MISCREG_DBGWVR2_EL1;
                      case 7:
                        return MISCREG_DBGWCR2_EL1;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 2:
                        return MISCREG_OSDTRTX_EL1;
                      case 4:
                        return MISCREG_DBGBVR3_EL1;
                      case 5:
                        return MISCREG_DBGBCR3_EL1;
                      case 6:
                        return MISCREG_DBGWVR3_EL1;
                      case 7:
                        return MISCREG_DBGWCR3_EL1;
                    }
                    break;
                  case 4:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR4_EL1;
                      case 5:
                        return MISCREG_DBGBCR4_EL1;
                    }
                    break;
                  case 5:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR5_EL1;
                      case 5:
                        return MISCREG_DBGBCR5_EL1;
                    }
                    break;
                  case 6:
                    switch (op2) {
                      case 2:
                        return MISCREG_OSECCR_EL1;
                    }
                    break;
                }
                break;
              case 2:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_TEECR32_EL1;
                    }
                    break;
                }
                break;
              case 3:
                switch (crm) {
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_MDCCSR_EL0;
                    }
                    break;
                  case 4:
                    switch (op2) {
                      case 0:
                        return MISCREG_MDDTR_EL0;
                    }
                    break;
                  case 5:
                    switch (op2) {
                      case 0:
                        return MISCREG_MDDTRRX_EL0;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 7:
                    switch (op2) {
                      case 0:
                        return MISCREG_DBGVCR32_EL2;
                    }
                    break;
                }
                break;
            }
            break;
          case 1:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_MDRAR_EL1;
                      case 4:
                        return MISCREG_OSLAR_EL1;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 4:
                        return MISCREG_OSLSR_EL1;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 4:
                        return MISCREG_OSDLR_EL1;
                    }
                    break;
                  case 4:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGPRCR_EL1;
                    }
                    break;
                }
                break;
              case 2:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_TEEHBR32_EL1;
                    }
                    break;
                }
                break;
            }
            break;
          case 7:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 8:
                    switch (op2) {
                      case 6:
                        return MISCREG_DBGCLAIMSET_EL1;
                    }
                    break;
                  case 9:
                    switch (op2) {
                      case 6:
                        return MISCREG_DBGCLAIMCLR_EL1;
                    }
                    break;
                  case 14:
                    switch (op2) {
                      case 6:
                        return MISCREG_DBGAUTHSTATUS_EL1;
                    }
                    break;
                }
                break;
            }
            break;
        }
        break;
      case 3:
        switch (crn) {
          case 0:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_MIDR_EL1;
                      case 5:
                        return MISCREG_MPIDR_EL1;
                      case 6:
                        return MISCREG_REVIDR_EL1;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_ID_PFR0_EL1;
                      case 1:
                        return MISCREG_ID_PFR1_EL1;
                      case 2:
                        return MISCREG_ID_DFR0_EL1;
                      case 3:
                        return MISCREG_ID_AFR0_EL1;
                      case 4:
                        return MISCREG_ID_MMFR0_EL1;
                      case 5:
                        return MISCREG_ID_MMFR1_EL1;
                      case 6:
                        return MISCREG_ID_MMFR2_EL1;
                      case 7:
                        return MISCREG_ID_MMFR3_EL1;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_ID_ISAR0_EL1;
                      case 1:
                        return MISCREG_ID_ISAR1_EL1;
                      case 2:
                        return MISCREG_ID_ISAR2_EL1;
                      case 3:
                        return MISCREG_ID_ISAR3_EL1;
                      case 4:
                        return MISCREG_ID_ISAR4_EL1;
                      case 5:
                        return MISCREG_ID_ISAR5_EL1;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_MVFR0_EL1;
                      case 1:
                        return MISCREG_MVFR1_EL1;
                      case 2:
                        return MISCREG_MVFR2_EL1;
                      case 3 ... 7:
                        return MISCREG_RAZ;
                    }
                    break;
                  case 4:
                    switch (op2) {
                      case 0:
                        return MISCREG_ID_AA64PFR0_EL1;
                      case 1:
                        return MISCREG_ID_AA64PFR1_EL1;
                      case 2 ... 7:
                        return MISCREG_RAZ;
                    }
                    break;
                  case 5:
                    switch (op2) {
                      case 0:
                        return MISCREG_ID_AA64DFR0_EL1;
                      case 1:
                        return MISCREG_ID_AA64DFR1_EL1;
                      case 4:
                        return MISCREG_ID_AA64AFR0_EL1;
                      case 5:
                        return MISCREG_ID_AA64AFR1_EL1;
                      case 2:
                      case 3:
                      case 6:
                      case 7:
                        return MISCREG_RAZ;
                    }
                    break;
                  case 6:
                    switch (op2) {
                      case 0:
                        return MISCREG_ID_AA64ISAR0_EL1;
                      case 1:
                        return MISCREG_ID_AA64ISAR1_EL1;
                      case 2 ... 7:
                        return MISCREG_RAZ;
                    }
                    break;
                  case 7:
                    switch (op2) {
                      case 0:
                        return MISCREG_ID_AA64MMFR0_EL1;
                      case 1:
                        return MISCREG_ID_AA64MMFR1_EL1;
                      case 2 ... 7:
                        return MISCREG_RAZ;
                    }
                    break;
                }
                break;
              case 1:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_CCSIDR_EL1;
                      case 1:
                        return MISCREG_CLIDR_EL1;
                      case 7:
                        return MISCREG_AIDR_EL1;
                    }
                    break;
                }
                break;
              case 2:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_CSSELR_EL1;
                    }
                    break;
                }
                break;
              case 3:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 1:
                        return MISCREG_CTR_EL0;
                      case 7:
                        return MISCREG_DCZID_EL0;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_VPIDR_EL2;
                      case 5:
                        return MISCREG_VMPIDR_EL2;
                    }
                    break;
                }
                break;
            }
            break;
          case 1:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_SCTLR_EL1;
                      case 1:
                        return MISCREG_ACTLR_EL1;
                      case 2:
                        return MISCREG_CPACR_EL1;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_SCTLR_EL2;
                      case 1:
                        return MISCREG_ACTLR_EL2;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_HCR_EL2;
                      case 1:
                        return MISCREG_MDCR_EL2;
                      case 2:
                        return MISCREG_CPTR_EL2;
                      case 3:
                        return MISCREG_HSTR_EL2;
                      case 7:
                        return MISCREG_HACR_EL2;
                    }
                    break;
                }
                break;
              case 6:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_SCTLR_EL3;
                      case 1:
                        return MISCREG_ACTLR_EL3;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_SCR_EL3;
                      case 1:
                        return MISCREG_SDER32_EL3;
                      case 2:
                        return MISCREG_CPTR_EL3;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 1:
                        return MISCREG_MDCR_EL3;
                    }
                    break;
                }
                break;
            }
            break;
          case 2:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_TTBR0_EL1;
                      case 1:
                        return MISCREG_TTBR1_EL1;
                      case 2:
                        return MISCREG_TCR_EL1;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_TTBR0_EL2;
                      case 2:
                        return MISCREG_TCR_EL2;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_VTTBR_EL2;
                      case 2:
                        return MISCREG_VTCR_EL2;
                    }
                    break;
                }
                break;
              case 6:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_TTBR0_EL3;
                      case 2:
                        return MISCREG_TCR_EL3;
                    }
                    break;
                }
                break;
            }
            break;
          case 3:
            switch (op1) {
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_DACR32_EL2;
                    }
                    break;
                }
                break;
            }
            break;
          case 4:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_SPSR_EL1;
                      case 1:
                        return MISCREG_ELR_EL1;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_SP_EL0;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_SPSEL;
                      case 2:
                        return MISCREG_CURRENTEL;
                    }
                    break;
                }
                break;
              case 3:
                switch (crm) {
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_NZCV;
                      case 1:
                        return MISCREG_DAIF;
                    }
                    break;
                  case 4:
                    switch (op2) {
                      case 0:
                        return MISCREG_FPCR;
                      case 1:
                        return MISCREG_FPSR;
                    }
                    break;
                  case 5:
                    switch (op2) {
                      case 0:
                        return MISCREG_DSPSR_EL0;
                      case 1:
                        return MISCREG_DLR_EL0;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_SPSR_EL2;
                      case 1:
                        return MISCREG_ELR_EL2;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_SP_EL1;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_SPSR_IRQ_AA64;
                      case 1:
                        return MISCREG_SPSR_ABT_AA64;
                      case 2:
                        return MISCREG_SPSR_UND_AA64;
                      case 3:
                        return MISCREG_SPSR_FIQ_AA64;
                    }
                    break;
                }
                break;
              case 6:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_SPSR_EL3;
                      case 1:
                        return MISCREG_ELR_EL3;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_SP_EL2;
                    }
                    break;
                }
                break;
            }
            break;
          case 5:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_AFSR0_EL1;
                      case 1:
                        return MISCREG_AFSR1_EL1;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_ESR_EL1;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 1:
                        return MISCREG_IFSR32_EL2;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_AFSR0_EL2;
                      case 1:
                        return MISCREG_AFSR1_EL2;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_ESR_EL2;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_FPEXC32_EL2;
                    }
                    break;
                }
                break;
              case 6:
                switch (crm) {
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_AFSR0_EL3;
                      case 1:
                        return MISCREG_AFSR1_EL3;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_ESR_EL3;
                    }
                    break;
                }
                break;
            }
            break;
          case 6:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_FAR_EL1;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_FAR_EL2;
                      case 4:
                        return MISCREG_HPFAR_EL2;
                    }
                    break;
                }
                break;
              case 6:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_FAR_EL3;
                    }
                    break;
                }
                break;
            }
            break;
          case 7:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 4:
                    switch (op2) {
                      case 0:
                        return MISCREG_PAR_EL1;
                    }
                    break;
                }
                break;
            }
            break;
          case 9:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 14:
                    switch (op2) {
                      case 1:
                        return MISCREG_PMINTENSET_EL1;
                      case 2:
                        return MISCREG_PMINTENCLR_EL1;
                    }
                    break;
                }
                break;
              case 3:
                switch (crm) {
                  case 12:
                    switch (op2) {
                      case 0:
                        return MISCREG_PMCR_EL0;
                      case 1:
                        return MISCREG_PMCNTENSET_EL0;
                      case 2:
                        return MISCREG_PMCNTENCLR_EL0;
                      case 3:
                        return MISCREG_PMOVSCLR_EL0;
                      case 4:
                        return MISCREG_PMSWINC_EL0;
                      case 5:
                        return MISCREG_PMSELR_EL0;
                      case 6:
                        return MISCREG_PMCEID0_EL0;
                      case 7:
                        return MISCREG_PMCEID1_EL0;
                    }
                    break;
                  case 13:
                    switch (op2) {
                      case 0:
                        return MISCREG_PMCCNTR_EL0;
                      case 1:
                        return MISCREG_PMXEVTYPER_EL0;
                      case 2:
                        return MISCREG_PMXEVCNTR_EL0;
                    }
                    break;
                  case 14:
                    switch (op2) {
                      case 0:
                        return MISCREG_PMUSERENR_EL0;
                      case 3:
                        return MISCREG_PMOVSSET_EL0;
                    }
                    break;
                }
                break;
            }
            break;
          case 10:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_MAIR_EL1;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_AMAIR_EL1;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_MAIR_EL2;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_AMAIR_EL2;
                    }
                    break;
                }
                break;
              case 6:
                switch (crm) {
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_MAIR_EL3;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_AMAIR_EL3;
                    }
                    break;
                }
                break;
            }
            break;
          case 11:
            switch (op1) {
              case 1:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 2:
                        return MISCREG_L2CTLR_EL1;
                      case 3:
                        return MISCREG_L2ECTLR_EL1;
                    }
                    break;
                }
                break;
            }
            break;
          case 12:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_VBAR_EL1;
                      case 1:
                        return MISCREG_RVBAR_EL1;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_ISR_EL1;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_VBAR_EL2;
                      case 1:
                        return MISCREG_RVBAR_EL2;
                    }
                    break;
                }
                break;
              case 6:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_VBAR_EL3;
                      case 1:
                        return MISCREG_RVBAR_EL3;
                      case 2:
                        return MISCREG_RMR_EL3;
                    }
                    break;
                }
                break;
            }
            break;
          case 13:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 1:
                        return MISCREG_CONTEXTIDR_EL1;
                      case 4:
                        return MISCREG_TPIDR_EL1;
                    }
                    break;
                }
                break;
              case 3:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 2:
                        return MISCREG_TPIDR_EL0;
                      case 3:
                        return MISCREG_TPIDRRO_EL0;
                    }
                    break;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 1:
                        return MISCREG_CONTEXTIDR_EL2;
                      case 2:
                        return MISCREG_TPIDR_EL2;
                    }
                    break;
                }
                break;
              case 6:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 2:
                        return MISCREG_TPIDR_EL3;
                    }
                    break;
                }
                break;
            }
            break;
          case 14:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTKCTL_EL1;
                    }
                    break;
                }
                break;
              case 3:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTFRQ_EL0;
                      case 1:
                        return MISCREG_CNTPCT_EL0;
                      case 2:
                        return MISCREG_CNTVCT_EL0;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTP_TVAL_EL0;
                      case 1:
                        return MISCREG_CNTP_CTL_EL0;
                      case 2:
                        return MISCREG_CNTP_CVAL_EL0;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTV_TVAL_EL0;
                      case 1:
                        return MISCREG_CNTV_CTL_EL0;
                      case 2:
                        return MISCREG_CNTV_CVAL_EL0;
                    }
                    break;
                  case 8:
                    switch (op2) {
                      case 0:
                        return MISCREG_PMEVCNTR0_EL0;
                      case 1:
                        return MISCREG_PMEVCNTR1_EL0;
                      case 2:
                        return MISCREG_PMEVCNTR2_EL0;
                      case 3:
                        return MISCREG_PMEVCNTR3_EL0;
                      case 4:
                        return MISCREG_PMEVCNTR4_EL0;
                      case 5:
                        return MISCREG_PMEVCNTR5_EL0;
                    }
                    break;
                  case 12:
                    switch (op2) {
                      case 0:
                        return MISCREG_PMEVTYPER0_EL0;
                      case 1:
                        return MISCREG_PMEVTYPER1_EL0;
                      case 2:
                        return MISCREG_PMEVTYPER2_EL0;
                      case 3:
                        return MISCREG_PMEVTYPER3_EL0;
                      case 4:
                        return MISCREG_PMEVTYPER4_EL0;
                      case 5:
                        return MISCREG_PMEVTYPER5_EL0;
                    }
                    break;
                  case 15:
                    switch (op2) {
                      case 7:
                        return MISCREG_PMCCFILTR_EL0;
                    }
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 3:
                        return MISCREG_CNTVOFF_EL2;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTHCTL_EL2;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTHP_TVAL_EL2;
                      case 1:
                        return MISCREG_CNTHP_CTL_EL2;
                      case 2:
                        return MISCREG_CNTHP_CVAL_EL2;
                    }
                    break;
                }
                break;
              case 7:
                switch (crm) {
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTPS_TVAL_EL1;
                      case 1:
                        return MISCREG_CNTPS_CTL_EL1;
                      case 2:
                        return MISCREG_CNTPS_CVAL_EL1;
                    }
                    break;
                }
                break;
            }
            break;
          case 15:
            switch (op1) {
              case 0:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_IL1DATA0_EL1;
                      case 1:
                        return MISCREG_IL1DATA1_EL1;
                      case 2:
                        return MISCREG_IL1DATA2_EL1;
                      case 3:
                        return MISCREG_IL1DATA3_EL1;
                    }
                    break;
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_DL1DATA0_EL1;
                      case 1:
                        return MISCREG_DL1DATA1_EL1;
                      case 2:
                        return MISCREG_DL1DATA2_EL1;
                      case 3:
                        return MISCREG_DL1DATA3_EL1;
                      case 4:
                        return MISCREG_DL1DATA4_EL1;
                    }
                    break;
                }
                break;
              case 1:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_L2ACTLR_EL1;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_CPUACTLR_EL1;
                      case 1:
                        return MISCREG_CPUECTLR_EL1;
                      case 2:
                        return MISCREG_CPUMERRSR_EL1;
                      case 3:
                        return MISCREG_L2MERRSR_EL1;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_CBAR_EL1;

                    }
                    break;
                }
                break;
            }
            break;
        }
        break;
    }

    return MISCREG_UNKNOWN;
}

bitset<NUM_MISCREG_INFOS> miscRegInfo[NUM_MISCREGS]; // initialized below

void
ISA::initializeMiscRegMetadata()
{
    // the MiscReg metadata tables are shared across all instances of the
    // ISA object, so there's no need to initialize them multiple times.
    static bool completed = false;
    if (completed)
        return;

    /**
     * Some registers alias with others, and therefore need to be translated.
     * When two mapping registers are given, they are the 32b lower and
     * upper halves, respectively, of the 64b register being mapped.
     * aligned with reference documentation ARM DDI 0487A.i pp 1540-1543
     *
     * NAM = "not architecturally mandated",
     * from ARM DDI 0487A.i, template text
     * "AArch64 System register ___ can be mapped to
     *  AArch32 System register ___, but this is not
     *  architecturally mandated."
     */

    InitReg(MISCREG_CPSR)
      .allPrivileges();
    InitReg(MISCREG_SPSR)
      .allPrivileges();
    InitReg(MISCREG_SPSR_FIQ)
      .allPrivileges();
    InitReg(MISCREG_SPSR_IRQ)
      .allPrivileges();
    InitReg(MISCREG_SPSR_SVC)
      .allPrivileges();
    InitReg(MISCREG_SPSR_MON)
      .allPrivileges();
    InitReg(MISCREG_SPSR_ABT)
      .allPrivileges();
    InitReg(MISCREG_SPSR_HYP)
      .allPrivileges();
    InitReg(MISCREG_SPSR_UND)
      .allPrivileges();
    InitReg(MISCREG_ELR_HYP)
      .allPrivileges();
    InitReg(MISCREG_FPSID)
      .allPrivileges();
    InitReg(MISCREG_FPSCR)
      .allPrivileges();
    InitReg(MISCREG_MVFR1)
      .allPrivileges();
    InitReg(MISCREG_MVFR0)
      .allPrivileges();
    InitReg(MISCREG_FPEXC)
      .allPrivileges();

    // Helper registers
    InitReg(MISCREG_CPSR_MODE)
      .allPrivileges();
    InitReg(MISCREG_CPSR_Q)
      .allPrivileges();
    InitReg(MISCREG_FPSCR_EXC)
      .allPrivileges();
    InitReg(MISCREG_FPSCR_QC)
      .allPrivileges();
    InitReg(MISCREG_LOCKADDR)
      .allPrivileges();
    InitReg(MISCREG_LOCKFLAG)
      .allPrivileges();
    InitReg(MISCREG_PRRR_MAIR0)
      .mutex()
      .banked();
    InitReg(MISCREG_PRRR_MAIR0_NS)
      .mutex()
      .bankedChild();
    InitReg(MISCREG_PRRR_MAIR0_S)
      .mutex()
      .bankedChild();
    InitReg(MISCREG_NMRR_MAIR1)
      .mutex()
      .banked();
    InitReg(MISCREG_NMRR_MAIR1_NS)
      .mutex()
      .bankedChild();
    InitReg(MISCREG_NMRR_MAIR1_S)
      .mutex()
      .bankedChild();
    InitReg(MISCREG_PMXEVTYPER_PMCCFILTR)
      .mutex();
    InitReg(MISCREG_SCTLR_RST)
      .allPrivileges();
    InitReg(MISCREG_SEV_MAILBOX)
      .allPrivileges();

    // AArch32 CP14 registers
    InitReg(MISCREG_DBGDIDR)
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGDSCRint)
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGDCCINT)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDTRTXint)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDTRRXint)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGWFAR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGVCR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDTRRXext)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDSCRext)
      .unimplemented()
      .warnNotFail()
      .allPrivileges();
    InitReg(MISCREG_DBGDTRTXext)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGOSECCR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBVR0)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBVR1)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBVR2)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBVR3)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBVR4)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBVR5)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBCR0)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBCR1)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBCR2)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBCR3)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBCR4)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBCR5)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGWVR0)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGWVR1)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGWVR2)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGWVR3)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGWCR0)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGWCR1)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGWCR2)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGWCR3)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDRAR)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGBXVR4)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBXVR5)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGOSLAR)
      .unimplemented()
      .allPrivileges().monSecureRead(0).monNonSecureRead(0);
    InitReg(MISCREG_DBGOSLSR)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGOSDLR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGPRCR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDSAR)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGCLAIMSET)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGCLAIMCLR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGAUTHSTATUS)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGDEVID2)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGDEVID1)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGDEVID0)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_TEECR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_JIDR)
      .allPrivileges();
    InitReg(MISCREG_TEEHBR)
      .allPrivileges();
    InitReg(MISCREG_JOSCR)
      .allPrivileges();
    InitReg(MISCREG_JMCR)
      .allPrivileges();

    // AArch32 CP15 registers
    InitReg(MISCREG_MIDR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CTR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_TCMTR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_TLBTR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_MPIDR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_REVIDR)
      .unimplemented()
      .warnNotFail()
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_PFR0)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_PFR1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_DFR0)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AFR0)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR0)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR2)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR3)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR0)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR2)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR3)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR4)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR5)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CCSIDR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CLIDR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_AIDR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CSSELR)
      .banked();
    InitReg(MISCREG_CSSELR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_CSSELR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_VPIDR)
      .hyp().monNonSecure();
    InitReg(MISCREG_VMPIDR)
      .hyp().monNonSecure();
    InitReg(MISCREG_SCTLR)
      .banked();
    InitReg(MISCREG_SCTLR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_SCTLR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_ACTLR)
      .banked();
    InitReg(MISCREG_ACTLR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_ACTLR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_CPACR)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_SCR)
      .mon().secure().exceptUserMode()
      .res0(0xff40)  // [31:16], [6]
      .res1(0x0030); // [5:4]
    InitReg(MISCREG_SDER)
      .mon();
    InitReg(MISCREG_NSACR)
      .allPrivileges().hypWrite(0).privNonSecureWrite(0).exceptUserMode();
    InitReg(MISCREG_HSCTLR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HACTLR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HCR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HDCR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HCPTR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HSTR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HACR)
      .unimplemented()
      .warnNotFail()
      .hyp().monNonSecure();
    InitReg(MISCREG_TTBR0)
      .banked();
    InitReg(MISCREG_TTBR0_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_TTBR0_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_TTBR1)
      .banked();
    InitReg(MISCREG_TTBR1_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_TTBR1_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_TTBCR)
      .banked();
    InitReg(MISCREG_TTBCR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_TTBCR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HTCR)
      .hyp().monNonSecure();
    InitReg(MISCREG_VTCR)
      .hyp().monNonSecure();
    InitReg(MISCREG_DACR)
      .banked();
    InitReg(MISCREG_DACR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_DACR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_DFSR)
      .banked();
    InitReg(MISCREG_DFSR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_DFSR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_IFSR)
      .banked();
    InitReg(MISCREG_IFSR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_IFSR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_ADFSR)
      .unimplemented()
      .warnNotFail()
      .banked();
    InitReg(MISCREG_ADFSR_NS)
      .unimplemented()
      .warnNotFail()
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_ADFSR_S)
      .unimplemented()
      .warnNotFail()
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_AIFSR)
      .unimplemented()
      .warnNotFail()
      .banked();
    InitReg(MISCREG_AIFSR_NS)
      .unimplemented()
      .warnNotFail()
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_AIFSR_S)
      .unimplemented()
      .warnNotFail()
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HADFSR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HAIFSR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HSR)
      .hyp().monNonSecure();
    InitReg(MISCREG_DFAR)
      .banked();
    InitReg(MISCREG_DFAR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_DFAR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_IFAR)
      .banked();
    InitReg(MISCREG_IFAR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_IFAR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HDFAR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HIFAR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HPFAR)
      .hyp().monNonSecure();
    InitReg(MISCREG_ICIALLUIS)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_BPIALLIS)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_PAR)
      .banked();
    InitReg(MISCREG_PAR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_PAR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_ICIALLU)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ICIMVAU)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_CP15ISB)
      .writes(1);
    InitReg(MISCREG_BPIALL)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_BPIMVA)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DCIMVAC)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DCISW)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS1CPR)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS1CPW)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS1CUR)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS1CUW)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS12NSOPR)
      .privSecureWrite().hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_ATS12NSOPW)
      .privSecureWrite().hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_ATS12NSOUR)
      .privSecureWrite().hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_ATS12NSOUW)
      .privSecureWrite().hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_DCCMVAC)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DCCSW)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_CP15DSB)
      .writes(1);
    InitReg(MISCREG_CP15DMB)
      .writes(1);
    InitReg(MISCREG_DCCMVAU)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DCCIMVAC)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DCCISW)
      .unimplemented()
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ATS1HR)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_ATS1HW)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIALLIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIASIDIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAAIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVALIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAALIS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ITLBIALL)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ITLBIMVA)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_ITLBIASID)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DTLBIALL)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DTLBIMVA)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DTLBIASID)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIALL)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVA)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIASID)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAA)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAL)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIMVAAL)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBIIPAS2IS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIIPAS2LIS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIALLHIS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIMVAHIS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIALLNSNHIS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIMVALHIS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIIPAS2)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIIPAS2L)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIALLH)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIMVAH)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIALLNSNH)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBIMVALH)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_PMCR)
      .allPrivileges();
    InitReg(MISCREG_PMCNTENSET)
      .allPrivileges();
    InitReg(MISCREG_PMCNTENCLR)
      .allPrivileges();
    InitReg(MISCREG_PMOVSR)
      .allPrivileges();
    InitReg(MISCREG_PMSWINC)
      .allPrivileges();
    InitReg(MISCREG_PMSELR)
      .allPrivileges();
    InitReg(MISCREG_PMCEID0)
      .allPrivileges();
    InitReg(MISCREG_PMCEID1)
      .allPrivileges();
    InitReg(MISCREG_PMCCNTR)
      .allPrivileges();
    InitReg(MISCREG_PMXEVTYPER)
      .allPrivileges();
    InitReg(MISCREG_PMCCFILTR)
      .allPrivileges();
    InitReg(MISCREG_PMXEVCNTR)
      .allPrivileges();
    InitReg(MISCREG_PMUSERENR)
      .allPrivileges().userNonSecureWrite(0).userSecureWrite(0);
    InitReg(MISCREG_PMINTENSET)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_PMINTENCLR)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_PMOVSSET)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_L2CTLR)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_L2ECTLR)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_PRRR)
      .banked();
    InitReg(MISCREG_PRRR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_PRRR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_MAIR0)
      .banked();
    InitReg(MISCREG_MAIR0_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_MAIR0_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_NMRR)
      .banked();
    InitReg(MISCREG_NMRR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_NMRR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_MAIR1)
      .banked();
    InitReg(MISCREG_MAIR1_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_MAIR1_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_AMAIR0)
      .banked();
    InitReg(MISCREG_AMAIR0_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_AMAIR0_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_AMAIR1)
      .banked();
    InitReg(MISCREG_AMAIR1_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_AMAIR1_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HMAIR0)
      .hyp().monNonSecure();
    InitReg(MISCREG_HMAIR1)
      .hyp().monNonSecure();
    InitReg(MISCREG_HAMAIR0)
      .unimplemented()
      .warnNotFail()
      .hyp().monNonSecure();
    InitReg(MISCREG_HAMAIR1)
      .unimplemented()
      .warnNotFail()
      .hyp().monNonSecure();
    InitReg(MISCREG_VBAR)
      .banked();
    InitReg(MISCREG_VBAR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_VBAR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_MVBAR)
      .mon().secure().exceptUserMode();
    InitReg(MISCREG_RMR)
      .unimplemented()
      .mon().secure().exceptUserMode();
    InitReg(MISCREG_ISR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_HVBAR)
      .hyp().monNonSecure();
    InitReg(MISCREG_FCSEIDR)
      .unimplemented()
      .warnNotFail()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CONTEXTIDR)
      .banked();
    InitReg(MISCREG_CONTEXTIDR_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_CONTEXTIDR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_TPIDRURW)
      .banked();
    InitReg(MISCREG_TPIDRURW_NS)
      .bankedChild()
      .allPrivileges().monSecure(0).privSecure(0);
    InitReg(MISCREG_TPIDRURW_S)
      .bankedChild()
      .secure();
    InitReg(MISCREG_TPIDRURO)
      .banked();
    InitReg(MISCREG_TPIDRURO_NS)
      .bankedChild()
      .allPrivileges().secure(0).userNonSecureWrite(0).userSecureRead(1);
    InitReg(MISCREG_TPIDRURO_S)
      .bankedChild()
      .secure().userSecureWrite(0);
    InitReg(MISCREG_TPIDRPRW)
      .banked();
    InitReg(MISCREG_TPIDRPRW_NS)
      .bankedChild()
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_TPIDRPRW_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HTPIDR)
      .hyp().monNonSecure();
    InitReg(MISCREG_CNTFRQ)
      .unverifiable()
      .reads(1).mon();
    InitReg(MISCREG_CNTKCTL)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CNTP_TVAL)
      .banked();
    InitReg(MISCREG_CNTP_TVAL_NS)
      .bankedChild()
      .allPrivileges().monSecure(0).privSecure(0);
    InitReg(MISCREG_CNTP_TVAL_S)
      .unimplemented()
      .bankedChild()
      .secure().user(1);
    InitReg(MISCREG_CNTP_CTL)
      .banked();
    InitReg(MISCREG_CNTP_CTL_NS)
      .bankedChild()
      .allPrivileges().monSecure(0).privSecure(0);
    InitReg(MISCREG_CNTP_CTL_S)
      .unimplemented()
      .bankedChild()
      .secure().user(1);
    InitReg(MISCREG_CNTV_TVAL)
      .allPrivileges();
    InitReg(MISCREG_CNTV_CTL)
      .allPrivileges();
    InitReg(MISCREG_CNTHCTL)
      .unimplemented()
      .hypWrite().monNonSecureRead();
    InitReg(MISCREG_CNTHP_TVAL)
      .unimplemented()
      .hypWrite().monNonSecureRead();
    InitReg(MISCREG_CNTHP_CTL)
      .unimplemented()
      .hypWrite().monNonSecureRead();
    InitReg(MISCREG_IL1DATA0)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA1)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA2)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA3)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA0)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA1)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA2)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA3)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA4)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_RAMINDEX)
      .unimplemented()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_L2ACTLR)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CBAR)
      .unimplemented()
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_HTTBR)
      .hyp().monNonSecure();
    InitReg(MISCREG_VTTBR)
      .hyp().monNonSecure();
    InitReg(MISCREG_CNTPCT)
      .reads(1);
    InitReg(MISCREG_CNTVCT)
      .unverifiable()
      .reads(1);
    InitReg(MISCREG_CNTP_CVAL)
      .banked();
    InitReg(MISCREG_CNTP_CVAL_NS)
      .bankedChild()
      .allPrivileges().monSecure(0).privSecure(0);
    InitReg(MISCREG_CNTP_CVAL_S)
      .unimplemented()
      .bankedChild()
      .secure().user(1);
    InitReg(MISCREG_CNTV_CVAL)
      .allPrivileges();
    InitReg(MISCREG_CNTVOFF)
      .hyp().monNonSecure();
    InitReg(MISCREG_CNTHP_CVAL)
      .unimplemented()
      .hypWrite().monNonSecureRead();
    InitReg(MISCREG_CPUMERRSR)
      .unimplemented()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_L2MERRSR)
      .unimplemented()
      .warnNotFail()
      .allPrivileges().exceptUserMode();

    // AArch64 registers (Op0=2);
    InitReg(MISCREG_MDCCINT_EL1)
      .allPrivileges();
    InitReg(MISCREG_OSDTRRX_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGDTRRXext);
    InitReg(MISCREG_MDSCR_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGDSCRext);
    InitReg(MISCREG_OSDTRTX_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGDTRTXext);
    InitReg(MISCREG_OSECCR_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGOSECCR);
    InitReg(MISCREG_DBGBVR0_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBVR0 /*, MISCREG_DBGBXVR0 */);
    InitReg(MISCREG_DBGBVR1_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBVR1 /*, MISCREG_DBGBXVR1 */);
    InitReg(MISCREG_DBGBVR2_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBVR2 /*, MISCREG_DBGBXVR2 */);
    InitReg(MISCREG_DBGBVR3_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBVR3 /*, MISCREG_DBGBXVR3 */);
    InitReg(MISCREG_DBGBVR4_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBVR4 /*, MISCREG_DBGBXVR4 */);
    InitReg(MISCREG_DBGBVR5_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBVR5 /*, MISCREG_DBGBXVR5 */);
    InitReg(MISCREG_DBGBCR0_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBCR0);
    InitReg(MISCREG_DBGBCR1_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBCR1);
    InitReg(MISCREG_DBGBCR2_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBCR2);
    InitReg(MISCREG_DBGBCR3_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBCR3);
    InitReg(MISCREG_DBGBCR4_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBCR4);
    InitReg(MISCREG_DBGBCR5_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGBCR5);
    InitReg(MISCREG_DBGWVR0_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGWVR0);
    InitReg(MISCREG_DBGWVR1_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGWVR1);
    InitReg(MISCREG_DBGWVR2_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGWVR2);
    InitReg(MISCREG_DBGWVR3_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGWVR3);
    InitReg(MISCREG_DBGWCR0_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGWCR0);
    InitReg(MISCREG_DBGWCR1_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGWCR1);
    InitReg(MISCREG_DBGWCR2_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGWCR2);
    InitReg(MISCREG_DBGWCR3_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGWCR3);
    InitReg(MISCREG_MDCCSR_EL0)
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0)
      .mapsTo(MISCREG_DBGDSCRint);
    InitReg(MISCREG_MDDTR_EL0)
      .allPrivileges();
    InitReg(MISCREG_MDDTRTX_EL0)
      .allPrivileges();
    InitReg(MISCREG_MDDTRRX_EL0)
      .allPrivileges();
    InitReg(MISCREG_DBGVCR32_EL2)
      .allPrivileges()
      .mapsTo(MISCREG_DBGVCR);
    InitReg(MISCREG_MDRAR_EL1)
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0)
      .mapsTo(MISCREG_DBGDRAR);
    InitReg(MISCREG_OSLAR_EL1)
      .allPrivileges().monSecureRead(0).monNonSecureRead(0)
      .mapsTo(MISCREG_DBGOSLAR);
    InitReg(MISCREG_OSLSR_EL1)
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0)
      .mapsTo(MISCREG_DBGOSLSR);
    InitReg(MISCREG_OSDLR_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGOSDLR);
    InitReg(MISCREG_DBGPRCR_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGPRCR);
    InitReg(MISCREG_DBGCLAIMSET_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGCLAIMSET);
    InitReg(MISCREG_DBGCLAIMCLR_EL1)
      .allPrivileges()
      .mapsTo(MISCREG_DBGCLAIMCLR);
    InitReg(MISCREG_DBGAUTHSTATUS_EL1)
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0)
      .mapsTo(MISCREG_DBGAUTHSTATUS);
    InitReg(MISCREG_TEECR32_EL1);
    InitReg(MISCREG_TEEHBR32_EL1);

    // AArch64 registers (Op0=1,3);
    InitReg(MISCREG_MIDR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_MPIDR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_REVIDR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_PFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_PFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_DFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_DFR0);
    InitReg(MISCREG_ID_AFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR2_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_MMFR3_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR0_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR1_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR2_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR3_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR4_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_ISAR5_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_MVFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_MVFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_MVFR2_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64PFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64PFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64DFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64DFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64AFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64AFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64ISAR0_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64ISAR1_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64MMFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ID_AA64MMFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CCSIDR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CLIDR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_AIDR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CSSELR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_CSSELR_NS);
    InitReg(MISCREG_CTR_EL0)
      .reads(1);
    InitReg(MISCREG_DCZID_EL0)
      .reads(1);
    InitReg(MISCREG_VPIDR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_VPIDR);
    InitReg(MISCREG_VMPIDR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_VMPIDR);
    InitReg(MISCREG_SCTLR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_SCTLR_NS);
    InitReg(MISCREG_ACTLR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_ACTLR_NS);
    InitReg(MISCREG_CPACR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_CPACR);
    InitReg(MISCREG_SCTLR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HSCTLR);
    InitReg(MISCREG_ACTLR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HACTLR);
    InitReg(MISCREG_HCR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HCR /*, MISCREG_HCR2*/);
    InitReg(MISCREG_MDCR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HDCR);
    InitReg(MISCREG_CPTR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HCPTR);
    InitReg(MISCREG_HSTR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HSTR);
    InitReg(MISCREG_HACR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HACR);
    InitReg(MISCREG_SCTLR_EL3)
      .mon();
    InitReg(MISCREG_ACTLR_EL3)
      .mon();
    InitReg(MISCREG_SCR_EL3)
      .mon()
      .mapsTo(MISCREG_SCR); // NAM D7-2005
    InitReg(MISCREG_SDER32_EL3)
      .mon()
      .mapsTo(MISCREG_SDER);
    InitReg(MISCREG_CPTR_EL3)
      .mon();
    InitReg(MISCREG_MDCR_EL3)
      .mon();
    InitReg(MISCREG_TTBR0_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_TTBR0_NS);
    InitReg(MISCREG_TTBR1_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_TTBR1_NS);
    InitReg(MISCREG_TCR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_TTBCR_NS);
    InitReg(MISCREG_TTBR0_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HTTBR);
    InitReg(MISCREG_TCR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HTCR);
    InitReg(MISCREG_VTTBR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_VTTBR);
    InitReg(MISCREG_VTCR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_VTCR);
    InitReg(MISCREG_TTBR0_EL3)
      .mon();
    InitReg(MISCREG_TCR_EL3)
      .mon();
    InitReg(MISCREG_DACR32_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_DACR_NS);
    InitReg(MISCREG_SPSR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_SPSR_SVC); // NAM C5.2.17 SPSR_EL1
    InitReg(MISCREG_ELR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_SP_EL0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_SPSEL)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CURRENTEL)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_NZCV)
      .allPrivileges();
    InitReg(MISCREG_DAIF)
      .allPrivileges();
    InitReg(MISCREG_FPCR)
      .allPrivileges();
    InitReg(MISCREG_FPSR)
      .allPrivileges();
    InitReg(MISCREG_DSPSR_EL0)
      .allPrivileges();
    InitReg(MISCREG_DLR_EL0)
      .allPrivileges();
    InitReg(MISCREG_SPSR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_SPSR_HYP); // NAM C5.2.18 SPSR_EL2
    InitReg(MISCREG_ELR_EL2)
      .hyp().mon();
    InitReg(MISCREG_SP_EL1)
      .hyp().mon();
    InitReg(MISCREG_SPSR_IRQ_AA64)
      .hyp().mon();
    InitReg(MISCREG_SPSR_ABT_AA64)
      .hyp().mon();
    InitReg(MISCREG_SPSR_UND_AA64)
      .hyp().mon();
    InitReg(MISCREG_SPSR_FIQ_AA64)
      .hyp().mon();
    InitReg(MISCREG_SPSR_EL3)
      .mon()
      .mapsTo(MISCREG_SPSR_MON); // NAM C5.2.19 SPSR_EL3
    InitReg(MISCREG_ELR_EL3)
      .mon();
    InitReg(MISCREG_SP_EL2)
      .mon();
    InitReg(MISCREG_AFSR0_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_ADFSR_NS);
    InitReg(MISCREG_AFSR1_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_AIFSR_NS);
    InitReg(MISCREG_ESR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IFSR32_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_IFSR_NS);
    InitReg(MISCREG_AFSR0_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HADFSR);
    InitReg(MISCREG_AFSR1_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HAIFSR);
    InitReg(MISCREG_ESR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HSR);
    InitReg(MISCREG_FPEXC32_EL2)
      .hyp().mon();
    InitReg(MISCREG_AFSR0_EL3)
      .mon();
    InitReg(MISCREG_AFSR1_EL3)
      .mon();
    InitReg(MISCREG_ESR_EL3)
      .mon();
    InitReg(MISCREG_FAR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DFAR_NS, MISCREG_IFAR_NS);
    InitReg(MISCREG_FAR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HDFAR, MISCREG_HIFAR);
    InitReg(MISCREG_HPFAR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HPFAR);
    InitReg(MISCREG_FAR_EL3)
      .mon();
    InitReg(MISCREG_IC_IALLUIS)
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_PAR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_PAR_NS);
    InitReg(MISCREG_IC_IALLU)
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DC_IVAC_Xt)
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DC_ISW_Xt)
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_AT_S1E1R_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_AT_S1E1W_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_AT_S1E0R_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_AT_S1E0W_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DC_CSW_Xt)
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DC_CISW_Xt)
      .warnNotFail()
      .writes(1).exceptUserMode();
    InitReg(MISCREG_DC_ZVA_Xt)
      .warnNotFail()
      .writes(1).userSecureWrite(0);
    InitReg(MISCREG_IC_IVAU_Xt)
      .writes(1);
    InitReg(MISCREG_DC_CVAC_Xt)
      .warnNotFail()
      .writes(1);
    InitReg(MISCREG_DC_CVAU_Xt)
      .warnNotFail()
      .writes(1);
    InitReg(MISCREG_DC_CIVAC_Xt)
      .warnNotFail()
      .writes(1);
    InitReg(MISCREG_AT_S1E2R_Xt)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_AT_S1E2W_Xt)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_AT_S12E1R_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_AT_S12E1W_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_AT_S12E0R_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_AT_S12E0W_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_AT_S1E3R_Xt)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_AT_S1E3W_Xt)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VMALLE1IS)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAE1IS_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_ASIDE1IS_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAAE1IS_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VALE1IS_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAALE1IS_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VMALLE1)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAE1_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_ASIDE1_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAAE1_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VALE1_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_VAALE1_Xt)
      .writes(1).exceptUserMode();
    InitReg(MISCREG_TLBI_IPAS2E1IS_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_IPAS2LE1IS_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_ALLE2IS)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBI_VAE2IS_Xt)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBI_ALLE1IS)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VALE2IS_Xt)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBI_VMALLS12E1IS)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_IPAS2E1_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_IPAS2LE1_Xt)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_ALLE2)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBI_VAE2_Xt)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBI_ALLE1)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VALE2_Xt)
      .monNonSecureWrite().hypWrite();
    InitReg(MISCREG_TLBI_VMALLS12E1)
      .hypWrite().monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_ALLE3IS)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VAE3IS_Xt)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VALE3IS_Xt)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_ALLE3)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VAE3_Xt)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_TLBI_VALE3_Xt)
      .monSecureWrite().monNonSecureWrite();
    InitReg(MISCREG_PMINTENSET_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_PMINTENSET);
    InitReg(MISCREG_PMINTENCLR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_PMINTENCLR);
    InitReg(MISCREG_PMCR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMCR);
    InitReg(MISCREG_PMCNTENSET_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMCNTENSET);
    InitReg(MISCREG_PMCNTENCLR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMCNTENCLR);
    InitReg(MISCREG_PMOVSCLR_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMOVSCLR);
    InitReg(MISCREG_PMSWINC_EL0)
      .writes(1).user()
      .mapsTo(MISCREG_PMSWINC);
    InitReg(MISCREG_PMSELR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMSELR);
    InitReg(MISCREG_PMCEID0_EL0)
      .reads(1).user()
      .mapsTo(MISCREG_PMCEID0);
    InitReg(MISCREG_PMCEID1_EL0)
      .reads(1).user()
      .mapsTo(MISCREG_PMCEID1);
    InitReg(MISCREG_PMCCNTR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMCCNTR);
    InitReg(MISCREG_PMXEVTYPER_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMXEVTYPER);
    InitReg(MISCREG_PMCCFILTR_EL0)
      .allPrivileges();
    InitReg(MISCREG_PMXEVCNTR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMXEVCNTR);
    InitReg(MISCREG_PMUSERENR_EL0)
      .allPrivileges().userNonSecureWrite(0).userSecureWrite(0)
      .mapsTo(MISCREG_PMUSERENR);
    InitReg(MISCREG_PMOVSSET_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_PMOVSSET);
    InitReg(MISCREG_MAIR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_PRRR_NS, MISCREG_NMRR_NS);
    InitReg(MISCREG_AMAIR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_AMAIR0_NS, MISCREG_AMAIR1_NS);
    InitReg(MISCREG_MAIR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HMAIR0, MISCREG_HMAIR1);
    InitReg(MISCREG_AMAIR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HAMAIR0, MISCREG_HAMAIR1);
    InitReg(MISCREG_MAIR_EL3)
      .mon();
    InitReg(MISCREG_AMAIR_EL3)
      .mon();
    InitReg(MISCREG_L2CTLR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_L2ECTLR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_VBAR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_VBAR_NS);
    InitReg(MISCREG_RVBAR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ISR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_VBAR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HVBAR);
    InitReg(MISCREG_RVBAR_EL2)
      .mon().hyp().writes(0);
    InitReg(MISCREG_VBAR_EL3)
      .mon();
    InitReg(MISCREG_RVBAR_EL3)
      .mon().writes(0);
    InitReg(MISCREG_RMR_EL3)
      .mon();
    InitReg(MISCREG_CONTEXTIDR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_CONTEXTIDR_NS);
    InitReg(MISCREG_TPIDR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_TPIDRPRW_NS);
    InitReg(MISCREG_TPIDR_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_TPIDRURW_NS);
    InitReg(MISCREG_TPIDRRO_EL0)
      .allPrivileges().userNonSecureWrite(0).userSecureWrite(0)
      .mapsTo(MISCREG_TPIDRURO_NS);
    InitReg(MISCREG_TPIDR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HTPIDR);
    InitReg(MISCREG_TPIDR_EL3)
      .mon();
    InitReg(MISCREG_CNTKCTL_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_CNTKCTL);
    InitReg(MISCREG_CNTFRQ_EL0)
      .reads(1).mon()
      .mapsTo(MISCREG_CNTFRQ);
    InitReg(MISCREG_CNTPCT_EL0)
      .reads(1)
      .mapsTo(MISCREG_CNTPCT); /* 64b */
    InitReg(MISCREG_CNTVCT_EL0)
      .unverifiable()
      .reads(1)
      .mapsTo(MISCREG_CNTVCT); /* 64b */
    InitReg(MISCREG_CNTP_TVAL_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_CNTP_TVAL_NS);
    InitReg(MISCREG_CNTP_CTL_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_CNTP_CTL_NS);
    InitReg(MISCREG_CNTP_CVAL_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_CNTP_CVAL_NS); /* 64b */
    InitReg(MISCREG_CNTV_TVAL_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_CNTV_TVAL);
    InitReg(MISCREG_CNTV_CTL_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_CNTV_CTL);
    InitReg(MISCREG_CNTV_CVAL_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_CNTV_CVAL); /* 64b */
    InitReg(MISCREG_PMEVCNTR0_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR0);
    InitReg(MISCREG_PMEVCNTR1_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR1);
    InitReg(MISCREG_PMEVCNTR2_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR2);
    InitReg(MISCREG_PMEVCNTR3_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR3);
    InitReg(MISCREG_PMEVCNTR4_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR4);
    InitReg(MISCREG_PMEVCNTR5_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVCNTR5);
    InitReg(MISCREG_PMEVTYPER0_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER0);
    InitReg(MISCREG_PMEVTYPER1_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER1);
    InitReg(MISCREG_PMEVTYPER2_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER2);
    InitReg(MISCREG_PMEVTYPER3_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER3);
    InitReg(MISCREG_PMEVTYPER4_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER4);
    InitReg(MISCREG_PMEVTYPER5_EL0)
      .allPrivileges();
//    .mapsTo(MISCREG_PMEVTYPER5);
    InitReg(MISCREG_CNTVOFF_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_CNTVOFF); /* 64b */
    InitReg(MISCREG_CNTHCTL_EL2)
      .unimplemented()
      .warnNotFail()
      .mon().monNonSecureWrite(0).hypWrite()
      .mapsTo(MISCREG_CNTHCTL);
    InitReg(MISCREG_CNTHP_TVAL_EL2)
      .unimplemented()
      .mon().monNonSecureWrite(0).hypWrite()
      .mapsTo(MISCREG_CNTHP_TVAL);
    InitReg(MISCREG_CNTHP_CTL_EL2)
      .unimplemented()
      .mon().monNonSecureWrite(0).hypWrite()
      .mapsTo(MISCREG_CNTHP_CTL);
    InitReg(MISCREG_CNTHP_CVAL_EL2)
      .unimplemented()
      .mon().monNonSecureWrite(0).hypWrite()
      .mapsTo(MISCREG_CNTHP_CVAL); /* 64b */
    InitReg(MISCREG_CNTPS_TVAL_EL1)
      .unimplemented()
      .mon().monNonSecureWrite(0).hypWrite();
    InitReg(MISCREG_CNTPS_CTL_EL1)
      .unimplemented()
      .mon().monNonSecureWrite(0).hypWrite();
    InitReg(MISCREG_CNTPS_CVAL_EL1)
      .unimplemented()
      .mon().monNonSecureWrite(0).hypWrite();
    InitReg(MISCREG_IL1DATA0_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA1_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA2_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_IL1DATA3_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA0_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA1_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA2_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA3_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DL1DATA4_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_L2ACTLR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CPUACTLR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CPUECTLR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CPUMERRSR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_L2MERRSR_EL1)
      .unimplemented()
      .warnNotFail()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CBAR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CONTEXTIDR_EL2)
      .mon().hyp();

    // Dummy registers
    InitReg(MISCREG_NOP)
      .allPrivileges();
    InitReg(MISCREG_RAZ)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_CP14_UNIMPL)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_CP15_UNIMPL)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_A64_UNIMPL)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_UNKNOWN);

    // Register mappings for some unimplemented registers:
    // ESR_EL1 -> DFSR
    // RMR_EL1 -> RMR
    // RMR_EL2 -> HRMR
    // DBGDTR_EL0 -> DBGDTR{R or T}Xint
    // DBGDTRRX_EL0 -> DBGDTRRXint
    // DBGDTRTX_EL0 -> DBGDTRRXint
    // MDCR_EL3 -> SDCR, NAM D7-2108 (the latter is unimpl. in gem5)

    completed = true;
}

} // namespace ArmISA
