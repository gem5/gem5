/*
 * Copyright (c) 2010-2012 ARM Limited
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
 */

#include "arch/arm/isa.hh"
#include "arch/arm/miscregs.hh"
#include "base/misc.hh"

namespace ArmISA
{

MiscRegIndex
decodeCP14Reg(unsigned crn, unsigned opc1, unsigned crm, unsigned opc2)
{
    switch(crn) {
      case 0:
        switch (opc2) {
          case 0:
            switch (crm) {
              case 0:
                return MISCREG_DBGDIDR;
              case 1:
                return MISCREG_DBGDSCR_INT;
              default:
                warn("CP14 unimplemented crn[%d], opc1[%d], crm[%d], opc2[%d]",
                     crn, opc1, crm, opc2);
                return NUM_MISCREGS;
            }
          default:
            warn("CP14 unimplemented crn[%d], opc1[%d], crm[%d], opc2[%d]",
                 crn, opc1, crm, opc2);
            return NUM_MISCREGS;
        }
      case 1:
        switch (opc1) {
          case 6:
            switch (crm) {
              case 0:
                switch (opc2) {
                  case 0:
                    return MISCREG_TEEHBR;
                  default:
                    warn("CP14 unimplemented crn[%d], opc1[%d], crm[%d], opc2[%d]",
                         crn, opc1, crm, opc2);
                    return NUM_MISCREGS;
                }
              default:
                warn("CP14 unimplemented crn[%d], opc1[%d], crm[%d], opc2[%d]",
                     crn, opc1, crm, opc2);
                return NUM_MISCREGS;
            }
          default:
            warn("CP14 unimplemented crn[%d], opc1[%d], crm[%d], opc2[%d]",
                 crn, opc1, crm, opc2);
            return NUM_MISCREGS;
        }
      default:
        warn("CP14 unimplemented crn[%d], opc1[%d], crm[%d], opc2[%d]",
             crn, opc1, crm, opc2);
        return NUM_MISCREGS;
    }

}

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
                    return MISCREG_V2PCWPR;
                  case 1:
                    return MISCREG_V2PCWPW;
                  case 2:
                    return MISCREG_V2PCWUR;
                  case 3:
                    return MISCREG_V2PCWUW;
                  case 4:
                    return MISCREG_V2POWPR;
                  case 5:
                    return MISCREG_V2POWPW;
                  case 6:
                    return MISCREG_V2POWUR;
                  case 7:
                    return MISCREG_V2POWUW;
                }
                break;
              case 10:
                switch (opc2) {
                  case 1:
                    return MISCREG_DCCMVAC;
                  case 2:
                    return MISCREG_MCCSW;
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
                }
                break;
            }
        }
        break;
      case 9:
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
                    return MISCREG_PMC_OTHER;
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
                }
                break;
            }
        } else if (opc1 == 1) {
            switch (crm) {
              case 0:
                switch (opc2) {
                  case 2: // L2CTLR, L2 Control Register
                    return MISCREG_L2CTLR;
                  default:
                    warn("Uknown miscregs: crn:%d crm:%d opc1:%d opc2:%d\n",
                         crn,crm, opc1,opc2);
                    break;
                }
                break;
              default:
                return MISCREG_L2LATENCY;
            }
        }
        //Reserved for Branch Predictor, Cache and TCM operations
        break;
      case 10:
        if (opc1 == 0) {
            // crm 0, 1, 4, and 8, with op2 0 - 7, reserved for TLB lockdown
            if (crm == 2) { // TEX Remap Registers
                if (opc2 == 0) {
                    return MISCREG_PRRR;
                } else if (opc2 == 1) {
                    return MISCREG_NMRR;
                }
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
        }
        break;
      case 13:
        if (opc1 == 0) {
            if (crm == 0) {
                switch (opc2) {
                  case 0:
                    return MISCREG_FCEIDR;
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
        }
        break;
      case 15:
        // Implementation defined
        return MISCREG_CRN15;
    }
    // Unrecognized register
    return NUM_MISCREGS;
}

}
