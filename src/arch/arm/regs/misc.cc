/*
 * Copyright (c) 2010-2013, 2015-2020 ARM Limited
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

#include "arch/arm/regs/misc.hh"

#include <tuple>

#include "arch/arm/isa.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "sim/full_system.hh"

namespace gem5
{

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
                  case 7:
                    return MISCREG_DBGVCR;
                }
                break;
              case 2:
                switch (crm) {
                  case 0:
                    return MISCREG_DBGDTRRXext;
                  case 2:
                    return MISCREG_DBGDSCRext;
                  case 3:
                    return MISCREG_DBGDTRTXext;
                  case 6:
                    return MISCREG_DBGOSECCR;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    return MISCREG_DBGBVR0;
                  case 1:
                    return MISCREG_DBGBVR1;
                  case 2:
                    return MISCREG_DBGBVR2;
                  case 3:
                    return MISCREG_DBGBVR3;
                  case 4:
                    return MISCREG_DBGBVR4;
                  case 5:
                    return MISCREG_DBGBVR5;
                  case 6:
                    return MISCREG_DBGBVR6;
                  case 7:
                    return MISCREG_DBGBVR7;
                  case 8:
                    return MISCREG_DBGBVR8;
                  case 9:
                    return MISCREG_DBGBVR9;
                  case 10:
                    return MISCREG_DBGBVR10;
                  case 11:
                    return MISCREG_DBGBVR11;
                  case 12:
                    return MISCREG_DBGBVR12;
                  case 13:
                    return MISCREG_DBGBVR13;
                  case 14:
                    return MISCREG_DBGBVR14;
                  case 15:
                    return MISCREG_DBGBVR15;
                }
                break;
              case 5:
                switch (crm) {
                  case 0:
                    return MISCREG_DBGBCR0;
                  case 1:
                    return MISCREG_DBGBCR1;
                  case 2:
                    return MISCREG_DBGBCR2;
                  case 3:
                    return MISCREG_DBGBCR3;
                  case 4:
                    return MISCREG_DBGBCR4;
                  case 5:
                    return MISCREG_DBGBCR5;
                  case 6:
                    return MISCREG_DBGBCR6;
                  case 7:
                    return MISCREG_DBGBCR7;
                  case 8:
                    return MISCREG_DBGBCR8;
                  case 9:
                    return MISCREG_DBGBCR9;
                  case 10:
                    return MISCREG_DBGBCR10;
                  case 11:
                    return MISCREG_DBGBCR11;
                  case 12:
                    return MISCREG_DBGBCR12;
                  case 13:
                    return MISCREG_DBGBCR13;
                  case 14:
                    return MISCREG_DBGBCR14;
                  case 15:
                    return MISCREG_DBGBCR15;
                }
                break;
              case 6:
                switch (crm) {
                  case 0:
                    return MISCREG_DBGWVR0;
                  case 1:
                    return MISCREG_DBGWVR1;
                  case 2:
                    return MISCREG_DBGWVR2;
                  case 3:
                    return MISCREG_DBGWVR3;
                  case 4:
                    return MISCREG_DBGWVR4;
                  case 5:
                    return MISCREG_DBGWVR5;
                  case 6:
                    return MISCREG_DBGWVR6;
                  case 7:
                    return MISCREG_DBGWVR7;
                  case 8:
                    return MISCREG_DBGWVR8;
                  case 9:
                    return MISCREG_DBGWVR9;
                  case 10:
                    return MISCREG_DBGWVR10;
                  case 11:
                    return MISCREG_DBGWVR11;
                  case 12:
                    return MISCREG_DBGWVR12;
                  case 13:
                    return MISCREG_DBGWVR13;
                  case 14:
                    return MISCREG_DBGWVR14;
                  case 15:
                    return MISCREG_DBGWVR15;
                    break;
                }
                break;
              case 7:
                switch (crm) {
                  case 0:
                    return MISCREG_DBGWCR0;
                  case 1:
                    return MISCREG_DBGWCR1;
                  case 2:
                    return MISCREG_DBGWCR2;
                  case 3:
                    return MISCREG_DBGWCR3;
                  case 4:
                    return MISCREG_DBGWCR4;
                  case 5:
                    return MISCREG_DBGWCR5;
                  case 6:
                    return MISCREG_DBGWCR6;
                  case 7:
                    return MISCREG_DBGWCR7;
                  case 8:
                    return MISCREG_DBGWCR8;
                  case 9:
                    return MISCREG_DBGWCR9;
                  case 10:
                    return MISCREG_DBGWCR10;
                  case 11:
                    return MISCREG_DBGWCR11;
                  case 12:
                    return MISCREG_DBGWCR12;
                  case 13:
                    return MISCREG_DBGWCR13;
                  case 14:
                    return MISCREG_DBGWCR14;
                  case 15:
                    return MISCREG_DBGWCR15;
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
          case 0:
            switch(opc2) {
              case 1:
                switch(crm) {
                  case 0:
                      return MISCREG_DBGBXVR0;
                  case 1:
                      return MISCREG_DBGBXVR1;
                  case 2:
                      return MISCREG_DBGBXVR2;
                  case 3:
                      return MISCREG_DBGBXVR3;
                  case 4:
                      return MISCREG_DBGBXVR4;
                  case 5:
                      return MISCREG_DBGBXVR5;
                  case 6:
                      return MISCREG_DBGBXVR6;
                  case 7:
                      return MISCREG_DBGBXVR7;
                  case 8:
                      return MISCREG_DBGBXVR8;
                  case 9:
                      return MISCREG_DBGBXVR9;
                  case 10:
                      return MISCREG_DBGBXVR10;
                  case 11:
                      return MISCREG_DBGBXVR11;
                  case 12:
                      return MISCREG_DBGBXVR12;
                  case 13:
                      return MISCREG_DBGBXVR13;
                  case 14:
                      return MISCREG_DBGBXVR14;
                  case 15:
                      return MISCREG_DBGBXVR15;
                }
                break;
              case 4:
                switch (crm) {
                  case 0:
                    return MISCREG_DBGOSLAR;
                  case 1:
                    return MISCREG_DBGOSLSR;
                  case 3:
                    return MISCREG_DBGOSDLR;
                  case 4:
                    return MISCREG_DBGPRCR;
                }
                break;
            }
            break;
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
                    return MISCREG_ID_MMFR4;
                  case 7:
                    return MISCREG_ID_ISAR6;
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
            } else if (crm == 3) {
                if ( opc2 == 1)
                    return MISCREG_SDCR;
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
                  case 4:
                    return MISCREG_HCR2;
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
      case 4:
        if (opc1 == 0 && crm == 6 && opc2 == 0) {
            return MISCREG_ICC_PMR;
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
              case 2:
                switch (opc2) {
                  case 7:
                    return MISCREG_DBGDEVID0;
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
            } else if (crm == 8) {
                switch (opc2) {
                    case 0:
                        return MISCREG_ICC_IAR0;
                    case 1:
                        return MISCREG_ICC_EOIR0;
                    case 2:
                        return MISCREG_ICC_HPPIR0;
                    case 3:
                        return MISCREG_ICC_BPR0;
                    case 4:
                        return MISCREG_ICC_AP0R0;
                    case 5:
                        return MISCREG_ICC_AP0R1;
                    case 6:
                        return MISCREG_ICC_AP0R2;
                    case 7:
                        return MISCREG_ICC_AP0R3;
                }
            } else if (crm == 9) {
                switch (opc2) {
                    case 0:
                        return MISCREG_ICC_AP1R0;
                    case 1:
                        return MISCREG_ICC_AP1R1;
                    case 2:
                        return MISCREG_ICC_AP1R2;
                    case 3:
                        return MISCREG_ICC_AP1R3;
                }
            } else if (crm == 11) {
                switch (opc2) {
                    case 1:
                        return MISCREG_ICC_DIR;
                    case 3:
                        return MISCREG_ICC_RPR;
                }
            } else if (crm == 12) {
                switch (opc2) {
                    case 0:
                        return MISCREG_ICC_IAR1;
                    case 1:
                        return MISCREG_ICC_EOIR1;
                    case 2:
                        return MISCREG_ICC_HPPIR1;
                    case 3:
                        return MISCREG_ICC_BPR1;
                    case 4:
                        return MISCREG_ICC_CTLR;
                    case 5:
                        return MISCREG_ICC_SRE;
                    case 6:
                        return MISCREG_ICC_IGRPEN0;
                    case 7:
                        return MISCREG_ICC_IGRPEN1;
                }
            }
        } else if (opc1 == 4) {
            if (crm == 0 && opc2 == 0) {
                return MISCREG_HVBAR;
            } else if (crm == 8) {
                switch (opc2) {
                    case 0:
                        return MISCREG_ICH_AP0R0;
                    case 1:
                        return MISCREG_ICH_AP0R1;
                    case 2:
                        return MISCREG_ICH_AP0R2;
                    case 3:
                        return MISCREG_ICH_AP0R3;
                }
            } else if (crm == 9) {
                switch (opc2) {
                    case 0:
                        return MISCREG_ICH_AP1R0;
                    case 1:
                        return MISCREG_ICH_AP1R1;
                    case 2:
                        return MISCREG_ICH_AP1R2;
                    case 3:
                        return MISCREG_ICH_AP1R3;
                    case 5:
                        return MISCREG_ICC_HSRE;
                }
            } else if (crm == 11) {
                switch (opc2) {
                    case 0:
                        return MISCREG_ICH_HCR;
                    case 1:
                        return MISCREG_ICH_VTR;
                    case 2:
                        return MISCREG_ICH_MISR;
                    case 3:
                        return MISCREG_ICH_EISR;
                    case 5:
                        return MISCREG_ICH_ELRSR;
                    case 7:
                        return MISCREG_ICH_VMCR;
                }
            } else if (crm == 12) {
                switch (opc2) {
                    case 0:
                        return MISCREG_ICH_LR0;
                    case 1:
                        return MISCREG_ICH_LR1;
                    case 2:
                        return MISCREG_ICH_LR2;
                    case 3:
                        return MISCREG_ICH_LR3;
                    case 4:
                        return MISCREG_ICH_LR4;
                    case 5:
                        return MISCREG_ICH_LR5;
                    case 6:
                        return MISCREG_ICH_LR6;
                    case 7:
                        return MISCREG_ICH_LR7;
                }
            } else if (crm == 13) {
                switch (opc2) {
                    case 0:
                        return MISCREG_ICH_LR8;
                    case 1:
                        return MISCREG_ICH_LR9;
                    case 2:
                        return MISCREG_ICH_LR10;
                    case 3:
                        return MISCREG_ICH_LR11;
                    case 4:
                        return MISCREG_ICH_LR12;
                    case 5:
                        return MISCREG_ICH_LR13;
                    case 6:
                        return MISCREG_ICH_LR14;
                    case 7:
                        return MISCREG_ICH_LR15;
                }
            } else if (crm == 14) {
                switch (opc2) {
                    case 0:
                        return MISCREG_ICH_LRC0;
                    case 1:
                        return MISCREG_ICH_LRC1;
                    case 2:
                        return MISCREG_ICH_LRC2;
                    case 3:
                        return MISCREG_ICH_LRC3;
                    case 4:
                        return MISCREG_ICH_LRC4;
                    case 5:
                        return MISCREG_ICH_LRC5;
                    case 6:
                        return MISCREG_ICH_LRC6;
                    case 7:
                        return MISCREG_ICH_LRC7;
                }
            } else if (crm == 15) {
                switch (opc2) {
                    case 0:
                        return MISCREG_ICH_LRC8;
                    case 1:
                        return MISCREG_ICH_LRC9;
                    case 2:
                        return MISCREG_ICH_LRC10;
                    case 3:
                        return MISCREG_ICH_LRC11;
                    case 4:
                        return MISCREG_ICH_LRC12;
                    case 5:
                        return MISCREG_ICH_LRC13;
                    case 6:
                        return MISCREG_ICH_LRC14;
                    case 7:
                        return MISCREG_ICH_LRC15;
                }
            }
        } else if (opc1 == 6) {
            if (crm == 12) {
                switch (opc2) {
                    case 4:
                        return MISCREG_ICC_MCTLR;
                    case 5:
                        return MISCREG_ICC_MSRE;
                    case 7:
                        return MISCREG_ICC_MGRPEN1;
                }
            }
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
      case 12:
        switch (opc1) {
          case 0:
            return MISCREG_ICC_SGI1R;
          case 1:
            return MISCREG_ICC_ASGI1R;
          case 2:
            return MISCREG_ICC_SGI0R;
          default:
            break;
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
canReadCoprocReg(MiscRegIndex reg, SCR scr, CPSR cpsr, ThreadContext *tc)
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
        canRead = miscRegInfo[reg][MISCREG_HYP_NS_RD];
        break;
      default:
        undefined = true;
    }

    switch (reg) {
      case MISCREG_CNTFRQ ... MISCREG_CNTVOFF:
        if (!undefined)
            undefined = AArch32isUndefinedGenericTimer(reg, tc);
        break;
      default:
        break;
    }

    // can't do permissions checkes on the root of a banked pair of regs
    assert(!miscRegInfo[reg][MISCREG_BANKED]);
    return std::make_tuple(canRead, undefined);
}

std::tuple<bool, bool>
canWriteCoprocReg(MiscRegIndex reg, SCR scr, CPSR cpsr, ThreadContext *tc)
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
        canWrite =  miscRegInfo[reg][MISCREG_HYP_NS_WR];
        break;
      default:
        undefined = true;
    }

    switch (reg) {
      case MISCREG_CNTFRQ ... MISCREG_CNTVOFF:
        if (!undefined)
            undefined = AArch32isUndefinedGenericTimer(reg, tc);
        break;
      default:
        break;
    }

    // can't do permissions checkes on the root of a banked pair of regs
    assert(!miscRegInfo[reg][MISCREG_BANKED]);
    return std::make_tuple(canWrite, undefined);
}

bool
AArch32isUndefinedGenericTimer(MiscRegIndex reg, ThreadContext *tc)
{
    if (currEL(tc) == EL0 && ELIs32(tc, EL1)) {
        const HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
        bool trap_cond = condGenericTimerSystemAccessTrapEL1(reg, tc);
        if (trap_cond && (!EL2Enabled(tc) || !hcr.tge))
            return true;
    }
    return false;
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
        reg_as_int += (ArmSystem::haveEL(tc, EL3) &&
                      !ArmSystem::highestELIs64(tc) && !ns) ? 2 : 1;
    }
    return reg_as_int;
}

int
snsBankedIndex64(MiscRegIndex reg, ThreadContext *tc)
{
    auto *isa = static_cast<ArmISA::ISA *>(tc->getIsaPtr());
    SCR scr = tc->readMiscReg(MISCREG_SCR);
    return isa->snsBankedIndex64(reg, scr.ns);
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
canReadAArch64SysReg(MiscRegIndex reg, HCR hcr, SCR scr, CPSR cpsr,
                     ThreadContext *tc)
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

    bool secure = ArmSystem::haveEL(tc, EL3) && !scr.ns;
    bool el2_host = EL2Enabled(tc) && hcr.e2h;

    switch (currEL(cpsr)) {
      case EL0:
        return secure ? miscRegInfo[reg][MISCREG_USR_S_RD] :
            miscRegInfo[reg][MISCREG_USR_NS_RD];
      case EL1:
        return secure ? miscRegInfo[reg][MISCREG_PRI_S_RD] :
            miscRegInfo[reg][MISCREG_PRI_NS_RD];
      case EL2:
        if (el2_host) {
            return secure ? miscRegInfo[reg][MISCREG_HYP_E2H_S_RD] :
                miscRegInfo[reg][MISCREG_HYP_E2H_NS_RD];
        } else {
            return secure ? miscRegInfo[reg][MISCREG_HYP_S_RD] :
                miscRegInfo[reg][MISCREG_HYP_NS_RD];
        }
      case EL3:
        return el2_host ? miscRegInfo[reg][MISCREG_MON_E2H_RD] :
            secure ? miscRegInfo[reg][MISCREG_MON_NS0_RD] :
            miscRegInfo[reg][MISCREG_MON_NS1_RD];
      default:
        panic("Invalid exception level");
    }
}

bool
canWriteAArch64SysReg(MiscRegIndex reg, HCR hcr, SCR scr, CPSR cpsr,
                      ThreadContext *tc)
{
    // Check for SP_EL0 access while SPSEL == 0
    if ((reg == MISCREG_SP_EL0) && (tc->readMiscReg(MISCREG_SPSEL) == 0))
        return false;
    ExceptionLevel el = currEL(cpsr);

    bool secure = ArmSystem::haveEL(tc, EL3) && !scr.ns;
    bool el2_host = EL2Enabled(tc) && hcr.e2h;

    switch (el) {
      case EL0:
        return secure ? miscRegInfo[reg][MISCREG_USR_S_WR] :
            miscRegInfo[reg][MISCREG_USR_NS_WR];
      case EL1:
        return secure ? miscRegInfo[reg][MISCREG_PRI_S_WR] :
            miscRegInfo[reg][MISCREG_PRI_NS_WR];
      case EL2:
        if (el2_host) {
            return secure ? miscRegInfo[reg][MISCREG_HYP_E2H_S_WR] :
                miscRegInfo[reg][MISCREG_HYP_E2H_NS_WR];
        } else {
            return secure ? miscRegInfo[reg][MISCREG_HYP_S_WR] :
                miscRegInfo[reg][MISCREG_HYP_NS_WR];
        }
      case EL3:
        return el2_host ? miscRegInfo[reg][MISCREG_MON_E2H_WR] :
            secure ? miscRegInfo[reg][MISCREG_MON_NS0_WR] :
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
          case 11:
          case 15:
            // SYS Instruction with CRn = { 11, 15 }
            // (Trappable by HCR_EL2.TIDCP)
            return MISCREG_IMPDEF_UNIMPL;
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
                      case 6:
                        return MISCREG_DBGWVR4_EL1;
                      case 7:
                        return MISCREG_DBGWCR4_EL1;
                    }
                    break;
                  case 5:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR5_EL1;
                      case 5:
                        return MISCREG_DBGBCR5_EL1;
                      case 6:
                        return MISCREG_DBGWVR5_EL1;
                      case 7:
                        return MISCREG_DBGWCR5_EL1;
                    }
                    break;
                  case 6:
                    switch (op2) {
                      case 2:
                        return MISCREG_OSECCR_EL1;
                      case 4:
                        return MISCREG_DBGBVR6_EL1;
                      case 5:
                        return MISCREG_DBGBCR6_EL1;
                      case 6:
                        return MISCREG_DBGWVR6_EL1;
                      case 7:
                        return MISCREG_DBGWCR6_EL1;
                    }
                    break;
                  case 7:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR7_EL1;
                      case 5:
                        return MISCREG_DBGBCR7_EL1;
                      case 6:
                        return MISCREG_DBGWVR7_EL1;
                      case 7:
                        return MISCREG_DBGWCR7_EL1;
                    }
                    break;
                  case 8:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR8_EL1;
                      case 5:
                        return MISCREG_DBGBCR8_EL1;
                      case 6:
                        return MISCREG_DBGWVR8_EL1;
                      case 7:
                        return MISCREG_DBGWCR8_EL1;
                    }
                    break;
                  case 9:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR9_EL1;
                      case 5:
                        return MISCREG_DBGBCR9_EL1;
                      case 6:
                        return MISCREG_DBGWVR9_EL1;
                      case 7:
                        return MISCREG_DBGWCR9_EL1;
                    }
                    break;
                  case 10:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR10_EL1;
                      case 5:
                        return MISCREG_DBGBCR10_EL1;
                      case 6:
                        return MISCREG_DBGWVR10_EL1;
                      case 7:
                        return MISCREG_DBGWCR10_EL1;
                    }
                    break;
                  case 11:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR11_EL1;
                      case 5:
                        return MISCREG_DBGBCR11_EL1;
                      case 6:
                        return MISCREG_DBGWVR11_EL1;
                      case 7:
                        return MISCREG_DBGWCR11_EL1;
                    }
                    break;
                  case 12:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR12_EL1;
                      case 5:
                        return MISCREG_DBGBCR12_EL1;
                      case 6:
                        return MISCREG_DBGWVR12_EL1;
                      case 7:
                        return MISCREG_DBGWCR12_EL1;
                    }
                    break;
                  case 13:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR13_EL1;
                      case 5:
                        return MISCREG_DBGBCR13_EL1;
                      case 6:
                        return MISCREG_DBGWVR13_EL1;
                      case 7:
                        return MISCREG_DBGWCR13_EL1;
                    }
                    break;
                  case 14:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR14_EL1;
                      case 5:
                        return MISCREG_DBGBCR14_EL1;
                      case 6:
                        return MISCREG_DBGWVR14_EL1;
                      case 7:
                        return MISCREG_DBGWCR14_EL1;
                    }
                    break;
                  case 15:
                    switch (op2) {
                      case 4:
                        return MISCREG_DBGBVR15_EL1;
                      case 5:
                        return MISCREG_DBGBCR15_EL1;
                      case 6:
                        return MISCREG_DBGWVR15_EL1;
                      case 7:
                        return MISCREG_DBGWCR15_EL1;
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
                      case 6:
                        return MISCREG_ID_MMFR4_EL1;
                      case 7:
                        return MISCREG_ID_ISAR6_EL1;
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
                      case 2 ... 3:
                        return MISCREG_RAZ;
                      case 4:
                        return MISCREG_ID_AA64ZFR0_EL1;
                      case 5 ... 7:
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
                      case 2:
                        return MISCREG_ID_AA64MMFR2_EL1;
                      case 3 ... 7:
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
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_ZCR_EL1;
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
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_ZCR_EL2;
                    }
                    break;
                }
                break;
              case 5:
                /* op0: 3 Crn:1 op1:5 */
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_SCTLR_EL12;
                      case 2:
                        return MISCREG_CPACR_EL12;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_ZCR_EL12;
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
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_ZCR_EL3;
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
                  case 0x1:
                    switch (op2) {
                      case 0x0:
                        return MISCREG_APIAKeyLo_EL1;
                      case 0x1:
                        return MISCREG_APIAKeyHi_EL1;
                      case 0x2:
                        return MISCREG_APIBKeyLo_EL1;
                      case 0x3:
                        return MISCREG_APIBKeyHi_EL1;
                    }
                    break;
                  case 0x2:
                    switch (op2) {
                      case 0x0:
                        return MISCREG_APDAKeyLo_EL1;
                      case 0x1:
                        return MISCREG_APDAKeyHi_EL1;
                      case 0x2:
                        return MISCREG_APDBKeyLo_EL1;
                      case 0x3:
                        return MISCREG_APDBKeyHi_EL1;
                    }
                    break;

                  case 0x3:
                    switch (op2) {
                      case 0x0:
                        return MISCREG_APGAKeyLo_EL1;
                      case 0x1:
                        return MISCREG_APGAKeyHi_EL1;
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
                      case 1:
                        return MISCREG_TTBR1_EL2;
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
                  case 6:
                    switch (op2) {
                      case 0:
                        return MISCREG_VSTTBR_EL2;
                      case 2:
                        return MISCREG_VSTCR_EL2;
                    }
                    break;
                }
                break;
              case 5:
                /* op0: 3 Crn:2 op1:5 */
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_TTBR0_EL12;
                      case 1:
                        return MISCREG_TTBR1_EL12;
                      case 2:
                        return MISCREG_TCR_EL12;
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
                      case 3:
                        return MISCREG_PAN;
                      case 4:
                        return MISCREG_UAO;
                    }
                    break;
                  case 6:
                    switch (op2) {
                      case 0:
                        return MISCREG_ICC_PMR_EL1;
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
              case 5:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_SPSR_EL12;
                      case 1:
                        return MISCREG_ELR_EL12;
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
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_ERRIDR_EL1;
                      case 1:
                        return MISCREG_ERRSELR_EL1;
                    }
                    break;
                  case 4:
                    switch (op2) {
                      case 0:
                        return MISCREG_ERXFR_EL1;
                      case 1:
                        return MISCREG_ERXCTLR_EL1;
                      case 2:
                        return MISCREG_ERXSTATUS_EL1;
                      case 3:
                        return MISCREG_ERXADDR_EL1;
                    }
                    break;
                  case 5:
                    switch (op2) {
                      case 0:
                        return MISCREG_ERXMISC0_EL1;
                      case 1:
                        return MISCREG_ERXMISC1_EL1;
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
                      case 3:
                        return MISCREG_VSESR_EL2;
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
              case 5:
                switch (crm) {
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_AFSR0_EL12;
                      case 1:
                        return MISCREG_AFSR1_EL12;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_ESR_EL12;
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
              case 5:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_FAR_EL12;
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
              case 5:
                switch (crm) {
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_MAIR_EL12;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_AMAIR_EL12;
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
                [[fallthrough]];
              default:
                // S3_<op1>_11_<Cm>_<op2>
                return MISCREG_IMPDEF_UNIMPL;
            }
            GEM5_UNREACHABLE;
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
                      case 1:
                        return MISCREG_DISR_EL1;
                    }
                    break;
                  case 8:
                    switch (op2) {
                      case 0:
                        return MISCREG_ICC_IAR0_EL1;
                      case 1:
                        return MISCREG_ICC_EOIR0_EL1;
                      case 2:
                        return MISCREG_ICC_HPPIR0_EL1;
                      case 3:
                        return MISCREG_ICC_BPR0_EL1;
                      case 4:
                        return MISCREG_ICC_AP0R0_EL1;
                      case 5:
                        return MISCREG_ICC_AP0R1_EL1;
                      case 6:
                        return MISCREG_ICC_AP0R2_EL1;
                      case 7:
                        return MISCREG_ICC_AP0R3_EL1;
                    }
                    break;
                  case 9:
                    switch (op2) {
                      case 0:
                        return MISCREG_ICC_AP1R0_EL1;
                      case 1:
                        return MISCREG_ICC_AP1R1_EL1;
                      case 2:
                        return MISCREG_ICC_AP1R2_EL1;
                      case 3:
                        return MISCREG_ICC_AP1R3_EL1;
                    }
                    break;
                  case 11:
                    switch (op2) {
                      case 1:
                        return MISCREG_ICC_DIR_EL1;
                      case 3:
                        return MISCREG_ICC_RPR_EL1;
                      case 5:
                        return MISCREG_ICC_SGI1R_EL1;
                      case 6:
                        return MISCREG_ICC_ASGI1R_EL1;
                      case 7:
                        return MISCREG_ICC_SGI0R_EL1;
                    }
                    break;
                  case 12:
                    switch (op2) {
                      case 0:
                        return MISCREG_ICC_IAR1_EL1;
                      case 1:
                        return MISCREG_ICC_EOIR1_EL1;
                      case 2:
                        return MISCREG_ICC_HPPIR1_EL1;
                      case 3:
                        return MISCREG_ICC_BPR1_EL1;
                      case 4:
                        return MISCREG_ICC_CTLR_EL1;
                      case 5:
                        return MISCREG_ICC_SRE_EL1;
                      case 6:
                        return MISCREG_ICC_IGRPEN0_EL1;
                      case 7:
                        return MISCREG_ICC_IGRPEN1_EL1;
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
                  case 1:
                    switch (op2) {
                      case 1:
                        return MISCREG_VDISR_EL2;
                    }
                    break;
                  case 8:
                    switch (op2) {
                      case 0:
                        return MISCREG_ICH_AP0R0_EL2;
                      case 1:
                        return MISCREG_ICH_AP0R1_EL2;
                      case 2:
                        return MISCREG_ICH_AP0R2_EL2;
                      case 3:
                        return MISCREG_ICH_AP0R3_EL2;
                    }
                    break;
                  case 9:
                    switch (op2) {
                      case 0:
                        return MISCREG_ICH_AP1R0_EL2;
                      case 1:
                        return MISCREG_ICH_AP1R1_EL2;
                      case 2:
                        return MISCREG_ICH_AP1R2_EL2;
                      case 3:
                        return MISCREG_ICH_AP1R3_EL2;
                      case 5:
                        return MISCREG_ICC_SRE_EL2;
                    }
                    break;
                  case 11:
                    switch (op2) {
                      case 0:
                        return MISCREG_ICH_HCR_EL2;
                      case 1:
                        return MISCREG_ICH_VTR_EL2;
                      case 2:
                        return MISCREG_ICH_MISR_EL2;
                      case 3:
                        return MISCREG_ICH_EISR_EL2;
                      case 5:
                        return MISCREG_ICH_ELRSR_EL2;
                      case 7:
                        return MISCREG_ICH_VMCR_EL2;
                    }
                    break;
                  case 12:
                    switch (op2) {
                      case 0:
                        return MISCREG_ICH_LR0_EL2;
                      case 1:
                        return MISCREG_ICH_LR1_EL2;
                      case 2:
                        return MISCREG_ICH_LR2_EL2;
                      case 3:
                        return MISCREG_ICH_LR3_EL2;
                      case 4:
                        return MISCREG_ICH_LR4_EL2;
                      case 5:
                        return MISCREG_ICH_LR5_EL2;
                      case 6:
                        return MISCREG_ICH_LR6_EL2;
                      case 7:
                        return MISCREG_ICH_LR7_EL2;
                    }
                    break;
                  case 13:
                    switch (op2) {
                      case 0:
                        return MISCREG_ICH_LR8_EL2;
                      case 1:
                        return MISCREG_ICH_LR9_EL2;
                      case 2:
                        return MISCREG_ICH_LR10_EL2;
                      case 3:
                        return MISCREG_ICH_LR11_EL2;
                      case 4:
                        return MISCREG_ICH_LR12_EL2;
                      case 5:
                        return MISCREG_ICH_LR13_EL2;
                      case 6:
                        return MISCREG_ICH_LR14_EL2;
                      case 7:
                        return MISCREG_ICH_LR15_EL2;
                    }
                    break;
                }
                break;
              case 5:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 0:
                        return MISCREG_VBAR_EL12;
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
                  case 12:
                    switch (op2) {
                      case 4:
                        return MISCREG_ICC_CTLR_EL3;
                      case 5:
                        return MISCREG_ICC_SRE_EL3;
                      case 7:
                        return MISCREG_ICC_IGRPEN1_EL3;
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
              case 5:
                switch (crm) {
                  case 0:
                    switch (op2) {
                      case 1:
                        return MISCREG_CONTEXTIDR_EL12;
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
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTHV_TVAL_EL2;
                      case 1:
                        return MISCREG_CNTHV_CTL_EL2;
                      case 2:
                        return MISCREG_CNTHV_CVAL_EL2;
                    }
                    break;
                }
                break;
              case 5:
                switch (crm) {
                  case 1:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTKCTL_EL12;
                    }
                    break;
                  case 2:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTP_TVAL_EL02;
                      case 1:
                        return MISCREG_CNTP_CTL_EL02;
                      case 2:
                        return MISCREG_CNTP_CVAL_EL02;
                    }
                    break;
                  case 3:
                    switch (op2) {
                      case 0:
                        return MISCREG_CNTV_TVAL_EL02;
                      case 1:
                        return MISCREG_CNTV_CTL_EL02;
                      case 2:
                        return MISCREG_CNTV_CVAL_EL02;
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
            // S3_<op1>_15_<Cm>_<op2>
            return MISCREG_IMPDEF_UNIMPL;
        }
        break;
    }

    return MISCREG_UNKNOWN;
}

std::bitset<NUM_MISCREG_INFOS> miscRegInfo[NUM_MISCREGS]; // initialized below

void
ISA::initializeMiscRegMetadata()
{
    // the MiscReg metadata tables are shared across all instances of the
    // ISA object, so there's no need to initialize them multiple times.
    static bool completed = false;
    if (completed)
        return;

    // This boolean variable specifies if the system is running in aarch32 at
    // EL3 (aarch32EL3 = true). It is false if EL3 is not implemented, or it
    // is running in aarch64 (aarch32EL3 = false)
    bool aarch32EL3 = release->has(ArmExtension::SECURITY) && !highestELIs64;

    // Set Privileged Access Never on taking an exception to EL1 (Arm 8.1+),
    // unsupported
    bool SPAN = false;

    // Implicit error synchronization event enable (Arm 8.2+), unsupported
    bool IESB = false;

    // Load Multiple and Store Multiple Atomicity and Ordering (Arm 8.2+),
    // unsupported
    bool LSMAOE = false;

    // No Trap Load Multiple and Store Multiple (Arm 8.2+), unsupported
    bool nTLSMD = false;

    // Pointer authentication (Arm 8.3+), unsupported
    bool EnDA = true; // using APDAKey_EL1 key of instr addrs in ELs 0,1
    bool EnDB = true; // using APDBKey_EL1 key of instr addrs in ELs 0,1
    bool EnIA = true; // using APIAKey_EL1 key of instr addrs in ELs 0,1
    bool EnIB = true; // using APIBKey_EL1 key of instr addrs in ELs 0,1

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
      .privSecure(!aarch32EL3)
      .bankedChild();
    InitReg(MISCREG_PRRR_MAIR0_S)
      .mutex()
      .bankedChild();
    InitReg(MISCREG_NMRR_MAIR1)
      .mutex()
      .banked();
    InitReg(MISCREG_NMRR_MAIR1_NS)
      .mutex()
      .privSecure(!aarch32EL3)
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
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGDTRRXext)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGDSCRext)
      .allPrivileges();
    InitReg(MISCREG_DBGDTRTXext)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGOSECCR)
      .unimplemented()
      .allPrivileges();
    InitReg(MISCREG_DBGBVR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR3)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR4)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR5)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR6)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR7)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR8)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR9)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR10)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR11)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR12)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR13)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR14)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBVR15)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR3)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR4)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR5)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR6)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR7)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR8)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR9)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR10)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR11)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR12)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR13)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR14)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBCR15)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR3)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR4)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR5)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR6)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR7)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR8)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR9)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR10)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR11)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR12)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR13)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR14)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWVR15)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR3)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR4)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR5)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR6)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR7)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR8)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR9)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR10)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR11)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR12)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR13)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR14)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGWCR15)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGDRAR)
      .unimplemented()
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGBXVR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR2)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR3)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR4)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR5)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR6)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR7)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR8)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR9)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR10)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR11)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR12)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR13)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR14)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGBXVR15)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_DBGOSLAR)
       .allPrivileges().monSecureRead(0).monNonSecureRead(0);
    InitReg(MISCREG_DBGOSLSR)
      .allPrivileges().monSecureWrite(0).monNonSecureWrite(0);
    InitReg(MISCREG_DBGOSDLR)
      .unimplemented()
      .warnNotFail()
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
    InitReg(MISCREG_ID_MMFR4)
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
    InitReg(MISCREG_ID_ISAR6)
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
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_CSSELR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_VPIDR)
      .hyp().monNonSecure();
    InitReg(MISCREG_VMPIDR)
      .hyp().monNonSecure();
    InitReg(MISCREG_SCTLR)
      .banked()
      // readMiscRegNoEffect() uses this metadata
      // despite using children (below) as backing store
      .res0(0x8d22c600)
      .res1(0x00400800 | (SPAN   ? 0 : 0x800000)
                       | (LSMAOE ? 0 :     0x10)
                       | (nTLSMD ? 0 :      0x8));
    InitReg(MISCREG_SCTLR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_SCTLR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_ACTLR)
      .banked();
    InitReg(MISCREG_ACTLR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_ACTLR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_CPACR)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_SDCR)
      .mon();
    InitReg(MISCREG_SCR)
      .mon().secure().exceptUserMode()
      .res0(0xff40)  // [31:16], [6]
      .res1(0x0030); // [5:4]
    InitReg(MISCREG_SDER)
      .mon();
    InitReg(MISCREG_NSACR)
      .allPrivileges().hypWrite(0).privNonSecureWrite(0).exceptUserMode();
    InitReg(MISCREG_HSCTLR)
      .hyp().monNonSecure()
      .res0(0x0512c7c0 | (EnDB   ? 0 :     0x2000)
                       | (IESB   ? 0 :   0x200000)
                       | (EnDA   ? 0 :  0x8000000)
                       | (EnIB   ? 0 : 0x40000000)
                       | (EnIA   ? 0 : 0x80000000))
      .res1(0x30c50830);
    InitReg(MISCREG_HACTLR)
      .hyp().monNonSecure();
    InitReg(MISCREG_HCR)
      .hyp().monNonSecure()
      .res0(0x90000000);
    InitReg(MISCREG_HCR2)
      .hyp().monNonSecure()
      .res0(0xffa9ff8c);
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
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_TTBR0_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_TTBR1)
      .banked();
    InitReg(MISCREG_TTBR1_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_TTBR1_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_TTBCR)
      .banked();
    InitReg(MISCREG_TTBCR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
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
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_DACR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_DFSR)
      .banked();
    InitReg(MISCREG_DFSR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_DFSR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_IFSR)
      .banked();
    InitReg(MISCREG_IFSR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
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
      .privSecure(!aarch32EL3)
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
      .privSecure(!aarch32EL3)
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
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_DFAR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_IFAR)
      .banked();
    InitReg(MISCREG_IFAR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
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
      .privSecure(!aarch32EL3)
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
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_PRRR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_MAIR0)
      .banked();
    InitReg(MISCREG_MAIR0_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_MAIR0_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_NMRR)
      .banked();
    InitReg(MISCREG_NMRR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_NMRR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_MAIR1)
      .banked();
    InitReg(MISCREG_MAIR1_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_MAIR1_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_AMAIR0)
      .banked();
    InitReg(MISCREG_AMAIR0_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_AMAIR0_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_AMAIR1)
      .banked();
    InitReg(MISCREG_AMAIR1_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
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
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_VBAR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_MVBAR)
      .mon().secure()
      .hypRead(FullSystem && system->highestEL() == EL2)
      .privRead(FullSystem && system->highestEL() == EL1)
      .exceptUserMode();
    InitReg(MISCREG_RMR)
      .unimplemented()
      .mon().secure().exceptUserMode();
    InitReg(MISCREG_ISR)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_HVBAR)
      .hyp().monNonSecure()
      .res0(0x1f);
    InitReg(MISCREG_FCSEIDR)
      .unimplemented()
      .warnNotFail()
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CONTEXTIDR)
      .banked();
    InitReg(MISCREG_CONTEXTIDR_NS)
      .bankedChild()
      .privSecure(!aarch32EL3)
      .nonSecure().exceptUserMode();
    InitReg(MISCREG_CONTEXTIDR_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_TPIDRURW)
      .banked();
    InitReg(MISCREG_TPIDRURW_NS)
      .bankedChild()
      .allPrivileges()
      .privSecure(!aarch32EL3)
      .monSecure(0);
    InitReg(MISCREG_TPIDRURW_S)
      .bankedChild()
      .secure();
    InitReg(MISCREG_TPIDRURO)
      .banked();
    InitReg(MISCREG_TPIDRURO_NS)
      .bankedChild()
      .allPrivileges()
      .userNonSecureWrite(0).userSecureRead(1)
      .privSecure(!aarch32EL3)
      .monSecure(0);
    InitReg(MISCREG_TPIDRURO_S)
      .bankedChild()
      .secure().userSecureWrite(0);
    InitReg(MISCREG_TPIDRPRW)
      .banked();
    InitReg(MISCREG_TPIDRPRW_NS)
      .bankedChild()
      .nonSecure().exceptUserMode()
      .privSecure(!aarch32EL3);
    InitReg(MISCREG_TPIDRPRW_S)
      .bankedChild()
      .secure().exceptUserMode();
    InitReg(MISCREG_HTPIDR)
      .hyp().monNonSecure();
    // BEGIN Generic Timer (AArch32)
    InitReg(MISCREG_CNTFRQ)
      .reads(1)
      .highest(system)
      .privSecureWrite(aarch32EL3);
    InitReg(MISCREG_CNTPCT)
      .unverifiable()
      .reads(1);
    InitReg(MISCREG_CNTVCT)
      .unverifiable()
      .reads(1);
    InitReg(MISCREG_CNTP_CTL)
      .banked();
    InitReg(MISCREG_CNTP_CTL_NS)
      .bankedChild()
      .nonSecure()
      .privSecure(!aarch32EL3)
      .res0(0xfffffff8);
    InitReg(MISCREG_CNTP_CTL_S)
      .bankedChild()
      .secure()
      .privSecure(aarch32EL3)
      .res0(0xfffffff8);
    InitReg(MISCREG_CNTP_CVAL)
      .banked();
    InitReg(MISCREG_CNTP_CVAL_NS)
      .bankedChild()
      .nonSecure()
      .privSecure(!aarch32EL3);
    InitReg(MISCREG_CNTP_CVAL_S)
      .bankedChild()
      .secure()
      .privSecure(aarch32EL3);
    InitReg(MISCREG_CNTP_TVAL)
      .banked();
    InitReg(MISCREG_CNTP_TVAL_NS)
      .bankedChild()
      .nonSecure()
      .privSecure(!aarch32EL3);
    InitReg(MISCREG_CNTP_TVAL_S)
      .bankedChild()
      .secure()
      .privSecure(aarch32EL3);
    InitReg(MISCREG_CNTV_CTL)
      .allPrivileges()
      .res0(0xfffffff8);
    InitReg(MISCREG_CNTV_CVAL)
      .allPrivileges();
    InitReg(MISCREG_CNTV_TVAL)
      .allPrivileges();
    InitReg(MISCREG_CNTKCTL)
      .allPrivileges()
      .exceptUserMode()
      .res0(0xfffdfc00);
    InitReg(MISCREG_CNTHCTL)
      .monNonSecure()
      .hyp()
      .res0(0xfffdff00);
    InitReg(MISCREG_CNTHP_CTL)
      .monNonSecure()
      .hyp()
      .res0(0xfffffff8);
    InitReg(MISCREG_CNTHP_CVAL)
      .monNonSecure()
      .hyp();
    InitReg(MISCREG_CNTHP_TVAL)
      .monNonSecure()
      .hyp();
    InitReg(MISCREG_CNTVOFF)
      .monNonSecure()
      .hyp();
    // END Generic Timer (AArch32)
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
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR0, MISCREG_DBGBXVR0);
    InitReg(MISCREG_DBGBVR1_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR1, MISCREG_DBGBXVR1);
    InitReg(MISCREG_DBGBVR2_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR2, MISCREG_DBGBXVR2);
    InitReg(MISCREG_DBGBVR3_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR3, MISCREG_DBGBXVR3);
    InitReg(MISCREG_DBGBVR4_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR4, MISCREG_DBGBXVR4);
    InitReg(MISCREG_DBGBVR5_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR5, MISCREG_DBGBXVR5);
    InitReg(MISCREG_DBGBVR6_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR6, MISCREG_DBGBXVR6);
    InitReg(MISCREG_DBGBVR7_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR7, MISCREG_DBGBXVR7);
    InitReg(MISCREG_DBGBVR8_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR8, MISCREG_DBGBXVR8);
    InitReg(MISCREG_DBGBVR9_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR9, MISCREG_DBGBXVR9);
    InitReg(MISCREG_DBGBVR10_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR10, MISCREG_DBGBXVR10);
    InitReg(MISCREG_DBGBVR11_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR11, MISCREG_DBGBXVR11);
    InitReg(MISCREG_DBGBVR12_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR12, MISCREG_DBGBXVR12);
    InitReg(MISCREG_DBGBVR13_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR13, MISCREG_DBGBXVR13);
    InitReg(MISCREG_DBGBVR14_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR14, MISCREG_DBGBXVR14);
    InitReg(MISCREG_DBGBVR15_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBVR15, MISCREG_DBGBXVR15);
    InitReg(MISCREG_DBGBCR0_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR0);
    InitReg(MISCREG_DBGBCR1_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR1);
    InitReg(MISCREG_DBGBCR2_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR2);
    InitReg(MISCREG_DBGBCR3_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR3);
    InitReg(MISCREG_DBGBCR4_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR4);
    InitReg(MISCREG_DBGBCR5_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR5);
    InitReg(MISCREG_DBGBCR6_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR6);
    InitReg(MISCREG_DBGBCR7_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR7);
    InitReg(MISCREG_DBGBCR8_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR8);
    InitReg(MISCREG_DBGBCR9_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR9);
    InitReg(MISCREG_DBGBCR10_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR10);
    InitReg(MISCREG_DBGBCR11_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR11);
    InitReg(MISCREG_DBGBCR12_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR12);
    InitReg(MISCREG_DBGBCR13_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR13);
    InitReg(MISCREG_DBGBCR14_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR14);
    InitReg(MISCREG_DBGBCR15_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGBCR15);
    InitReg(MISCREG_DBGWVR0_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR0);
    InitReg(MISCREG_DBGWVR1_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR1);
    InitReg(MISCREG_DBGWVR2_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR2);
    InitReg(MISCREG_DBGWVR3_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR3);
    InitReg(MISCREG_DBGWVR4_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR4);
    InitReg(MISCREG_DBGWVR5_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR5);
    InitReg(MISCREG_DBGWVR6_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR6);
    InitReg(MISCREG_DBGWVR7_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR7);
    InitReg(MISCREG_DBGWVR8_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR8);
    InitReg(MISCREG_DBGWVR9_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR9);
    InitReg(MISCREG_DBGWVR10_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR10);
    InitReg(MISCREG_DBGWVR11_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR11);
    InitReg(MISCREG_DBGWVR12_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR12);
    InitReg(MISCREG_DBGWVR13_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR13);
    InitReg(MISCREG_DBGWVR14_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR14);
    InitReg(MISCREG_DBGWVR15_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWVR15);
    InitReg(MISCREG_DBGWCR0_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR0);
    InitReg(MISCREG_DBGWCR1_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR1);
    InitReg(MISCREG_DBGWCR2_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR2);
    InitReg(MISCREG_DBGWCR3_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR3);
    InitReg(MISCREG_DBGWCR4_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR4);
    InitReg(MISCREG_DBGWCR5_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR5);
    InitReg(MISCREG_DBGWCR6_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR6);
    InitReg(MISCREG_DBGWCR7_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR7);
    InitReg(MISCREG_DBGWCR8_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR8);
    InitReg(MISCREG_DBGWCR9_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR9);
    InitReg(MISCREG_DBGWCR10_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR10);
    InitReg(MISCREG_DBGWCR11_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR11);
    InitReg(MISCREG_DBGWCR12_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR12);
    InitReg(MISCREG_DBGWCR13_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR13);
    InitReg(MISCREG_DBGWCR14_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR14);
    InitReg(MISCREG_DBGWCR15_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DBGWCR15);
    InitReg(MISCREG_MDCCSR_EL0)
      .allPrivileges().writes(0)
      //monSecureWrite(0).monNonSecureWrite(0)
      .mapsTo(MISCREG_DBGDSCRint);
    InitReg(MISCREG_MDDTR_EL0)
      .allPrivileges();
    InitReg(MISCREG_MDDTRTX_EL0)
      .allPrivileges();
    InitReg(MISCREG_MDDTRRX_EL0)
      .allPrivileges();
    InitReg(MISCREG_DBGVCR32_EL2)
      .hyp().mon()
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
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_PFR0);
    InitReg(MISCREG_ID_PFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_PFR1);
    InitReg(MISCREG_ID_DFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_DFR0);
    InitReg(MISCREG_ID_AFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_AFR0);
    InitReg(MISCREG_ID_MMFR0_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_MMFR0);
    InitReg(MISCREG_ID_MMFR1_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_MMFR1);
    InitReg(MISCREG_ID_MMFR2_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_MMFR2);
    InitReg(MISCREG_ID_MMFR3_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_MMFR3);
    InitReg(MISCREG_ID_MMFR4_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_MMFR4);
    InitReg(MISCREG_ID_ISAR0_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_ISAR0);
    InitReg(MISCREG_ID_ISAR1_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_ISAR1);
    InitReg(MISCREG_ID_ISAR2_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_ISAR2);
    InitReg(MISCREG_ID_ISAR3_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_ISAR3);
    InitReg(MISCREG_ID_ISAR4_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_ISAR4);
    InitReg(MISCREG_ID_ISAR5_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_ISAR5);
    InitReg(MISCREG_ID_ISAR6_EL1)
      .allPrivileges().exceptUserMode().writes(0)
      .mapsTo(MISCREG_ID_ISAR6);
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
    InitReg(MISCREG_ID_AA64MMFR2_EL1)
      .allPrivileges().exceptUserMode().writes(0);

    InitReg(MISCREG_APDAKeyHi_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APDAKeyLo_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APDBKeyHi_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APDBKeyLo_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APGAKeyHi_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APGAKeyLo_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APIAKeyHi_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APIAKeyLo_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APIBKeyHi_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_APIBKeyLo_EL1)
      .allPrivileges().exceptUserMode();

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
      .res0( 0x20440 | (EnDB   ? 0 :     0x2000)
                     | (IESB   ? 0 :   0x200000)
                     | (EnDA   ? 0 :  0x8000000)
                     | (EnIB   ? 0 : 0x40000000)
                     | (EnIA   ? 0 : 0x80000000))
      .res1(0x500800 | (SPAN   ? 0 :   0x800000)
                     | (nTLSMD ? 0 :  0x8000000)
                     | (LSMAOE ? 0 : 0x10000000))
      .mapsTo(MISCREG_SCTLR_NS);
    InitReg(MISCREG_SCTLR_EL12)
      .allPrivileges().exceptUserMode()
      .res0( 0x20440 | (EnDB   ? 0 :     0x2000)
                     | (IESB   ? 0 :   0x200000)
                     | (EnDA   ? 0 :  0x8000000)
                     | (EnIB   ? 0 : 0x40000000)
                     | (EnIA   ? 0 : 0x80000000))
      .res1(0x500800 | (SPAN   ? 0 :   0x800000)
                     | (nTLSMD ? 0 :  0x8000000)
                     | (LSMAOE ? 0 : 0x10000000))
      .mapsTo(MISCREG_SCTLR_EL1);
    InitReg(MISCREG_ACTLR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_ACTLR_NS);
    InitReg(MISCREG_CPACR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_CPACR);
    InitReg(MISCREG_CPACR_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_CPACR_EL1);
    InitReg(MISCREG_SCTLR_EL2)
      .hyp().mon()
      .res0(0x0512c7c0 | (EnDB   ? 0 :     0x2000)
                       | (IESB   ? 0 :   0x200000)
                       | (EnDA   ? 0 :  0x8000000)
                       | (EnIB   ? 0 : 0x40000000)
                       | (EnIA   ? 0 : 0x80000000))
      .res1(0x30c50830)
      .mapsTo(MISCREG_HSCTLR);
    InitReg(MISCREG_ACTLR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HACTLR);
    InitReg(MISCREG_HCR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HCR, MISCREG_HCR2);
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
      .mon()
      .res0(0x0512c7c0 | (EnDB   ? 0 :     0x2000)
                       | (IESB   ? 0 :   0x200000)
                       | (EnDA   ? 0 :  0x8000000)
                       | (EnIB   ? 0 : 0x40000000)
                       | (EnIA   ? 0 : 0x80000000))
      .res1(0x30c50830);
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
      .mon()
      .mapsTo(MISCREG_SDCR);
    InitReg(MISCREG_TTBR0_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_TTBR0_NS);
    InitReg(MISCREG_TTBR0_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_TTBR0_EL1);
    InitReg(MISCREG_TTBR1_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_TTBR1_NS);
    InitReg(MISCREG_TTBR1_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_TTBR1_EL1);
    InitReg(MISCREG_TCR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_TTBCR_NS);
    InitReg(MISCREG_TCR_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_TTBCR_NS);
    InitReg(MISCREG_TTBR0_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HTTBR);
    InitReg(MISCREG_TTBR1_EL2)
      .hyp().mon();
    InitReg(MISCREG_TCR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_HTCR);
    InitReg(MISCREG_VTTBR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_VTTBR);
    InitReg(MISCREG_VTCR_EL2)
      .hyp().mon()
      .mapsTo(MISCREG_VTCR);
    InitReg(MISCREG_VSTTBR_EL2)
      .hypSecure().mon();
    InitReg(MISCREG_VSTCR_EL2)
      .hypSecure().mon();
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
    InitReg(MISCREG_SPSR_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_SPSR_SVC);
    InitReg(MISCREG_ELR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ELR_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_ELR_EL1);
    InitReg(MISCREG_SP_EL0)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_SPSEL)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_CURRENTEL)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_PAN)
      .allPrivileges().exceptUserMode()
      .implemented(release->has(ArmExtension::FEAT_PAN));
    InitReg(MISCREG_UAO)
      .allPrivileges().exceptUserMode();
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
    InitReg(MISCREG_AFSR0_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_ADFSR_NS);
    InitReg(MISCREG_AFSR1_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_AIFSR_NS);
    InitReg(MISCREG_AFSR1_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_AIFSR_NS);
    InitReg(MISCREG_ESR_EL1)
      .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ESR_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_ESR_EL1);
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
      .hyp().mon().mapsTo(MISCREG_FPEXC);
    InitReg(MISCREG_AFSR0_EL3)
      .mon();
    InitReg(MISCREG_AFSR1_EL3)
      .mon();
    InitReg(MISCREG_ESR_EL3)
      .mon();
    InitReg(MISCREG_FAR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_DFAR_NS, MISCREG_IFAR_NS);
    InitReg(MISCREG_FAR_EL12)
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
    InitReg(MISCREG_MAIR_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_PRRR_NS, MISCREG_NMRR_NS);
    InitReg(MISCREG_AMAIR_EL1)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_AMAIR0_NS, MISCREG_AMAIR1_NS);
    InitReg(MISCREG_AMAIR_EL12)
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
    InitReg(MISCREG_VBAR_EL12)
      .allPrivileges().exceptUserMode()
      .mapsTo(MISCREG_VBAR_NS);
    InitReg(MISCREG_RVBAR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ISR_EL1)
      .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_VBAR_EL2)
      .hyp().mon()
      .res0(0x7ff)
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
    InitReg(MISCREG_CONTEXTIDR_EL12)
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
    // BEGIN Generic Timer (AArch64)
    InitReg(MISCREG_CNTFRQ_EL0)
      .reads(1)
      .highest(system)
      .privSecureWrite(aarch32EL3)
      .mapsTo(MISCREG_CNTFRQ);
    InitReg(MISCREG_CNTPCT_EL0)
      .unverifiable()
      .reads(1)
      .mapsTo(MISCREG_CNTPCT);
    InitReg(MISCREG_CNTVCT_EL0)
      .unverifiable()
      .reads(1)
      .mapsTo(MISCREG_CNTVCT);
    InitReg(MISCREG_CNTP_CTL_EL0)
      .allPrivileges()
      .res0(0xfffffffffffffff8)
      .mapsTo(MISCREG_CNTP_CTL_NS);
    InitReg(MISCREG_CNTP_CVAL_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_CNTP_CVAL_NS);
    InitReg(MISCREG_CNTP_TVAL_EL0)
      .allPrivileges()
      .res0(0xffffffff00000000)
      .mapsTo(MISCREG_CNTP_TVAL_NS);
    InitReg(MISCREG_CNTV_CTL_EL0)
      .allPrivileges()
      .res0(0xfffffffffffffff8)
      .mapsTo(MISCREG_CNTV_CTL);
    InitReg(MISCREG_CNTV_CVAL_EL0)
      .allPrivileges()
      .mapsTo(MISCREG_CNTV_CVAL);
    InitReg(MISCREG_CNTV_TVAL_EL0)
      .allPrivileges()
      .res0(0xffffffff00000000)
      .mapsTo(MISCREG_CNTV_TVAL);
    InitReg(MISCREG_CNTP_CTL_EL02)
      .monE2H()
      .hypE2H()
      .res0(0xfffffffffffffff8)
      .mapsTo(MISCREG_CNTP_CTL_NS);
    InitReg(MISCREG_CNTP_CVAL_EL02)
      .monE2H()
      .hypE2H()
      .mapsTo(MISCREG_CNTP_CVAL_NS);
    InitReg(MISCREG_CNTP_TVAL_EL02)
      .monE2H()
      .hypE2H()
      .res0(0xffffffff00000000)
      .mapsTo(MISCREG_CNTP_TVAL_NS);
    InitReg(MISCREG_CNTV_CTL_EL02)
      .monE2H()
      .hypE2H()
      .res0(0xfffffffffffffff8)
      .mapsTo(MISCREG_CNTV_CTL);
    InitReg(MISCREG_CNTV_CVAL_EL02)
      .monE2H()
      .hypE2H()
      .mapsTo(MISCREG_CNTV_CVAL);
    InitReg(MISCREG_CNTV_TVAL_EL02)
      .monE2H()
      .hypE2H()
      .res0(0xffffffff00000000)
      .mapsTo(MISCREG_CNTV_TVAL);
    InitReg(MISCREG_CNTKCTL_EL1)
      .allPrivileges()
      .exceptUserMode()
      .res0(0xfffffffffffdfc00)
      .mapsTo(MISCREG_CNTKCTL);
    InitReg(MISCREG_CNTKCTL_EL12)
      .monE2H()
      .hypE2H()
      .res0(0xfffffffffffdfc00)
      .mapsTo(MISCREG_CNTKCTL);
    InitReg(MISCREG_CNTPS_CTL_EL1)
      .mon()
      .privSecure()
      .res0(0xfffffffffffffff8);
    InitReg(MISCREG_CNTPS_CVAL_EL1)
      .mon()
      .privSecure();
    InitReg(MISCREG_CNTPS_TVAL_EL1)
      .mon()
      .privSecure()
      .res0(0xffffffff00000000);
    InitReg(MISCREG_CNTHCTL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffc0000)
      .mapsTo(MISCREG_CNTHCTL);
    InitReg(MISCREG_CNTHP_CTL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffffff8)
      .mapsTo(MISCREG_CNTHP_CTL);
    InitReg(MISCREG_CNTHP_CVAL_EL2)
      .mon()
      .hyp()
      .mapsTo(MISCREG_CNTHP_CVAL);
    InitReg(MISCREG_CNTHP_TVAL_EL2)
      .mon()
      .hyp()
      .res0(0xffffffff00000000)
      .mapsTo(MISCREG_CNTHP_TVAL);
    InitReg(MISCREG_CNTHPS_CTL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffffff8)
      .unimplemented();
    InitReg(MISCREG_CNTHPS_CVAL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffffff8)
      .unimplemented();
    InitReg(MISCREG_CNTHPS_TVAL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffffff8)
      .unimplemented();
    InitReg(MISCREG_CNTHV_CTL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffffff8);
    InitReg(MISCREG_CNTHV_CVAL_EL2)
      .mon()
      .hyp();
    InitReg(MISCREG_CNTHV_TVAL_EL2)
      .mon()
      .hyp()
      .res0(0xffffffff00000000);
    InitReg(MISCREG_CNTHVS_CTL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffffff8)
      .unimplemented();
    InitReg(MISCREG_CNTHVS_CVAL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffffff8)
      .unimplemented();
    InitReg(MISCREG_CNTHVS_TVAL_EL2)
      .mon()
      .hyp()
      .res0(0xfffffffffffffff8)
      .unimplemented();
    // ENDIF Armv8.1-VHE
    InitReg(MISCREG_CNTVOFF_EL2)
      .mon()
      .hyp()
      .mapsTo(MISCREG_CNTVOFF);
    // END Generic Timer (AArch64)
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

    // GICv3 AArch64
    InitReg(MISCREG_ICC_PMR_EL1)
        .res0(0xffffff00) // [31:8]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_PMR);
    InitReg(MISCREG_ICC_IAR0_EL1)
        .allPrivileges().exceptUserMode().writes(0)
        .mapsTo(MISCREG_ICC_IAR0);
    InitReg(MISCREG_ICC_EOIR0_EL1)
        .allPrivileges().exceptUserMode().reads(0)
        .mapsTo(MISCREG_ICC_EOIR0);
    InitReg(MISCREG_ICC_HPPIR0_EL1)
        .allPrivileges().exceptUserMode().writes(0)
        .mapsTo(MISCREG_ICC_HPPIR0);
    InitReg(MISCREG_ICC_BPR0_EL1)
        .res0(0xfffffff8) // [31:3]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_BPR0);
    InitReg(MISCREG_ICC_AP0R0_EL1)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP0R0);
    InitReg(MISCREG_ICC_AP0R1_EL1)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP0R1);
    InitReg(MISCREG_ICC_AP0R2_EL1)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP0R2);
    InitReg(MISCREG_ICC_AP0R3_EL1)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP0R3);
    InitReg(MISCREG_ICC_AP1R0_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_AP1R0);
    InitReg(MISCREG_ICC_AP1R0_EL1_NS)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R0_NS);
    InitReg(MISCREG_ICC_AP1R0_EL1_S)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R0_S);
    InitReg(MISCREG_ICC_AP1R1_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_AP1R1);
    InitReg(MISCREG_ICC_AP1R1_EL1_NS)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R1_NS);
    InitReg(MISCREG_ICC_AP1R1_EL1_S)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R1_S);
    InitReg(MISCREG_ICC_AP1R2_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_AP1R2);
    InitReg(MISCREG_ICC_AP1R2_EL1_NS)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R2_NS);
    InitReg(MISCREG_ICC_AP1R2_EL1_S)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R2_S);
    InitReg(MISCREG_ICC_AP1R3_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_AP1R3);
    InitReg(MISCREG_ICC_AP1R3_EL1_NS)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R3_NS);
    InitReg(MISCREG_ICC_AP1R3_EL1_S)
        .bankedChild()
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_AP1R3_S);
    InitReg(MISCREG_ICC_DIR_EL1)
        .res0(0xFF000000) // [31:24]
        .allPrivileges().exceptUserMode().reads(0)
        .mapsTo(MISCREG_ICC_DIR);
    InitReg(MISCREG_ICC_RPR_EL1)
        .allPrivileges().exceptUserMode().writes(0)
        .mapsTo(MISCREG_ICC_RPR);
    InitReg(MISCREG_ICC_SGI1R_EL1)
        .allPrivileges().exceptUserMode().reads(0)
        .mapsTo(MISCREG_ICC_SGI1R);
    InitReg(MISCREG_ICC_ASGI1R_EL1)
        .allPrivileges().exceptUserMode().reads(0)
        .mapsTo(MISCREG_ICC_ASGI1R);
    InitReg(MISCREG_ICC_SGI0R_EL1)
        .allPrivileges().exceptUserMode().reads(0)
        .mapsTo(MISCREG_ICC_SGI0R);
    InitReg(MISCREG_ICC_IAR1_EL1)
        .allPrivileges().exceptUserMode().writes(0)
        .mapsTo(MISCREG_ICC_IAR1);
    InitReg(MISCREG_ICC_EOIR1_EL1)
        .res0(0xFF000000) // [31:24]
        .allPrivileges().exceptUserMode().reads(0)
        .mapsTo(MISCREG_ICC_EOIR1);
    InitReg(MISCREG_ICC_HPPIR1_EL1)
        .allPrivileges().exceptUserMode().writes(0)
        .mapsTo(MISCREG_ICC_HPPIR1);
    InitReg(MISCREG_ICC_BPR1_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_BPR1);
    InitReg(MISCREG_ICC_BPR1_EL1_NS)
        .bankedChild()
        .res0(0xfffffff8) // [31:3]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_BPR1_NS);
    InitReg(MISCREG_ICC_BPR1_EL1_S)
        .bankedChild()
        .res0(0xfffffff8) // [31:3]
        .secure().exceptUserMode()
        .mapsTo(MISCREG_ICC_BPR1_S);
    InitReg(MISCREG_ICC_CTLR_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_CTLR);
    InitReg(MISCREG_ICC_CTLR_EL1_NS)
        .bankedChild()
        .res0(0xFFFB00BC) // [31:19, 17:16, 7, 5:2]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_CTLR_NS);
    InitReg(MISCREG_ICC_CTLR_EL1_S)
        .bankedChild()
        .res0(0xFFFB00BC) // [31:19, 17:16, 7, 5:2]
        .secure().exceptUserMode()
        .mapsTo(MISCREG_ICC_CTLR_S);
    InitReg(MISCREG_ICC_SRE_EL1)
        .banked()
        .mapsTo(MISCREG_ICC_SRE);
    InitReg(MISCREG_ICC_SRE_EL1_NS)
        .bankedChild()
        .res0(0xFFFFFFF8) // [31:3]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_SRE_NS);
    InitReg(MISCREG_ICC_SRE_EL1_S)
        .bankedChild()
        .res0(0xFFFFFFF8) // [31:3]
        .secure().exceptUserMode()
        .mapsTo(MISCREG_ICC_SRE_S);
    InitReg(MISCREG_ICC_IGRPEN0_EL1)
        .res0(0xFFFFFFFE) // [31:1]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_IGRPEN0);
    InitReg(MISCREG_ICC_IGRPEN1_EL1)
        .banked64()
        .mapsTo(MISCREG_ICC_IGRPEN1);
    InitReg(MISCREG_ICC_IGRPEN1_EL1_NS)
        .bankedChild()
        .res0(0xFFFFFFFE) // [31:1]
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_IGRPEN1_NS);
    InitReg(MISCREG_ICC_IGRPEN1_EL1_S)
        .bankedChild()
        .res0(0xFFFFFFFE) // [31:1]
        .secure().exceptUserMode()
        .mapsTo(MISCREG_ICC_IGRPEN1_S);
    InitReg(MISCREG_ICC_SRE_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICC_HSRE);
    InitReg(MISCREG_ICC_CTLR_EL3)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_MCTLR);
    InitReg(MISCREG_ICC_SRE_EL3)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_MSRE);
    InitReg(MISCREG_ICC_IGRPEN1_EL3)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ICC_MGRPEN1);

    InitReg(MISCREG_ICH_AP0R0_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_AP0R0);
    InitReg(MISCREG_ICH_AP0R1_EL2)
        .hyp().mon()
        .unimplemented()
        .mapsTo(MISCREG_ICH_AP0R1);
    InitReg(MISCREG_ICH_AP0R2_EL2)
        .hyp().mon()
        .unimplemented()
        .mapsTo(MISCREG_ICH_AP0R2);
    InitReg(MISCREG_ICH_AP0R3_EL2)
        .hyp().mon()
        .unimplemented()
        .mapsTo(MISCREG_ICH_AP0R3);
    InitReg(MISCREG_ICH_AP1R0_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_AP1R0);
    InitReg(MISCREG_ICH_AP1R1_EL2)
        .hyp().mon()
        .unimplemented()
        .mapsTo(MISCREG_ICH_AP1R1);
    InitReg(MISCREG_ICH_AP1R2_EL2)
        .hyp().mon()
        .unimplemented()
        .mapsTo(MISCREG_ICH_AP1R2);
    InitReg(MISCREG_ICH_AP1R3_EL2)
        .hyp().mon()
        .unimplemented()
        .mapsTo(MISCREG_ICH_AP1R3);
    InitReg(MISCREG_ICH_HCR_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_HCR);
    InitReg(MISCREG_ICH_VTR_EL2)
        .hyp().mon().writes(0)
        .mapsTo(MISCREG_ICH_VTR);
    InitReg(MISCREG_ICH_MISR_EL2)
        .hyp().mon().writes(0)
        .mapsTo(MISCREG_ICH_MISR);
    InitReg(MISCREG_ICH_EISR_EL2)
        .hyp().mon().writes(0)
        .mapsTo(MISCREG_ICH_EISR);
    InitReg(MISCREG_ICH_ELRSR_EL2)
        .hyp().mon().writes(0)
        .mapsTo(MISCREG_ICH_ELRSR);
    InitReg(MISCREG_ICH_VMCR_EL2)
        .hyp().mon()
        .mapsTo(MISCREG_ICH_VMCR);
    InitReg(MISCREG_ICH_LR0_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR1_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR2_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR3_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR4_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR5_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR6_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR7_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR8_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR9_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR10_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR11_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR12_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR13_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR14_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICH_LR15_EL2)
        .hyp().mon()
        .allPrivileges().exceptUserMode();

    // GICv3 AArch32
    InitReg(MISCREG_ICC_AP0R0)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP0R1)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP0R2)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP0R3)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R0)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R0_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R0_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R1)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R1_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R1_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R2)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R2_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R2_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R3)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R3_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_AP1R3_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_ASGI1R)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_BPR0)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_BPR1)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_BPR1_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_BPR1_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_CTLR)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_CTLR_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_CTLR_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_DIR)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_EOIR0)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_EOIR1)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_HPPIR0)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ICC_HPPIR1)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ICC_HSRE)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_IAR0)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ICC_IAR1)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ICC_IGRPEN0)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_IGRPEN1)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_IGRPEN1_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_IGRPEN1_S)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_MCTLR)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_MGRPEN1)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_MSRE)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_PMR)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_RPR)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ICC_SGI0R)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_SGI1R)
        .allPrivileges().exceptUserMode().reads(0);
    InitReg(MISCREG_ICC_SRE)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_SRE_NS)
        .allPrivileges().exceptUserMode();
    InitReg(MISCREG_ICC_SRE_S)
        .allPrivileges().exceptUserMode();

    InitReg(MISCREG_ICH_AP0R0)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP0R1)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP0R2)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP0R3)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP1R0)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP1R1)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP1R2)
        .hyp().mon();
    InitReg(MISCREG_ICH_AP1R3)
        .hyp().mon();
    InitReg(MISCREG_ICH_HCR)
        .hyp().mon();
    InitReg(MISCREG_ICH_VTR)
        .hyp().mon().writes(0);
    InitReg(MISCREG_ICH_MISR)
        .hyp().mon().writes(0);
    InitReg(MISCREG_ICH_EISR)
        .hyp().mon().writes(0);
    InitReg(MISCREG_ICH_ELRSR)
        .hyp().mon().writes(0);
    InitReg(MISCREG_ICH_VMCR)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR0)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR1)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR2)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR3)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR4)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR5)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR6)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR7)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR8)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR9)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR10)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR11)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR12)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR13)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR14)
        .hyp().mon();
    InitReg(MISCREG_ICH_LR15)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC0)
        .mapsTo(MISCREG_ICH_LR0)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC1)
        .mapsTo(MISCREG_ICH_LR1)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC2)
        .mapsTo(MISCREG_ICH_LR2)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC3)
        .mapsTo(MISCREG_ICH_LR3)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC4)
        .mapsTo(MISCREG_ICH_LR4)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC5)
        .mapsTo(MISCREG_ICH_LR5)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC6)
        .mapsTo(MISCREG_ICH_LR6)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC7)
        .mapsTo(MISCREG_ICH_LR7)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC8)
        .mapsTo(MISCREG_ICH_LR8)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC9)
        .mapsTo(MISCREG_ICH_LR9)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC10)
        .mapsTo(MISCREG_ICH_LR10)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC11)
        .mapsTo(MISCREG_ICH_LR11)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC12)
        .mapsTo(MISCREG_ICH_LR12)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC13)
        .mapsTo(MISCREG_ICH_LR13)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC14)
        .mapsTo(MISCREG_ICH_LR14)
        .hyp().mon();
    InitReg(MISCREG_ICH_LRC15)
        .mapsTo(MISCREG_ICH_LR15)
        .hyp().mon();

    // SVE
    InitReg(MISCREG_ID_AA64ZFR0_EL1)
        .allPrivileges().exceptUserMode().writes(0);
    InitReg(MISCREG_ZCR_EL3)
        .mon();
    InitReg(MISCREG_ZCR_EL2)
        .hyp().mon();
    InitReg(MISCREG_ZCR_EL12)
        .allPrivileges().exceptUserMode()
        .mapsTo(MISCREG_ZCR_EL1);
    InitReg(MISCREG_ZCR_EL1)
        .allPrivileges().exceptUserMode();

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
    InitReg(MISCREG_UNKNOWN);
    InitReg(MISCREG_IMPDEF_UNIMPL)
      .unimplemented()
      .warnNotFail(impdefAsNop);

    // RAS extension (unimplemented)
    InitReg(MISCREG_ERRIDR_EL1)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_ERRSELR_EL1)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_ERXFR_EL1)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_ERXCTLR_EL1)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_ERXSTATUS_EL1)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_ERXADDR_EL1)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_ERXMISC0_EL1)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_ERXMISC1_EL1)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_DISR_EL1)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_VSESR_EL2)
      .unimplemented()
      .warnNotFail();
    InitReg(MISCREG_VDISR_EL2)
      .unimplemented()
      .warnNotFail();

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
} // namespace gem5
