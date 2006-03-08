/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "arch/mips/isa_traits.hh"
#include "config/full_system.hh"
#include "cpu/static_inst.hh"
#include "sim/serialize.hh"

using namespace MipsISA;


//Function now Obsolete in current state.
//If anyting this should return the correct miscreg index
//but that is handled implicitly with enums anyway
void
MipsISA::getMiscRegIdx(int reg_name,int &idx, int &sel)
{
    switch(reg_name)
    {
      case Index: idx = 0; sel = 0; break;           //0-0 Index into the TLB array
      case MVPControl:    idx = 0; sel = 1; break;   //0-1 Per-processor register containing global
      case MVPConf0:     idx = 0; sel = 2; break;    //0-2 Per-processor register containing global
      case MVPConf1:     idx = 0; sel = 3; break;    //0-3 Per-processor register containing global
      case Random:       idx = 1; sel = 3; break;    //1-0 Randomly generated index into the TLB array
      case VPEControl:  idx = 1; sel = 1; break;     //1-1 Per-VPE register containing relatively volatile
        //thread configuration data
      case VPEConf0:     idx = 1; sel = 2; break;    //1-2 Per-VPE multi-thread configuration
        //information
      case VPEConf1:     idx = 1; sel = 3; break;    //1-3 Per-VPE multi-thread configuration
        //information
      case YQMask:       idx = 1; sel = 4; break;    //Per-VPE register defining which YIELD
        //qualifier bits may be used without generating
        //an exception
      case VPESchedule:   idx = 1; sel = 5; break;
      case VPEScheFBack:  idx = 1; sel = 6; break;
      case VPEOpt:        idx = 1; sel = 7; break;
      case EntryLo0:  idx = 1; sel = 5; break;
      case TCStatus:  idx = 1; sel = 5; break;
      case TCBind:  idx = 1; sel = 5; break;
      case TCRestart:  idx = 1; sel = 5; break;
      case TCHalt:  idx = 1; sel = 5; break;
      case TCContext:  idx = 1; sel = 5; break;
      case TCSchedule:  idx = 1; sel = 5; break;
      case TCScheFBack: panic("Accessing Unimplemented CP0 Register"); break;
      case EntryLo1: panic("Accessing Unimplemented CP0 Register"); break;
      case Context: panic("Accessing Unimplemented CP0 Register"); break;
      case ContextConfig: panic("Accessing Unimplemented CP0 Register"); break;
        //case PageMask: panic("Accessing Unimplemented CP0 Register"); break;
      case PageGrain: panic("Accessing Unimplemented CP0 Register"); break;
      case Wired: panic("Accessing Unimplemented CP0 Register"); break;
      case SRSConf0: panic("Accessing Unimplemented CP0 Register"); break;
      case SRSConf1: panic("Accessing Unimplemented CP0 Register"); break;
      case SRSConf2: panic("Accessing Unimplemented CP0 Register"); break;
      case SRSConf3: panic("Accessing Unimplemented CP0 Register"); break;
      case SRSConf4: panic("Accessing Unimplemented CP0 Register"); break;
      case BadVAddr: panic("Accessing Unimplemented CP0 Register"); break;
      case Count: panic("Accessing Unimplemented CP0 Register"); break;
      case EntryHi: panic("Accessing Unimplemented CP0 Register"); break;
      case Compare: panic("Accessing Unimplemented CP0 Register"); break;
      case Status:  idx = 12; sel = 0; break;      //12-0 Processor status and control
      case IntCtl:  idx = 12; sel = 1; break;      //12-1 Interrupt system status and control
      case SRSCtl:  idx = 12; sel = 2; break;      //12-2 Shadow register set status and control
      case SRSMap:  idx = 12; sel = 3; break;      //12-3 Shadow set IPL mapping
      case Cause:  idx = 13; sel = 0; break;       //13-0 Cause of last general exception
      case EPC:  idx = 14; sel = 0; break;         //14-0 Program counter at last exception
      case PrId:  idx = 15; sel = 0; break;        //15-0 Processor identification and revision
      case EBase:  idx = 15; sel = 1; break;       //15-1 Exception vector base register
      case Config: panic("Accessing Unimplemented CP0 Register"); break;
      case Config1: panic("Accessing Unimplemented CP0 Register"); break;
      case Config2: panic("Accessing Unimplemented CP0 Register"); break;
      case Config3: panic("Accessing Unimplemented CP0 Register"); break;
      case LLAddr: panic("Accessing Unimplemented CP0 Register"); break;
      case WatchLo: panic("Accessing Unimplemented CP0 Register"); break;
      case WatchHi: panic("Accessing Unimplemented CP0 Register"); break;
      case Debug: panic("Accessing Unimplemented CP0 Register"); break;
      case TraceControl1: panic("Accessing Unimplemented CP0 Register"); break;
      case TraceControl2: panic("Accessing Unimplemented CP0 Register"); break;
      case UserTraceData: panic("Accessing Unimplemented CP0 Register"); break;
      case TraceBPC: panic("Accessing Unimplemented CP0 Register"); break;
      case DEPC: panic("Accessing Unimplemented CP0 Register"); break;
      case PerfCnt: panic("Accessing Unimplemented CP0 Register"); break;
      case ErrCtl: panic("Accessing Unimplemented CP0 Register"); break;
      case CacheErr0: panic("Accessing Unimplemented CP0 Register"); break;
      case CacheErr1: panic("Accessing Unimplemented CP0 Register"); break;
      case CacheErr2: panic("Accessing Unimplemented CP0 Register"); break;
      case CacheErr3: panic("Accessing Unimplemented CP0 Register"); break;
      case TagLo: panic("Accessing Unimplemented CP0 Register"); break;
      case DataLo: panic("Accessing Unimplemented CP0 Register"); break;
      case TagHi: panic("Accessing Unimplemented CP0 Register"); break;
      case DataHi: panic("Accessing Unimplemented CP0 Register"); break;
      case ErrorEPC: panic("Accessing Unimplemented CP0 Register"); break;

      default:
        panic("Accessing Unimplemented Misc. Register");
    }
}

void RegFile::coldReset()
{
    //CP0 Random Reg:
    //Randomly generated index into the TLB array
    miscRegs[Random] = 0x0000003f;

    //CP0 Wired Reg.
    miscRegs[Wired] = 0x0000000;

   //CP0 HWRENA
    miscRegs[HWRena] = 0x0000000;

    //CP0 Status Reg.
    miscRegs[Status] = 0x0400004;

   //CP0 INTCNTL
    miscRegs[IntCtl] = 0xfc00000;

   //CP0 SRSCNTL
    miscRegs[SRSCtl] = 0x0c00000;

   //CP0 SRSMAP
    miscRegs[SRSMap] = 0x0000000;

   //CP0 Cause
    miscRegs[Cause] = 0x0000000;

    //CP0 Processor ID
    miscRegs[PrId] = 0x0019300;

    //CP0 EBASE
    miscRegs[EBase] = 0x8000000;

    //CP0 Config Reg.
    miscRegs[Config] = 0x80040482;

    //CP0 Config 1 Reg.
    miscRegs[Config1] = 0xfee3719e;

    //CP0 Config 2 Reg.
    miscRegs[Config2] = 0x8000000;

    //CP0 Config 3 Reg.
    miscRegs[Config3] = 0x0000020;

    //CP0 Config 7 Reg.
    miscRegs[Config7] = 0x0000000;

   //CP0 Debug
    miscRegs[Debug] = 0x0201800;

   //CP0 PERFCNTL1
    miscRegs[PerfCnt0] = 0x0000000;

   //CP0 PERFCNTL2
    miscRegs[PerfCnt1] = 0x0000000;

}

void RegFile::createCP0Regs()
{
//Resize Coprocessor Register Banks to
// the number specified in MIPS32K VOL.III
// Chapter 8
    /*
    //Cop-0 Regs. Bank 0: Index,
    miscRegs[0].resize(4);

    //Cop-0 Regs. Bank 1:
    miscRegs[1].resize(8);

    //Cop-0 Regs. Bank 2:
    miscRegs[2].resize(8);

    //Cop-0 Regs. Bank 3:
    miscRegs[3].resize(1);

    //Cop-0 Regs. Bank 4:
    miscRegs[4].resize(2);

    //Cop-0 Regs. Bank 5:
    miscRegs[5].resize(2);

    //Cop-0 Regs. Bank 6:
    miscRegs[6].resize(6);

    //Cop-0 Regs. Bank 7:
    miscRegs[7].resize(1);

    //Cop-0 Regs. Bank 8:
    miscRegs[8].resize(1);

    //Cop-0 Regs. Bank 9:
    miscRegs[9].resize(1);

    //Cop-0 Regs. Bank 10:
    miscRegs[10].resize(1);

    //Cop-0 Regs. Bank 11:
    miscRegs[11].resize(1);

    //Cop-0 Regs. Bank 12:
    miscRegs[12].resize(4);

    //Cop-0 Regs. Bank 13:
    miscRegs[13].resize(1);

    //Cop-0 Regs. Bank 14:
    miscRegs[14].resize(1);

    //Cop-0 Regs. Bank 15:
    miscRegs[15].resize(2);

    //Cop-0 Regs. Bank 16:
    miscRegs[16].resize(4);

    //Cop-0 Regs. Bank 17:
    miscRegs[17].resize(1);

    //Cop-0 Regs. Bank 18:
    miscRegs[18].resize(8);

    //Cop-0 Regs. Bank 19:
    miscRegs[19].resize(8);

    //Cop-0 Regs. Bank 20:
    miscRegs[20].resize(1);

    //Cop-0 Regs. Bank 21:
    //miscRegs[21].resize(1);
    //Reserved for future extensions

    //Cop-0 Regs. Bank 22:
    //miscRegs[22].resize(4);
    //Available for implementation dependent use

    //Cop-0 Regs. Bank 23:
    miscRegs[23].resize(5);

    //Cop-0 Regs. Bank 24:
    miscRegs[24].resize(1);

    //Cop-0 Regs. Bank 25:
    miscRegs[25].resize(8);

    //Cop-0 Regs. Bank 26:
    miscRegs[26].resize(1);

    //Cop-0 Regs. Bank 27:
    miscRegs[27].resize(4);

    //Cop-0 Regs. Bank 28:
    miscRegs[28].resize(8);

    //Cop-0 Regs. Bank 29:
    miscRegs[29].resize(8);

    //Cop-0 Regs. Bank 30:
    miscRegs[30].resize(1);

    //Cop-0 Regs. Bank 31:
    miscRegs[31].resize(1);*/

}


const Addr MipsISA::PageShift = 13;
const Addr MipsISA::PageBytes = ULL(1) << PageShift;
const Addr MipsISA::PageMask = ~(PageBytes - 1);
const Addr MipsISA::PageOffset = PageBytes - 1;

#if FULL_SYSTEM

////////////////////////////////////////////////////////////////////////
//
//  Translation stuff
//

const Addr MipsISA::PteShift = 3;
const Addr MipsISA::NPtePageShift = PageShift - PteShift;
const Addr MipsISA::NPtePage = ULL(1) << NPtePageShift;
const Addr MipsISA::PteMask = NPtePage - 1;

// User Virtual
const Addr MipsISA::USegBase = ULL(0x0);
const Addr MipsISA::USegEnd = ULL(0x000003ffffffffff);

// Kernel Direct Mapped
const Addr MipsISA::K0SegBase = ULL(0xfffffc0000000000);
const Addr MipsISA::K0SegEnd = ULL(0xfffffdffffffffff);

// Kernel Virtual
const Addr MipsISA::K1SegBase = ULL(0xfffffe0000000000);
const Addr MipsISA::K1SegEnd = ULL(0xffffffffffffffff);

#endif

// Mips UNOP (sll r0,r0,r0)
const MachInst MipsISA::NoopMachInst = 0x00000000;

static inline Addr
TruncPage(Addr addr)
{ return addr & ~(MipsISA::PageBytes - 1); }

static inline Addr
RoundPage(Addr addr)
{ return (addr + MipsISA::PageBytes - 1) & ~(MipsISA::PageBytes - 1); }

void
RegFile::serialize(std::ostream &os)
{
    SERIALIZE_ARRAY(intRegFile, NumIntRegs);
    SERIALIZE_ARRAY(floatRegFile.q, NumFloatRegs);
    //SERIALIZE_SCALAR(miscRegs.fpcr);
    //SERIALIZE_SCALAR(miscRegs.uniq);
    //SERIALIZE_SCALAR(miscRegs.lock_flag);
    //SERIALIZE_SCALAR(miscRegs.lock_addr);
    SERIALIZE_SCALAR(pc);
    SERIALIZE_SCALAR(npc);
    SERIALIZE_SCALAR(nnpc);
#if FULL_SYSTEM
    SERIALIZE_ARRAY(palregs, NumIntRegs);
    SERIALIZE_ARRAY(ipr, NumInternalProcRegs);
    SERIALIZE_SCALAR(intrflag);
    SERIALIZE_SCALAR(pal_shadow);
#endif
}


void
RegFile::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(intRegFile, NumIntRegs);
    UNSERIALIZE_ARRAY(floatRegFile.q, NumFloatRegs);
    //UNSERIALIZE_SCALAR(miscRegs.fpcr);
    //UNSERIALIZE_SCALAR(miscRegs.uniq);
    //UNSERIALIZE_SCALAR(miscRegs.lock_flag);
    //UNSERIALIZE_SCALAR(miscRegs.lock_addr);
    UNSERIALIZE_SCALAR(pc);
    UNSERIALIZE_SCALAR(npc);
    UNSERIALIZE_SCALAR(nnpc);
#if FULL_SYSTEM
    UNSERIALIZE_ARRAY(palregs, NumIntRegs);
    UNSERIALIZE_ARRAY(ipr, NumInternalProcRegs);
    UNSERIALIZE_SCALAR(intrflag);
    UNSERIALIZE_SCALAR(pal_shadow);
#endif
}


#if FULL_SYSTEM
void
PTE::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(tag);
    SERIALIZE_SCALAR(ppn);
    SERIALIZE_SCALAR(xre);
    SERIALIZE_SCALAR(xwe);
    SERIALIZE_SCALAR(asn);
    SERIALIZE_SCALAR(asma);
    SERIALIZE_SCALAR(fonr);
    SERIALIZE_SCALAR(fonw);
    SERIALIZE_SCALAR(valid);
}


void
PTE::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(tag);
    UNSERIALIZE_SCALAR(ppn);
    UNSERIALIZE_SCALAR(xre);
    UNSERIALIZE_SCALAR(xwe);
    UNSERIALIZE_SCALAR(asn);
    UNSERIALIZE_SCALAR(asma);
    UNSERIALIZE_SCALAR(fonr);
    UNSERIALIZE_SCALAR(fonw);
    UNSERIALIZE_SCALAR(valid);
}

#endif //FULL_SYSTEM
