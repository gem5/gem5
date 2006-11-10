/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Korey Sewell
 */

#ifndef __ARCH_MIPS_REGFILE_MISC_REGFILE_HH__
#define __ARCH_MIPS_REGFILE_MISC_REGFILE_HH__

#include "arch/mips/types.hh"
#include "sim/faults.hh"

class ThreadContext;

namespace MipsISA
{
    static inline std::string getMiscRegName(RegIndex)
    {
        return "";
    }

    //Coprocessor 0 Register Names
    enum MiscRegTags {
        //Reference MIPS32 Arch. for Programmers, Vol. III, Ch.8
        //(Register Number-Register Select) Summary of Register
        //------------------------------------------------------
        Index = 0,       //Bank 0: 0 - 3
        MVPControl,
        MVPConf0,
        MVPConf1,

        Random = 8,      //Bank 1: 8 - 15
        VPEControl,
        VPEConf0,
        VPEConf1,
        YQMask,
        VPESchedule,
        VPEScheFBack,
        VPEOpt,

        EntryLo0 = 16,   //Bank 2: 16 - 23
        TCStatus,
        TCBind,
        TCRestart,
        TCHalt,
        TCContext,
        TCSchedule,
        TCScheFBack,

        EntryLo1 = 24,   // Bank 3: 24

        Context = 32,    // Bank 4: 32 - 33
        ContextConfig,

        //PageMask = 40, //Bank 5: 40 - 41
        PageGrain = 41,

        Wired = 48,      //Bank 6: 48 - 55
        SRSConf0,
        SRSConf1,
        SRSConf2,
        SRSConf3,
        SRSConf4,

        HWRena = 56,     //Bank 7: 56

        BadVAddr = 63,   //Bank 8: 63

        Count = 64,      //Bank 9: 64

        EntryHi = 72,   //Bank 10:72 - 79

        Compare = 80,   //Bank 10:80 - 87

        Status = 88,    //Bank 12:88 - 96
        IntCtl = 89,
        SRSCtl = 90,
        SRSMap = 91,

        Cause = 97,     //97-104

        EPC = 105,      //105-112

        PRId = 113,     //113-120,
        EBase = 114,

        Config = 121,   //Bank 16: 121-128
        Config1 = 122,
        Config2 = 123,
        Config3 = 124,
        Config6 = 127,
        Config7 = 128,


        LLAddr = 129,   //Bank 17: 129-136

        WatchLo0 = 137, //Bank 18: 137-144
        WatchLo1 = 138,
        WatchLo2 = 139,
        WatchLo3 = 140,
        WatchLo4 = 141,
        WatchLo5 = 142,
        WatchLo6 = 143,
        WatchLo7 = 144,

        WatchHi0 = 145,//Bank 19: 145-152
        WatchHi1 = 146,
        WatchHi2 = 147,
        WatchHi3 = 148,
        WatchHi4 = 149,
        WatchHi5 = 150,
        WatchHi6 = 151,
        WatchHi7 = 152,

        XCContext64 = 153, //Bank 20: 153-160

        //Bank 21: 161-168

        //Bank 22: 169-176

        Debug = 177, //Bank 23: 177-184
        TraceControl1 = 178,
        TraceControl2 = 179,
        UserTraceData = 180,
        TraceBPC = 181,

        DEPC = 185,//Bank 24: 185-192

        PerfCnt0 = 193,//Bank 25: 193 - 200
        PerfCnt1 = 194,
        PerfCnt2 = 195,
        PerfCnt3 = 196,
        PerfCnt4 = 197,
        PerfCnt5 = 198,
        PerfCnt6 = 199,
        PerfCnt7 = 200,

        ErrCtl = 201, //Bank 26: 201 - 208

        CacheErr0 = 209, //Bank 27: 209 - 216
        CacheErr1 = 210,
        CacheErr2 = 211,
        CacheErr3 = 212,

        TagLo0 = 217,//Bank 28: 217 - 224
        DataLo1 = 218,
        TagLo2 = 219,
        DataLo3 = 220,
        TagLo4 = 221,
        DataLo5 = 222,
        TagLo6 = 223,
        DataLo7 = 234,

        TagHi0 = 233,//Bank 29: 233 - 240
        DataHi1 = 234,
        TagHi2 = 235,
        DataHi3 = 236,
        TagHi4 = 237,
        DataHi5 = 238,
        TagHi6 = 239,
        DataHi7 = 240,


        ErrorEPC = 249,//Bank 30: 241 - 248

        DESAVE = 257//Bank 31: 249-256
    };

    class MiscRegFile {

      protected:
        uint64_t	fpcr;		// floating point condition codes
                                        // FPCR is not used in MIPS. Condition
                                        // codes are kept as part of the FloatRegFile

        bool		lock_flag;	// lock flag for LL/SC
                                        // use LL reg. in the future

        Addr		lock_addr;	// lock address for LL/SC
                                        // use LLAddr reg. in the future

        MiscReg miscRegFile[NumMiscRegs];

      public:
        void clear()
        {
            fpcr = 0;
            lock_flag = 0;
            lock_addr = 0;
        }

        void copyMiscRegs(ThreadContext *tc);

        MiscReg readReg(int misc_reg)
        {
            return miscRegFile[misc_reg];
        }

        MiscReg readRegWithEffect(int misc_reg, ThreadContext *tc)
        {
            return miscRegFile[misc_reg];
        }

        void setReg(int misc_reg, const MiscReg &val)
        {
            miscRegFile[misc_reg] = val;
        }

        void setRegWithEffect(int misc_reg, const MiscReg &val,
                               ThreadContext *tc)
        {
            miscRegFile[misc_reg] = val;
        }

        friend class RegFile;
    };
} // namespace MipsISA

#endif
