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

#ifndef __ARCH_MIPS_ISA_TRAITS_HH__
#define __ARCH_MIPS_ISA_TRAITS_HH__

//#include "arch/mips/misc_regfile.hh"
#include "arch/mips/faults.hh"
#include "base/misc.hh"
#include "config/full_system.hh"
#include "sim/byteswap.hh"
#include "sim/host.hh"
#include "sim/faults.hh"

#include <vector>

class FastCPU;
class FullCPU;
class Checkpoint;
class ExecContext;

namespace LittleEndianGuest {};

#define TARGET_MIPS

class StaticInst;
class StaticInstPtr;

namespace MIPS34K {
int DTB_ASN_ASN(uint64_t reg);
int ITB_ASN_ASN(uint64_t reg);
};

#if !FULL_SYSTEM
class SyscallReturn {
        public:
           template <class T>
           SyscallReturn(T v, bool s)
           {
               retval = (uint32_t)v;
               success = s;
           }

           template <class T>
           SyscallReturn(T v)
           {
               success = (v >= 0);
               retval = (uint32_t)v;
           }

           ~SyscallReturn() {}

           SyscallReturn& operator=(const SyscallReturn& s) {
               retval = s.retval;
               success = s.success;
               return *this;
           }

           bool successful() { return success; }
           uint64_t value() { return retval; }


       private:
           uint64_t retval;
           bool success;
};
#endif

namespace MipsISA
{
    using namespace LittleEndianGuest;

    typedef uint32_t MachInst;
    typedef uint32_t MachInst;
    typedef uint64_t ExtMachInst;
    typedef uint8_t  RegIndex;
//  typedef uint64_t Addr;

       // Constants Related to the number of registers

    const int NumIntArchRegs = 32;
    const int NumPALShadowRegs = 8;
    const int NumFloatArchRegs = 32;
    // @todo: Figure out what this number really should be.
    const int NumMiscArchRegs = 265;

    const int NumIntRegs = NumIntArchRegs + NumPALShadowRegs;
    const int NumFloatRegs = NumFloatArchRegs;
    const int NumMiscRegs = NumMiscArchRegs;

    const int TotalNumRegs = NumIntRegs + NumFloatRegs +
    NumMiscRegs + 0/*NumInternalProcRegs*/;

    const int TotalDataRegs = NumIntRegs + NumFloatRegs;

    // Static instruction parameters
    const int MaxInstSrcRegs = 3;
    const int MaxInstDestRegs = 2;

    // semantically meaningful register indices
    const int ZeroReg = 0;
    const int AssemblerReg = 1;
    const int ReturnValueReg1 = 2;
    const int ReturnValueReg2 = 3;
    const int ArgumentReg0 = 4;
    const int ArgumentReg1 = 5;
    const int ArgumentReg2 = 6;
    const int ArgumentReg3 = 7;
    const int KernelReg0 = 26;
    const int KernelReg1 = 27;
    const int GlobalPointerReg = 28;
    const int StackPointerReg = 29;
    const int FramePointerReg = 30;
    const int ReturnAddressReg = 31;

    const int SyscallNumReg = ReturnValueReg1;
    const int SyscallPseudoReturnReg = ReturnValueReg1;
    const int SyscallSuccessReg = ArgumentReg3;

    const int LogVMPageSize = 13;	// 8K bytes
    const int VMPageSize = (1 << LogVMPageSize);

    const int BranchPredAddrShiftAmt = 2; // instructions are 4-byte aligned

    const int MachineBytes = 4;
    const int WordBytes = 4;
    const int HalfwordBytes = 2;
    const int ByteBytes = 1;


    // These enumerate all the registers for dependence tracking.
    enum DependenceTags {
        // 0..31 are the integer regs 0..31
        // 32..63 are the FP regs 0..31, i.e. use (reg + FP_Base_DepTag)
        FP_Base_DepTag = 32,
        Ctrl_Base_DepTag = 64,
        Fpcr_DepTag = 64,		// floating point control register
        Uniq_DepTag = 65,
        IPR_Base_DepTag = 66,
        MiscReg_DepTag = 67
    };

    typedef uint64_t IntReg;
    class IntRegFile
    {
      protected:
        IntReg regs[NumIntRegs];

      public:
        IntReg readReg(int intReg)
        {
            return regs[intReg];
        }

        Fault setReg(int intReg, const IntReg &val)
        {
            regs[intReg] = val;
            return NoFault;
        }

        void serialize(std::ostream &os);

        void unserialize(Checkpoint *cp, const std::string &section);

    };

    typedef float FloatReg;

    typedef uint32_t FloatReg32;
    typedef uint64_t FloatReg64;
    typedef uint64_t FloatRegBits;

//  const uint64_t hi_mask64 = 0xFFFFFFFF00000000;
//const uint64_t lo_mask64 = 0x00000000FFFFFFFF;

    const int SingleWidth = 32;
    const int SingleBytes = 4;

    const int DoubleWidth = 64;
    const int DoubleBytes = 8;

    const int QuadWidth = 128;
    const int QuadBytes = QuadWidth / 4;

    class FloatRegFile
    {
      protected:

        //Since the floating point registers overlap each other,
        //A generic storage space is used. The float to be returned is
        //pulled from the appropriate section of this region.
        //char regSpace[SingleBytes * NumFloatRegs];
        FloatReg32 regs[NumFloatRegs];

      public:

        void clear()
        {
            bzero(regs, sizeof(regs));
        }

        double readReg(int floatReg, int width)
        {
            using namespace std;

            switch(width)
            {
              case SingleWidth:
                void *float_ptr = &regs[floatReg];
                cout << "reading as float, reg." << floatReg << ": " << *(float *) float_ptr << endl;
                cout << "reading as uint32_t, reg." << floatReg << ": " << *(uint32_t *) float_ptr << endl;
                return *(float *) float_ptr;

              case DoubleWidth:
                void *double_ptr = &regs[floatReg];
                cout << "reading as double, reg." << floatReg <<": " << *(double *) double_ptr << endl;
                cout << "reading as uint64_t, reg." << floatReg << hex << ": 0x" << *(uint64_t *) float_ptr << endl;
                return *(double *) double_ptr;

              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }
        }

        FloatRegBits readRegBits(int floatReg, int width)
        {
            using namespace std;

            switch(width)
            {
              case SingleWidth:
                return regs[floatReg];

              case DoubleWidth:
                cout << hex << "Combining " << regs[floatReg + 1] << " & " << regs[floatReg + 1] << endl;
                cout << hex << "Returning " << ((FloatReg64)regs[floatReg] << 32 | regs[floatReg + 1]) << endl;
                return (FloatReg64)regs[floatReg] << 32 | regs[floatReg + 1];

              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }
        }

        Fault setReg(int floatReg, const FloatReg &val, int width)
        {

            switch(width)
            {
              case SingleWidth:
                float temp = val;
                void *float_ptr = &temp;
                regs[floatReg] = *(FloatReg32 *) float_ptr;
                break;

              case DoubleWidth:
                const void *double_ptr = &val;
                FloatReg64 temp_double = *(FloatReg64 *) double_ptr;
                regs[floatReg] = temp_double >> 32;
                regs[floatReg + 1] = temp_double;
                break;

              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }

            return NoFault;
        }

        Fault setRegBits(int floatReg, const FloatRegBits &val, int width)
        {
            using namespace std;

            switch(width)
            {
              case SingleWidth:
                regs[floatReg] = val;
                break;

              case DoubleWidth:
                cout << hex << "Setting val: " << val << endl;
                regs[floatReg] = val >> 32;
                regs[floatReg + 1] = val;
                cout << dec << "f" << floatReg << ": " << hex<< readRegBits(floatReg,32) << endl;
                cout << dec << "f" << floatReg + 1 << ": " << hex << readRegBits(floatReg+1,32) << endl;
                break;

              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }
            return NoFault;
        }

        void serialize(std::ostream &os);

        void unserialize(Checkpoint *cp, const std::string &section);
    };

        enum ConvertType{
            SINGLE_TO_DOUBLE,
            SINGLE_TO_WORD,
            SINGLE_TO_LONG,

            DOUBLE_TO_SINGLE,
            DOUBLE_TO_WORD,
            DOUBLE_TO_LONG,

            LONG_TO_SINGLE,
            LONG_TO_DOUBLE,
            LONG_TO_WORD,

            WORD_TO_SINGLE,
            WORD_TO_DOUBLE,
            WORD_TO_LONG,

            PLOWER_TO_SINGLE,
            PUPPER_TO_SINGLE
        };

        enum RoundMode{
            RND_ZERO,
            RND_DOWN,
            RND_UP,
            RND_NEAREST
        };

        uint64_t convert_and_round(uint32_t fp_val,ConvertType cvt_type, int rnd_mode = 0);
        uint64_t convert_and_round(uint64_t fp_val,ConvertType cvt_type, int rnd_mode = 0);
        uint64_t convert_and_round(float fp_val,ConvertType cvt_type, int rnd_mode = 0);

        void copyRegs(ExecContext *src, ExecContext *dest);

    // cop-0/cop-1 system control register file
    typedef uint64_t MiscReg;
//typedef MiscReg MiscRegFile[NumMiscRegs];
    class MiscRegFile {

      protected:
        uint64_t	fpcr;		// floating point condition codes
        uint64_t	uniq;		// process-unique register
        bool		lock_flag;	// lock flag for LL/SC
        Addr		lock_addr;	// lock address for LL/SC

        MiscReg miscRegFile[NumMiscRegs];

      public:
        //These functions should be removed once the simplescalar cpu model
        //has been replaced.
        int getInstAsid();
        int getDataAsid();

        void copyMiscRegs(ExecContext *xc);

        MiscReg readReg(int misc_reg)
        { return miscRegFile[misc_reg]; }

        MiscReg readRegWithEffect(int misc_reg, Fault &fault, ExecContext *xc)
        { return miscRegFile[misc_reg];}

        Fault setReg(int misc_reg, const MiscReg &val)
        { miscRegFile[misc_reg] = val; return NoFault; }

        Fault setRegWithEffect(int misc_reg, const MiscReg &val,
                               ExecContext *xc)
        { miscRegFile[misc_reg] = val; return NoFault; }

#if FULL_SYSTEM
        void clearIprs() { }

      protected:
        InternalProcReg ipr[NumInternalProcRegs]; // Internal processor regs

      private:
        MiscReg readIpr(int idx, Fault &fault, ExecContext *xc) { }

        Fault setIpr(int idx, uint64_t val, ExecContext *xc) { }
#endif
        friend class RegFile;
    };

    enum MiscRegTags {
        //Coprocessor 0 Registers
        //Reference MIPS32 Arch. for Programmers, Vol. III, Ch.8
        //(Register Number-Register Select) Summary of Register
        //------------------------------------------------------
        Index = 0,       //0-0 Index into the TLB array

        MVPControl = 1,  //0-1 Per-processor register containing global
                     //MIPS® MT configuration data

        MVPConf0 = 2,    //0-2 Per-processor register containing global
                     //MIPS® MT configuration data

        MVPConf1 = 3,    //0-3 Per-processor register containing global
                     //MIPS® MT configuration data

        Random = 8,      //1-0 Randomly generated index into the TLB array

        VPEControl = 9,  //1-1 Per-VPE register containing relatively volatile
                     //thread configuration data

        VPEConf0 = 10,    //1-2 Per-VPE multi-thread configuration
                     //information


        VPEConf1 = 11,    //1-2 Per-VPE multi-thread configuration
                     //information

        YQMask = 12,      //Per-VPE register defining which YIELD
                     //qualifier bits may be used without generating
                     //an exception

        VPESchedule = 13,
        VPEScheFBack =  14,
        VPEOpt = 15,
        EntryLo0 = 16, // Bank 3: 16 - 23
        TCStatus = 17,
        TCBind = 18,
        TCRestart = 19,
        TCHalt = 20,
        TCContext = 21,
        TCSchedule = 22,
        TCScheFBack = 23,

        EntryLo1 = 24,// Bank 4: 24 - 31

        Context = 32, // Bank 5: 32 - 39
        ContextConfig = 33,

        //PageMask = 40, //Bank 6: 40 - 47
        PageGrain = 41,

        Wired = 48, //Bank 7:48 - 55
        SRSConf0 = 49,
        SRSConf1 = 50,
        SRSConf2 = 51,
        SRSConf3 = 52,
        SRSConf4 = 53,
        BadVAddr = 54,

        HWRena = 56,//Bank 8:56 - 63

        Count = 64, //Bank 9:64 - 71

        EntryHi = 72,//Bank 10:72 - 79

        Compare = 80,//Bank 11:80 - 87

        Status = 88,//Bank 12:88 - 96     //12-0 Processor status and control
        IntCtl = 89,                      //12-1 Interrupt system status and control
        SRSCtl = 90,                      //12-2 Shadow register set status and control
        SRSMap = 91,                      //12-3 Shadow set IPL mapping

        Cause = 97,//97-104      //13-0 Cause of last general exception

        EPC = 105,//105-112        //14-0 Program counter at last exception

        PRId = 113,//113-120,       //15-0 Processor identification and revision
        EBase = 114,      //15-1 Exception vector base register

        Config = 121,//Bank 16: 121-128
        Config1 = 122,
        Config2 = 123,
        Config3 = 124,
        Config6 = 127,
        Config7 = 128,


        LLAddr = 129,//Bank 17: 129-136

        WatchLo0 = 137,//Bank 18: 137-144
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

        XCContext64 = 153,//Bank 20: 153-160

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

        DESAVE = 257,//Bank 31: 249-256

        //More Misc. Regs
        Hi,
        Lo,
        FIR,
        FCSR,
        FPCR,

        //Alpha Regs, but here now, for
        //compiling sake
        UNIQ,
        LockAddr,
        LockFlag
    };

extern const Addr PageShift;
extern const Addr PageBytes;
extern const Addr PageMask;
extern const Addr PageOffset;

#if FULL_SYSTEM

    typedef uint64_t InternalProcReg;

#include "arch/mips/isa_fullsys_traits.hh"

#else
    enum {
        NumInternalProcRegs = 0
    };
#endif

    typedef union {
        IntReg  intreg;
        FloatReg   fpreg;
        MiscReg ctrlreg;
    } AnyReg;

    class RegFile {
      protected:
        IntRegFile intRegFile;		// (signed) integer register file
        FloatRegFile floatRegFile;	// floating point register file
        MiscRegFile miscRegFile;	// control register file

      public:

        void clear()
        {
            bzero(&intRegFile, sizeof(intRegFile));
            bzero(&floatRegFile, sizeof(floatRegFile));
            bzero(&miscRegFile, sizeof(miscRegFile));
        }

        MiscReg readMiscReg(int miscReg)
        {
            return miscRegFile.readReg(miscReg);
        }

        MiscReg readMiscRegWithEffect(int miscReg,
                Fault &fault, ExecContext *xc)
        {
            fault = NoFault;
            return miscRegFile.readRegWithEffect(miscReg, fault, xc);
        }

        Fault setMiscReg(int miscReg, const MiscReg &val)
        {
            return miscRegFile.setReg(miscReg, val);
        }

        Fault setMiscRegWithEffect(int miscReg, const MiscReg &val,
                ExecContext * xc)
        {
            return miscRegFile.setRegWithEffect(miscReg, val, xc);
        }

        FloatReg readFloatReg(int floatReg)
        {
            return floatRegFile.readReg(floatReg,SingleWidth);
        }

        FloatReg readFloatReg(int floatReg, int width)
        {
            return floatRegFile.readReg(floatReg,width);
        }

        FloatRegBits readFloatRegBits(int floatReg)
        {
            return floatRegFile.readRegBits(floatReg,SingleWidth);
        }

        FloatRegBits readFloatRegBits(int floatReg, int width)
        {
            return floatRegFile.readRegBits(floatReg,width);
        }

        Fault setFloatReg(int floatReg, const FloatReg &val)
        {
            return floatRegFile.setReg(floatReg, val, SingleWidth);
        }

        Fault setFloatReg(int floatReg, const FloatReg &val, int width)
        {
            return floatRegFile.setReg(floatReg, val, width);
        }

        Fault setFloatRegBits(int floatReg, const FloatRegBits &val)
        {
            return floatRegFile.setRegBits(floatReg, val, SingleWidth);
        }

        Fault setFloatRegBits(int floatReg, const FloatRegBits &val, int width)
        {
            return floatRegFile.setRegBits(floatReg, val, width);
        }

        IntReg readIntReg(int intReg)
        {
            return intRegFile.readReg(intReg);
        }

        Fault setIntReg(int intReg, const IntReg &val)
        {
            return intRegFile.setReg(intReg, val);
        }
      protected:

        Addr pc;			// program counter
        Addr npc;			// next-cycle program counter
        Addr nnpc;			// next-next-cycle program counter
                                        // used to implement branch delay slot
                                        // not real register
      public:
        Addr readPC()
        {
            return pc;
        }

        void setPC(Addr val)
        {
            pc = val;
        }

        Addr readNextPC()
        {
            return npc;
        }

        void setNextPC(Addr val)
        {
            npc = val;
        }

        Addr readNextNPC()
        {
            return nnpc;
        }

        void setNextNPC(Addr val)
        {
            nnpc = val;
        }

        MiscReg hi;                     // MIPS HI Register
        MiscReg lo;                     // MIPS LO Register


#if FULL_SYSTEM
        IntReg palregs[NumIntRegs];	// PAL shadow registers
        InternalProcReg ipr[NumInternalProcRegs]; // internal processor regs
        int intrflag;			// interrupt flag
        bool pal_shadow;		// using pal_shadow registers
        inline int instAsid() { return MIPS34K::ITB_ASN_ASN(ipr[IPR_ITB_ASN]); }
        inline int dataAsid() { return MIPS34K::DTB_ASN_ASN(ipr[IPR_DTB_ASN]); }
#endif // FULL_SYSTEM

        //void initCP0Regs();
        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);

        void createCP0Regs();
        void coldReset();

        typedef int ContextParam;
        typedef int ContextVal;

        void changeContext(ContextParam param, ContextVal val)
        {
        }
    };

    StaticInstPtr decodeInst(ExtMachInst);

    // return a no-op instruction... used for instruction fetch faults
    extern const MachInst NoopMachInst;

    enum annotes {
        ANNOTE_NONE = 0,
        // An impossible number for instruction annotations
        ITOUCH_ANNOTE = 0xffffffff,
    };

//void getMiscRegIdx(int reg_name,int &idx, int &sel);

    static inline ExtMachInst
    makeExtMI(MachInst inst, const uint64_t &pc) {
#if FULL_SYSTEM
        ExtMachInst ext_inst = inst;
        if (pc && 0x1)
            return ext_inst|=(static_cast<ExtMachInst>(pc & 0x1) << 32);
        else
            return ext_inst;
#else
        return ExtMachInst(inst);
#endif
    }

    static inline bool isCallerSaveIntegerRegister(unsigned int reg) {
        panic("register classification not implemented");
        return (reg >= 1 && reg <= 8 || reg >= 22 && reg <= 25 || reg == 27);
    }

    static inline bool isCalleeSaveIntegerRegister(unsigned int reg) {
        panic("register classification not implemented");
        return (reg >= 9 && reg <= 15);
    }

    static inline bool isCallerSaveFloatRegister(unsigned int reg) {
        panic("register classification not implemented");
        return false;
    }

    static inline bool isCalleeSaveFloatRegister(unsigned int reg) {
        panic("register classification not implemented");
        return false;
    }

    static inline Addr alignAddress(const Addr &addr,
                                         unsigned int nbytes) {
        return (addr & ~(nbytes - 1));
    }

    // Instruction address compression hooks
    static inline Addr realPCToFetchPC(const Addr &addr) {
        return addr;
    }

    static inline Addr fetchPCToRealPC(const Addr &addr) {
        return addr;
    }

    // the size of "fetched" instructions (not necessarily the size
    // of real instructions for PISA)
    static inline size_t fetchInstSize() {
        return sizeof(MachInst);
    }

    static inline MachInst makeRegisterCopy(int dest, int src) {
        panic("makeRegisterCopy not implemented");
        return 0;
    }

    static inline void setSyscallReturn(SyscallReturn return_value, RegFile *regs)
    {
        if (return_value.successful()) {
            // no error
            regs->setIntReg(SyscallSuccessReg, 0);
            regs->setIntReg(ReturnValueReg1, return_value.value());
        } else {
            // got an error, return details
            regs->setIntReg(SyscallSuccessReg, (IntReg) -1);
            regs->setIntReg(ReturnValueReg1, -return_value.value());
        }
    }

    // Machine operations

    void saveMachineReg(AnyReg &savereg, const RegFile &reg_file,
                               int regnum);

    void restoreMachineReg(RegFile &regs, const AnyReg &reg,
                                  int regnum);

#if 0
    static void serializeSpecialRegs(const Serializable::Proxy &proxy,
                                     const RegFile &regs);

    static void unserializeSpecialRegs(const IniFile *db,
                                       const std::string &category,
                                       ConfigNode *node,
                                       RegFile &regs);
#endif

    /**
     * Function to insure ISA semantics about 0 registers.
     * @param xc The execution context.
     */
    template <class XC>
    void zeroRegisters(XC *xc);

    const Addr MaxAddr = (Addr)-1;
};

#if FULL_SYSTEM
//typedef TheISA::InternalProcReg InternalProcReg;
//const int NumInternalProcRegs  = TheISA::NumInternalProcRegs;
//const int NumInterruptLevels = TheISA::NumInterruptLevels;

#include "arch/mips/mips34k.hh"
#endif

using namespace MipsISA;

#endif // __ARCH_MIPS_ISA_TRAITS_HH__
