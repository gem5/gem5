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

#include "arch/mips/constants.hh"
#include "arch/mips/types.hh"
#include "arch/mips/regfile.hh"
#include "arch/mips/faults.hh"
#include "arch/mips/utility.hh"
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

    StaticInstPtr decodeInst(ExtMachInst);

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

    /**
     * Function to insure ISA semantics about 0 registers.
     * @param xc The execution context.
     */
    template <class XC>
    void zeroRegisters(XC *xc);

    const Addr MaxAddr = (Addr)-1;

    void copyRegs(ExecContext *src, ExecContext *dest);

    uint64_t fpConvert(double fp_val, ConvertType cvt_type);

    float roundFP(float val);
    double roundFP(double val);
    float roundFP(uint64_t val);

    float truncFP(float val);
    double truncFP(uint64_t val);
    double truncFP(double val);

    bool unorderedFP(float val);
    bool unorderedFP(double val);
    bool getFPConditionCode(int cc);
    void setFPConditionCode(int num, bool val);

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

};

#if FULL_SYSTEM

#include "arch/mips/mips34k.hh"

#endif

using namespace MipsISA;

#endif // __ARCH_MIPS_ISA_TRAITS_HH__
