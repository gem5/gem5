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
 *
 * Authors: Gabe Black
 *          Ali Saidi
 */

#ifndef __ARCH_SPARC_INTREGFILE_HH__
#define __ARCH_SPARC_INTREGFILE_HH__

#include "arch/sparc/faults.hh"
#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/types.hh"

#include <string>

namespace SparcISA
{
    class RegFile;

    //This function translates integer register file indices into names
    std::string getIntRegName(RegIndex);

    class IntRegFile
    {
      private:
        friend class RegFile;
      protected:
        static const int FrameOffsetBits = 3;
        static const int FrameNumBits = 2;

        static const int RegsPerFrame = 1 << FrameOffsetBits;
        static const int FrameNumMask =
                (FrameNumBits == sizeof(int)) ?
                (unsigned int)(-1) :
                (1 << FrameNumBits) - 1;
        static const int FrameOffsetMask =
                (FrameOffsetBits == sizeof(int)) ?
                (unsigned int)(-1) :
                (1 << FrameOffsetBits) - 1;

        IntReg regGlobals[MaxGL][RegsPerFrame];
        IntReg regSegments[2 * NWindows][RegsPerFrame];
        IntReg microRegs[NumMicroIntRegs];

        enum regFrame {Globals, Outputs, Locals, Inputs, NumFrames};

        IntReg * regView[NumFrames];

        static const int RegGlobalOffset = 0;
        static const int FrameOffset = MaxGL * RegsPerFrame;
        int offset[NumFrames];

      public:

        int flattenIndex(int reg);

        void clear();

        IntRegFile();

        IntReg readReg(int intReg);

        Fault setReg(int intReg, const IntReg &val);

        void serialize(std::ostream &os);

        void unserialize(Checkpoint *cp, const std::string &section);

      protected:
        //This doesn't effect the actual CWP register.
        //It's purpose is to adjust the view of the register file
        //to what it would be if CWP = cwp.
        void setCWP(int cwp);

        void setGlobals(int gl);
    };
}

#endif
