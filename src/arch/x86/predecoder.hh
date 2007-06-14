/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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
 */

#ifndef __ARCH_X86_PREDECODER_HH__
#define __ARCH_X86_PREDECODER_HH__

#include "arch/x86/types.hh"
#include "base/bitfield.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "sim/host.hh"

class ThreadContext;

namespace X86ISA
{
    class Predecoder
    {
      private:
        //These are defined and documented in predecoder_tables.cc
        static const uint8_t Prefixes[256];
        static const uint8_t UsesModRM[2][256];
        static const uint8_t ImmediateType[2][256];
        static const uint8_t SizeTypeToSize[3][10];

      protected:
        ThreadContext * tc;
        //The bytes to be predecoded
        MachInst fetchChunk;
        //The pc of the start of fetchChunk
        Addr basePC;
        //The pc the current instruction started at
        Addr origPC;
        //The offset into fetchChunk of current processing
        int offset;
        //The extended machine instruction being generated
        ExtMachInst emi;

        inline uint8_t getNextByte()
        {
            return (fetchChunk >> (offset * 8)) & 0xff;
        }

        void getImmediate(int &collected, uint64_t &current, int size)
        {
            //Figure out how many bytes we still need to get for the
            //immediate.
            int toGet = size - collected;
            //Figure out how many bytes are left in our "buffer"
            int remaining = sizeof(MachInst) - offset;
            //Get as much as we need, up to the amount available.
            toGet = toGet > remaining ? remaining : toGet;

            //Shift the bytes we want to be all the way to the right
            uint64_t partialDisp = fetchChunk >> (offset * 8);
            //Mask off what we don't want
            partialDisp &= mask(toGet * 8);
            //Shift it over to overlay with our displacement.
            partialDisp <<= (displacementCollected * 8);
            //Put it into our displacement
            current |= partialDisp;
            //Update how many bytes we've collected.
            collected += toGet;
            consumeBytes(toGet);
        }

        inline void consumeByte()
        {
            offset++;
            assert(offset <= sizeof(MachInst));
            if(offset == sizeof(MachInst))
                outOfBytes = true;
        }

        inline void consumeBytes(int numBytes)
        {
            offset += numBytes;
            assert(offset <= sizeof(MachInst));
            if(offset == sizeof(MachInst))
                outOfBytes = true;
        }

        void reset();

        //State machine state
      protected:
        //Whether or not we're out of bytes
        bool outOfBytes;
        //Whether we've completed generating an ExtMachInst
        bool emiIsReady;
        //The size of the displacement value
        int displacementSize;
        int displacementCollected;
        //The size of the immediate value
        int immediateSize;
        int immediateCollected;

        enum State {
            ResetState,
            PrefixState,
            OpcodeState,
            ModRMState,
            SIBState,
            DisplacementState,
            ImmediateState,
            //We should never get to this state. Getting here is an error.
            ErrorState
        };

        State state;

        //Functions to handle each of the states
        State doPrefixState(uint8_t);
        State doOpcodeState(uint8_t);
        State doModRMState(uint8_t);
        State doSIBState(uint8_t);
        State doDisplacementState();
        State doImmediateState();

      public:
        Predecoder(ThreadContext * _tc) :
            tc(_tc), basePC(0), origPC(0), offset(0),
            outOfBytes(true), emiIsReady(false),
            state(ResetState)
        {
            emi.mode.mode = LongMode;
            emi.mode.submode = SixtyFourBitMode;
        }

        ThreadContext * getTC()
        {
            return tc;
        }

        void setTC(ThreadContext * _tc)
        {
            tc = _tc;
        }

        void process();

        //Use this to give data to the predecoder. This should be used
        //when there is control flow.
        void moreBytes(Addr pc, Addr fetchPC, Addr off, MachInst data)
        {
            basePC = fetchPC;
            offset = off;
            fetchChunk = data;
            assert(off < sizeof(MachInst));
            outOfBytes = false;
            process();
        }

        bool needMoreBytes()
        {
            return outOfBytes;
        }

        bool extMachInstReady()
        {
            return emiIsReady;
        }

        //This returns a constant reference to the ExtMachInst to avoid a copy
        const ExtMachInst & getExtMachInst()
        {
            assert(emiIsReady);
            emiIsReady = false;
            return emi;
        }

        int getInstSize()
        {
            DPRINTF(Predecoder,
                    "Calculating the instruction size: "
                    "basePC: %#x offset: %#x origPC: %#x\n",
                    basePC, offset, origPC);
            return basePC + offset - origPC;
        }
    };
};

#endif // __ARCH_X86_PREDECODER_HH__
