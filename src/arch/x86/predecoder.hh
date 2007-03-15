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
        static const uint8_t ImmediateTypeToSize[3][10];

      protected:
        ThreadContext * tc;
        //The bytes to be predecoded
        MachInst fetchChunk;
        //The pc of the start of fetchChunk
        Addr basePC;
        //The offset into fetchChunk of current processing
        int offset;
        //The extended machine instruction being generated
        ExtMachInst emi;

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

        //These are local to some of the states. I need to turn the states
        //into inline functions to clean things up a bit.
        int toGet;
        int remaining;
        MachInst partialDisp;

        enum State {
            Prefix,
            Opcode,
            ModRM,
            SIB,
            Displacement,
            Immediate
        };

        State state;

      public:
        Predecoder(ThreadContext * _tc) :
            tc(_tc), basePC(0), offset(0),
            outOfBytes(true), emiIsReady(false),
            state(Prefix)
        {}

        ThreadContext * getTC()
        {
            return tc;
        }

        void setTC(ThreadContext * _tc)
        {
            tc = _tc;
        }

        void process()
        {
            assert(!outOfBytes);
            assert(!emiIsReady);
            while(!emiIsReady && !outOfBytes)
            {
                uint8_t nextByte = (fetchChunk >> (offset * 8)) & 0xff;
                switch(state)
                {
                  case Prefix:
                    uint8_t prefix = Prefixes[nextByte];
                    if(prefix)
                        offset++;
                    switch(prefix)
                    {
                        //Operand size override prefixes
                      case OperandSizeOverride:
                        DPRINTF(Predecoder, "Found operand size override prefix.\n");
                        break;
                      case AddressSizeOverride:
                        DPRINTF(Predecoder, "Found address size override prefix.\n");
                        break;
                        //Segment override prefixes
                      case CSOverride:
                        DPRINTF(Predecoder, "Found cs segment override.\n");
                        break;
                      case DSOverride:
                        DPRINTF(Predecoder, "Found ds segment override.\n");
                        break;
                      case ESOverride:
                        DPRINTF(Predecoder, "Found es segment override.\n");
                        break;
                      case FSOverride:
                        DPRINTF(Predecoder, "Found fs segment override.\n");
                        break;
                      case GSOverride:
                        DPRINTF(Predecoder, "Found gs segment override.\n");
                        break;
                      case SSOverride:
                        DPRINTF(Predecoder, "Found ss segment override.\n");
                        break;
                      case Lock:
                        DPRINTF(Predecoder, "Found lock prefix.\n");
                        break;
                      case Rep:
                        DPRINTF(Predecoder, "Found rep prefix.\n");
                        break;
                      case Repne:
                        DPRINTF(Predecoder, "Found repne prefix.\n");
                        break;
                      case Rex:
                        DPRINTF(Predecoder, "Found Rex prefix %#x.\n", nextByte);
                        emi.rexPrefix = nextByte;
                        break;
                      case 0:
                        emi.numOpcodes = 0;
                        state = Opcode;
                        break;
                      default:
                        panic("Unrecognized prefix %#x\n", nextByte);
                    }
                    break;
                  case Opcode:
                    emi.numOpcodes++;
                    assert(emi.numOpcodes < 2);
                    if(nextByte == 0xf0)
                    {
                        DPRINTF(Predecoder, "Found two byte opcode.\n");
                    }
                    else
                    {
                        immediateCollected = 0;
                        displacementCollected = 0;
                        emi.immediate = 0;
                        emi.displacement = 0;
                        int immType = ImmediateType[
                            emi.numOpcodes - 1][nextByte];
                        if(0) //16 bit mode
                            immediateSize = ImmediateTypeToSize[0][immType];
                        else if(!(emi.rexPrefix & 0x4)) //32 bit mode
                            immediateSize = ImmediateTypeToSize[1][immType];
                        else //64 bit mode
                            immediateSize = ImmediateTypeToSize[2][immType];
                        DPRINTF(Predecoder, "Found opcode %#x.\n", nextByte);
                        if (UsesModRM[emi.numOpcodes - 1][nextByte]) {
                            state = ModRM;
                        } else if(immediateSize) {
                            state = Immediate;
                        } else {
                            emiIsReady = true;
                            state = Prefix;
                        }
                    }
                    offset++;
                    break;
                  case ModRM:
                    DPRINTF(Predecoder, "Found modrm byte %#x.\n", nextByte);
                    if (0) {//in 16 bit mode
                        //figure out 16 bit displacement size
                        if(nextByte & 0xC7 == 0x06 ||
                                nextByte & 0xC0 == 0x80)
                            displacementSize = 2;
                        else if(nextByte & 0xC0 == 0x40)
                            displacementSize = 1;
                        else
                            displacementSize = 0;
                    } else {
                        //figure out 32/64 bit displacement size
                        if(nextByte & 0xC7 == 0x05 ||
                                nextByte & 0xC0 == 0x80)
                            displacementSize = 4;
                        else if(nextByte & 0xC0 == 0x40)
                            displacementSize = 2;
                        else
                            displacementSize = 0;
                    }
                    //If there's an SIB, get that next.
                    //There is no SIB in 16 bit mode.
                    if(nextByte & 0x7 == 4 &&
                            nextByte & 0xC0 != 0xC0) {
                            // && in 32/64 bit mode)
                        state = SIB;
                    } else if(displacementSize) {
                        state = Displacement;
                    } else if(immediateSize) {
                        state = Immediate;
                    } else {
                        emiIsReady = true;
                        state = Prefix;
                    }
                    //The ModRM byte is consumed no matter what
                    offset++;
                    break;
                  case SIB:
                    DPRINTF(Predecoder, "Found SIB byte %#x.\n", nextByte);
                    offset++;
                    if(displacementSize) {
                        state = Displacement;
                    } else if(immediateSize) {
                        state = Immediate;
                    } else {
                        emiIsReady = true;
                        state = Prefix;
                    }
                    break;
                  case Displacement:
                    //Gather up the displacement, or at least as much of it
                    //as we can get.

                    //Figure out how many bytes we still need to get for the
                    //displacement.
                    toGet = displacementSize - displacementCollected;
                    //Figure out how many bytes are left in our "buffer"
                    remaining = sizeof(MachInst) - offset;
                    //Get as much as we need, up to the amount available.
                    toGet = toGet > remaining ? remaining : toGet;

                    //Shift the bytes we want to be all the way to the right
                    partialDisp = fetchChunk >> (offset * 8);
                    //Mask off what we don't want
                    partialDisp &= mask(toGet * 8);
                    //Shift it over to overlay with our displacement.
                    partialDisp <<= (displacementCollected * 8);
                    //Put it into our displacement
                    emi.displacement |= partialDisp;
                    //Update how many bytes we've collected.
                    displacementCollected += toGet;
                    offset += toGet;
                    DPRINTF(Predecoder, "Collecting %d byte displacement, got %d bytes.\n",
                            displacementSize, displacementCollected);

                    if(displacementSize == displacementCollected) {
                        //Sign extend the displacement
                        switch(displacementSize)
                        {
                          case 1:
                            emi.displacement = sext<8>(emi.displacement);
                            break;
                          case 2:
                            emi.displacement = sext<16>(emi.displacement);
                            break;
                          case 4:
                            emi.displacement = sext<32>(emi.displacement);
                            break;
                          default:
                            panic("Undefined displacement size!\n");
                        }
                        DPRINTF(Predecoder, "Collected displacement %#x.\n",
                                emi.displacement);
                        if(immediateSize) {
                            state = Immediate;
                        } else {
                            emiIsReady = true;
                            state = Prefix;
                        }
                    }
                    break;
                  case Immediate:
                    //Gather up the displacement, or at least as much of it
                    //as we can get

                    //Figure out how many bytes we still need to get for the
                    //immediate.
                    toGet = immediateSize - immediateCollected;
                    //Figure out how many bytes are left in our "buffer"
                    remaining = sizeof(MachInst) - offset;
                    //Get as much as we need, up to the amount available.
                    toGet = toGet > remaining ? remaining : toGet;

                    //Shift the bytes we want to be all the way to the right
                    partialDisp = fetchChunk >> (offset * 8);
                    //Mask off what we don't want
                    partialDisp &= mask(toGet * 8);
                    //Shift it over to overlay with our immediate.
                    partialDisp <<= (immediateCollected * 8);
                    //Put it into our immediate
                    emi.immediate |= partialDisp;
                    //Update how many bytes we've collected.
                    immediateCollected += toGet;
                    offset += toGet;
                    DPRINTF(Predecoder, "Collecting %d byte immediate, got %d bytes.\n",
                            immediateSize, immediateCollected);

                    if(immediateSize == immediateCollected)
                    {
                        DPRINTF(Predecoder, "Collected immediate %#x.\n",
                                emi.immediate);
                        emiIsReady = true;
                        state = Prefix;
                    }
                    break;
                  default:
                    panic("Unrecognized state! %d\n", state);
                }
                if(offset == sizeof(MachInst))
                    outOfBytes = true;
            }
        }

        //Use this to give data to the predecoder. This should be used
        //when there is control flow.
        void moreBytes(Addr currPC, Addr off, MachInst data)
        {
            basePC = currPC;
            offset = off;
            fetchChunk = data;
            assert(off < sizeof(MachInst));
            outOfBytes = false;
            process();
        }

        //Use this to give data to the predecoder. This should be used
        //when instructions are executed in order.
        void moreBytes(MachInst machInst)
        {
            moreBytes(basePC + sizeof(machInst), 0, machInst);
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
    };
};

#endif // __ARCH_X86_PREDECODER_HH__
