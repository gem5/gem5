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

#include "arch/x86/predecoder.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "sim/host.hh"

namespace X86ISA
{
    void Predecoder::process()
    {
        //This function drives the predecoder state machine.

        //Some sanity checks. You shouldn't try to process more bytes if
        //there aren't any, and you shouldn't overwrite an already
        //predecoder ExtMachInst.
        assert(!outOfBytes);
        assert(!emiIsReady);

        //While there's still something to do...
        while(!emiIsReady && !outOfBytes)
        {
            uint8_t nextByte = getNextByte();
            switch(state)
            {
              case Prefix:
                state = doPrefixState(nextByte);
                break;
              case Opcode:
                state = doOpcodeState(nextByte);
                break;
              case ModRM:
                state = doModRMState(nextByte);
                break;
              case SIB:
                state = doSIBState(nextByte);
                break;
              case Displacement:
                state = doDisplacementState();
                break;
              case Immediate:
                state = doImmediateState();
                break;
              case ErrorState:
                panic("Went to the error state in the predecoder.\n");
              default:
                panic("Unrecognized state! %d\n", state);
            }
        }
    }

    //Either get a prefix and record it in the ExtMachInst, or send the
    //state machine on to get the opcode(s).
    Predecoder::State Predecoder::doPrefixState(uint8_t nextByte)
    {
        uint8_t prefix = Prefixes[nextByte];
        State nextState = Prefix;
        if(prefix)
            consumeByte();
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
            nextState = Opcode;
            break;
          default:
            panic("Unrecognized prefix %#x\n", nextByte);
        }
        return nextState;
    }

    //Load all the opcodes (currently up to 2) and then figure out
    //what immediate and/or ModRM is needed.
    Predecoder::State Predecoder::doOpcodeState(uint8_t nextByte)
    {
        State nextState = ErrorState;
        emi.numOpcodes++;
        //We can't handle 3+ byte opcodes right now
        assert(emi.numOpcodes < 2);
        consumeByte();
        if(nextByte == 0xf0)
        {
            nextState = Opcode;
            DPRINTF(Predecoder, "Found two byte opcode.\n");
        }
        else
        {
            DPRINTF(Predecoder, "Found opcode %#x.\n", nextByte);

            //Prepare for any immediate/displacement we might need
            immediateCollected = 0;
            emi.immediate = 0;
            displacementCollected = 0;
            emi.displacement = 0;

            //Figure out how big of an immediate we'll retreive based
            //on the opcode.
            int immType = ImmediateType[
                emi.numOpcodes - 1][nextByte];
            if(0) //16 bit mode
                immediateSize = ImmediateTypeToSize[0][immType];
            else if(!(emi.rexPrefix & 0x4)) //32 bit mode
                immediateSize = ImmediateTypeToSize[1][immType];
            else //64 bit mode
                immediateSize = ImmediateTypeToSize[2][immType];

            //Determine what to expect next
            if (UsesModRM[emi.numOpcodes - 1][nextByte]) {
                nextState = ModRM;
            } else if(immediateSize) {
                nextState = Immediate;
            } else {
                emiIsReady = true;
                nextState = Prefix;
            }
        }
        return nextState;
    }

    //Get the ModRM byte and determine what displacement, if any, there is.
    //Also determine whether or not to get the SIB byte, displacement, or
    //immediate next.
    Predecoder::State Predecoder::doModRMState(uint8_t nextByte)
    {
        State nextState = ErrorState;
        DPRINTF(Predecoder, "Found modrm byte %#x.\n", nextByte);
        if (0) {//FIXME in 16 bit mode
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
            nextState = SIB;
        } else if(displacementSize) {
            nextState = Displacement;
        } else if(immediateSize) {
            nextState = Immediate;
        } else {
            emiIsReady = true;
            nextState = Prefix;
        }
        //The ModRM byte is consumed no matter what
        consumeByte();
        return nextState;
    }

    //Get the SIB byte. We don't do anything with it at this point, other
    //than storing it in the ExtMachInst. Determine if we need to get a
    //displacement or immediate next.
    Predecoder::State Predecoder::doSIBState(uint8_t nextByte)
    {
        State nextState = ErrorState;
        DPRINTF(Predecoder, "Found SIB byte %#x.\n", nextByte);
        consumeByte();
        if(displacementSize) {
            nextState = Displacement;
        } else if(immediateSize) {
            nextState = Immediate;
        } else {
            emiIsReady = true;
            nextState = Prefix;
        }
        return nextState;
    }

    //Gather up the displacement, or at least as much of it
    //as we can get.
    Predecoder::State Predecoder::doDisplacementState()
    {
        State nextState = ErrorState;

        getImmediate(displacementCollected,
                emi.displacement,
                displacementSize);

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
                nextState = Immediate;
            } else {
                emiIsReady = true;
                nextState = Prefix;
            }
        }
        else
            nextState = Displacement;
        return nextState;
    }

    //Gather up the immediate, or at least as much of it
    //as we can get
    Predecoder::State Predecoder::doImmediateState()
    {
        State nextState = ErrorState;

        getImmediate(immediateCollected,
                emi.immediate,
                immediateSize);

        DPRINTF(Predecoder, "Collecting %d byte immediate, got %d bytes.\n",
                immediateSize, immediateCollected);

        if(immediateSize == immediateCollected)
        {
            DPRINTF(Predecoder, "Collected immediate %#x.\n",
                    emi.immediate);
            emiIsReady = true;
            nextState = Prefix;
        }
        else
            nextState = Immediate;
        return nextState;
    }
}
