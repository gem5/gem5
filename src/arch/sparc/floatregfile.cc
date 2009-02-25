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

#include "arch/sparc/floatregfile.hh"
#include "base/trace.hh"
#include "sim/byteswap.hh"
#include "sim/serialize.hh"

#include <string.h>

using namespace SparcISA;
using namespace std;

class Checkpoint;

void FloatRegFile::clear()
{
    memset(regSpace, 0, sizeof(regSpace));
}

FloatReg FloatRegFile::readReg(int floatReg, int width)
{
    //In each of these cases, we have to copy the value into a temporary
    //variable. This is because we may otherwise try to access an
    //unaligned portion of memory.
    FloatReg result;
    switch(width)
    {
      case SingleWidth:
        uint32_t result32;
        float32_t fresult32;
        memcpy(&result32, regSpace + 4 * floatReg, sizeof(result32));
        result32 = htog(result32);
        memcpy(&fresult32, &result32, sizeof(result32));
        result = fresult32;
        DPRINTF(FloatRegs, "Read FP32 register %d = [%f]0x%x\n",
                floatReg, result, result32);
        break;
      case DoubleWidth:
        uint64_t result64;
        float64_t fresult64;
        memcpy(&result64, regSpace + 4 * floatReg, sizeof(result64));
        result64 = htog(result64);
        memcpy(&fresult64, &result64, sizeof(result64));
        result = fresult64;
        DPRINTF(FloatRegs, "Read FP64 register %d = [%f]0x%x\n",
                floatReg, result, result64);
        break;
      case QuadWidth:
        panic("Quad width FP not implemented.");
        break;
      default:
        panic("Attempted to read a %d bit floating point register!", width);
    }
    return result;
}

FloatRegBits FloatRegFile::readRegBits(int floatReg, int width)
{
    //In each of these cases, we have to copy the value into a temporary
    //variable. This is because we may otherwise try to access an
    //unaligned portion of memory.
    FloatRegBits result;
    switch(width)
    {
      case SingleWidth:
        uint32_t result32;
        memcpy(&result32, regSpace + 4 * floatReg, sizeof(result32));
        result = htog(result32);
        DPRINTF(FloatRegs, "Read FP32 bits register %d = 0x%x\n",
                floatReg, result);
        break;
      case DoubleWidth:
        uint64_t result64;
        memcpy(&result64, regSpace + 4 * floatReg, sizeof(result64));
        result = htog(result64);
        DPRINTF(FloatRegs, "Read FP64 bits register %d = 0x%x\n",
                floatReg, result);
        break;
      case QuadWidth:
        panic("Quad width FP not implemented.");
        break;
      default:
        panic("Attempted to read a %d bit floating point register!", width);
    }
    return result;
}

Fault FloatRegFile::setReg(int floatReg, const FloatReg &val, int width)
{
    //In each of these cases, we have to copy the value into a temporary
    //variable. This is because we may otherwise try to access an
    //unaligned portion of memory.

    uint32_t result32;
    uint64_t result64;
    float32_t fresult32;
    float64_t fresult64;
    switch(width)
    {
      case SingleWidth:
        fresult32 = val;
        memcpy(&result32, &fresult32, sizeof(result32));
        result32 = gtoh(result32);
        memcpy(regSpace + 4 * floatReg, &result32, sizeof(result32));
        DPRINTF(FloatRegs, "Write FP64 register %d = 0x%x\n",
                floatReg, result32);
        break;
      case DoubleWidth:
        fresult64 = val;
        memcpy(&result64, &fresult64, sizeof(result64));
        result64 = gtoh(result64);
        memcpy(regSpace + 4 * floatReg, &result64, sizeof(result64));
        DPRINTF(FloatRegs, "Write FP64 register %d = 0x%x\n",
                floatReg, result64);
        break;
      case QuadWidth:
        panic("Quad width FP not implemented.");
        break;
      default:
        panic("Attempted to read a %d bit floating point register!", width);
    }
    return NoFault;
}

Fault FloatRegFile::setRegBits(int floatReg, const FloatRegBits &val, int width)
{
    //In each of these cases, we have to copy the value into a temporary
    //variable. This is because we may otherwise try to access an
    //unaligned portion of memory.
    uint32_t result32;
    uint64_t result64;
    switch(width)
    {
      case SingleWidth:
        result32 = gtoh((uint32_t)val);
        memcpy(regSpace + 4 * floatReg, &result32, sizeof(result32));
        DPRINTF(FloatRegs, "Write FP64 bits register %d = 0x%x\n",
                floatReg, result32);
        break;
      case DoubleWidth:
        result64 = gtoh((uint64_t)val);
        memcpy(regSpace + 4 * floatReg, &result64, sizeof(result64));
        DPRINTF(FloatRegs, "Write FP64 bits register %d = 0x%x\n",
                floatReg, result64);
        break;
      case QuadWidth:
        panic("Quad width FP not implemented.");
        break;
      default:
        panic("Attempted to read a %d bit floating point register!", width);
    }
    return NoFault;
}

void FloatRegFile::serialize(std::ostream &os)
{
    uint8_t *float_reg = (uint8_t*)regSpace;
    SERIALIZE_ARRAY(float_reg,
            SingleWidth / 8 * NumFloatRegs);
}

void FloatRegFile::unserialize(Checkpoint *cp, const std::string &section)
{
    uint8_t *float_reg = (uint8_t*)regSpace;
    UNSERIALIZE_ARRAY(float_reg,
            SingleWidth / 8 * NumFloatRegs);
}

