/*
 * Copyright (c) 2012-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Reinhardt
 */

#include "arch/hsail/insts/mem.hh"

#include "arch/hsail/Brig.h"
#include "enums/OpType.hh"

using namespace Brig;

namespace HsailISA
{
    const char* atomicOpToString(BrigAtomicOperation brigOp);

    Enums::MemOpType
    brigAtomicToMemOpType(BrigOpcode brigOpCode, BrigAtomicOperation brigOp)
    {
        if (brigOpCode == Brig::BRIG_OPCODE_ATOMIC) {
            switch (brigOp) {
              case BRIG_ATOMIC_AND:
                return Enums::MO_AAND;
              case BRIG_ATOMIC_OR:
                return Enums::MO_AOR;
              case BRIG_ATOMIC_XOR:
                return Enums::MO_AXOR;
              case BRIG_ATOMIC_CAS:
                return Enums::MO_ACAS;
              case BRIG_ATOMIC_EXCH:
                return Enums::MO_AEXCH;
              case BRIG_ATOMIC_ADD:
                return Enums::MO_AADD;
              case BRIG_ATOMIC_WRAPINC:
                return Enums::MO_AINC;
              case BRIG_ATOMIC_WRAPDEC:
                return Enums::MO_ADEC;
              case BRIG_ATOMIC_MIN:
                return Enums::MO_AMIN;
              case BRIG_ATOMIC_MAX:
                return Enums::MO_AMAX;
              case BRIG_ATOMIC_SUB:
                return Enums::MO_ASUB;
              default:
                fatal("Bad BrigAtomicOperation code %d\n", brigOp);
            }
        } else if (brigOpCode == Brig::BRIG_OPCODE_ATOMICNORET) {
            switch (brigOp) {
              case BRIG_ATOMIC_AND:
                  return Enums::MO_ANRAND;
              case BRIG_ATOMIC_OR:
                  return Enums::MO_ANROR;
              case BRIG_ATOMIC_XOR:
                  return Enums::MO_ANRXOR;
              case BRIG_ATOMIC_CAS:
                  return Enums::MO_ANRCAS;
              case BRIG_ATOMIC_EXCH:
                  return Enums::MO_ANREXCH;
              case BRIG_ATOMIC_ADD:
                  return Enums::MO_ANRADD;
              case BRIG_ATOMIC_WRAPINC:
                  return Enums::MO_ANRINC;
              case BRIG_ATOMIC_WRAPDEC:
                  return Enums::MO_ANRDEC;
              case BRIG_ATOMIC_MIN:
                  return Enums::MO_ANRMIN;
              case BRIG_ATOMIC_MAX:
                  return Enums::MO_ANRMAX;
              case BRIG_ATOMIC_SUB:
                  return Enums::MO_ANRSUB;
              default:
                fatal("Bad BrigAtomicOperation code %d\n", brigOp);
            }
        } else {
            fatal("Bad BrigAtomicOpcode %d\n", brigOpCode);
        }
    }

    const char*
    atomicOpToString(BrigAtomicOperation brigOp)
    {
        switch (brigOp) {
          case BRIG_ATOMIC_AND:
            return "and";
          case BRIG_ATOMIC_OR:
            return "or";
          case BRIG_ATOMIC_XOR:
            return "xor";
          case BRIG_ATOMIC_CAS:
            return "cas";
          case BRIG_ATOMIC_EXCH:
            return "exch";
          case BRIG_ATOMIC_ADD:
            return "add";
          case BRIG_ATOMIC_WRAPINC:
            return "inc";
          case BRIG_ATOMIC_WRAPDEC:
            return "dec";
          case BRIG_ATOMIC_MIN:
            return "min";
          case BRIG_ATOMIC_MAX:
            return "max";
          case BRIG_ATOMIC_SUB:
            return "sub";
          default:
            return "unknown";
        }
    }
} // namespace HsailISA
