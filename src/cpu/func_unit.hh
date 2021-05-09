/*
 * Copyright (c) 2002-2006 The Regents of The University of Michigan
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

#ifndef __CPU_FUNC_UNIT_HH__
#define __CPU_FUNC_UNIT_HH__

#include <array>
#include <bitset>
#include <string>
#include <vector>

#include "cpu/op_class.hh"
#include "params/FUDesc.hh"
#include "params/OpDesc.hh"
#include "sim/sim_object.hh"

namespace gem5
{

////////////////////////////////////////////////////////////////////////////
//
//  The SimObjects we use to get the FU information into the simulator
//
////////////////////////////////////////////////////////////////////////////

//
//  We use 2 objects to specify this data in the INI file:
//    (1) OpDesc - Describes the operation class & latencies
//                   (multiple OpDesc objects can refer to the same
//                   operation classes)
//    (2) FUDesc - Describes the operations available in the unit &
//                   the number of these units
//
//

////////////////////////////////////////////////////////////////////////////
//
//  Structures used ONLY during the initialization phase...
//
//
//

class OpDesc : public SimObject
{
  public:
    OpClass opClass;
    Cycles opLat;
    bool pipelined;

    OpDesc(const OpDescParams &p)
        : SimObject(p), opClass(p.opClass), opLat(p.opLat),
          pipelined(p.pipelined) {};
};

class FUDesc : public SimObject
{
  public:
    std::vector<OpDesc *> opDescList;
    unsigned         number;

    FUDesc(const FUDescParams &p)
        : SimObject(p), opDescList(p.opList), number(p.count) {};
};

typedef std::vector<OpDesc *>::const_iterator OPDDiterator;
typedef std::vector<FUDesc *>::const_iterator FUDDiterator;




////////////////////////////////////////////////////////////////////////////
//
//  The actual FU object
//
//
//
class FuncUnit
{
  private:
    std::array<unsigned, Num_OpClasses> opLatencies;
    std::array<bool, Num_OpClasses> pipelined;
    std::bitset<Num_OpClasses> capabilityList;

  public:
    FuncUnit();
    FuncUnit(const FuncUnit &fu);

    std::string name;

    void addCapability(OpClass cap, unsigned oplat, bool pipelined);

    bool provides(OpClass capability);
    std::bitset<Num_OpClasses> capabilities();

    unsigned &opLatency(OpClass capability);
    bool isPipelined(OpClass capability);
};

} // namespace gem5

#endif // __FU_POOL_HH__
