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
 *
 * Authors: Steve Raasch
 */

#ifndef __CPU_FUNC_UNIT_HH__
#define __CPU_FUNC_UNIT_HH__

#include <bitset>
#include <string>
#include <vector>

#include "cpu/op_class.hh"
#include "sim/sim_object.hh"

////////////////////////////////////////////////////////////////////////////
//
//  Structures used ONLY during the initialization phase...
//
//
//

struct OpDesc : public SimObject
{
    OpClass opClass;
    unsigned    opLat;
    unsigned    issueLat;

    OpDesc(std::string name, OpClass c, unsigned o, unsigned i)
        : SimObject(name), opClass(c), opLat(o), issueLat(i) {};
};

struct FUDesc : public SimObject
{
    std::vector<OpDesc *> opDescList;
    unsigned         number;

    FUDesc(std::string name, std::vector<OpDesc *> l, unsigned n)
        : SimObject(name), opDescList(l), number(n) {};
};

typedef std::vector<OpDesc *>::iterator OPDDiterator;
typedef std::vector<FUDesc *>::iterator FUDDiterator;




////////////////////////////////////////////////////////////////////////////
//
//  The actual FU object
//
//
//
class FuncUnit
{
  private:
    unsigned opLatencies[Num_OpClasses];
    unsigned issueLatencies[Num_OpClasses];
    std::bitset<Num_OpClasses> capabilityList;

  public:
    FuncUnit();
    FuncUnit(const FuncUnit &fu);

    std::string name;

    void addCapability(OpClass cap, unsigned oplat, unsigned issuelat);

    bool provides(OpClass capability);
    std::bitset<Num_OpClasses> capabilities();

    unsigned &opLatency(OpClass capability);
    unsigned issueLatency(OpClass capability);
};

#endif // __FU_POOL_HH__
