/*
 * Copyright (c) 2012-2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include "cpu/o3/fu_pool.hh"

#include <sstream>

#include "cpu/func_unit.hh"

namespace gem5
{

namespace o3
{

////////////////////////////////////////////////////////////////////////////
//
//  A pool of function units
//

inline void
FUPool::FUIdxQueue::addFU(int fu_idx)
{
    funcUnitsIdx.push_back(fu_idx);
    ++size;
}

inline int
FUPool::FUIdxQueue::getFU()
{
    int retval = funcUnitsIdx[idx++];

    if (idx == size)
        idx = 0;

    return retval;
}

FUPool::~FUPool()
{
    for (FuncUnit* fu : funcUnits) {
        delete fu;
    }
}


// Constructor
FUPool::FUPool(const Params &p)
    : SimObject(p)
{
    numFU = 0;

    funcUnits.clear();

    maxOpLatencies.fill(Cycles(0));
    pipelined.fill(true);

    //
    //  Iterate through the list of FUDescData structures
    //
    for (FUDesc *i : p.FUList) {
        //
        //  Don't bother with this if we're not going to create any FU's
        //
        if (i->number) {
            //
            //  Create the FuncUnit object from this structure
            //   - add the capabilities listed in the FU's operation
            //     description
            //
            //  We create the first unit, then duplicate it as needed
            //
            FuncUnit *fu = new FuncUnit;

            for (OpDesc *j : i->opDescList) {
                // indicate that this pool has this capability
                capabilityList.set(j->opClass);

                // Add each of the FU's that will have this capability to the
                // appropriate queue.
                for (int k = 0; k < i->number; ++k)
                    fuPerCapList[j->opClass].addFU(numFU + k);

                // indicate that this FU has the capability
                fu->addCapability(j->opClass, j->opLat, j->pipelined);

                if (j->opLat > maxOpLatencies[j->opClass])
                    maxOpLatencies[j->opClass] = j->opLat;

                if (!j->pipelined)
                    pipelined[j->opClass] = false;
            }

            numFU++;

            //  Add the appropriate number of copies of this FU to the list
            fu->name = i->name() + "(0)";
            funcUnits.push_back(fu);

            for (int c = 1; c < i->number; ++c) {
                std::ostringstream s;
                numFU++;
                FuncUnit *fu2 = new FuncUnit(*fu);

                s << i->name() << "(" << c << ")";
                fu2->name = s.str();
                funcUnits.push_back(fu2);
            }
        }
    }

    unitBusy.resize(numFU);

    for (int i = 0; i < numFU; i++) {
        unitBusy[i] = false;
    }
}

int FUPool::findFreeUnit(OpClass capability)
{
    int fu_idx = fuPerCapList[capability].getFU();
    int start_idx = fu_idx;

    // Iterate through the circular queue if needed, stopping if we've reached
    // the first element again.
    while (unitBusy[fu_idx]) {
        fu_idx = fuPerCapList[capability].getFU();
        if (fu_idx == start_idx) {
            // No FU available
            return NoFreeFU;
        }
    }

    assert(fu_idx < numFU);

    return fu_idx;
}

int
FUPool::getUnit(OpClass capability, bool is_shadow, OpClass &approx_capability)
{
    //  If this pool doesn't have the specified capability,
    //  return this information to the caller
    if (!capabilityList[capability])
        return NoCapableFU;

    int fu_idx = NoFreeFU;
    int fu_idx_aux = NoFreeFU;
    int fu_idx_aux_2 = NoFreeFU;

    approx_capability = capability;

    if (is_shadow)
    {
        switch (capability)
        {
            case OpClass::IntAlu:
                fu_idx = findFreeUnit(capability);
                //fu_idx_aux = findFreeUnit(OpClass::FloatAdd);
                //fu_idx_aux_2 = findFreeUnit(OpClass::FloatCmp);

                //if (fu_idx == NoFreeFU)
                //{
                //    fu_idx = fu_idx_aux;
                //    approx_capability = OpClass::FloatAdd;
                //    if (fu_idx_aux == NoFreeFU)
                //    {
                //        approx_capability = OpClass::FloatCmp;
                //        fu_idx = fu_idx_aux_2;
                //    }
                //}
                    
                break;
            case OpClass::IntMult:
                fu_idx = findFreeUnit(capability);
                //fu_idx_aux = findFreeUnit(OpClass::FloatMult);

                //if (fu_idx == NoFreeFU)
               // {
                //    approx_capability = OpClass::FloatMult;
                //    fu_idx = fu_idx_aux;
               // }

                break;
            case OpClass::IntDiv:
                fu_idx = findFreeUnit(capability);
                //fu_idx_aux = findFreeUnit(OpClass::FloatDiv);
                
                //if (fu_idx == NoFreeFU)
                //{
                //    approx_capability = OpClass::FloatDiv;
                //    fu_idx = fu_idx_aux;
                //}

                break;
            case OpClass::FloatAdd:
                fu_idx = findFreeUnit(capability);
                ///fu_idx_aux = findFreeUnit(OpClass::IntAlu);
                
               // if (fu_idx == NoFreeFU)
               // {
                //    approx_capability = OpClass::IntAlu;
               // //    fu_idx = fu_idx_aux;
                //}

                break;
            case OpClass::FloatMult:
                fu_idx = findFreeUnit(capability);
                //fu_idx_aux = findFreeUnit(OpClass::IntAlu);
                //
                //if (fu_idx == NoFreeFU)
                //{
                //    approx_capability = OpClass::IntAlu;
                //    fu_idx = fu_idx_aux;
                //}

                break;
            case OpClass::FloatDiv:
                fu_idx = findFreeUnit(capability);
               // fu_idx_aux = findFreeUnit(OpClass::IntAlu);
             //   
               // if (fu_idx == NoFreeFU)
              //  {
               //     approx_capability = OpClass::IntAlu;
               //     fu_idx = fu_idx_aux;
              //  }

                break;
            case OpClass::FloatSqrt:
                fu_idx = findFreeUnit(capability);
               // fu_idx_aux = findFreeUnit(OpClass::IntAlu);
               // 
               // if (fu_idx == NoFreeFU)
               // {
               //     approx_capability = OpClass::IntAlu;
               //     fu_idx = fu_idx_aux;
               // }
                    
                break;
            case OpClass::FloatMultAcc:
            case OpClass::FloatCvt:
            case OpClass::FloatCmp:
            case OpClass::FloatMisc:
                fu_idx = findFreeUnit(capability);
                break;
            default:
                fu_idx = NoShadowFU;
                break;
        }
    }
    else
    {
        fu_idx = findFreeUnit(capability);
    }

    if (fu_idx == NoShadowFU)
        return NoShadowFU;
    else if (fu_idx == NoFreeFU)
        return NoFreeFU;
        
    unitBusy[fu_idx] = true;

    return fu_idx;
}

void
FUPool::freeUnitNextCycle(int fu_idx)
{
    assert(unitBusy[fu_idx]);
    unitsToBeFreed.push_back(fu_idx);
}

void
FUPool::freeUnitImmediately(int fu_idx)
{
    assert(fu_idx >= 0 && fu_idx < numFU);
    assert(unitBusy[fu_idx]);
    unitBusy[fu_idx] = false;
}

int
FUPool::countFreeUnits(OpClass capability)
{
    if (!capabilityList[capability])
        return 0;

    int freeCount = 0;
    for (int i = 0; i < numFU; i++) {
        if (!unitBusy[i] && funcUnits[i]->provides(capability)) {
            freeCount++;
        }
    }
    return freeCount;
}

void
FUPool::processFreeUnits()
{
    while (!unitsToBeFreed.empty()) {
        int fu_idx = unitsToBeFreed.back();
        unitsToBeFreed.pop_back();

        assert(unitBusy[fu_idx]);

        unitBusy[fu_idx] = false;
    }
}

void
FUPool::dump()
{
    std::cout << "Function Unit Pool (" << name() << ")\n";
    std::cout << "======================================\n";
    std::cout << "Free List:\n";

    for (int i = 0; i < numFU; ++i) {
        if (unitBusy[i]) {
            continue;
        }

        std::cout << "  [" << i << "] : ";

        std::cout << funcUnits[i]->name << " ";

        std::cout << "\n";
    }

    std::cout << "======================================\n";
    std::cout << "Busy List:\n";
    for (int i = 0; i < numFU; ++i) {
        if (!unitBusy[i]) {
            continue;
        }

        std::cout << "  [" << i << "] : ";

        std::cout << funcUnits[i]->name << " ";

        std::cout << "\n";
    }
}

bool
FUPool::isDrained() const
{
    bool is_drained = true;
    for (int i = 0; i < numFU; i++)
        is_drained = is_drained && !unitBusy[i];

    return is_drained;
}

} // namespace o3
} // namespace gem5
