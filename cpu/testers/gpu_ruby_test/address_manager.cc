/*
 * Copyright (c) 2017-2020 Advanced Micro Devices, Inc.
 * All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
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
 */

#include "cpu/testers/gpu_ruby_test/address_manager.hh"

#include <algorithm>
#include <climits>
#include <random>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"

namespace gem5
{

const int AddressManager::INVALID_VALUE = -1;
const int AddressManager::INVALID_LOCATION = -1;

AddressManager::AddressManager(int n_atomic_locs, int n_normal_locs_per_atomic)
      : numAtomicLocs(n_atomic_locs),
        numLocsPerAtomic(n_normal_locs_per_atomic)
{
    assert(numAtomicLocs > 0 && numLocsPerAtomic > 0);
    numNormalLocs = numAtomicLocs * numLocsPerAtomic;

    // generate random address map
    randAddressMap.resize(numAtomicLocs + numNormalLocs);
    for (Location i = 0; i < numAtomicLocs + numNormalLocs; ++i) {
        // all addresses are sizeof(Value) (i.e., 4-byte) aligned
        randAddressMap[i] = (Addr)((i + 128) << floorLog2(sizeof(Value)));
    }

    // randomly shuffle randAddressMap. The seed is determined by the random_mt
    // gem5 rng. This allows for deterministic randomization.
    std::shuffle(
        randAddressMap.begin(),
        randAddressMap.end(),
        std::default_random_engine(random_mt.random<unsigned>(0,UINT_MAX))
    );

    // initialize atomic locations
    // first and last normal location per atomic location
    Location first, last;
    for (Location atomic_loc = 0; atomic_loc < numAtomicLocs; ++atomic_loc) {
        first = numAtomicLocs + numLocsPerAtomic * atomic_loc;
        last = first + numLocsPerAtomic - 1;
        atomicStructs.push_back(new AtomicStruct(atomic_loc, first, last));
    }

    // initialize log table
    for (Location loc = 0; loc < numAtomicLocs + numNormalLocs; ++loc) {
        logTable.push_back(new LastWriter());
    }
}

AddressManager::~AddressManager()
{
    for (AtomicStruct* atomic_struct : atomicStructs)
        delete atomic_struct;
    for (LastWriter* lw : logTable)
        delete lw;
}

Addr
AddressManager::getAddress(Location loc)
{
    assert(loc < numAtomicLocs + numNormalLocs && loc >= 0);
    return randAddressMap[loc];
}

AddressManager::Location
AddressManager::getAtomicLoc()
{
    Location ret_atomic_loc = random() % numAtomicLocs;
    atomicStructs[ret_atomic_loc]->startLocSelection();
    return ret_atomic_loc;
}

AddressManager::Location
AddressManager::getLoadLoc(Location atomic_loc)
{
    assert(atomic_loc >= 0 && atomic_loc < numAtomicLocs);
    return atomicStructs[atomic_loc]->getLoadLoc();
}

AddressManager::Location
AddressManager::getStoreLoc(Location atomic_loc)
{
    assert(atomic_loc >= 0 && atomic_loc < numAtomicLocs);
    return atomicStructs[atomic_loc]->getStoreLoc();
}

void
AddressManager::finishLocSelection(Location atomic_loc)
{
    assert(atomic_loc >= 0 && atomic_loc < numAtomicLocs);
    atomicStructs[atomic_loc]->endLocSelection();
}

void
AddressManager::releaseLocation(Location atomic_loc, Location loc)
{
    assert(atomic_loc >= 0 && atomic_loc < numAtomicLocs);
    atomicStructs[atomic_loc]->releaseLoc(loc);
}

std::string
AddressManager::printLastWriter(Location loc) const
{
    return logTable[loc]->print();
}

// ------------------- AtomicStruct --------------------------
AddressManager::AtomicStruct::AtomicStruct(Location atomic_loc,
                                           Location loc_begin,
                                           Location loc_end)
{
    // the location range must have at least 1 location
    assert(loc_begin <= loc_end);

    atomicLoc = atomic_loc;
    arraySize = loc_end - loc_begin + 1;
    locationBase = loc_begin;

    // allocate an array of arrray_size
    locArray = new Location[arraySize];

    // initialize locArray & locProps
    Location loc;
    for (int offset = 0; offset < arraySize; ++offset) {
        loc = locationBase + offset;
        locArray[offset] = loc;
        locProps.push_back(LocProperty(offset, 0));
    }

    // region (1) and (3) are initially empty
    firstMark = 0;
    secondMark = arraySize;
    // no request made at this location so far
    requestCount = 0;
}

AddressManager::AtomicStruct::~AtomicStruct()
{
    delete[] locArray;
}

void
AddressManager::AtomicStruct::startLocSelection()
{
    assert(firstMark >= 0);
    assert(firstMark <= secondMark);
    assert(secondMark <= arraySize);
    // make sure loadStoreMap has been cleared
    assert(loadStoreMap.empty());

    // this atomic location is picked for Atomic_ACQ
    // and Atomic_REL in an episode
    requestCount += 2;
    // add two expected values in expectedValues set
    expectedValues.insert(requestCount - 1);
    expectedValues.insert(requestCount - 2);
}

AddressManager::Location
AddressManager::AtomicStruct::getLoadLoc()
{
    assert(firstMark >= 0);
    assert(firstMark <= secondMark);
    assert(secondMark <= arraySize);

    if (firstMark == arraySize) {
        // no location can be picked for a LD now, so return an empty location
        return INVALID_LOCATION;
    } else {
        // we can pick any location btw
        // locArray [firstMark : arraySize-1]
        int range_size = arraySize - firstMark;
        Location ret_loc = locArray[firstMark + random() % range_size];

        // update loadStoreMap
        LdStMap::iterator it = loadStoreMap.find(ret_loc);

        if (it == loadStoreMap.end()) {
            // insert a new entry to the map b/c the entry is not there yet
            // to mark this location has been picked for a LD
            loadStoreMap.insert(std::pair<Location, LdStBits>
                                            (ret_loc, LdStBits(true,false)));
        } else {
            // otherwise, just update the LD bit
            (it->second).first = true;
        }

        return ret_loc;
    }
}

AddressManager::Location
AddressManager::AtomicStruct::getStoreLoc()
{
    assert(firstMark >= 0);
    assert(firstMark <= secondMark);
    assert(secondMark <= arraySize);

    if (firstMark == secondMark) {
        // no location can be picked for a ST now, return an invalid location
        return INVALID_LOCATION;
    } else {
        // we can pick any location btw [firstMark : secondMark-1]
        int range_size = secondMark - firstMark;
        Location ret_loc = locArray[firstMark + random() % range_size];

        // update loadStoreMap
        LdStMap::iterator it = loadStoreMap.find(ret_loc);

        if (it == loadStoreMap.end()) {
            // insert a new entry to the map b/c the entry is not there yet
            // to mark this location has been picked for a ST
            loadStoreMap.insert(std::pair<Location, LdStBits>
                                            (ret_loc, LdStBits(false,true)));
        } else {
            // otherwise, just update the ST bit
            (it->second).second = true;
        }

        return ret_loc;
    }
}

// for each entry in loadStoreMap,
//  if <LD_bit, ST_bit> == <1,0>
//    - if the location is in (2), then move it to (3)
//    - if the location is in (3), no move
//    - otherwise, throw an error
//  if <LD_bit, ST_bit> == <0,1> or <1,1>
//    - move it from (2) to (1)
void
AddressManager::AtomicStruct::endLocSelection()
{
    assert(firstMark >= 0);
    assert(firstMark <= secondMark);
    assert(secondMark <= arraySize);

    for (auto& it : loadStoreMap) {
        Location loc = it.first;
        LdStBits p = it.second;

        assert(loc >= locationBase && loc < locationBase + arraySize);
        LocProperty& loc_prop = locProps[loc - locationBase];

        if (p.first && !p.second) {
            // this location has been picked for LD(s) but not ST
            // it must be in either region (2) or (3)
            assert(inSecondRegion(loc_prop.first) ||
                   inThirdRegion(loc_prop.first));

            if (inSecondRegion(loc_prop.first)) {
                // there is no owner of this location yet
                assert(loc_prop.second == 0);

                // pick the last location in (2) to swap
                Location swapped_loc = locArray[secondMark - 1];
                LocProperty& swapped_loc_prop =
                                         locProps[swapped_loc - locationBase];

                // swap loc and swapped_loc
                swap(loc_prop, swapped_loc_prop);

                // then, expand (3)
                secondMark--;
            }

            // increment the location's number of owners
            loc_prop.second++;
        } else if (p.second) {
            // this location has been picked for ST(s) and/or LD(s)
            // it must be in region (2)
            assert(inSecondRegion(loc_prop.first) && loc_prop.second == 0);

            // pick the first location in (2) to swap
            Location swapped_loc = locArray[firstMark];
            LocProperty& swapped_loc_prop =
                                        locProps[swapped_loc - locationBase];

            // swap loc and swapped_loc
            swap(loc_prop, swapped_loc_prop);

            // then, expand (1)
            firstMark++;

            // increment the location's number of owners
            loc_prop.second++;
        } else {
            panic("Location in loadStoreMap but wasn't picked in any"
                            " action\n");
        }
    }

    // clear the ld_st_map
    loadStoreMap.clear();
}

void
AddressManager::AtomicStruct::releaseLoc(Location loc)
{
    assert(loc >= locationBase && loc < locationBase + arraySize);

    LocProperty& loc_prop = locProps[loc - locationBase];

    if (inFirstRegion(loc_prop.first)) {
        // this location must have exactly 1 owner
        assert(loc_prop.second == 1);

        // pick the last location in region 1 to swap
        Location swapped_loc = locArray[firstMark - 1];
        LocProperty& swapped_loc_prop = locProps[swapped_loc - locationBase];

        // swap loc and swapped_loc
        swap(loc_prop, swapped_loc_prop);

        // then shrink (1)
        firstMark--;

        // reset the location's number of owners
        loc_prop.second = 0;
    } else if (inThirdRegion(loc_prop.first)) {
        // this location must have at least 1 owner
        assert(loc_prop.second >= 1);

        if (loc_prop.second == 1) {
            // pick the first location in region 3 to swap
            Location swapped_loc = locArray[secondMark];
            LocProperty& swapped_loc_prop =
                                        locProps[swapped_loc - locationBase];

            // swap loc and swapped_loc
            swap(loc_prop, swapped_loc_prop);

            // then shrink (3)
            secondMark++;
        }
        // decrement the loc's number of owners
        loc_prop.second--;
    } else {
        // some one else must already reset this counter
        assert(inSecondRegion(loc_prop.first) && loc_prop.second == 0);
    }
}

bool
AddressManager::AtomicStruct::isExpectedValue(Value val)
{
    ExpectedValueSet::iterator it = expectedValues.find(val);

    if (it == expectedValues.end()) {
        std::stringstream exp_val_ss;
        for (auto& val : expectedValues) {
            exp_val_ss << " " << val;
        }

        warn("Expected return values are:\n\t%s\n", exp_val_ss.str());

        return false;
    }

    // erase this value b/c it's done
    expectedValues.erase(it);

    return true;
}

void
AddressManager::AtomicStruct::swap(LocProperty& prop_1, LocProperty& prop_2)
{
    int new_idx_1 = prop_2.first;
    int new_idx_2 = prop_1.first;

    // swap the two locations in locArray
    Location tmp = locArray[prop_1.first];
    locArray[prop_1.first] = locArray[prop_2.first];
    locArray[prop_2.first] = tmp;

    // update their new indices
    prop_1.first = new_idx_1;
    prop_2.first = new_idx_2;
}

// ------------------ log table ---------------------
void
AddressManager::updateLogTable(Location loc, int thread_id, int episode_id,
                               Value new_value, Tick cur_tick, int cu_id)
{
    assert(loc >= 0 && loc < numAtomicLocs + numNormalLocs);
    logTable[loc]->update(thread_id, cu_id, episode_id, new_value, cur_tick);
}

AddressManager::Value
AddressManager::getLoggedValue(Location loc) const
{
    assert(loc >= 0 && loc < numAtomicLocs + numNormalLocs);
    return logTable[loc]->getLastStoredValue();
}

bool
AddressManager::validateAtomicResp(Location loc, Value ret_val)
{
    assert(loc >= 0 && loc < numAtomicLocs);
    return atomicStructs[loc]->isExpectedValue(ret_val);
}

} // namespace gem5
