/*
 * Copyright (c) 2017-2021 Advanced Micro Devices, Inc.
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

#ifndef CPU_TESTERS_PROTOCOL_TESTER_ADDRESS_MANAGER_HH_
#define CPU_TESTERS_PROTOCOL_TESTER_ADDRESS_MANAGER_HH_

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "base/random.hh"
#include "base/types.hh"
#include "sim/eventq.hh"

namespace gem5
{

/*
 * --- AddressManager has 3 main tasks ---
 *    (1) generate DRF request sequences
 *    (2) maintain internal log table
 *    (3) validate return values against ones in the log table
 *
 * A location is an abstract index of a unique real address.
 *    It's used internally within the tester only.
 *    randAddressMap has the mapping between a location and its real address.
 *
 * A value is an integer that a location in real memory can store.
 *    for now, we assume a value is 4-byte
 *
 * The location range (randAddressMap) has two distinct parts:
 *    Atomic locations: in the 1st part of randAddressMap &
 *    Non-atomic locations (or just locations): in the 2nd part
 */

/*
 * --- DRF request sequence generation ---
 *    Each lane of an episode starts selecting its location by calling:
 *      (1) getAtomicLoc
 *      (2) getLoadLoc/getStoreLoc
 *      (3) finishLocSelection
 *
 *    Each lane of an episode completes its executing by calling:
 *      releaseLocation for all locations it selected
 */

/*
 * --- Internal structures ---
 *  There are multiple atomic structures, each of which corresponds
 *    to an atomic location.
 *
 *  Each atomic structure manages a distinct range of locations in locArray
 *  This array is partitioned into 3 parts that are used to select locations
 *  for LDs and STs. Here is the location selecting rule:
 *                  |    (1)    |    (2)    |    (3)    |
 *    - all locations in (1) cannot be picked for any LD and ST action
 *    - all locations in (2) can be picked for either LD or ST action
 *    - all locations in (3) can be picked for LD action only
 *
 *  We maintain the 3 parts by 2 indices firstMark and secondMark.
 *  As locations are moved between partitions, both indices are updated
 *  accordingly.
 *    [0 .. firstMark-1]                  part (1)
 *    [firstMark .. secondMark-1]      part (2)
 *    [secondMark .. arraySize-1]        part (3)
 *
 *  Each location has its context/property. locProps maintains
 *  contexts/properties of all locations. Context/property includes
 *      - current index of a location in locArray
 *      - the number of owners who are currently using the location
 *
 *  To guarantee DRF constraints, the following conditions must hold
 *    - all locations in (1) have exactly 1 owner
 *    - all locations in (2) have exactly 0 owner
 *    - all locations in (3) have at least 1 owner
 *    - A LD request can randomly pick any location in (2) & (3)
 *    - A ST request can randomly pick any location in (2)
 *
 *  loadStoreMap maintains all locations already selected for LDs/STs so far
 *
 *  When endLocSelection is called (i.e., we've picked all locations for an
 *  episode), we need to move each selected location to its right partition.
 *    if LD_bit == 1 && ST_bit == 0 (i.e., picked for LDs), then move the
 *          location to (3) -> future LDs can pick it.
 *    if LD_bit == 0 && ST_bit == 1, then move the location to (1) -> NO future
 *          action can pick it until this episode is done.
 *    if LD_bit == 1 && ST_bit == 1, then move the location to (1) -> NO future
 *          action can pick it until this episode is done.
 *    clear the loadStoreMap
 */

class AddressManager
{
  public:
    AddressManager(int n_atomic_locs, int numNormalLocsPerAtomic);
    ~AddressManager();

    typedef int32_t Value;
    typedef int32_t Location;

    // return the unique address mapped to a location
    Addr getAddress(Location loc);
    // return a unique atomic location & start picking locations
    Location getAtomicLoc();
    // return a random location for LD
    Location getLoadLoc(Location atomic_loc);
    // return a random location for ST
    Location getStoreLoc(Location atomic_loc);
    // finish picking locations
    void finishLocSelection(Location atomic_loc);
    // an episode is done, release location I've picked
    void releaseLocation(Location atomic_loc, Location loc);
    // update a log table entry with a given set of values
    void updateLogTable(Location loc, int threadId, int episodeId,
                        Value new_value, Tick curTick, int cuId = -1);
    // return the current value in the log table
    Value getLoggedValue(Location loc) const;
    // validate atomic response
    bool validateAtomicResp(Location loc, Value ret_val);

    std::string printLastWriter(Location loc) const;

    static const int INVALID_VALUE;
    static const int INVALID_LOCATION;

  private:
    class LastWriter
    {
      public:
        LastWriter()
            : threadId(-1), cuId(-1), episodeId(-1), value(0),
              writeTick(0)
        { }

        const std::string print() const
        {
            return "(TesterThread ID " + std::to_string(threadId) +
                   ", CU ID " + std::to_string(cuId) +
                   ", Episode ID " + std::to_string(episodeId) +
                   ", Value " + std::to_string(value) +
                   ", Tick " + std::to_string(writeTick) +
                   ")";
        }

        void update(int _thread, int _cu, int _episode, Value _value,
                    Tick _tick)
        {
            threadId = _thread;
            cuId = _cu;
            episodeId = _episode;
            value = _value;
            writeTick = _tick;
        }

        Value getLastStoredValue() const { return value; }

      private:
        int threadId;
        int cuId;
        int episodeId;
        Value value;
        Tick writeTick;
    };

    class AtomicStruct
    {
      public:
        AtomicStruct(Location atom_loc, Location loc_begin, Location loc_end);
        ~AtomicStruct();

        // functions picking locations for LD/ST/ATOMIC ops
        void startLocSelection();
        Location getLoadLoc();
        Location getStoreLoc();
        void endLocSelection();

        // an episode completed its actions
        // return locations to their correct positions
        void releaseLoc(Location loc);
        // is the value what we expect?
        bool isExpectedValue(Value val);

      private:
        Location atomicLoc;
        Location locationBase;

        // array storing all locations this structure is managing
        Location* locArray;
        int firstMark, secondMark;
        int arraySize;

        // a vector of location's properties
        typedef std::pair<int, int> LocProperty;
        typedef std::vector<LocProperty> LocPropTable;
        LocPropTable locProps;

        // a temporary map of location and its LD/ST selection
        typedef std::pair<bool, bool> LdStBits;
        typedef std::unordered_map<Location, LdStBits> LdStMap;
        LdStMap loadStoreMap;

        // number of atomic requests at this location so far
        int requestCount;
        // a set of expected values
        // when we request the first n atomic ops, we expect to receive n
        // return values from [0 .. n-1]
        typedef std::unordered_set<Value> ExpectedValueSet;
        ExpectedValueSet expectedValues;

        Random::RandomPtr rng = Random::genRandom();

        // swap two locations in locArray
        void swap(LocProperty& prop_1, LocProperty& prop_2);

        bool inFirstRegion(int idx) const
        {
            return (idx >= 0 && idx < firstMark);
        }
        bool inSecondRegion(int idx) const
        {
            return (idx >= firstMark && idx < secondMark);
        }
        bool inThirdRegion(int idx) const
        {
            return (idx >= secondMark && idx < arraySize);
        }
    };

    // number of atomic locations
    int numAtomicLocs;
    // number of normal/non-atomic locations per atomic structure
    int numLocsPerAtomic;
    // total number of non-atomic locations
    int numNormalLocs;

    // location - address mapping
    typedef std::vector<Addr> AddressMap;
    AddressMap randAddressMap;

    // a list of atomic structures
    typedef std::vector<AtomicStruct*> AtomicStructTable;
    AtomicStructTable atomicStructs;

    // internal log table
    typedef std::vector<LastWriter*> LogTable;
    LogTable logTable;

    Random::RandomPtr rng = Random::genRandom();
};

} // namespace gem5

#endif /* CPU_TESTERS_PROTOCOL_TESTER_ADDRESS_MANAGER_HH_ */
