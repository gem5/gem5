

#include <vector>

#include "base/intmath.hh"
#include "mem/ruby/common/TypeDefines.hh"
#include "mem/ruby/system/BankedArray.hh"
#include "sim/eventq.hh"

BankedArray::BankedArray(unsigned int banks, unsigned int accessLatency, unsigned int startIndexBit) :
    EventManager(&mainEventQueue)
{
    this->banks = banks;
    this->accessLatency = accessLatency;
    this->startIndexBit = startIndexBit;

    if (banks != 0) {
        bankBits = floorLog2(banks);
    }

    busyBanks.resize(banks);
}

bool
BankedArray::tryAccess(Index idx)
{
    if (accessLatency == 0)
        return true;

    unsigned int bank = mapIndexToBank(idx);
    assert(bank < banks);

    if (busyBanks[bank].scheduled()) {
        if (!(busyBanks[bank].startAccess == curTick() && busyBanks[bank].idx == idx)) {
            return false;
        } else {
            return true;  // We tried to allocate resources twice in the same cycle for the same addr
        }
    }

    busyBanks[bank].idx = idx;
    busyBanks[bank].startAccess = curTick();

    // substract 1 so that next cycle the resource available
    schedule(busyBanks[bank], curTick()+accessLatency-1);

    return true;
}

unsigned int
BankedArray::mapIndexToBank(Index idx)
{
    if (banks == 1) {
        return 0;
    }
    return idx % banks;
}
