
#ifndef __MEM_RUBY_SYSTEM_BANKEDARRAY_HH__
#define __MEM_RUBY_SYSTEM_BANKEDARRAY_HH__

#include <vector>

#include "mem/ruby/common/TypeDefines.hh"
#include "sim/eventq.hh"



class BankedArray : public EventManager
{
private:
    unsigned int banks;
    unsigned int accessLatency;
    unsigned int bankBits;
    unsigned int startIndexBit;

    //std::vector<bool> busyBanks;

    class TickEvent : public Event
    {
    public:
        TickEvent() : Event() {}
        void process() {}
        Index idx;
        Tick startAccess;
    };
    friend class TickEvent;

    // If the tick event is scheduled then the bank is busy
    // otherwise, schedule the event and wait for it to complete
    std::vector<TickEvent> busyBanks;

    unsigned int mapIndexToBank(Index idx);

public:
    BankedArray(unsigned int banks, unsigned int accessLatency, unsigned int startIndexBit);

    // Note: We try the access based on the cache index, not the address
    // This is so we don't get aliasing on blocks being replaced
    bool tryAccess(Index idx);

};

#endif
