#include "base/misc.hh"
#include "cpu/beta_cpu/sat_counter.hh"

SatCounter::SatCounter()
    : maxVal(0), counter(0)
{
}

SatCounter::SatCounter(unsigned bits)
    : maxVal((1 << bits) - 1), counter(0)
{
}

SatCounter::SatCounter(unsigned bits, unsigned initial_val)
    : maxVal((1 << bits) - 1), counter(initial_val)
{
    // Check to make sure initial value doesn't exceed the max counter value.
    if (initial_val > maxVal) {
        panic("BP: Initial counter value exceeds max size.");
    }
}

void
SatCounter::setBits(unsigned bits)
{
    maxVal = (1 << bits) - 1;
}

void
SatCounter::increment()
{
    if(counter < maxVal) {
        ++counter;
    }
}

void
SatCounter::decrement()
{
    if(counter > 0) {
        --counter;
    }
}
