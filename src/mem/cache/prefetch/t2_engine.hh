// Deterministic T2 rules; independent of simulator packets and timing events.
#ifndef __MEM_CACHE_PREFETCH_T2_ENGINE_HH__
#define __MEM_CACHE_PREFETCH_T2_ENGINE_HH__
#include <algorithm>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>
namespace gem5::prefetch {
class T2Mean {
    unsigned limit;
    std::deque<uint64_t> samples;
    uint64_t sum = 0;
  public:
    explicit T2Mean(unsigned n) : limit(n) {}
    void clear() { samples.clear(); sum = 0; }
    bool add(uint64_t value) {
        if (!value || !limit || value > std::numeric_limits<uint64_t>::max()/limit)
            return false;
        if (samples.size() == limit) { sum -= samples.front(); samples.pop_front(); }
        samples.push_back(value); sum += value; return true;
    }
    uint64_t value() const { return samples.empty() ? 0 : sum/samples.size(); }
};
inline bool t2Delta(uint64_t a, uint64_t b, int64_t &out) {
    if (a >= b) {
        if (a-b > uint64_t(INT64_MAX)) return false;
        out = int64_t(a-b);
    } else {
        if (b-a > uint64_t(INT64_MAX)) return false;
        out = -int64_t(b-a);
    }
    return true;
}
inline bool t2Target(uint64_t base, int64_t stride, uint64_t n, uint64_t &out) {
    uint64_t magnitude = stride < 0 ? uint64_t(-(stride+1))+1 : uint64_t(stride);
    if (magnitude && n > UINT64_MAX/magnitude) return false;
    uint64_t offset = magnitude*n;
    if (stride < 0) { if (offset > base) return false; out = base-offset; }
    else { if (offset > UINT64_MAX-base) return false; out = base+offset; }
    return true;
}
inline unsigned t2Distance(uint64_t latency, uint64_t period, uint64_t margin,
                           unsigned maximum) {
    if (!period) return 1;
    uint64_t numerator = latency > UINT64_MAX-margin ? UINT64_MAX : latency+margin;
    uint64_t d = numerator/period + (numerator%period != 0);
    return unsigned(std::max<uint64_t>(1, std::min<uint64_t>(maximum, d)));
}
struct T2DeltaRun {
    enum State { Unknown, Observation, Strided, NonStrided };
    State state = Observation;
    uint64_t last = 0;
    int64_t delta = 0;
    bool deltaValid = false;
    unsigned equal = 0, changing = 0;
    bool sample(uint64_t address, unsigned confirm, unsigned reject) {
        int64_t next;
        if (!t2Delta(address, last, next)) return false;
        last = address;
        if (state != Observation) return true;
        if (!deltaValid) { delta = next; deltaValid = true; equal = 1; }
        else if (next == delta) { equal = std::min(confirm, equal+1); changing = 0; }
        else { delta = next; equal = 1; changing = std::min(reject, changing+1); }
        if (equal >= confirm) state = Strided;
        else if (changing >= reject) state = NonStrided;
        return true;
    }
};
class T2Loop {
    const unsigned capacity, minSegment;
    std::vector<uint64_t> nonLoops;
    T2Mean average;
  public:
    bool valid = false, seen = false;
    uint64_t pc = 0, target = 0, count = 0, generation = 0, prevIterCycle = 0;
    explicit T2Loop(unsigned n, unsigned segment, unsigned window)
        : capacity(n), minSegment(segment), average(window) {}
    uint64_t loopPeriod() const { return average.value(); }
    bool recognized() const { return valid && seen; }
    bool branch(uint64_t incoming, uint64_t destination, bool taken, uint64_t tick) {
        if (!taken || destination >= incoming ||
            std::find(nonLoops.begin(), nonLoops.end(), incoming) != nonLoops.end()) return false;
        if (valid && incoming == pc) {
            if (count != UINT64_MAX) ++count;
            seen = true;
            if (prevIterCycle && tick > prevIterCycle) average.add(tick-prevIterCycle);
            prevIterCycle = tick;
            return false;
        }
        bool outsidePC = !(incoming > target && incoming < pc);
        bool outsideTarget = !(destination > target && destination < pc);
        if (!valid || !seen || (outsidePC && outsideTarget)) {
            // Save outgoing values before the Figure 3 switch resets CNT.
            if (valid && count > minSegment && capacity) {
                if (nonLoops.size() == capacity) nonLoops.erase(nonLoops.begin());
                nonLoops.push_back(pc);
            }
            valid = true; seen = false; pc = incoming; target = destination;
            count = 0; ++generation; prevIterCycle = tick; average.clear();
            return true;
        }
        return false;
    }
};
}
#endif
