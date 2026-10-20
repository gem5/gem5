/**
 * Adaptive-window controller of the L2 window mode (window_mode): the
 * group scheduler and the N controllers (fixed, tcp, amc). Pure logic
 * with no gem5 dependency, so a plain unit test can drive it
 * (tests/retention/window_controller_test.cc).
 *
 * Frames and groups. A frame is one refresh-all sweep. The cache is split
 * into G groups (group = set mod G). Group g has a counter c_g, the
 * frames since the frame of its last decision, saturating at counterMax.
 * At the start of every frame a group is due if c_g >= N - 1 (the
 * deciding frame counts, so a group decides every N frames; N = 1 is
 * every frame), and at most K = max(1, G >> floor(log2 N)) due groups,
 * picked round-robin, get decide_g = 1 for the frame; the other due
 * groups wait. At the end of the frame c_g = 0 for the groups that
 * decided and c_g += 1 for the others. A line's sweep slot in a deciding
 * group keeps the line if it was used since its group's last decision
 * and drops it otherwise.
 *
 * tcp: N grows at a frame end when the retention misses R since the last
 * reset exceed A >> up_shift (A: demand accesses): doubling up to
 * ssthresh, then +1 up to N_max. The first frame with no growth after a
 * frame that grew sets ssthresh = N. A global check counter runs a shrink
 * check every N frames: if A >= min_A, a check with R < A >> down_shift
 * is calm; calm_len calm checks in a row decrement N (streak_len times),
 * then halve it (ssthresh = N first). R and A are reset at every N change
 * and at every judged check; with A < min_A they keep accumulating.
 *
 * amc: every interval, ideal = demand misses - R and target = frac *
 * ideal; R > hi * target asks for 2N, R < lo * target for N / 2 (capped
 * to [1, N_max]), otherwise hold. The latest decision replaces a pending
 * one; an increase applies at the next frame end, a decrease at the next
 * check (every N frames).
 *
 * Every change of N takes effect from the next frame's grant.
 */

#ifndef __MEM_CACHE_RETENTION_WINDOW_CONTROLLER_HH__
#define __MEM_CACHE_RETENTION_WINDOW_CONTROLLER_HH__

#include <algorithm>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace gem5
{

class WindowController
{
  public:
    enum class Mode { Off, Fixed, Tcp, Amc };

    /** What the listener is told about. */
    enum class Event
    {
        /** tcp growth that changed N (old, new N). */
        Grow,
        /** tcp growth test that fired at N_max (N, N). */
        GrowCapped,
        /** tcp calm decrement (old, new N; may be equal at N = 1). */
        Dec,
        /** tcp halving (old, new N; may be equal at N = 1). */
        Halve,
        /** ssthresh changed (old, new ssthresh). */
        Ssthresh,
        /** amc decision (old, new pending N). */
        AmcDecide,
        /** amc increase applied (old, new N). */
        AmcGrow,
        /** amc decrease applied (old, new N). */
        AmcShrink
    };

    struct Params
    {
        Mode mode = Mode::Off;
        unsigned nFixed = 8;
        unsigned tcpNInit = 1;
        unsigned amcNInit = 8;
        unsigned ssthreshInit = 8;
        unsigned nMax = 64;
        unsigned nMin = 1;
        unsigned upShift = 7;
        unsigned downShift = 9;
        uint64_t minAccesses = 512;
        unsigned streakLen = 3;
        unsigned calmLen = 2;
        double amcTargetFrac = 0.5;
        double amcHi = 1.5;
        double amcLo = 0.5;
        unsigned groups = 64;
        unsigned counterMax = 127;
    };

    using Listener = std::function<void(Event, unsigned, unsigned)>;

    /** An empty string if p is valid, else what is wrong. */
    static std::string
    check(const Params &p)
    {
        if (p.mode == Mode::Off)
            return "";
        if (p.groups == 0)
            return "groups must be > 0";
        if (p.counterMax == 0)
            return "the group counter maximum must be > 0";
        if (p.nMax == 0 || p.nMax > p.counterMax)
            return "N_max must be in [1, group counter maximum]";
        if (p.nMin == 0 || p.nMin > p.nMax)
            return "N_min must be in [1, N_max]";
        if (p.mode == Mode::Tcp && p.tcpNInit < p.nMin)
            return "tcp N_init must be >= N_min";
        if (p.mode == Mode::Amc && p.amcNInit < p.nMin)
            return "amc N_init must be >= N_min";
        if (p.mode == Mode::Fixed && p.nFixed == 0)
            return "N_fixed must be >= 1";
        if (p.mode == Mode::Tcp &&
            (p.tcpNInit == 0 || p.tcpNInit > p.nMax))
            return "tcp N_init must be in [1, N_max]";
        if (p.mode == Mode::Tcp &&
            (p.ssthreshInit == 0 || p.ssthreshInit > p.nMax))
            return "ssthresh_init must be in [1, N_max]";
        if (p.mode == Mode::Tcp &&
            (p.calmLen == 0 || p.upShift >= 64 || p.downShift >= 64))
            return "calm_len must be >= 1 and the shifts < 64";
        if (p.mode == Mode::Amc &&
            (p.amcNInit == 0 || p.amcNInit > p.nMax))
            return "amc N_init must be in [1, N_max]";
        if (p.mode == Mode::Amc &&
            !(p.amcTargetFrac > 0.0 && p.amcLo >= 0.0 &&
              p.amcLo < p.amcHi))
            return "amc needs target_frac > 0 and 0 <= lo < hi";
        return "";
    }

    explicit WindowController(const Params &_p)
        : p(_p),
          n(_p.mode == Mode::Tcp ? _p.tcpNInit :
            _p.mode == Mode::Amc ? _p.amcNInit : _p.nFixed),
          ssth(_p.ssthreshInit),
          pend(n),
          counters(_p.groups, 0),
          decideFlags(_p.groups, 0)
    {
    }

    void setListener(Listener l) { listener = std::move(l); }

    /** Grant the first frame. */
    void start() { grant(); }

    /** K for N: max(1, G >> floor(log2 N)). */
    static unsigned
    kFor(unsigned groups, uint64_t n_val)
    {
        unsigned lg = 0;
        while ((n_val >> (lg + 1)) != 0)
            lg++;
        const unsigned k = lg >= 32 ? 0 : (groups >> lg);
        return std::max(1u, k);
    }

    /** A demand access (hit or miss) reached the cache. */
    void
    demandAccess(bool hit)
    {
        a++;
        if (!hit)
            missesI++;
    }

    /** A demand access matched an expired tag. */
    void
    retentionMiss()
    {
        r++;
        rI++;
    }

    /**
     * End of a frame: update the group counters, run the controller and
     * grant the next frame.
     */
    void
    frameEnd()
    {
        for (unsigned g = 0; g < p.groups; g++) {
            counters[g] = decideFlags[g] ? 0 :
                std::min(counters[g] + 1, p.counterMax);
        }

        if (p.mode == Mode::Tcp) {
            bool grew = false;
            if (r > (a >> p.upShift)) {
                const unsigned old_n = n;
                n = n < ssth ? std::min({2 * n, ssth, p.nMax}) :
                    std::min(n + 1, p.nMax);
                streak = 0;
                calm = 0;
                r = 0;
                a = 0;
                grew = n > old_n;
                notify(grew ? Event::Grow : Event::GrowCapped, old_n, n);
            }
            if (!grew && grewLast)
                setSsthresh(n);
            grewLast = grew;
        }

        checkCtr++;
        lastCheckFlag = checkCtr >= n;
        if (lastCheckFlag)
            checkCtr = 0;

        if (p.mode == Mode::Tcp && lastCheckFlag && a >= p.minAccesses) {
            if (r < (a >> p.downShift)) {
                calm++;
            } else {
                calm = 0;
                streak = 0;
            }
            if (calm >= p.calmLen) {
                const unsigned old_n = n;
                if (streak < p.streakLen) {
                    n = std::max(n - 1, p.nMin);
                    streak++;
                    notify(Event::Dec, old_n, n);
                } else {
                    setSsthresh(n);
                    n = std::max(n >> 1, p.nMin);
                    notify(Event::Halve, old_n, n);
                }
                calm = 0;
            }
            // Every judged check starts the next one on fresh data
            r = 0;
            a = 0;
        }

        if (p.mode == Mode::Amc) {
            if (pend > n) {
                const unsigned old_n = n;
                n = pend;
                notify(Event::AmcGrow, old_n, n);
            } else if (lastCheckFlag && pend < n) {
                const unsigned old_n = n;
                n = pend;
                notify(Event::AmcShrink, old_n, n);
            }
        }

        grant();
    }

    /** amc: one decision from the interval's counters. */
    void
    amcInterval()
    {
        if (p.mode != Mode::Amc)
            return;
        const double ideal = double(missesI) - double(rI);
        const double target = p.amcTargetFrac * ideal;
        const unsigned old_pend = pend;
        if (double(rI) > p.amcHi * target) {
            pend = std::min(2 * n, p.nMax);
        } else if (double(rI) < p.amcLo * target) {
            pend = std::max(n / 2, p.nMin);
        } else {
            pend = n;
        }
        lastAmcMisses = missesI;
        lastAmcR = rI;
        missesI = 0;
        rI = 0;
        notify(Event::AmcDecide, old_pend, pend);
    }

    bool decide(unsigned g) const { return decideFlags[g] != 0; }
    unsigned nValue() const { return n; }
    unsigned ssthresh() const { return ssth; }
    unsigned pending() const { return pend; }
    unsigned k() const { return kFor(p.groups, n); }
    unsigned counter(unsigned g) const { return counters[g]; }
    unsigned groups() const { return p.groups; }
    uint64_t rCount() const { return r; }
    uint64_t aCount() const { return a; }
    unsigned streakValue() const { return streak; }
    unsigned calmValue() const { return calm; }
    unsigned checkCounter() const { return checkCtr; }
    /** The last frameEnd ran a check. */
    bool lastCheck() const { return lastCheckFlag; }
    /** The last grant: groups picked, due, and due but waiting. */
    unsigned lastPicked() const { return picked; }
    unsigned lastDue() const { return due; }
    unsigned lastWaited() const { return due - picked; }
    /** The last amc decision's interval counters. */
    uint64_t lastAmcMissCount() const { return lastAmcMisses; }
    uint64_t lastAmcRCount() const { return lastAmcR; }
    const Params &params() const { return p; }

  private:
    void
    notify(Event e, unsigned old_val, unsigned new_val)
    {
        if (listener)
            listener(e, old_val, new_val);
    }

    void
    setSsthresh(unsigned v)
    {
        if (v != ssth) {
            const unsigned old = ssth;
            ssth = v;
            notify(Event::Ssthresh, old, v);
        }
    }

    void
    grant()
    {
        const unsigned k_now = k();
        std::fill(decideFlags.begin(), decideFlags.end(), 0);
        picked = 0;
        due = 0;
        unsigned last = 0;
        for (unsigned i = 0; i < p.groups; i++) {
            const unsigned g = (rr + i) % p.groups;
            // Due every N frames: c_g counts the frames after the
            // deciding one
            if (uint64_t(counters[g]) + 1 >= n) {
                due++;
                if (picked < k_now) {
                    decideFlags[g] = 1;
                    picked++;
                    last = g;
                }
            }
        }
        if (picked)
            rr = (last + 1) % p.groups;
    }

    const Params p;
    unsigned n;
    unsigned ssth;
    unsigned pend;
    std::vector<unsigned> counters;
    std::vector<char> decideFlags;
    unsigned rr = 0;
    unsigned checkCtr = 0;
    bool lastCheckFlag = false;
    unsigned picked = 0;
    unsigned due = 0;
    unsigned streak = 0;
    unsigned calm = 0;
    bool grewLast = false;
    uint64_t r = 0;
    uint64_t a = 0;
    uint64_t missesI = 0;
    uint64_t rI = 0;
    uint64_t lastAmcMisses = 0;
    uint64_t lastAmcR = 0;
    Listener listener;
};

} // namespace gem5

#endif // __MEM_CACHE_RETENTION_WINDOW_CONTROLLER_HH__
