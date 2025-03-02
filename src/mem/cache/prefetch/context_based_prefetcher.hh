#ifndef __MEM_CACHE_PREFETCH_CONTEXT_BASED_PREFETCHER_HH__
#define __MEM_CACHE_PREFETCH_CONTEXT_BASED_PREFETCHER_HH__

#include <unordered_map>
#include <deque>
#include <vector>
#include <fstream>
#include <queue>
#include <cstdlib> // Include for rand() and srand()
#include <ctime> // Include for time()
#include "mem/cache/prefetch/queued.hh"

namespace gem5
{

struct ContextBasedPrefetcherParams;

namespace prefetch
{

class RewardFunction
{
public:
    RewardFunction(int target_distance, int window_size)
        : targetDistance(target_distance), windowSize(window_size) {}

    int operator()(int distance) const
    {
        int peak_distance = 30;
        int zero_distance = 90;
        double a = 0.1; // Controls the steepness of the quadratic curve

        // Reward peaks at 30 and decreases linearly to 0 at 90
        if (distance >= peak_distance && distance <= zero_distance) {
            return static_cast<int>(100 * (1 - (distance - peak_distance) / static_cast<double>(zero_distance - peak_distance)));
        }
        // Quadratic transition to negative rewards for distances below 30
        else if (distance < peak_distance) {
            return static_cast<int>(-a * std::pow(distance - peak_distance, 2));
        }
        // Quadratic transition to negative rewards for distances above 90
        else {
            return static_cast<int>(-a * std::pow(distance - zero_distance, 2));
        }
    }

  private:
    int targetDistance;
    int windowSize;
};

class ContextBasedPrefetcher : public Queued
{
  public:
    ContextBasedPrefetcher(const ContextBasedPrefetcherParams &p);
    ~ContextBasedPrefetcher() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;

    void notifyFill(const CacheAccessProbeArg &arg) override;

  private:
    struct State {
        std::priority_queue<std::pair<int, Addr>> ptrs; // Heap for pointers to addresses
        int counter; // Counter for the context
    };

    struct PrefetchEntry {
        Addr addr; // Prefetch address
        int key;  // Context key
        int index; // Index in the prefetch queue
    };

    int prefetchWindow; // Size of the prefetch window
    RewardFunction rewardFunction; // Reward function
    std::unordered_map<int, State> states; // States for each context
    std::deque<Addr> previousAccesses; // Previous accesses for context
    std::deque<PrefetchEntry> prefetchQueue; // Prefetch queue
    std::vector<int> mostSeenOffsets; // List of most seen offsets
    std::vector<int> mostProbableOffsets; // List of most probable offsets
    int rewardThreshold; // Reward threshold for prefetching
    int confidenceThreshold; // Confidence threshold for prefetching
    std::vector<int> rewards; // Track rewards for dynamic threshold adjustment
    std::unordered_map<int, int> offsetFrequencies; // Track frequencies of offsets

    int hash(Addr addr) const; // Hash function for context
    void addToState(int key, Addr addr, int index); // Add address to state
    std::vector<Addr> getPrefetches(int key) ; // Get the best addresses for a context
    void updateScores(Addr addr); // Update scores based on prefetch queue
    void updatePreviousAccesses(Addr addr); // Update previous accesses
    std::vector<int> getMostSeenOffsets() const; // Get most seen offsets
    void updateRewardThreshold(); // Update the reward threshold dynamically
    void updateOffsets(); // Update offsets dynamically

};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_CONTEXT_BASED_PREFETCHER_HH__