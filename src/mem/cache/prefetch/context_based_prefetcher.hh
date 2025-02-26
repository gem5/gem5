#ifndef __MEM_CACHE_PREFETCH_CONTEXT_BASED_PREFETCHER_HH__
#define __MEM_CACHE_PREFETCH_CONTEXT_BASED_PREFETCHER_HH__

#include <unordered_map>
#include <deque>
#include <vector>
#include <fstream>
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
        // Bell-shaped reward function centered around targetDistance
        double sigma = 10;
        double center = 30;
        double exponent = -std::pow(distance - center, 2) / (2 * std::pow(sigma, 2));
        int reward = static_cast<int>(std::exp(exponent) * 100);

        // Make rewards negative towards the extreme ends
        if (distance < center - 2 * sigma || distance > center + 2 * sigma) {
            reward = -reward * 20;  // Increased penalty for incorrect predictions
        }

        return reward;
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
        std::unordered_map<Addr, int> ptrs; // Pointers to addresses
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
    int rewardThreshold; // Reward threshold for prefetching
    int confidenceThreshold; // Confidence threshold for prefetching
    std::vector<int> rewards; // Track rewards for dynamic threshold adjustment
    std::unordered_map<int, int> offsetFrequencies; // Track frequencies of offsets

    int hash(Addr addr, int pc) const; // Hash function for context
    void addToState(int key, Addr addr); // Add address to state
    Addr getBestAddress(int key) const; // Get the best address for a context
    void updateScores(Addr addr); // Update scores based on prefetch queue
    void updatePreviousAccesses(Addr addr); // Update previous accesses
    std::vector<int> getMostSeenOffsets() const; // Get most seen offsets
    void updateRewardThreshold(); // Update the reward threshold dynamically
    void updateOffsets(); // Update offsets dynamically

};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_CONTEXT_BASED_PREFETCHER_HH__