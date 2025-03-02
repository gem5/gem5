#include "mem/cache/prefetch/context_based_prefetcher.hh"
#include "debug/HWPrefetch.hh"
#include "params/ContextBasedPrefetcher.hh"
#include <cmath>
#include <numeric> // Include for std::accumulate
#include <algorithm> // Include for std::max_element
#include <cstdlib> // Include for rand() and srand()
#include <ctime> // Include for time()
#include <sstream> // Include for std::ostringstream

namespace gem5
{

namespace prefetch
{
    int index = 0;
    int cumulativeReward = 0;
    int rewardCounter = 0;


ContextBasedPrefetcher::ContextBasedPrefetcher(const ContextBasedPrefetcherParams &p)
    : Queued(p),
      prefetchWindow(p.prefetch_window),
      rewardFunction(p.target_prefetch_distance, p.prefetch_window),
      rewardThreshold(60), // Initial reward threshold
      confidenceThreshold(10) // Initial confidence threshold
{
    // Seed the random number generator
    std::srand(std::time(nullptr));
    mostProbableOffsets = { 64, 128, 256, 512, 1024, 2048, 4096};
    mostSeenOffsets = { 128, 1024};
   
}

void
ContextBasedPrefetcher::calculatePrefetch(const PrefetchInfo &pfi,
                                          std::vector<AddrPriority> &addresses,
                                          const CacheAccessor &cache)
{
    Addr addr = blockAddress(pfi.getAddr());
    int key = hash(addr);

    // Data collection: Update state with previous accesses
    for (size_t i = 0; i < previousAccesses.size(); ++i) {
        if( i % 40 != 0) {
            continue;
        }
        Addr a = previousAccesses[i];
        int prev_key = hash(a);
        addToState(prev_key, addr, i);
    }

    for (int offset : mostSeenOffsets) {
        Addr target_addr = addr + offset;
        int reward = rewardFunction(30);
        cumulativeReward += reward;
        rewardCounter++;
        states[hash(addr)].ptrs.push({reward, target_addr});
    }

    // Prediction: Get the best addresses for the current context
    if (states.find(key) != states.end()) {
        std::vector<Addr> best_addrs = getPrefetches(key);
        for (const auto &best_addr : best_addrs) {
            prefetchQueue.push_back({best_addr, key, (int)prefetchQueue.size()});
            addresses.push_back(AddrPriority(best_addr, 0));
        }
    }

    // Feedback: Update scores based on prefetch queue
    updateScores(addr);
    // Update previous accesses
    updatePreviousAccesses(addr);

    // Update reward threshold
    updateRewardThreshold();

    // Update offsets dynamically
    // updateOffsets();
}

void
ContextBasedPrefetcher::notifyFill(const CacheAccessProbeArg &arg)
{

}

int
ContextBasedPrefetcher::hash(Addr addr) const
{
    int hash_value = (addr >> 2) & 0x3FF;  // Use lower bits of address
    hash_value *= 2654435761;              // Multiply by a large prime for better mixing
    hash_value &= 0x3FF;                   // Keep the result within a 10-bit range
    return hash_value;
}

void
ContextBasedPrefetcher::addToState(int key, Addr addr, int index)
{
    if (states.find(key) == states.end()) {
        states[key] = State();
    }
    int reward = rewardFunction(index);
    cumulativeReward += reward;
    rewardCounter++;
    states[key].ptrs.push({reward, addr});
}

std::vector<Addr>
ContextBasedPrefetcher::getPrefetches(int key) 
{
    const auto &ptrs = states.at(key).ptrs;
    std::vector<Addr> best_addrs;

    if (ptrs.empty()) {
        return best_addrs; // Return empty vector if there are no addresses
    }

    // Generate a random number between 0 and 1
    double random_value = static_cast<double>(std::rand()) / RAND_MAX;
    bool explore = random_value < 0.2; // Explore with 20% probability

    auto temp_ptrs = ptrs; // Copy the heap to a temporary variable
    int degree = 0;

    while (!temp_ptrs.empty() && degree < 2) {
        auto top = temp_ptrs.top();
        if ((explore && top.first < confidenceThreshold) || (!explore && top.first > confidenceThreshold)) {
            best_addrs.push_back(top.second);
            degree++;
        }
        temp_ptrs.pop();
    }

    return best_addrs;
}

void
ContextBasedPrefetcher::updateScores(Addr addr)
{
    auto it = prefetchQueue.begin();
    while (it != prefetchQueue.end()) {
        it = std::find_if(it, prefetchQueue.end(), [&](const PrefetchEntry &e) {
            return e.addr == addr;
        });

        if (it != prefetchQueue.end()) {
            int distance = std::distance(prefetchQueue.begin(), it);
            int reward = rewardFunction(distance);
            cumulativeReward += reward;
            rewardCounter++;
            states[it->key].ptrs.push({reward, addr});
            ++it; // Move iterator to the next element to continue searching
        }
    }

    // Ensure the prefetch queue does not exceed the size of prefetchWindow
    while (prefetchQueue.size() > prefetchWindow) {
        prefetchQueue.pop_front();
    }
}

void
ContextBasedPrefetcher::updatePreviousAccesses(Addr addr)
{
    previousAccesses.push_back(addr);
    if (previousAccesses.size() > prefetchWindow) {
        previousAccesses.pop_front();
    }
}

std::vector<int>
ContextBasedPrefetcher::getMostSeenOffsets() const
{
    // Return the most seen offsets (as in Python script)
    return mostSeenOffsets;
}

void
ContextBasedPrefetcher::updateRewardThreshold()
{
    if (rewardCounter > 0) {
        int mean_reward = cumulativeReward / rewardCounter;
        rewardThreshold = mean_reward * 1.5; // Adjust multiplier as needed
    }
}

void
ContextBasedPrefetcher::updateOffsets()
{
    // Update the most seen offsets based on observed frequencies
    std::unordered_map<int, int> offset_count;
    for (size_t i = 0; i < previousAccesses.size(); i += 10) { // Traverse through every 10th element
        Addr addr = previousAccesses[i];
        for (const auto &offset : mostProbableOffsets) {
            Addr target_addr = addr + offset;
            if (std::find(previousAccesses.begin(), previousAccesses.end(), target_addr) != previousAccesses.end()) {
                offset_count[offset]++;
            }
        }
    }

    std::vector<std::pair<int, int>> sorted_offsets(offset_count.begin(), offset_count.end());
    std::partial_sort(sorted_offsets.begin(), sorted_offsets.begin() + std::min(size_t(10), sorted_offsets.size()), sorted_offsets.end(), [](const auto &a, const auto &b) {
        return a.second > b.second;
    });

    mostSeenOffsets.clear();
    for (size_t i = 0; i < sorted_offsets.size() && i < 10; ++i) {
        if (sorted_offsets[i].second > 0) { // Only add offsets with a count greater than 0
            mostSeenOffsets.push_back(sorted_offsets[i].first);
        }
    }
}

} // namespace prefetch
} // namespace gem5 