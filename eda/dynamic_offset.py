import math
import re
from collections import (
    defaultdict,
    deque,
)

# Filepath to the address stream file
file_path = './eda/addr.log'

# Initialize lists to store addresses and PCs
addresses = []
pcs = []

# Read the address stream file
with open(file_path, 'r') as file:
    for line in file:
        match = re.match(r'Addr: ([0-9a-fA-F]+), PC: ([0-9a-fA-F]+)', line)
        if match:
            addr = int(match.group(1), 16)  # Convert hex address to integer
            pc = int(match.group(2), 16)    # Convert hex PC to integer
            addresses.append(addr)
            pcs.append(pc)

class ContextBasedPrefetcher:
    def __init__(self, prefetch_window, reward_function):
        self.prefetchWindow = prefetch_window
        self.rewardFunction = reward_function
        self.states = defaultdict(lambda: {'ptrs': defaultdict(int), 'counter': 0})
        self.previousAccesses = deque()
        self.prefetchQueue = deque()
        self.reward_threshold = 600  # Initial reward threshold
        self.rewards = []  # Track rewards for dynamic threshold adjustment

    def hash(self, addr, pc):
        # Improved hash function: Mix address and PC more effectively
        # Use a combination of shifts, XOR, and multiplication
        hash_value = (addr >> 2) & 0x3FF  # Use lower bits of address
        hash_value ^= (pc & 0x3FF)        # XOR with lower bits of PC
        hash_value *= 2654435761          # Multiply by a large prime for better mixing
        hash_value &= 0x3FF               # Keep the result within a 10-bit range
        return hash_value

        # return ((addr >> 2) & 0x3FF) ^ (pc & 0x3FF)  # Combine addr and pc for a more unique key

    def addToState(self, key, addr):
        self.states[key]['ptrs'][addr] += 1

    def getBestAddress(self, key):
        ptrs = self.states[key]['ptrs']
        if not ptrs:
            return 0, 0  # Return 0 address and 0 reward if no pointers
        best_addr = max(ptrs, key=ptrs.get)
        reward = ptrs[best_addr]
        return best_addr, reward

    def getMostSeenOffsets(self):
        offset_counts = defaultdict(int)
        for i in range(1, len(self.previousAccesses)):
            offset = self.previousAccesses[i] - self.previousAccesses[i-1]
            offset_counts[offset] += 1
        # Return the top 10 most common offsets
        return sorted(offset_counts, key=offset_counts.get, reverse=True)[:10]

    def updateScores(self, addr):
        most_seen_offsets = self.getMostSeenOffsets()
        for index, p in enumerate(self.prefetchQueue):
            for offset in most_seen_offsets:
                target_addr = addr - offset
                if p['addr'] == target_addr:
                    distance = index
                    reward = self.rewardFunction(distance)
                    self.states[p['key']]['ptrs'][addr + offset] += reward
                    print(f'Reward: {reward}, Distance: {distance}, Addr: {addr:x}, Offset: {offset}')
                    break

        # Evict oldest entries first
        while len(self.prefetchQueue) > 128:
            oldest_entry = self.prefetchQueue.popleft()
            # Optionally, log or analyze evicted entries

    def updatePreviousAccesses(self, addr):
        self.previousAccesses.append(addr)
        if len(self.previousAccesses) > self.prefetchWindow:
            self.previousAccesses.popleft()

    def update_reward_threshold(self):
        if len(self.rewards) > 0:
            mean_reward = sum(self.rewards) / len(self.rewards)
            self.reward_threshold = mean_reward * 1.5 # Adjust multiplier as needed

    def predict(self, pc, addr):
        context = self.hash(addr, pc)
        for a in self.previousAccesses:
            prev_key = self.hash(a, pc)
            self.addToState(prev_key, addr)

        key = self.hash(addr, pc)
        best_addr, reward = 0, 0
        if key in self.states:
            best_addr, reward = self.getBestAddress(key)
            self.prefetchQueue.append({'addr': best_addr, 'key': key, 'index': len(self.prefetchQueue)})

        self.updateScores(addr)
        self.updatePreviousAccesses(addr)
        self.rewards.append(reward)
        self.update_reward_threshold()

        return best_addr if key in self.states else None, reward

# Improved reward function
def reward_function(distance):
    sigma = 10
    center = 30
    exponent = -pow(distance - center, 2) / (2 * pow(sigma, 2))
    reward = math.exp(exponent) * 100

    # Make rewards negative towards the extreme ends
    if distance < center - 2 * sigma or distance > center + 2 * sigma:
        reward = -reward * 20  # Increased penalty for incorrect predictions

    return reward

# Initialize the prefetcher
prefetcher = ContextBasedPrefetcher(prefetch_window=128, reward_function=reward_function)

# Track prediction accuracy and history
predictions = []
correct_predictions = 0

# Feed data to the algorithm
block_size = 64  # Define the block size
for pc, addr in zip(pcs, addresses):
    prediction, reward = prefetcher.predict(pc, addr)
    if reward >= prefetcher.reward_threshold:
        predictions.append((pc, addr, prediction))
        if any((p['addr'] // block_size) == (addr // block_size) for p in prefetcher.prefetchQueue):
            correct_predictions += 1

# Calculate accuracy
accuracy = correct_predictions / len(predictions) if predictions else 0

# Print results
print(f'Prediction Accuracy: {accuracy * 100:.2f}%')
print(len(predictions), 'predictions made')