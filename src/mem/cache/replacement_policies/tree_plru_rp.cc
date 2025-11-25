/**
 * Copyright (c) 2018-2020 Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * Definitions of a Tree-PLRU replacement policy, along with some helper
 * tree indexing functions, which map an index to the tree 2D-array.
 */

#include "mem/cache/replacement_policies/tree_plru_rp.hh"

#include <cassert>
#include <cmath>
#include <cstdint>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "mem/packet.hh"
#include "params/TreePLRURP.hh"

namespace gem5
{

namespace replacement_policy
{

/**
 * Get the index of the parent of the given indexed subtree.
 *
 * @param Index of the queried tree.
 * @return The index of the parent tree.
 */
static uint64_t
parentIndex(const uint64_t index)
{
    return std::floor((index-1)/2);
}

/**
 * Get index of the subtree on the left of the given indexed tree.
 *
 * @param index The index of the queried tree.
 * @return The index of the subtree to the left of the queried tree.
 */
static uint64_t
leftSubtreeIndex(const uint64_t index)
{
    return 2*index + 1;
}

/**
 * Get index of the subtree on the right of the given indexed tree.
 *
 * @param index The index of the queried tree.
 * @return The index of the subtree to the right of the queried tree.
 */
static uint64_t
rightSubtreeIndex(const uint64_t index)
{
    return 2*index + 2;
}

/**
 * Find out if the subtree at index corresponds to the right or left subtree
 * of its parent tree.
 *
 * @param index The index of the subtree.
 * @return True if it is a right subtree, false otherwise.
 */
static bool
isRightSubtree(const uint64_t index)
{
    return index%2 == 0;
}


TreePLRU::TreePLRUReplData::TreePLRUReplData(
        const uint64_t index, std::shared_ptr<PLRUTree> tree, TreePLRU *parent)
    : index(index), tree(tree), parent(parent)
{
}

void
TreePLRU::TreePLRUReplData::setOwnerSet(uint32_t set)
{
        owner_set = set;
        if (parent && tree)
                parent->registerTreeForSet(set, tree);
}

TreePLRU::TreePLRU(const Params &p)
  : Base(p), numLeaves(p.num_leaves), count(0), treeInstance(nullptr)
{
    fatal_if(numLeaves < 1,
        "numLeaves should never be 0");

    // initialize per-set domain policy containers lazily in setDomainPolicy.

    // precompute nodeLeafMask for internal nodes
    // represent leaves as bits in a uint64_t (up to 64-way assoc)
    // if numLeaves is 1, there are no internal nodes.
    const unsigned int numNodes = (numLeaves > 0) ? (numLeaves - 1) : 0;
    nodeLeafMask.resize(numNodes);

    for (unsigned int leaf = 0; leaf < numLeaves; ++leaf) {
        // virtual node index representing this leaf in the full binary tree
        unsigned int node_idx = leaf + numNodes;
        // walk up to root, marking parent internal nodes with this leaf
        while (true) {
            if (node_idx == 0) break;
            unsigned int parent = parentIndex(node_idx);
            if (parent < numNodes) {
                nodeLeafMask[parent] |= (uint64_t(1) << leaf);
                node_idx = parent;
            } else {
                break;
            }
        }
    }

    // setTrees initialized when sets are known at tagsInit time
}

void
TreePLRU::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    // Cast replacement data
    std::shared_ptr<TreePLRUReplData> treePLRU_replacement_data =
        std::static_pointer_cast<TreePLRUReplData>(replacement_data);
    PLRUTree* tree = treePLRU_replacement_data->tree.get();

    // Index of the tree entry we are currently checking
    // Make this entry the new LRU entry
    uint64_t tree_index = treePLRU_replacement_data->index;

    // Parse and update tree to make it point to the new LRU
    do {
        // Store whether we are coming from a left or right node
        const bool right = isRightSubtree(tree_index);

        // Go to the parent tree node
        tree_index = parentIndex(tree_index);

        // Update parent node to make it point to the node we just came from
        tree->at(tree_index) = right;
    } while (tree_index != 0);
}

void
TreePLRU::touch(const std::shared_ptr<ReplacementData>& replacement_data)
const
{
    // Cast replacement data
    std::shared_ptr<TreePLRUReplData> treePLRU_replacement_data =
        std::static_pointer_cast<TreePLRUReplData>(replacement_data);
    PLRUTree* tree = treePLRU_replacement_data->tree.get();

    // Index of the tree entry we are currently checking
    // Make this entry the MRU entry
    uint64_t tree_index = treePLRU_replacement_data->index;

    // Parse and update tree to make every bit point away from the new MRU
    do {
        // Store whether we are coming from a left or right node
        const bool right = isRightSubtree(tree_index);

        // Go to the parent tree node
        tree_index = parentIndex(tree_index);

        // Update node to not point to the touched leaf
        tree->at(tree_index) = !right;
    } while (tree_index != 0);
}

void
TreePLRU::reset(const std::shared_ptr<ReplacementData>& replacement_data)
const
{
    // A reset has the same functionality of a touch
    touch(replacement_data);
}

ReplaceableEntry*
TreePLRU::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);
    // Get tree
    const PLRUTree* tree = std::static_pointer_cast<TreePLRUReplData>(
            candidates[0]->replacementData)->tree.get();

    // Index of the tree entry we are currently checking. Start with root.
    uint64_t tree_index = 0;

    // Parse tree
    while (tree_index < tree->size()) {
        // Go to the next tree entry
        if (tree->at(tree_index)) {
            tree_index = rightSubtreeIndex(tree_index);
        } else {
            tree_index = leftSubtreeIndex(tree_index);
        }
    }

    // The tree index is currently at the leaf of the victim displaced by the
    // number of non-leaf nodes
    return candidates.at(tree_index - (numLeaves - 1));
}

ReplaceableEntry*
TreePLRU::getVictimForDomain(
    const ReplacementCandidates& candidates,
    const uint32_t domain_id) const
{
    // if no per-set policies exist, fall back to default behavior
    if (setDomainPolicies.empty())
        return getVictim(candidates);

    // all candidates share the same set index
    uint32_t set = candidates[0]->getSet();

    if (set >= setDomainPolicies.size())
        return getVictim(candidates);

    const DomainPolicyMap &map = setDomainPolicies[set];
    auto it = map.find(domain_id);

    // if no policy for this domain, fallback
    if (it == map.end())
        return getVictim(candidates);

    uint64_t policy = it->second;

    // if policy is all-ones, fallback to default victim selection.
    if (policy == UINT64_MAX)
        return getVictim(candidates);

    const PLRUTree* tree = std::static_pointer_cast<TreePLRUReplData>(
            candidates[0]->replacementData)->tree.get();

    const unsigned int numNodes = (numLeaves > 0) ? (numLeaves - 1) : 0;

    unsigned int node = 0;
    // load precomputed masks if available
    uint64_t nodeMaskPacked = 0;
    uint64_t singleDirPacked = 0;
    if (set < setDomainNodeMasks.size()) {
        auto itnm = setDomainNodeMasks[set].find(domain_id);
        if (itnm != setDomainNodeMasks[set].end())
            nodeMaskPacked = itnm->second;

        auto itsd = setDomainSingleDir[set].find(domain_id);
        if (itsd != setDomainSingleDir[set].end())
            singleDirPacked = itsd->second;
    }

    while (node < numNodes) {
        unsigned int left = leftSubtreeIndex(node);
        unsigned int right = rightSubtreeIndex(node);

        // if nodeMaskPacked has bit=1, there are multiple allowed leaves under
        // this node and follow the tree bit to choose MRU/LRU direction
        if (nodeMaskPacked & (uint64_t(1) << node)) {
            bool goLeft = !tree->at(node);
            node = goLeft ? left : right;
            continue;
        }

        // if single-direction bit is set, force to that side
        // otherwise determine which side has allowed leaves by checking policy directly
        if (singleDirPacked & (uint64_t(1) << node)) {
            node = right;
            continue;
        }

        // fallback-> compute which side has allowed leaves
        uint64_t leftLeaves = 0;
        uint64_t rightLeaves = 0;

        if (left < numNodes)
            leftLeaves = nodeLeafMask[left];
        else {
            unsigned int leafIdx = left - numNodes;
            if (leafIdx < numLeaves)
                leftLeaves = (uint64_t(1) << leafIdx);
        }

        if (right < numNodes)
            rightLeaves = nodeLeafMask[right];
        else {
            unsigned int leafIdx = right - numNodes;
            if (leafIdx < numLeaves)
                rightLeaves = (uint64_t(1) << leafIdx);
        }

        bool leftHas = (policy & leftLeaves) != 0;
        bool rightHas = (policy & rightLeaves) != 0;

        if (leftHas && rightHas)
            node = (!tree->at(node)) ? left : right;
        else if (leftHas)
            node = left;
        else if (rightHas)
            node = right;
        else
            return getVictim(candidates);
    }

    unsigned int leaf = node - numNodes;
    unsigned int target_way = leaf;

    for (auto *c : candidates) {
        if (c->getWay() == target_way)
            return const_cast<ReplaceableEntry*>(c);
    }

    // If target way not in candidates, pick first candidate allowed by policy.
    for (auto *c : candidates) {
        if (policy & (uint64_t(1) << c->getWay()))
            return const_cast<ReplaceableEntry*>(c);
    }

    return getVictim(candidates);
}

void
TreePLRU::touch(const std::shared_ptr<ReplacementData>& replacement_data,
                const PacketPtr pkt)
{
    // If no packet or no request, fallback to non-domain touch
    if (!pkt || !pkt->req) {
        touch(replacement_data);
        return;
    }

    const uint32_t domain = pkt->req->domainId();

    std::shared_ptr<TreePLRUReplData> treePLRU_replacement_data =
        std::static_pointer_cast<TreePLRUReplData>(replacement_data);
    PLRUTree* tree = treePLRU_replacement_data->tree.get();

    uint64_t policy = UINT64_MAX;
    uint32_t set = treePLRU_replacement_data->owner_set;
    if (set < setDomainPolicies.size()) {
        auto it = setDomainPolicies[set].find(domain);
        if (it != setDomainPolicies[set].end())
            policy = it->second;
    }

    // If no policy is present, do full touch
    if (policy == UINT64_MAX) {
        touch(replacement_data);
        return;
    }

    // Load precomputed node mask for this (set,domain), if present.
    uint64_t nodeMaskPacked = 0;
    if (set < setDomainNodeMasks.size()) {
        auto itnm = setDomainNodeMasks[set].find(domain);
        if (itnm != setDomainNodeMasks[set].end())
            nodeMaskPacked = itnm->second;
    }

    // Update only ancestors whose corresponding bit is set in nodeMaskPacked
    uint64_t tree_index = treePLRU_replacement_data->index;
    do {
        const bool right = isRightSubtree(tree_index);
        tree_index = parentIndex(tree_index);

        if (tree_index < nodeLeafMask.size()) {
            if (nodeMaskPacked & (uint64_t(1) << tree_index)) {
                tree->at(tree_index) = !right;
            }
        }
    } while (tree_index != 0);
}

void
TreePLRU::reset(const std::shared_ptr<ReplacementData>& replacement_data,
                const PacketPtr pkt)
{
    // Reset behaves like touch for masked updates
    touch(replacement_data, pkt);
}

std::shared_ptr<ReplacementData>
TreePLRU::instantiateEntry()
{
    // Generate a tree instance every numLeaves created
    if (count % numLeaves == 0) {
        treeInstance = new PLRUTree(numLeaves - 1, false);
    }

    // Create replacement data using current tree instance
    TreePLRUReplData* treePLRUReplData = new TreePLRUReplData(
        (count % numLeaves) + numLeaves - 1,
        std::shared_ptr<PLRUTree>(treeInstance), this);

    // Update instance counter
    count++;

    return std::shared_ptr<ReplacementData>(treePLRUReplData);
}

void
TreePLRU::setDomainPolicy(const uint32_t set, const uint32_t domain_id,
                          const uint64_t policy_fillmap)
{
    if (set >= setDomainPolicies.size()) {
        setDomainPolicies.resize(set + 1);
        setDomainNodeMasks.resize(set + 1);
        setDomainSingleDir.resize(set + 1);
    }

    setDomainPolicies[set][domain_id] = policy_fillmap;

    // precompute node masks and single-direction bits for this (set,domain).
    const unsigned int numNodes = (numLeaves > 0) ? (numLeaves - 1) : 0;

    uint64_t nodeMaskPacked = 0; // bit i -> node i has >1 allowed leaves
    uint64_t singleDirPacked = 0; // bit i -> node i's single allowed leaf is on right

    for (unsigned int node = 0; node < numNodes; ++node) {
        uint64_t leftLeaves = 0;
        uint64_t rightLeaves = 0;

        unsigned int left = leftSubtreeIndex(node);
        unsigned int right = rightSubtreeIndex(node);

        if (left < numNodes)
            leftLeaves = nodeLeafMask[left];
        else {
            unsigned int leafIdx = left - numNodes;
            if (leafIdx < numLeaves)
                leftLeaves = (uint64_t(1) << leafIdx);
        }

        if (right < numNodes)
            rightLeaves = nodeLeafMask[right];
        else {
            unsigned int leafIdx = right - numNodes;
            if (leafIdx < numLeaves)
                rightLeaves = (uint64_t(1) << leafIdx);
        }

        const uint64_t leftOverlap = policy_fillmap & leftLeaves;
        const uint64_t rightOverlap = policy_fillmap & rightLeaves;

        const int leftCount = __builtin_popcountll(leftOverlap);
        const int rightCount = __builtin_popcountll(rightOverlap);
        const int total = leftCount + rightCount;

        if (total > 1) {
            nodeMaskPacked |= (uint64_t(1) << node);
        } else if (total == 1) {
            // set single-dir bit to indicate which side contains the lone leaf
            if (rightCount == 1)
                singleDirPacked |= (uint64_t(1) << node);
        } else {
            // no allowed leaves under this node -> leave both bits 0
        }
    }

    setDomainNodeMasks[set][domain_id] = nodeMaskPacked;
    setDomainSingleDir[set][domain_id] = singleDirPacked;

    // container for setTrees exists
    if (set >= setTrees.size())
        setTrees.resize(set + 1);

    // sanitize: for each registered tree instance in the set, adjust
    // the internal node bits so that nodes covering exactly one allowed leaf
    // point toward that leaf deterministically.
    //
    // for nodes with multiple allowed leaves -> leave bits unchanged
    // for nodes with zero allowed leaves -> skip modification (victim selection will fallback)
    for (auto &weak_tree : setTrees[set]) {
        if (auto tree_ptr = weak_tree.lock()) {
            // for each internal node that has single allowed leaf -> force the corresponding direction
            // for nodes with multiple allowed leaves -> we don't alter existing bits to preserve recency order.
            for (unsigned int node = 0; node < numNodes; ++node) {
                uint64_t maskBit = (uint64_t(1) << node);
                // if node has exactly one allowed leaf and singleDirPacked set -> force the
                // node bit to point to the side containing that leaf
                if (singleDirPacked & maskBit) {
                    // singleDirPacked==1 means single leaf is on right subtree
                    tree_ptr->at(node) = true;
                } else if ((nodeMaskPacked & maskBit) == 0) {
                    // node has zero or one allowed leaf but not right-side single

                    // if it has zero allowed leaves-> avoid changing it
                    // if it had exactly one allowed leaf on left -> ensure it points left when possible
                    // check if left side contains any allowed leaf -> inspect nodeLeafMask
                    uint64_t overlap = policy_fillmap & nodeLeafMask[node];
                    if (__builtin_popcountll(overlap) == 1) {
                        // determine whether the lone leaf is on right or left
                        unsigned int left = leftSubtreeIndex(node);
                        uint64_t leftLeaves = 0;
                        if (left < numNodes)
                            leftLeaves = nodeLeafMask[left];
                        else {
                            unsigned int leafIdx = left - numNodes;
                            if (leafIdx < numLeaves)
                                leftLeaves = (uint64_t(1) << leafIdx);
                        }
                        bool loneOnRight = (policy_fillmap & leftLeaves) == 0;
                        tree_ptr->at(node) = loneOnRight;
                    }
                }
            }
        }
    }
}

void
TreePLRU::registerTreeForSet(uint32_t set, const std::shared_ptr<PLRUTree> &tree)
{
    if (set >= setTrees.size())
        setTrees.resize(set + 1);
    setTrees[set].push_back(std::weak_ptr<PLRUTree>(tree));
}

} // namespace replacement_policy
} // namespace gem5
