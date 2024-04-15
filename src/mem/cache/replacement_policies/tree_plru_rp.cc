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

#include <cmath>

#include "base/intmath.hh"
#include "base/logging.hh"
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
    return std::floor((index - 1) / 2);
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
    return 2 * index + 1;
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
    return 2 * index + 2;
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
    return index % 2 == 0;
}

TreePLRU::TreePLRUReplData::TreePLRUReplData(const uint64_t index,
                                             std::shared_ptr<PLRUTree> tree)
    : index(index), tree(tree)
{}

TreePLRU::TreePLRU(const Params &p)
    : Base(p), numLeaves(p.num_leaves), count(0), treeInstance(nullptr)
{
    fatal_if(!isPowerOf2(numLeaves),
             "Number of leaves must be non-zero and a power of 2");
}

void
TreePLRU::invalidate(const std::shared_ptr<ReplacementData> &replacement_data)
{
    // Cast replacement data
    std::shared_ptr<TreePLRUReplData> treePLRU_replacement_data =
        std::static_pointer_cast<TreePLRUReplData>(replacement_data);
    PLRUTree *tree = treePLRU_replacement_data->tree.get();

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
TreePLRU::touch(const std::shared_ptr<ReplacementData> &replacement_data) const
{
    // Cast replacement data
    std::shared_ptr<TreePLRUReplData> treePLRU_replacement_data =
        std::static_pointer_cast<TreePLRUReplData>(replacement_data);
    PLRUTree *tree = treePLRU_replacement_data->tree.get();

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
TreePLRU::reset(const std::shared_ptr<ReplacementData> &replacement_data) const
{
    // A reset has the same functionality of a touch
    touch(replacement_data);
}

ReplaceableEntry *
TreePLRU::getVictim(const ReplacementCandidates &candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Get tree
    const PLRUTree *tree = std::static_pointer_cast<TreePLRUReplData>(
                               candidates[0]->replacementData)
                               ->tree.get();

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
    return candidates[tree_index - (numLeaves - 1)];
}

std::shared_ptr<ReplacementData>
TreePLRU::instantiateEntry()
{
    // Generate a tree instance every numLeaves created
    if (count % numLeaves == 0) {
        treeInstance = new PLRUTree(numLeaves - 1, false);
    }

    // Create replacement data using current tree instance
    TreePLRUReplData *treePLRUReplData =
        new TreePLRUReplData((count % numLeaves) + numLeaves - 1,
                             std::shared_ptr<PLRUTree>(treeInstance));

    // Update instance counter
    count++;

    return std::shared_ptr<ReplacementData>(treePLRUReplData);
}

} // namespace replacement_policy
} // namespace gem5
