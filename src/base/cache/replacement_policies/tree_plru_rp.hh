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
 * Declaration of a Pseudo-Least Recently Used replacement policy.
 * The victim is chosen using a tree of bit timestamps.
 *
 * A tree contains consists of leafs that represent the direction to take when
 * searching for the LRU entry.
 *
 * Let's assume each tree contains 8 replacement data entries. For example, if
 * these entries are named from A to H, and the tree's bits are:
 *    ____1____
 *  __0__   __1__
 * _0_ _1_ _1_ _0_
 * A B C D E F G H
 *
 * Then the current PLRU entry is given by the sequence:
 *     1 (get right child) -> 1 (get right child) -> 0 (get left child) -> G
 *
 * When an entry is touched the bits of the parent nodes are iteratively
 * updated to point away from it. Therefore, if entry B is touched, its parent,
 * grandparents, etc would be updated, and we'd end up with the following tree:
 *    ____1____
 *  __1__   __1__
 * _0_ _1_ _1_ _0_
 * A B C D E F G H
 *
 * Explanation: The parent of B must point away from it, that is, to the left
 * child, but it is already doing so, so it is left unmodified (0). Then the
 * grandparent must point to the right subtree, as B belongs to its left
 * subtree (0 becomes 1). Lastly, the root must point away from the
 * grandparent, so it is left unmodified (0).
 *
 * For invalidations the process is similar to touches, but instead of pointing
 * away, the bits point toward the entry.
 *
 * Consecutive calls to instantiateEntry() use the same tree up to numLeaves.
 * When numLeaves replacement datas have been created, a new tree is generated,
 * and the counter is reset.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_TREE_PLRU_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_TREE_PLRU_RP_HH__

#include <cstdint>
#include <memory>
#include <vector>

#include "base/cache/replacement_policies/base.hh"

namespace gem5
{

struct TreePLRURPParams;

namespace replacement_policy
{

class TreePLRU : public Base
{
  private:
    /**
     * Instead of implementing the tree itself with pointers, it is implemented
     * as an array of bits. The index of the node defines its position in the
     * tree, and its parent. Index 0 represents the root, 1 is its left node,
     * and 2 is its right node. Then, in a BFS fashion this is expanded to the
     * following nodes (3 and 4 are respectively 1's left and right nodes, and
     * 5 and 6 are 2's left and right nodes, and so on).
     *
     * i.e., the following trees are equivalent in this representation:
     *    ____1____
     *  __0__   __1__
     * _0_ _1_ _1_ _0_
     * A B C D E F G H
     *
     * 1 0 1 0 1 1 0
     *
     * Notice that the replacement data entries are not represented in the tree
     * to avoid unnecessary storage costs.
     */
    typedef std::vector<bool> PLRUTree;

    /**
     * Number of leaves that share a single replacement data.
     */
    const uint64_t numLeaves;

    /**
     * Count of the number of sharers of a replacement data. It is used when
     * instantiating entries to share a replacement data among many replaceable
     * entries.
     */
    uint64_t count;

    /**
     * Holds the latest temporary tree instance created by instantiateEntry().
     */
    PLRUTree* treeInstance;

  protected:
    /**
     * Tree-PLRU-specific implementation of replacement data. Each replacement
     * data shares its tree with other entries.
     */
    struct TreePLRUReplData : ReplacementData
    {
        /**
         * Theoretical index of this replacement data in the tree. In practice,
         * the corresponding node does not exist, as the tree stores only the
         * nodes that are not leaves.
         */
        const uint64_t index;

        /**
         * Shared tree pointer. A tree is shared between numLeaves nodes, so
         * that accesses to a replacement data entry updates the PLRU bits of
         * all other replacement data entries in its set.
         */
        std::shared_ptr<PLRUTree> tree;

        /**
         * Default constructor. Invalidate data.
         *
         * @param index Index of the corresponding entry in the tree.
         * @param tree The shared tree pointer.
         */
        TreePLRUReplData(const uint64_t index, std::shared_ptr<PLRUTree> tree);
    };

  public:
    typedef TreePLRURPParams Params;
    TreePLRU(const Params &p);
    ~TreePLRU() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Makes tree leaf of replacement data the LRU (tree bits point to it).
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;

    /**
     * Touch an entry to update its replacement data.
     * Makes tree leaf of replacement data the MRU.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Reset replacement data. Used when an entry is inserted. Provides the
     * same functionality as touch().
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Find replacement victim using TreePLRU bits. It is assumed that all
     * candidates share the same replacement data tree.
     *
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const
                                                                     override;

    /**
     * Instantiate a replacement data entry. Consecutive calls to this
     * function use the same tree up to numLeaves. When numLeaves replacement
     * data have been created, a new tree is generated, and the counter is
     * reset.
     * Therefore, it is essential that entries that share the same replacement
     * data call this function consecutively.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_TREE_PLRU_RP_HH__
