/*
 * Copyright (c) 2014-2015 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 *
 * Authors: Kanishk Sugand
 *          Andreas Hansson
 */

#ifndef __MEM_STACK_DIST_CALC_HH__
#define __MEM_STACK_DIST_CALC_HH__

#include <limits>
#include <map>
#include <vector>

#include "base/types.hh"

/**
  * The stack distance calculator is a passive object that merely
  * observes the addresses pass to it. It calculates stack distances
  * of incoming addresses based on the partial sum hierarchy tree
  * algorithm described by Alamasi et al.
  * http://doi.acm.org/10.1145/773039.773043.
  *
  * A tree structure is maintained and updated at each transaction
  * (unique or non-unique). The tree is implemented as an STL vector
  * with layers of the form <map> Each layer in this tree is an
  * ordered map <uint64_t, Node*>. Nodes are structs which take form
  * of leaf, intermediate and root nodes. For example, in a tree with 3
  * layers, tree[0][5] gives a leaf node pointer for key=5 tree[1][1]
  * gives an intermediate node pointer for key=1 tree[2][0] gives the
  * root node in the tree.
  *
  * At every transaction a hash-map (aiMap) is looked up to check if
  * the address was already encountered before. Based on this lookup a
  * transaction can be termed as unique or non-unique.
  *
  * In addition to the normal stack distance calculation, a feature to
  * mark an old node in the tree is added. This is useful if it is
  * required to see the reuse pattern. For example, BackInvalidates
  * from a lower level (e.g. membus to L2), can be marked (isMarked
  * flag of Node set to True). Then later if this same address is
  * accessed (by L1), the value of the isMarked flag would be
  * True. This would give some insight on how the BackInvalidates
  * policy of the lower level affect the read/write accesses in an
  * application.
  *
  * There are two functions provided to interface with the calculator:
  * 1. pair<uint64_t, bool> calcStackDistAndUpdate(Addr r_address,
  *                                                bool addNewNode)
  * At every unique transaction a new leaf node is added at tree[0](leaf layer)
  * and linked to the layer above (if addNewNode is True). The sums of all
  * the intermediate nodes is updated till the root. The stack-distance is
  * returned as a Constant representing INFINITY.
  *
  * At every non-unique transaction the tree is traversed from the
  * leaf at the returned index to the root, the old node is deleted
  * from the tree, and the sums (to the right are collected) and
  * decremented. The collected sum represets the stack distance of the
  * found node. If this node was marked then a bool flag set to True
  * is returned with the stack_distance. During this operation a node
  * is discarded at the leaf layer always. Moreover during the
  * traversal upwards using the updateSum() method, if an intermediate
  * node is found with no children connected to it, then that is
  * discarded too.
  *
  * The return value of this function is a pair representing the
  * stack_distance and the value of the marked flag.
  *
  * 2. pair<uint64_t , bool> calcStackDist(Addr r_address, bool mark)
  * This is a stripped down version of the above function which is used to
  * just inspect the tree, and mark a leaf node (if mark flag is set). The
  * functionality to add a new node is removed.
  *
  * At every unique transaction the stack-distance is returned as a constant
  * representing INFINITY.
  *
  * At every non-unique transaction the tree is traversed from the
  * leaf at the returned index to the root, and the sums (to the right)
  * are collected. The collected sum represets the stack distance of
  * the found node.
  *
  * This function does NOT Modify the stack. (No node is added or
  * deleted).  It is just used to mark a node already created and get
  * its stack distance.
  *
  * The return value of this function is a pair representing the stack
  * distance and the value of the marked flag.
  *
  * The table below depicts the usage of the Algorithm using the functions:
  * pair<uint64_t Stack_dist, bool isMarked> calcStackDistAndUpdate
  *                                      (Addr r_address, bool addNewNode)
  * pair<uint64_t Stack_dist, bool isMarked> calcStackDist
  *                                              (Addr r_address, bool mark)
  *
  * |   Function           |   Arguments   |Return Val |Use For|
  * |calcStackDistAndUpdate|r_address, True|I/SD,False |A,GD,GM|
  * |calcStackDistAndUpdate|r_address,False|SD,prevMark|D,GD,GM|
  * |calcStackDist         |r_address,False|SD,prevMark|  GD,GM|
  * |calcStackDist         |r_address, True|SD,prevMark|  GD,GM|
  *
  * (*A: Allocate an address in stack, if old entry present then it is deleted,
  *  *U: Delete old-address from stack, no new entry is added
  *  *GD: Get-Stack distance of an address,
  *  *GM: Get value of Mark flag, indicates if that address has been touched
  *                                                                  before,
  *  *I: stack-distance = infinity,
  *  *SD: Stack Distance
  *  *r_address: address to be added, *prevMark: value of isMarked flag
  *                                                              of the Node)
  *
  * Invalidates refer to a type of packet that removes something from
  * a cache, either autonoumously (due-to cache's own replacement
  * policy), or snoops from other caches which invalidate something
  * inside our cache.
  *
  * Usage            |   Function to use    |Typical Use           |
  * Add new entry    |calcStackDistAndUpdate|Read/Write Allocate   |
  * Delete Old Entry |calcStackDistAndUpdate|Writebacks/Cleanevicts|
  * Dist.of Old entry|calcStackDist         |Cleanevicts/Invalidate|
  *
  * Node Balancing: The tree structure is maintained by an
  * updateTree() operation called when an intermediate node is
  * required. The update operation is roughly categorized as a root
  * update or intermediate layer update. When number of leaf nodes
  * grow over a power of 2 then a new layer is added at the top of the
  * tree and a new root node is initialized. The old node at the lower
  * layer is connected to this.  In an intermediate node update
  * operation a new intermediate node is added to the required layer.
  *
  * Debugging: Debugging can be enabled by setting the verifyStack flag
  * true. Debugging is implemented using a dummy stack that behaves in
  * a naive way, using STL vectors (i.e each unique address is pushed
  * on the top of an STL vector stack, and SD is returned as
  * Infinity. If a non unique address is encountered then the previous
  * entry in the STL vector is removed, all the entities above it are
  * pushed down, and the address is pushed at the top of the stack).
  *
  * A printStack(int numOfEntitiesToPrint) is provided to print top n entities
  * in both (tree and STL based dummy stack).
  */
class StackDistCalc
{

  private:

    struct Node;

    typedef std::map<uint64_t, Node*> IndexNodeMap;
    typedef std::map<Addr, uint64_t> AddressIndexMap;
    typedef std::vector<IndexNodeMap> TreeType;

    /**
     * Gets sum from the node upwards recursively till the root.  This
     * function is called first by getSumsLeavesToRoot, and then
     * recursively calls itself.
     *
     * @param node pointer to the node which is updated
     * @param from_left variable which says that the request arrived
     *        from the left
     * @param sum_from_below Sum of left and right children below
     * @param level level in the tree the calling node is located
     * @param stack_dist stack distance of the node below
     * @return The stack distance of the current address.
     *
     */
    uint64_t getSum(Node* node, bool from_left, uint64_t sum_from_below,
                    uint64_t stack_dist, uint64_t level) const;

    /**
     * Gets the sum from the leaf node specified. This function
     * is called by calcStackDist.
     *
     * @param node pointer to the node which is updated
     * @return The stack distance of the current address.
     *
     */
    uint64_t getSumsLeavesToRoot(Node* node) const;

    /**
     * Updates the nodes upwards recursively till the root.
     * This function is first called by updateSumsLeavesToRoot,
     * and then it recursively calls itself.
     *
     * @param node pointer to the node which is updated
     * @param from_left variable which says that the request arrived
     * from the left
     * @param sum_from_below Sum of left and right children below
     * @param level level in the tree the calling node is located
     * @param stack_dist stack distance of the node below
     * @param discard_node whether the calling node was discarded or not
     * @return The stack distance of the current address.
     *
     */
    uint64_t updateSum(Node* node,
                       bool from_left, uint64_t sum_from_below, uint64_t level,
                       uint64_t stack_dist, bool discard_node);

    /**
     * Updates the leaf nodes and nodes above. This function is
     * called by the calcStackDistAndUpdate.
     *
     * @param node pointer to the node which is updated
     * @param is_new_leaf is true if this is a newly added node
     * @return The stack distance of the current address.
     *
     */
    uint64_t updateSumsLeavesToRoot(Node* node, bool is_new_leaf);

    /**
     * updateTree is a tree balancing operation, which maintains the
     * binary tree structure.
     * This method is called whenever index%2 == 0 (i.e. every
     * alternate cycle) The two main operation are :
     * OP1. Moving the root node one layer up if index counter
     *       crosses power of 2
     * OP2. Addition of intermediate nodes as and when required
     *      and linking them to their parents in the layer above.
     */
    void updateTree();

    /**
     * This method is used for verification purposes
     * It recursively traverses upwards from the given node till
     * the root to check if the ultimate parent node (root-node) points
     * to null.
     *
     * @param node pointer to the node whose sanity is being checked
     * @param level the level at which this node is located in the tree
     *
     */
    void sanityCheckTree(const Node* node, uint64_t level = 0) const;

    /**
     * Return the counter for address accesses (unique and
     * non-unique). This is further used to dump stats at
     * regular intervals.
     *
     * @return The stack distance of the current address.
     */
    uint64_t getIndex() const { return index; }

    /**
     * Query depth of the tree (tree[0] represents leaf layer while
     * tree[treeDepth] represents the root layer, all layers in
     * between contain intermediate nodes)
     *
     * @return Tree depth
     */
    uint64_t getTreeDepth() const { return tree.size() - 1; }

    /**
     * Print the last n items on the stack.
     * This method prints top n entries in the tree based implementation as
     * well as dummy stack.
     * @param n Number of entries to print
     */
    void printStack(int n = 5) const;

    /**
     * This is an alternative implementation of the stack-distance
     * in a naive way. It uses simple STL vector to represent the stack.
     * It can be used in parallel for debugging purposes.
     * It is 10x slower than the tree based implemenation.
     *
     * @param r_address The current address to process
     * @param update_stack Flag to indicate if stack should be updated
     * @return  Stack distance which is calculated by this alternative
     * implementation
     *
     */
    uint64_t verifyStackDist(const Addr r_address,
                             bool update_stack = false);

  public:
    StackDistCalc(bool verify_stack = false);

    ~StackDistCalc();

    /**
     * A convenient way of refering to infinity.
     */
    static constexpr uint64_t Infinity = std::numeric_limits<uint64_t>::max();


    /**
     * Process the given address. If Mark is true then set the
     * mark flag of the leaf node.
     * This function returns the stack distance of the incoming
     * address and the previous status of the mark flag.
     *
     * @param r_address The current address to process
     * @param mark set the mark flag for the address.
     * @return The stack distance of the current address and the mark flag.
     */
    std::pair<uint64_t, bool> calcStackDist(const Addr r_address,
                                            bool mark = false);

    /**
     * Process the given address:
     *  - Lookup the tree for the given address
     *  - delete old node if found in tree
     *  - add a new node (if addNewNode flag is set)
     * This function returns the stack distance of the incoming
     * address and the status of the mark flag.
     *
     * @param r_address The current address to process
     * @param addNewNode If true, a new node is added to the tree
     * @return The stack distance of the current address and the mark flag.
     */
    std::pair<uint64_t, bool> calcStackDistAndUpdate(const Addr r_address,
                                                     bool addNewNode = true);

  private:

    /**
     * Node which takes form of Leaf, INode or Root
     */
    struct Node{
        // Sum of the left children
        uint64_t sumLeft;

        // Sum of the right children
        uint64_t sumRight;

        // Flag to indicate that sumLeft has gone from non-zero value to 0
        bool discardLeft;

        // Flag to indicate that sumRight has gone from non-zero value to 0
        bool discardRight;

        // Index of the current element in the Map
        uint64_t nodeIndex;

        // Pointer to the parent
        Node* parent;

        // Flag to mark the node as the right/left child
        bool isLeftNode;

        /**
         * Flag to indicate if this address is marked. Used in case
         * where stack distance of a touched address is required.
         */
        bool isMarked;

        /**
         * The discard flags are false by default they become true if
         * the node is reached again in a future lookup.
         */
        Node() : sumLeft(0), sumRight(0), discardLeft(false),
                 discardRight(false), nodeIndex(0),
                 parent(nullptr), isLeftNode(true), isMarked(false)
        { }
    };

    /**
     * Internal counter for address accesses (unique and non-unique)
     * This counter increments everytime the calcStackDist() method is
     * called. This counter is used as a key for the hash- map at the
     * leaf layer. Practically at every call to the calculator this
     * counter is incremented and a new leaf node is added in the tree
     * at the leaf layer using this counter value as the key.
     */
    uint64_t index;

    // Binary tree of partial sums
    TreeType tree;

    // Hash map which returns last seen index of each address
    AddressIndexMap aiMap;

    // Keeps count of number of the next unique index for each
    // level in the tree
    std::vector<uint64_t> nextIndex;

    // Dummy Stack for verification
    std::vector<uint64_t> stack;

    // Flag to enable verification of stack. (Slows down the simulation)
    const bool verifyStack;
};


#endif //__STACK_DIST_CALC_HH__
