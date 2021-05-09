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
 */

#include "mem/stack_dist_calc.hh"

#include "base/chunk_generator.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/StackDist.hh"

namespace gem5
{

StackDistCalc::StackDistCalc(bool verify_stack)
    : index(0),
      verifyStack(verify_stack)
{
    // Instantiate a new root and leaf layer
    // Map type variable, representing a layer in the tree
    IndexNodeMap tree_level;

    // Initialize tree count for leaves
    nextIndex.push_back(0);

    // Add the initial leaf layer to the tree
    tree.push_back(tree_level);

    // Create a root node. Node type variable in the topmost layer
    Node* root_node = new Node();

    // Initialize tree count for root
    nextIndex.push_back(1);

    // Add the empty root layer to the tree
    tree.push_back(tree_level);

    // Add the initial root to the tree
    tree[1][root_node->nodeIndex] = root_node;
}

StackDistCalc::~StackDistCalc()
{
    // Walk through each layer and destroy the nodes
    for (auto& layer : tree) {
        for (auto& index_node : layer) {
            // each map entry contains an index and a node
            delete index_node.second;
        }
        // Clear each layer in the tree
        layer.clear();
    }

    // Clear the tree
    tree.clear();
    aiMap.clear();
    nextIndex.clear();

    // For verification
    stack.clear();
}

// The updateSum method is a recursive function which updates
// the node sums till the root. It also deletes the nodes that
// are not used anymore.
uint64_t
StackDistCalc::updateSum(Node* node, bool from_left,
                         uint64_t sum_from_below, uint64_t level,
                         uint64_t stack_dist, bool discard_node)
{
    ++level;

    // Make a copy of the node variables and work on them
    // as a node may be deleted by this function
    uint64_t node_sum_l = node->sumLeft;
    uint64_t node_sum_r = node->sumRight;
    bool node_left = node->isLeftNode;
    bool node_discard_left = node->discardLeft;
    bool node_discard_right = node->discardRight;
    uint64_t node_n_index = node->nodeIndex;
    Node* node_parent_ptr = node->parent;

    // For verification
    if (verifyStack) {
        // This sanity check makes sure that the left_sum and
        // right_sum of the node is not greater than the
        // maximum possible value given by the leaves below it
        // for example a node in layer 3 (tree[3]) can at most
        // have 8 leaves (4 to the left and 4 to the right)
        // thus left_sum and right_sum should be <= 4
        panic_if(node_sum_l > (1 << (level - 1)),
                 "Error in sum left of level %ul, node index %ull, "
                 "Sum =  %ull \n", level, node_n_index, node_sum_l);

        panic_if(node_sum_r > (1 << (level - 1)),
                 "Error in sum right of level %ul, node index %ull, "
                 "Sum =  %ull \n", level, node_n_index, node_sum_r);
    }

    // Update the left sum or the right sum depending on the
    // from_left flag. Variable stack_dist is updated only
    // when arriving from Left.
    if (from_left) {
        // update sumLeft
        node_sum_l = sum_from_below;
        stack_dist +=  node_sum_r;
    } else {
        // update sum_r
        node_sum_r = sum_from_below;
    }

    // sum_from_below == 0 can be a leaf discard operation
    if (discard_node && !sum_from_below) {
        if (from_left)
            node_discard_left = true;
        else
            node_discard_right = true;
    }

    // Update the node variables with new values
    node->nodeIndex = node_n_index;
    node->sumLeft = node_sum_l;
    node->sumRight = node_sum_r;
    node->isLeftNode = node_left;
    node->discardLeft = node_discard_left;
    node->discardRight = node_discard_right;

    // Delete the node if it is not required anymore
    if (node_discard_left && node_discard_right &&
        discard_node && node_parent_ptr && !sum_from_below) {
        delete node;
        tree[level].erase(node_n_index);
        discard_node = true;
    } else {
        // propogate discard_node as false upwards if the
        // above conditions are not met.
        discard_node = false;
    }

    // Recursively call the updateSum operation till the
    // root node is reached
    if (node_parent_ptr) {
        stack_dist = updateSum(node_parent_ptr, node_left,
                               node_sum_l + node_sum_r,
                               level, stack_dist, discard_node);
    }

    return stack_dist;
}

// This function is called by the calcStackDistAndUpdate function
// If is_new_leaf is true then a new leaf is added otherwise a leaf
// removed from the tree. In both cases the tree is updated using
// the updateSum operation.
uint64_t
StackDistCalc::updateSumsLeavesToRoot(Node* node, bool is_new_leaf)
{
    uint64_t level = 0;
    uint64_t stack_dist = 0;

    if (is_new_leaf) {
        node->sumLeft = 1;
        updateSum(node->parent,
                  node->isLeftNode, node->sumLeft,
                  level, 0, false);

        stack_dist = Infinity;
    } else {
        node->sumLeft = 0;
        stack_dist = updateSum(node->parent,
                                  node->isLeftNode, 0,
                                  level, stack_dist, true);
    }

    return stack_dist;
}

// This method is a recursive function which calculates
// the node sums till the root.
uint64_t
StackDistCalc::getSum(Node* node, bool from_left, uint64_t sum_from_below,
                      uint64_t stack_dist, uint64_t level) const
{
    ++level;
    // Variable stack_dist is updated only
    // when arriving from Left.
    if (from_left) {
        stack_dist += node->sumRight;
    }

    // Recursively call the getSum operation till the
    // root node is reached
    if (node->parent) {
        stack_dist = getSum(node->parent, node->isLeftNode,
                            node->sumLeft + node->sumRight,
                            stack_dist, level);
    }

    return stack_dist;
}

// This function is called by the calcStackDistance function
uint64_t
StackDistCalc::getSumsLeavesToRoot(Node* node) const
{
    return  getSum(node->parent, node->isLeftNode, 0, 0, 0);
}

// Update tree is a tree balancing operation which maintains
// the binary tree structure. This method is called whenever
// index%2 == 0 (i.e. every alternate cycle)
// The two main operation are :
// OP1. moving the root node one layer up if index counter
//    crosses power of 2
// OP2. Addition of intermediate nodes as and when required
//      and linking them to their parents in the layer above.
void
StackDistCalc::updateTree()
{
    uint64_t i;

    if (isPowerOf2(index)) {
        // OP1. moving the root node one layer up if index counter
        //    crosses power of 2
        // If index counter crosses a power of 2, then add a
        // new tree layer above and create a new Root node in it.
        // After the root is created the old node
        // in the layer below is updated to point to this
        // newly created root node. The sum_l of this new root node
        // becomes the sum_l + sum_r of the old node.
        //
        // After this root update operation a chain of intermediate
        // nodes is created from root layer to tree[1](one layer
        // above the leaf layer)

        // Create a new root node
        Node* newRootNode = new Node();

        // Update its sum_l as the sum_l+sum_r from below
        newRootNode->sumLeft = tree[getTreeDepth()][0]->sumRight +
            tree[getTreeDepth()][0]->sumLeft;
        // Update its discard left flag if the node below has
        // has both discardLeft and discardRight set.
        newRootNode->discardLeft = tree[getTreeDepth()][0]->discardLeft &&
            tree[getTreeDepth()][0]->discardRight;

        // Map type variable, representing a layer in the tree
        IndexNodeMap treeLevel;
        // Add a new layer to the tree
        tree.push_back(treeLevel);
        nextIndex.push_back(1);
        tree[getTreeDepth()][newRootNode->nodeIndex] = newRootNode;

        // Update the parent pointer at lower layer to
        // point to newly created root node
        tree[getTreeDepth() - 1][0]->parent = tree[getTreeDepth()][0];

        // Add intermediate nodes from root till bottom (one layer above the
        // leaf layer)
        for (i = getTreeDepth() - 1; i >= 1; --i) {
            Node* newINode = new Node();
            // newNode is left or right child depending on the number of nodes
            // in that layer
            if (nextIndex[i] % 2 == 0) {
                newINode->isLeftNode = true;
            } else {
                newINode->isLeftNode = false;
            }

            newINode->parent = tree[i + 1][nextIndex[i + 1] - 1];
            newINode->nodeIndex = ++nextIndex[i] - 1;
            tree[i][newINode->nodeIndex] = newINode;
        }
    } else {
        // OP2. Addition of intermediate nodes as and when required
        //      and linking them to their parents in the layer above.
        //
        // At layer 1 a new INode is added whenever index%(2^1)==0
        // (multiples of 2)
        //
        // At layer 2 a new INode is added whenever index%(2^2)==0
        // (multiples of 4)
        //
        // At layer 3 a new INode is added whenever index%(2^3)==0
        // (multiples of 8)
        //...
        //
        // At layer N a new INode is added whenever index%(2^N)==0
        // (multiples of 2^N)
        for (i = getTreeDepth() - 1; i >= 1; --i) {
            // Traverse each layer from root to leaves and add a new
            // intermediate node if required. Link the parent_ptr of
            // the new node to the parent in the above layer.

            if ((index % (1 << i)) == 0) {
                // Checks if current (index % 2^treeDepth) == 0 if true
                // a new node at that layer is created
                Node* newINode = new Node();

                // newNode is left or right child depending on the
                // number of nodes in that layer.
                if (nextIndex[i] % 2 == 0) {
                    newINode->isLeftNode = true;
                } else {
                    newINode->isLeftNode = false;
                }

                // Pointing to its parent in the upper layer
                newINode->parent = tree[i + 1][nextIndex[i + 1] - 1];
                newINode->nodeIndex = ++nextIndex[i] - 1;
                tree[i][newINode->nodeIndex] = newINode;
            }
        }
    }
}

// This function is called everytime to get the stack distance and add
// a new node. A feature to mark an old node in the tree is
// added. This is useful if it is required to see the reuse
// pattern. For example, BackInvalidates from the lower level (Membus)
// to L2, can be marked (isMarked flag of Node set to True). And then
// later if this same address is accessed by L1, the value of the
// isMarked flag would be True. This would give some insight on how
// the BackInvalidates policy of the lower level affect the read/write
// accesses in an application.
std::pair< uint64_t, bool>
StackDistCalc::calcStackDistAndUpdate(const Addr r_address, bool addNewNode)
{
    Node* newLeafNode;

    auto ai = aiMap.lower_bound(r_address);

    // Default value of flag indicating as the left or right leaf
    bool isLeft     = true;
    // Default value of isMarked flag for each node.
    bool _mark = false;
    // By default stackDistacne is treated as infinity
    uint64_t stack_dist;

    // Lookup aiMap by giving address as the key:
    // If found take address and Lookup in tree
    // Update tree from leaves by making B(found index) = 0
    // Add sums to right till root while Updating them
    // Stack Distance of that address sums to right
    if (ai != aiMap.end() && !(aiMap.key_comp()(r_address, ai->first))) {
        // key already exists
        // save the index counter value when this address was
        // encountered before and update it to the current index value
        uint64_t r_index = ai->second;

        if (addNewNode) {
            // Update aiMap aiMap(Address) = current index
            ai->second   = index;
        } else {
            aiMap.erase(r_address);
        }

        // Call update tree operation on the tree starting with
        // the r_index value found above. This function would return
        // the value of the stack distcance.
        stack_dist = updateSumsLeavesToRoot(tree[0][r_index], false);
        newLeafNode = tree[0][r_index];
        // determine if this node was marked earlier
        _mark = newLeafNode->isMarked;
        delete newLeafNode;
        tree[0].erase(r_index);
    } else {
        if (addNewNode) {
            // Update aiMap aiMap(Address) = current index
            aiMap[r_address] = index;
        }
        // Update infinity bin count
        // By default stackDistacne is treated as infinity
        stack_dist = Infinity;
    }

    if (addNewNode) {
        // If index%2 == 0 then update tree
        if (index % 2 == 0) {
            updateTree();
        } else {
            // At odd values of index counter, a new right-type node is
            // added to the leaf layer, else a left-type node is added
            isLeft = false;
        }

        // Add new leaf node in the leaf layer (tree[0])
        // set n_index = current index
        newLeafNode = new Node();
        ++nextIndex[0];
        newLeafNode->nodeIndex=index;
        newLeafNode->isLeftNode=isLeft;
        // Point the parent pointer to the intermediate node above
        newLeafNode->parent = tree[1][nextIndex[1] - 1];
        tree[0][index] = newLeafNode;
        // call an update operation which would update the tree after
        // addition of this new leaf node.
        updateSumsLeavesToRoot(tree[0][index], true);

        // For verification
        if (verifyStack) {
            // This function checks the sanity of the tree to make sure the
            // last node in the link of parent pointers is the root node.
            // It takes a leaf node as an argument and traveses upwards till
            // the root layer to check if the last parent is null
            sanityCheckTree(tree[0][index]);

            // Push the same element in debug stack, and check
            uint64_t verify_stack_dist = verifyStackDist(r_address, true);
            panic_if(verify_stack_dist != stack_dist,
                     "Expected stack-distance for address \
                             %#lx is %#lx but found %#lx",
                     r_address, verify_stack_dist, stack_dist);
            printStack();
        }

        // The index counter is updated at the end of each transaction
        // (unique or non-unique)
        ++index;
    }

    return (std::make_pair(stack_dist, _mark));
}

// This function is called everytime to get the stack distance
// no new node is added. It can be used to mark a previous access
// and inspect the value of the mark flag.
std::pair< uint64_t, bool>
StackDistCalc::calcStackDist(const Addr r_address, bool mark)
{
    // Default value of isMarked flag for each node.
    bool _mark = false;

    auto ai = aiMap.lower_bound(r_address);

    // By default stackDistacne is treated as infinity
    uint64_t stack_dist = 0;

    // Lookup aiMap by giving address as the key:
    // If found take address and Lookup in tree
    // Add sums to right till root
    // Stack Distance of that address sums to right
    if (ai != aiMap.end() && !(aiMap.key_comp()(r_address, ai->first))) {
        // key already exists
        // save the index counter value when this address was
        // encountered before
        uint64_t r_index = ai->second;

        // Get the value of mark flag if previously marked
        _mark = tree[0][r_index]->isMarked;
        // Mark the leaf node if required
        tree[0][r_index]->isMarked = mark;

        // Call get sums operation on the tree starting with
        // the r_index value found above. This function would return
        // the value of the stack distcance.
        stack_dist = getSumsLeavesToRoot(tree[0][r_index]);
    } else {
        // Update infinity bin count
        // By default stackDistacne is treated as infinity
        stack_dist = Infinity;
    }

    // For verification
    if (verifyStack) {
        // Calculate the SD of the same address in the debug stack
        uint64_t verify_stack_dist = verifyStackDist(r_address);
        panic_if(verify_stack_dist != stack_dist,
                 "Expected stack-distance for address \
                             %#lx is %#lx but found %#lx",
                 r_address, verify_stack_dist, stack_dist);

        printStack();
    }

    return std::make_pair(stack_dist, _mark);
}

// For verification
// Simple sanity check for the tree
void
StackDistCalc::sanityCheckTree(const Node* node, uint64_t level) const
{
    const Node* next_up = node->parent;

    for (uint64_t i = level + 1; i < getTreeDepth() - level; ++i) {
        next_up = next_up->parent;
        panic_if(!next_up, "Sanity check failed for node %ull \n",
                 node->nodeIndex);
    }

    // At the root layer the parent_ptr should be null
    panic_if(next_up->parent, "Sanity check failed for node %ull \n",
             node->nodeIndex);
}

// This method can be called to compute the stack distance in a naive
// way It can be used to verify the functionality of the stack
// distance calculator. It uses std::vector to compute the stack
// distance using a naive stack.
uint64_t
StackDistCalc::verifyStackDist(const Addr r_address, bool update_stack)
{
    bool found = false;
    uint64_t stack_dist = 0;
    auto a = stack.rbegin();

    for (; a != stack.rend(); ++a) {
        if (*a == r_address) {
            found = true;
            break;
        } else {
            ++stack_dist;
        }
    }

    if (found) {
        ++a;
        if (update_stack)
            stack.erase(a.base());
    } else {
        stack_dist = Infinity;
    }

    if (update_stack)
        stack.push_back(r_address);

    return stack_dist;
}

// This method is useful to print top n entities in the stack.
void
StackDistCalc::printStack(int n) const
{
    Node* node;
    int count = 0;

    DPRINTF(StackDist, "Printing last %d entries in tree\n", n);

    // Walk through leaf layer to display the last n nodes
    for (auto it = tree[0].rbegin(); (count < n) && (it != tree[0].rend());
         ++it, ++count) {
        node = it->second;
        uint64_t r_index = node->nodeIndex;

        // Lookup aiMap using the index returned by the leaf iterator
        for (auto ai = aiMap.rbegin(); ai != aiMap.rend(); ++ai) {
            if (ai->second == r_index) {
                DPRINTF(StackDist,"Tree leaves, Rightmost-[%d] = %#lx\n",
                        count, ai->first);
                break;
            }
        }
    }

    DPRINTF(StackDist,"Tree depth = %#ld\n", getTreeDepth());

    if (verifyStack) {
        DPRINTF(StackDist,"Printing Last %d entries in VerifStack \n", n);
        count = 0;
        for (auto a = stack.rbegin(); (count < n) && (a != stack.rend());
             ++a, ++count) {
            DPRINTF(StackDist, "Verif Stack, Top-[%d] = %#lx\n", count, *a);
        }
    }
}

} // namespace gem5
