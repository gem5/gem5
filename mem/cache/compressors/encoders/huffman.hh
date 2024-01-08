/*
 * Copyright (c) 2019, 2020 Inria
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

#ifndef __MEM_CACHE_COMPRESSORS_ENCODERS_HUFFMAN_HH__
#define __MEM_CACHE_COMPRESSORS_ENCODERS_HUFFMAN_HH__

#include <cassert>
#include <cstdint>
#include <map>
#include <memory>
#include <queue>
#include <vector>

#include "base/compiler.hh"
#include "mem/cache/compressors/encoders/base.hh"

namespace gem5
{

namespace compression
{
namespace encoder
{

/**
 * This encoder builds a Huffman tree using the frequency of each value to
 * be encoded.
 */
class Huffman : public Base
{
  public:
    Huffman(uint64_t max_code_length);
    ~Huffman() = default;

    /**
     * Inserts the value-frequency pair in the tree.
     *
     * @param value The value.
     * @param frequency The value's frequency.
     */
    void sample(uint64_t value, uint64_t frequency);

    /** Generation of the code maps. This automatically builds the tree. */
    void generateCodeMaps();

    Code encode(const uint64_t val) const override;
    uint64_t decode(const uint64_t code) const override;

  private:
    /** Node for the Huffman tree. */
    class Node
    {
      private:
        /** Frequency of the value represented by this node. */
        const uint64_t _frequency;

        /** Value represented by this node, if this is a leaf node. */
        const uint64_t _value;

        /** The left tree. */
        std::unique_ptr<Node> _left;

        /** The right tree. */
        std::unique_ptr<Node> _right;

      public:
        /** Initialize node as a leaf node. */
        Node(uint64_t value, uint64_t frequency)
          : _frequency(frequency), _value(value), _left(), _right()
        {
        }

        /** Initialize node as an internal node. */
        Node(Node* left, Node* right)
          : _frequency(left->getFrequency() + right->getFrequency()),
            _value(0), _left(left), _right(right)
        {
        }

        /** Getter for the frequency counter. */
        uint64_t getFrequency() const { return _frequency; }

        /**
         * Determine if the node is a leaf node by checking if it does not
         * have sub-trees.
         *
         * @return Wether the node is a leaf node.
         */
        bool
        isLeaf() const
        {
            return (_left == nullptr) && (_right == nullptr);
        }

        /**
         * Get the leaf's value.
         *
         * @return The leaf's value.
         */
        uint64_t
        getValue() const
        {
            assert(isLeaf());
            return _value;
        }

        const Node* getLeftSubTree() const { return _left.get(); }
        const Node* getRightSubTree() const { return _right.get(); }
    };

    /**
     * Maximum number of bits in a codeword. If a codeword requires more
     * than this amount of bits, its respective value is discarded.
     */
    const unsigned maxCodeLength;

    /**
     * Table containing the codewords and their respective lengths. Some
     * entries are discarded due to their lengths being too big.
     */
    std::map<uint64_t, Code> valueToCode;
    std::map<uint64_t, uint64_t> codeToValue;

    /**
     * Entries are not inserted directly into the tree. First they are sorted
     * based on their frequencies.
     */
    struct NodeComparator
    {
        bool
        operator()(const Node* lhs, const Node* rhs) const
        {
            return lhs->getFrequency() > rhs->getFrequency();
        }
    };
    std::priority_queue<Node*, std::vector<Node*>, NodeComparator> trees;

    /**
     * Build a Huffman tree using the values and their respective
     * frequencies, which have been informed through the insertion
     * function.
     *
     * @return A pointer to the root of the tree.
     */
    std::unique_ptr<Node> buildTree();

    /**
     * Recursive function that generates the huffman codes based on
     * the tree provided. The generated codes are added to the code
     * map structure.
     *
     * @param node The node being analyzed.
     * @param current_code The code so far.
     */
    void generateCodes(const Node* node, const Code& current_code);
};

} // namespace encoder
} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_ENCODERS_HUFFMAN_HH__
