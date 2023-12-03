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

#include "mem/cache/compressors/encoders/huffman.hh"

#include <cassert>

#include "base/logging.hh"

namespace gem5
{

namespace compression
{
namespace encoder
{

Huffman::Huffman(uint64_t max_code_length)
    : Base(), maxCodeLength(max_code_length)
{
    fatal_if(maxCodeLength > 64,
             "Code length cannot surpass its underlying container");
}

void
Huffman::sample(uint64_t value, uint64_t frequency)
{
    if (frequency != 0) {
        trees.push(new Node(value, frequency));
    }
}

std::unique_ptr<Huffman::Node>
Huffman::buildTree()
{
    // Construct tree by assigning left and right nodes. The left path leads
    // to the most frequent values
    while (trees.size() > 1) {
        Node *left = trees.top();
        trees.pop();

        Node *right = trees.top();
        trees.pop();

        Node *parent = new Node(left, right);
        trees.push(parent);
    }

    // All queue entries have been merged into a single entry containing
    // the tree
    Node *root = trees.top();
    trees.pop();
    return std::unique_ptr<Node>(root);
}

void
Huffman::generateCodeMaps()
{
    valueToCode.clear();
    codeToValue.clear();
    generateCodes(buildTree().get(), Code());
}

void
Huffman::generateCodes(const Node *node, const Code &current_code)
{
    // Drop all entries with length greater than maxCodeLength
    if (current_code.length > maxCodeLength) {
        return;
    }

    if (node->isLeaf()) {
        valueToCode[node->getValue()] = current_code;
        codeToValue[current_code.code] = node->getValue();
    } else {
        Code right_code = current_code;
        right_code.code = (right_code.code << 1) + 1;
        right_code.length++;
        generateCodes(node->getRightSubTree(), right_code);

        Code left_code = current_code;
        left_code.code = left_code.code << 1;
        left_code.length++;
        generateCodes(node->getLeftSubTree(), left_code);
    }
}

Code
Huffman::encode(const uint64_t val) const
{
    auto it = valueToCode.find(val);
    if (it == valueToCode.end()) {
        // If the value is unknown, generate a dummy code with invalid
        // length to let the caller know the encoding is invalid
        Code dummy_code;
        dummy_code.code = 0;
        dummy_code.length = 65;
        return dummy_code;
    } else {
        return it->second;
    }
}

uint64_t
Huffman::decode(const uint64_t code) const
{
    // A code that does not exist cannot be decoded
    auto it = codeToValue.find(code);
    assert(it != codeToValue.end());
    return it->second;
}

} // namespace encoder
} // namespace compression
} // namespace gem5
