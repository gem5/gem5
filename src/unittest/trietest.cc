/*
 * Copyright (c) 2012 Google
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
 *
 * Authors: Gabe Black
 */

#include <cassert>
#include <iostream>

#include "base/cprintf.hh"
#include "base/trie.hh"
#include "base/types.hh"
#include "unittest/unittest.hh"

using UnitTest::setCase;

typedef Trie<Addr, uint32_t> TestTrie;

int
main()
{
    // Create an empty Ptr and verify it's data pointer is NULL.
    setCase("An empty trie.");
    TestTrie trie1;
    trie1.dump("Empty");
    cprintf("\n\n");

    setCase("A single entry.");
    trie1.insert(0x0123456789abcdef, 40, (uint32_t *)(uintptr_t)(1));
    trie1.dump("One entry");
    cprintf("\n\n");

    setCase("Two entries, one on the way to the other.");
    TestTrie trie2;
    trie2.insert(0x0123456789abcdef, 40, (uint32_t *)(uintptr_t)(1));
    trie2.insert(0x0123456789abcdef, 36, (uint32_t *)(uintptr_t)(2));
    trie2.dump("Two entries inline v1");
    cprintf("\n\n");

    TestTrie trie3;
    trie3.insert(0x0123456789abcdef, 36, (uint32_t *)(uintptr_t)(2));
    trie3.insert(0x0123456789abcdef, 40, (uint32_t *)(uintptr_t)(1));
    trie3.dump("Two entries inline v2");
    cprintf("\n\n");

    setCase("Two entries on different paths.");
    TestTrie trie4;
    trie4.insert(0x0123456789abcdef, 40, (uint32_t *)(uintptr_t)(2));
    trie4.insert(0x0123456776543210, 40, (uint32_t *)(uintptr_t)(1));
    trie4.dump("Two split entries");
    cprintf("\n\n");

    setCase("Skipping past an entry but not two.");
    TestTrie trie5;
    trie5.insert(0x0123456789000000, 40, (uint32_t *)(uintptr_t)(4));
    trie5.insert(0x0123000000000000, 40, (uint32_t *)(uintptr_t)(1));
    trie5.insert(0x0123456780000000, 40, (uint32_t *)(uintptr_t)(3));
    trie5.insert(0x0123456700000000, 40, (uint32_t *)(uintptr_t)(2));
    trie5.dump("Complex insertion");
    cprintf("\n\n");

    setCase("Looking things up.");
    EXPECT_EQ((uintptr_t)trie5.lookup(0x0123000000000000), 1);
    EXPECT_EQ((uintptr_t)trie5.lookup(0x0123456700000000), 2);
    EXPECT_EQ((uintptr_t)trie5.lookup(0x0123456780000000), 3);
    EXPECT_EQ((uintptr_t)trie5.lookup(0x0123456789000000), 4);

    setCase("Removing entries.");
    TestTrie trie6;
    TestTrie::Handle node1, node2;
    trie6.insert(0x0123456789000000, 40, (uint32_t *)(uintptr_t)(4));
    trie6.insert(0x0123000000000000, 40, (uint32_t *)(uintptr_t)(1));
    trie6.insert(0x0123456780000000, 40, (uint32_t *)(uintptr_t)(3));
    node1 = trie6.insert(0x0123456700000000, 40, (uint32_t *)(uintptr_t)(2));
    node2 = trie6.insert(0x0123456700000000, 32, (uint32_t *)(uintptr_t)(10));
    trie6.dump("Fill before removal");
    cprintf("\n\n");

    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123000000000000), 1);
    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123456700000000), 10);
    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123456780000000), 10);
    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123456789000000), 10);

    trie6.remove(node2);
    trie6.dump("One node removed");
    cprintf("\n\n");

    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123000000000000), 1);
    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123456700000000), 2);
    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123456780000000), 3);
    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123456789000000), 4);

    trie6.remove(node1);
    trie6.dump("Two nodes removed");
    cprintf("\n\n");

    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123000000000000), 1);
    EXPECT_EQ(trie6.lookup(0x0123456700000000), NULL);
    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123456780000000), 3);
    EXPECT_EQ((uintptr_t)trie6.lookup(0x0123456789000000), 4);

    return UnitTest::printResults();
}
