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
 */

#ifndef __BASE_TRIE_HH__
#define __BASE_TRIE_HH__

#include <cassert>
#include <iostream>
#include <type_traits>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/types.hh"

namespace gem5
{

/**
 * A trie is a tree-based data structure used for data retrieval. It uses
 * bits masked from the msb of the key to to determine a value's location,
 * so its lookups have their worst case time dictated by the key's size.
 *
 * @tparam Key Type of the key of the tree nodes. Must be an integral type.
 * @tparam Value Type of the values associated to the keys.
 *
 * @ingroup api_base_utils
 */
template <class Key, class Value>
class Trie
{
  protected:
    static_assert(std::is_integral_v<Key>, "Key has to be an integral type");

    struct Node
    {
        Key key;
        Key mask;

        bool
        matches(Key test)
        {
            return (test & mask) == key;
        }

        Value *value;

        Node *parent;
        std::unique_ptr<Node> kids[2];

        Node(Key _key, Key _mask, Value *_val)
            : key(_key & _mask), mask(_mask), value(_val), parent(NULL)
        {
            kids[0] = NULL;
            kids[1] = NULL;
        }

        void
        clear()
        {
            kids[1].reset();
            kids[0].reset();
        }

        void
        dump(std::ostream &os, int level)
        {
            for (int i = 1; i < level; i++) {
                ccprintf(os, "|");
            }
            if (level == 0)
                ccprintf(os, "Root ");
            else
                ccprintf(os, "+ ");
            ccprintf(os, "(%p, %p, %#X, %#X, %p)\n", parent, this, key, mask,
                     value);
            if (kids[0])
                kids[0]->dump(os, level + 1);
            if (kids[1])
                kids[1]->dump(os, level + 1);
        }
    };

  protected:
    Node head;

  public:
    /**
     * @ingroup api_base_utils
     */
    typedef Node *Handle;

    /**
     * @ingroup api_base_utils
     */
    Trie() : head(0, 0, NULL) {}

    /**
     * @ingroup api_base_utils
     */
    static const unsigned MaxBits = sizeof(Key) * 8;

  private:
    /**
     * A utility method which checks whether the key being looked up lies
     * beyond the Node being examined. If so, it returns true and advances the
     * node being examined.
     * @param parent The node we're currently "at", which can be updated.
     * @param kid The node we may want to move to.
     * @param key The key we're looking for.
     * @param new_mask The mask to use when matching against the key.
     * @return Whether the current Node was advanced.
     */
    bool
    goesAfter(Node **parent, Node *kid, Key key, Key new_mask)
    {
        if (kid && kid->matches(key) && (kid->mask & new_mask) == kid->mask) {
            *parent = kid;
            return true;
        } else {
            return false;
        }
    }

    /**
     * A utility method which extends a mask value one more bit towards the
     * lsb. This is almost just a signed right shift, except that the shifted
     * in bits are technically undefined. This is also slightly complicated by
     * the zero case.
     * @param orig The original mask to extend.
     * @return The extended mask.
     */
    Key
    extendMask(Key orig)
    {
        // Just in case orig was 0.
        const Key msb = 1ULL << (MaxBits - 1);
        return orig | (orig >> 1) | msb;
    }

    /**
     * Method which looks up the Handle corresponding to a particular key. This
     * is useful if you want to delete the Handle corresponding to a key since
     * the "remove" function takes a Handle as its argument.
     * @param key The key to look up.
     * @return The first Handle matching this key, or NULL if none was found.
     */
    Handle
    lookupHandle(Key key)
    {
        Node *node = &head;
        while (node) {
            if (node->value)
                return node;

            if (node->kids[0] && node->kids[0]->matches(key))
                node = node->kids[0].get();
            else if (node->kids[1] && node->kids[1]->matches(key))
                node = node->kids[1].get();
            else
                node = NULL;
        }

        return NULL;
    }

  public:
    /**
     * Method which inserts a key/value pair into the trie.
     * @param key The key which can later be used to look up this value.
     * @param width How many bits of the key (from msb) should be used.
     * @param val A pointer to the value to store in the trie.
     * @return A Handle corresponding to this value.
     *
     * @ingroup api_base_utils
     */
    Handle
    insert(Key key, unsigned width, Value *val)
    {
        // We use NULL value pointers to mark internal nodes of the trie, so
        // we don't allow inserting them as real values.
        assert(val);

        // Build a mask which masks off all the bits we don't care about.
        Key new_mask = ~(Key)0;
        if (width < MaxBits)
            new_mask <<= (MaxBits - width);
        // Use it to tidy up the key.
        key &= new_mask;

        // Walk past all the nodes this new node will be inserted after. They
        // can be ignored for the purposes of this function.
        Node *node = &head;
        while (goesAfter(&node, node->kids[0].get(), key, new_mask) ||
               goesAfter(&node, node->kids[1].get(), key, new_mask)) {}
        assert(node);

        Key cur_mask = node->mask;
        // If we're already where the value needs to be...
        if (cur_mask == new_mask) {
            assert(!node->value);
            node->value = val;
            return node;
        }

        for (unsigned int i = 0; i < 2; i++) {
            auto &kid = node->kids[i];
            if (!kid) {
                // No kid. Add a new one.
                auto new_node = std::make_unique<Node>(key, new_mask, val);
                new_node->parent = node;
                kid = std::move(new_node);
                return kid.get();
            }

            // Walk down the leg until something doesn't match or we run out
            // of bits.
            Key last_mask;
            bool done;
            do {
                last_mask = cur_mask;
                cur_mask = extendMask(cur_mask);
                done = ((key & cur_mask) != (kid->key & cur_mask)) ||
                       last_mask == new_mask;
            } while (!done);
            cur_mask = last_mask;

            // If this isn't the right leg to go down at all, skip it.
            if (cur_mask == node->mask)
                continue;

            // At the point we walked to above, add a new node.
            auto new_node = std::make_unique<Node>(key, cur_mask, nullptr);
            new_node->parent = node;
            kid->parent = new_node.get();
            new_node->kids[0] = std::move(kid);
            kid = std::move(new_node);

            // If we ran out of bits, the value goes right here.
            if (cur_mask == new_mask) {
                kid->value = val;
                return kid.get();
            }

            // Still more bits to deal with, so add a new node for that path.
            new_node = std::make_unique<Node>(key, new_mask, val);
            new_node->parent = kid.get();
            kid->kids[1] = std::move(new_node);
            return kid->kids[1].get();
        }

        panic("Reached the end of the Trie insert function!\n");
        return NULL;
    }

    /**
     * Method which looks up the Value corresponding to a particular key.
     * @param key The key to look up.
     * @return The first Value matching this key, or NULL if none was found.
     *
     * @ingroup api_base_utils
     */
    Value *
    lookup(Key key)
    {
        Node *node = lookupHandle(key);
        if (node)
            return node->value;
        else
            return NULL;
    }

    /**
     * Method to delete a value from the trie.
     * @param node A Handle to remove.
     * @return The Value pointer from the removed entry.
     *
     * @ingroup api_base_utils
     */
    Value *
    remove(Handle handle)
    {
        Node *node = handle;
        Value *val = node->value;
        if (node->kids[1]) {
            assert(node->value);
            node->value = NULL;
            return val;
        }
        if (!node->parent)
            panic("Trie: Can't remove root node.\n");

        Node *parent = node->parent;

        // If there's a kid, fix up it's parent pointer.
        if (node->kids[0])
            node->kids[0]->parent = parent;
        // Figure out which kid we are, and update our parent's pointers.
        if (parent->kids[0].get() == node)
            parent->kids[0] = std::move(node->kids[0]);
        else if (parent->kids[1].get() == node)
            parent->kids[1] = std::move(node->kids[0]);
        else
            panic("Trie: Inconsistent parent/kid relationship.\n");
        // Make sure if the parent only has one kid, it's kid[0].
        if (parent->kids[1] && !parent->kids[0]) {
            parent->kids[0] = std::move(parent->kids[1]);
            parent->kids[1] = nullptr;
        }

        // If the parent has less than two kids and no cargo and isn't the
        // root, delete it too.
        if (!parent->kids[1] && !parent->value && parent->parent)
            remove(parent);
        return val;
    }

    /**
     * Method to lookup a value from the trie and then delete it.
     * @param key The key to look up and then remove.
     * @return The Value pointer from the removed entry, if any.
     *
     * @ingroup api_base_utils
     */
    Value *
    remove(Key key)
    {
        Handle handle = lookupHandle(key);
        if (!handle)
            return NULL;
        return remove(handle);
    }

    /**
     * A method which removes all key/value pairs from the trie. This is more
     * efficient than trying to remove elements individually.
     *
     * @ingroup api_base_utils
     */
    void
    clear()
    {
        head.clear();
    }

    /**
     * A debugging method which prints the contents of this trie.
     * @param title An identifying title to put in the dump header.
     */
    void
    dump(const char *title, std::ostream &os = std::cout)
    {
        ccprintf(os, "**************************************************\n");
        ccprintf(os, "*** Start of Trie: %s\n", title);
        ccprintf(os, "*** (parent, me, key, mask, value pointer)\n");
        ccprintf(os, "**************************************************\n");
        head.dump(os, 0);
    }
};

} // namespace gem5

#endif
