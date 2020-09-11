/*
 * Copyright 2018 Google, Inc.
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

#ifndef __SYSTEMC_CORE_LIST_HH__
#define __SYSTEMC_CORE_LIST_HH__

#include <functional>

#include "base/fiber.hh"
#include "systemc/core/object.hh"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/ext/core/sc_process_handle.hh"

namespace sc_gem5
{

struct ListNode
{
    ListNode() : nextListNode(nullptr), prevListNode(nullptr) {}
    virtual ~ListNode() {}

    ListNode *nextListNode;
    ListNode *prevListNode;

    void
    popListNode()
    {
        if (nextListNode)
            nextListNode->prevListNode = prevListNode;
        if (prevListNode)
            prevListNode->nextListNode = nextListNode;
        nextListNode = nullptr;
        prevListNode = nullptr;
    }
};

template <typename T>
struct NodeList : public ListNode
{
    NodeList()
    {
        nextListNode = this;
        prevListNode = this;
    }

    void
    pushFirst(T *t)
    {
        // Make sure this node isn't currently in a different list.
        t->popListNode();

        // The node behind t is whoever used to be first.
        t->nextListNode = nextListNode;
        // The node that used to be first is behind t.
        nextListNode->prevListNode = t;

        // Nobody is in front of t.
        t->prevListNode = this;
        // The first node is t.
        nextListNode = t;
    }

    void
    pushLast(T *t)
    {
        // Make sure this node isn't currently in a different list.
        t->popListNode();

        // The node in front of t is whoever used to be last.
        t->prevListNode = prevListNode;
        // The node that used to be last is in front of t.
        prevListNode->nextListNode = t;

        // Nobody is behind t.
        t->nextListNode = this;
        // The last node is t.
        prevListNode = t;
    }

    T *
    getNext()
    {
        return empty() ? nullptr : static_cast<T *>(nextListNode);
    }

    bool empty() { return nextListNode == this; }
};

} // namespace sc_gem5

#endif  //__SYSTEMC_CORE_LIST_HH__
