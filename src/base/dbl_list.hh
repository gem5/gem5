/*
 * Copyright (c) 2000-2001, 2003-2005 The Regents of The University of Michigan
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

#ifndef __DBL_LIST_HH__
#define __DBL_LIST_HH__

class DblListEl {
    DblListEl *next;
    DblListEl *prev;

    // remove this from list
    void remove() {
        prev->next = next;
        next->prev = prev;
    }

    // insert this before old_el
    void insertBefore(DblListEl *old_el) {
        prev = old_el->prev;
        next = old_el;
        prev->next = this;
        next->prev = this;
    }

    // insert this after old_el
    void insertAfter(DblListEl *old_el) {
        next = old_el->next;
        prev = old_el;
        next->prev = this;
        prev->next = this;
    }

    friend class DblListBase;
};


//
// doubly-linked list of DblListEl objects
//
class DblListBase {
    // dummy list head element: dummy.next is head, dummy.prev is tail
    DblListEl dummy;

    // length counter
    unsigned length;

    DblListEl *valid_or_null(DblListEl *el) {
        // make sure users never see the dummy element
        return (el == &dummy) ? NULL : el;
    }

  public:

    DblListEl *head() {
        return valid_or_null(dummy.next);
    }

    DblListEl *tail() {
        return valid_or_null(dummy.prev);
    }

    DblListEl *next(DblListEl *el) {
        return valid_or_null(el->next);
    }

    DblListEl *prev(DblListEl *el) {
        return valid_or_null(el->prev);
    }

    bool is_empty() {
        return (dummy.next == &dummy);
    }

    void remove(DblListEl *el) {
        el->remove();
        --length;
    }

    void insertBefore(DblListEl *new_el, DblListEl *old_el) {
        new_el->insertBefore(old_el);
        ++length;
    }

    void insertAfter(DblListEl *new_el, DblListEl *old_el) {
        new_el->insertAfter(old_el);
        ++length;
    }

    // append to end of list, i.e. as dummy.prev
    void append(DblListEl *el) {
        insertBefore(el, &dummy);
    }

    // prepend to front of list (push), i.e. as dummy.next
    void prepend(DblListEl *el) {
        insertAfter(el, &dummy);
    }

    DblListEl *pop() {
        DblListEl *hd = head();
        if (hd != NULL)
            remove(hd);
        return hd;
    }

    // constructor
    DblListBase() {
        dummy.next = dummy.prev = &dummy;
        length = 0;
    }
};


//
// Template class serves solely to cast args & return values
// to appropriate type (T *)
//
template<class T> class DblList : private DblListBase {

  public:

    T *head() { return (T *)DblListBase::head(); }
    T *tail() { return (T *)DblListBase::tail(); }

    T *next(T *el) { return (T *)DblListBase::next(el); }
    T *prev(T *el) { return (T *)DblListBase::prev(el); }

    bool is_empty() { return DblListBase::is_empty(); }

    void remove(T *el) { DblListBase::remove(el); }

    void append(T *el) { DblListBase::append(el); }
    void prepend(T *el) { DblListBase::prepend(el); }

    T *pop() { return (T *)DblListBase::pop(); }

    DblList<T>() { }
};

#endif // __DBL_LIST_HH__
