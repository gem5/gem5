/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __RES_LIST_HH__
#define __RES_LIST_HH__

#include "cprintf.hh"
#include <assert.h>

#define DEBUG_REMOVE 0

#define DEBUG_MEMORY 0
//#define DEBUG_MEMORY DEBUG

class res_list_base
{
#if DEBUG_MEMORY
  protected:
    static long long allocated_elements;
    static long long allocated_lists;

  public:
    long long get_elements(void) {
        return allocated_elements;
    }
    long long get_lists(void) {
        return allocated_lists;
    }

#endif
};

#if DEBUG_MEMORY
extern void what_the(void);
#endif

template<class T>
class res_list : public res_list_base
{
  public:
    class iterator;

    class res_element
    {
        res_element *next;
        res_element *prev;
        T *data;
        bool allocate_data;

      public:
        //  always adds to the END of the list
        res_element(res_element *_prev, bool allocate);
        ~res_element();
        void dump(void);

        friend class res_list<T>;
        friend class res_list<T>::iterator;
    };

    class iterator
    {
      private:
        res_element *p;

        friend class res_list<T>;

      public:
        // Constructors
        iterator(res_element *q) : p(q) {}
        iterator(void) { p=0; };

        void dump(void);
        T* data_ptr(void);
        res_element *res_el_ptr(void) { return p;}
        void point_to(T &d) { p->data = &d; }

        iterator next(void) { return iterator(p->next); }
        iterator prev(void) { return iterator(p->prev); }
        bool operator== (iterator x) { return (x.p == this->p); }
        bool operator != (iterator x) { return (x.p != this->p); }
        T& operator * (void) { return *(p->data); }
        T* operator -> (void) { return p->data; }
        bool isnull(void) { return (p==0); }
        bool notnull(void) { return (p!=0); }
    };

  private:
    iterator unused_elements;
    iterator head_ptr;
    iterator tail_ptr;

    unsigned base_elements;
    unsigned extra_elements;
    unsigned active_elements;
    bool     allocate_storage;
    unsigned build_size;

    int remove_count;

    //
    //  Allocate new elements, and assign them to the unused_elements
    //  list.
    //
    unsigned allocate_elements(unsigned num, bool allocate_storage);

  public:
    //
    //  List Constructor
    //
    res_list(unsigned size, bool alloc_storage = false,
             unsigned build_sz = 5);

    //
    //  List Destructor
    //
    ~res_list();

    iterator head(void) {return head_ptr;};
    iterator tail(void) {return tail_ptr;};

    unsigned num_free(void) { return size() - count(); }
    unsigned size(void) { return base_elements + extra_elements; }
    unsigned count(void) { return active_elements; }
    bool empty(void) { return count() == 0; }
    bool full(void);

    //
    //  Insert with data copy
    //
    iterator insert_after(iterator prev, T *d);
    iterator insert_after(iterator prev, T &d);
    iterator insert_before(iterator prev, T *d);
    iterator insert_before(iterator prev, T &d);

    //
    //  Insert new list element (no data copy)
    //
    iterator insert_after(iterator prev);
    iterator insert_before(iterator prev);

    iterator add_tail(T *d) { return insert_after(tail_ptr, d); }
    iterator add_tail(T &d) { return insert_after(tail_ptr, d); }
    iterator add_tail(void) { return insert_after(tail_ptr); }
    iterator add_head(T *d) { return insert_before(head_ptr, d); }
    iterator add_head(T &d) { return insert_before(head_ptr, d); }
    iterator add_head(void) { return insert_before(head_ptr); }

    iterator remove(iterator q);
    iterator remove_head(void) {return remove(head_ptr);}
    iterator remove_tail(void) {return remove(tail_ptr);}

    bool in_list(iterator j);
    void free_extras(void);
    void clear(void);
    void dump(void);
    void raw_dump(void);
};

template <class T>
inline
res_list<T>::res_element::res_element(res_element *_prev, bool allocate)
{
    allocate_data = allocate;
    prev = _prev;
    next = 0;

    if (prev)
        prev->next = this;

    if (allocate)
        data = new T;
    else
        data = 0;

#if DEBUG_MEMORY
    ++allocated_elements;
#endif
}

template <class T>
inline
res_list<T>::res_element::~res_element(void)
{
    if (prev)
        prev->next = next;

    if (next)
        next->prev = prev;

    if (allocate_data)
        delete data;

#if DEBUG_MEMORY
    --allocated_elements;
#endif
}

template <class T>
inline void
res_list<T>::res_element::dump(void)
{
    cprintf("  prev = %#x\n", prev);
    cprintf("  next = %#x\n", next);
    cprintf("  data = %#x\n", data);
}

template <class T>
inline void
res_list<T>::iterator::dump(void)
{
    if (p && p->data)
        p->data->dump();
    else {
        if (!p)
            cprintf("  Null Pointer\n");
        else
            cprintf("  Null 'data' Pointer\n");
    }
}

template <class T>
inline T *
res_list<T>::iterator::data_ptr(void)
{
    if (p)
        return p->data;
    else
        return 0;
}


//
//  Allocate new elements, and assign them to the unused_elements
//  list.
//
template <class T>
inline unsigned
res_list<T>::allocate_elements(unsigned num, bool allocate_storage)
{
    res_element *pnew, *plast = 0, *pfirst=0;

    for (int i=0; i<num; ++i) {
        pnew = new res_element(plast, allocate_storage);
        if (i==0)
            pfirst = pnew;
        plast = pnew;
    }

    if (unused_elements.notnull()) {
        //  Add these new elements to the front of the list
        plast->next = unused_elements.res_el_ptr();
        unused_elements.res_el_ptr()->prev = plast;
    }

    unused_elements = iterator(pfirst);

    return num;
}

template <class T>
inline
res_list<T>::res_list(unsigned size, bool alloc_storage, unsigned build_sz)
{
#if DEBUG_MEMORY
    ++allocated_lists;
#endif
    extra_elements = 0;
    active_elements = 0;
    build_size = build_sz;
    allocate_storage = alloc_storage;
    remove_count = 0;

    //  Create the new elements
    base_elements = allocate_elements(size, alloc_storage);

    //  The list of active elements
    head_ptr = iterator(0);
    tail_ptr = iterator(0);
}

//
//  List Destructor
//
template <class T>
inline
res_list<T>::~res_list(void)
{
    iterator n;

#if DEBUG_MEMORY
    --allocated_lists;
#endif

    //  put everything into the unused list
    clear();

    //  rudely delete all the res_elements
    for (iterator p = unused_elements;
         p.notnull();
         p = n) {

        n = p.next();

        //  delete the res_element
        //  (it will take care of deleting the data)
        delete p.res_el_ptr();
    }
}

template <class T>
inline bool
res_list<T>::full(void)
{
    if (build_size)
        return false;
    else
        return unused_elements.isnull();
}

//
//  Insert with data copy
//
template <class T>
inline typename res_list<T>::iterator
res_list<T>::insert_after(iterator prev, T *d)
{
    iterator p;

    if (!allocate_storage)
        panic("Can't copy data... not allocating storage");

    p = insert_after(prev);
    if (p.notnull())
        *p = *d;

    return p;
}


template <class T>
inline typename res_list<T>::iterator
res_list<T>::insert_after(iterator prev, T &d)
{
    iterator p;

    p = insert_after(prev);
    if (p.notnull()) {

        if (allocate_storage) {
            //  if we allocate storage, then copy the contents of the
            //  specified object to our object
            *p = d;
        }
        else {
            //  if we don't allocate storage, then we just want to
            //  point to the specified object
            p.point_to(d);
        }
    }

    return p;
}


template <class T>
inline typename res_list<T>::iterator
res_list<T>::insert_after(iterator prev)
{

#if DEBUG_MEMORY
    if (active_elements > 2*base_elements) {
        what_the();
    }
#endif

    //  If we have no unused elements, make some more
    if (unused_elements.isnull()) {

        if (build_size == 0) {
            return 0;   // No space left, and can't allocate more....
        }

        extra_elements += allocate_elements(build_size, allocate_storage);
    }

    //  grab the first unused element
    res_element *p = unused_elements.res_el_ptr();

    unused_elements = unused_elements.next();

    ++active_elements;

    //  Insert the new element
    if (head_ptr.isnull()) {
        //
        //  Special case #1: Empty List
        //
        head_ptr = p;
        tail_ptr = p;
        p->prev = 0;
        p->next = 0;
    }
    else if (prev.isnull()) {
        //
        //  Special case #2: Insert at head
        //

        // our next ptr points to old head element
        p->next = head_ptr.res_el_ptr();

        // our element becomes the new head element
        head_ptr = p;

        // no previous element for the head
        p->prev = 0;

        // old head element points back to this element
        p->next->prev = p;
    }
    else if (prev.next().isnull()) {
        //
        //  Special case #3 Insert at tail
        //

        // our prev pointer points to old tail element
        p->prev = tail_ptr.res_el_ptr();

        // our element becomes the new tail
        tail_ptr = p;

        // no next element for the tail
        p->next = 0;

        // old tail element point to this element
        p->prev->next = p;
    }
    else {
        //
        //  Normal insertion (after prev)
        //
        p->prev = prev.res_el_ptr();
        p->next = prev.next().res_el_ptr();

        prev.res_el_ptr()->next = p;
        p->next->prev = p;
    }

    return iterator(p);
}

template <class T>
inline typename res_list<T>::iterator
res_list<T>::insert_before(iterator next, T &d)
{
    iterator p;

    p = insert_before(next);
    if (p.notnull()) {

        if (allocate_storage) {
            //  if we allocate storage, then copy the contents of the
            //  specified object to our object
            *p = d;
        }
        else {
            //  if we don't allocate storage, then we just want to
            //  point to the specified object
            p.point_to(d);
        }
    }

    return p;
}


template <class T>
inline typename res_list<T>::iterator
res_list<T>::insert_before(iterator next)
{

#if DEBUG_MEMORY
    if (active_elements > 2*base_elements) {
        what_the();
    }
#endif

    //  If we have no unused elements, make some more
    if (unused_elements.isnull()) {

        if (build_size == 0) {
            return 0;   // No space left, and can't allocate more....
        }

        extra_elements += allocate_elements(build_size, allocate_storage);
    }

    //  grab the first unused element
    res_element *p = unused_elements.res_el_ptr();

    unused_elements = unused_elements.next();

    ++active_elements;

    //  Insert the new element
    if (head_ptr.isnull()) {
        //
        //  Special case #1: Empty List
        //
        head_ptr = p;
        tail_ptr = p;
        p->prev = 0;
        p->next = 0;
    }
    else if (next.isnull()) {
        //
        //  Special case #2 Insert at tail
        //

        // our prev pointer points to old tail element
        p->prev = tail_ptr.res_el_ptr();

        // our element becomes the new tail
        tail_ptr = p;

        // no next element for the tail
        p->next = 0;

        // old tail element point to this element
        p->prev->next = p;
    }
    else if (next.prev().isnull()) {
        //
        //  Special case #3: Insert at head
        //

        // our next ptr points to old head element
        p->next = head_ptr.res_el_ptr();

        // our element becomes the new head element
        head_ptr = p;

        // no previous element for the head
        p->prev = 0;

        // old head element points back to this element
        p->next->prev = p;
    }
    else {
        //
        //  Normal insertion (before next)
        //
        p->next = next.res_el_ptr();
        p->prev = next.prev().res_el_ptr();

        next.res_el_ptr()->prev = p;
        p->prev->next = p;
    }

    return iterator(p);
}


template <class T>
inline typename res_list<T>::iterator
res_list<T>::remove(iterator q)
{
    res_element *p = q.res_el_ptr();
    iterator n = 0;

    //  Handle the special cases
    if (active_elements == 1) {    // This is the only element
        head_ptr = 0;
        tail_ptr = 0;
    }
    else if (q == head_ptr) {      // This is the head element
        head_ptr = q.next();
        head_ptr.res_el_ptr()->prev = 0;

        n = head_ptr;
    }
    else if (q == tail_ptr) {      // This is the tail element
        tail_ptr = q.prev();
        tail_ptr.res_el_ptr()->next = 0;
    }
    else {                         // This is between two elements
        p->prev->next = p->next;
        p->next->prev = p->prev;

        // Get the "next" element for return
        n = p->next;
    }

    --active_elements;

    //  Put this element back onto the unused list
    p->next = unused_elements.res_el_ptr();
    p->prev = 0;
    if (p->next) {           // NULL if unused list is empty
        p->next->prev = p;
    }

    if (!allocate_storage) {
        p->data = 0;
    }

    unused_elements = q;

    //  A little "garbage collection"
    if (++remove_count > 10) {
        //	    free_extras();
        remove_count = 0;
    }

#if DEBUG_REMOVE
    unsigned unused_count = 0;
    for (iterator i=unused_elements;
         i.notnull();
         i = i.next()) {

        ++unused_count;
    }

    assert((active_elements+unused_count) == (base_elements+extra_elements));
#endif

    return iterator(n);
}


template <class T>
inline bool
res_list<T>::in_list(iterator j)
{
    iterator i;

    for (i=head(); i.notnull(); i=i.next()) {
        if (j.res_el_ptr() == i.res_el_ptr()) {
            return true;
        }
    }

    return false;
}

template <class T>
inline void
res_list<T>::free_extras(void)
{
    unsigned num_unused = base_elements + extra_elements - active_elements;
    unsigned to_free = extra_elements;
    res_element *p;


    if (extra_elements != 0) {
        //
        //  Free min(extra_elements, # unused elements)
        //
        if (extra_elements > num_unused) {
            to_free = num_unused;
        }

        p = unused_elements.res_el_ptr();
        for(int i=0; i<to_free; ++i) {
            res_element *q = p->next;

            delete p;

            p = q;
        }

        //  update the unused element pointer to point to the first
        //  element that wasn't deleted.
        unused_elements = iterator(p);

        //  Update the number of extra elements
        extra_elements -= to_free;
    }

    return;
}


template <class T>
inline void
res_list<T>::clear(void)
{
    iterator i,n;

    for (i=head_ptr; i.notnull(); i=n) {
        n = i.next();
        remove(i);
    }

    free_extras();
}

template <class T>
inline void
res_list<T>::dump(void)
{
    for (iterator i=head(); !i.isnull(); i=i.next())
        i->dump();
}

template <class T>
inline void
res_list<T>::raw_dump(void)
{
    int j = 0;
    res_element *p;
    for (iterator i=head(); !i.isnull(); i=i.next()) {
        cprintf("Element %d:\n", j);

        if (i.notnull()) {
            p = i.res_el_ptr();
            cprintf("  points to res_element @ %#x\n", p);
            p->dump();
            cprintf("  Data Element:\n");
            i->dump();
        }
        else {
            cprintf("  NULL iterator!\n");
        }

        ++j;
    }

}

#endif // __RES_LIST_HH__
