/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  sc_list.cpp -- Simple implementation of a doubly linked list.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/


#include <assert.h>
#include <cstddef>

#include "sysc/kernel/sc_cmnhdr.h"
#include "sysc/utils/sc_iostream.h"
#include "sysc/utils/sc_list.h"
#include "sysc/utils/sc_mempool.h"
#include "sysc/utils/sc_report.h"
#include "sysc/utils/sc_utils_ids.h"

namespace sc_core {

class sc_plist_elem {
    friend class sc_plist_base_iter;
    friend class sc_plist_base;

private:
    sc_plist_elem() : data(0), prev(0), next(0) 
    {}
    sc_plist_elem( void* d, sc_plist_elem* p, sc_plist_elem* n ) :
        data(d), prev(p), next(n) 
    {}
    ~sc_plist_elem()
    {}

    static void* operator new(std::size_t sz)            { return sc_mempool::allocate(sz); }
    static void operator delete(void* p, std::size_t sz) { sc_mempool::release(p, sz);      }
    
    void* data;
    sc_plist_elem* prev;
    sc_plist_elem* next;
};

sc_plist_base::sc_plist_base() : head(0), tail(0) {}

sc_plist_base::~sc_plist_base()
{
    handle_t p;
    for( handle_t h = head; h != 0; h = p ) {
        p = h->next;
        delete h;
    }
}

void
sc_plist_base::erase_all()
{
    handle_t p;
    for( handle_t h = head; h != 0; h = p ) {
        p = h->next;
        delete h;
    }
    head = 0;
    tail = 0;
}

int
sc_plist_base::size() const
{
    int n = 0;
    for( handle_t h = head; h != 0; h = h->next ) {
        n++;
    }
    return n;
}

sc_plist_base::handle_t
sc_plist_base::push_back( void* d )
{
    handle_t q = new sc_plist_elem( d, tail, 0 );
    if (tail) {
        tail->next = q;
        tail = q;
    }
    else {
        head = tail = q;
    }
    return q;
}

sc_plist_base::handle_t
sc_plist_base::push_front( void* d )
{
    handle_t q = new sc_plist_elem( d, (sc_plist_elem*) 0, head );
    if (head) {
        head->prev = q;
        head = q;
    }
    else {
        head = tail = q;
    }
    return q;
}

void*
sc_plist_base::pop_back()
{
    handle_t q = tail;
    void* d = q->data;
    tail = tail->prev;
    delete q;
    if (tail != 0) {
        tail->next = 0;
    }
    else {
        head = 0;
    }
    return d;
}

void*
sc_plist_base::pop_front()
{
    handle_t q = head;
    void* d = q->data;
    head = head->next;
    delete q;
    if (head != 0) {
        head->prev = 0;
    }
    else {
        tail = 0;
    }
    return d;
}

sc_plist_base::handle_t
sc_plist_base::insert_before( handle_t h, void* d )
{
    if (h == 0) {
        return push_back(d);
    }
    else {
        handle_t q = new sc_plist_elem( d, h->prev, h );
        h->prev->next = q;
        h->prev = q;
        return q;
    }
}

sc_plist_base::handle_t
sc_plist_base::insert_after( handle_t h, void* d )
{
    if (h == 0) {
        return push_front(d);
    }
    else {
        handle_t q = new sc_plist_elem( d, h, h->next );
        h->next->prev = q;
        h->next = q;
        return q;
    }
}

void*
sc_plist_base::remove( handle_t h )
{
    if (h == head)
        return pop_front();
    else if (h == tail)
        return pop_back();
    else {
        void* d = h->data;
        h->prev->next = h->next;
        h->next->prev = h->prev;
        delete h;
        return d;
    }
}

void*
sc_plist_base::get( handle_t h ) const
{
    return h->data;
}

void
sc_plist_base::set( handle_t h, void* d )
{
    h->data = d;
}

void
sc_plist_base::mapcar( sc_plist_map_fn f, void* arg )
{
    for (handle_t h = head; h != 0; h = h->next) {
        f( h->data, arg );
    }
}

void*
sc_plist_base::front() const
{

   if (head) {			
        return head->data;
    }				
    else {
      SC_REPORT_ERROR( SC_ID_FRONT_ON_EMPTY_LIST_ , 0 );
      // never reached
      return 0;
    }
}

void*
sc_plist_base::back() const
{
   if (tail) {
        return tail->data;
    }
    else {
      SC_REPORT_ERROR( SC_ID_BACK_ON_EMPTY_LIST_, 0 );
      // never reached
      return 0;
    }
}



sc_plist_base_iter::sc_plist_base_iter( sc_plist_base* l, bool from_tail ) :
    lst(l), ptr( from_tail ? l->tail : l->head )
{
}

void
sc_plist_base_iter::reset( sc_plist_base* l, bool from_tail )
{
    lst = l;
    if (from_tail) {
        ptr = l->tail;
    }
    else {
        ptr = l->head;
    }
}

sc_plist_base_iter::~sc_plist_base_iter()
{

}

bool
sc_plist_base_iter::empty() const
{
    return ptr == 0;
}

void
sc_plist_base_iter::operator++(int)
{
    ptr = ptr->next;
}

void
sc_plist_base_iter::operator--(int)
{
    ptr = ptr->prev;
}

void*
sc_plist_base_iter::get() const
{
    return ptr->data;
}

void
sc_plist_base_iter::set( void* d )
{
    ptr->data = d;
}

void
sc_plist_base_iter::remove()
{
    sc_plist_base::handle_t nptr = ptr->next;
    lst->remove(ptr);
    ptr = nptr;
}

void
sc_plist_base_iter::remove(int direction)
{
    sc_plist_base::handle_t nptr = (direction == 1) ? ptr->next : ptr->prev;
    lst->remove(ptr);
    ptr = nptr;
}

void
sc_plist_base_iter::set_handle( sc_plist_elem* h )
{
    ptr = h;
}

} // namespace sc_core

// $Log: sc_list.cpp,v $
// Revision 1.4  2011/08/26 20:46:18  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.3  2011/08/24 22:05:56  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.2  2011/02/18 20:38:43  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:10  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.

// taf
