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

  sc_pq.h -- A simple priority queue (which can be used to model multiple 
             clocks). From Cormen-Leiserson-Rivest, Ch.7.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#ifndef SC_PQ_H
#define SC_PQ_H


#include <cassert>

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_ppq_base
//
//  Priority queue base class.
// ----------------------------------------------------------------------------

class sc_ppq_base
{
public:

    typedef int (*compare_fn_t)( const void*, const void* );

    sc_ppq_base( int sz, compare_fn_t cmp );

    ~sc_ppq_base();

    void* top() const
	{ return m_heap[1]; }

    void* extract_top();

    void insert( void* elem );

    int size() const
	{ return m_heap_size; }

    bool empty() const
	{ return (m_heap_size == 0); }

protected:

    int parent( int i ) const
	{ return i >> 1; }

    int left( int i ) const
	{ return i << 1; }

    int right( int i ) const
	{ return (i << 1) + 1; }

    void heapify( int i );

private:

    void**       m_heap;
    int          m_size_alloc;
    int          m_heap_size;
    compare_fn_t m_compar;
};


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_ppq<T>
//
//  This class is a simple implementation of a priority queue based on
//  binary heaps. The class is templatized on its data type. A comparison
//  function needs to be supplied.
// ----------------------------------------------------------------------------

template <class T>
class sc_ppq
    : public sc_ppq_base
{
public:

    // constructor - specify the maximum size of the queue and
    // give a comparison function.

    sc_ppq( int sz, compare_fn_t cmp )
        : sc_ppq_base( sz, cmp )
	{}

    ~sc_ppq()
	{}

    // returns the value of the top element in the priority queue.
    T top() const
	{ return (T) sc_ppq_base::top(); }

    // pops the first element of the priority queue.

    T extract_top()
	{ return (T) sc_ppq_base::extract_top(); }

    // insert a new element to the priority queue.

    void insert( T elem )
	{ sc_ppq_base::insert( (void*) elem ); }

    // size() and empty() are inherited.
};

} // namespace sc_core

// $Log: sc_pq.h,v $
// Revision 1.5  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.4  2011/08/26 20:46:18  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.3  2011/08/24 22:05:56  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.2  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.
//

#endif
