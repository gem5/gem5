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

  sc_vector.h -- Simple implementation of a vector class.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#ifndef SC_VECTOR_H
#define SC_VECTOR_H

#include <vector>

namespace sc_core {

extern "C" {
  typedef int (*CFT)( const void*, const void* );
} 


// #define ACCESS(I) m_vector.at(I) // index checking
#define ACCESS(I) m_vector[I]
#define ADDR_ACCESS(I) (m_vector.size() != 0 ? &m_vector[I] : 0 )

// ----------------------------------------------------------------------------
//  CLASS : sc_pvector<T>
//
//  Simple vector class.
// ----------------------------------------------------------------------------

template< class T >
class sc_pvector
{
public:

    typedef const T* const_iterator;
    typedef       T* iterator;
	// typedef typename ::std::vector<T>::const_iterator const_iterator;
	// typedef typename ::std::vector<T>::iterator       iterator;

    sc_pvector( int alloc_n = 0 )
	{
	}

    sc_pvector( const sc_pvector<T>& rhs )
	: m_vector( rhs.m_vector )
	{}

    ~sc_pvector()
	{}


    std::size_t size() const
	{ return m_vector.size(); }


    iterator begin()
        { return (iterator) ADDR_ACCESS(0); }

    const_iterator begin() const
        { return (const_iterator) ADDR_ACCESS(0); }

    iterator end()
        { return static_cast<iterator> (ADDR_ACCESS(m_vector.size())); }

    const_iterator end() const
    { 
        return static_cast<const_iterator> (ADDR_ACCESS(m_vector.size()));
    }


    sc_pvector<T>& operator = ( const sc_pvector<T>& rhs )
	{ m_vector = rhs.m_vector; return *this; }


    T& operator [] ( unsigned int i )
	{ 
	    if ( i >= m_vector.size() ) m_vector.resize(i+1);
	    return (T&) m_vector.operator [] ( i );
	}

    const T& operator [] ( unsigned int i ) const
	{ 
	    if ( i >= m_vector.size() ) m_vector.resize(i+1);
	    return (const T&) m_vector.operator [] ( i );
	}

    T& fetch( int i )
	{ return ACCESS(i); }

    const T& fetch( int i ) const
	{ return (const T&) ACCESS(i); }


    T* raw_data()
	{ return (T*) &ACCESS(0); }

    const T* raw_data() const
	{ return (const T*) &ACCESS(0); }


    operator const ::std::vector<T>& () const
        { return m_vector; }

    void push_back( T item )
	{ m_vector.push_back( item ); }


    void erase_all()
	{ m_vector.resize(0); }

    void sort( CFT compar )
	{qsort( (void*)&m_vector[0], m_vector.size(), sizeof(void*), compar );}

    /* These methods have been added from Ptr_Array */
    
    void put( T item, int i )
	{ ACCESS(i) = item; }

    void decr_count()
	{ m_vector.resize(m_vector.size()-1); }

    void decr_count( int k )
	{ m_vector.resize(m_vector.size()-k); }



  protected:
    mutable ::std::vector<T> m_vector;    // Actual vector of pointers.
};

#undef ACCESS
#undef ADDR_ACCESS

} // namespace sc_core

// $Log: sc_pvector.h,v $
// Revision 1.4  2011/08/26 20:46:19  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.3  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.2  2011/01/20 16:52:21  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.1  2010/12/07 20:11:45  acg
//  Andy Goodrich: moved sc_pvector class to new header file to allow the
//  use of sc_vector.h for Philipp Hartmann's new sc_vector class.
//
// Revision 1.4  2010/08/03 17:52:15  acg
//   Andy Goodrich: fix signature for size() method of sc_pvector.
//
// Revision 1.3  2008/10/09 21:20:33  acg
//  Andy Goodrich: fixed the way the end() methods calculate their results.
//  I had incorrectly cut and pasted code from the begin() method.
//
// Revision 1.2  2007/01/17 22:44:34  acg
//  Andy Goodrich: fix for Microsoft compiler.
//
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.
//

#endif
