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

  scfx_mant.h - 

  Original Author: Robert Graulich, Synopsys, Inc.
                   Martin Janssen,  Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: scfx_mant.h,v $
// Revision 1.2  2011/08/24 22:05:43  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef SCFX_MANT_H
#define SCFX_MANT_H


#include "sysc/datatypes/fx/scfx_ieee.h"
#include "sysc/datatypes/fx/scfx_utils.h"
#include "sysc/kernel/sc_macros.h"


namespace sc_dt
{

// classes defined in this module
class scfx_mant;
class scfx_mant_ref;


typedef unsigned int  word;       // Using int because of 64-bit machines.
typedef unsigned short half_word;


// ----------------------------------------------------------------------------
//  CLASS : scfx_mant
//
//  Mantissa class.
// ----------------------------------------------------------------------------

class scfx_mant
{

    word* m_array;
    int   m_size;

public:

    explicit scfx_mant( std::size_t );
             scfx_mant( const scfx_mant& );

    scfx_mant& operator = ( const scfx_mant& );

    ~scfx_mant();

    void clear();

    void resize_to( int, int = 0 );

    int size() const;

    word  operator [] ( int ) const;
    word& operator [] ( int );

    half_word  half_at( int ) const;
    half_word& half_at( int );

    half_word* half_addr( int = 0 ) const;

private:

    static word* alloc( std::size_t );
    static void free( word*, std::size_t );

    static word* alloc_word( std::size_t size );
    static void free_word( word* array, std::size_t size );

};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

inline
int
scfx_mant::size() const
{
    return m_size;
}


inline
word*
scfx_mant::alloc( std::size_t size )
{
#if defined( SC_BIG_ENDIAN )
    return alloc_word( size ) + ( size - 1 );
#elif defined( SC_LITTLE_ENDIAN )
    return alloc_word( size );
#endif
}

inline
void
scfx_mant::free( word* mant, std::size_t size )
{
#if defined( SC_BIG_ENDIAN )
    free_word( mant - ( size - 1 ), size );
#elif defined( SC_LITTLE_ENDIAN )
    free_word( mant, size );
#endif
}

inline
word
scfx_mant::operator[]( int i ) const
{
    SC_ASSERT_( i >= 0 && i < m_size, "mantissa index out of range" );
#if defined( SC_BIG_ENDIAN )
    return m_array[-i];
#elif defined( SC_LITTLE_ENDIAN )
    return m_array[i];
#endif
}

inline
word&
scfx_mant::operator[]( int i )
{
    SC_ASSERT_( i >= 0 && i < m_size, "mantissa index out of range" );
#if defined( SC_BIG_ENDIAN )
    return m_array[-i];
#elif defined( SC_LITTLE_ENDIAN )
    return m_array[i];
#endif
}

inline
scfx_mant::scfx_mant( std::size_t size )
: m_array(0), m_size(size)
{
    m_array = alloc( size );
}

inline
scfx_mant::scfx_mant( const scfx_mant& rhs )
: m_array(0), m_size(rhs.m_size)
{
    m_array = alloc( m_size );
    for( int i = 0; i < m_size; i ++ )
    {
        (*this)[i] = rhs[i];
    }
}

inline
scfx_mant&
scfx_mant::operator = ( const scfx_mant& rhs )
{
    if( &rhs != this )
    {
        if( m_size != rhs.m_size )
	{
	    free( m_array, m_size );
	    m_array = alloc( m_size = rhs.m_size );
	}

	for( int i = 0; i < m_size; i ++ )
	{
	    (*this)[i] = rhs[i];
	}
    }
    return *this;
}

inline
scfx_mant::~scfx_mant()
{
    if( m_array != 0 )
    {
        free( m_array, m_size );
    }
}

inline
void
scfx_mant::clear()
{
    for( int i = 0; i < m_size; i ++ )
    {
        (*this)[i] = 0;
    }
}

inline
void
scfx_mant::resize_to( int size, int restore )
{
    if( size == m_size )
    {
        return;
    }

    if( ! m_array )
    {
        m_array = alloc( m_size = size );
    }
    else
    {
        word* p = alloc( size );

	if( restore )
	{
	    int end = sc_min( size, m_size );
	    if( restore == 1 )		// msb resized -> align at 0
	    {
	        for( int i = 0; i < size; i ++ )
		{
		    if( i < end )
		    {
#if defined( SC_BIG_ENDIAN )
		        p[-i] = m_array[-i];
#elif defined( SC_LITTLE_ENDIAN )
			p[i] = m_array[i];
#endif
		    }
		    else
		    {
#if defined( SC_BIG_ENDIAN )
		        p[-i] = 0;
#elif defined( SC_LITTLE_ENDIAN )
			p[i] = 0;
#endif
		    }
		}
	    }
	    else			// lsb resized -> align at size-1
	    {
	        for( int i = 0; i < size; i ++ )
		{
		    if( i < end )
		    {
#if defined( SC_BIG_ENDIAN )
		        p[-size+1+i] = m_array[-m_size+1+i];
#elif defined( SC_LITTLE_ENDIAN )
			p[size-1-i] = m_array[m_size-1-i];
#endif
		    }
		    else
		    {
#if defined( SC_BIG_ENDIAN )
		        p[-size+1+i] = 0;
#elif defined( SC_LITTLE_ENDIAN )
			p[size-1-i] = 0;
#endif
		    }
		}
	    }
	}

	free( m_array, m_size );
	m_array = p;
	m_size = size;
    }
}

inline
half_word
scfx_mant::half_at( int i ) const
{
    SC_ASSERT_( ( i >> 1 ) >= 0 && ( i >> 1 ) < m_size,
		"mantissa index out of range" );
#if defined( SC_BIG_ENDIAN )
    return reinterpret_cast<half_word*>( m_array )[-i];
#elif defined( SC_LITTLE_ENDIAN )
    return reinterpret_cast<half_word*>( m_array )[i];
#endif
}

inline
half_word&
scfx_mant::half_at( int i )
{
    SC_ASSERT_( ( i >> 1 ) >= 0 && ( i >> 1 ) < m_size,
		"mantissa index out of range" );
#if defined( SC_BIG_ENDIAN )
    return reinterpret_cast<half_word*>( m_array )[-i];
#elif defined( SC_LITTLE_ENDIAN )
    return reinterpret_cast<half_word*>( m_array )[i];
#endif
}

inline
half_word*
scfx_mant::half_addr( int i ) const
{
    SC_ASSERT_( i >= 0 && i < m_size, "mantissa index out of range" );
#if defined( SC_BIG_ENDIAN )
    return reinterpret_cast<half_word*>( m_array - i ) + 1;
#elif defined( SC_LITTLE_ENDIAN )
    return reinterpret_cast<half_word*>( m_array + i );
#endif
}


// ----------------------------------------------------------------------------
//  one's complement of a mantissa
// ----------------------------------------------------------------------------

inline
void
complement( scfx_mant& target, const scfx_mant& source, int size )
{
    for( int i = 0; i < size; i ++ )
    {
        target[i] = ~source[i];
    }
}


// ----------------------------------------------------------------------------
//  increment mantissa
// ----------------------------------------------------------------------------

inline
void
inc( scfx_mant& mant )
{
    for( int i = 0; i < mant.size(); i ++ )
    {
        if( ++ mant[i] )
	{
	    break;
	}
    }
}


// ----------------------------------------------------------------------------
//  CLASS : scfx_mant_ref
//
//  Mantissa reference class.
// ----------------------------------------------------------------------------

class scfx_mant_ref
{

    scfx_mant* m_mant;
    bool       m_not_const;

public:

    scfx_mant_ref();
    scfx_mant_ref( const scfx_mant& );
    scfx_mant_ref( scfx_mant* );

    scfx_mant_ref& operator = ( const scfx_mant& );
    scfx_mant_ref& operator = ( scfx_mant* );

    ~scfx_mant_ref();

    operator scfx_mant&();

    word operator [] ( int );

private:

    void remove_it();

    scfx_mant_ref( const scfx_mant_ref& );
    scfx_mant_ref& operator = ( const scfx_mant_ref& );

    void* operator new( std::size_t sz ) { return ::operator new( sz ); }

};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

inline
void
scfx_mant_ref::remove_it()
{
    if( m_not_const )
    {
        delete m_mant;
    }
}

inline
scfx_mant_ref::scfx_mant_ref()
: m_mant( 0 ), m_not_const( false )
{}

inline
scfx_mant_ref::scfx_mant_ref( const scfx_mant& mant )
: m_mant( const_cast<scfx_mant*>( &mant ) ), m_not_const( false )
{}

inline
scfx_mant_ref::scfx_mant_ref( scfx_mant* mant )
: m_mant( mant ), m_not_const( true )
{}

inline
scfx_mant_ref&
scfx_mant_ref::operator = ( const scfx_mant& mant )
{
    remove_it();

    m_mant = const_cast<scfx_mant*>( &mant );
    m_not_const = false;

    return *this;
}

inline
scfx_mant_ref&
scfx_mant_ref::operator = ( scfx_mant* mant )
{
    remove_it();

    m_mant = mant;
    m_not_const = true;

    return *this;
}

inline
scfx_mant_ref::~scfx_mant_ref()
{
    remove_it();
}

inline
scfx_mant_ref::operator scfx_mant&()
{
    // SC_ASSERT_( m_not_const, "not allowed to modify mant" );
    return *m_mant;
}

inline
word
scfx_mant_ref::operator [] ( int i )
{
    return (*m_mant)[i];
}

} // namespace sc_dt


#endif

// Taf!
