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

  sc_lv_base.h -- Arbitrary size logic vector class.

  Original Author: Gene Bushuyev, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:
	Andy Goodrich, Forte Design Systems
	  Fixed bug in clean_tail for sizes that are modulo 32, which caused
	  zeroing of values.

 *****************************************************************************/

// $Log: sc_lv_base.h,v $
// Revision 1.4  2011/08/26 22:32:00  acg
//  Torsten Maehne: added parentheses to make opearator ordering more obvious.
//
// Revision 1.3  2010/01/27 19:41:29  acg
//  Andy Goodrich: fix 8 instances of sc_concref constructor invocations
//  that failed to indicate that their arguments should be freed when the
//  object was freed.
//
// Revision 1.2  2009/02/28 00:26:14  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.2  2007/03/14 17:47:49  acg
//  Andy Goodrich: Formatting.
//
// Revision 1.1.1.1  2006/12/15 20:31:36  acg
// SystemC 2.2
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef SC_LV_BASE_H
#define SC_LV_BASE_H


#include "sysc/datatypes/bit/sc_bit_ids.h"
#include "sysc/datatypes/bit/sc_bv_base.h"
#include "sysc/datatypes/bit/sc_logic.h"
#include "sysc/datatypes/int/sc_length_param.h"


namespace sc_dt
{

// classes defined in this module
class sc_lv_base;


// ----------------------------------------------------------------------------
//  CLASS : sc_lv_base
//
//  Arbitrary size logic vector base class.
// ----------------------------------------------------------------------------

class sc_lv_base
    : public sc_proxy<sc_lv_base>
{
    friend class sc_bv_base;


    void init( int length_, const sc_logic& init_value = SC_LOGIC_X );

    void assign_from_string( const std::string& );

public:

    // typedefs

    typedef sc_proxy<sc_lv_base> base_type;


    // constructors

    explicit sc_lv_base( int length_ = sc_length_param().len() )
	: m_len( 0 ), m_size( 0 ), m_data( 0 ), m_ctrl( 0 )
	{ init( length_ ); }

    explicit sc_lv_base( const sc_logic& a,
			 int length_ = sc_length_param().len()  )
	: m_len( 0 ), m_size( 0 ), m_data( 0 ), m_ctrl( 0 )
	{ init( length_, a ); }

    sc_lv_base( const char* a );

    sc_lv_base( const char* a, int length_ );

    template <class X>
    sc_lv_base( const sc_proxy<X>& a )
	: m_len( 0 ), m_size( 0 ), m_data( 0 ), m_ctrl( 0 )
	{ init( a.back_cast().length() ); base_type::assign_( a ); }

    sc_lv_base( const sc_lv_base& a );

#ifdef SC_DT_DEPRECATED

    explicit sc_lv_base( const sc_unsigned& a )
	: m_len( 0 ), m_size( 0 ), m_data( 0 ), m_ctrl( 0 )
	{ init( a.length() ); base_type::assign_( a ); }

    explicit sc_lv_base( const sc_signed& a )
	: m_len( 0 ), m_size( 0 ), m_data( 0 ), m_ctrl( 0 )
	{ init( a.length() ); base_type::assign_( a ); }

    explicit sc_lv_base( const sc_uint_base& a )
	: m_len( 0 ), m_size( 0 ), m_data( 0 ), m_ctrl( 0 )
	{ init( a.length() ); base_type::assign_( a ); }

    explicit sc_lv_base( const sc_int_base& a )
	: m_len( 0 ), m_size( 0 ), m_data( 0 ), m_ctrl( 0 )
	{ init( a.length() ); base_type::assign_( a ); }

#endif


    // destructor

    virtual ~sc_lv_base()
	{ delete [] m_data; }


    // assignment operators

    template <class X>
    sc_lv_base& operator = ( const sc_proxy<X>& a )
	{ assign_p_( *this, a ); return *this; }

    sc_lv_base& operator = ( const sc_lv_base& a )
	{ assign_p_( *this, a ); return *this; }

    sc_lv_base& operator = ( const char* a );

    sc_lv_base& operator = ( const bool* a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( const sc_logic* a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( const sc_unsigned& a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( const sc_signed& a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( const sc_uint_base& a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( const sc_int_base& a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( unsigned long a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( long a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( unsigned int a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( int a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( uint64 a )
	{ base_type::assign_( a ); return *this; }

    sc_lv_base& operator = ( int64 a )
	{ base_type::assign_( a ); return *this; }


#if 0

    // bitwise complement

    sc_lv_base& b_not()
	{ return sc_proxy<sc_lv_base>::b_not(); }

    const sc_lv_base operator ~ () const
	{ sc_lv_base a( *this ); return a.b_not(); }


    // bitwise left shift

    sc_lv_base& operator <<= ( int n )
	{ return sc_proxy<sc_lv_base>::operator <<= ( n ); }

    const sc_lv_base operator << ( int n ) const
	{ sc_lv_base a( *this ); return ( a <<= n ); }


    // bitwise right shift

    sc_lv_base& operator >>= ( int n )
	{ return sc_proxy<sc_lv_base>::operator >>= ( n ); }

    const sc_lv_base operator >> ( int n ) const
	{ sc_lv_base a( *this ); return ( a >>= n ); }


    // bitwise left rotate

    sc_lv_base& lrotate( int n )
	{ return sc_proxy<sc_lv_base>::lrotate( n ); }


    // bitwise right rotate

    sc_lv_base& rrotate( int n )
	{ return sc_proxy<sc_lv_base>::rrotate( n ); }

#endif


    // common methods

    int length() const
	{ return m_len; }

    int size() const
	{ return m_size; }

    sc_logic_value_t get_bit( int i ) const;
    void set_bit( int i, sc_logic_value_t value );

    sc_digit get_word( int wi ) const
	{ return m_data[wi]; }

	// note the test for out of range access here. this is necessary 
	// because of the hair-brained way concatenations are set up.
	// an extend_sign on a concatenation uses the whole length of 
	// the concatenation to determine how many words to set.
    void set_word( int wi, sc_digit w )
	{ assert ( wi < m_size ); m_data[wi] = w; }
	 

    sc_digit get_cword( int wi ) const
	{ return m_ctrl[wi]; }

    void set_cword( int wi, sc_digit w )
	{ assert ( wi < m_size ); m_ctrl[wi] = w; }

    void clean_tail();


    // other methods

    bool is_01() const;

protected:

    int     m_len;   // length in bits
    int     m_size;  // size of the data array
    sc_digit* m_data;  // data array
    sc_digit* m_ctrl;  // dito (control part)
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

#if 0

// bitwise left rotate

inline
const sc_lv_base
lrotate( const sc_lv_base& x, int n )
{
    sc_lv_base a( x );
    return a.lrotate( n );
}


// bitwise right rotate

inline
const sc_lv_base
rrotate( const sc_lv_base& x, int n )
{
    sc_lv_base a( x );
    return a.rrotate( n );
}

#endif


inline
sc_logic_value_t
sc_lv_base::get_bit( int i ) const
{
    int wi = i / SC_DIGIT_SIZE;
    int bi = i % SC_DIGIT_SIZE;
    return sc_logic_value_t( ((m_data[wi] >> bi) & SC_DIGIT_ONE) |
			     (((m_ctrl[wi] >> bi) << 1) & SC_DIGIT_TWO) );
}

inline
void
sc_lv_base::set_bit( int i, sc_logic_value_t value )
{
    int wi = i / SC_DIGIT_SIZE; // word index
    int bi = i % SC_DIGIT_SIZE; // bit index
    sc_digit mask = SC_DIGIT_ONE << bi;
    m_data[wi] |= mask; // set bit to 1
    m_ctrl[wi] |= mask; // set bit to 1
    m_data[wi] &= value << bi | ~mask;
    m_ctrl[wi] &= value >> 1 << bi | ~mask;
}


inline
void
sc_lv_base::clean_tail()
{
    int wi = m_size - 1;
    int bi = m_len % SC_DIGIT_SIZE;
    sc_digit mask = ~SC_DIGIT_ZERO >> (SC_DIGIT_SIZE - bi);
	if ( mask )
	{
		m_data[wi] &= mask;
		m_ctrl[wi] &= mask;
	}
}


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_proxy
//
//  Base class template for bit/logic vector classes.
//  (Barton/Nackmann implementation)
// ----------------------------------------------------------------------------

// bitwise operators and functions

// bitwise complement

template <class X>
inline
const sc_lv_base
sc_proxy<X>::operator ~ () const
{
    sc_lv_base a( back_cast() );
    return a.b_not();
}


// bitwise and

template <class X, class Y>
inline
X&
operator &= ( sc_proxy<X>& px, const sc_proxy<Y>& py )
{
    X& x = px.back_cast();
    sc_lv_base a( x.length() );
    a = py.back_cast();
    return b_and_assign_( x, a );
}


#define DEFN_BITWISE_AND_ASN_OP_T(tp)                                         \
template <class X>                                                            \
inline                                                                        \
X&                                                                            \
sc_proxy<X>::operator &= ( tp b )                                             \
{                                                                             \
    X& x = back_cast();                                                       \
    sc_lv_base a( x.length() );                                               \
    a = b;                                                                    \
    return b_and_assign_( x, a );                                             \
}

DEFN_BITWISE_AND_ASN_OP_T(const char*)
DEFN_BITWISE_AND_ASN_OP_T(const bool*)
DEFN_BITWISE_AND_ASN_OP_T(const sc_logic*)
DEFN_BITWISE_AND_ASN_OP_T(const sc_unsigned&)
DEFN_BITWISE_AND_ASN_OP_T(const sc_signed&)
DEFN_BITWISE_AND_ASN_OP_T(unsigned long)
DEFN_BITWISE_AND_ASN_OP_T(long)
DEFN_BITWISE_AND_ASN_OP_T(uint64)
DEFN_BITWISE_AND_ASN_OP_T(int64)

#undef DEFN_BITWISE_AND_ASN_OP_T


template <class X, class Y>
inline
const sc_lv_base
operator & ( const sc_proxy<X>& px, const sc_proxy<Y>& py )
{
    sc_lv_base a( px.back_cast() );
    return ( a &= py.back_cast() );
}


#define DEFN_BITWISE_AND_OP_T_A(tp)                                           \
template <class X>                                                            \
inline                                                                        \
const sc_lv_base                                                              \
sc_proxy<X>::operator & ( tp b ) const                                        \
{                                                                             \
    sc_lv_base a( back_cast() );                                              \
    return ( a &= b );                                                        \
}

DEFN_BITWISE_AND_OP_T_A(const char*)
DEFN_BITWISE_AND_OP_T_A(const bool*)
DEFN_BITWISE_AND_OP_T_A(const sc_logic*)
DEFN_BITWISE_AND_OP_T_A(const sc_unsigned&)
DEFN_BITWISE_AND_OP_T_A(const sc_signed&)
DEFN_BITWISE_AND_OP_T_A(const sc_uint_base&)
DEFN_BITWISE_AND_OP_T_A(const sc_int_base&)
DEFN_BITWISE_AND_OP_T_A(unsigned long)
DEFN_BITWISE_AND_OP_T_A(long)
DEFN_BITWISE_AND_OP_T_A(unsigned int)
DEFN_BITWISE_AND_OP_T_A(int)
DEFN_BITWISE_AND_OP_T_A(uint64)
DEFN_BITWISE_AND_OP_T_A(int64)

#undef DEFN_BITWISE_AND_OP_T_A


#define DEFN_BITWISE_AND_OP_T_B(tp)                                           \
template <class X>                                                            \
inline                                                                        \
const sc_lv_base                                                              \
operator & ( tp b, const sc_proxy<X>& px )                                    \
{                                                                             \
    return ( px & b );                                                        \
}

DEFN_BITWISE_AND_OP_T_B(const char*)
DEFN_BITWISE_AND_OP_T_B(const bool*)
DEFN_BITWISE_AND_OP_T_B(const sc_logic*)
DEFN_BITWISE_AND_OP_T_B(const sc_unsigned&)
DEFN_BITWISE_AND_OP_T_B(const sc_signed&)
DEFN_BITWISE_AND_OP_T_B(const sc_uint_base&)
DEFN_BITWISE_AND_OP_T_B(const sc_int_base&)
DEFN_BITWISE_AND_OP_T_B(unsigned long)
DEFN_BITWISE_AND_OP_T_B(long)
DEFN_BITWISE_AND_OP_T_B(unsigned int)
DEFN_BITWISE_AND_OP_T_B(int)
DEFN_BITWISE_AND_OP_T_B(uint64)
DEFN_BITWISE_AND_OP_T_B(int64)

#undef DEFN_BITWISE_AND_OP_T_B


// bitwise or

template <class X, class Y>
inline
X&
operator |= ( sc_proxy<X>& px, const sc_proxy<Y>& py )
{
    X& x = px.back_cast();
    sc_lv_base a( x.length() );
    a = py.back_cast();
    return b_or_assign_( x, a );
}


#define DEFN_BITWISE_OR_ASN_OP_T(tp)                                          \
template <class X>                                                            \
inline                                                                        \
X&                                                                            \
sc_proxy<X>::operator |= ( tp b )                                             \
{                                                                             \
    X& x = back_cast();                                                       \
    sc_lv_base a( x.length() );                                               \
    a = b;                                                                    \
    return b_or_assign_( x, a );                                              \
}

DEFN_BITWISE_OR_ASN_OP_T(const char*)
DEFN_BITWISE_OR_ASN_OP_T(const bool*)
DEFN_BITWISE_OR_ASN_OP_T(const sc_logic*)
DEFN_BITWISE_OR_ASN_OP_T(const sc_unsigned&)
DEFN_BITWISE_OR_ASN_OP_T(const sc_signed&)
DEFN_BITWISE_OR_ASN_OP_T(unsigned long)
DEFN_BITWISE_OR_ASN_OP_T(long)
DEFN_BITWISE_OR_ASN_OP_T(uint64)
DEFN_BITWISE_OR_ASN_OP_T(int64)

#undef DEFN_BITWISE_OR_ASN_OP_T


template <class X, class Y>
inline
const sc_lv_base
operator | ( const sc_proxy<X>& px, const sc_proxy<Y>& py )
{
    sc_lv_base a( px.back_cast() );
    return ( a |= py.back_cast() );
}


#define DEFN_BITWISE_OR_OP_T_A(tp)                                            \
template <class X>                                                            \
inline                                                                        \
const sc_lv_base                                                              \
sc_proxy<X>::operator | ( tp b ) const                                        \
{                                                                             \
    sc_lv_base a( back_cast() );                                              \
    return ( a |= b );                                                        \
}

DEFN_BITWISE_OR_OP_T_A(const char*)
DEFN_BITWISE_OR_OP_T_A(const bool*)
DEFN_BITWISE_OR_OP_T_A(const sc_logic*)
DEFN_BITWISE_OR_OP_T_A(const sc_unsigned&)
DEFN_BITWISE_OR_OP_T_A(const sc_signed&)
DEFN_BITWISE_OR_OP_T_A(const sc_uint_base&)
DEFN_BITWISE_OR_OP_T_A(const sc_int_base&)
DEFN_BITWISE_OR_OP_T_A(unsigned long)
DEFN_BITWISE_OR_OP_T_A(long)
DEFN_BITWISE_OR_OP_T_A(unsigned int)
DEFN_BITWISE_OR_OP_T_A(int)
DEFN_BITWISE_OR_OP_T_A(uint64)
DEFN_BITWISE_OR_OP_T_A(int64)

#undef DEFN_BITWISE_OR_OP_T_A


#define DEFN_BITWISE_OR_OP_T_B(tp)                                           \
template <class X>                                                            \
inline                                                                        \
const sc_lv_base                                                              \
operator | ( tp b, const sc_proxy<X>& px )                                    \
{                                                                             \
    return ( px | b );                                                        \
}

DEFN_BITWISE_OR_OP_T_B(const char*)
DEFN_BITWISE_OR_OP_T_B(const bool*)
DEFN_BITWISE_OR_OP_T_B(const sc_logic*)
DEFN_BITWISE_OR_OP_T_B(const sc_unsigned&)
DEFN_BITWISE_OR_OP_T_B(const sc_signed&)
DEFN_BITWISE_OR_OP_T_B(const sc_uint_base&)
DEFN_BITWISE_OR_OP_T_B(const sc_int_base&)
DEFN_BITWISE_OR_OP_T_B(unsigned long)
DEFN_BITWISE_OR_OP_T_B(long)
DEFN_BITWISE_OR_OP_T_B(unsigned int)
DEFN_BITWISE_OR_OP_T_B(int)
DEFN_BITWISE_OR_OP_T_B(uint64)
DEFN_BITWISE_OR_OP_T_B(int64)

#undef DEFN_BITWISE_OR_OP_T_B


// bitwise xor

template <class X, class Y>
inline
X&
operator ^= ( sc_proxy<X>& px, const sc_proxy<Y>& py )
{
    X& x = px.back_cast();
    sc_lv_base a( x.length() );
    a = py.back_cast();
    return b_xor_assign_( x, a );
}


#define DEFN_BITWISE_XOR_ASN_OP_T(tp)                                         \
template <class X>                                                            \
inline                                                                        \
X&                                                                            \
sc_proxy<X>::operator ^= ( tp b )                                             \
{                                                                             \
    X& x = back_cast();                                                       \
    sc_lv_base a( x.length() );                                               \
    a = b;                                                                    \
    return b_xor_assign_( x, a );                                             \
}

DEFN_BITWISE_XOR_ASN_OP_T(const char*)
DEFN_BITWISE_XOR_ASN_OP_T(const bool*)
DEFN_BITWISE_XOR_ASN_OP_T(const sc_logic*)
DEFN_BITWISE_XOR_ASN_OP_T(const sc_unsigned&)
DEFN_BITWISE_XOR_ASN_OP_T(const sc_signed&)
DEFN_BITWISE_XOR_ASN_OP_T(unsigned long)
DEFN_BITWISE_XOR_ASN_OP_T(long)
DEFN_BITWISE_XOR_ASN_OP_T(uint64)
DEFN_BITWISE_XOR_ASN_OP_T(int64)

#undef DEFN_BITWISE_XOR_ASN_OP_T


template <class X, class Y>
inline
const sc_lv_base
operator ^ ( const sc_proxy<X>& px, const sc_proxy<Y>& py )
{
    sc_lv_base a( px.back_cast() );
    return ( a ^= py.back_cast() );
}


#define DEFN_BITWISE_XOR_OP_T_A(tp)                                           \
template <class X>                                                            \
inline                                                                        \
const sc_lv_base                                                              \
sc_proxy<X>::operator ^ ( tp b ) const                                        \
{                                                                             \
    sc_lv_base a( back_cast() );                                              \
    return ( a ^= b );                                                        \
}

DEFN_BITWISE_XOR_OP_T_A(const char*)
DEFN_BITWISE_XOR_OP_T_A(const bool*)
DEFN_BITWISE_XOR_OP_T_A(const sc_logic*)
DEFN_BITWISE_XOR_OP_T_A(const sc_unsigned&)
DEFN_BITWISE_XOR_OP_T_A(const sc_signed&)
DEFN_BITWISE_XOR_OP_T_A(const sc_uint_base&)
DEFN_BITWISE_XOR_OP_T_A(const sc_int_base&)
DEFN_BITWISE_XOR_OP_T_A(unsigned long)
DEFN_BITWISE_XOR_OP_T_A(long)
DEFN_BITWISE_XOR_OP_T_A(unsigned int)
DEFN_BITWISE_XOR_OP_T_A(int)
DEFN_BITWISE_XOR_OP_T_A(uint64)
DEFN_BITWISE_XOR_OP_T_A(int64)

#undef DEFN_BITWISE_XOR_OP_T_A


#define DEFN_BITWISE_XOR_OP_T_B(tp)                                           \
template <class X>                                                            \
inline                                                                        \
const sc_lv_base                                                              \
operator ^ ( tp b, const sc_proxy<X>& px )                                    \
{                                                                             \
    return ( px ^ b );                                                        \
}

DEFN_BITWISE_XOR_OP_T_B(const char*)
DEFN_BITWISE_XOR_OP_T_B(const bool*)
DEFN_BITWISE_XOR_OP_T_B(const sc_logic*)
DEFN_BITWISE_XOR_OP_T_B(const sc_unsigned&)
DEFN_BITWISE_XOR_OP_T_B(const sc_signed&)
DEFN_BITWISE_XOR_OP_T_B(const sc_uint_base&)
DEFN_BITWISE_XOR_OP_T_B(const sc_int_base&)
DEFN_BITWISE_XOR_OP_T_B(unsigned long)
DEFN_BITWISE_XOR_OP_T_B(long)
DEFN_BITWISE_XOR_OP_T_B(unsigned int)
DEFN_BITWISE_XOR_OP_T_B(int)
DEFN_BITWISE_XOR_OP_T_B(uint64)
DEFN_BITWISE_XOR_OP_T_B(int64)

#undef DEFN_BITWISE_XOR_OP_T_B


// bitwise left shift

template <class X>
inline
const sc_lv_base
sc_proxy<X>::operator << ( int n ) const
{
    sc_lv_base a( back_cast().length()+n );
	a = back_cast();
    return ( a <<= n );
}


// bitwise right shift

template <class X>
inline
const sc_lv_base
sc_proxy<X>::operator >> ( int n ) const
{
    sc_lv_base a( back_cast() );
    return ( a >>= n );
}


// bitwise left rotate

template <class X>
inline
X&
sc_proxy<X>::lrotate( int n )
{
    X& x = back_cast();
    if( n < 0 ) {
	char msg[BUFSIZ];
	std::sprintf( msg,
		 "left rotate operation is only allowed with positive "
		 "rotate values, rotate value = %d", n );
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, msg );
    }
    int len = x.length();
    n %= len;
    // x = (x << n) | (x >> (len - n));
    sc_lv_base a( x << n );
    sc_lv_base b( x >> (len - n) );
    int sz = x.size();
    for( int i = 0; i < sz; ++ i ) {
	x.set_word( i, a.get_word( i ) | b.get_word( i ) );
	x.set_cword( i, a.get_cword( i ) | b.get_cword( i ) );
    }
    x.clean_tail();
    return x;
}

template <class X>
inline
const sc_lv_base
lrotate( const sc_proxy<X>& x, int n )
{
    sc_lv_base a( x.back_cast() );
    return a.lrotate( n );
}


// bitwise right rotate

template <class X>
inline
X&
sc_proxy<X>::rrotate( int n )
{
    X& x = back_cast();
    if( n < 0 ) {
	char msg[BUFSIZ];
	std::sprintf( msg,
		 "right rotate operation is only allowed with positive "
		 "rotate values, rotate value = %d", n );
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, msg );
    }
    int len = x.length();
    n %= len;
    // x = (x >> n) | (x << (len - n));
    sc_lv_base a( x >> n );
    sc_lv_base b( x << (len - n) );
    int sz = x.size();
    for( int i = 0; i < sz; ++ i ) {
	x.set_word( i, a.get_word( i ) | b.get_word( i ) );
	x.set_cword( i, a.get_cword( i ) | b.get_cword( i ) );
    }
    x.clean_tail();
    return x;
}

template <class X>
inline
const sc_lv_base
rrotate( const sc_proxy<X>& x, int n )
{
    sc_lv_base a( x.back_cast() );
    return a.rrotate( n );
}


// bitwise reverse

template <class X>
inline
const sc_lv_base
reverse( const sc_proxy<X>& x )
{
    sc_lv_base a( x.back_cast() );
    return a.reverse();
}


// relational operators

template <class X, class Y>
inline
bool
operator == ( const sc_proxy<X>& px, const sc_proxy<Y>& py )
{
    const X& x = px.back_cast();
    const Y& y = py.back_cast();
    int x_len = x.length();
    int y_len = y.length();
    if( x_len != y_len ) {
	return false;
    }
    int sz = x.size();
    for( int i = 0; i < sz; ++ i ) {
	if( x.get_word( i ) != y.get_word( i ) ||
	    x.get_cword( i ) != y.get_cword( i ) ) {
	    return false;
	}
    }
    return true;
}


#define DEFN_REL_OP_T(tp)                                                     \
template <class X>                                                            \
inline                                                                        \
bool                                                                          \
sc_proxy<X>::operator == ( tp b ) const                                       \
{                                                                             \
    const X& x = back_cast();                                                 \
    sc_lv_base y( x.length() );                                               \
    y = b;                                                                    \
    return ( x == y );                                                        \
}

DEFN_REL_OP_T(const char*)
DEFN_REL_OP_T(const bool*)
DEFN_REL_OP_T(const sc_logic*)
DEFN_REL_OP_T(const sc_unsigned&)
DEFN_REL_OP_T(const sc_signed&)
DEFN_REL_OP_T(const sc_uint_base&)
DEFN_REL_OP_T(const sc_int_base&)
DEFN_REL_OP_T(unsigned long)
DEFN_REL_OP_T(long)
DEFN_REL_OP_T(unsigned int)
DEFN_REL_OP_T(int)
DEFN_REL_OP_T(uint64)
DEFN_REL_OP_T(int64)

#undef DEFN_REL_OP_T


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_bitref_r<X>
//
//  Proxy class for sc_proxy bit selection (r-value only).
// ----------------------------------------------------------------------------

// r-value concatenation operators and functions

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
operator , ( sc_bitref_r<T> a, const char* b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
operator , ( const char* a, sc_bitref_r<T> b )
{
    return sc_concref_r<sc_lv_base,sc_bitref_r<T> >(
	*new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
operator , ( sc_bitref_r<T> a, const sc_logic& b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
operator , ( const sc_logic& a, sc_bitref_r<T> b )
{
    return sc_concref_r<sc_lv_base,sc_bitref_r<T> >(
	*new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_bv_base>
operator , ( sc_bitref_r<T> a, bool b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,sc_bitref_r<T> >
operator , ( bool a, sc_bitref_r<T> b )
{
    return sc_concref_r<sc_bv_base,sc_bitref_r<T> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}


template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
concat( sc_bitref_r<T> a, const char* b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
concat( const char* a, sc_bitref_r<T> b )
{
    return sc_concref_r<sc_lv_base,sc_bitref_r<T> >(
	*new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
concat( sc_bitref_r<T> a, const sc_logic& b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
concat( const sc_logic& a, sc_bitref_r<T> b )
{
    return sc_concref_r<sc_lv_base,sc_bitref_r<T> >(
	*new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_bv_base>
concat( sc_bitref_r<T> a, bool b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,sc_bitref_r<T> >
concat( bool a, sc_bitref_r<T> b )
{
    return sc_concref_r<sc_bv_base,sc_bitref_r<T> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
operator , ( sc_bitref<T> a, const char* b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
operator , ( const char* a, sc_bitref<T> b )
{
    return sc_concref_r<sc_lv_base,sc_bitref_r<T> >(
	*new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
operator , ( sc_bitref<T> a, const sc_logic& b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
operator , ( const sc_logic& a, sc_bitref<T> b )
{
    return sc_concref_r<sc_lv_base,sc_bitref_r<T> >(
	*new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_bv_base>
operator , ( sc_bitref<T> a, bool b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,sc_bitref_r<T> >
operator , ( bool a, sc_bitref<T> b )
{
    return sc_concref_r<sc_bv_base,sc_bitref_r<T> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}


template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
concat( sc_bitref<T> a, const char* b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
concat( const char* a, sc_bitref<T> b )
{
    return sc_concref_r<sc_lv_base,sc_bitref_r<T> >(
	*new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
concat( sc_bitref<T> a, const sc_logic& b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
concat( const sc_logic& a, sc_bitref<T> b )
{
    return sc_concref_r<sc_lv_base,sc_bitref_r<T> >(
	*new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_bv_base>
concat( sc_bitref<T> a, bool b )
{
    return sc_concref_r<sc_bitref_r<T>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,sc_bitref_r<T> >
concat( bool a, sc_bitref<T> b )
{
    return sc_concref_r<sc_bv_base,sc_bitref_r<T> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}

#endif


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_subref_r<X>
//
//  Proxy class for sc_proxy part selection (r-value only).
// ----------------------------------------------------------------------------

// r-value concatenation operators and functions

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
operator , ( sc_subref_r<T> a, const char* b )
{
    return sc_concref_r<sc_subref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
operator , ( const char* a, sc_subref_r<T> b )
{
    return sc_concref_r<sc_lv_base,sc_subref_r<T> >(
	*new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
operator , ( sc_subref_r<T> a, const sc_logic& b )
{
    return sc_concref_r<sc_subref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
operator , ( const sc_logic& a, sc_subref_r<T> b )
{
    return sc_concref_r<sc_lv_base,sc_subref_r<T> >(
	*new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_bv_base>
operator , ( sc_subref_r<T> a, bool b )
{
    return sc_concref_r<sc_subref_r<T>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,sc_subref_r<T> >
operator , ( bool a, sc_subref_r<T> b )
{
    return sc_concref_r<sc_bv_base,sc_subref_r<T> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}


template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
concat( sc_subref_r<T> a, const char* b )
{
    return sc_concref_r<sc_subref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
concat( const char* a, sc_subref_r<T> b )
{
    return sc_concref_r<sc_lv_base,sc_subref_r<T> >(
	*new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
concat( sc_subref_r<T> a, const sc_logic& b )
{
    return sc_concref_r<sc_subref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
concat( const sc_logic& a, sc_subref_r<T> b )
{
    return sc_concref_r<sc_lv_base,sc_subref_r<T> >(
	*new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_bv_base>
concat( sc_subref_r<T> a, bool b )
{
    return sc_concref_r<sc_subref_r<T>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,sc_subref_r<T> >
concat( bool a, sc_subref_r<T> b )
{
    return sc_concref_r<sc_bv_base,sc_subref_r<T> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
operator , ( sc_subref<T> a, const char* b )
{
    return sc_concref_r<sc_subref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
operator , ( const char* a, sc_subref<T> b )
{
    return sc_concref_r<sc_lv_base,sc_subref_r<T> >(
	*new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
operator , ( sc_subref<T> a, const sc_logic& b )
{
    return sc_concref_r<sc_subref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
operator , ( const sc_logic& a, sc_subref<T> b )
{
    return sc_concref_r<sc_lv_base,sc_subref_r<T> >(
	*new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_bv_base>
operator , ( sc_subref<T> a, bool b )
{
    return sc_concref_r<sc_subref_r<T>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,sc_subref_r<T> >
operator , ( bool a, sc_subref<T> b )
{
    return sc_concref_r<sc_bv_base,sc_subref_r<T> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}


template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
concat( sc_subref<T> a, const char* b )
{
    return sc_concref_r<sc_subref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
concat( const char* a, sc_subref<T> b )
{
    return sc_concref_r<sc_lv_base,sc_subref_r<T> >(
	*new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
concat( sc_subref<T> a, const sc_logic& b )
{
    return sc_concref_r<sc_subref_r<T>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
concat( const sc_logic& a, sc_subref<T> b )
{
    return sc_concref_r<sc_lv_base,sc_subref_r<T> >(
	*new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_bv_base>
concat( sc_subref<T> a, bool b )
{
    return sc_concref_r<sc_subref_r<T>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,sc_subref_r<T> >
concat( bool a, sc_subref<T> b )
{
    return sc_concref_r<sc_bv_base,sc_subref_r<T> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}

#endif


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_subref<X>
//
//  Proxy class for sc_proxy part selection (r-value and l-value).
// ----------------------------------------------------------------------------

template <class X>
inline
sc_subref<X>&
sc_subref<X>::operator = ( const sc_subref_r<X>& b )
{
    sc_lv_base t( b ); // (partial) self assignment protection
    int len = sc_min( this->length(), t.length() );
    if( ! this->reversed() ) {
        for( int i = len - 1; i >= 0; -- i ) {
            this->m_obj.set_bit( this->m_lo + i, t[i].value() );
        }
    } else {
        for( int i = len - 1; i >= 0; -- i ) {
            this->m_obj.set_bit( this->m_lo - i, t[i].value() );
        }
    }
    return *this;
}

template <class X>
inline
sc_subref<X>&
sc_subref<X>::operator = ( const sc_subref<X>& b )
{
    sc_lv_base t( b ); // (partial) self assignment protection
    int len = sc_min( this->length(), t.length() );
    if( ! this->reversed() ) {
        for( int i = len - 1; i >= 0; -- i ) {
            this->m_obj.set_bit( this->m_lo + i, t[i].value() );
        }
    } else {
        for( int i = len - 1; i >= 0; -- i ) {
            this->m_obj.set_bit( this->m_lo - i, t[i].value() );
        }
    }
    return *this;
}


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_concref_r<X,Y>
//
//  Proxy class for sc_proxy concatenation (r-value only).
// ----------------------------------------------------------------------------

// r-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
operator , ( sc_concref_r<T1,T2> a, const char* b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
operator , ( const char* a, sc_concref_r<T1,T2> b )
{
    return sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >(
	*new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
operator , ( sc_concref_r<T1,T2> a, const sc_logic& b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>(
	*a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
operator , ( const sc_logic& a, sc_concref_r<T1,T2> b )
{
    return sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >(
	*new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
operator , ( sc_concref_r<T1,T2> a, bool b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
operator , ( bool a, sc_concref_r<T1,T2> b )
{
    return sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}


template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
concat( sc_concref_r<T1,T2> a, const char* b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
        ( *a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
concat( const char* a, sc_concref_r<T1,T2> b )
{
    return sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
        ( *new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
concat( sc_concref_r<T1,T2> a, const sc_logic& b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
        ( *a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
concat( const sc_logic& a, sc_concref_r<T1,T2> b )
{
    return sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
        ( *new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
concat( sc_concref_r<T1,T2> a, bool b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
concat( bool a, sc_concref_r<T1,T2> b )
{
    return sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
operator , ( sc_concref<T1,T2> a, const char* b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
        ( *a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
operator , ( const char* a, sc_concref<T1,T2> b )
{
    return sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
        ( *new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
operator , ( sc_concref<T1,T2> a, const sc_logic& b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
        ( *a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
operator , ( const sc_logic& a, sc_concref<T1,T2> b )
{
    return sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
        ( *new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
operator , ( sc_concref<T1,T2> a, bool b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
operator , ( bool a, sc_concref<T1,T2> b )
{
    return sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}


template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
concat( sc_concref<T1,T2> a, const char* b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
        ( *a.clone(), *new sc_lv_base( b ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
concat( const char* a, sc_concref<T1,T2> b )
{
    return sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
        ( *new sc_lv_base( a ), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
concat( sc_concref<T1,T2> a, const sc_logic& b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
        ( *a.clone(), *new sc_lv_base( b, 1 ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
concat( const sc_logic& a, sc_concref<T1,T2> b )
{
    return sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
        ( *new sc_lv_base( a, 1 ), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
concat( sc_concref<T1,T2> a, bool b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
        ( *a.clone(), *new sc_bv_base( b, 1 ), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
concat( bool a, sc_concref<T1,T2> b )
{
    return sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
        ( *new sc_bv_base( a, 1 ), *b.clone(), 3 );
}

#endif


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_proxy<T>
//
//  Base class template for bit/logic vector classes.
//  (Barton/Nackmann implementation)
// ----------------------------------------------------------------------------

// r-value concatenation operators and functions

template <class T>
inline
sc_concref_r<T,sc_lv_base>
operator , ( const sc_proxy<T>& a, const char* b )
{
    return sc_concref_r<T,sc_lv_base>
      ( a.back_cast(), *new sc_lv_base( b ), 2 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,T>
operator , ( const char* a, const sc_proxy<T>& b )
{
    return sc_concref_r<sc_lv_base,T>
      ( *new sc_lv_base( a ), b.back_cast(), 1 );
}

template <class T>
inline
sc_concref_r<T,sc_lv_base>
operator , ( const sc_proxy<T>& a, const sc_logic& b )
{
    return sc_concref_r<T,sc_lv_base>
      ( a.back_cast(), *new sc_lv_base( b, 1 ), 2 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,T>
operator , ( const sc_logic& a, const sc_proxy<T>& b )
{
    return sc_concref_r<sc_lv_base,T>
      ( *new sc_lv_base( a, 1 ), b.back_cast(), 1 );
}

template <class T>
inline
sc_concref_r<T,sc_bv_base>
operator , ( const sc_proxy<T>& a, bool b )
{
    return sc_concref_r<T,sc_bv_base>
        ( a.back_cast(), *new sc_bv_base( b, 1 ), 2 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,T>
operator , ( bool a, const sc_proxy<T>& b )
{
    return sc_concref_r<sc_bv_base,T>
      ( *new sc_bv_base( a, 1 ), b.back_cast(), 1 );
}


template <class T>
inline
sc_concref_r<T,sc_lv_base>
concat( const sc_proxy<T>& a, const char* b )
{
    return sc_concref_r<T,sc_lv_base>
      ( a.back_cast(), *new sc_lv_base( b ), 2 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,T>
concat( const char* a, const sc_proxy<T>& b )
{
    return sc_concref_r<sc_lv_base,T>
      ( *new sc_lv_base( a ), b.back_cast(), 1 );
}

template <class T>
inline
sc_concref_r<T,sc_lv_base>
concat( const sc_proxy<T>& a, const sc_logic& b )
{
    return sc_concref_r<T,sc_lv_base>
      ( a.back_cast(), *new sc_lv_base( b, 1 ), 2 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,T>
concat( const sc_logic& a, const sc_proxy<T>& b )
{
    return sc_concref_r<sc_lv_base,T>
      ( *new sc_lv_base( a, 1 ), b.back_cast(), 1 );
}

template <class T>
inline
sc_concref_r<T,sc_bv_base>
concat( const sc_proxy<T>& a, bool b )
{
    return sc_concref_r<T,sc_bv_base>
      ( a.back_cast(), *new sc_bv_base( b, 1 ), 2 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,T>
concat( bool a, const sc_proxy<T>& b )
{
    return sc_concref_r<sc_bv_base,T>
      ( *new sc_bv_base( a, 1 ), b.back_cast(), 1 );
}


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T>
inline
sc_concref_r<T,sc_lv_base>
operator , ( sc_proxy<T>& a, const char* b )
{
    return sc_concref_r<T,sc_lv_base>
      ( a.back_cast(), *new sc_lv_base( b ), 2 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,T>
operator , ( const char* a, sc_proxy<T>& b )
{
    return sc_concref_r<sc_lv_base,T>
        ( *new sc_lv_base( a ), b.back_cast(), 1 );
}

template <class T>
inline
sc_concref_r<T,sc_lv_base>
operator , ( sc_proxy<T>& a, const sc_logic& b )
{
    return sc_concref_r<T,sc_lv_base>
        ( a.back_cast(), *new sc_lv_base( b, 1 ), 2 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,T>
operator , ( const sc_logic& a, sc_proxy<T>& b )
{
    return sc_concref_r<sc_lv_base,T>
        ( *new sc_lv_base( a, 1 ), b.back_cast(), 1 );
}

template <class T>
inline
sc_concref_r<T,sc_bv_base>
operator , ( sc_proxy<T>& a, bool b )
{
    return sc_concref_r<T,sc_bv_base>
        ( a.back_cast(), *new sc_bv_base( b, 1 ), 2 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,T>
operator , ( bool a, sc_proxy<T>& b )
{
    return sc_concref_r<sc_bv_base,T>
        ( *new sc_bv_base( a, 1 ), b.back_cast(), 1 );
}


template <class T>
inline
sc_concref_r<T,sc_lv_base>
concat( sc_proxy<T>& a, const char* b )
{
    return sc_concref_r<T,sc_lv_base>
        ( a.back_cast(), *new sc_lv_base( b ), 2 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,T>
concat( const char* a, sc_proxy<T>& b )
{
    return sc_concref_r<sc_lv_base,T>
        ( *new sc_lv_base( a ), b.back_cast(), 1 );
}

template <class T>
inline
sc_concref_r<T,sc_lv_base>
concat( sc_proxy<T>& a, const sc_logic& b )
{
    return sc_concref_r<T,sc_lv_base>
        ( a.back_cast(), *new sc_lv_base( b, 1 ), 2 );
}

template <class T>
inline
sc_concref_r<sc_lv_base,T>
concat( const sc_logic& a, sc_proxy<T>& b )
{
    return sc_concref_r<sc_lv_base,T>
        ( *new sc_lv_base( a, 1 ), b.back_cast(), 1 );
}

template <class T>
inline
sc_concref_r<T,sc_bv_base>
concat( sc_proxy<T>& a, bool b )
{
    return sc_concref_r<T,sc_bv_base>
        ( a.back_cast(), *new sc_bv_base( b, 1 ), 2 );
}

template <class T>
inline
sc_concref_r<sc_bv_base,T>
concat( bool a, sc_proxy<T>& b )
{
    return sc_concref_r<sc_bv_base,T>
        ( *new sc_bv_base( a, 1 ), b.back_cast(), 1 );
}

#endif

} // namespace sc_dt


#endif
