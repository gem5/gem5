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

  sc_bit_proxies.h -- Proxy classes for vector data types.

  Original Author: Gene Bushuyev, Synopsys, Inc.

 CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_BIT_PROXIES_H
#define SC_BIT_PROXIES_H


#include "sysc/datatypes/bit/sc_bit_ids.h"
#include "sysc/datatypes/bit/sc_proxy.h"


namespace sc_dt
{

// classes defined in this module
template <class X> class sc_bitref_r;
template <class X> class sc_bitref;
template <class X> class sc_subref_r;
template <class X> class sc_subref;
template <class X, class Y> class sc_concref_r;
template <class X, class Y> class sc_concref;


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_bitref_r<T>
//
//  Proxy class for sc_proxy bit selection (r-value only).
// ----------------------------------------------------------------------------

template <class T>
class sc_bitref_r
{
    friend class sc_bv_base;
    friend class sc_lv_base;

public:

    // typedefs

    typedef typename T::traits_type          traits_type;
    typedef typename traits_type::bit_type   bit_type;

    // constructor

    sc_bitref_r( const T& obj_, int index_ )
	: m_obj( CCAST<T&>( obj_ ) ), m_index( index_ )
	{}


    // copy constructor

    sc_bitref_r( const sc_bitref_r<T>& a )
	: m_obj( a.m_obj ), m_index( a.m_index )
	{}

    // cloning

    sc_bitref_r<T>* clone() const
	{ return new sc_bitref_r<T>( *this ); }


    // bitwise operators and functions

    // bitwise complement

    const bit_type operator ~ () const
        { return bit_type( sc_logic::not_table[value()] ); }


    // implicit conversion to bit_type

    operator const bit_type() const
        { return bit_type( m_obj.get_bit( m_index ) ); }


    // explicit conversions

    sc_logic_value_t value() const
	{ return m_obj.get_bit( m_index ); }


    bool is_01() const
	{ return sc_logic( value() ).is_01(); }

    bool to_bool() const
	{ return sc_logic( value() ).to_bool(); }

    char to_char() const
	{ return sc_logic( value() ).to_char(); }


    // common methods

    int length() const
	{ return 1; }

    int size() const
	{ return ( (length() - 1) / SC_DIGIT_SIZE + 1 ); }

    sc_logic_value_t get_bit( int n ) const;

    sc_digit get_word( int i ) const;
    sc_digit get_cword( int i ) const;


    // other methods

    void print( ::std::ostream& os = ::std::cout ) const
	{ os << to_char(); }

protected:

    T&  m_obj;
    int m_index;

private:

    // disabled
    sc_bitref_r();
    sc_bitref_r<T>& operator = ( const sc_bitref_r<T>& );
};


// bitwise operators and functions

// bitwise and

template <class T1, class T2>
inline
const sc_logic
operator & ( const sc_bitref_r<T1>& a, const sc_bitref_r<T2>& b );


// bitwise or

template <class T1, class T2>
inline
const sc_logic
operator | ( const sc_bitref_r<T1>& a, const sc_bitref_r<T2>& b );


// bitwise xor

template <class T1, class T2>
inline
const sc_logic
operator ^ ( const sc_bitref_r<T1>& a, const sc_bitref_r<T2>& b );


// relational operators and functions

template <class T1, class T2>
inline
bool
operator == ( const sc_bitref_r<T1>& a, const sc_bitref_r<T2>& b );

template <class T1, class T2>
inline
bool
operator != ( const sc_bitref_r<T1>& a, const sc_bitref_r<T2>& b );


// r-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_bitref_r<T1>, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
operator , ( sc_bitref_r<T1>, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_bitref_r<T1>, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
operator , ( sc_bitref_r<T1>, const sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
operator , ( sc_bitref_r<T>, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
operator , ( const char*, sc_bitref_r<T> );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
operator , ( sc_bitref_r<T>, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
operator , ( const sc_logic&, sc_bitref_r<T> );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
operator , ( sc_bitref_r<T>, bool );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
operator , ( bool, sc_bitref_r<T> );


template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
concat( sc_bitref_r<T1>, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
concat( sc_bitref_r<T1>, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_bitref_r<T1>, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
concat( sc_bitref_r<T1>, const sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
concat( sc_bitref_r<T>, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
concat( const char*, sc_bitref_r<T> );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
concat( sc_bitref_r<T>, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
concat( const sc_logic&, sc_bitref_r<T> );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
concat( sc_bitref_r<T>, bool );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
concat( bool, sc_bitref_r<T> );


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_bitref_r<T1>, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_bitref<T1>, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
operator , ( sc_bitref_r<T1>, sc_subref<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
operator , ( sc_bitref<T1>, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_bitref_r<T1>, sc_concref<T2,T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_bitref<T1>, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
operator , ( sc_bitref<T1>, const sc_proxy<T2>& );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
operator , ( sc_bitref_r<T1>, sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
operator , ( sc_bitref<T>, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
operator , ( const char*, sc_bitref<T> );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
operator , ( sc_bitref<T>, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
operator , ( const sc_logic&, sc_bitref<T> );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
operator , ( sc_bitref<T>, bool );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
operator , ( bool, sc_bitref<T> );


template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
concat( sc_bitref_r<T1>, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
concat( sc_bitref<T1>, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
concat( sc_bitref_r<T1>, sc_subref<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
concat( sc_bitref<T1>, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_bitref_r<T1>, sc_concref<T2,T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_bitref<T1>, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
concat( sc_bitref<T1>, const sc_proxy<T2>& );

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
concat( sc_bitref_r<T1>, sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
concat( sc_bitref<T>, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
concat( const char*, sc_bitref<T> );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
concat( sc_bitref<T>, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
concat( const sc_logic&, sc_bitref<T> );

template <class T>
inline
sc_concref_r<sc_bitref_r<T>,sc_lv_base>
concat( sc_bitref<T>, bool );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_bitref_r<T> >
concat( bool, sc_bitref<T> );

#endif


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_bitref<X>
//
//  Proxy class for sc_proxy bit selection (r-value and l-value).
// ----------------------------------------------------------------------------

template <class X>
class sc_bitref
    : public sc_bitref_r<X>
{
    friend class sc_bv_base;
    friend class sc_lv_base;

public:

    // constructor

    sc_bitref( X& obj_, int index_ )
	: sc_bitref_r<X>( obj_, index_ )
	{}


    // copy constructor

    sc_bitref( const sc_bitref<X>& a )
	: sc_bitref_r<X>( a )
	{}


    // cloning

    sc_bitref<X>* clone() const
	{ return new sc_bitref<X>( *this ); }


    // assignment operators

    sc_bitref<X>& operator = ( const sc_bitref_r<X>& a );
    sc_bitref<X>& operator = ( const sc_bitref<X>& a );

    sc_bitref<X>& operator = ( const sc_logic& a )
	{ this->m_obj.set_bit( this->m_index, a.value() ); return *this; }

    sc_bitref<X>& operator = ( sc_logic_value_t v )
	{ *this = sc_logic( v ); return *this; }

    sc_bitref<X>& operator = ( bool a )
	{ *this = sc_logic( a ); return *this; }

    sc_bitref<X>& operator = ( char a )
	{ *this = sc_logic( a ); return *this; }

    sc_bitref<X>& operator = ( int a )
	{ *this = sc_logic( a ); return *this; }

    sc_bitref<X>& operator = ( const sc_bit& a )
	{ *this = sc_logic( a ); return *this; }


    // bitwise assignment operators

    sc_bitref<X>& operator &= ( const sc_bitref_r<X>& a );
    sc_bitref<X>& operator &= ( const sc_logic& a );

    sc_bitref<X>& operator &= ( sc_logic_value_t v )
	{ *this &= sc_logic( v ); return *this; }

    sc_bitref<X>& operator &= ( bool a )
	{ *this &= sc_logic( a ); return *this; }

    sc_bitref<X>& operator &= ( char a )
	{ *this &= sc_logic( a ); return *this; }

    sc_bitref<X>& operator &= ( int a )
	{ *this &= sc_logic( a ); return *this; }


    sc_bitref<X>& operator |= ( const sc_bitref_r<X>& a );
    sc_bitref<X>& operator |= ( const sc_logic& a );

    sc_bitref<X>& operator |= ( sc_logic_value_t v )
	{ *this |= sc_logic( v ); return *this; }

    sc_bitref<X>& operator |= ( bool a )
	{ *this |= sc_logic( a ); return *this; }

    sc_bitref<X>& operator |= ( char a )
	{ *this |= sc_logic( a ); return *this; }

    sc_bitref<X>& operator |= ( int a )
	{ *this |= sc_logic( a ); return *this; }


    sc_bitref<X>& operator ^= ( const sc_bitref_r<X>& a );
    sc_bitref<X>& operator ^= ( const sc_logic& a );

    sc_bitref<X>& operator ^= ( sc_logic_value_t v )
	{ *this ^= sc_logic( v ); return *this; }

    sc_bitref<X>& operator ^= ( bool a )
	{ *this ^= sc_logic( a ); return *this; }

    sc_bitref<X>& operator ^= ( char a )
	{ *this ^= sc_logic( a ); return *this; }

    sc_bitref<X>& operator ^= ( int a )
	{ *this ^= sc_logic( a ); return *this; }


    // bitwise operators and functions

    // bitwise complement

    sc_bitref<X>& b_not();


    // common methods

    void set_bit( int n, sc_logic_value_t value );

    void set_word( int i, sc_digit w );
    void set_cword( int i, sc_digit w );

    void clean_tail()
	{ this->m_obj.clean_tail(); }


    // other methods

    void scan( ::std::istream& is = ::std::cin );

private:

    // disabled
    sc_bitref();
};


// l-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,sc_bitref<T2> >
operator , ( sc_bitref<T1>, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,sc_subref<T2> >
operator , ( sc_bitref<T1>, sc_subref<T2> );

template <class T1, class T2, class T3>
inline
sc_concref<sc_bitref<T1>,sc_concref<T2,T3> >
operator , ( sc_bitref<T1>, sc_concref<T2,T3> );

template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,T2>
operator , ( sc_bitref<T1>, sc_proxy<T2>& );


template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,sc_bitref<T2> >
concat( sc_bitref<T1>, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,sc_subref<T2> >
concat( sc_bitref<T1>, sc_subref<T2> );

template <class T1, class T2, class T3>
inline
sc_concref<sc_bitref<T1>,sc_concref<T2,T3> >
concat( sc_bitref<T1>, sc_concref<T2,T3> );

template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,T2>
concat( sc_bitref<T1>, sc_proxy<T2>& );


template <class T>
::std::istream&
operator >> ( ::std::istream&, sc_bitref<T> );


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_subref_r<X>
//
//  Proxy class for sc_proxy part selection (r-value only).
// ----------------------------------------------------------------------------

template <class X>
class sc_subref_r
    : public sc_proxy<sc_subref_r<X> >
{
    void check_bounds();

public:

    // constructor

    sc_subref_r( const X& obj_, int hi_, int lo_ )
	: m_obj( CCAST<X&>( obj_ ) ), m_hi( hi_ ), m_lo( lo_ ), m_len( 0 )
	{ check_bounds(); }


    // copy constructor

    sc_subref_r( const sc_subref_r<X>& a )
	: m_obj( a.m_obj ), m_hi( a.m_hi ), m_lo( a.m_lo ), m_len( a.m_len )
	{}


    // cloning

    sc_subref_r<X>* clone() const
	{ return new sc_subref_r<X>( *this ); }


    // common methods

    int length() const
	{ return m_len; }

    int size() const
	{ return ( (length() - 1) / SC_DIGIT_SIZE + 1 ); }

    sc_logic_value_t get_bit( int n ) const;
    void set_bit( int n, sc_logic_value_t value );

    sc_digit get_word( int i )const;
    void set_word( int i, sc_digit w );

    sc_digit get_cword( int i ) const;
    void set_cword( int i, sc_digit w );

    void clean_tail()
	{ m_obj.clean_tail(); }


    // other methods

    bool is_01() const;

    bool reversed() const
	{ return m_lo > m_hi; }

protected:

    X&  m_obj;
    int m_hi;
    int m_lo;
    int m_len;

private:

    // disabled
    sc_subref_r();
    sc_subref_r<X>& operator = ( const sc_subref_r<X>& );
};


// r-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_subref_r<T1>, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
operator , ( sc_subref_r<T1>, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_subref_r<T1>, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
operator , ( sc_subref_r<T1>, const sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
operator , ( sc_subref_r<T>, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
operator , ( const char*, sc_subref_r<T> );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
operator , ( sc_subref_r<T>, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
operator , ( const sc_logic&, sc_subref_r<T> );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_bv_base>
operator , ( sc_subref_r<T>, bool );

template <class T>
inline
sc_concref_r<sc_bv_base,sc_subref_r<T> >
operator , ( bool, sc_subref_r<T> );


template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
concat( sc_subref_r<T1>, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
concat( sc_subref_r<T1>, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_subref_r<T1>, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
concat( sc_subref_r<T1>, const sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
concat( sc_subref_r<T>, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
concat( const char*, sc_subref_r<T> );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
concat( sc_subref_r<T>, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
concat( const sc_logic&, sc_subref_r<T> );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_bv_base>
concat( sc_subref_r<T>, bool );

template <class T>
inline
sc_concref_r<sc_bv_base,sc_subref_r<T> >
concat( bool, sc_subref_r<T> );


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_subref_r<T1>, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_subref<T1>, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
operator , ( sc_subref_r<T1>, sc_subref<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
operator , ( sc_subref<T1>, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_subref_r<T1>, sc_concref<T2,T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_subref<T1>, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
operator , ( sc_subref<T1>, const sc_proxy<T2>& );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
operator , ( sc_subref_r<T1>, sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
operator , ( sc_subref<T>, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
operator , ( const char*, sc_subref<T> );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
operator , ( sc_subref<T>, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
operator , ( const sc_logic&, sc_subref<T> );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_bv_base>
operator , ( sc_subref<T>, bool );

template <class T>
inline
sc_concref_r<sc_bv_base,sc_subref_r<T> >
operator , ( bool, sc_subref<T> );


template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
concat( sc_subref_r<T1>, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
concat( sc_subref<T1>, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
concat( sc_subref_r<T1>, sc_subref<T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
concat( sc_subref<T1>, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_subref_r<T1>, sc_concref<T2,T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_subref<T1>, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
concat( sc_subref<T1>, const sc_proxy<T2>& );

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
concat( sc_subref_r<T1>, sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
concat( sc_subref<T>, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
concat( const char*, sc_subref<T> );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_lv_base>
concat( sc_subref<T>, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,sc_subref_r<T> >
concat( const sc_logic&, sc_subref<T> );

template <class T>
inline
sc_concref_r<sc_subref_r<T>,sc_bv_base>
concat( sc_subref<T>, bool );

template <class T>
inline
sc_concref_r<sc_bv_base,sc_subref_r<T> >
concat( bool, sc_subref<T> );

#endif


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_subref<X>
//
//  Proxy class for sc_proxy part selection (r-value and l-value).
// ----------------------------------------------------------------------------

template <class X>
class sc_subref
    : public sc_subref_r<X>
{
public:

    // typedefs

    typedef sc_subref_r<X> base_type;


    // constructor

    sc_subref( X& obj_, int hi_, int lo_ )
	: sc_subref_r<X>( obj_, hi_, lo_ )
	{}


    // copy constructor

    sc_subref( const sc_subref<X>& a )
	: sc_subref_r<X>( a )
	{}


    // cloning

    sc_subref<X>* clone() const
	{ return new sc_subref<X>( *this ); }


    // assignment operators

    template <class Y>
    sc_subref<X>& operator = ( const sc_proxy<Y>& a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( const sc_subref_r<X>& a );
    sc_subref<X>& operator = ( const sc_subref<X>& a );

    sc_subref<X>& operator = ( const char* a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( const bool* a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( const sc_logic* a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( const sc_unsigned& a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( const sc_signed& a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( const sc_uint_base& a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( const sc_int_base& a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( unsigned long a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( long a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( unsigned int a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( int a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( uint64 a )
	{ base_type::assign_( a ); return *this; }

    sc_subref<X>& operator = ( int64 a )
	{ base_type::assign_( a ); return *this; }


    // other methods

    void scan( ::std::istream& = ::std::cin );

private:

    // disabled
    sc_subref();
};


// l-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,sc_bitref<T2> >
operator , ( sc_subref<T1>, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,sc_subref<T2> >
operator , ( sc_subref<T1>, sc_subref<T2> );

template <class T1, class T2, class T3>
inline
sc_concref<sc_subref<T1>,sc_concref<T2,T3> >
operator , ( sc_subref<T1>, sc_concref<T2,T3> );

template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,T2>
operator , ( sc_subref<T1>, sc_proxy<T2>& );


template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,sc_bitref<T2> >
concat( sc_subref<T1>, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,sc_subref<T2> >
concat( sc_subref<T1>, sc_subref<T2> );

template <class T1, class T2, class T3>
inline
sc_concref<sc_subref<T1>,sc_concref<T2,T3> >
concat( sc_subref<T1>, sc_concref<T2,T3> );

template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,T2>
concat( sc_subref<T1>, sc_proxy<T2>& );


template <class T>
inline
::std::istream&
operator >> ( ::std::istream&, sc_subref<T> );


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_concref_r<X,Y>
//
//  Proxy class for sc_proxy concatenation (r-value only).
// ----------------------------------------------------------------------------

template <class X, class Y>
class sc_concref_r
    : public sc_proxy<sc_concref_r<X,Y> >
{
public:

    // constructor

    sc_concref_r( const X& left_, const Y& right_, int delete_ = 0 )
	: m_left( CCAST<X&>( left_ ) ), m_right( CCAST<Y&>( right_ ) ),
	  m_delete( delete_ ), m_refs( *new int( 1 ) )
	{}


    // copy constructor

    sc_concref_r( const sc_concref_r<X,Y>& a )
	: m_left( a.m_left ), m_right( a.m_right ),
	  m_delete( a.m_delete ), m_refs( a.m_refs )
	{ ++ m_refs; }


    // destructor

    virtual ~sc_concref_r();


    // cloning

    sc_concref_r<X,Y>* clone() const
	{ return new sc_concref_r<X,Y>( *this ); }


    // common methods

    int length() const
	{ return ( m_left.length() + m_right.length() ); }

    int size() const
	{ return ( (length() - 1) / SC_DIGIT_SIZE + 1 ); }

    sc_logic_value_t get_bit( int n ) const;
    void set_bit( int n, sc_logic_value_t value );

    sc_digit get_word( int i ) const;
    void set_word( int i, sc_digit w );

    sc_digit get_cword( int i ) const;
    void set_cword( int i, sc_digit w );

    void clean_tail()
	{ m_left.clean_tail(); m_right.clean_tail(); }


    // other methods

    bool is_01() const
	{ return ( m_left.is_01() && m_right.is_01() ); }

protected:

    X&           m_left;
    Y&           m_right;
    mutable int  m_delete;
    int&         m_refs;

private:

    // disabled
    sc_concref_r();
    sc_concref_r<X,Y>& operator = ( const sc_concref_r<X,Y>& );
};


// r-value concatenation operators and functions

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
operator , ( sc_concref_r<T1,T2>, sc_bitref_r<T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
operator , ( sc_concref_r<T1,T2>, sc_subref_r<T3> );

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
operator , ( sc_concref_r<T1,T2>, sc_concref_r<T3,T4> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
operator , ( sc_concref_r<T1,T2>, const sc_proxy<T3>& );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
operator , ( sc_concref_r<T1,T2>, const char* );

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
operator , ( const char*, sc_concref_r<T1,T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
operator , ( sc_concref_r<T1,T2>, const sc_logic& );

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
operator , ( const sc_logic&, sc_concref_r<T1,T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
operator , ( sc_concref_r<T1,T2>, bool );

template <class T1, class T2>
inline
sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
operator , ( bool, sc_concref_r<T1,T2> );


template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
concat( sc_concref_r<T1,T2>, sc_bitref_r<T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
concat( sc_concref_r<T1,T2>, sc_subref_r<T3> );

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
concat( sc_concref_r<T1,T2>, sc_concref_r<T3,T4> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
concat( sc_concref_r<T1,T2>, const sc_proxy<T3>& );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
concat( sc_concref_r<T1,T2>, const char* );

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
concat( const char*, sc_concref_r<T1,T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
concat( sc_concref_r<T1,T2>, const sc_logic& );

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
concat( const sc_logic&, sc_concref_r<T1,T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
concat( sc_concref_r<T1,T2>, bool );

template <class T1, class T2>
inline
sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
concat( bool, sc_concref_r<T1,T2> );


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
operator , ( sc_concref_r<T1,T2>, sc_bitref<T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
operator , ( sc_concref<T1,T2>, sc_bitref_r<T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
operator , ( sc_concref_r<T1,T2>, sc_subref<T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
operator , ( sc_concref<T1,T2>, sc_subref_r<T3> );

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
operator , ( sc_concref_r<T1,T2>, sc_concref<T3,T4> );

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
operator , ( sc_concref<T1,T2>, sc_concref_r<T3,T4> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
operator , ( sc_concref<T1,T2>, const sc_proxy<T3>& );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
operator , ( sc_concref_r<T1,T2>, sc_proxy<T3>& );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
operator , ( sc_concref<T1,T2>, const char* );

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
operator , ( const char*, sc_concref<T1,T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
operator , ( sc_concref<T1,T2>, const sc_logic& );

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
operator , ( const sc_logic&, sc_concref<T1,T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
operator , ( sc_concref<T1,T2>, bool );

template <class T1, class T2>
inline
sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
operator , ( bool, sc_concref<T1,T2> );


template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
concat( sc_concref_r<T1,T2>, sc_bitref<T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
concat( sc_concref<T1,T2>, sc_bitref_r<T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
concat( sc_concref_r<T1,T2>, sc_subref<T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
concat( sc_concref<T1,T2>, sc_subref_r<T3> );

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
concat( sc_concref_r<T1,T2>, sc_concref<T3,T4> );

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
concat( sc_concref<T1,T2>, sc_concref_r<T3,T4> );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
concat( sc_concref<T1,T2>, const sc_proxy<T3>& );

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
concat( sc_concref_r<T1,T2>, sc_proxy<T3>& );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
concat( sc_concref<T1,T2>, const char* );

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
concat( const char*, sc_concref<T1,T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_lv_base>
concat( sc_concref<T1,T2>, const sc_logic& );

template <class T1, class T2>
inline
sc_concref_r<sc_lv_base,sc_concref_r<T1,T2> >
concat( const sc_logic&, sc_concref<T1,T2> );

template <class T1, class T2>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bv_base>
concat( sc_concref<T1,T2>, bool );

template <class T1, class T2>
inline
sc_concref_r<sc_bv_base,sc_concref_r<T1,T2> >
concat( bool, sc_concref<T1,T2> );

#endif


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_concref<X,Y>
//
//  Proxy class for sc_proxy concatenation (r-value and l-value).
// ----------------------------------------------------------------------------

template <class X, class Y>
class sc_concref
    : public sc_concref_r<X,Y>
{
public:

    // typedefs

    typedef sc_concref_r<X,Y> base_type;


    // constructor

    sc_concref( X& left_, Y& right_, int delete_ = 0 )
	: sc_concref_r<X,Y>( left_, right_, delete_ )
	{}


    // copy constructor

    sc_concref( const sc_concref<X,Y>& a )
	: sc_concref_r<X,Y>( a )
	{}


    // cloning

    sc_concref<X,Y>* clone() const
	{ return new sc_concref<X,Y>( *this ); }


    // assignment operators

    template <class Z>
    sc_concref<X,Y>& operator = ( const sc_proxy<Z>& a )
        { base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( const sc_concref<X,Y>& a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( const char* a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( const bool* a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( const sc_logic* a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( const sc_unsigned& a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( const sc_signed& a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( const sc_uint_base& a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( const sc_int_base& a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( unsigned long a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( long a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( unsigned int a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( int a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( uint64 a )
	{ base_type::assign_( a ); return *this; }

    sc_concref<X,Y>& operator = ( int64 a )
	{ base_type::assign_( a ); return *this; }


    // other methods

    void scan( ::std::istream& = ::std::cin );

private:

    // disabled
    sc_concref();
};


// l-value concatenation operators and functions

template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,sc_bitref<T3> >
operator , ( sc_concref<T1,T2>, sc_bitref<T3> );

template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,sc_subref<T3> >
operator , ( sc_concref<T1,T2>, sc_subref<T3> );

template <class T1, class T2, class T3, class T4>
inline
sc_concref<sc_concref<T1,T2>,sc_concref<T3,T4> >
operator , ( sc_concref<T1,T2>, sc_concref<T3,T4> );

template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,T3>
operator , ( sc_concref<T1,T2>, sc_proxy<T3>& );


template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,sc_bitref<T3> >
concat( sc_concref<T1,T2>, sc_bitref<T3> );

template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,sc_subref<T3> >
concat( sc_concref<T1,T2>, sc_subref<T3> );

template <class T1, class T2, class T3, class T4>
inline
sc_concref<sc_concref<T1,T2>,sc_concref<T3,T4> >
concat( sc_concref<T1,T2>, sc_concref<T3,T4> );

template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,T3>
concat( sc_concref<T1,T2>, sc_proxy<T3>& );


template <class T1, class T2>
inline
::std::istream&
operator >> ( ::std::istream&, sc_concref<T1,T2> );


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_proxy<T>
//
//  Base class template for bit/logic vector classes.
//  (Barton/Nackmann implementation)
// ----------------------------------------------------------------------------

// r-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
operator , ( const sc_proxy<T1>&, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
operator , ( const sc_proxy<T1>&, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
operator , ( const sc_proxy<T1>&, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
operator , ( const sc_proxy<T1>&, const sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<T,sc_lv_base>
operator , ( const sc_proxy<T>&, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,T>
operator , ( const char*, const sc_proxy<T>& );

template <class T>
inline
sc_concref_r<T,sc_lv_base>
operator , ( const sc_proxy<T>&, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,T>
operator , ( const sc_logic&, const sc_proxy<T>& );

template <class T>
inline
sc_concref_r<T,sc_bv_base>
operator , ( const sc_proxy<T>&, bool );

template <class T>
inline
sc_concref_r<sc_bv_base,T>
operator , ( bool, const sc_proxy<T>& );


template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
concat( const sc_proxy<T1>&, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
concat( const sc_proxy<T1>&, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
concat( const sc_proxy<T1>&, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
concat( const sc_proxy<T1>&, const sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<T,sc_lv_base>
concat( const sc_proxy<T>&, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,T>
concat( const char*, const sc_proxy<T>& );

template <class T>
inline
sc_concref_r<T,sc_lv_base>
concat( const sc_proxy<T>&, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,T>
concat( const sc_logic&, const sc_proxy<T>& );

template <class T>
inline
sc_concref_r<T,sc_bv_base>
concat( const sc_proxy<T>&, bool );

template <class T>
inline
sc_concref_r<sc_bv_base,T>
concat( bool, const sc_proxy<T>& );


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
operator , ( const sc_proxy<T1>&, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
operator , ( sc_proxy<T1>&, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
operator , ( const sc_proxy<T1>&, sc_subref<T2> );

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
operator , ( sc_proxy<T1>&, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
operator , ( const sc_proxy<T1>&, sc_concref<T2,T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
operator , ( sc_proxy<T1>&, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
operator , ( const sc_proxy<T1>&, sc_proxy<T2>& );

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
operator , ( sc_proxy<T1>&, const sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<T,sc_lv_base>
operator , ( sc_proxy<T>&, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,T>
operator , ( const char*, sc_proxy<T>& );

template <class T>
inline
sc_concref_r<T,sc_lv_base>
operator , ( sc_proxy<T>&, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,T>
operator , ( const sc_logic&, sc_proxy<T>& );

template <class T>
inline
sc_concref_r<T,sc_bv_base>
operator , ( sc_proxy<T>&, bool );

template <class T>
inline
sc_concref_r<sc_bv_base,T>
operator , ( bool, sc_proxy<T>& );


template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
concat( const sc_proxy<T1>&, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
concat( sc_proxy<T1>&, sc_bitref_r<T2> );

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
concat( const sc_proxy<T1>&, sc_subref<T2> );

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
concat( sc_proxy<T1>&, sc_subref_r<T2> );

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
concat( const sc_proxy<T1>&, sc_concref<T2,T3> );

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
concat( sc_proxy<T1>&, sc_concref_r<T2,T3> );

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
concat( const sc_proxy<T1>&, sc_proxy<T2>& );

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
concat( sc_proxy<T1>&, const sc_proxy<T2>& );

template <class T>
inline
sc_concref_r<T,sc_lv_base>
concat( sc_proxy<T>&, const char* );

template <class T>
inline
sc_concref_r<sc_lv_base,T>
concat( const char*, sc_proxy<T>& );

template <class T>
inline
sc_concref_r<T,sc_lv_base>
concat( sc_proxy<T>&, const sc_logic& );

template <class T>
inline
sc_concref_r<sc_lv_base,T>
concat( const sc_logic&, sc_proxy<T>& );

template <class T>
inline
sc_concref_r<T,sc_bv_base>
concat( sc_proxy<T>&, bool );

template <class T>
inline
sc_concref_r<sc_bv_base,T>
concat( bool, sc_proxy<T>& );

#endif


// l-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref<T1,sc_bitref<T2> >
operator , ( sc_proxy<T1>&, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref<T1,sc_subref<T2> >
operator , ( sc_proxy<T1>&, sc_subref<T2> );

template <class T1, class T2, class T3>
inline
sc_concref<T1,sc_concref<T2,T3> >
operator , ( sc_proxy<T1>&, sc_concref<T2,T3> );

template <class T1, class T2>
inline
sc_concref<T1,T2>
operator , ( sc_proxy<T1>&, sc_proxy<T2>& );


template <class T1, class T2>
inline
sc_concref<T1,sc_bitref<T2> >
concat( sc_proxy<T1>&, sc_bitref<T2> );

template <class T1, class T2>
inline
sc_concref<T1,sc_subref<T2> >
concat( sc_proxy<T1>&, sc_subref<T2> );

template <class T1, class T2, class T3>
inline
sc_concref<T1,sc_concref<T2,T3> >
concat( sc_proxy<T1>&, sc_concref<T2,T3> );

template <class T1, class T2>
inline
sc_concref<T1,T2>
concat( sc_proxy<T1>&, sc_proxy<T2>& );


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_bitref_r<T>
//
//  Proxy class for sc_proxy bit selection (r-value only).
// ----------------------------------------------------------------------------

// bitwise operators and functions

// bitwise and

template <class T1, class T2>
inline
const sc_logic
operator & ( const sc_bitref_r<T1>& a, const sc_bitref_r<T2>& b )
{
    return sc_logic( sc_logic::and_table[a.value()][b.value()] );
}


// bitwise or

template <class T1, class T2>
inline
const sc_logic
operator | ( const sc_bitref_r<T1>& a, const sc_bitref_r<T2>& b )
{
    return sc_logic( sc_logic::or_table[a.value()][b.value()] );
}


// bitwise xor

template <class T1, class T2>
inline
const sc_logic
operator ^ ( const sc_bitref_r<T1>& a, const sc_bitref_r<T2>& b )
{
    return sc_logic( sc_logic::xor_table[a.value()][b.value()] );
}


// relational operators and functions

template <class T1, class T2>
inline
bool
operator == ( const sc_bitref_r<T1>& a, const sc_bitref_r<T2>& b )
{
    return ( (int) a.value() == b.value() );
}

template <class T1, class T2>
inline
bool
operator != ( const sc_bitref_r<T1>& a, const sc_bitref_r<T2>& b )
{
    return ( (int) a.value() != b.value() );
}


// common methods

template <class T>
inline
sc_logic_value_t
sc_bitref_r<T>::get_bit( int n ) const
{
    if( n == 0 ) {
	return m_obj.get_bit( m_index );
    } else {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_ , 0 );
        // never reached
	return Log_0;
    }
}


template <class T>
inline
sc_digit
sc_bitref_r<T>::get_word( int n ) const
{
    if( n == 0 ) {
	return ( get_bit( n ) & SC_DIGIT_ONE );
    } else {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
	// never reached
	return 0;
    }
}

template <class T>
inline
sc_digit
sc_bitref_r<T>::get_cword( int n ) const
{
    if( n == 0 ) {
	return ( (get_bit( n ) & SC_DIGIT_TWO) >> 1 );
    } else {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
	// never reached
	return 0;
    }
}


// r-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_bitref_r<T1> a, sc_bitref_r<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
operator , ( sc_bitref_r<T1> a, sc_subref_r<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_bitref_r<T1> a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
operator , ( sc_bitref_r<T1> a, const sc_proxy<T2>& b )
{
    return sc_concref_r<sc_bitref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}


template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
concat( sc_bitref_r<T1> a, sc_bitref_r<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
concat( sc_bitref_r<T1> a, sc_subref_r<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_bitref_r<T1> a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
concat( sc_bitref_r<T1> a, const sc_proxy<T2>& b )
{
    return sc_concref_r<sc_bitref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_bitref_r<T1> a, sc_bitref<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_bitref<T1> a, sc_bitref_r<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
operator , ( sc_bitref_r<T1> a, sc_subref<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
operator , ( sc_bitref<T1> a, sc_subref_r<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_bitref_r<T1> a, sc_concref<T2,T3> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_bitref<T1> a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
operator , ( sc_bitref<T1> a, const sc_proxy<T2>& b )
{
    return sc_concref_r<sc_bitref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
operator , ( sc_bitref_r<T1> a, sc_proxy<T2>& b )
{
    return sc_concref_r<sc_bitref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}


template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
concat( sc_bitref_r<T1> a, sc_bitref<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >
concat( sc_bitref<T1> a, sc_bitref_r<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
concat( sc_bitref_r<T1> a, sc_subref<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >
concat( sc_bitref<T1> a, sc_subref_r<T2> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_bitref_r<T1> a, sc_concref<T2,T3> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_bitref<T1> a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<sc_bitref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
concat( sc_bitref<T1> a, const sc_proxy<T2>& b )
{
    return sc_concref_r<sc_bitref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_bitref_r<T1>,T2>
concat( sc_bitref_r<T1> a, sc_proxy<T2>& b )
{
    return sc_concref_r<sc_bitref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}

#endif


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_bitref<X>
//
//  Proxy class for sc_proxy bit selection (r-value and l-value).
// ----------------------------------------------------------------------------

// assignment operators

template <class X>
inline
sc_bitref<X>&
sc_bitref<X>::operator = ( const sc_bitref_r<X>& a )
{
    this->m_obj.set_bit( this->m_index, a.value() );
	return *this;
}

template <class X>
inline
sc_bitref<X>&
sc_bitref<X>::operator = ( const sc_bitref<X>& a )
{
    if( &a != this ) {
	this->m_obj.set_bit( this->m_index, a.value() );
    }
    return *this;
}


// bitwise assignment operators

template <class X>
inline
sc_bitref<X>&
sc_bitref<X>::operator &= ( const sc_bitref_r<X>& a )
{
    if( &a != this ) {
	this->m_obj.set_bit( this->m_index,
			     sc_logic::and_table[this->value()][a.value()] );
    }
    return *this;
}

template <class X>
inline
sc_bitref<X>&
sc_bitref<X>::operator &= ( const sc_logic& a )
{
    this->m_obj.set_bit( this->m_index,
			 sc_logic::and_table[this->value()][a.value()] );
    return *this;
}


template <class X>
inline
sc_bitref<X>&
sc_bitref<X>::operator |= ( const sc_bitref_r<X>& a )
{
    if( &a != this ) {
	this->m_obj.set_bit( this->m_index,
			     sc_logic::or_table[this->value()][a.value()] );
    }
    return *this;
}

template <class X>
inline
sc_bitref<X>&
sc_bitref<X>::operator |= ( const sc_logic& a )
{
    this->m_obj.set_bit( this->m_index,
			 sc_logic::or_table[this->value()][a.value()] );
    return *this;
}


template <class X>
inline
sc_bitref<X>&
sc_bitref<X>::operator ^= ( const sc_bitref_r<X>& a )
{
    if( &a != this ) {
	this->m_obj.set_bit( this->m_index,
			     sc_logic::xor_table[this->value()][a.value()] );
    }
    return *this;
}

template <class X>
inline
sc_bitref<X>&
sc_bitref<X>::operator ^= ( const sc_logic& a )
{
    this->m_obj.set_bit( this->m_index,
			 sc_logic::xor_table[this->value()][a.value()] );
    return *this;
}


// bitwise operators and functions

// bitwise complement

template <class X>
inline
sc_bitref<X>&
sc_bitref<X>::b_not()
{
    this->m_obj.set_bit( this->m_index,
			 sc_logic::not_table[this->value()] );
    return *this;
}


// common methods

template <class X>
inline
void
sc_bitref<X>::set_bit( int n, sc_logic_value_t value )
{
    if( n == 0 ) {
	this->m_obj.set_bit( this->m_index, value );
    } else {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
    }
}

template <class X>
inline
void
sc_bitref<X>::set_word( int n, sc_digit w )
{
    unsigned int bi = this->m_index % (8*sizeof(sc_digit));
    sc_digit     temp;
    unsigned int wi = this->m_index / (8*sizeof(sc_digit));
    if( n == 0 ) {
        temp = this->m_obj.get_word(wi);
        temp = (temp & ~(1 << bi)) | ((w&1) << bi);
        this->m_obj.set_word(wi, temp);
    } else {
        SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
    }

}

template <class X>
inline
void
sc_bitref<X>::set_cword( int n, sc_digit w )
{
    unsigned int bi = this->m_index % (8*sizeof(sc_digit));
    sc_digit     temp;
    unsigned int wi = this->m_index / (8*sizeof(sc_digit));
    if( n == 0 ) {
        temp = this->m_obj.get_cword(wi);
        temp = (temp & ~(1 << bi)) | ((w&1) << bi);
        this->m_obj.set_cword(wi, temp);
    } else {
        SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
    }
}

// other methods

template <class X>
inline
void
sc_bitref<X>::scan( ::std::istream& is )
{
    char c;
    is >> c;
    *this = c;
}


// l-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,sc_bitref<T2> >
operator , ( sc_bitref<T1> a, sc_bitref<T2> b )
{
    return sc_concref<sc_bitref<T1>,sc_bitref<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,sc_subref<T2> >
operator , ( sc_bitref<T1> a, sc_subref<T2> b )
{
    return sc_concref<sc_bitref<T1>,sc_subref<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref<sc_bitref<T1>,sc_concref<T2,T3> >
operator , ( sc_bitref<T1> a, sc_concref<T2,T3> b )
{
    return sc_concref<sc_bitref<T1>,sc_concref<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,T2>
operator , ( sc_bitref<T1> a, sc_proxy<T2>& b )
{
    return sc_concref<sc_bitref<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}


template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,sc_bitref<T2> >
concat( sc_bitref<T1> a, sc_bitref<T2> b )
{
    return sc_concref<sc_bitref<T1>,sc_bitref<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,sc_subref<T2> >
concat( sc_bitref<T1> a, sc_subref<T2> b )
{
    return sc_concref<sc_bitref<T1>,sc_subref<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref<sc_bitref<T1>,sc_concref<T2,T3> >
concat( sc_bitref<T1> a, sc_concref<T2,T3> b )
{
    return sc_concref<sc_bitref<T1>,sc_concref<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref<sc_bitref<T1>,T2>
concat( sc_bitref<T1> a, sc_proxy<T2>& b )
{
    return sc_concref<sc_bitref<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}


template <class X>
inline
::std::istream&
operator >> ( ::std::istream& is, sc_bitref<X> a )
{
    a.scan( is );
    return is;
}


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_subref_r<X>
//
//  Proxy class for sc_proxy part selection (r-value only).
// ----------------------------------------------------------------------------

template <class X>
inline
void
sc_subref_r<X>::check_bounds()
{
    int len = m_obj.length();
    if( m_hi < 0 || m_hi >= len || m_lo < 0 || m_lo >= len ) {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
    }
    if( reversed() ) {
	m_len = m_lo - m_hi + 1;
    } else {
	m_len = m_hi - m_lo + 1;
    }
}


// common methods

template <class X>
inline
sc_logic_value_t
sc_subref_r<X>::get_bit( int n ) const
{
    if( reversed() ) {
	return m_obj.get_bit( m_lo - n );
    } else {
	return m_obj.get_bit( m_lo + n );
    }
}

template <class X>
inline
void
sc_subref_r<X>::set_bit( int n, sc_logic_value_t value )
{
    if( reversed() ) {
	m_obj.set_bit( m_lo - n, value );
    } else {
	m_obj.set_bit( m_lo + n, value );
    }
}


template <class X>
inline
sc_digit
sc_subref_r<X>::get_word( int i ) const
{
    int n1 = 0;
    int n2 = 0;
    sc_digit result = 0;
    int k = 0;
    if( reversed() ) {
	n1 = m_lo - i * SC_DIGIT_SIZE;
	n2 = sc_max( n1 - SC_DIGIT_SIZE, m_hi - 1 );
	for( int n = n1; n > n2; n -- ) {
	    result |= (m_obj[n].value() & SC_DIGIT_ONE) << k ++;
	}
    } else {
	n1 = m_lo + i * SC_DIGIT_SIZE;
	n2 = sc_min( n1 + SC_DIGIT_SIZE, m_hi + 1 );
	for( int n = n1; n < n2; n ++ ) {
	    result |= (m_obj[n].value() & SC_DIGIT_ONE) << k ++;
	}
    }
    return result;
}

template <class X>
inline
void
sc_subref_r<X>::set_word( int i, sc_digit w )
{
    int n1 = 0;
    int n2 = 0;
    int k = 0;
    if( reversed() ) {
	n1 = m_lo - i * SC_DIGIT_SIZE;
	n2 = sc_max( n1 - SC_DIGIT_SIZE, m_hi - 1 );
	for( int n = n1; n > n2; n -- ) {
	    m_obj.set_bit( n, sc_logic_value_t( 
	                              ( (w >> k ++) & SC_DIGIT_ONE ) |
				      ( m_obj[n].value() & SC_DIGIT_TWO ) ) );
	}
    } else {
	n1 = m_lo + i * SC_DIGIT_SIZE;
	n2 = sc_min( n1 + SC_DIGIT_SIZE, m_hi + 1 );
	for( int n = n1; n < n2; n ++ ) {
	    m_obj.set_bit( n, sc_logic_value_t( 
	                                ( (w >> k ++) & SC_DIGIT_ONE ) |
					( m_obj[n].value() & SC_DIGIT_TWO ) ) );
	}
    }
}


template <class X>
inline
sc_digit
sc_subref_r<X>::get_cword( int i ) const
{
    int n1 = 0;
    int n2 = 0;
    sc_digit result = 0;
    int k = 0;
    if( reversed() ) {
	n1 = m_lo - i * SC_DIGIT_SIZE;
	n2 = sc_max( n1 - SC_DIGIT_SIZE, m_hi - 1 );
	for( int n = n1; n > n2; n -- ) {
	    result |= ((m_obj[n].value() & SC_DIGIT_TWO) >> 1) << k ++;
	}
    } else {
	n1 = m_lo + i * SC_DIGIT_SIZE;
	n2 = sc_min( n1 + SC_DIGIT_SIZE, m_hi + 1 );
	for( int n = n1; n < n2; n ++ ) {
	    result |= ((m_obj[n].value() & SC_DIGIT_TWO) >> 1) << k ++;
	}
    }
    return result;
}

template <class X>
inline
void
sc_subref_r<X>::set_cword( int i, sc_digit w )
{
    int n1 = 0;
    int n2 = 0;
    int k = 0;
    if( reversed() ) {
	n1 = m_lo - i * SC_DIGIT_SIZE;
	n2 = sc_max( n1 - SC_DIGIT_SIZE, m_hi - 1 );
	for( int n = n1; n > n2; n -- ) {
	    m_obj.set_bit( n, sc_logic_value_t( 
	                             ( ((w >> k ++) & SC_DIGIT_ONE) << 1 ) |
				     ( m_obj[n].value() & SC_DIGIT_ONE ) ) );
	}
    } else {
	n1 = m_lo + i * SC_DIGIT_SIZE;
	n2 = sc_min( n1 + SC_DIGIT_SIZE, m_hi + 1 );
	for( int n = n1; n < n2; n ++ ) {
	    m_obj.set_bit( n, sc_logic_value_t( 
	                                ( ((w >> k ++) & SC_DIGIT_ONE) << 1 ) |
					( m_obj[n].value() & SC_DIGIT_ONE ) ) );
	}
    }
}


// other methods

template <class X>
inline
bool
sc_subref_r<X>::is_01() const
{
    int sz = size();
    for( int i = 0; i < sz; ++ i ) {
	if( get_cword( i ) != SC_DIGIT_ZERO ) {
	    return false;
	}
    }
    return true;
}


// r-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_subref_r<T1> a, sc_bitref_r<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
operator , ( sc_subref_r<T1> a, sc_subref_r<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_subref_r<T1> a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
operator , ( sc_subref_r<T1> a, const sc_proxy<T2>& b )
{
    return sc_concref_r<sc_subref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}


template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
concat( sc_subref_r<T1> a, sc_bitref_r<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
concat( sc_subref_r<T1> a, sc_subref_r<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_subref_r<T1> a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
concat( sc_subref_r<T1> a, const sc_proxy<T2>& b )
{
    return sc_concref_r<sc_subref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_subref_r<T1> a, sc_bitref<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
operator , ( sc_subref<T1> a, sc_bitref_r<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
operator , ( sc_subref_r<T1> a, sc_subref<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
operator , ( sc_subref<T1> a, sc_subref_r<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_subref_r<T1> a, sc_concref<T2,T3> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
operator , ( sc_subref<T1> a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
operator , ( sc_subref<T1> a, const sc_proxy<T2>& b )
{
    return sc_concref_r<sc_subref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
operator , ( sc_subref_r<T1> a, sc_proxy<T2>& b )
{
    return sc_concref_r<sc_subref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}


template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
concat( sc_subref_r<T1> a, sc_bitref<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >
concat( sc_subref<T1> a, sc_bitref_r<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_bitref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
concat( sc_subref_r<T1> a, sc_subref<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >
concat( sc_subref<T1> a, sc_subref_r<T2> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_subref_r<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_subref_r<T1> a, sc_concref<T2,T3> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >
concat( sc_subref<T1> a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<sc_subref_r<T1>,sc_concref_r<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
concat( sc_subref<T1> a, const sc_proxy<T2>& b )
{
    return sc_concref_r<sc_subref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}

template <class T1, class T2>
inline
sc_concref_r<sc_subref_r<T1>,T2>
concat( sc_subref_r<T1> a, sc_proxy<T2>& b )
{
    return sc_concref_r<sc_subref_r<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}

#endif


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_subref<X>
//
//  Proxy class for sc_proxy part selection (r-value and l-value).
// ----------------------------------------------------------------------------

// assignment operators

// sc_subref<X>::operator = ( const sc_subref_r<X>& ) in sc_lv_base.h
// sc_subref<X>::operator = ( const sc_subref<X>& )   in sc_lv_base.h


// other methods

template <class T>
inline
void
sc_subref<T>::scan( ::std::istream& is )
{
    std::string s;
    is >> s;
    *this = s.c_str();
}


// l-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,sc_bitref<T2> >
operator , ( sc_subref<T1> a, sc_bitref<T2> b )
{
    return sc_concref<sc_subref<T1>,sc_bitref<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,sc_subref<T2> >
operator , ( sc_subref<T1> a, sc_subref<T2> b )
{
    return sc_concref<sc_subref<T1>,sc_subref<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref<sc_subref<T1>,sc_concref<T2,T3> >
operator , ( sc_subref<T1> a, sc_concref<T2,T3> b )
{
    return sc_concref<sc_subref<T1>,sc_concref<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,T2>
operator , ( sc_subref<T1> a, sc_proxy<T2>& b )
{
    return sc_concref<sc_subref<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}


template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,sc_bitref<T2> >
concat( sc_subref<T1> a, sc_bitref<T2> b )
{
    return sc_concref<sc_subref<T1>,sc_bitref<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,sc_subref<T2> >
concat( sc_subref<T1> a, sc_subref<T2> b )
{
    return sc_concref<sc_subref<T1>,sc_subref<T2> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref<sc_subref<T1>,sc_concref<T2,T3> >
concat( sc_subref<T1> a, sc_concref<T2,T3> b )
{
    return sc_concref<sc_subref<T1>,sc_concref<T2,T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2>
inline
sc_concref<sc_subref<T1>,T2>
concat( sc_subref<T1> a, sc_proxy<T2>& b )
{
    return sc_concref<sc_subref<T1>,T2>(
	*a.clone(), b.back_cast(), 1 );
}


template <class X>
inline
::std::istream&
operator >> ( ::std::istream& is, sc_subref<X> a )
{
    a.scan( is );
    return is;
}


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_concref_r<X,Y>
//
//  Proxy class for sc_proxy concatenation (r-value only).
// ----------------------------------------------------------------------------

// destructor

template <class X, class Y>
inline
sc_concref_r<X,Y>::~sc_concref_r()
{
    if( -- m_refs == 0 ) {
	delete &m_refs;
	if( m_delete == 0 ) {
	    return;
	}
	if( m_delete & 1 ) {
	    delete &m_left;
	}
	if( m_delete & 2 ) {
	    delete &m_right;
	}
    }
}


// common methods

template <class X, class Y>
inline
sc_logic_value_t
sc_concref_r<X,Y>::get_bit( int n ) const
{
    int r_len = m_right.length();
    if( n < r_len ) {
	return m_right.get_bit( n );
    } else if( n < r_len + m_left.length() ) {
	return m_left.get_bit( n - r_len );
    } else {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
	// never reached
	return Log_0;
    }
}

template <class X, class Y>
inline
void
sc_concref_r<X,Y>::set_bit( int n, sc_logic_value_t v )
{
    int r_len = m_right.length();
    if( n < r_len ) {
	m_right.set_bit( n, v );
    } else if( n < r_len + m_left.length() ) {
	m_left.set_bit( n - r_len, v );
    } else {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
    }
}


template <class X, class Y>
inline
sc_digit
sc_concref_r<X,Y>::get_word( int i ) const
{
    if( i < 0 || i >= size() ) {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
    }
    // 0 <= i < size()
    Y& r = m_right;
    int r_len = r.length();
    int border = r_len / SC_DIGIT_SIZE;
    if( i < border ) {
	return r.get_word( i );
    }
    // border <= i < size()
    X& l = m_left;
    int shift = r_len % SC_DIGIT_SIZE;
    int j = i - border;
    if( shift == 0 ) {
	return l.get_word( j );
    }
    // border <= i < size() && shift != 0
    int nshift = SC_DIGIT_SIZE - shift;
    if( i == border ) {
	sc_digit rl_mask = ~SC_DIGIT_ZERO >> nshift;
	return ( (r.get_word( i ) & rl_mask) | (l.get_word( 0 ) << shift) );
    }
    // border < i < size() && shift != 0
    if ( j < l.size() )
	return ( (l.get_word( j - 1 ) >> nshift) | (l.get_word( j ) << shift) );
    else
	return (l.get_word( j - 1 ) >> nshift);
}

template <class X, class Y>
inline
void
sc_concref_r<X,Y>::set_word( int i, sc_digit w )
{
    if( i < 0 || i >= size() ) {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
    }
    // 0 <= i < size()
    Y& r = m_right;
    int r_len = r.length();
    int border = r_len / SC_DIGIT_SIZE;
    if( i < border ) {
	r.set_word( i, w );
	return;
    }
    // border <= i < size()
    X& l = m_left;
    int shift = r_len % SC_DIGIT_SIZE;
    int j = i - border;
    if( shift == 0 ) {
	l.set_word( j, w );
	return;
    }
    // border <= i < size() && shift != 0
    int nshift = SC_DIGIT_SIZE - shift;
    sc_digit lh_mask = ~SC_DIGIT_ZERO << nshift;
    if( i == border ) {
	sc_digit rl_mask = ~SC_DIGIT_ZERO >> nshift;
	r.set_word( i, w & rl_mask );
	l.set_word( 0, (l.get_word( 0 ) & lh_mask) | (w >> shift) );
	return;
    }
    // border < i < size() && shift != 0
    sc_digit ll_mask = ~SC_DIGIT_ZERO >> shift;
    l.set_word( j - 1, (l.get_word( j - 1 ) & ll_mask) | (w << nshift) );
    if ( j < l.size() )
	l.set_word( j, (l.get_word( j ) & lh_mask) | (w >> shift) );
}


template <class X, class Y>
inline
sc_digit
sc_concref_r<X,Y>::get_cword( int i ) const
{
    if( i < 0 || i >= size() ) {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
    }
    // 0 <= i < size()
    Y& r = m_right;
    int r_len = r.length();
    int border = r_len / SC_DIGIT_SIZE;
    if( i < border ) {
	return r.get_cword( i );
    }
    // border <= i < size()
    X& l = m_left;
    int shift = r_len % SC_DIGIT_SIZE;
    int j = i - border;
    if( shift == 0 ) {
	return l.get_cword( j );
    }
    // border <= i < size() && shift != 0
    int nshift = SC_DIGIT_SIZE - shift;
    if( i == border ) {
	sc_digit rl_mask = ~SC_DIGIT_ZERO >> nshift;
	return ( (r.get_cword( i ) & rl_mask) | (l.get_cword( 0 ) << shift) );
    }
    // border < i < size() && shift != 0
    if ( j < l.size() )
	return ( (l.get_cword(j - 1) >> nshift) | (l.get_cword(j) << shift) );
    else
	return (l.get_cword( j - 1 ) >> nshift);
}

template <class X, class Y>
inline
void
sc_concref_r<X,Y>::set_cword( int i, sc_digit w )
{
    if( i < 0 || i >= size() ) {
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, 0 );
    }
    // 0 <= i < size()
    Y& r = m_right;
    int r_len = r.length();
    int border = r_len / SC_DIGIT_SIZE;
    if( i < border ) {
	r.set_cword( i, w );
	return;
    }
    // border <= i < size()
    X& l = m_left;
    int shift = r_len % SC_DIGIT_SIZE;
    int j = i - border;
    if( shift == 0 ) {
	l.set_cword( j, w );
	return;
    }
    // border <= i < size() && shift != 0
    int nshift = SC_DIGIT_SIZE - shift;
    sc_digit lh_mask = ~SC_DIGIT_ZERO << nshift;
    if( i == border ) {
	sc_digit rl_mask = ~SC_DIGIT_ZERO >> nshift;
	r.set_cword( i, w & rl_mask );
	l.set_cword( 0, (l.get_cword( 0 ) & lh_mask) | (w >> shift) );
	return;
    }
    // border < i < size() && shift != 0
    sc_digit ll_mask = ~SC_DIGIT_ZERO >> shift;
    l.set_cword( j - 1, (l.get_cword( j - 1 ) & ll_mask) | (w << nshift) );
    if ( j < l.size() )
	l.set_cword( j, (l.get_cword( j ) & lh_mask) | (w >> shift) );
}


// r-value concatenation operators and functions

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
operator , ( sc_concref_r<T1,T2> a, sc_bitref_r<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
operator , ( sc_concref_r<T1,T2> a, sc_subref_r<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
operator , ( sc_concref_r<T1,T2> a, sc_concref_r<T3,T4> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
operator , ( sc_concref_r<T1,T2> a, const sc_proxy<T3>& b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,T3>(
	*a.clone(), b.back_cast(), 1 );
}


template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
concat( sc_concref_r<T1,T2> a, sc_bitref_r<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
concat( sc_concref_r<T1,T2> a, sc_subref_r<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
concat( sc_concref_r<T1,T2> a, sc_concref_r<T3,T4> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
concat( sc_concref_r<T1,T2> a, const sc_proxy<T3>& b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,T3>(
	*a.clone(), b.back_cast(), 1 );
}


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
operator , ( sc_concref_r<T1,T2> a, sc_bitref<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
operator , ( sc_concref<T1,T2> a, sc_bitref_r<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
operator , ( sc_concref_r<T1,T2> a, sc_subref<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
operator , ( sc_concref<T1,T2> a, sc_subref_r<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
operator , ( sc_concref_r<T1,T2> a, sc_concref<T3,T4> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
operator , ( sc_concref<T1,T2> a, sc_concref_r<T3,T4> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
operator , ( sc_concref<T1,T2> a, const sc_proxy<T3>& b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,T3>(
	*a.clone(), b.back_cast(), 1 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
operator , ( sc_concref_r<T1,T2> a, sc_proxy<T3>& b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,T3>(
	*a.clone(), b.back_cast(), 1 );
}


template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
concat( sc_concref_r<T1,T2> a, sc_bitref<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >
concat( sc_concref<T1,T2> a, sc_bitref_r<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_bitref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
concat( sc_concref_r<T1,T2> a, sc_subref<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >
concat( sc_concref<T1,T2> a, sc_subref_r<T3> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_subref_r<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
concat( sc_concref_r<T1,T2> a, sc_concref<T3,T4> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3, class T4>
inline
sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >
concat( sc_concref<T1,T2> a, sc_concref_r<T3,T4> b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,sc_concref_r<T3,T4> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
concat( sc_concref<T1,T2> a, const sc_proxy<T3>& b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,T3>(
	*a.clone(), b.back_cast(), 1 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<sc_concref_r<T1,T2>,T3>
concat( sc_concref_r<T1,T2> a, sc_proxy<T3>& b )
{
    return sc_concref_r<sc_concref_r<T1,T2>,T3>(
	*a.clone(), b.back_cast(), 1 );
}

#endif


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_concref<X,Y>
//
//  Proxy class for sc_proxy concatenation (r-value and l-value).
// ----------------------------------------------------------------------------

// other methods

template <class T1, class T2>
inline
void
sc_concref<T1,T2>::scan( ::std::istream& is )
{
    std::string s;
    is >> s;
    *this = s.c_str();
}


// l-value concatenation operators and functions

template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,sc_bitref<T3> >
operator , ( sc_concref<T1,T2> a, sc_bitref<T3> b )
{
    return sc_concref<sc_concref<T1,T2>,sc_bitref<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,sc_subref<T3> >
operator , ( sc_concref<T1,T2> a, sc_subref<T3> b )
{
    return sc_concref<sc_concref<T1,T2>,sc_subref<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3, class T4>
inline
sc_concref<sc_concref<T1,T2>,sc_concref<T3,T4> >
operator , ( sc_concref<T1,T2> a, sc_concref<T3,T4> b )
{
    return sc_concref<sc_concref<T1,T2>,sc_concref<T3,T4> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,T3>
operator , ( sc_concref<T1,T2> a, sc_proxy<T3>& b )
{
    return sc_concref<sc_concref<T1,T2>,T3>(
	*a.clone(), b.back_cast(), 1 );
}


template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,sc_bitref<T3> >
concat( sc_concref<T1,T2> a, sc_bitref<T3> b )
{
    return sc_concref<sc_concref<T1,T2>,sc_bitref<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,sc_subref<T3> >
concat( sc_concref<T1,T2> a, sc_subref<T3> b )
{
    return sc_concref<sc_concref<T1,T2>,sc_subref<T3> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3, class T4>
inline
sc_concref<sc_concref<T1,T2>,sc_concref<T3,T4> >
concat( sc_concref<T1,T2> a, sc_concref<T3,T4> b )
{
    return sc_concref<sc_concref<T1,T2>,sc_concref<T3,T4> >(
	*a.clone(), *b.clone(), 3 );
}

template <class T1, class T2, class T3>
inline
sc_concref<sc_concref<T1,T2>,T3>
concat( sc_concref<T1,T2> a, sc_proxy<T3>& b )
{
    return sc_concref<sc_concref<T1,T2>,T3>(
	*a.clone(), b.back_cast(), 1 );
}


template <class X, class Y>
inline
::std::istream&
operator >> ( ::std::istream& is, sc_concref<X,Y> a )
{
    a.scan( is );
    return is;
}


// ----------------------------------------------------------------------------
//  CLASS TEMPLATE : sc_proxy<T>
//
//  Base class template for bit/logic vector classes.
//  (Barton/Nackmann implementation)
// ----------------------------------------------------------------------------

// r-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
operator , ( const sc_proxy<T1>& a, sc_bitref_r<T2> b )
{
    return sc_concref_r<T1,sc_bitref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
operator , ( const sc_proxy<T1>& a, sc_subref_r<T2> b )
{
    return sc_concref_r<T1,sc_subref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
operator , ( const sc_proxy<T1>& a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<T1,sc_concref_r<T2,T3> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
operator , ( const sc_proxy<T1>& a, const sc_proxy<T2>& b )
{
    return sc_concref_r<T1,T2>(
	a.back_cast(), b.back_cast() );
}


template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
concat( const sc_proxy<T1>& a, sc_bitref_r<T2> b )
{
    return sc_concref_r<T1,sc_bitref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
concat( const sc_proxy<T1>& a, sc_subref_r<T2> b )
{
    return sc_concref_r<T1,sc_subref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
concat( const sc_proxy<T1>& a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<T1,sc_concref_r<T2,T3> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
concat( const sc_proxy<T1>& a, const sc_proxy<T2>& b )
{
    return sc_concref_r<T1,T2>(
	a.back_cast(), b.back_cast() );
}


#ifdef SC_DT_MIXED_COMMA_OPERATORS

template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
operator , ( const sc_proxy<T1>& a, sc_bitref<T2> b )
{
    return sc_concref_r<T1,sc_bitref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
operator , ( sc_proxy<T1>& a, sc_bitref_r<T2> b )
{
    return sc_concref_r<T1,sc_bitref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
operator , ( const sc_proxy<T1>& a, sc_subref<T2> b )
{
    return sc_concref_r<T1,sc_subref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
operator , ( sc_proxy<T1>& a, sc_subref_r<T2> b )
{
    return sc_concref_r<T1,sc_subref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
operator , ( const sc_proxy<T1>& a, sc_concref<T2,T3> b )
{
    return sc_concref_r<T1,sc_concref_r<T2,T3> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
operator , ( sc_proxy<T1>& a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<T1,sc_concref_r<T2,T3> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
operator , ( const sc_proxy<T1>& a, sc_proxy<T2>& b )
{
    return sc_concref_r<T1,T2>(
	a.back_cast(), b.back_cast() );
}

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
operator , ( sc_proxy<T1>& a, const sc_proxy<T2>& b )
{
    return sc_concref_r<T1,T2>(
	a.back_cast(), b.back_cast() );
}


template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
concat( const sc_proxy<T1>& a, sc_bitref<T2> b )
{
    return sc_concref_r<T1,sc_bitref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,sc_bitref_r<T2> >
concat( sc_proxy<T1>& a, sc_bitref_r<T2> b )
{
    return sc_concref_r<T1,sc_bitref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
concat( const sc_proxy<T1>& a, sc_subref<T2> b )
{
    return sc_concref_r<T1,sc_subref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,sc_subref_r<T2> >
concat( sc_proxy<T1>& a, sc_subref_r<T2> b )
{
    return sc_concref_r<T1,sc_subref_r<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
concat( const sc_proxy<T1>& a, sc_concref<T2,T3> b )
{
    return sc_concref_r<T1,sc_concref_r<T2,T3> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2, class T3>
inline
sc_concref_r<T1,sc_concref_r<T2,T3> >
concat( sc_proxy<T1>& a, sc_concref_r<T2,T3> b )
{
    return sc_concref_r<T1,sc_concref_r<T2,T3> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
concat( const sc_proxy<T1>& a, sc_proxy<T2>& b )
{
    return sc_concref_r<T1,T2>(
	a.back_cast(), b.back_cast() );
}

template <class T1, class T2>
inline
sc_concref_r<T1,T2>
concat( sc_proxy<T1>& a, const sc_proxy<T2>& b )
{
    return sc_concref_r<T1,T2>(
	a.back_cast(), b.back_cast() );
}

#endif


// l-value concatenation operators and functions

template <class T1, class T2>
inline
sc_concref<T1,sc_bitref<T2> >
operator , ( sc_proxy<T1>& a, sc_bitref<T2> b )
{
    return sc_concref<T1,sc_bitref<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref<T1,sc_subref<T2> >
operator , ( sc_proxy<T1>& a, sc_subref<T2> b )
{
    return sc_concref<T1,sc_subref<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2, class T3>
inline
sc_concref<T1,sc_concref<T2,T3> >
operator , ( sc_proxy<T1>& a, sc_concref<T2,T3> b )
{
    return sc_concref<T1,sc_concref<T2,T3> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref<T1,T2>
operator , ( sc_proxy<T1>& a, sc_proxy<T2>& b )
{
    return sc_concref<T1,T2>(
	a.back_cast(), b.back_cast() );
}


template <class T1, class T2>
inline
sc_concref<T1,sc_bitref<T2> >
concat( sc_proxy<T1>& a, sc_bitref<T2> b )
{
    return sc_concref<T1,sc_bitref<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref<T1,sc_subref<T2> >
concat( sc_proxy<T1>& a, sc_subref<T2> b )
{
    return sc_concref<T1,sc_subref<T2> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2, class T3>
inline
sc_concref<T1,sc_concref<T2,T3> >
concat( sc_proxy<T1>& a, sc_concref<T2,T3> b )
{
    return sc_concref<T1,sc_concref<T2,T3> >(
	a.back_cast(), *b.clone(), 2 );
}

template <class T1, class T2>
inline
sc_concref<T1,T2>
concat( sc_proxy<T1>& a, sc_proxy<T2>& b )
{
    return sc_concref<T1,T2>(
	a.back_cast(), b.back_cast() );
}

} // namespace sc_dt

// $Log: sc_bit_proxies.h,v $
// Revision 1.10  2011/09/05 21:19:53  acg
//  Philipp A. Hartmann: added parentheses to expressions to eliminate
//  compiler warnings.
//
// Revision 1.9  2011/09/01 15:03:42  acg
//  Philipp A. Hartmann: add parentheses to eliminate compiler warnings.
//
// Revision 1.8  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.7  2011/08/24 22:05:40  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.6  2010/02/22 14:25:43  acg
//  Andy Goodrich: removed 'mutable' directive from references, since it
//  is not a legal C++ construct.
//
// Revision 1.5  2009/02/28 00:26:14  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.4  2007/03/14 17:48:37  acg
//  Andy Goodrich: fixed bug.
//
// Revision 1.3  2007/01/18 19:29:18  acg
//  Andy Goodrich: fixed bug in concatenations of bit selects on sc_lv and
//  sc_bv types. The offending code was in sc_bitref<X>::set_word and
//  sc_bitref<X>::get_word. These methods were not writing the bit they
//  represented, but rather writing an entire word whose index was the
//  index of the bit they represented. This not only did not write the
//  correct bit, but clobbered a word that might not even be in the
//  variable the reference was for.
//
// Revision 1.2  2007/01/17 22:45:08  acg
//  Andy Goodrich: fixed sc_bitref<X>::set_bit().
//
// Revision 1.1.1.1  2006/12/15 20:31:36  acg
// SystemC 2.2
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//


#endif
