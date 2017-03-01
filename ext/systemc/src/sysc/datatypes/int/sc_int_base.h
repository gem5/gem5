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

  sc_int_base.h -- A sc_int is a signed integer whose length is less than the
              machine's native integer length. We provide two implementations
              (i) sc_int with length between 1 - 64, and (ii) sc_int with
              length between 1 - 32. Implementation (i) is the default
              implementation, while implementation (ii) can be used only if
              the class library is compiled with -D_32BIT_. Unlike arbitrary
              precision, arithmetic and bitwise operations are performed
              using the native types (hence capped at 32/64 bits). The sc_int
              integer is useful when the user does not need arbitrary
              precision and the performance is superior to
              sc_bigint/sc_biguint.

  Original Author: Amit Rao, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Ali Dasdan, Synopsys, Inc.
  Description of Modification: - Resolved ambiguity with sc_(un)signed.
                               - Merged the code for 64- and 32-bit versions
                                 via the constants in sc_nbdefs.h.
                               - Eliminated redundant file inclusions.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_int_base.h,v $
// Revision 1.3  2011/08/24 22:05:45  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.2  2011/02/18 20:19:15  acg
//  Andy Goodrich: updating Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.4  2006/05/08 17:50:01  acg
//   Andy Goodrich: Added David Long's declarations for friend operators,
//   functions, and methods, to keep the Microsoft compiler happy.
//
// Revision 1.3  2006/01/13 18:49:31  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#ifndef SC_INT_BASE_H
#define SC_INT_BASE_H

#include "sysc/kernel/sc_object.h"
#include "sysc/datatypes/misc/sc_value_base.h"
#include "sysc/datatypes/int/sc_int_ids.h"
#include "sysc/datatypes/int/sc_length_param.h"
#include "sysc/datatypes/int/sc_nbdefs.h"
#include "sysc/datatypes/int/sc_uint_base.h"
#include "sysc/utils/sc_iostream.h"
#include "sysc/utils/sc_temporary.h"


namespace sc_dt
{

class sc_concatref;

// classes defined in this module
class sc_int_bitref_r;
class sc_int_bitref;
class sc_int_subref_r;
class sc_int_subref;
class sc_int_base;
class sc_signed_subref_r;
class sc_unsigned_subref_r;

// forward class declarations
class sc_bv_base;
class sc_lv_base;
class sc_signed;
class sc_unsigned;
class sc_fxval;
class sc_fxval_fast;
class sc_fxnum;
class sc_fxnum_fast;


extern const uint_type mask_int[SC_INTWIDTH][SC_INTWIDTH];

// friend operator declarations
    // relational operators

    inline bool operator == ( const sc_int_base& a, const sc_int_base& b );

    inline bool operator != ( const sc_int_base& a, const sc_int_base& b );

    inline bool operator <  ( const sc_int_base& a, const sc_int_base& b );

    inline bool operator <= ( const sc_int_base& a, const sc_int_base& b );

    inline bool operator >  ( const sc_int_base& a, const sc_int_base& b );

    inline bool operator >= ( const sc_int_base& a, const sc_int_base& b );


// ----------------------------------------------------------------------------
//  CLASS : sc_int_bitref_r
//
//  Proxy class for sc_int bit selection (r-value only).
// ----------------------------------------------------------------------------

class sc_int_bitref_r : public sc_value_base
{
    friend class sc_int_base;

protected:

    // constructor

    sc_int_bitref_r() : sc_value_base(), m_index(), m_obj_p()
        {}

    // initializer for sc_core::sc_vpool:

    void initialize( const sc_int_base* obj_p, int index_ )
    {
	m_obj_p = (sc_int_base*)obj_p;
	m_index = index_;
    }

public:

    // copy constructor

    sc_int_bitref_r( const sc_int_bitref_r& a ) :
        sc_value_base(a), m_index(a.m_index), m_obj_p(a.m_obj_p)
        {}

    // destructor

    virtual ~sc_int_bitref_r()
	{}

    // capacity

    int length() const
	{ return 1; }

#ifdef SC_DT_DEPRECATED
    int bitwidth() const
	{ return length(); }
#endif

    // concatenation support

    virtual int concat_length( bool *xz_present_p ) const
	{ if (xz_present_p) *xz_present_p = false; return 1; }
    virtual bool concat_get_ctrl( sc_digit* dst_p, int low_i ) const
        {
	    int bit_mask = 1 << (low_i % BITS_PER_DIGIT);
	    int word_i = low_i / BITS_PER_DIGIT;

	    dst_p[word_i] &= ~bit_mask;
	    return false;
	}
    virtual bool concat_get_data( sc_digit* dst_p, int low_i ) const
        {
	    bool non_zero;
	    int bit_mask = 1 << (low_i % BITS_PER_DIGIT);
	    int word_i = low_i / BITS_PER_DIGIT;

	    if ( operator uint64() )
	    {
		dst_p[word_i] |= bit_mask;
		non_zero = true;
	    }
	    else
	    {
		dst_p[word_i] &= ~bit_mask;
		non_zero = false;
	    }
	    return non_zero;
	}
    virtual uint64 concat_get_uint64() const
	{ return operator uint64(); }




    // implicit conversions

    operator uint64 () const;
    bool operator ! () const;
    bool operator ~ () const;


    // explicit conversions

    uint64 value() const
	{ return operator uint64(); }

    bool to_bool() const
	{ return operator uint64(); }


    // other methods

    void print( ::std::ostream& os = ::std::cout ) const
	{ os << to_bool(); }

protected:
    int          m_index;
    sc_int_base* m_obj_p;

private:

    // disabled
    sc_int_bitref_r& operator = ( const sc_int_bitref_r& );
};


inline
::std::ostream&
operator << ( ::std::ostream&, const sc_int_bitref_r& );


// ----------------------------------------------------------------------------
//  CLASS : sc_int_bitref
//
//  Proxy class for sc_int bit selection (r-value and l-value).
// ----------------------------------------------------------------------------

class sc_int_bitref
    : public sc_int_bitref_r
{
    friend class sc_int_base;
    friend class sc_core::sc_vpool<sc_int_bitref>;


    // constructor

    sc_int_bitref() : sc_int_bitref_r()
      {}


public:

    // copy constructor

    sc_int_bitref( const sc_int_bitref& a ) : sc_int_bitref_r( a )
      {}

    // assignment operators

    sc_int_bitref& operator = ( const sc_int_bitref_r& b );
    sc_int_bitref& operator = ( const sc_int_bitref& b );
    sc_int_bitref& operator = ( bool b );

    sc_int_bitref& operator &= ( bool b );
    sc_int_bitref& operator |= ( bool b );
    sc_int_bitref& operator ^= ( bool b );

	// concatenation methods

    virtual void concat_set(int64 src, int low_i);
    virtual void concat_set(const sc_signed& src, int low_i);
    virtual void concat_set(const sc_unsigned& src, int low_i);
    virtual void concat_set(uint64 src, int low_i);


    // other methods

    void scan( ::std::istream& is = ::std::cin );

public:
    static sc_core::sc_vpool<sc_int_bitref> m_pool;

};



inline
::std::istream&
operator >> ( ::std::istream&, sc_int_bitref& );


// ----------------------------------------------------------------------------
//  CLASS : sc_int_subref_r
//
//  Proxy class for sc_int part selection (r-value only).
// ----------------------------------------------------------------------------

class sc_int_subref_r : public sc_value_base
{
    friend class sc_int_base;
    friend class sc_int_signal;
    friend class sc_int_subref;

protected:

    // constructor

    sc_int_subref_r() : sc_value_base(), m_left(0), m_obj_p(0), m_right(0)
        {}

    // initializer for sc_core::sc_vpool:

    void initialize( const sc_int_base* obj_p, int left_i, int right_i )
    {
	m_obj_p = (sc_int_base*)obj_p;
	m_left = left_i;
	m_right = right_i;
    }


public:
    // copy constructor

    sc_int_subref_r( const sc_int_subref_r& a ) :
        sc_value_base(a), m_left( a.m_left ), m_obj_p( a.m_obj_p ), 
	m_right( a.m_right )
        {}

    // destructor

    virtual ~sc_int_subref_r()
	{}

    // capacity

    int length() const
        { return ( m_left - m_right + 1 ); }

#ifdef SC_DT_DEPRECATED
    int bitwidth() const
	{ return length(); }
#endif

    // concatenation support

    virtual int concat_length(bool* xz_present_p) const
	{ if ( xz_present_p ) *xz_present_p = false; return length(); }
    virtual bool concat_get_ctrl( sc_digit* dst_p, int low_i ) const;
    virtual bool concat_get_data( sc_digit* dst_p, int low_i ) const;
    virtual uint64 concat_get_uint64() const
    {
	int    len = length();
	uint64 val = operator uint_type();
	if ( len < 64 )
	    return (uint64)(val & ~((uint_type)-1 << len));
	else
	    return (uint64)val;
    }

    // reduce methods

    bool and_reduce() const;

    bool nand_reduce() const
	{ return ( ! and_reduce() ); }

    bool or_reduce() const;

    bool nor_reduce() const
	{ return ( ! or_reduce() ); }

    bool xor_reduce() const;

    bool xnor_reduce() const
	{ return ( ! xor_reduce() ); }


    // implicit conversion to uint_type

    operator uint_type () const;


    // explicit conversions

    uint_type value() const
	{ return operator uint_type(); }


    int           to_int() const;
    unsigned int  to_uint() const;
    long          to_long() const;
    unsigned long to_ulong() const;
    int64         to_int64() const;
    uint64        to_uint64() const;
    double        to_double() const;


    // explicit conversion to character string

    const std::string to_string( sc_numrep numrep = SC_DEC ) const;
    const std::string to_string( sc_numrep numrep, bool w_prefix ) const;


    // other methods

    void print( ::std::ostream& os = ::std::cout ) const
	{ os << to_string(sc_io_base(os,SC_DEC),sc_io_show_base(os)); }

protected:

    int          m_left;
    sc_int_base* m_obj_p;
    int          m_right;

private:
    const sc_int_subref_r& operator = ( const sc_int_subref_r& );
};



inline
::std::ostream&
operator << ( ::std::ostream&, const sc_int_subref_r& );


// ----------------------------------------------------------------------------
//  CLASS : sc_int_subref
//
//  Proxy class for sc_int part selection (r-value and l-value).
// ----------------------------------------------------------------------------

class sc_int_subref
    : public sc_int_subref_r
{
    friend class sc_int_base;
    friend class sc_core::sc_vpool<sc_int_subref>;


protected:

    // constructor
    sc_int_subref() : sc_int_subref_r()
        {}

public:

    // copy constructor

    sc_int_subref( const sc_int_subref& a ) : sc_int_subref_r( a )
        {}

    // assignment operators

    sc_int_subref& operator = ( int_type v );
    sc_int_subref& operator = ( const sc_int_base& a );

    sc_int_subref& operator = ( const sc_int_subref_r& a )
	{ return operator = ( a.operator uint_type() ); }

    sc_int_subref& operator = ( const sc_int_subref& a )
	{ return operator = ( a.operator uint_type() ); }

    template< class T >
    sc_int_subref& operator = ( const sc_generic_base<T>& a )
        { return operator = ( a->to_int64() ); }

    sc_int_subref& operator = ( const char* a );

    sc_int_subref& operator = ( unsigned long a )
	{ return operator = ( (int_type) a ); }

    sc_int_subref& operator = ( long a )
	{ return operator = ( (int_type) a ); }

    sc_int_subref& operator = ( unsigned int a )
	{ return operator = ( (int_type) a ); }

    sc_int_subref& operator = ( int a )
	{ return operator = ( (int_type) a ); }

    sc_int_subref& operator = ( uint64 a )
	{ return operator = ( (int_type) a ); }

    sc_int_subref& operator = ( double a )
	{ return operator = ( (int_type) a ); }

    sc_int_subref& operator = ( const sc_signed& );
    sc_int_subref& operator = ( const sc_unsigned& );
    sc_int_subref& operator = ( const sc_bv_base& );
    sc_int_subref& operator = ( const sc_lv_base& );

	// concatenation methods

    virtual void concat_set(int64 src, int low_i);
    virtual void concat_set(const sc_signed& src, int low_i);
    virtual void concat_set(const sc_unsigned& src, int low_i);
    virtual void concat_set(uint64 src, int low_i);

    // other methods

    void scan( ::std::istream& is = ::std::cin );

public:
    static sc_core::sc_vpool<sc_int_subref> m_pool;

};



inline
::std::istream&
operator >> ( ::std::istream&, sc_int_subref& );


// ----------------------------------------------------------------------------
//  CLASS : sc_int_base
//
//  Base class for sc_int.
// ----------------------------------------------------------------------------

class sc_int_base : public sc_value_base
{
    friend class sc_int_bitref_r;
    friend class sc_int_bitref;
    friend class sc_int_subref_r;
    friend class sc_int_subref;


    // support methods

    void invalid_length() const;
    void invalid_index( int i ) const;
    void invalid_range( int l, int r ) const;

    void check_length() const
	{ if( m_len <= 0 || m_len > SC_INTWIDTH ) { invalid_length(); } }

    void check_index( int i ) const
	{ if( i < 0 || i >= m_len ) { invalid_index( i ); } }

    void check_range( int l, int r ) const
	{ if( r < 0 || l >= m_len || l < r ) { invalid_range( l, r ); } }

    void check_value() const;

    void extend_sign()
	{
#ifdef DEBUG_SYSTEMC
	    check_value();
#endif
	    m_val = ( m_val << m_ulen >> m_ulen );
	}

public:

    // constructors

    explicit sc_int_base( int w = sc_length_param().len() )
	: m_val( 0 ), m_len( w ), m_ulen( SC_INTWIDTH - m_len )
	{ check_length(); }

    sc_int_base( int_type v, int w )
	: m_val( v ), m_len( w ), m_ulen( SC_INTWIDTH - m_len )
	{ check_length(); extend_sign(); }

    sc_int_base( const sc_int_base& a )
	: sc_value_base(a), m_val( a.m_val ), m_len( a.m_len ), 
	  m_ulen( a.m_ulen )
	{}

    explicit sc_int_base( const sc_int_subref_r& a )
        : m_val( a ), m_len( a.length() ), m_ulen( SC_INTWIDTH - m_len )
        { extend_sign(); }

    template< class T >
    explicit sc_int_base( const sc_generic_base<T>& a ) :
        m_val( a->to_int64() ), m_len( a->length() ),
	m_ulen( SC_INTWIDTH - m_len )
	{ check_length(); extend_sign(); }

    explicit sc_int_base( const sc_signed& a );
    explicit sc_int_base( const sc_unsigned& a );
    explicit sc_int_base( const sc_bv_base& v );
    explicit sc_int_base( const sc_lv_base& v );
    explicit sc_int_base( const sc_uint_subref_r& v );
    explicit sc_int_base( const sc_signed_subref_r& v );
    explicit sc_int_base( const sc_unsigned_subref_r& v );



    // destructor

    virtual ~sc_int_base()
	{}

    // assignment operators

    sc_int_base& operator = ( int_type v )
	{ m_val = v; extend_sign(); return *this; }

    sc_int_base& operator = ( const sc_int_base& a )
	{ m_val = a.m_val; extend_sign(); return *this; }

    sc_int_base& operator = ( const sc_int_subref_r& a )
        { m_val = a; extend_sign(); return *this; }

    template<class T>
    sc_int_base& operator = ( const sc_generic_base<T>& a )
        { m_val = a->to_int64(); extend_sign(); return *this; }

    sc_int_base& operator = ( const sc_signed& a );
    sc_int_base& operator = ( const sc_unsigned& a );

#ifdef SC_INCLUDE_FX
    sc_int_base& operator = ( const sc_fxval& a );
    sc_int_base& operator = ( const sc_fxval_fast& a );
    sc_int_base& operator = ( const sc_fxnum& a );
    sc_int_base& operator = ( const sc_fxnum_fast& a );
#endif

    sc_int_base& operator = ( const sc_bv_base& a );
    sc_int_base& operator = ( const sc_lv_base& a );

    sc_int_base& operator = ( const char* a );

    sc_int_base& operator = ( unsigned long a )
	{ m_val = a; extend_sign(); return *this; }

    sc_int_base& operator = ( long a )
	{ m_val = a; extend_sign(); return *this; }

    sc_int_base& operator = ( unsigned int a )
	{ m_val = a; extend_sign(); return *this; }

    sc_int_base& operator = ( int a )
	{ m_val = a; extend_sign(); return *this; }

    sc_int_base& operator = ( uint64 a )
	{ m_val = a; extend_sign(); return *this; }

    sc_int_base& operator = ( double a )
	{ m_val = (int_type) a; extend_sign(); return *this; }


    // arithmetic assignment operators

    sc_int_base& operator += ( int_type v )
	{ m_val += v; extend_sign(); return *this; }

    sc_int_base& operator -= ( int_type v )
	{ m_val -= v; extend_sign(); return *this; }

    sc_int_base& operator *= ( int_type v )
	{ m_val *= v; extend_sign(); return *this; }

    sc_int_base& operator /= ( int_type v )
	{ m_val /= v; extend_sign(); return *this; }

    sc_int_base& operator %= ( int_type v )
	{ m_val %= v; extend_sign(); return *this; }


    // bitwise assignment operators

    sc_int_base& operator &= ( int_type v )
	{ m_val &= v; extend_sign(); return *this; }

    sc_int_base& operator |= ( int_type v )
	{ m_val |= v; extend_sign(); return *this; }

    sc_int_base& operator ^= ( int_type v )
	{ m_val ^= v; extend_sign(); return *this; }


    sc_int_base& operator <<= ( int_type v )
	{ m_val <<= v; extend_sign(); return *this; }

    sc_int_base& operator >>= ( int_type v )
	{ m_val >>= v; /* no sign extension needed */ return *this; }


    // prefix and postfix increment and decrement operators

    sc_int_base& operator ++ () // prefix
	{ ++ m_val; extend_sign(); return *this; }

    const sc_int_base operator ++ ( int ) // postfix
	{ sc_int_base tmp( *this ); ++ m_val; extend_sign(); return tmp; }

    sc_int_base& operator -- () // prefix
	{ -- m_val; extend_sign(); return *this; }

    const sc_int_base operator -- ( int ) // postfix
	{ sc_int_base tmp( *this ); -- m_val; extend_sign(); return tmp; }


    // relational operators

    friend bool operator == ( const sc_int_base& a, const sc_int_base& b )
	{ return a.m_val == b.m_val; }

    friend bool operator != ( const sc_int_base& a, const sc_int_base& b )
	{ return a.m_val != b.m_val; }

    friend bool operator <  ( const sc_int_base& a, const sc_int_base& b )
	{ return a.m_val < b.m_val; }

    friend bool operator <= ( const sc_int_base& a, const sc_int_base& b )
	{ return a.m_val <= b.m_val; }

    friend bool operator >  ( const sc_int_base& a, const sc_int_base& b )
	{ return a.m_val > b.m_val; }

    friend bool operator >= ( const sc_int_base& a, const sc_int_base& b )
	{ return a.m_val >= b.m_val; }


    // bit selection

    sc_int_bitref&         operator [] ( int i );
    const sc_int_bitref_r& operator [] ( int i ) const;

    sc_int_bitref&         bit( int i );
    const sc_int_bitref_r& bit( int i ) const;


    // part selection

    sc_int_subref&         operator () ( int left, int right );
    const sc_int_subref_r& operator () ( int left, int right ) const;

    sc_int_subref&         range( int left, int right );
    const sc_int_subref_r& range( int left, int right ) const;


    // bit access, without bounds checking or sign extension

    bool test( int i ) const
	{ return ( 0 != (m_val & (UINT_ONE << i)) ); }

    void set( int i )
	{ m_val |= (UINT_ONE << i); }

    void set( int i, bool v )
	{ v ? m_val |= (UINT_ONE << i) : m_val &= ~(UINT_ONE << i); }


    // capacity

    int length() const
	{ return m_len; }

#ifdef SC_DT_DEPRECATED
    int bitwidth() const
	{ return length(); }
#endif

    // concatenation support

    virtual int concat_length(bool* xz_present_p) const
	{ if ( xz_present_p ) *xz_present_p = false; return length(); }
    virtual bool concat_get_ctrl( sc_digit* dst_p, int low_i ) const;
    virtual bool concat_get_data( sc_digit* dst_p, int low_i ) const;
    virtual uint64 concat_get_uint64() const
	{
	    if ( m_len < 64 )
		return (uint64)(m_val & ~((uint_type)-1 << m_len));
	    else
		return (uint64)m_val;
	}
    virtual void concat_set(int64 src, int low_i);
    virtual void concat_set(const sc_signed& src, int low_i);
    virtual void concat_set(const sc_unsigned& src, int low_i);
    virtual void concat_set(uint64 src, int low_i);


    // reduce methods

    bool and_reduce() const;

    bool nand_reduce() const
	{ return ( ! and_reduce() ); }

    bool or_reduce() const;

    bool nor_reduce() const
	{ return ( ! or_reduce() ); }

    bool xor_reduce() const;

    bool xnor_reduce() const
	{ return ( ! xor_reduce() ); }


    // implicit conversion to int_type

    operator int_type() const
	{ return m_val; }


    // explicit conversions

    int_type value() const
	{ return operator int_type(); }


    int to_int() const
	{ return (int) m_val; }

    unsigned int to_uint() const
	{ return (unsigned int) m_val; }

    long to_long() const
	{ return (long) m_val; }

    unsigned long to_ulong() const
	{ return (unsigned long) m_val; }

    int64 to_int64() const
	{ return (int64) m_val; }

    uint64 to_uint64() const
	{ return (uint64) m_val; }

    double to_double() const
	{ return (double) m_val; }


#ifndef _32BIT_
    long long_low() const
	{ return (long) (m_val & UINT64_32ONES); }

    long long_high() const
	{ return (long) ((m_val >> 32) & UINT64_32ONES); }
#endif


    // explicit conversion to character string

    const std::string to_string( sc_numrep numrep = SC_DEC ) const;
    const std::string to_string( sc_numrep numrep, bool w_prefix ) const;


    // other methods

    void print( ::std::ostream& os = ::std::cout ) const
	{ os << to_string(sc_io_base(os,SC_DEC),sc_io_show_base(os)); }

    void scan( ::std::istream& is = ::std::cin );

protected:

    int_type m_val;   // value
    int      m_len;   // length
    int      m_ulen;  // unused length
};



inline
::std::ostream&
operator << ( ::std::ostream&, const sc_int_base& );

inline
::std::istream&
operator >> ( ::std::istream&, sc_int_base& );



// ----------------------------------------------------------------------------
//  CLASS : sc_int_bitref_r
//
//  Proxy class for sc_int bit selection (r-value only).
// ----------------------------------------------------------------------------

// implicit conversion to uint64

inline
sc_int_bitref_r::operator uint64 () const
{
    return m_obj_p->test( m_index );
}

inline
bool
sc_int_bitref_r::operator ! () const
{
    return ! m_obj_p->test( m_index );
}

inline
bool
sc_int_bitref_r::operator ~ () const
{
    return ! m_obj_p->test( m_index );
}



inline
::std::ostream&
operator << ( ::std::ostream& os, const sc_int_bitref_r& a )
{
    a.print( os );
    return os;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_int_bitref
//
//  Proxy class for sc_int bit selection (r-value and l-value).
// ----------------------------------------------------------------------------

// assignment operators

inline
sc_int_bitref&
sc_int_bitref::operator = ( const sc_int_bitref_r& b )
{
    m_obj_p->set( m_index, (bool) b );
    m_obj_p->extend_sign();
    return *this;
}

inline
sc_int_bitref&
sc_int_bitref::operator = ( const sc_int_bitref& b )
{
    m_obj_p->set( m_index, (bool) b );
    m_obj_p->extend_sign();
    return *this;
}

inline
sc_int_bitref&
sc_int_bitref::operator = ( bool b )
{
    m_obj_p->set( m_index, b );
    m_obj_p->extend_sign();
    return *this;
}


inline
sc_int_bitref&
sc_int_bitref::operator &= ( bool b )
{
    if( ! b ) {
	m_obj_p->set( m_index, b );
	m_obj_p->extend_sign();
    }
    return *this;
}

inline
sc_int_bitref&
sc_int_bitref::operator |= ( bool b )
{
    if( b ) {
	m_obj_p->set( m_index, b );
	m_obj_p->extend_sign();
    }
    return *this;
}

inline
sc_int_bitref&
sc_int_bitref::operator ^= ( bool b )
{
    if( b ) {
	m_obj_p->m_val ^= (UINT_ONE << m_index);
	m_obj_p->extend_sign();
    }
    return *this;
}



inline
::std::istream&
operator >> ( ::std::istream& is, sc_int_bitref& a )
{
    a.scan( is );
    return is;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_int_subref_r
//
//  Proxy class for sc_int part selection (r-value only).
// ----------------------------------------------------------------------------

// implicit conversion to int_type

inline
sc_int_subref_r::operator uint_type() const
{
    uint_type /*int_type*/ val = m_obj_p->m_val;
    int uleft = SC_INTWIDTH - (m_left + 1);
    int uright = uleft + m_right;
    return ( val << uleft >> uright );
}


// reduce methods

inline
bool
sc_int_subref_r::and_reduce() const
{
    sc_int_base a( *this );
    return a.and_reduce();
}

inline
bool
sc_int_subref_r::or_reduce() const
{
    sc_int_base a( *this );
    return a.or_reduce();
}

inline
bool
sc_int_subref_r::xor_reduce() const
{
    sc_int_base a( *this );
    return a.xor_reduce();
}


// explicit conversions

inline
int
sc_int_subref_r::to_int() const
{
	int result = static_cast<int>(operator uint_type());
	return result;
}

inline
unsigned int
sc_int_subref_r::to_uint() const
{
	unsigned int result = static_cast<unsigned int>(operator uint_type());
	return result;
}

inline
long
sc_int_subref_r::to_long() const
{
	long result = static_cast<long>(operator uint_type());
	return result;
}

inline
unsigned long
sc_int_subref_r::to_ulong() const
{
	unsigned long result = static_cast<unsigned long>(operator uint_type());
	return result;
}

inline
int64
sc_int_subref_r::to_int64() const
{
	int64 result = operator uint_type();
	return result;
}

inline
uint64
sc_int_subref_r::to_uint64() const
{
	uint64 result = operator uint_type();
	return result;
}

inline
double
sc_int_subref_r::to_double() const
{
	double result = static_cast<double>(operator uint_type());
	return result;
}


// explicit conversion to character string

inline
const std::string
sc_int_subref_r::to_string( sc_numrep numrep ) const
{
	sc_uint_base a(length());
    a = operator uint_type();
    return a.to_string( numrep );
}

inline
const std::string
sc_int_subref_r::to_string( sc_numrep numrep, bool w_prefix ) const
{
	sc_uint_base a(length());
    a = operator uint_type();
    return a.to_string( numrep, w_prefix );
}


// functional notation for the reduce methods

inline
bool
and_reduce( const sc_int_subref_r& a )
{
    return a.and_reduce();
}

inline
bool
nand_reduce( const sc_int_subref_r& a )
{
    return a.nand_reduce();
}

inline
bool
or_reduce( const sc_int_subref_r& a )
{
    return a.or_reduce();
}

inline
bool
nor_reduce( const sc_int_subref_r& a )
{
    return a.nor_reduce();
}

inline
bool
xor_reduce( const sc_int_subref_r& a )
{
    return a.xor_reduce();
}

inline
bool
xnor_reduce( const sc_int_subref_r& a )
{
    return a.xnor_reduce();
}



inline
::std::ostream&
operator << ( ::std::ostream& os, const sc_int_subref_r& a )
{
    a.print( os );
    return os;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_int_subref
//
//  Proxy class for sc_int part selection (r-value and l-value).
// ----------------------------------------------------------------------------

// assignment operators

inline
sc_int_subref&
sc_int_subref::operator = ( const sc_int_base& a )
{
    return operator = ( a.operator int_type() );
}

inline
sc_int_subref&
sc_int_subref::operator = ( const char* a )
{
    sc_int_base aa( length() );
    return ( *this = aa = a );
}



inline
::std::istream&
operator >> ( ::std::istream& is, sc_int_subref& a )
{
    a.scan( is );
    return is;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_int_base
//
//  Base class for sc_int.
// ----------------------------------------------------------------------------

// bit selection

inline
sc_int_bitref&
sc_int_base::operator [] ( int i )
{
    check_index( i );
    sc_int_bitref* result_p = sc_int_bitref::m_pool.allocate();
    result_p->initialize(this, i);
    return *result_p;
}

inline
const sc_int_bitref_r&
sc_int_base::operator [] ( int i ) const
{
    check_index( i );
    sc_int_bitref* result_p = sc_int_bitref::m_pool.allocate();
    result_p->initialize(this, i);
    return *result_p;
}


inline
sc_int_bitref&
sc_int_base::bit( int i )
{
    check_index( i );
    sc_int_bitref* result_p = sc_int_bitref::m_pool.allocate();
    result_p->initialize(this, i);
    return *result_p;
}

inline
const sc_int_bitref_r&
sc_int_base::bit( int i ) const
{
    check_index( i );
    sc_int_bitref* result_p = sc_int_bitref::m_pool.allocate();
    result_p->initialize(this, i);
    return *result_p;
}


// part selection

inline
sc_int_subref&
sc_int_base::operator () ( int left, int right )
{
    check_range( left, right );
    sc_int_subref* result_p = sc_int_subref::m_pool.allocate();
    result_p->initialize(this, left, right);
    return *result_p;
}

inline
const sc_int_subref_r&
sc_int_base::operator () ( int left, int right ) const
{
    check_range( left, right );
    sc_int_subref* result_p = sc_int_subref::m_pool.allocate();
    result_p->initialize(this, left, right);
    return *result_p;
}


inline
sc_int_subref&
sc_int_base::range( int left, int right )
{
    check_range( left, right );
    sc_int_subref* result_p = sc_int_subref::m_pool.allocate();
    result_p->initialize(this, left, right);
    return *result_p;
}

inline
const sc_int_subref_r&
sc_int_base::range( int left, int right ) const
{
    check_range( left, right );
    sc_int_subref* result_p = sc_int_subref::m_pool.allocate();
    result_p->initialize(this, left, right);
    return *result_p;
}


// functional notation for the reduce methods

inline
bool
and_reduce( const sc_int_base& a )
{
    return a.and_reduce();
}

inline
bool
nand_reduce( const sc_int_base& a )
{
    return a.nand_reduce();
}

inline
bool
or_reduce( const sc_int_base& a )
{
    return a.or_reduce();
}

inline
bool
nor_reduce( const sc_int_base& a )
{
    return a.nor_reduce();
}

inline
bool
xor_reduce( const sc_int_base& a )
{
    return a.xor_reduce();
}

inline
bool
xnor_reduce( const sc_int_base& a )
{
    return a.xnor_reduce();
}



inline
::std::ostream&
operator << ( ::std::ostream& os, const sc_int_base& a )
{
    a.print( os );
    return os;
}

inline
::std::istream&
operator >> ( ::std::istream& is, sc_int_base& a )
{
    a.scan( is );
    return is;
}

} // namespace sc_dt


#endif

// Taf!
