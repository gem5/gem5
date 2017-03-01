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

  scfx_rep.h - 

  Original Author: Robert Graulich, Synopsys, Inc.
                   Martin Janssen,  Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: scfx_rep.h,v $
// Revision 1.6  2011/08/24 22:05:43  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.5  2011/07/25 10:20:29  acg
//  Andy Goodrich: check in aftermath of call to automake.
//
// Revision 1.4  2010/12/07 20:09:08  acg
// Andy Goodrich: Philipp Hartmann's constructor disambiguation fix
//
// Revision 1.3  2010/08/03 15:54:52  acg
//  Andy Goodrich: formatting.
//
// Revision 1.2  2010/03/15 18:29:01  acg
//  Andy Goodrich: Moved default argument specifications from friend
//  declarations to the actual function signatures.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.4  2006/03/13 20:24:27  acg
//  Andy Goodrich: Addition of function declarations, e.g., neg_scfx_rep(),
//  to keep gcc 4.x happy.
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef SCFX_REP_H
#define SCFX_REP_H


#include <climits>

#include "sysc/datatypes/fx/scfx_mant.h"
#include "sysc/datatypes/fx/scfx_params.h"
#include "sysc/datatypes/fx/scfx_string.h"


namespace sc_dt
{

// classes defined in this module
class scfx_index;
class scfx_rep;

// forward class declarations
class sc_bv_base;
class sc_signed;
class sc_unsigned;

// function declarations
void multiply( scfx_rep&, const scfx_rep&, const scfx_rep&, 
	       int max_wl = SC_DEFAULT_MAX_WL_ );
scfx_rep*  neg_scfx_rep( const scfx_rep& );
scfx_rep*  mult_scfx_rep( const scfx_rep&, const scfx_rep&, 
	                  int max_wl = SC_DEFAULT_MAX_WL_ );
scfx_rep*  div_scfx_rep( const scfx_rep&, const scfx_rep&, 
	                 int max_wl = SC_DEFAULT_DIV_WL_ );
scfx_rep*  add_scfx_rep( const scfx_rep&, const scfx_rep&, 
	                 int max_wl = SC_DEFAULT_MAX_WL_ );
scfx_rep*  sub_scfx_rep( const scfx_rep&, const scfx_rep&, 
	                 int max_wl = SC_DEFAULT_MAX_WL_ );
scfx_rep*  lsh_scfx_rep( const scfx_rep&, int );
scfx_rep*  rsh_scfx_rep( const scfx_rep&, int );
int        cmp_scfx_rep( const scfx_rep&, const scfx_rep& );


const int min_mant = 4;

const int bits_in_int  = sizeof(int)  * CHAR_BIT;
const int bits_in_word = sizeof(word) * CHAR_BIT;


// ----------------------------------------------------------------------------
//  CLASS : scfx_index
// ----------------------------------------------------------------------------

class scfx_index
{

public:

    scfx_index( int wi_, int bi_ ) : m_wi( wi_ ), m_bi( bi_ ) {}

    int wi() const { return m_wi; }
    int bi() const { return m_bi; }

    void wi( int wi_ ) { m_wi = wi_; }

private:

    int m_wi;
    int m_bi;

};


// ----------------------------------------------------------------------------
//  CLASS : scfx_rep
//
//  Arbitrary-precision fixed-point implementation class.
// ----------------------------------------------------------------------------

class scfx_rep
{
    enum state
    {
        normal,
        infinity,
        not_a_number
    };

public:

    // constructors

             scfx_rep();
    explicit scfx_rep( int );
    explicit scfx_rep( unsigned int );
    explicit scfx_rep( long );
    explicit scfx_rep( unsigned long );
    explicit scfx_rep( double );
    explicit scfx_rep( const char* );
    explicit scfx_rep( int64 );
    explicit scfx_rep( uint64 );
    explicit scfx_rep( const sc_signed& );
    explicit scfx_rep( const sc_unsigned& );


    // copy constructor

             scfx_rep( const scfx_rep& );


    // destructor

    ~scfx_rep();


    void* operator new( std::size_t );
    void  operator delete( void*, std::size_t );


    void from_string( const char*, int );

    double to_double() const;

    const char* to_string( sc_numrep,
			   int,
			   sc_fmt,
			   const scfx_params* = 0 ) const;


    // assignment operator

    void operator = ( const scfx_rep& );

    friend void multiply( scfx_rep&, const scfx_rep&, const scfx_rep&, int );

    friend scfx_rep* neg_scfx_rep( const scfx_rep& );
    friend scfx_rep* mult_scfx_rep( const scfx_rep&, const scfx_rep&, int );
    friend scfx_rep* div_scfx_rep( const scfx_rep&, const scfx_rep&, int );
    friend scfx_rep* add_scfx_rep( const scfx_rep&, const scfx_rep&, int );
    friend scfx_rep* sub_scfx_rep( const scfx_rep&, const scfx_rep&, int );
    friend scfx_rep* lsh_scfx_rep( const scfx_rep&, int );
    friend scfx_rep* rsh_scfx_rep( const scfx_rep&, int );

    void lshift( int );
    void rshift( int );

    friend int        cmp_scfx_rep( const scfx_rep&, const scfx_rep& );

    void cast( const scfx_params&, bool&, bool& );

    bool is_neg() const;
    bool is_zero() const;
    bool is_nan() const;
    bool is_inf() const;
    bool is_normal() const;

    void set_zero( int = 1 );
    void set_nan();
    void set_inf( int );

    bool   get_bit( int ) const;
    bool   set( int, const scfx_params& );
    bool clear( int, const scfx_params& );

    bool get_slice( int, int, const scfx_params&, sc_bv_base& ) const;
    bool set_slice( int, int, const scfx_params&, const sc_bv_base& );

    void print( ::std::ostream& ) const;
    void dump( ::std::ostream& ) const;

    void get_type( int&, int&, sc_enc& ) const;

    friend scfx_rep* quantization_scfx_rep( const scfx_rep&,
					     const scfx_params&,
					     bool& );
    friend scfx_rep*     overflow_scfx_rep( const scfx_rep&,
					     const scfx_params&,
					     bool& );

    bool rounding_flag() const;

private:

    friend void  align( const scfx_rep&, const scfx_rep&, int&, int&,
			scfx_mant_ref&, scfx_mant_ref& );
    friend int   compare_msw( const scfx_rep&, const scfx_rep& );
    friend int   compare_msw_ff( const scfx_rep& lhs, const scfx_rep& rhs );
    unsigned int divide_by_ten();
    int          find_lsw() const;
    int          find_msw() const;
    void         find_sw();
    void         multiply_by_ten();
    void         normalize( int );
    scfx_mant*   resize( int, int ) const;
    void         set_bin( int );
    void         set_oct( int, int );
    void         set_hex( int, int );
    void         shift_left( int );
    void         shift_right( int );

    const scfx_index calc_indices( int ) const;

    void o_extend( const scfx_index&, sc_enc );
    bool o_bit_at( const scfx_index& ) const;
    bool o_zero_left( const scfx_index& ) const;
    bool o_zero_right( const scfx_index& ) const;
    void o_set_low( const scfx_index&, sc_enc );
    void o_set_high( const scfx_index&, const scfx_index&, sc_enc, int = 1 );
    void o_set( const scfx_index&, const scfx_index&, sc_enc, bool );
    void o_invert( const scfx_index& );
    bool q_bit( const scfx_index& ) const;
    void q_clear( const scfx_index& );
    void q_incr( const scfx_index& );
    bool q_odd( const scfx_index& ) const;
    bool q_zero( const scfx_index& ) const;

    void resize_to( int, int = 0 );
    int  size() const;
    void toggle_tc();

    friend void print_dec( scfx_string&, const scfx_rep&, int, sc_fmt );
    friend void print_other( scfx_string&, const scfx_rep&, sc_numrep, int,
			     sc_fmt, const scfx_params* );

    void quantization( const scfx_params&, bool& );
    void     overflow( const scfx_params&, bool& );

    friend int compare_abs( const scfx_rep&, const scfx_rep& );

    void round( int );

private:

    scfx_mant m_mant;     // mantissa (bits of the value).
    int       m_wp;       // index of highest order word in value.
    int       m_sign;     // sign of value.
    state     m_state;    // value state, e.g., normal, inf, etc.
    int       m_msw;      // index of most significant non-zero word.
    int       m_lsw;      // index of least significant non-zero word.
    bool      m_r_flag;   // true if founding occurred.

};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

inline
void
scfx_rep::set_zero( int sign )
{
    m_mant.clear();
    m_wp = m_msw = m_lsw = 0;
    m_sign = sign;
    m_state = normal;
}

inline
void
scfx_rep::set_nan()
{
    m_mant.resize_to( min_mant );
    m_state = not_a_number;
}

inline
void
scfx_rep::set_inf( int sign )
{
    m_mant.resize_to( min_mant );
    m_state = infinity;
    m_sign = sign;
}


// constructors

inline
scfx_rep::scfx_rep( const char* s )
: m_mant( min_mant ), m_wp( 2 ), m_sign( 1 ), m_state( normal ),
  m_msw(0), m_lsw(0), m_r_flag( false )
{
    from_string( s, SC_DEFAULT_CTE_WL_ );
}


// destructor

inline
scfx_rep::~scfx_rep()
{}


// assignment operator

inline
void
scfx_rep::operator = ( const scfx_rep& f )
{
    if( &f != this )
    {
        m_mant  = f.m_mant;
	m_wp    = f.m_wp;
	m_sign  = f.m_sign;
	m_state = f.m_state;
	m_msw   = f.m_msw;
	m_lsw   = f.m_lsw;
	round( SC_DEFAULT_MAX_WL_ );
    }
}

inline
scfx_rep*
neg_scfx_rep( const scfx_rep& a )
{
    scfx_rep& c = *new scfx_rep( a );
    c.m_sign = - c.m_sign;
    return &c;
}

inline
scfx_rep*
mult_scfx_rep( const scfx_rep& a, const scfx_rep& b, int max_wl )
{
    scfx_rep& c = *new scfx_rep;
    sc_dt::multiply( c, a, b, max_wl );
    return &c;
}

inline
scfx_rep*
lsh_scfx_rep( const scfx_rep& a, int b )
{
    scfx_rep& c = *new scfx_rep( a );
    c.lshift( b );
    return &c;
}

inline
scfx_rep*
rsh_scfx_rep( const scfx_rep& a, int b )
{
    scfx_rep& c = *new scfx_rep( a );
    c.rshift( b );
    return &c;
}

inline
int
scfx_rep::size() const
{
    return m_mant.size();
}

inline
bool
scfx_rep::is_neg() const
{
    return ( m_sign == -1 );
}

inline
bool
scfx_rep::is_zero() const
{
    if( m_state != normal )
        return false;

    for( int i = 0; i < size(); i ++ )
    {
        if( m_mant[i] )
	    return false;
    }

    return true;
}

inline
bool
scfx_rep::is_nan() const
{
    return ( m_state == not_a_number );
}

inline
bool
scfx_rep::is_inf() const
{
    return ( m_state == infinity );
}

inline
bool
scfx_rep::is_normal() const
{
    return ( m_state == normal );
}

inline
scfx_rep*
quantization_scfx_rep( const scfx_rep& a,
			const scfx_params& params,
			bool& q_flag )
{
    scfx_rep& c = *new scfx_rep( a );
    c.quantization( params, q_flag );
    return &c;
}

inline
scfx_rep*
overflow_scfx_rep( const scfx_rep& a,
		    const scfx_params& params,
		    bool& o_flag )
{
    scfx_rep& c = *new scfx_rep( a );
    c.overflow( params, o_flag );
    return &c;
}

inline
bool
scfx_rep::rounding_flag() const
{
    return m_r_flag;
}

inline
void
scfx_rep::resize_to( int new_size, int restore )
{
    if( restore == -1 )
    {
        int size_incr = new_size - size();
	m_wp += size_incr;
	m_msw += size_incr;
	m_lsw += size_incr;
    }
    m_mant.resize_to( new_size, restore );
}

inline
const scfx_index
scfx_rep::calc_indices( int n ) const
{
    int wi = n / bits_in_word + m_wp;
    int bi = n % bits_in_word;

    if( bi < 0 )
    {
        bi += bits_in_word;
	-- wi;
    }

    return scfx_index( wi, bi );
}

inline
void
scfx_rep::o_extend( const scfx_index& x, sc_enc enc )
{
    int wi = x.wi();
    int bi = x.bi();

    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );
    
    if( enc == SC_US_ || ( m_mant[wi] & ( ((word)1) << bi ) ) == 0 )
    {
        if( bi != bits_in_word - 1 )
	    m_mant[wi] &= ~( ((word)-1) << ( bi + 1 ) );
	for( int i = wi + 1; i < size(); ++ i )
	    m_mant[i] = 0;
	m_sign = 1;
    }
    else
    {
        if( bi != bits_in_word - 1 )
	    m_mant[wi] |= ( ((word)-1) << ( bi + 1 ) );
	for( int i = wi + 1; i < size(); ++ i )
	    m_mant[i] = static_cast<word>( -1 );
	m_sign = -1;
    }
}

inline
bool
scfx_rep::o_bit_at( const scfx_index& x ) const
{
    int wi = x.wi();
    int bi = x.bi();
    
    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );

    return ( m_mant[wi] & ( ((word)1) << bi ) ) != 0;
}

inline
bool
scfx_rep::o_zero_left( const scfx_index& x ) const
{
    int wi = x.wi();
    int bi = x.bi();

    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );

    bool zero = true;
    if( bi != bits_in_word - 1 )
        zero = ( m_mant[wi] & ( ((word)-1) << ( bi + 1 ) ) ) == 0;
    for( int i = wi + 1; i < size(); ++ i )
	zero = zero && m_mant[i] == 0;

    return zero;
}

inline
bool
scfx_rep::o_zero_right( const scfx_index& x ) const
{
    int wi = x.wi();
    int bi = x.bi();

    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );

    bool zero = ( m_mant[wi] & ~( ((word)-1) << bi ) ) == 0;
    for( int i = wi - 1; i >= 0; -- i )
	zero = zero && m_mant[i] == 0;

    return zero;
}

inline
void
scfx_rep::o_set_low( const scfx_index& x, sc_enc enc )
{
    int wi = x.wi();
    int bi = x.bi();

    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );

    m_mant.clear();

    if( enc == SC_TC_ )
    {
	m_mant[wi] |= ( ((word)1) << bi );
	m_sign = -1;
    }
    else
	m_sign = 1;
}

inline
void
scfx_rep::o_set_high( const scfx_index& x, const scfx_index& x2,
		      sc_enc enc, int sign )
{
    int wi = x.wi();
    int bi = x.bi();
    int wi2 = x2.wi();
    int bi2 = x2.bi();

    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );
    SC_ASSERT_( wi2 >= 0 && wi2 < size(), "word index out of range" );
    
    int i;

    for( i = 0; i < size(); ++ i )
	m_mant[i] = static_cast<word>( -1 );

    m_mant[wi] &= ~( ((word)-1) << bi );
    for( i = wi + 1; i < size(); ++ i )
	m_mant[i] = 0;

    m_mant[wi2] &= ( ((word)-1) << bi2 );
    for( i = wi2 - 1; i >= 0; -- i )
	m_mant[i] = 0;
    
    if( enc == SC_TC_ )
	m_sign = sign;
    else
    {
	m_mant[wi] |= ( ((word)1) << bi );
	m_sign = 1;
    }
}

inline
void
scfx_rep::o_set( const scfx_index& x, const scfx_index& x3,
		 sc_enc enc, bool under )
{
    int wi = x.wi();
    int bi = x.bi();
    int wi3 = x3.wi();
    int bi3 = x3.bi();
    
    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );
    SC_ASSERT_( wi3 >= 0 && wi3 < size(), "word index out of range" );

    if( bi3 != bits_in_word - 1 )
    {
	if( under )
	    m_mant[wi3] &= ~( ((word)-1) << ( bi3 + 1 ) );
	else
	    m_mant[wi3] |= ( ((word)-1) << ( bi3 + 1 ) );
    }
    for( int i = wi3 + 1; i < size(); ++ i )
    {
	if( under )
	    m_mant[i] = 0;
	else
	    m_mant[i] = static_cast<word>( -1 );
    }
	
    if( enc == SC_TC_ )
    {
	if( under )
	    m_mant[wi] |= ( ((word)1) << bi );
	else
	    m_mant[wi] &= ~( ((word)1) << bi );
    }
}

inline
void
scfx_rep::o_invert( const scfx_index& x2 )
{
    int wi2 = x2.wi();
    int bi2 = x2.bi();

    m_mant[wi2] ^= ( ((word)-1) << bi2 );
    for( int i = wi2 + 1; i < size(); ++ i )
	m_mant[i] = ~ m_mant[i];
}

inline
bool
scfx_rep::q_bit( const scfx_index& x ) const
{
    int wi = x.wi();
    int bi = x.bi();

    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );

    if( bi != 0 )
        return ( m_mant[wi] & ( ((word)1) << ( bi - 1 ) ) ) != 0;
    else if( wi != 0 )
        return ( m_mant[wi - 1] & ( ((word)1) << ( bits_in_word - 1 ) ) ) != 0;
    else
        return false;
}

inline
void
scfx_rep::q_clear( const scfx_index& x )
{
    int wi = x.wi();
    int bi = x.bi();
    
    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );

    m_mant[wi] &= ( ((word)-1) << bi );
    for( int i = wi - 1; i >= 0; -- i )
        m_mant[i] = 0;
}

inline
void
scfx_rep::q_incr( const scfx_index& x )
{
    int wi = x.wi();
    int bi = x.bi();
    
    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );

    word old_val = m_mant[wi];
    m_mant[wi] += ( ((word)1) << bi );
    if( m_mant[wi] <= old_val )
    {
        if( wi + 1 == size() )
          resize_to( size() + 1, 1 );

        for( int i = wi + 1; i < size(); ++ i )
	{
	    if( ++ m_mant[i] != 0 )
	        break;
	}
    }
}

inline
bool
scfx_rep::q_odd( const scfx_index& x ) const
{
    int wi = x.wi();
    int bi = x.bi();

    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );

    return ( m_mant[wi] & ( ((word)1) << bi ) ) != 0;
}

inline
bool
scfx_rep::q_zero( const scfx_index& x ) const
{
    int wi = x.wi();
    int bi = x.bi();

    SC_ASSERT_( wi >= 0 && wi < size(), "word index out of range" );

    bool zero;

    if( bi != 0 )
    {
        zero = ( m_mant[wi] & ~( ((word)-1) << (bi - 1) ) ) == 0;
	for( int i = wi - 1; i >= 0; -- i )
	    zero = zero && m_mant[i] == 0;
    }
    else if( wi != 0 )
    {
        zero = ( m_mant[wi - 1] & ~( ((word)-1) << (bits_in_word - 1) ) ) == 0;
	for( int i = wi - 2; i >= 0; -- i )
	    zero = zero && m_mant[i] == 0;
    }
    else
        zero = true;

    return zero;
}

inline
int
scfx_rep::find_lsw() const
{
    for( int i = 0; i < size(); i ++ )
    {
        if( m_mant[i] )
	    return i;
    }
    return 0;
}

inline
int
scfx_rep::find_msw() const
{
    for( int i = size() - 1; i >= 0; i -- )
    {
        if( m_mant[i] )
	    return i;
    }
    return 0;
}

inline
void
scfx_rep::find_sw()
{
    m_lsw = find_lsw();
    m_msw = find_msw();
}

inline
void
scfx_rep::toggle_tc()
{
    if( is_neg() )
    {
        complement( m_mant, m_mant, m_mant.size() );
	inc( m_mant );
    }
}

} // namespace sc_dt


#endif

// Taf!
