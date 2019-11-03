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

  sc_unsigned_subref.h -- Proxy class that is declared in sc_unsigned.h.  

  Original Author: Ali Dasdan, Synopsys, Inc.
 
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned_subref_r
//
//  Proxy class for sc_unsigned part selection (r-value only).
// ----------------------------------------------------------------------------

// concatenation support

uint64 sc_unsigned_subref_r::concat_get_uint64() const // #### Speed up!
{
    sc_unsigned a( m_obj_p, m_left, m_right );
    return a.to_uint64();
}


bool sc_unsigned_subref_r::concat_get_ctrl(sc_digit* dst_p, int low_i) const
    // #### Speed up!
{
    sc_unsigned a( m_obj_p, m_left, m_right );  
    return a.concat_get_ctrl( dst_p, low_i );
}

bool sc_unsigned_subref_r::concat_get_data(sc_digit* dst_p, int low_i) const
    // #### Speed up!
{
    sc_unsigned a( m_obj_p, m_left, m_right );  
    return a.concat_get_data( dst_p, low_i );
}


// implicit conversion to sc_unsigned

sc_unsigned_subref_r::operator sc_unsigned () const
{
    return sc_unsigned( m_obj_p, m_left, m_right );
}


// explicit conversions

int
sc_unsigned_subref_r::to_int() const
{
    sc_unsigned a(  m_obj_p, m_left, m_right );
    return a.to_int();
}

unsigned int
sc_unsigned_subref_r::to_uint() const
{
    sc_unsigned a( m_obj_p, m_left, m_right );
    return a.to_uint();
}

long
sc_unsigned_subref_r::to_long() const
{
    sc_unsigned a( m_obj_p, m_left, m_right );
    return a.to_long();
}

unsigned long
sc_unsigned_subref_r::to_ulong() const
{
    sc_unsigned a( m_obj_p, m_left, m_right );
    return a.to_ulong();
}

int64
sc_unsigned_subref_r::to_int64() const
{
    sc_unsigned a( m_obj_p, m_left, m_right );
    return a.to_int64();
}

uint64
sc_unsigned_subref_r::to_uint64() const
{
    sc_unsigned a( m_obj_p, m_left, m_right );
    return a.to_uint64();
}

double
sc_unsigned_subref_r::to_double() const
{
    sc_unsigned a( m_obj_p, m_left, m_right );
    return a.to_double();
}


// explicit conversion to character string

const std::string
sc_unsigned_subref_r::to_string( sc_numrep numrep ) const
{
    sc_unsigned a( length() );
    a = *this;
    return a.to_string( numrep );
}

const std::string
sc_unsigned_subref_r::to_string( sc_numrep numrep, bool w_prefix ) const
{
    sc_unsigned a( length() );
    a = *this;
    return a.to_string( numrep, w_prefix );
}


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned_subref
//
//  Proxy class for sc_unsigned part selection (r-value and l-value).
// ----------------------------------------------------------------------------

// assignment operators

const sc_unsigned_subref& 
sc_unsigned_subref::operator = ( const sc_unsigned_subref_r& a )
{
    return operator = ( (sc_unsigned)( a ) );
}

const sc_unsigned_subref& 
sc_unsigned_subref::operator = ( const sc_unsigned_subref& a )
{
    if( this == &a ) {
	return *this;
    }
    return operator = ( (sc_unsigned)( a ) );
}

const sc_unsigned_subref& 
sc_unsigned_subref::operator = ( const sc_unsigned& v )
{
    int i;
    int l = sc_min( m_left, v.nbits - 1 + m_right );

    for( i = m_right; i <= l; ++ i ) m_obj_p->set( i, v.test( i - m_right ) );
    for ( ; i <= m_left; i++ ) m_obj_p->set( i, v.test( l ) );
    return *this;
}

const sc_unsigned_subref& 
sc_unsigned_subref::operator = ( const sc_signed_subref_r& v )
{
    return operator = ( (sc_unsigned)( v ) );
}

const sc_unsigned_subref& 
sc_unsigned_subref::operator = ( const sc_signed& v )
{
    int i;
    int l = sc_min( m_left, v.nbits - 1 + m_right );

    for( i = m_right; i <= l; ++ i ) m_obj_p->set( i, v.test( i - m_right ) );
    for ( ; i <= m_left; i++ ) m_obj_p->set( i, 0 );
    return *this;
}

const sc_unsigned_subref& 
sc_unsigned_subref::operator = ( unsigned long v )
{
    for( int i = m_right; i <= m_left; ++ i ) {
	m_obj_p->set( i, static_cast<bool>( v & 1 ) );
	v >>= 1;
    }
    return *this;
}

const sc_unsigned_subref& 
sc_unsigned_subref::operator = ( long v ) 
{
    unsigned long v2 = (unsigned long) v;
    for( int i = m_right; i <= m_left; ++ i ) {
	m_obj_p->set( i, static_cast<bool>( v2 & 1 ) );
	v2 >>= 1;
    }
    return *this;
}

const sc_unsigned_subref& 
sc_unsigned_subref::operator = ( uint64  v )
{
    for( int i = m_right; i <= m_left; ++ i ) {
	m_obj_p->set( i, static_cast<bool>( v & 1 ) );
	v >>= 1;
    }
    return *this;
}

const sc_unsigned_subref& 
sc_unsigned_subref::operator = ( int64 v ) 
{
    uint64 v2 = (uint64) v;
    for( int i = m_right; i <= m_left; ++ i ) {
	m_obj_p->set( i, static_cast<bool>( v2 & 1 ) );
	v2 >>= 1;
    }
    return *this;
}

const sc_unsigned_subref& 
sc_unsigned_subref::operator = ( double v )
{
    is_bad_double(v);

    int nb = m_left - m_right + 1;
    int nd = DIV_CEIL(nb);

#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS];
#else
    sc_digit *d = new sc_digit[nd];
#endif

    if (v < 0)
	v = -v;

    int i = 0;

    while (floor(v) && (i < nd)) {
#ifndef _WIN32
	d[i++] = (sc_digit) floor(remainder(v, DIGIT_RADIX));
#else
	d[i++] = (sc_digit) floor(fmod(v, DIGIT_RADIX));
#endif
	v /= DIGIT_RADIX;
    }

    vec_zero(i, nd, d);

    sc_digit val = 1;  // Bit value.
    int j = 0;   // Current digit in d.

    i = 0;  // Current bit in d.

    while (i < nb) {

	m_obj_p->set(i + m_right, (bool) (d[j] & val));

	++i;

	if (i % BITS_PER_DIGIT == 0) {
	    val = 1;
	    ++j;
	}
	else
	    val <<= 1;
    }

#ifndef SC_MAX_NBITS
    delete [] d;
#endif

    return *this;
}

const sc_unsigned_subref&
sc_unsigned_subref::operator = ( const sc_int_base& a )
{
    return operator = ( (int64) a );
}

const sc_unsigned_subref&
sc_unsigned_subref::operator = ( const sc_uint_base& a )
{
    return operator = ( (uint64) a );
}

// concatenation methods

void sc_unsigned_subref::concat_set( int64 src, int low_i )
{
    int  i;
    int  l;
    bool sign = src < 0;

    if ( low_i < 64 )    
    {    
	src = src >> low_i;
	l = sc_min( m_left, (63-low_i) + m_right );    
	for( i = m_right; i <= l; ++ i ) {    
		m_obj_p->set( i, src & 1 );    
		src = src >> 1;    
	}    
	for ( ; i <= m_left; i++ ) m_obj_p->set(sign);     
    }    
    else    
    {    
	for( i = m_right; i <= m_left; ++ i ) m_obj_p->set(sign);     
    }    
}

void sc_unsigned_subref::concat_set( const sc_signed& src, int low_i )
{
    int i;
    int l;
    int src_i;
    bool sign = src.test(src.nbits-1);
    l = src.nbits - (low_i+1);
    if ( l >= 0 )
    {
	src_i = low_i;
	l = sc_min( m_left, l + m_right );
	for( i = m_right; i <= l; ++ i, src_i++ ) {
	    m_obj_p->set( i, src.test( src_i ) );
	}
	for ( ; i <= m_left; i++ ) m_obj_p->set(i, sign);
    }
    else
    {    
	for( i = m_right; i <= m_left; ++ i ) m_obj_p->set(i, sign);     
    }    
}

void sc_unsigned_subref::concat_set( const sc_unsigned& src, int low_i )
{
    int i;
    int l;
    int src_i;
    l = src.nbits - (low_i+2);
    if ( l >= 0 )
    {
	src_i = low_i;
	l = sc_min( m_left, l + m_right );
	for( i = m_right; i <= l; ++ i, src_i++ ) {
	    m_obj_p->set( i, src.test( src_i ) );
	}
	for ( ; i <= m_left; i++ ) m_obj_p->set(i, false);
    }
    else
    {    
	for( i = m_right; i <= m_left; ++ i ) m_obj_p->set(i, false);     
    }    
}

void sc_unsigned_subref::concat_set( uint64 src, int low_i )
{
    int  i;
    int  l;

    if ( low_i < 64 )    
    {    
	src = src >> low_i;
	l = sc_min( m_left, (63-low_i) + m_right );    
	for( i = m_right; i <= l; ++ i ) {    
		m_obj_p->set( i, src & 1 );    
		src = src >> 1;    
	}    
	for ( ; i <= m_left; i++ ) m_obj_p->set(false);     
    }    
    else    
    {    
	for( i = m_right; i <= m_left; ++ i ) m_obj_p->set(false);     
    }    
}
// other methods

void
sc_unsigned_subref::scan( ::std::istream& is )
{
    std::string s;
    is >> s;
    *this = s.c_str();
}


// End of file
