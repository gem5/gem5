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

  sc_lv_base.cpp -- Arbitrary size logic vector class.

  Original Author: Gene Bushuyev, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: sc_lv_base.cpp,v $
// Revision 1.2  2011/08/24 22:05:40  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:53  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include "sysc/datatypes/bit/sc_bit_ids.h"
#include "sysc/datatypes/bit/sc_lv_base.h"


namespace sc_dt
{

// ----------------------------------------------------------------------------
//  CLASS : sc_lv_base
//
//  Arbitrary size logic vector base class.
// ----------------------------------------------------------------------------

static const sc_digit data_array[] =
    { SC_DIGIT_ZERO, ~SC_DIGIT_ZERO, SC_DIGIT_ZERO, ~SC_DIGIT_ZERO };

static const sc_digit ctrl_array[] =
    { SC_DIGIT_ZERO, SC_DIGIT_ZERO, ~SC_DIGIT_ZERO, ~SC_DIGIT_ZERO };


void
sc_lv_base::init( int length_, const sc_logic& init_value )
{
    // check the length
    if( length_ <= 0 ) {
	SC_REPORT_ERROR( sc_core::SC_ID_ZERO_LENGTH_, 0 );
    }
    // allocate memory for the data and control words
    m_len = length_;
    m_size = (m_len - 1) / SC_DIGIT_SIZE + 1;
    m_data = new sc_digit[m_size * 2];
    m_ctrl = m_data + m_size;
    // initialize the bits to 'init_value'
    sc_digit dw = data_array[init_value.value()];
    sc_digit cw = ctrl_array[init_value.value()];
    int sz = m_size;
    for( int i = 0; i < sz; ++ i ) {
	m_data[i] = dw;
	m_ctrl[i] = cw;
    }
    clean_tail();
}


void
sc_lv_base::assign_from_string( const std::string& s )
{
    // s must have been converted to bin
    int len = m_len;
    int s_len = s.length() - 1;
    int min_len = sc_min( len, s_len );
    int i = 0;
    for( ; i < min_len; ++ i ) {
	char c = s[s_len - i - 1];
	set_bit( i, sc_logic::char_to_logic[(int)c] );
    }
    // if formatted, fill the rest with sign(s), otherwise fill with zeros
    sc_logic_value_t fill = (s[s_len] == 'F' ? sc_logic_value_t( s[0] - '0' )
		                             : sc_logic_value_t( 0 ));
    for( ; i < len; ++ i ) {
	set_bit( i, fill );
    }
}


// constructors

sc_lv_base::sc_lv_base( const char* a )
    : m_len( 0 ), m_size( 0 ), m_data( 0 ), m_ctrl( 0 )
{
    std::string s = convert_to_bin( a );
    init( s.length() - 1 );
    assign_from_string( s );
}

sc_lv_base::sc_lv_base( const char* a, int length_ )
    : m_len( 0 ), m_size( 0 ), m_data( 0 ), m_ctrl( 0 )
{
    init( length_ );
    assign_from_string( convert_to_bin( a ) );
}

sc_lv_base::sc_lv_base( const sc_lv_base& a )
    : sc_proxy<sc_lv_base>(),
      m_len( a.m_len ),
      m_size( a.m_size ),
      m_data( new sc_digit[m_size * 2] ),
      m_ctrl( m_data + m_size )
{
    // copy the bits
    int sz = m_size;
    for( int i = 0; i < sz; ++ i ) {
	m_data[i] = a.m_data[i];
	m_ctrl[i] = a.m_ctrl[i];
    }
}


// assignment operators

sc_lv_base&
sc_lv_base::operator = ( const char* a )
{
    assign_from_string( convert_to_bin( a ) );
    return *this;
}


// returns true if logic vector contains only 0's and 1's

bool
sc_lv_base::is_01() const
{
    int sz = m_size;
    for( int i = 0; i < sz; ++ i ) {
	if( m_ctrl[i] != 0 ) {
	    return false;
	}
    }
    return true;
}

} // namespace sc_dt
