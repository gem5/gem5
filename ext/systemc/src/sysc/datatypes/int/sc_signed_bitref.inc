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

  sc_signed_bitref.h -- Proxy class that is declared in sc_signed.h.

  Original Author: Ali Dasdan, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// ----------------------------------------------------------------------------
//  CLASS : sc_signed_bitref_r
//
//  Proxy class for sc_signed bit selection (r-value only).
// ----------------------------------------------------------------------------

// implicit conversion to uint64

sc_signed_bitref_r::operator uint64 () const
{
    return m_obj_p->test( m_index );
}

bool
sc_signed_bitref_r::operator ! () const
{
    return ( ! m_obj_p->test( m_index ) );
}

bool
sc_signed_bitref_r::operator ~ () const
{
    return ( ! m_obj_p->test( m_index ) );
}


// ----------------------------------------------------------------------------
//  CLASS : sc_signed_bitref
//
//  Proxy class for sc_signed bit selection (r-value and l-value).
// ----------------------------------------------------------------------------

// assignment operators

const sc_signed_bitref&
sc_signed_bitref::operator = ( const sc_signed_bitref_r& b )
{
    m_obj_p->set( m_index, (bool) b );
    return *this;
}

const sc_signed_bitref&
sc_signed_bitref::operator = ( const sc_signed_bitref& b )
{
    m_obj_p->set( m_index, (bool) b );
    return *this;
}

const sc_signed_bitref&
sc_signed_bitref::operator = ( bool b )
{
    m_obj_p->set( m_index, b );
    return *this;
}


const sc_signed_bitref&
sc_signed_bitref::operator &= ( bool b )
{
    if( ! b ) {
	m_obj_p->clear( m_index );
    }
    return *this;
}

const sc_signed_bitref&
sc_signed_bitref::operator |= ( bool b )
{
    if( b ) {
	m_obj_p->set( m_index );
    }
    return *this;
}

const sc_signed_bitref&
sc_signed_bitref::operator ^= ( bool b )
{
    if( b ) {
	m_obj_p->invert( m_index );
    }
    return *this;
}

// #### OPTIMIZE
void sc_signed_bitref::concat_set(int64 src, int low_i)  
{    
	bool value = 1 & ((low_i < 64) ? (src >> low_i) : (src >> 63));
    m_obj_p->set(low_i, value);
}

void sc_signed_bitref::concat_set(const sc_signed& src, int low_i)
{
    if ( low_i < src.length() )
        m_obj_p->set(low_i, src.test(low_i));
    else
        m_obj_p->set(low_i, src<0);
}       

void sc_signed_bitref::concat_set(const sc_unsigned& src, int low_i)
{
    if ( low_i < src.length() )
        m_obj_p->set(low_i, src.test(low_i));
    else
        m_obj_p->set(low_i, 0);
}

void sc_signed_bitref::concat_set(uint64 src, int low_i)
{
	bool value = 1 & ((low_i < 64) ? (src >> low_i) : 0);
    m_obj_p->set(low_i, value);
}


// other methods

void
sc_signed_bitref::scan( ::std::istream& is )
{
    bool b;
    is >> b;
    *this = b;
}


// End of file
