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

  scfx_other_defs.h - 

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: scfx_other_defs.h,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef SCFX_OTHER_DEFS_H
#define SCFX_OTHER_DEFS_H


#include "sysc/datatypes/fx/sc_fx_ids.h"
#include "sysc/datatypes/int/sc_signed.h"
#include "sysc/datatypes/int/sc_unsigned.h"
#include "sysc/datatypes/int/sc_int_base.h"
#include "sysc/datatypes/int/sc_uint_base.h"
#include "sysc/tracing/sc_trace.h"


namespace sc_dt
{

#ifdef SC_INCLUDE_FX

// ----------------------------------------------------------------------------
//  CLASS : sc_signed
// ----------------------------------------------------------------------------

// assignment operators

inline
const sc_signed&
sc_signed::operator = ( const sc_fxval& v )
{
    if( ! v.is_normal() ) /* also triggers OBSERVER_READ call */
    {
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_signed::operator = ( const sc_fxval& )" );
    }

    for( int i = 0; i < length(); ++ i )
	(*this)[i] = v.get_bit( i );

    return *this;
}

inline
const sc_signed&
sc_signed::operator = ( const sc_fxval_fast& v )
{
    if( ! v.is_normal() ) /* also triggers OBSERVER_READ call */
    {
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_signed::operator = ( const sc_fxval_fast& )" );
    }

    for( int i = 0; i < length(); ++ i )
	(*this)[i] = v.get_bit( i );

    return *this;
}

inline
const sc_signed&
sc_signed::operator = ( const sc_fxnum& v )
{
    if( ! v.is_normal() ) /* also triggers OBSERVER_READ call */
    {
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_signed::operator = ( const sc_fxnum& )" );
    }

    for( int i = 0; i < length(); ++ i )
	(*this)[i] = v.get_bit( i );

    return *this;
}

inline
const sc_signed&
sc_signed::operator = ( const sc_fxnum_fast& v )
{
    if( ! v.is_normal() ) /* also triggers OBSERVER_READ call */
    {
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_signed::operator = ( const sc_fxnum_fast& )" );
    }

    for( int i = 0; i < length(); ++ i )
	(*this)[i] = v.get_bit( i );

    return *this;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_unsigned
// ----------------------------------------------------------------------------

// assignment operators

inline
const sc_unsigned&
sc_unsigned::operator = ( const sc_fxval& v )
{
    if( ! v.is_normal() ) /* also triggers OBSERVER_READ call */
    {
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_unsigned::operator = ( const sc_fxval& )" );
    }

    for( int i = 0; i < length(); ++ i )
	(*this)[i] = v.get_bit( i );

    return *this;
}

inline
const sc_unsigned&
sc_unsigned::operator = ( const sc_fxval_fast& v )
{
    if( ! v.is_normal() ) /* also triggers OBSERVER_READ call */
    {
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_unsigned::operator = ( const sc_fxval_fast& )" );
    }

    for( int i = 0; i < length(); ++ i )
	(*this)[i] = v.get_bit( i );

    return *this;
}

inline
const sc_unsigned&
sc_unsigned::operator = ( const sc_fxnum& v )
{
    if( ! v.is_normal() ) /* also triggers OBSERVER_READ call */
    {
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_unsigned::operator = ( const sc_fxnum& )" );
    }

    for( int i = 0; i < length(); ++ i )
	(*this)[i] = v.get_bit( i );

    return *this;
}

inline
const sc_unsigned&
sc_unsigned::operator = ( const sc_fxnum_fast& v )
{
    if( ! v.is_normal() ) /* also triggers OBSERVER_READ call */
    {
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_unsigned::operator = ( const sc_fxnum_fast& )" );
    }

    for( int i = 0; i < length(); ++ i )
	(*this)[i] = v.get_bit( i );

    return *this;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_int_base
// ----------------------------------------------------------------------------

#ifndef _32BIT_
#define NUM_WIDTH LLWIDTH
#else
#define NUM_WIDTH INTWIDTH
#endif


// assignment operators

inline
sc_int_base&
sc_int_base::operator = ( const sc_fxval& v )
{
    if( ! v.is_normal() ) { /* also triggers OBSERVER_READ call */
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_int_base::operator = ( const sc_fxval& )" );
    }
    for( int i = 0; i < m_len; ++ i ) {
	set( i, v.get_bit( i ) );
    }
    extend_sign();
    return *this;
}

inline
sc_int_base&
sc_int_base::operator = ( const sc_fxval_fast& v )
{
    if( ! v.is_normal() ) { /* also triggers OBSERVER_READ call */
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_int_base::operator = ( const sc_fxval_fast& )" );
    }
    for( int i = 0; i < m_len; ++ i ) {
	set( i, v.get_bit( i ) );
    }
    extend_sign();
    return *this;
}

inline
sc_int_base&
sc_int_base::operator = ( const sc_fxnum& v )
{
    if( ! v.is_normal() ) { /* also triggers OBSERVER_READ call */
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_int_base::operator = ( const sc_fxnum& )" );
    }
    for( int i = 0; i < m_len; ++ i ) {
	set( i, v.get_bit( i ) );
    }
    extend_sign();
    return *this;
}

inline
sc_int_base&
sc_int_base::operator = ( const sc_fxnum_fast& v )
{
    if( ! v.is_normal() ) { /* also triggers OBSERVER_READ call */
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_int_base::operator = ( const sc_fxnum_fast& )" );
    }
    for( int i = 0; i < m_len; ++ i ) {
	set( i, v.get_bit( i ) );
    }
    extend_sign();
    return *this;
}

#undef NUM_WIDTH


// ----------------------------------------------------------------------------
//  CLASS : sc_uint_base
// ----------------------------------------------------------------------------

// assignment operators

inline
sc_uint_base&
sc_uint_base::operator = ( const sc_fxval& v )
{
    if( ! v.is_normal() ) { /* also triggers OBSERVER_READ call */
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_uint_base::operator = ( const sc_fxval& )" );
    }
    for( int i = 0; i < m_len; ++ i ) {
	set( i, v.get_bit( i ) );
    }
    extend_sign();
    return *this;
}

inline
sc_uint_base&
sc_uint_base::operator = ( const sc_fxval_fast& v )
{
    if( ! v.is_normal() ) { /* also triggers OBSERVER_READ call */
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_uint_base::operator = ( const sc_fxval_fast& )" );
    }
    for( int i = 0; i < m_len; ++ i ) {
	set( i, v.get_bit( i ) );
    }
    extend_sign();
    return *this;
}

inline
sc_uint_base&
sc_uint_base::operator = ( const sc_fxnum& v )
{
    if( ! v.is_normal() ) { /* also triggers OBSERVER_READ call */
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_uint_base::operator = ( const sc_fxnum& )" );
    }
    for( int i = 0; i < m_len; ++ i ) {
	set( i, v.get_bit( i ) );
    }
    extend_sign();
    return *this;
}

inline
sc_uint_base&
sc_uint_base::operator = ( const sc_fxnum_fast& v )
{
    if( ! v.is_normal() ) { /* also triggers OBSERVER_READ call */
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_FX_VALUE_,
			 "sc_uint_base::operator = ( const sc_fxnum_fast& )" );
    }
    for( int i = 0; i < m_len; ++ i ) {
	set( i, v.get_bit( i ) );
    }
    extend_sign();
    return *this;
}


#endif


// ----------------------------------------------------------------------------
//  FUNCTION : sc_trace
// ----------------------------------------------------------------------------

inline
void
sc_trace( sc_core::sc_trace_file* tf,
	  const sc_fxval& object, const std::string& name )
{
    if( tf )
	tf->trace( object, name );
}

inline
void
sc_trace( sc_core::sc_trace_file* tf,
	  const sc_fxval* object, const std::string& name )
{
    if( tf )
	tf->trace( *object, name );
}

inline
void
sc_trace( sc_core::sc_trace_file* tf,
	  const sc_fxval_fast& object, const std::string& name )
{
    if( tf )
	tf->trace( object, name );
}

inline
void
sc_trace( sc_core::sc_trace_file* tf,
	  const sc_fxval_fast* object, const std::string& name )
{
    if( tf )
	tf->trace( *object, name );
}

inline
void
sc_trace( sc_core::sc_trace_file* tf,
	  const sc_fxnum& object, const std::string& name )
{
    if( tf )
	tf->trace( object, name );
}

inline
void
sc_trace( sc_core::sc_trace_file* tf,
	  const sc_fxnum* object, const std::string& name )
{
    if( tf )
	tf->trace( *object, name );
}

inline
void
sc_trace( sc_core::sc_trace_file* tf,
	  const sc_fxnum_fast& object, const std::string& name )
{
    if( tf )
	tf->trace( object, name );
}

inline
void
sc_trace( sc_core::sc_trace_file* tf,
	  const sc_fxnum_fast* object, const std::string& name )
{
    if( tf )
	tf->trace( *object, name );
}

} // namespace sc_dt


#endif

// Taf!
