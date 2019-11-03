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

  sc_fxdefs.h - 

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_fxdefs.h,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:57  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef SC_FXDEFS_H
#define SC_FXDEFS_H


#include "sysc/utils/sc_machine.h"
#include "sysc/datatypes/fx/sc_fx_ids.h"
#include "sysc/datatypes/int/sc_nbutils.h"


namespace sc_dt
{

// ----------------------------------------------------------------------------
//  ENUM : sc_enc
//
//  Enumeration of sign encodings.
// ----------------------------------------------------------------------------

enum sc_enc
{
    SC_TC_,	// two's complement
    SC_US_	// unsigned
};


const std::string to_string( sc_enc );


inline
::std::ostream&
operator << ( ::std::ostream& os, sc_enc enc )
{
    return os << to_string( enc );
}


// ----------------------------------------------------------------------------
//  ENUM : sc_q_mode
//
//  Enumeration of quantization modes.
// ----------------------------------------------------------------------------

enum sc_q_mode
{
    SC_RND,		// rounding to plus infinity
    SC_RND_ZERO,	// rounding to zero
    SC_RND_MIN_INF,	// rounding to minus infinity
    SC_RND_INF,		// rounding to infinity
    SC_RND_CONV,	// convergent rounding
    SC_TRN,		// truncation
    SC_TRN_ZERO		// truncation to zero
};


const std::string to_string( sc_q_mode );


inline
::std::ostream&
operator << ( ::std::ostream& os, sc_q_mode q_mode )
{
    return os << to_string( q_mode );
}


// ----------------------------------------------------------------------------
//  ENUM : sc_o_mode
//
//  Enumeration of overflow modes.
// ----------------------------------------------------------------------------

enum sc_o_mode
{
    SC_SAT,		// saturation
    SC_SAT_ZERO,	// saturation to zero
    SC_SAT_SYM,		// symmetrical saturation
    SC_WRAP,		// wrap-around (*)
    SC_WRAP_SM		// sign magnitude wrap-around (*)
};

// (*) uses the number of saturated bits argument, see the documentation.


const std::string to_string( sc_o_mode );


inline
::std::ostream&
operator << ( ::std::ostream& os, sc_o_mode o_mode )
{
    return os << to_string( o_mode );
}


// ----------------------------------------------------------------------------
//  ENUM : sc_switch
//
//  Enumeration of switch states.
// ----------------------------------------------------------------------------

enum sc_switch
{
    SC_OFF,
    SC_ON
};


const std::string to_string( sc_switch );


inline
::std::ostream&
operator << ( ::std::ostream& os, sc_switch sw )
{
    return os << to_string( sw );
}


// ----------------------------------------------------------------------------
//  ENUM : sc_fmt
//
//  Enumeration of formats for character string conversion.
// ----------------------------------------------------------------------------

enum sc_fmt
{
    SC_F,	// fixed
    SC_E	// scientific
};


const std::string to_string( sc_fmt );


inline
::std::ostream&
operator << ( ::std::ostream& os, sc_fmt fmt )
{
    return os << to_string( fmt );
}


// ----------------------------------------------------------------------------
//  Built-in & default fixed-point type parameter values.
// ----------------------------------------------------------------------------

const int       SC_BUILTIN_WL_     = 32;
const int       SC_BUILTIN_IWL_    = 32;
const sc_q_mode SC_BUILTIN_Q_MODE_ = SC_TRN;
const sc_o_mode SC_BUILTIN_O_MODE_ = SC_WRAP;
const int       SC_BUILTIN_N_BITS_ = 0;


const int       SC_DEFAULT_WL_     = SC_BUILTIN_WL_;
const int       SC_DEFAULT_IWL_    = SC_BUILTIN_IWL_;
const sc_q_mode SC_DEFAULT_Q_MODE_ = SC_BUILTIN_Q_MODE_;
const sc_o_mode SC_DEFAULT_O_MODE_ = SC_BUILTIN_O_MODE_;
const int       SC_DEFAULT_N_BITS_ = SC_BUILTIN_N_BITS_;


// ----------------------------------------------------------------------------
//  Built-in & default fixed-point cast switch parameter values.
// ----------------------------------------------------------------------------

const sc_switch SC_BUILTIN_CAST_SWITCH_ = SC_ON;


const sc_switch SC_DEFAULT_CAST_SWITCH_ = SC_BUILTIN_CAST_SWITCH_;


// ----------------------------------------------------------------------------
//  Built-in & default fixed-point value type parameter values.
// ----------------------------------------------------------------------------

const int SC_BUILTIN_DIV_WL_ = 64;
const int SC_BUILTIN_CTE_WL_ = 64;
const int SC_BUILTIN_MAX_WL_ = 1024;


#if defined( SC_FXDIV_WL ) && ( SC_FXDIV_WL > 0 )
const int SC_DEFAULT_DIV_WL_ = SC_FXDIV_WL;
#else
const int SC_DEFAULT_DIV_WL_ = SC_BUILTIN_DIV_WL_;
#endif

#if defined( SC_FXCTE_WL ) && ( SC_FXCTE_WL > 0 )
const int SC_DEFAULT_CTE_WL_ = SC_FXCTE_WL;
#else
const int SC_DEFAULT_CTE_WL_ = SC_BUILTIN_CTE_WL_;
#endif

#if defined( SC_FXMAX_WL ) && ( SC_FXMAX_WL > 0 || SC_FXMAX_WL == -1 )
const int SC_DEFAULT_MAX_WL_ = SC_FXMAX_WL;
#else
const int SC_DEFAULT_MAX_WL_ = SC_BUILTIN_MAX_WL_;
#endif


// ----------------------------------------------------------------------------
//  Dedicated error reporting and checking.
// ----------------------------------------------------------------------------

#ifdef DEBUG_SYSTEMC
#define SC_ASSERT_(cnd,msg)                                                   \
{                                                                             \
    if( ! (cnd) )                                                             \
        SC_REPORT_ERROR( sc_core::SC_ID_INTERNAL_ERROR_, msg );                        \
}
#else
#define SC_ASSERT_(cnd,msg)
#endif

#define SC_ERROR_IF_(cnd,id)                                                  \
{                                                                             \
    if( cnd )                                                                 \
        SC_REPORT_ERROR( id, 0 );                                             \
}


#define SC_CHECK_WL_(wl)                                                      \
    SC_ERROR_IF_( (wl) <= 0, sc_core::SC_ID_INVALID_WL_ )

#define SC_CHECK_N_BITS_(n_bits)                                              \
    SC_ERROR_IF_( (n_bits) < 0, sc_core::SC_ID_INVALID_N_BITS_ )

#define SC_CHECK_DIV_WL_(div_wl)                                              \
    SC_ERROR_IF_( (div_wl) <= 0, sc_core::SC_ID_INVALID_DIV_WL_ )

#define SC_CHECK_CTE_WL_(cte_wl)                                              \
    SC_ERROR_IF_( (cte_wl) <= 0, sc_core::SC_ID_INVALID_CTE_WL_ )

#define SC_CHECK_MAX_WL_(max_wl)                                              \
    SC_ERROR_IF_( (max_wl) <= 0 && (max_wl) != -1,                            \
	    sc_core::SC_ID_INVALID_MAX_WL_ )


// ----------------------------------------------------------------------------
//  Generic observer macros.
// ----------------------------------------------------------------------------

#define SC_OBSERVER_(object,observer_type,event)                              \
{                                                                             \
    if( (object).observer() != 0 )                                            \
    {                                                                         \
	observer_type observer = (object).lock_observer();                    \
	observer->event( (object) );                                          \
	(object).unlock_observer( observer );                                 \
    }                                                                         \
}

#define SC_OBSERVER_DEFAULT_(observer_type)                                   \
{                                                                             \
    if( m_observer == 0 && observer_type ## ::default_observer != 0 )         \
        m_observer = (* ## observer_type ## ::default_observer)();            \
}

} // namespace sc_dt


#endif

// Taf!
