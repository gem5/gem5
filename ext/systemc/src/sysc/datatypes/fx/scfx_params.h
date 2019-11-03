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

  scfx_params.h - 

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: scfx_params.h,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef SCFX_PARAMS_H
#define SCFX_PARAMS_H


#include "sysc/datatypes/fx/sc_fx_ids.h"
#include "sysc/datatypes/fx/sc_fxcast_switch.h"
#include "sysc/datatypes/fx/sc_fxtype_params.h"


namespace sc_dt
{

// classes defined in this module
class scfx_params;


// ----------------------------------------------------------------------------
//  CLASS : scfx_params
//
//  ...
// ----------------------------------------------------------------------------

class scfx_params
{

public:

    // constructor

    scfx_params( const sc_fxtype_params&,
		 sc_enc,
		 const sc_fxcast_switch& );


    // query functions

    const sc_fxtype_params& type_params() const;
    sc_enc enc() const;
    const sc_fxcast_switch& cast_switch() const;


    // shortcuts

    int wl() const;
    int iwl() const;
    int fwl() const;
    sc_q_mode q_mode() const;
    sc_o_mode o_mode() const;
    int n_bits() const;


    // dump content

    void dump( ::std::ostream& ) const;

private:

    sc_fxtype_params m_type_params;
    sc_enc           m_enc;
    sc_fxcast_switch m_cast_switch;
    
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// constructor

inline
scfx_params::scfx_params( const sc_fxtype_params& type_params_,
			  sc_enc enc_,
			  const sc_fxcast_switch& cast_sw )
: m_type_params( type_params_ ),
  m_enc( enc_ ),
  m_cast_switch( cast_sw )
{
    if( m_enc == SC_US_ && m_type_params.o_mode() == SC_WRAP_SM )
    {
	SC_REPORT_ERROR( sc_core::SC_ID_INVALID_O_MODE_,
			 "SC_WRAP_SM not defined for unsigned numbers" );
    }

}


// query functions

inline
const sc_fxtype_params&
scfx_params::type_params() const
{
    return m_type_params;
}

inline
sc_enc
scfx_params::enc() const
{
    return m_enc;
}

inline
const sc_fxcast_switch&
scfx_params::cast_switch() const
{
    return m_cast_switch;
}


// shortcuts

inline
int
scfx_params::wl() const
{
    return m_type_params.wl();
}

inline
int
scfx_params::iwl() const
{
    return m_type_params.iwl();
}

inline
int
scfx_params::fwl() const
{
    return ( m_type_params.wl() - m_type_params.iwl() );
}

inline
sc_q_mode
scfx_params::q_mode() const
{
    return m_type_params.q_mode();
}

inline
sc_o_mode
scfx_params::o_mode() const
{
    return m_type_params.o_mode();
}

inline
int
scfx_params::n_bits() const
{
    return m_type_params.n_bits();
}


// dump content

inline
void
scfx_params::dump( ::std::ostream& os ) const
{
    os << "scfx_params" << ::std::endl;
    os << "(" << ::std::endl;
    os << "type_params = ";
    m_type_params.dump( os );
    os << "enc         = " << m_enc << ::std::endl;
    os << "cast_switch = ";
    m_cast_switch.dump( os );
    os << ")" << ::std::endl;
}

} // namespace sc_dt


#endif

// Taf!
