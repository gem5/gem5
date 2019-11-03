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

  sc_length_param.h -

  Original Author: Martin Janssen, Synopsys, Inc., 2002-03-19

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_length_param.h,v $
// Revision 1.3  2011/08/24 22:05:46  acg
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
// Revision 1.3  2006/01/13 18:49:32  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#ifndef SC_LENGTH_PARAM_H
#define SC_LENGTH_PARAM_H


#include "sysc/datatypes/fx/sc_context.h"


namespace sc_dt
{

// classes defined in this module
class sc_length_param;

// friend operator declarations
    bool operator == ( const sc_length_param&,
                              const sc_length_param& );
    bool operator != ( const sc_length_param&,
			      const sc_length_param& );


// ----------------------------------------------------------------------------
//  CLASS : sc_length_param
//
//  Length parameter type.
// ----------------------------------------------------------------------------

class sc_length_param
{
public:

             sc_length_param();
             sc_length_param( int );
             sc_length_param( const sc_length_param& );
    explicit sc_length_param( sc_without_context );

    sc_length_param& operator = ( const sc_length_param& );

    friend bool operator == ( const sc_length_param&,
                              const sc_length_param& );
    friend bool operator != ( const sc_length_param&,
			      const sc_length_param& );

    int len() const;
    void len( int );

    const std::string to_string() const;

    void print( ::std::ostream& = ::std::cout ) const;
    void dump( ::std::ostream& = ::std::cout ) const;

private:

    int m_len;
};


// ----------------------------------------------------------------------------
//  TYPEDEF : sc_length_context
//
//  Context type for the length parameter type.
// ----------------------------------------------------------------------------

typedef sc_context<sc_length_param> sc_length_context;


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

inline
sc_length_param::sc_length_param() : m_len()
{
    *this = sc_length_context::default_value();
}

inline
sc_length_param::sc_length_param( int len_ ) : m_len(len_)
{
    SC_CHECK_WL_( len_ );
}

inline
sc_length_param::sc_length_param( const sc_length_param& a )
    : m_len( a.m_len )
{}

inline
sc_length_param::sc_length_param( sc_without_context )
    : m_len( SC_DEFAULT_WL_ )
{}


inline
sc_length_param&
sc_length_param::operator = ( const sc_length_param& a )
{
    if( &a != this )
    {
	m_len = a.m_len;
    }
    return *this;
}


inline
bool
operator == ( const sc_length_param& a, const sc_length_param& b )
{
    return ( a.m_len == b.m_len );
}

inline
bool
operator != ( const sc_length_param& a, const sc_length_param& b )
{
    return ( a.m_len != b.m_len );
}


inline
int
sc_length_param::len() const
{
    return m_len;
}

inline
void
sc_length_param::len( int len_ )
{
    SC_CHECK_WL_( len_ );
    m_len = len_;
}


inline
::std::ostream&
operator << ( ::std::ostream& os, const sc_length_param& a )
{
    a.print( os );
    return os;
}

} // namespace sc_dt


#endif

// Taf!
