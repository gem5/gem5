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

  test01.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of signals with user-defined datatypes
// -- operator =, operator ==, and operator << must be defined
// -- sc_trace() does not have to be defined (unless used)
// -- here, sc_trace() is not defined and not used

#include "systemc.h"

class my_datatype
{
public:
    my_datatype()
        : m_val( 0 ) {}
    my_datatype( int val_ )
        : m_val( val_ ) {}
    my_datatype( const my_datatype& a )
        : m_val( a.m_val ) {}
    ~my_datatype()
        {}
    my_datatype& operator = ( const my_datatype& a )
        { m_val = a.m_val; return *this; }
    friend bool operator == ( const my_datatype& a, const my_datatype& b )
        { return ( a.m_val == b.m_val ); }
    void print( ostream& os ) const
        { os << m_val; }
private:
    int m_val;
};

ostream&
operator << ( ostream& os, const my_datatype& a )
{
    a.print( os );
    return os;
}

int
sc_main( int, char*[] )
{
    my_datatype a( 123 );
    a.print( cout );
    cout << endl;

    sc_signal<my_datatype> sig;

    return 0;
}
