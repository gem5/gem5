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

  test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_signal<sc_logic> posedge and negedge

#include "systemc.h"

SC_MODULE( pro )
{
    sc_out<sc_logic> out;

    void main_action()
    {
        int i = 0;
        while( true ) {
            sc_logic tmp( i );
            out.write( tmp );
            cout << sc_time_stamp() << ": " << tmp.to_char() << endl;
            wait( 3, SC_NS );
            i = (i + 1) % 4;
        }
    }

    SC_CTOR( pro )
    {
        SC_THREAD( main_action );
    }
};

SC_MODULE( con )
{
    sc_in<sc_logic> in;

    void pos_action()
    {
        cout << sc_time_stamp() << ": posedge" << endl;
    }

    void neg_action()
    {
        cout << sc_time_stamp() << ": negedge" << endl;
    }

    SC_CTOR( con )
    {
        SC_METHOD( pos_action );
        sensitive << in.pos();
        dont_initialize();
        SC_METHOD( neg_action );
        sensitive << in.neg();
        dont_initialize();
    }
};

int
sc_main( int, char*[] )
{
    sc_signal<sc_logic> sig;
    pro p( "p" );
    con c( "c" );
    p.out( sig );
    c.in( sig );
    sc_start( 100, SC_NS );
    return 0;
}
