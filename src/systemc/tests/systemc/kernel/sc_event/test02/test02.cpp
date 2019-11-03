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

  test02.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of immediate event notification

#include "systemc.h"

SC_MODULE( source )
{
    sc_in_clk clk;
    sc_out<int> out;

    void main_action()
    {
        int a = 0;
        while( true ) {
            wait();
            out = a ++;
        }
    }

    SC_CTOR( source )
    {
        SC_THREAD( main_action );
        sensitive << clk.pos();
    }
};

SC_MODULE( sink )
{
    sc_in_clk clk;
    sc_in<int> in;

    sc_event e;

    void main_action()
    {
        int a;
        while( true ) {
            wait();
            cout << sc_delta_count() << " -- " << in.read() << endl;
            a = in.read();
            if( ( a % 3 ) == 0 ) {
                e.notify();
            }
        }
    }

    void other_action()
    {
        while( true ) {
            wait( e );
            cout << sc_delta_count() << " AA " << in.read() << endl;
            wait( e | e );  // same as wait( e )
            cout << sc_delta_count() << " BB " << in.read() << endl;
            wait( e & e );  // same as wait( e )
            cout << sc_delta_count() << " CC " << in.read() << endl;
            wait( e | e | e );  // same as wait( e )
            cout << sc_delta_count() << " DD " << in.read() << endl;
            wait( e & e & e );  // same as wait( e )
            cout << sc_delta_count() << " EE " << in.read() << endl;
            wait( e & clk->negedge_event() );
            cout << sc_delta_count() << " FF " << in.read() << endl;
            wait( e | clk->negedge_event() );
            cout << sc_delta_count() << " GG " << in.read() << endl;
        }
    }

    SC_CTOR( sink )
    {
        SC_THREAD( main_action );
        sensitive << clk.pos();
        SC_THREAD( other_action );
    }
};

int sc_main( int, char** )
{
    sc_clock clk;

    sc_signal<int> sig;
    source src( "src" );
    sink snk( "snk" );

    src.clk( clk );
    src.out( sig );
    snk.clk( clk );
    snk.in( sig );

    sc_start( 100, SC_NS );

    return 0;
}
