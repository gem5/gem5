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

// test of process initialization -- the dont_initialize() method

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_in_clk clk;

    void write( const char* msg )
    {
        cout << sc_time_stamp() << " " << msg << endl;
    }

    void method_a()
    {
        write( "method_a" );
    }

    void thread_a()
    {
        while( true ) {
            write( "thread_a" );
            wait();
        }
    }

    void cthread_a()
    {
        while( true ) {
            write( "cthread_a" );
            wait();
        }
    }

    SC_CTOR( mod_a )
    {
        SC_METHOD( method_a );
        sensitive << clk.neg();
        SC_THREAD( thread_a );
        sensitive << clk.neg();
        SC_CTHREAD( cthread_a, clk.neg() );
    }
};

SC_MODULE( mod_b )
{
    sc_in_clk clk;

    void write( const char* msg )
    {
        cout << sc_time_stamp() << " " << msg << endl;
    }

    void method_b()
    {
        write( "method_b" );
    }

    void thread_b()
    {
        while( true ) {
            write( "thread_b" );
            wait();
        }
    }

    void cthread_b()
    {
        while( true ) {
            write( "cthread_b" );
            wait();
        }
    }

    SC_CTOR( mod_b )
    {
        SC_METHOD( method_b );
        sensitive << clk.neg();
        dont_initialize();
        SC_THREAD( thread_b );
        sensitive << clk.neg();
        dont_initialize();
        SC_CTHREAD( cthread_b, clk.neg() );
        dont_initialize();
    }
};

int
sc_main( int, char*[] )
{
    sc_clock clk;

    mod_a a( "a" );
    mod_b b( "b" );

    a.clk( clk );
    b.clk( clk );

    sc_start( 3, SC_NS );

    return 0;
}
