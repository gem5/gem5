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

  test03.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_fifo ports - interface access shortcut methods

#include "systemc.h"

SC_MODULE( writer )
{
    // port(s)
    sc_fifo_out<int> out;

    // process(es)
    void main_action()
    {
        int val = 0;
        while( true ) {
            wait( 10, SC_NS ); // wait for 10 ns
            cout << "writer: blocking write\n";
            for( int i = 0; i < 20; i ++ ) {
                out.write( val ++ ); // blocking write
            }
            wait( 10, SC_NS );
            cout << "writer: " << out.num_free() << " free spaces\n";
            cout << "writer: non-blocking write\n";
            for( int i = 0; i < 20; i ++ ) {
                while( ! out.nb_write( val ++ ) ) { // non-blocking write
                    cout << "writer: waiting\n";
                    wait( 1, SC_NS );
                }
            }
        }
    }

    SC_CTOR( writer )
    {
        SC_THREAD( main_action );
    }
};

SC_MODULE( reader )
{
    // port(s)
    sc_fifo_in<int> in;

    // process(es)
    void main_action()
    {
        int val;
        while( true ) {
            wait( 10, SC_NS ); // wait for 10 ns
            cout << "reader: blocking read 1\n";
            for( int i = 0; i < 15; i ++ ) {
                in.read( val ); // blocking read
                cout << val << endl;
            }
            wait( 10, SC_NS );
            cout << "reader: " << in.num_available() << " available samples\n";
            cout << "reader: blocking read 2\n";
            for( int i = 0; i < 15; i ++ ) {
                cout << in.read() << endl; // blocking read
            }
            wait( 10, SC_NS );
            cout << "reader: " << in.num_available() << " available samples\n";
            cout << "reader: non-blocking read\n";
            for( int i = 0; i < 15; i ++ ) {
                while( ! in.nb_read( val ) ) { // non-blocking read
                    cout << "reader: waiting\n";
                    wait( 1, SC_NS );
                }
                cout << val << endl;
            }
        }
    }

    SC_CTOR( reader )
    {
        SC_THREAD( main_action );
    }
};

int sc_main( int, char*[] )
{
    // sc_clock c;

    // declare channel(s)
    sc_fifo<int> fifo( 10 );

    // instantiate block(s) and connect to channel(s)
    writer w( "writer" );
    reader r( "reader" );

    w.out( fifo );
    r.in( fifo );

    // run the simulation
    sc_start( 100, SC_NS );

    return 0;
}
