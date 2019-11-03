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

  test05.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of the child objects of a module and the simcontext

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_in_clk clk;
    sc_out<int> out;

    int a;

    void main_action()
    {
        out = ++ a;
    }

    SC_CTOR( mod_a )
    {
        SC_METHOD( main_action );
        sensitive << clk.pos();
        a = 0;
    }
};

SC_MODULE( mod_b )
{
    sc_in<int> in;

    void main_action()
    {
        while( true ) {
            wait();
            cout << in.read() << endl;
        }
    }

    SC_CTOR( mod_b )
    {
        SC_THREAD( main_action );
        sensitive << in;
    }
};

SC_MODULE( mod_c )
{
    sc_in_clk clk;

    void main_action()
    {
        while( true ) {
            cout << sc_time_stamp() << endl;
            wait();
        }
    }

    mod_a a;
    mod_b b;
    sc_signal<int> sig;

    SC_CTOR( mod_c )
    : a( "a" ), b( "b" )
    {
        SC_CTHREAD( main_action, clk.neg() );
        a.clk( clk );
        a.out( sig );
        b.in( sig );
    }
};

void
print_child_objects( const ::std::vector<sc_object*>& child_objects_ )
{
    int size = child_objects_.size();
    cout << "***\n";
    for( int i = 0; i < size; ++ i ) {
        sc_object* object = child_objects_[i];
        cout << object->kind() << "  " << object->name() << endl;
    }
}

int
sc_main( int, char*[] )
{
    mod_a a( "a" );
    mod_b b( "b" );
    sc_clock clk;
    sc_signal<int> sig;

    a.clk( clk );
    a.out( sig );
    b.in( sig );

    mod_c c( "c" );
    c.clk( clk );

    sc_start(1, SC_NS);

    print_child_objects( sc_get_top_level_objects() );
    print_child_objects( a.get_child_objects() );
    print_child_objects( b.get_child_objects() );
    print_child_objects( c.get_child_objects() );
    print_child_objects( c.a.get_child_objects() );
    print_child_objects( c.b.get_child_objects() );

    return 0;
}
