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

// test of checks in the sc_[u]int classes

#include "systemc.h"

int
sc_main( int, char*[] )
{
    // check_length for sc_int_base

    try {
        sc_int<-3> a;
    }
    catch( sc_report ) {
        cout << "caught exception for sc_int<-3>\n";
    }

    try {
        sc_int<0>  a;
    }
    catch( sc_report ) {
        cout << "caught exception for sc_int<0>\n";
    }

    try {
        sc_int<100> a;
    }
    catch( sc_report ) {
        cout << "caught exception for sc_int<100>\n";
    }


    // check_index for sc_int_base

    try {
        sc_int<8> a = 42;
        cout << a[-1] << "\n";
    }
    catch( sc_report ) {
        cout << "caught exception for sc_int<8>[-1]\n";
    }

    try {
        sc_int<8> a = 42;
        cout << a[8] << "\n";
    }
    catch( sc_report ) {
        cout << "caught exception for sc_int<8>[8]\n";
    }


    // check_range for sc_int_base

    try {
        sc_int<8> a = 42;
        cout << a( 3, -1 ) << "\n";
    }
    catch( sc_report ) {
        cout << "caught exception for sc_int<8>( 3, -1 )\n";
    }

    try {
        sc_int<8> a = 42;
        cout << a( 8, 4 ) << "\n";
    }
    catch( sc_report ) {
        cout << "caught exception for sc_int<8>( 8, 4 )\n";
    }

    try {
        sc_int<8> a = 42;
        cout << a( 0, 3 ) << endl;
    }
    catch( sc_report ) {
        cout << "caught exception for sc_int<8>( 0, 3 )\n";
    }


    // check_length for sc_int_concref<T1,T2>

    try {
        sc_int<42> a;
        cout << ( a, a ) << "\n";
    }
    catch( sc_report ) {
        cout << "caught exception for ( sc_int<42>, sc_int<42> )\n";
    }


    // check_length for sc_uint_base

    try {
        sc_uint<-3> a;
    }
    catch( sc_report ) {
        cout << "caught exception for sc_uint<-3>\n";
    }

    try {
        sc_uint<0>  a;
    }
    catch( sc_report ) {
        cout << "caught exception for sc_uint<0>\n";
    }

    try {
        sc_uint<100> a;
    }
    catch( sc_report ) {
        cout << "caught exception for sc_uint<100>\n";
    }


    // check_index for sc_uint_base

    try {
        sc_uint<8> a = 42;
        cout << a[-1] << "\n";
    }
    catch( sc_report ) {
        cout << "caught exception for sc_uint<8>[-1]\n";
    }

    try {
        sc_uint<8> a = 42;
        cout << a[8] << "\n";
    }
    catch( sc_report ) {
        cout << "caught exception for sc_uint<8>[8]\n";
    }


    // check_range for sc_uint_base

    try {
        sc_uint<8> a = 42;
        cout << a( 3, -1 ) << "\n";
    }
    catch( sc_report ) {
        cout << "caught exception for sc_uint<8>( 3, -1 )\n";
    }

    try {
        sc_uint<8> a = 42;
        cout << a( 8, 4 ) << "\n";
    }
    catch( sc_report ) {
        cout << "caught exception for sc_uint<8>( 8, 4 )\n";
    }

    try {
        sc_uint<8> a = 42;
        cout << a( 0, 3 ) << endl;
    }
    catch( sc_report ) {
        cout << "caught exception for sc_uint<8>( 0, 3 )\n";
    }


    // check_length for sc_uint_concref<T1,T2>

    try {
        sc_uint<42> a;
        cout << ( a, a ) << "\n";
    }
    catch( sc_report ) {
        cout << "caught exception for ( sc_uint<42>, sc_uint<42> )\n";
    }

    return 0;
}
