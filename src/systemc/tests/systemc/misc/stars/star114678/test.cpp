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

/*
Dec/19/00 ulrich

Assignment to a bit-slice of an sc_bv, even with an sc_bv with proper size, does not
compile on Solaris SC5.0. It works for gcc, though.

It works for sc_int and sc_bigint on both compilers.


I am using SystemC 1.0.1 (someone **PLEASE** add 1.0.1 to the Stellar list of release!)

Example:
*/

#include <systemc.h>

int sc_main(int argc, char* arg[]) 
{
  sc_bv<8> bv8 = 3;
  sc_bv<4> bv4 = 3;
  sc_int<8> i8 = 3;
  sc_int<4> i4 = 3;
  sc_bigint<8> bi8=3;
  sc_bigint<4> bi4=3;

  // OK
  bi8.range(5,2) = bi4.range(3,0);
  bi8.range(5,2) = bi4;
  bi8.range(5,2) = 3;
  bi8.range(5,2) = (sc_bigint<4>(3)).range(3,0);

  // OK
  i8.range(5,2) = i4.range(3,0);
  i8.range(5,2) = i4;
  i8.range(5,2) = 3;
  i8.range(5,2) = (sc_int<4>(3)).range(3,0);


  // OK
  bv8.range(5,2) = bv4.range(3,0);

  // OK gcc, error SC5.0
  bv8.range(5,2) = bv4;
  bv8.range(5,2) = (sc_bv<4>(3)).range(3,0);

  return 0;
}
