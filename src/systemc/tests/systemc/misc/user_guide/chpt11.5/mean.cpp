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

  mean.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename mean.cc */
/* This is the implementation file for synchronous process `mean' */

#include "mean.h"

void calculate_geometric_mean(const sc_signed& a, const sc_signed& b, 
			      const sc_signed& c, const sc_signed& d,
			      sc_signed& mean){}

void calculate_harmonic_mean(const sc_signed& a, const sc_signed& b, 
			      const sc_signed& c, const sc_signed& d,
			      sc_signed& mean){}

void mean::entry()
{
  sc_signed i(24);  // Will hold input a
  sc_signed j(24);  // Will hold input b
  sc_signed k(24);  // Will hold input c
  sc_signed l(24);  // Will hold input d

  sc_signed arith(24);    // Arithmetic mean
  sc_signed geom(24);     // Geometric mean
  sc_signed harmonic(24); // Harmonic mean

  while (true) {
    // read all inputs
    do { wait(); } while (input_available != true);
    send_data.write(true);
    wait();
    i = in.read().to_int();
    wait();
    j = in.read().to_int();
    wait();
    k = in.read().to_int();
    wait();
    l = in.read().to_int();
    send_data.write(false);
    
    // Calculate arithmetic mean
    arith = (i + j + k + l) / 4;
    // Calculate geometric mean
    calculate_geometric_mean(i, j, k, l, geom);
    // Calculate harmonic mean
    calculate_harmonic_mean(i, j, k, l, harmonic);

    // write all outputs
    output_ready.write(true);
    do { wait(); } while (receiver_ready != true);
    wait();
    out.write(arith);
    wait();
    out.write(geom);
    wait();
    out.write(harmonic);
    wait();
    output_ready.write(false);
  }
} // end of entry function

// Just to make the compiler shut up

int sc_main(int ac, char *av[])
{
   return 0;
}
