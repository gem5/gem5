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

  paramadd.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename paramadd.cc */
/* This is the implementation file for synchronous process `paramadd' */

#include "paramadd.h"

void paramadd::entry()
{
  sc_biguint<5>	sum1;
  sc_unsigned  	a1(data_width);
  sc_unsigned  	b1(data_width);
  bool c1;
 
  // Reset handler
  sum.write(0); 
  co.write(0);
  wait();

  while(true) {	 // Main Loop

    // HANDSHAKE
    done.write(0);
    do { wait(); } while (ready != 1);

    // COMPUTATION
    a1 = a.read();
    b1 = b.read();
    c1 = cin.read();

    sum1 = a1 + b1 + c1;

    // sum.write( sum1.range(data_width-1,0) );
    sc_unsigned tmp( data_width );
    tmp = sum1.range( data_width - 1, 0 );
    sum.write( tmp );
    co.write( sum1[data_width] );
    wait( 2 );

    cout  << sc_time_stamp() << "\t : "
          << a1 << " + " 
          << b1 << " + " 
          << cin << " = " << sum1
          << "\t => carry = " << co 
          << " sum = " << sum
          << endl; 


    // HANDSHAKE
    done.write(1);
    wait();
  } 
}

