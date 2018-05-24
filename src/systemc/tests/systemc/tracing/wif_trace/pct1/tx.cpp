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

  tx.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename driver.cc */
/* This is a SIO transmitter */

#include "tx.h"

void sio_tx::byte (char c)
{
  bool parity = false;
  int n;

  // Start bit
  tx.write(false);
  wait();
  
  // 8 data bits
  for (n=0; n<8; n++) {
    if (c & 0x1) parity=!parity;
    tx.write (c & 0x1);
    c = c >>1;
    wait();
  }
  
  // parity bit
  tx.write(parity);
  wait();
  
  // stop bits;
  if (two_stop_bits) {
    tx.write(true);
    wait();
    tx.write(true);
    wait();
  } else {
    tx.write(true);
    wait();
  }
} // end of entry function


void sio_tx::mark (int cycles)
{
  int n;

  for (n=0; n<cycles; n++) {
    tx.write(true);
    wait();
  }
}


void sio_tx::entry()
{
  while (1) {
    mark(5);
    byte((char)0x5a);
    byte((char)0xff);
    mark(1);
    byte((char)0xab);
  }
} // end of entry function

