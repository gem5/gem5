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

  peripheral.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "peripheral.h"

void peripheral::entry() {
  unsigned int buffer_in = 0;
  unsigned int buffer_out = 0;
  wait();

  while(true) {
    unsigned int addr = mem_addr.read().to_uint();
    unsigned int data = mem_data_out.read().to_uint();

    if(!mem_wr_n.read() && (addr==0x10)) {
      // write
      cout << "peripheral: receive " << data << endl;
      buffer_in = data;
      wait(100);
      buffer_out = buffer_in;
    }

    if(!mem_rd_n.read() && (addr==0x11)) {
      // read
      mem_data_in.write( sc_bv<8>( buffer_out ) );
      cout << "peripheral: send " << buffer_out << endl;
      wait();
    }
    wait();
  }
}
