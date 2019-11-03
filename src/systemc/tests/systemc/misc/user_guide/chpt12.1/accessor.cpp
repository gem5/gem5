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

  accessor.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename accessor.cc */
/* This is the implementation file for synchronous process 'accessor' */

#include "accessor.h"

void accessor::entry()
{
  int addr;
  int datao;
  int datai;

  addr = 10;
  datao = 0xdeadbeef;

  while (true) {
    // Write memory location first
    chip_select.write(true);
    write_enable.write(true);
    address.write(addr);
    dataout.write(datao);
    cout << "Accessor: Data Written = " << hex << datao << " at address "
         << hex << addr << endl;
    wait(); // To make all the outputs appear at the interface

    // some process functionality not shown here during which chip
    // chip select is deasserted
    chip_select.write(false);
    dataout.write(0);
    wait();

    // Now read memory location
    chip_select.write(true);
    write_enable.write(false);
    address.write(addr);
    wait(); // To make all the outputs appear at the interface

    datai = datain.read().to_int();
    cout << "Accessor: Data Read = " << hex << datai << " from address "
         << hex << addr << endl;
    chip_select.write(false);
    wait();

    addr++;
    datao++;
  }
} // end of entry function

