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

  add_chain_main.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: add_chain_main.cpp,v $
// Revision 1.2  2011/09/05 21:23:35  acg
//  Philipp A. Hartmann: eliminate compiler warnings.
//
// Revision 1.1.1.1  2006/12/15 20:26:13  acg
// systemc_tests-2.3
//
// Revision 1.4  2006/01/24 21:05:23  acg
//  Andy Goodrich: replacement of deprecated features with their non-deprecated
//  counterparts.
//
// Revision 1.3  2006/01/20 00:43:23  acg
// Andy Goodrich: Changed over to use putenv() instead of setenv() to accommodate old versions of Solaris.
//
// Revision 1.2  2006/01/19 00:47:31  acg
// Andy Goodrich: Changes for the fact signal write checking is enabled.
//

#include "common.h"
#include "add_chain.h"
#include "add_chain_tb.h" 	/** Definition of testbench Structure **/

int
sc_main(int ac, char *av[])
{
  sc_clock clk( "CLOCK", 20, SC_NS, 0.5, 10, SC_NS); // Clock function
  testbench tb1("TB1", clk );	// Testbench Instance
  sc_start();	 // Simulation runs forever 
					 // due to negative value
  return 0;
}
