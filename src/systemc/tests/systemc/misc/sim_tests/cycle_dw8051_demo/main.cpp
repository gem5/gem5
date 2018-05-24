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

  main.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

//***************************************************************************
// FILE: main.cc
// 
// AUTHOR: Luc Semeria    September, 21, 1998
//
// ABSTRACT: main (instanciates dw8051 cycle-accurate model and 
//           and peripheral on memory bus
//
// MODIFICATION HISTORY:
//         Luc Semeria: 9/21/98 created
//
//***************************************************************************
// 
// DESCRIPTION
//
//   This program tests the communication between the cycle-accurate model
//   and the peripheral on the memory bus
//
//   The peripheral copies the input after a given number of clocks
//
//   Reference: Synopsys DesignWare DW8051 MacroCell Databook
//
//***************************************************************************
#include "cycle_model.h"
#include "peripheral.h"
// #define DEBUG
char *hex_file_name;


void usage() {
  fprintf(stdout,"\nusage: hw_sim.x [-a] filename.hex\n"); 
  exit(-1);
}

void parse_arg(int argc, char *argv[]) {
  extern bool ALL_CYCLES;
  int i = 1;

  ALL_CYCLES = 0;

#if 0
  // parse -a
  if(argc<=i) 
    usage();
    
  if(!strcmp(argv[i],"-a")) {
    ALL_CYCLES = 1;
    i++;
  }	
 
  // parse file name
  if(argc<=i) 
    usage();
  
  hex_file_name=(char *)malloc(strlen(argv[i])*sizeof(char));  
  strcpy(hex_file_name,argv[i]);
  i++;

  // no more param
  if(i<argc)
    usage();
#else
  hex_file_name = (char*)malloc(50*sizeof(char));
  strcpy(hex_file_name,"cycle_dw8051_demo/test.hex");
#endif
  return;
}

//-------------------------------------------------------------------------
// int sc_main(int argc, char **argv)
//
//-------------------------------------------------------------------------
int sc_main(int argc, char *argv[])
{
  // clock -------------------------------------------------------------------
  sc_clock             clock  ("CLOCK", 40, SC_NS, 0.5); // assume 25MHz

    // signals for read/write to External RAM/ROM
  signal_bool_vector16    mem_addr("mem_addr");          // address bus
  signal_bool_vector8     mem_data_out("mem_data_out");  // data out bus
  signal_bool_vector8     mem_data_in("mem_data_in");    // data in bus
  sc_signal<bool>         mem_wr_n("mem_wr_n");          // write strobe
  sc_signal<bool>         mem_rd_n("mem_rd_n");          // read enable sampled
  sc_signal<bool>         mem_pswr_n("mem_pswr_n");      // write enable (ROM)
  sc_signal<bool>         mem_psrd_n("mem_psrd_n");      // read enable (ROM)
  sc_signal<bool>         mem_ale("mem_ale");            // ext latch enable
  sc_signal<bool>         mem_ea_n("mem_ea_n");          // ext prog mem enable
  
  // sc_signal<bool>      port_pin_reg_n(); // select read from ext reg or pin
  sc_signal<bool>         p0_mem_reg_n("p0_mem_reg_n");  // select port reg
  sc_signal<bool>         p0_addr_data_n("p0_addr_data_n");// select data
  sc_signal<bool>         p2_mem_reg_n("p2_mem_reg_n");  // cf p0_mem_reg_n

  parse_arg(argc, argv);
    
  cycle_model CYCLE_MODEL("dw8051",
			  clock,
			  hex_file_name,        // name of the hex file

			  mem_addr,
			  mem_data_out,
			  mem_data_in,
			  mem_wr_n,
			  mem_rd_n,
			  mem_pswr_n,
			  mem_psrd_n,
			  mem_ale,
			  mem_ea_n,
			  
			  // port_pin_reg_n,
			  p0_mem_reg_n,
			  p0_addr_data_n,
			  p2_mem_reg_n);

  peripheral PERIPHERAL("peripheral",
		       clock,
		       mem_addr,
		       mem_data_out,
		       mem_data_in,
		       mem_wr_n,
                       mem_rd_n,
                       mem_pswr_n,
                       mem_psrd_n,
                       mem_ale,
                       mem_ea_n,
                       
                       p0_mem_reg_n,
                       p0_addr_data_n,
                       p2_mem_reg_n);
  


  sc_start();
  return 0;
}
