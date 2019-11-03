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

  io_controller1.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*
############################################################################
#  Siemens AG                        copyright 2000
#                                    All Rights Reserved
#
#  File name : io_controller.h
# 
#  Title     : I/O-Controller
#  
#  Purpose   : definitions for I/O-Controller-module
#
#  Author    : Hannes Muhr
#              PSE EZE MSA
#
##############################################################################
#  Modification History :
#
#   
##############################################################################*/

#ifndef IO_CONTROLLER_INC
#define IO_CONTROLLER_INC

#ifdef LOGGING
#include <fstream>
#endif
#include "systemc.h"


#ifdef LOGGING
/* stream for logging */
extern ofstream flog;
#endif

#define SCAN_INTERVAL 200000   // 200 us
#define NS *1e-9

#define MII_FRAME_SIZE 400

SC_MODULE(io_controller_m){ 

   /* ports */
   sc_in_clk clk_i486_if;
   
   sc_out<sc_uint<30> > addr30_o1;
   sc_out<sc_uint<30> > addr30_o2;
   sc_inout<sc_uint<32> > data32_i;
   sc_out<sc_uint<32> > data32_o1;
   sc_out<sc_uint<32> > data32_o2;
   sc_out<bool> ads_n_o1;
   sc_out<bool> ads_n_o2;
   sc_out<bool> wr_n_o1;
   sc_out<bool> wr_n_o2;
   sc_in<bool> rdy_n_i;
   sc_in<bool> ar_i;
   sc_in<bool> res_n_i;
   
   sc_out<sc_uint<4> > mii_data4_o;
   sc_out<bool> mii_en_o;
   sc_in<sc_uint<4> > mii_data4_i;
   sc_in<bool> mii_en_i;
   sc_in<bool> mii_coll_det;
   sc_in_clk clk_mii;
   
   /* signals */
   sc_signal<bool> start_mux;
   sc_signal<bool> ready_mux;
   sc_signal<bool> start_read;
   sc_signal<bool> out_fifo_reset;
   
   /* variables */
   sc_uint<32> addr_tx_frame_ptr;
   sc_uint<32> rx_ptr_array;
   sc_signal<bool>  sem1;      // mutual exclusion for i486-if
   sc_signal<bool>  sem2;      // mutual exclusion for i486-if
   sc_uint<32> shared_mem1[MII_FRAME_SIZE];  // for write
   sc_uint<32> shared_mem2[MII_FRAME_SIZE];  // for read
   
   SC_CTOR(io_controller_m){ 
      
      SC_CTHREAD(control_write, clk_i486_if.pos());
      //reset_signal_is(mii_coll_det, true);
	  reset_signal_is(res_n_i, false);
      
      SC_CTHREAD(control_read, clk_i486_if.pos());
      
      SC_CTHREAD(mux, clk_mii.pos());
      SC_CTHREAD(shift, clk_mii.pos());
      
      
      /* Initialize */
      start_mux = 0;
      ready_mux = 0;      
      start_read = 0;
      out_fifo_reset = 0;
   
      sem1 = false;
      sem2 = false;
      // init shared memory
      for (int i=0; i < MII_FRAME_SIZE; i++)
         shared_mem1[i] = shared_mem2[i] = 0;   
   } 
   void control_write();
   void control_read();
   void mux();
   void shift();
   sc_uint<32> read_from_memory0(sc_uint<32>);
   sc_uint<32> read_from_memory1(sc_uint<32>);
   void write_into_memory0(sc_uint<32>, sc_uint<32>);
   void write_into_memory1(sc_uint<32>, sc_uint<32>);
   
}; 

#endif
