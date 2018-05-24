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

  io_controller.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*############################################################################
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
//#include "mii_if.h"
//#include "mbdatm.h"   

// class semaphore {
   
//    bool value;
   
//    public:
//       semaphore();
//       void P();
//       void V();
//       bool get_value();
// };

//void sc_trace(sc_trace_file *, const semaphore&, const std::string&);

#ifdef LOGGING
/* stream for logging */
extern ofstream flog;
#endif

#define MII_FIFO_SIZE 400
#define SCAN_INTERVAL 200000   // 200 us
#define NS *1e-9

SC_MODULE(io_controller_m){ 

   /* ports */
   sc_in_clk clk_i486_if;
   
   sc_out<sc_uint<30> > addr30_o;
   sc_inout<sc_uint<32> > data32_i;
   sc_out<sc_uint<32> > data32_o;
   sc_out<bool> ads_n_o;
   sc_out<bool> wr_n_o;
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
   sc_signal<sc_uint<32> > mux_data32;
   sc_signal<sc_uint<32> > in_fifo_data32;
   sc_signal<sc_uint<32> > out_fifo_data32;
   sc_signal<sc_uint<32> > control_data32;
   sc_signal<bool> out_fifo_en;
   sc_signal<bool> out_fifo_act;
   sc_signal<bool> in_fifo_en;
   sc_signal<bool> control_en;
   sc_signal<bool> out_fifo_reset;
   
   /* variables */
   sc_uint<32> addr_tx_frame_ptr;
   sc_uint<32> rx_ptr_array;
   sc_signal<bool> value;
   
   void P();
   void V();
   bool get_value();
   //   semaphore sem;
   
   /* modules */
//    mux_m *mux;
//    shifter_m *shifter;
//    out_fifo_m *out_fifo;
//    in_fifo_m *in_fifo;
   
   SC_CTOR(io_controller_m){ 
      
      SC_CTHREAD(control_read, clk_i486_if.pos());
      
//       mux = new mux_m("mux");
//       mux->clk(clk_mii);
//       mux->data4_o(mii_data4_o);
//       mux->data32_i(mux_data32);
//       mux->en_i(out_fifo_act);
//       mux->en_o(mii_en_o);
      
//       shifter = new shifter_m("shifter");
//       shifter->clk(clk_mii);
//       shifter->data32_o(in_fifo_data32);
//       shifter->data4_i(mii_data4_i);
//       shifter->en_i(mii_en_i);
//       shifter->en_o(in_fifo_en);
      
//       out_fifo = new out_fifo_m("out_fifo");
//       out_fifo->clk_out(clk_mii);
//       out_fifo->clk_in(clk_i486_if);
//       out_fifo->data32_o(mux_data32);
//       out_fifo->data32_i(out_fifo_data32);
//       out_fifo->en_i(out_fifo_en);
//       out_fifo->act_o(out_fifo_act);
//       out_fifo->reset(out_fifo_reset);
      
//       in_fifo = new in_fifo_m("in_fifo");
//       in_fifo->clk_out(clk_i486_if);
//       in_fifo->clk_in(clk_mii);
//       in_fifo->data32_o(control_data32);
//       in_fifo->data32_i(in_fifo_data32);
//       in_fifo->en_i(in_fifo_en);
//       in_fifo->en_o(control_en);
      
//       /* Initialize */
//       in_fifo_data32 = (sc_uint<32>) 0;
//       mux_data32 = (sc_uint<32>) 0;
//       out_fifo_en = 0;
//       out_fifo_act = 0;      
//       //en_o = 0;
//       in_fifo_en = 0;
//       control_en = 0;
//       out_fifo_reset = 0;
      
   } 
   void control_write();
   void control_read();
   sc_uint<32> read_from_memory(sc_uint<32>);
   void write_into_memory(sc_uint<32>, sc_uint<32>);

}; 

#endif

