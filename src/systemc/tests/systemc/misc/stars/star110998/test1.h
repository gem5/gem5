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

  test1.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

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
   
   SC_CTOR(io_controller_m)
     { 
      
       SC_CTHREAD(control_write, clk_i486_if.pos());
     
   } 
   void control_write();

}; 
