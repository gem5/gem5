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

  star111004.cpp -- 

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
#  File name : io_controller.cpp
# 
#  Title     : I/O-Controller
#  
#  Purpose   : functionality for I/O-Controller-module 
#
#  Author    : Hannes Muhr
#              PSE EZE MSA
#
##############################################################################
#  Modification History :
#
#   
##############################################################################*/

#include "systemc.h"
#include "io_controller.h"

#define MII_FRAME_SIZE 400

// ::semaphore(){
   
//    value = false;
// }

void io_controller_m::P(){

   while (value) wait();
   value = true;
}
         
void io_controller_m::V(){

   /*if (!value) {
      cerr << "V-operation on semaphore that is not P'd\n";
      exit(-1);
   }*/
   value = false;
}

bool io_controller_m::get_value(){
   
   return value;
}

/*void sc_trace(sc_trace_file *tf, const semaphore& sem, const std::string& str){
   
   sc_trace(tf, sem.get_value(), str);
}*/
         
sc_uint<32> io_controller_m::read_from_memory(sc_uint<32> mp){

   // read from mbdatm-memory over i486-IF
   
   addr30_o = mp >> 2;
   ads_n_o = 0;
   wr_n_o = 0;
   wait();
   ads_n_o = 1;
   do { wait(); } while (rdy_n_i == 1);
   sc_uint<32> data = data32_i.read();
   wr_n_o = 1;
   addr30_o = 0;
   return data;      
}

void io_controller_m::write_into_memory(sc_uint<32> mp, sc_uint<32> data){

   addr30_o = mp >> 2;
   ads_n_o = 0;
   wr_n_o = 1;
   wait();
   ads_n_o = 1;
   data32_o = data;
   do { wait(); } while (rdy_n_i == 1);
   wr_n_o = 1;
   addr30_o = 0;
   data32_o = 0;
}

void io_controller_m::control_write(){
   sc_uint<32> word_cnt;
   
   if (!res_n_i.read()){
      do { wait(); } while (!res_n_i);
   
      // initialize
      
      // wait for 1. AR (HWS-Daten)
      do { wait(); } while (!ar_i);
      sc_uint<32> hws = data32_i.read();
      
      wait();

      // wait for 2. AR (ACB-Pointer)
      do { wait(); } while (!ar_i);
      addr_tx_frame_ptr = data32_i.read();
      
   } 
  /* else if (mii_coll_det){
      out_fifo_reset = 1;
      out_fifo_en = 0;
      out_fifo_data32 = (sc_uint<32>) 0;
      
      // reset i486-IF
      addr30_o = 0;
      data32_io = 0;
      ads_n_o = 1;
      wr_n_o = 1; 
   
      // release Semaphore if it is set
      sem.V();
      
      wait();
      out_fifo_reset = 0;
   }*/
   
   while(true){
      // normally Attention Request - Signal from MBDATM
      // would wake up IO-Controller to read data from the memory,
      // but the model from Hr. Wahl said: wait for some ms !!!
      
     // wait(unsigned ((SCAN_INTERVAL NS)/40e-9));
      //do { wait(); } while (ar_i);
      
      #ifdef LOGGING
         flog << sc_time_stamp()<<": "<<name()<<"::control_write - Attention Request" << endl;
      #endif
   
      P();
      sc_uint<32> tx_frame_ptr = read_from_memory(addr_tx_frame_ptr);
      if (tx_frame_ptr != 0)
         word_cnt = read_from_memory(tx_frame_ptr+(MII_FIFO_SIZE+1)*4);
      V();
      
      // check, if frame available and frame is full (word_cnt == MII_FRAME_SIZE)
      
      while (tx_frame_ptr != 0 && word_cnt == MII_FRAME_SIZE){
         #ifdef LOGGING
            flog << sc_time_stamp()<<": "<<name()<<"::control_write - writing mii_frame into out_fifo" << endl;
         #endif
         
         
         for (int i = 0; i<MII_FIFO_SIZE; i++){
            // reading from i486-IF and writing into
            // out_fifo is mixed, so read_from_memory could not be applied
            
            P();
            sc_uint<32> data = read_from_memory(tx_frame_ptr+i*4);
            V();
            
            out_fifo_en = 1;
            out_fifo_data32 = data;
            wait();
            out_fifo_en = 0;
            
         }
         
         while (out_fifo_act.read() != 0) wait(2);
         
         // write 0xFFFFFFFF (>MII_FRAME_SIZE) into tx_frame_ptr 
         // to signal software in mbdatm that io-controller has 
         // read out the frames and sent successfully
         P();
         write_into_memory(tx_frame_ptr+(MII_FIFO_SIZE+1)*4, 0xFFFFFFFF);
         V();
         
         // read next frame_pointer and word_cnt from MBDATM
         P();
         tx_frame_ptr = read_from_memory(tx_frame_ptr+MII_FIFO_SIZE*4);
         if (tx_frame_ptr != 0)
            word_cnt = read_from_memory(tx_frame_ptr+(MII_FIFO_SIZE+1)*4);
         V();
         
         
      }
      
   }
}

void io_controller_m::control_read(){

  int arr_ptr = 0;
   
   while (true){
      do { wait(); } while (!control_en);
      #ifdef LOGGING
         flog << sc_time_stamp()<<": "<<name()<<"::control_read " << endl;
      #endif
      
      // read rx_frame_ptr from MBDATM
      P();
      sc_uint<32> rx_frame_ptr = read_from_memory(rx_ptr_array+arr_ptr*4);
      V();
      /*if (rx_frame_ptr == 0){
         cerr << "\nIO-Controller has read NULL-ptr from rx_array in MBDATM\n";
         cerr << "MBDATM did not fill rx_array fast enough\n";
         exit(-1);
      }*/
      if (++arr_ptr == MII_FIFO_SIZE)
         arr_ptr = 0;
      
      // write data from in_fifo into MBDATM-memory
      for (int i = 0; i < MII_FIFO_SIZE-1; i++){
         sc_uint<32> d = control_data32.read();
         // grab the semaphore
         P();
         write_into_memory(rx_frame_ptr + i*4, d);
         // release semaphore
         V();
         do { wait(); } while (!control_en);
         
      }
      // separate last loop because we don't want to wait for
      // another control_en at this time
      sc_uint<32> d = control_data32.read();
      P();
      write_into_memory(rx_frame_ptr + (MII_FIFO_SIZE-1)*4, d);
      V();
      
      // write 0xFFFFFFFF into word_cnt from frame
      // to indicate the software (MBDATM) that frame has been filled
      P();
      write_into_memory(rx_frame_ptr + (MII_FIFO_SIZE+1)*4, 0xFFFFFFFF);
      V();
   }
}

