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

  star111657.cpp -- 

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
#include "io_controller1.h"


void io_controller_m::mux(){
   
   sc_uint<32> data;
   
   while (true){
      while (start_mux.read() == 0) wait();
      #ifdef LOGGING
         flog << sc_time_stamp()<<": "<<name()<<"::select - enabled" << endl;
      #endif
      mii_en_o = 1;
      for (int i = 0; i < MII_FRAME_SIZE; i++){
         data = shared_mem1[i];
      
         mii_data4_o = data.range(3,0);
         wait();
         mii_data4_o = data.range(7,4);
         wait();
         mii_data4_o = data.range(11,8);
         wait();
         mii_data4_o = data.range(15,12);
         wait();
         mii_data4_o = data.range(19,16);
         wait();
         mii_data4_o = data.range(23,20);
         wait();
         mii_data4_o = data.range(27,24);
         wait();
         mii_data4_o = data.range(31,28);
         wait();
      }
      mii_en_o = 0;
      mii_data4_o = 0;
      ready_mux = 1;
      wait();
      ready_mux = 0;
      wait();
   }
}

void io_controller_m::shift(){
   
    sc_uint<32> data;
   
   while (true){
      while (mii_en_i.read() == false) wait();
      #ifdef LOGGING
         flog << sc_time_stamp()<<": "<<name()<<"::collect - enabled" << endl;
      #endif
      
      for (int i = 0; i < MII_FRAME_SIZE; i++){      
         data.range(3,0) = mii_data4_i;
         wait();
         data.range(7,4) = mii_data4_i;
         wait();
         data.range(11,8) = mii_data4_i;
         wait();
         data.range(15,12) = mii_data4_i;
         wait();
         data.range(19,16) = mii_data4_i;
         wait();
         data.range(23,20) = mii_data4_i;
         wait();
         data.range(27,24) = mii_data4_i;
         wait();
         data.range(31,28) = mii_data4_i;
         shared_mem2[i] = data;
         wait();
      }   
      start_read = 1;
      wait();
      start_read = 0;
      wait();
   }
}
         
sc_uint<32> io_controller_m::read_from_memory0(sc_uint<32> mp){

   // read from mbdatm-memory over i486-IF
   
   addr30_o1 = mp >> 2;
   ads_n_o1  = 0;
   wr_n_o1   = 0;
   wait();
   ads_n_o1 = 1;
   while (rdy_n_i.read() == 1) wait();
   sc_uint<32> data = data32_i.read();
   wr_n_o1  = 1;
   addr30_o1 = 0;
   return data;      
}

sc_uint<32> io_controller_m::read_from_memory1(sc_uint<32> mp){

   // read from mbdatm-memory over i486-IF
   
   addr30_o2 = mp >> 2;
   ads_n_o2  = 0;
   wr_n_o2   = 0;
   wait();
   ads_n_o2 = 1;
   while (rdy_n_i.read() == 1) wait();
   sc_uint<32> data = data32_i.read();
   wr_n_o2  = 1;
   addr30_o2 = 0;
   return data;      
}

void io_controller_m::write_into_memory0(sc_uint<32> mp, sc_uint<32> data){

   addr30_o1 = mp >> 2;
   ads_n_o1 = 0;
   wr_n_o1 = 1;
   wait();
   ads_n_o1 = 1;
   data32_o1 = data;
   while (rdy_n_i.read() == 1) wait();
   wr_n_o1 = 1;
   addr30_o1 = 0;
   data32_o1 = 0;
}

void io_controller_m::write_into_memory1(sc_uint<32> mp, sc_uint<32> data){

   addr30_o2 = mp >> 2;
   ads_n_o2 = 0;
   wr_n_o2 = 1;
   wait();
   ads_n_o2 = 1;
   data32_o2 = data;
   while (rdy_n_i.read() == 1) wait();
   wr_n_o2 = 1;
   addr30_o2 = 0;
   data32_o2 = 0;
}

void io_controller_m::control_write(){
   sc_uint<32> word_cnt;
   
   while (res_n_i.read() == 0) wait();
   
   // initialize
   
   // wait for 1. AR (HWS-Daten)
   while (ar_i.read() == 0) wait();
   sc_uint<32> hws = data32_i.read();
   
   wait();

   // wait for 2. AR (ACB-Pointer)
   while (ar_i.read() == 0) wait();
   addr_tx_frame_ptr = data32_i.read();
   wait();
   
   
   while(true){
      // normally Attention Request - Signal from MBDATM
      // would wake up IO-Controller to read data from the memory,
      // but the model from Hr. Wahl said: wait for some ms !!!
      
      wait(1000);
      
      #ifdef LOGGING
         flog << sc_time_stamp()<<": "<<name()<<"::control_write - Attention Request" << endl;
      #endif
   
      while (sem2) wait(); sem2 = true;  // P-operation
      sc_uint<32> tx_frame_ptr = read_from_memory0(addr_tx_frame_ptr);
      if (tx_frame_ptr != 0)
         word_cnt = read_from_memory0(tx_frame_ptr+(MII_FRAME_SIZE+1)*4);
      sem2 = false;  // V-operation
      
      // check, if frame available and frame is full (word_cnt == MII_FRAME_SIZE)
      
      while (tx_frame_ptr != 0 && word_cnt == MII_FRAME_SIZE){
         #ifdef LOGGING
            flog << sc_time_stamp()<<": "<<name()<<"::control_write - writing mii_frame into out_fifo" << endl;
         #endif
         
         
         for (int i = 0; i<MII_FRAME_SIZE; i++){
            // reading from i486-IF and writing into
            // out_fifo is mixed, so read_from_memory could not be applied
            
            while (sem2) wait(); sem2 = true;  // P-operation
            sc_uint<32> data = read_from_memory0(tx_frame_ptr+i*4);
            sem2 = false;  // V-operation
            
            if (i == 0){
               start_mux = 1;
               shared_mem1[i] = data;
               wait();
               start_mux = 0;
            }
            else {
               shared_mem1[i] = data;
               wait();
            }
            // wait(); ??
         }
         
         while (ready_mux.read() == 0) wait();
         
         // write 0xFFFFFFFF (>MII_FRAME_SIZE) into tx_frame_ptr 
         // to signal software in mbdatm that io-controller has 
         // read out the frames and sent successfully
         while (sem2) wait(); sem2 = true;  // P-operation
         write_into_memory0(tx_frame_ptr+(MII_FRAME_SIZE+1)*4, 0xFFFFFFFF);
         sem2 = false;  // V-operation
         
         // read next frame_pointer and word_cnt from MBDATM
         while (sem2) wait(); sem2 = true;  // P-operation
         tx_frame_ptr = read_from_memory0(tx_frame_ptr+MII_FRAME_SIZE*4);
         if (tx_frame_ptr != 0)
            word_cnt = read_from_memory0(tx_frame_ptr+(MII_FRAME_SIZE+1)*4);
         sem2 = false;  // V-operation
         
         
      }
      
   }
}

void io_controller_m::control_read(){

    int arr_ptr = 0;
   
   while (true){
      while (start_read.read() == 0) wait();
      #ifdef LOGGING
         flog << sc_time_stamp()<<": "<<name()<<"::control_read " << endl;
      #endif
      
      // read rx_frame_ptr from MBDATM
      while (sem1) wait(); sem1 = true;  // P-operation
      sc_uint<32> rx_frame_ptr = read_from_memory1(rx_ptr_array+arr_ptr*4);
      sem1 = false;  // V-operation
      /*if (rx_frame_ptr == 0){
         cerr << "\nIO-Controller has read NULL-ptr from rx_array in MBDATM\n";
         cerr << "MBDATM did not fill rx_array fast enough\n";
         exit(-1);
      }*/
      if (++arr_ptr == MII_FRAME_SIZE)
         arr_ptr = 0;
      
      // write data from in_fifo into MBDATM-memory
      for (int i = 0; i < MII_FRAME_SIZE; i++){
         sc_uint<32> d = shared_mem2[i];
         // grab the semaphore
         while (sem1) wait(); sem1 = true;  // P-operation
         write_into_memory1(rx_frame_ptr + i*4, d);
         // release semaphore
         sem1 = false;  // V-operation
         wait();
         
      }
      
      // write 0xFFFFFFFF into word_cnt from frame
      // to indicate the software (MBDATM) that frame has been filled
      while (sem1) wait(); sem1 = true;  // P-operation
      write_into_memory1(rx_frame_ptr + (MII_FRAME_SIZE+1)*4, 0xFFFFFFFF);
      sem1 = false;  // V-operation
   }
}

