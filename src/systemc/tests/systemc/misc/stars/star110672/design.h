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

  design.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


SC_MODULE(design) {
  // ports
  sc_in_clk    clock;
  sc_in<bool > take_the_data;
  sc_in<bool > data_request;
  sc_out<bool > data_ready;
  sc_out<bool > write_in_is_done;
  sc_in <DATA_TYPE > input;
  sc_out<DATA_TYPE > output;
  sc_signal<DATA_TYPE > ring_buffer[RING_BUFFER_SIZE];
  sc_signal<INDEX_TYPE > read_pointer;
  sc_signal<INDEX_TYPE > write_pointer;
  // processes
  void write_in_fifo();
  void read_out_fifo();
  SC_CTOR(design) {
    SC_CTHREAD(write_in_fifo, clock.pos() );
    SC_CTHREAD(read_out_fifo, clock.pos() );
    // bad reset
    write_pointer = (INDEX_TYPE)0 ;
    read_pointer = (INDEX_TYPE)0 ;
  }  

};
