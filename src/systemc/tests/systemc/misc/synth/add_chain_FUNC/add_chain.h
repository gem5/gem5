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

  add_chain.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#define LIMIT 	257       // Last stimulus vector memory address location
#define WIDTH 	8         // Width of stimulus vector
#define LATENCY 3	  // Latency of sum
 


extern bool_vector8 mem[];

extern void f_RESET_STIM 	(const char*, 
			  	sc_clock&, 
			  	sc_signal<bool>&, 
			  	sc_signal<bool>&, 
			  	sc_signal<int>& );

extern void f_DATA_GEN 		(const char*, 
				sc_clock&, 
				const sc_signal<bool>&, 
				signal_bool_vector8&, 
				sc_signal<int>& );

extern void f_add_chain 	(const char*, 
				sc_clock&, 
				const sc_signal<bool>&, 
				const signal_bool_vector8&, 
				signal_bool_vector4&, 
				sc_signal<bool>& );

extern void f_DISPLAY 		(const char*,
                		const sc_signal<bool>&,
                		const signal_bool_vector8&,
                		const signal_bool_vector4&);

