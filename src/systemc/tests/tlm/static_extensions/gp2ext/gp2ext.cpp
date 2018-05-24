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
#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "tlm.h"

#include "SimpleLTInitiator1_DMI.h"
#include "SimpleLTTarget_ext.h"
#include "extension_adaptors.h"


int sc_main(int argc, char* argv[])
{
  SimpleLTInitiator1_dmi initiator("initiator1", 10, 0x0);
  adapt_gp2ext<32>       bridge("bridge");
  SimpleLTTarget_ext     target("target1");

  initiator.socket(bridge.target_socket);
  bridge.initiator_socket(target.socket);

  sc_core::sc_start();
  sc_core::sc_stop();

  return 0;
}
