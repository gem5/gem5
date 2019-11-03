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

  main.cpp -- Test consistency check of SC_DEFAULT_WRITER_POLICY setting
              (see test-unchecked.cpp)

  Original Author: Philipp A. Hartmann, OFFIS, 2013-11-05

*****************************************************************************/

#ifndef SC_DEFAULT_WRITER_POLICY
# define SC_DEFAULT_WRITER_POLICY SC_MANY_WRITERS
#endif
#include <systemc>

int sc_main(int,char*[])
{
  SC_REPORT_INFO( "sc_api_version", "in sc_main" );
  return 0;
}
