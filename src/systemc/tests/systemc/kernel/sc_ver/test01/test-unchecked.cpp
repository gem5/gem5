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

  test-unchecked.cpp -- force SC_DEFAULT_WRITER_POLICY to SC_MANY_WRITERS
                        (for consistency checking)

  Original Author: Philipp A. Hartmann, OFFIS, 2013-11-05

*****************************************************************************/

#ifdef SC_DEFAULT_WRITER_POLICY
# undef SC_DEFAULT_WRITER_POLICY
#endif
#define SC_DEFAULT_WRITER_POLICY SC_MANY_WRITERS
#include <systemc>
