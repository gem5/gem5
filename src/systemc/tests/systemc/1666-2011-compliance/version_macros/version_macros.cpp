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

// version_macros.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: version_macros.cpp,v $
// Revision 1.4  2011/05/16 17:17:43  acg
//  Andy Goodrich: update test to mask information that changes by SystemC
//  revision.
//
// Revision 1.3  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Process control method throw_it

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

int sc_main(int argc, char* argv[])
{
  sc_assert(  SC_VERSION_MAJOR        == sc_version_major );
  sc_assert(  SC_VERSION_MINOR        == sc_version_minor );
  sc_assert(  SC_VERSION_PATCH        == sc_version_patch );
  sc_assert(  SC_VERSION_ORIGINATOR   == sc_version_originator );
  sc_assert(  SC_VERSION_RELEASE_DATE == sc_version_release_date );
  sc_assert(  SC_VERSION_PRERELEASE   == sc_version_prerelease );
  sc_assert(  SC_IS_PRERELEASE        == sc_is_prerelease );
  sc_assert(  SC_VERSION              == sc_version_string );
  sc_assert(  sc_release()            == sc_version_string );
  sc_assert(  SC_COPYRIGHT            == sc_copyright_string );
  sc_assert(  sc_copyright()          == sc_copyright_string );

  cout << "IEEE_1666_SYSTEMC = "       << IEEE_1666_SYSTEMC << endl;

  //sc_start();
 
  cout << endl << "Success" << endl;
  return 0;
}
  
