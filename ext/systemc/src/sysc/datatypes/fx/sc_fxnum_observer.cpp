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

  sc_fxnum_observer.cpp - 

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: sc_fxnum_observer.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include "sysc/datatypes/fx/sc_fxnum_observer.h"


namespace sc_dt
{

// ----------------------------------------------------------------------------
//  CLASS : sc_fxnum_observer
//
//  Abstract base class for fixed-point types observers; arbitrary precision.
// ----------------------------------------------------------------------------

sc_fxnum_observer* (*sc_fxnum_observer::default_observer) () = 0;


// ----------------------------------------------------------------------------
//  CLASS : sc_fxnum_fast_observer
//
//  Abstract base class for fixed-point types observers; limited precision.
// ----------------------------------------------------------------------------

sc_fxnum_fast_observer* (*sc_fxnum_fast_observer::default_observer) () = 0;

} // namespace sc_dt


// Taf!
