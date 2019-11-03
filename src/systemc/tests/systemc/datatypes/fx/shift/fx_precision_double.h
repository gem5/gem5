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

  fx_precision_double.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// ============================================================================
//
//  This confidential and proprietary software may be used only
//  as authorized by a licensing agreement from Synopsys, Inc.
//  In the event of publication, the following notice is applicable:
//
//            Copyright (c) 1999 by Synopsys, Inc.
//                     ALL RIGHTS RESERVED
//
//  The entire notice above must be reproduced on all authorized copies.
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
//      File : fx_precision_double.h
//
//  Abstract : Define double precision.
//
//    Author : Martin Janssen
//
//   Created : 21-jun-1999
//
// ============================================================================

#include "fx_precision_arbitrary.h"

#define sc_fix    sc_fix_fast
#define sc_fixed  sc_fixed_fast
#define sc_fxval  sc_fxval_fast
#define sc_ufix   sc_ufix_fast
#define sc_ufixed sc_ufixed_fast

// That's all folks!

