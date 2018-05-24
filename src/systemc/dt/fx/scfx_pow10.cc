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

  scfx_pow10.cpp -

  Original Author: Robert Graulich, Synopsys, Inc.
                   Martin Janssen,  Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: scfx_pow10.cpp,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:58  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#include "systemc/ext/dt/fx/scfx_pow10.hh"

namespace sc_dt
{

// ----------------------------------------------------------------------------
//  CLASS : scfx_pow10
//
//  Class to compute (and cache) powers of 10 in arbitrary precision.
// ----------------------------------------------------------------------------

scfx_pow10::scfx_pow10()
{
    m_pos[0] = scfx_rep(10.0);
    m_neg[0] = scfx_rep(0.1);

    for (int i = 1; i < SCFX_POW10_TABLE_SIZE; i++) {
        m_pos[i].set_nan();
        m_neg[i].set_nan();
    }
}

scfx_pow10::~scfx_pow10() {}

const scfx_rep
scfx_pow10::operator() (int i)
{
    if (i == 0) {
        return scfx_rep(1.0);
    }

    if (i > 0) {
        int bit = scfx_find_msb(i);
        scfx_rep result = *pos(bit);
        if (bit) {
            while (--bit >= 0) {
                if ((1 << bit) & i) {
                    scfx_rep *tmp = mult_scfx_rep(result, *pos(bit));
                    result = *tmp;
                    delete tmp;
                }
            }
        }
        return result;
    } else {
        i = -i;
        int bit = scfx_find_msb(i);
        scfx_rep result = *neg(bit);
        if (bit) {
            while (--bit >= 0) {
                if ((1 << bit) & i) {
                    scfx_rep *tmp = mult_scfx_rep(result, *neg(bit));
                    result = *tmp;
                    delete tmp;
                }
            }
        }
        return result;
    }
}


scfx_rep *
scfx_pow10::pos(int i)
{
    if (!m_pos[i].is_normal()) {
        multiply(m_pos[i], *pos(i - 1), *pos(i - 1));
    }
    return &m_pos[i];
}

scfx_rep *
scfx_pow10::neg(int i)
{
    if (!m_neg[i].is_normal()) {
        multiply(m_neg[i], *neg(i - 1), *neg(i - 1));
    }
    return &m_neg[i];
}

} // namespace sc_dt
