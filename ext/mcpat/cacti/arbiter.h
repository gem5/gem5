/*****************************************************************************
 *                                McPAT/CACTI
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *                          All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/

#ifndef __ARBITER__
#define __ARBITER__

#include <assert.h>

#include <iostream>

#include "basic_circuit.h"
#include "cacti_interface.h"
#include "component.h"
#include "mat.h"
#include "parameter.h"
#include "wire.h"

class Arbiter : public Component
{
  public:
    Arbiter(
      double Req,
      double flit_sz,
      double output_len,
      TechnologyParameter::DeviceType *dt = &(g_tp.peri_global));
    ~Arbiter();

    void print_arbiter();
    double arb_req();
    double arb_pri();
    double arb_grant();
    double arb_int();
    void compute_power();
    double Cw3(double len);
    double crossbar_ctrline();
    double transmission_buf_ctrcap();



  private:
    double NTn1, PTn1, NTn2, PTn2, R, PTi, NTi;
    double flit_size;
    double NTtr, PTtr;
    double o_len;
    TechnologyParameter::DeviceType *deviceType;
    double TriS1, TriS2;
    double min_w_pmos, Vdd;

};

#endif
