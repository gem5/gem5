/*****************************************************************************
 *                                McPAT/CACTI
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *            Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/


#ifndef __NUCA_H__
#define __NUCA_H__

#include <iostream>

#include "assert.h"
#include "basic_circuit.h"
#include "cacti_interface.h"
#include "component.h"
#include "io.h"
#include "mat.h"
#include "parameter.h"
#include "router.h"
#include "wire.h"

class nuca_org_t {
public:
    ~nuca_org_t();
//    int size;
    /* area, power, access time, and cycle time stats */
    Component nuca_pda;
    Component bank_pda;
    Component wire_pda;
    Wire *h_wire;
    Wire *v_wire;
    Router *router;
    /* for particular network configuration
     * calculated based on a cycle accurate
     * simulation Ref: CACTI 6 - Tech report
     */
    double contention;

    /* grid network stats */
    double avg_hops;
    int rows;
    int columns;
    int bank_count;
};



class Nuca : public Component {
public:
    Nuca(
        TechnologyParameter::DeviceType *dt);
    void print_router();
    ~Nuca();
    void sim_nuca();
    void init_cont();
    int calc_cycles(double lat, double oper_freq);
    void calculate_nuca_area (nuca_org_t *nuca);
    int check_nuca_org (nuca_org_t *n, min_values_t *minval);
    nuca_org_t * find_optimal_nuca (list<nuca_org_t *> *n, min_values_t *minval);
    void print_nuca(nuca_org_t *n);
    void print_cont_stats();

private:

    TechnologyParameter::DeviceType *deviceType;
    int wt_min, wt_max;
    Wire *wire_vertical[WIRE_TYPES],
    *wire_horizontal[WIRE_TYPES];

};


#endif
