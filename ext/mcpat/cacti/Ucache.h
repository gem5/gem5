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


#ifndef __UCACHE_H__
#define __UCACHE_H__

#include <list>

#include "area.h"
#include "nuca.h"
#include "router.h"

class min_values_t {
public:
    double min_delay;
    double min_dyn;
    double min_leakage;
    double min_area;
    double min_cyc;

    min_values_t() : min_delay(BIGNUM), min_dyn(BIGNUM), min_leakage(BIGNUM), min_area(BIGNUM), min_cyc(BIGNUM) { }

    void update_min_values(const min_values_t * val);
    void update_min_values(const uca_org_t & res);
    void update_min_values(const nuca_org_t * res);
    void update_min_values(const mem_array * res);
};



struct solution {
    int    tag_array_index;
    int    data_array_index;
    list<mem_array *>::iterator tag_array_iter;
    list<mem_array *>::iterator data_array_iter;
    double access_time;
    double cycle_time;
    double area;
    double efficiency;
    powerDef total_power;
};



bool calculate_time(
    bool is_tag,
    int pure_ram,
    bool pure_cam,
    double Nspd,
    unsigned int Ndwl,
    unsigned int Ndbl,
    unsigned int Ndcm,
    unsigned int Ndsam_lev_1,
    unsigned int Ndsam_lev_2,
    mem_array *ptr_array,
    int flag_results_populate,
    results_mem_array *ptr_results,
    uca_org_t *ptr_fin_res,
    bool is_main_mem);
void update(uca_org_t *fin_res);

void solve(uca_org_t *fin_res);
void init_tech_params(double tech, bool is_tag);


struct calc_time_mt_wrapper_struct {
    uint32_t tid;
    bool     is_tag;
    bool     pure_ram;
    bool     pure_cam;
    bool     is_main_mem;
    double   Nspd_min;

    min_values_t * data_res;
    min_values_t * tag_res;

    list<mem_array *> data_arr;
    list<mem_array *> tag_arr;
};

void *calc_time_mt_wrapper(void * void_obj);

#endif
