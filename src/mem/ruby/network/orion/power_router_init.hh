/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
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
 *
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
 */

#ifndef _POWER_ROUTER_INIT_H
#define _POWER_ROUTER_INIT_H

#include "power_array.hh"
#include "power_arbiter.hh"
#include "power_crossbar.hh"

/* ------------ Models ------------------------ */
/*typedef enum {
        GENERIC_SEL = 1,
        SEL_MAX_MODEL
} power_sel_model;
*/

/* ------------ Misc --------------------------- */

/*typedef struct {
        int model;
        unsigned width;
        unsigned long long n_anyreq;
        unsigned long long n_chgreq;
        unsigned long long n_grant;
        unsigned long long n_enc[MAX_SEL_LEVEL];
        double e_anyreq;
        double e_chgreq;
        double e_grant;
        double e_enc[MAX_SEL_LEVEL];
} power_sel;
*/

/* --------------- Loading --------------- */
typedef enum {
        ACTUAL_LOADING =1,
        HALF_LOADING,
        MAX_LOADING
}loading;

/* ----------------- Router --------------- */

typedef struct {
        power_crossbar crossbar;
        power_array in_buf;
        power_arbiter vc_in_arb;
        power_arbiter vc_out_arb;
        power_arbiter sw_in_arb;
        power_arbiter sw_out_arb;
        double i_leakage;
} power_router;

typedef struct {
        //general
        unsigned n_in;
        unsigned n_out;
        unsigned flit_width;
        //vc
        unsigned n_v_channel;
        unsigned n_v_class;
        int in_share_buf;
        int out_share_buf;
        int in_share_switch;
        int out_share_switch;
        //xbar
        int crossbar_model;
        int degree;
        int connect_type;
        int trans_type;
        double crossbar_in_len;
        double crossbar_out_len;

    int in_buf;

        //buffer
        power_array_info in_buf_info;
        unsigned pipe_depth;
        //arbiter
        int vc_in_arb_model;
        int vc_out_arb_model;
        int vc_in_arb_ff_model;
        int vc_out_arb_ff_model;
        int sw_in_arb_model;
        int sw_out_arb_model;
        int sw_in_arb_ff_model;
        int sw_out_arb_ff_model;

        power_array_info vc_in_arb_queue_info;
        power_array_info vc_out_arb_queue_info;
        power_array_info sw_in_arb_queue_info;
        power_array_info sw_out_arb_queue_info;
        //derived
        unsigned n_total_in;
        unsigned n_total_out;
        unsigned in_n_switch;
        unsigned n_switch_in;
        unsigned n_switch_out;
} power_router_info;

extern int power_router_init(power_router *router, power_router_info *info);
#endif
