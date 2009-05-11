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

// Arbiter


#ifndef _POWER_ARBITER_H
#define _POWER_ARBITER_H

#include "power_array.hh"

typedef enum {
        RR_ARBITER =1,
        MATRIX_ARBITER,
        QUEUE_ARBITER,
        ARBITER_MAX_MODEL
} power_arbiter_model;

typedef enum {
        NEG_DFF = 1,    /* negative egde-triggered D flip-flop */
        FF_MAX_MODEL
} power_ff_model;

typedef struct {
        int model;
        unsigned long int n_switch;
        unsigned long int n_keep_1;
        unsigned long int n_keep_0;
        unsigned long int n_clock;
        double e_switch;
        double e_keep_1;
        double e_keep_0;
        double e_clock;
        double i_leakage;
} power_ff;

typedef struct{
        int model;
        unsigned req_width;
        unsigned long int n_chg_req;
        unsigned long int n_chg_grant;
        unsigned long int n_chg_carry; //internal node of rr arbiter
        unsigned long int n_chg_carry_in; //internal node of rr arbiter
        unsigned long int n_chg_mint; //internal node of matrix arbiter
        unsigned long int mask;
        double e_chg_req;
        double e_chg_grant;
        double e_chg_carry;
        double e_chg_carry_in;
        double e_chg_mint;
        power_ff pri_ff; //priority ff
        power_array queue; //request queue
        double i_leakage;
} power_arbiter;

extern int arbiter_record(power_arbiter *arb, unsigned long int new_req, unsigned long int old_req, unsigned new_grant, unsigned old_grant);

extern double arbiter_report(power_arbiter *arb);

extern int power_arbiter_init(power_arbiter *arb, int arbiter_model, int ff_model, unsigned req_width, double length, power_array_info *info);

#endif



