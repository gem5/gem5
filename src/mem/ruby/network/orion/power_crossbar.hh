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

// Crossbar

#ifndef _POWER_CROSSBAR_H
#define _POWER_CROSSBAR_H

typedef enum {
        TRANS_GATE,
        TRISTATE_GATE
} power_connect_model;

/* transmission gate type */
typedef enum {
        N_GATE,
        NP_GATE
} power_trans;

typedef enum {
        MATRIX_CROSSBAR =1,
        MULTREE_CROSSBAR,
        CUT_THRU_CROSSBAR,
        CROSSBAR_MAX_MODEL
} power_crossbar_model;


typedef struct {
        int model;
        unsigned n_in;
        unsigned n_out;
        unsigned data_width;
        unsigned degree;       //used only for multree xbar
        unsigned connect_type;
        unsigned trans_type;
        unsigned long int n_chg_in;
        unsigned long int n_chg_int;
        unsigned long int n_chg_out;
        unsigned long int n_chg_ctr;
        unsigned long int mask;
        double e_chg_in;
        double e_chg_int;
        double e_chg_out;
        double e_chg_ctr;
        unsigned depth;  //used only for multree xbar
        double i_leakage;
} power_crossbar;


extern int crossbar_record(power_crossbar *xb, int io, unsigned long int new_data, unsigned long int old_data, unsigned new_port, unsigned old_port);

extern int power_crossbar_init(power_crossbar *crsbar, int model, unsigned n_in, unsigned n_out, unsigned data_width, unsigned degree, int connect_type, int trans_type, double in_len, double out_len, double *req_len);

extern double crossbar_report(power_crossbar *crsbar);

#endif
