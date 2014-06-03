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



#include <cassert>

#include "Ucache.h"
#include "nuca.h"

unsigned int MIN_BANKSIZE = 65536;
#define FIXED_OVERHEAD 55e-12 /* clock skew and jitter in s. Ref: Hrishikesh et al ISCA 01 */
#define LATCH_DELAY 28e-12 /* latch delay in s (later should use FO4 TODO) */
#define CONTR_2_BANK_LAT 0

int cont_stats[2 /*l2 or l3*/][5/* cores */][ROUTER_TYPES][7 /*banks*/][8 /* cycle time */];

Nuca::Nuca(
    TechnologyParameter::DeviceType *dt = &(g_tp.peri_global)
): deviceType(dt) {
    init_cont();
}

void
Nuca::init_cont() {
    FILE *cont;
    char line[5000];
    char jk[5000];
    cont = fopen("contention.dat", "r");
    if (!cont) {
        cout << "contention.dat file is missing!\n";
        exit(0);
    }

    for (int i = 0; i < 2; i++) {
        for (int j = 2; j < 5; j++) {
            for (int k = 0; k < ROUTER_TYPES; k++) {
                for (int l = 0; l < 7; l++) {
                    int *temp = cont_stats[i/*l2 or l3*/][j/*core*/][k/*64 or 128 or 256 link bw*/][l /* no banks*/];
                    assert(fscanf(cont, "%[^\n]\n", line) != EOF);
                    sscanf(line, "%[^:]: %d %d %d %d %d %d %d %d", jk,
                           &temp[0], &temp[1], &temp[2], &temp[3],
                           &temp[4], &temp[5], &temp[6], &temp[7]);
                }
            }
        }
    }
    fclose(cont);
}

void
Nuca::print_cont_stats() {
    for (int i = 0; i < 2; i++) {
        for (int j = 2; j < 5; j++) {
            for (int k = 0; k < ROUTER_TYPES; k++) {
                for (int l = 0; l < 7; l++) {
                    for (int m = 0; l < 7; l++) {
                        cout << cont_stats[i][j][k][l][m] << " ";
                    }
                    cout << endl;
                }
            }
        }
    }
    cout << endl;
}

Nuca::~Nuca() {
    for (int i = wt_min; i <= wt_max; i++) {
        delete wire_vertical[i];
        delete wire_horizontal[i];
    }
}

/* converts latency (in s) to cycles depending upon the FREQUENCY (in GHz) */
int
Nuca::calc_cycles(double lat, double oper_freq) {
    //TODO: convert latch delay to FO4 */
    double cycle_time = (1.0 / (oper_freq * 1e9)); /*s*/
    cycle_time -= LATCH_DELAY;
    cycle_time -= FIXED_OVERHEAD;

    return (int)ceil(lat / cycle_time);
}


nuca_org_t::~nuca_org_t() {
    // if(h_wire) delete h_wire;
    // if(v_wire) delete v_wire;
    // if(router) delete router;
}

/*
 * Version - 6.0
 *
 * Perform exhaustive search across different bank organizatons,
 * router configurations, grid organizations, and wire models and
 * find an optimal NUCA organization
 * For different bank count values
 * 1. Optimal bank organization is calculated
 * 2. For each bank organization, find different NUCA organizations
 *    using various router configurations, grid organizations,
 *    and wire models.
 * 3. NUCA model with the least cost is picked for
 *    this particular bank count
 * Finally include contention statistics and find the optimal
 *    NUCA configuration
 */
void
Nuca::sim_nuca() {
    /* temp variables */
    int it, ro, wr;
    int num_cyc;
    unsigned int i, j, k;
    unsigned int r, c;
    int l2_c;
    int bank_count = 0;
    uca_org_t ures;
    nuca_org_t *opt_n;
    mem_array tag, data;
    list<nuca_org_t *> nuca_list;
    Router *router_s[ROUTER_TYPES];
    router_s[0] = new Router(64.0, 8, 4, &(g_tp.peri_global));
    router_s[0]->print_router();
    router_s[1] = new Router(128.0, 8, 4, &(g_tp.peri_global));
    router_s[1]->print_router();
    router_s[2] = new Router(256.0, 8, 4, &(g_tp.peri_global));
    router_s[2]->print_router();

    int core_in; // to store no. of cores

    /* to search diff grid organizations */
    double curr_hop, totno_hops, totno_hhops, totno_vhops, tot_lat,
    curr_acclat;
    double avg_lat, avg_hop, avg_hhop, avg_vhop, avg_dyn_power,
    avg_leakage_power;

    double opt_acclat = INF, opt_avg_lat = INF, opt_tot_lat = INF;
    int opt_rows = 0;
    int opt_columns = 0;
    double opt_totno_hops = 0;
    double opt_avg_hop = 0;
    double opt_dyn_power = 0, opt_leakage_power = 0;
    min_values_t minval;

    int bank_start = 0;

    int flit_width = 0;

    /* vertical and horizontal hop latency values */
    int ver_hop_lat, hor_hop_lat; /* in cycles */


    /* no. of different bank sizes to consider */
    int iterations;


    g_ip->nuca_cache_sz = g_ip->cache_sz;
    nuca_list.push_back(new nuca_org_t());

    if (g_ip->cache_level == 0) l2_c = 1;
    else l2_c = 0;

    if (g_ip->cores <= 4) core_in = 2;
    else if (g_ip->cores <= 8) core_in = 3;
    else if (g_ip->cores <= 16) core_in = 4;
    else {
        cout << "Number of cores should be <= 16!\n";
        exit(0);
    }


    // set the lower bound to an appropriate value. this depends on cache associativity
    if (g_ip->assoc > 2) {
        i = 2;
        while (i != g_ip->assoc) {
            MIN_BANKSIZE *= 2;
            i *= 2;
        }
    }

    iterations = (int)logtwo((int)g_ip->cache_sz / MIN_BANKSIZE);

    if (g_ip->force_wiretype) {
        if (g_ip->wt == Low_swing) {
            wt_min = Low_swing;
            wt_max = Low_swing;
        } else {
            wt_min = Global;
            wt_max = Low_swing - 1;
        }
    } else {
        wt_min = Global;
        wt_max = Low_swing;
    }
    if (g_ip->nuca_bank_count != 0) { // simulate just one bank
        if (g_ip->nuca_bank_count != 2 && g_ip->nuca_bank_count != 4 &&
                g_ip->nuca_bank_count != 8 && g_ip->nuca_bank_count != 16 &&
                g_ip->nuca_bank_count != 32 && g_ip->nuca_bank_count != 64) {
            fprintf(stderr, "Incorrect bank count value! Please fix the ",
                    "value in cache.cfg\n");
        }
        bank_start = (int)logtwo((double)g_ip->nuca_bank_count);
        iterations = bank_start + 1;
        g_ip->cache_sz = g_ip->cache_sz / g_ip->nuca_bank_count;
    }
    cout << "Simulating various NUCA configurations\n";
    for (it = bank_start; it < iterations; it++) {
        /* different bank count values */
        ures.tag_array2 = &tag;
        ures.data_array2 = &data;
        /*
         * find the optimal bank organization
         */
        solve(&ures);
//    output_UCA(&ures);
        bank_count = g_ip->nuca_cache_sz / g_ip->cache_sz;
        cout << "====" <<  g_ip->cache_sz << "\n";

        for (wr = wt_min; wr <= wt_max; wr++) {

            for (ro = 0; ro < ROUTER_TYPES; ro++) {
                flit_width = (int) router_s[ro]->flit_size; //initialize router
                nuca_list.back()->nuca_pda.cycle_time = router_s[ro]->cycle_time;

                /* calculate router and wire parameters */

                double vlength = ures.cache_ht; /* length of the wire (u)*/
                double hlength = ures.cache_len; // u

                /* find delay, area, and power for wires */
                wire_vertical[wr] = new Wire((enum Wire_type) wr, vlength);
                wire_horizontal[wr] = new Wire((enum Wire_type) wr, hlength);


                hor_hop_lat =
                    calc_cycles(wire_horizontal[wr]->delay,
                                1 /(nuca_list.back()->nuca_pda.cycle_time *
                                    .001));
                ver_hop_lat =
                    calc_cycles(wire_vertical[wr]->delay,
                                1 / (nuca_list.back()->nuca_pda.cycle_time *
                                     .001));

                /*
                 * assume a grid like topology and explore for optimal network
                 * configuration using different row and column count values.
                 */
                for (c = 1; c <= (unsigned int)bank_count; c++) {
                    while (bank_count % c != 0) c++;
                    r = bank_count / c;

                    /*
                     * to find the avg access latency of a NUCA cache, uncontended
                     * access time to each bank from the
                     * cache controller is calculated.
                     * avg latency =
                     * sum of the access latencies to individual banks)/bank
                     * count value.
                     */
                    totno_hops = totno_hhops = totno_vhops = tot_lat = 0;
                    k = 1;
                    for (i = 0; i < r; i++) {
                        for (j = 0; j < c; j++) {
                            /*
                             * vertical hops including the
                             * first hop from the cache controller
                             */
                            curr_hop = i + 1;
                            curr_hop += j; /* horizontal hops */
                            totno_hhops += j;
                            totno_vhops += (i + 1);
                            curr_acclat = (i * ver_hop_lat + CONTR_2_BANK_LAT +
                                           j * hor_hop_lat);

                            tot_lat += curr_acclat;
                            totno_hops += curr_hop;
                        }
                    }
                    avg_lat = tot_lat / bank_count;
                    avg_hop = totno_hops / bank_count;
                    avg_hhop = totno_hhops / bank_count;
                    avg_vhop = totno_vhops / bank_count;

                    /* net access latency */
                    curr_acclat = 2 * avg_lat + 2 * (router_s[ro]->delay *
                                                     avg_hop) +
                        calc_cycles(ures.access_time,
                                    1 /
                                    (nuca_list.back()->nuca_pda.cycle_time *
                                     .001));

                    /* avg access lat of nuca */
                    avg_dyn_power =
                        avg_hop *
                        (router_s[ro]->power.readOp.dynamic) + avg_hhop *
                        (wire_horizontal[wr]->power.readOp.dynamic) *
                        (g_ip->block_sz * 8 + 64) + avg_vhop *
                        (wire_vertical[wr]->power.readOp.dynamic) *
                        (g_ip->block_sz * 8 + 64) + ures.power.readOp.dynamic;

                    avg_leakage_power =
                        bank_count * router_s[ro]->power.readOp.leakage +
                        avg_hhop * (wire_horizontal[wr]->power.readOp.leakage *
                                    wire_horizontal[wr]->delay) * flit_width +
                        avg_vhop * (wire_vertical[wr]->power.readOp.leakage *
                                    wire_horizontal[wr]->delay);

                    if (curr_acclat < opt_acclat) {
                        opt_acclat = curr_acclat;
                        opt_tot_lat = tot_lat;
                        opt_avg_lat = avg_lat;
                        opt_totno_hops = totno_hops;
                        opt_avg_hop = avg_hop;
                        opt_rows = r;
                        opt_columns = c;
                        opt_dyn_power = avg_dyn_power;
                        opt_leakage_power = avg_leakage_power;
                    }
                    totno_hops = 0;
                    tot_lat = 0;
                    totno_hhops = 0;
                    totno_vhops = 0;
                }
                nuca_list.back()->wire_pda.power.readOp.dynamic =
                    opt_avg_hop * flit_width *
                    (wire_horizontal[wr]->power.readOp.dynamic +
                     wire_vertical[wr]->power.readOp.dynamic);
                nuca_list.back()->avg_hops = opt_avg_hop;
                /* network delay/power */
                nuca_list.back()->h_wire = wire_horizontal[wr];
                nuca_list.back()->v_wire = wire_vertical[wr];
                nuca_list.back()->router = router_s[ro];
                /* bank delay/power */

                nuca_list.back()->bank_pda.delay = ures.access_time;
                nuca_list.back()->bank_pda.power = ures.power;
                nuca_list.back()->bank_pda.area.h = ures.cache_ht;
                nuca_list.back()->bank_pda.area.w = ures.cache_len;
                nuca_list.back()->bank_pda.cycle_time = ures.cycle_time;

                num_cyc = calc_cycles(nuca_list.back()->bank_pda.delay /*s*/,
                                      1 /
                                      (nuca_list.back()->nuca_pda.cycle_time *
                                       .001/*GHz*/));
                if (num_cyc % 2 != 0) num_cyc++;
                if (num_cyc > 16) num_cyc = 16; // we have data only up to 16 cycles

                if (it < 7) {
                    nuca_list.back()->nuca_pda.delay = opt_acclat +
                                                       cont_stats[l2_c][core_in][ro][it][num_cyc/2-1];
                    nuca_list.back()->contention =
                        cont_stats[l2_c][core_in][ro][it][num_cyc/2-1];
                } else {
                    nuca_list.back()->nuca_pda.delay = opt_acclat +
                                                       cont_stats[l2_c][core_in][ro][7][num_cyc/2-1];
                    nuca_list.back()->contention =
                        cont_stats[l2_c][core_in][ro][7][num_cyc/2-1];
                }
                nuca_list.back()->nuca_pda.power.readOp.dynamic = opt_dyn_power;
                nuca_list.back()->nuca_pda.power.readOp.leakage = opt_leakage_power;

                /* array organization */
                nuca_list.back()->bank_count = bank_count;
                nuca_list.back()->rows = opt_rows;
                nuca_list.back()->columns = opt_columns;
                calculate_nuca_area (nuca_list.back());

                minval.update_min_values(nuca_list.back());
                nuca_list.push_back(new nuca_org_t());
                opt_acclat = BIGNUM;

            }
        }
        g_ip->cache_sz /= 2;
    }

    delete(nuca_list.back());
    nuca_list.pop_back();
    opt_n = find_optimal_nuca(&nuca_list, &minval);
    print_nuca(opt_n);
    g_ip->cache_sz = g_ip->nuca_cache_sz / opt_n->bank_count;

    list<nuca_org_t *>::iterator niter;
    for (niter = nuca_list.begin(); niter != nuca_list.end(); ++niter) {
        delete *niter;
    }
    nuca_list.clear();

    for (int i = 0; i < ROUTER_TYPES; i++) {
        delete router_s[i];
    }
    g_ip->display_ip();
    //  g_ip->force_cache_config = true;
    //  g_ip->ndwl = 8;
    //  g_ip->ndbl = 16;
    //  g_ip->nspd = 4;
    //  g_ip->ndcm = 1;
    //  g_ip->ndsam1 = 8;
    //  g_ip->ndsam2 = 32;

}


void
Nuca::print_nuca (nuca_org_t *fr) {
    printf("\n---------- CACTI version 6.5, Non-uniform Cache Access "
           "----------\n\n");
    printf("Optimal number of banks - %d\n", fr->bank_count);
    printf("Grid organization rows x columns - %d x %d\n",
           fr->rows, fr->columns);
    printf("Network frequency - %g GHz\n",
           (1 / fr->nuca_pda.cycle_time)*1e3);
    printf("Cache dimension (mm x mm) - %g x %g\n",
           fr->nuca_pda.area.h,
           fr->nuca_pda.area.w);

    fr->router->print_router();

    printf("\n\nWire stats:\n");
    if (fr->h_wire->wt == Global) {
        printf("\tWire type - Full swing global wires with least "
               "possible delay\n");
    } else if (fr->h_wire->wt == Global_5) {
        printf("\tWire type - Full swing global wires with "
               "5%% delay penalty\n");
    } else if (fr->h_wire->wt == Global_10) {
        printf("\tWire type - Full swing global wires with "
               "10%% delay penalty\n");
    } else if (fr->h_wire->wt == Global_20) {
        printf("\tWire type - Full swing global wires with "
               "20%% delay penalty\n");
    } else if (fr->h_wire->wt == Global_30) {
        printf("\tWire type - Full swing global wires with "
               "30%% delay penalty\n");
    } else if (fr->h_wire->wt == Low_swing) {
        printf("\tWire type - Low swing wires\n");
    }

    printf("\tHorizontal link delay - %g (ns)\n",
           fr->h_wire->delay*1e9);
    printf("\tVertical link delay - %g (ns)\n",
           fr->v_wire->delay*1e9);
    printf("\tDelay/length - %g (ns/mm)\n",
           fr->h_wire->delay*1e9 / fr->bank_pda.area.w);
    printf("\tHorizontal link energy -dynamic/access %g (nJ)\n"
           "\t                       -leakage %g (nW)\n\n",
           fr->h_wire->power.readOp.dynamic*1e9,
           fr->h_wire->power.readOp.leakage*1e9);
    printf("\tVertical link energy -dynamic/access %g (nJ)\n"
           "\t                     -leakage %g (nW)\n\n",
           fr->v_wire->power.readOp.dynamic*1e9,
           fr->v_wire->power.readOp.leakage*1e9);
    printf("\n\n");
    fr->v_wire->print_wire();
    printf("\n\nBank stats:\n");
}


nuca_org_t *
Nuca::find_optimal_nuca (list<nuca_org_t *> *n, min_values_t *minval) {
    double cost = 0;
    double min_cost = BIGNUM;
    nuca_org_t *res = NULL;
    float d, a, dp, lp, c;
    int v;
    dp = g_ip->dynamic_power_wt_nuca;
    lp = g_ip->leakage_power_wt_nuca;
    a = g_ip->area_wt_nuca;
    d = g_ip->delay_wt_nuca;
    c = g_ip->cycle_time_wt_nuca;

    list<nuca_org_t *>::iterator niter;


    for (niter = n->begin(); niter != n->end(); niter++) {
        fprintf(stderr, "\n-----------------------------"
                "---------------\n");


        printf("NUCA___stats %d \tbankcount: lat = %g \tdynP = %g \twt = %d\t "
               "bank_dpower = %g \tleak = %g \tcycle = %g\n",
               (*niter)->bank_count,
               (*niter)->nuca_pda.delay,
               (*niter)->nuca_pda.power.readOp.dynamic,
               (*niter)->h_wire->wt,
               (*niter)->bank_pda.power.readOp.dynamic,
               (*niter)->nuca_pda.power.readOp.leakage,
               (*niter)->nuca_pda.cycle_time);


        if (g_ip->ed == 1) {
            cost = ((*niter)->nuca_pda.delay / minval->min_delay) *
                   ((*niter)->nuca_pda.power.readOp.dynamic / minval->min_dyn);
            if (min_cost > cost) {
                min_cost = cost;
                res = ((*niter));
            }
        } else if (g_ip->ed == 2) {
            cost = ((*niter)->nuca_pda.delay / minval->min_delay) *
                   ((*niter)->nuca_pda.delay / minval->min_delay) *
                   ((*niter)->nuca_pda.power.readOp.dynamic / minval->min_dyn);
            if (min_cost > cost) {
                min_cost = cost;
                res = ((*niter));
            }
        } else {
            /*
             * check whether the current organization
             * meets the input deviation constraints
             */
            v = check_nuca_org((*niter), minval);
            if (minval->min_leakage == 0) minval->min_leakage = 0.1; //FIXME remove this after leakage modeling

            if (v) {
                cost = (d  * ((*niter)->nuca_pda.delay / minval->min_delay) +
                        c  * ((*niter)->nuca_pda.cycle_time / minval->min_cyc) +
                        dp * ((*niter)->nuca_pda.power.readOp.dynamic /
                              minval->min_dyn) +
                        lp * ((*niter)->nuca_pda.power.readOp.leakage /
                              minval->min_leakage) +
                        a  * ((*niter)->nuca_pda.area.get_area() /
                              minval->min_area));
                fprintf(stderr, "cost = %g\n", cost);

                if (min_cost > cost) {
                    min_cost = cost;
                    res = ((*niter));
                }
            } else {
                niter = n->erase(niter);
                if (niter != n->begin())
                    niter --;
            }
        }
    }
    return res;
}

int
Nuca::check_nuca_org (nuca_org_t *n, min_values_t *minval) {
    if (((n->nuca_pda.delay - minval->min_delay)*100 / minval->min_delay) >
        g_ip->delay_dev_nuca) {
        return 0;
    }
    if (((n->nuca_pda.power.readOp.dynamic - minval->min_dyn) /
         minval->min_dyn)*100 >
        g_ip->dynamic_power_dev_nuca) {
        return 0;
    }
    if (((n->nuca_pda.power.readOp.leakage - minval->min_leakage) /
         minval->min_leakage)*100 >
        g_ip->leakage_power_dev_nuca) {
        return 0;
    }
    if (((n->nuca_pda.cycle_time - minval->min_cyc) / minval->min_cyc)*100 >
        g_ip->cycle_time_dev_nuca) {
        return 0;
    }
    if (((n->nuca_pda.area.get_area() - minval->min_area) / minval->min_area) *
        100 >
        g_ip->area_dev_nuca) {
        return 0;
    }
    return 1;
}

void
Nuca::calculate_nuca_area (nuca_org_t *nuca) {
    nuca->nuca_pda.area.h =
        nuca->rows * ((nuca->h_wire->wire_width +
                       nuca->h_wire->wire_spacing)
                      * nuca->router->flit_size +
                      nuca->bank_pda.area.h);

    nuca->nuca_pda.area.w =
        nuca->columns * ((nuca->v_wire->wire_width +
                          nuca->v_wire->wire_spacing)
                         * nuca->router->flit_size +
                         nuca->bank_pda.area.w);
}

