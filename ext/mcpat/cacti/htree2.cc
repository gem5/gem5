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
#include <iostream>

#include "htree2.h"
#include "wire.h"

Htree2::Htree2(
    enum Wire_type wire_model, double mat_w, double mat_h,
    int a_bits, int d_inbits, int search_data_in, int d_outbits,
    int search_data_out, int bl, int wl, enum Htree_type htree_type,
    bool uca_tree_, bool search_tree_, TechnologyParameter::DeviceType *dt)
    : in_rise_time(0), out_rise_time(0),
      tree_type(htree_type), mat_width(mat_w), mat_height(mat_h),
      add_bits(a_bits), data_in_bits(d_inbits),
      search_data_in_bits(search_data_in), data_out_bits(d_outbits),
      search_data_out_bits(search_data_out), ndbl(bl), ndwl(wl),
      uca_tree(uca_tree_), search_tree(search_tree_), wt(wire_model),
      deviceType(dt) {
    assert(ndbl >= 2 && ndwl >= 2);

//  if (ndbl == 1 && ndwl == 1)
//  {
//    delay = 0;
//    power.readOp.dynamic = 0;
//    power.readOp.leakage = 0;
//    area.w = mat_w;
//    area.h = mat_h;
//    return;
//  }
//  if (ndwl == 1) ndwl++;
//  if (ndbl == 1) ndbl++;

    max_unpipelined_link_delay = 0; //TODO
    min_w_nmos = g_tp.min_w_nmos_;
    min_w_pmos = deviceType->n_to_p_eff_curr_drv_ratio * min_w_nmos;

    switch (htree_type) {
    case Add_htree:
        wire_bw = init_wire_bw = add_bits;
        in_htree();
        break;
    case Data_in_htree:
        wire_bw = init_wire_bw = data_in_bits;
        in_htree();
        break;
    case Data_out_htree:
        wire_bw = init_wire_bw = data_out_bits;
        out_htree();
        break;
    case Search_in_htree:
        wire_bw = init_wire_bw = search_data_in_bits;//in_search_tree is broad cast, out_htree is not.
        in_htree();
        break;
    case Search_out_htree:
        wire_bw = init_wire_bw = search_data_out_bits;
        out_htree();
        break;
    default:
        assert(0);
        break;
    }

    power_bit = power;
    power.readOp.dynamic *= init_wire_bw;

    assert(power.readOp.dynamic >= 0);
    assert(power.readOp.leakage >= 0);
}



// nand gate sizing calculation
void Htree2::input_nand(double s1, double s2, double l_eff) {
    Wire w1(wt, l_eff);
    double pton_size = deviceType->n_to_p_eff_curr_drv_ratio;
    // input capacitance of a repeater  = input capacitance of nand.
    double nsize = s1 * (1 + pton_size) / (2 + pton_size);
    nsize = (nsize < 1) ? 1 : nsize;

    double tc = 2 * tr_R_on(nsize * min_w_nmos, NCH, 1) *
        (drain_C_(nsize * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def) * 2 +
         2 * gate_C(s2 * (min_w_nmos + min_w_pmos), 0));
    delay += horowitz(w1.out_rise_time, tc,
                      deviceType->Vth / deviceType->Vdd, deviceType->Vth /
                      deviceType->Vdd, RISE);
    power.readOp.dynamic += 0.5 *
        (2 * drain_C_(pton_size * nsize * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def)
         + drain_C_(nsize * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def)
         + 2 * gate_C(s2 * (min_w_nmos + min_w_pmos), 0)) *
        deviceType->Vdd * deviceType->Vdd;

    power.searchOp.dynamic += 0.5 *
        (2 * drain_C_(pton_size * nsize * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def)
         + drain_C_(nsize * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def)
         + 2 * gate_C(s2 * (min_w_nmos + min_w_pmos), 0)) *
        deviceType->Vdd * deviceType->Vdd * wire_bw ;
    power.readOp.leakage += (wire_bw *
                             cmos_Isub_leakage(min_w_nmos * (nsize * 2),
                                               min_w_pmos * nsize * 2, 2,
                                               nand)) * deviceType->Vdd;
    power.readOp.gate_leakage += (wire_bw *
                                  cmos_Ig_leakage(min_w_nmos * (nsize * 2),
                                                  min_w_pmos * nsize * 2, 2,
                                                  nand)) * deviceType->Vdd;
}



// tristate buffer model consisting of not, nand, nor, and driver transistors
void Htree2::output_buffer(double s1, double s2, double l_eff) {
    Wire w1(wt, l_eff);
    double pton_size = deviceType->n_to_p_eff_curr_drv_ratio;
    // input capacitance of repeater = input capacitance of nand + nor.
    double size = s1 * (1 + pton_size) / (2 + pton_size + 1 + 2 * pton_size);
    double s_eff =  //stage eff of a repeater in a wire
        (gate_C(s2 * (min_w_nmos + min_w_pmos), 0) + w1.wire_cap(l_eff * 1e-6,
                                                                 true)) /
        gate_C(s2 * (min_w_nmos + min_w_pmos), 0);
    double tr_size = gate_C(s1 * (min_w_nmos + min_w_pmos), 0) * 1 / 2 /
        (s_eff * gate_C(min_w_pmos, 0));
    size = (size < 1) ? 1 : size;

    double res_nor = 2 * tr_R_on(size * min_w_pmos, PCH, 1);
    double res_ptrans = tr_R_on(tr_size * min_w_nmos, NCH, 1);
    double cap_nand_out =
        drain_C_(size * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def) +
        drain_C_(size * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def) * 2 +
        gate_C(tr_size * min_w_pmos, 0);
    double cap_ptrans_out = 2 *
        (drain_C_(tr_size * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def) +
         drain_C_(tr_size * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def)) +
        gate_C(s1 * (min_w_nmos + min_w_pmos), 0);

    double tc = res_nor * cap_nand_out + (res_nor + res_ptrans) * cap_ptrans_out;


    delay += horowitz(w1.out_rise_time, tc,
                      deviceType->Vth / deviceType->Vdd, deviceType->Vth /
                      deviceType->Vdd, RISE);

    //nand
    power.readOp.dynamic += 0.5 *
        (2 * drain_C_(size * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def) +
         drain_C_(size * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def) +
         gate_C(tr_size * (min_w_pmos), 0)) *
        deviceType->Vdd * deviceType->Vdd;

    power.searchOp.dynamic += 0.5 *
        (2 * drain_C_(size * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def) +
         drain_C_(size * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def) +
         gate_C(tr_size * (min_w_pmos), 0)) *
        deviceType->Vdd * deviceType->Vdd * init_wire_bw;

    //not
    power.readOp.dynamic += 0.5 *
        (drain_C_(size * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def)
         + drain_C_(size * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def)
         + gate_C(size * (min_w_nmos + min_w_pmos), 0)) *
        deviceType->Vdd * deviceType->Vdd;

    power.searchOp.dynamic += 0.5 *
        (drain_C_(size * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def)
         + drain_C_(size * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def)
         + gate_C(size * (min_w_nmos + min_w_pmos), 0)) *
        deviceType->Vdd * deviceType->Vdd * init_wire_bw;

    //nor
    power.readOp.dynamic += 0.5 *
        (drain_C_(size * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def)
         + 2 * drain_C_(size * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def)
         + gate_C(tr_size * (min_w_nmos + min_w_pmos), 0)) *
        deviceType->Vdd * deviceType->Vdd;

    power.searchOp.dynamic += 0.5 *
        (drain_C_(size * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def)
         + 2 * drain_C_(size * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def)
         + gate_C(tr_size * (min_w_nmos + min_w_pmos), 0)) *
        deviceType->Vdd * deviceType->Vdd * init_wire_bw;

    //output transistor
    power.readOp.dynamic += 0.5 *
        ((drain_C_(tr_size * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def)
          + drain_C_(tr_size * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def)) * 2
         + gate_C(s1 * (min_w_nmos + min_w_pmos), 0)) *
        deviceType->Vdd * deviceType->Vdd;

    power.searchOp.dynamic += 0.5 *
        ((drain_C_(tr_size * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def)
          + drain_C_(tr_size * min_w_nmos, NCH, 1, 1, g_tp.cell_h_def)) * 2
         + gate_C(s1 * (min_w_nmos + min_w_pmos), 0)) *
        deviceType->Vdd * deviceType->Vdd * init_wire_bw;

    if (uca_tree) {
        power.readOp.leakage +=
            cmos_Isub_leakage(min_w_nmos * tr_size * 2, min_w_pmos * tr_size *
                              2, 1, inv) *
            deviceType->Vdd * wire_bw;/*inverter + output tr*/
        power.readOp.leakage +=
            cmos_Isub_leakage(min_w_nmos * size * 3, min_w_pmos * size * 3, 2,
                              nand) * deviceType->Vdd * wire_bw;//nand
        power.readOp.leakage +=
            cmos_Isub_leakage(min_w_nmos * size * 3, min_w_pmos * size * 3, 2,
                              nor) * deviceType->Vdd * wire_bw;//nor

        power.readOp.gate_leakage +=
            cmos_Ig_leakage(min_w_nmos * tr_size * 2, min_w_pmos * tr_size * 2,
                            1, inv) *
            deviceType->Vdd * wire_bw;/*inverter + output tr*/
        power.readOp.gate_leakage +=
            cmos_Ig_leakage(min_w_nmos * size * 3, min_w_pmos * size * 3, 2,
                            nand) * deviceType->Vdd * wire_bw;//nand
        power.readOp.gate_leakage +=
            cmos_Ig_leakage(min_w_nmos * size * 3, min_w_pmos * size * 3, 2,
                            nor) * deviceType->Vdd * wire_bw;//nor
    } else {
        power.readOp.leakage +=
            cmos_Isub_leakage(min_w_nmos * tr_size * 2, min_w_pmos * tr_size *
                              2, 1, inv) *
            deviceType->Vdd * wire_bw;/*inverter + output tr*/
        power.readOp.leakage +=
            cmos_Isub_leakage(min_w_nmos * size * 3, min_w_pmos * size * 3, 2,
                              nand) * deviceType->Vdd * wire_bw;//nand
        power.readOp.leakage +=
            cmos_Isub_leakage(min_w_nmos * size * 3, min_w_pmos * size * 3, 2,
                              nor) * deviceType->Vdd * wire_bw;//nor

        power.readOp.gate_leakage +=
            cmos_Ig_leakage(min_w_nmos * tr_size * 2, min_w_pmos * tr_size * 2,
                            1, inv) *
            deviceType->Vdd * wire_bw;/*inverter + output tr*/
        power.readOp.gate_leakage +=
            cmos_Ig_leakage(min_w_nmos * size * 3, min_w_pmos * size * 3, 2,
                            nand) * deviceType->Vdd * wire_bw;//nand
        power.readOp.gate_leakage +=
            cmos_Ig_leakage(min_w_nmos * size * 3, min_w_pmos * size * 3, 2,
                            nor) * deviceType->Vdd * wire_bw;//nor
    }
}



/* calculates the input h-tree delay/power
 * A nand gate is used at each node to
 * limit the signal
 * The area of an unbalanced htree (rows != columns)
 * depends on how data is traversed.
 * In the following function, if ( no. of rows < no. of columns),
 * then data first traverse in excess hor. links until vertical
 * and horizontal nodes are same.
 * If no. of rows is bigger, then data traverse in
 * a hor. link followed by a ver. link in a repeated
 * fashion (similar to a balanced tree) until there are no
 * hor. links left. After this it goes through the remaining vertical
 * links.
 */
void
Htree2::in_htree() {
    //temp var
    double s1 = 0, s2 = 0, s3 = 0;
    double l_eff = 0;
    Wire *wtemp1 = 0, *wtemp2 = 0, *wtemp3 = 0;
    double len = 0, ht = 0;
    int option = 0;

    int h = (int) _log2(ndwl / 2); // horizontal nodes
    int v = (int) _log2(ndbl / 2); // vertical nodes
    double len_temp;
    double ht_temp;
    if (uca_tree) {
        //Sheng: this computation do not consider the wires that route from
        //edge to middle.
        ht_temp = (mat_height * ndbl / 2 +
                   /* since uca_tree models interbank tree,
                      mat_height => bank height */
                   ((add_bits + data_in_bits + data_out_bits +
                     (search_data_in_bits + search_data_out_bits)) *
                    g_tp.wire_outside_mat.pitch *
                    2 * (1 - pow(0.5, h)))) / 2;
        len_temp = (mat_width * ndwl / 2 +
                    ((add_bits + data_in_bits + data_out_bits +
                      (search_data_in_bits + search_data_out_bits)) *
                     g_tp.wire_outside_mat.pitch *
                     2 * (1 - pow(0.5, v)))) / 2;
    } else {
        if (ndwl == ndbl) {
            ht_temp = ((mat_height * ndbl / 2) +
                       ((add_bits + (search_data_in_bits +
                                     search_data_out_bits)) * (ndbl / 2 - 1) *
                        g_tp.wire_outside_mat.pitch) +
                       ((data_in_bits + data_out_bits) *
                        g_tp.wire_outside_mat.pitch * h)
                      ) / 2;
            len_temp = (mat_width * ndwl / 2 +
                        ((add_bits + (search_data_in_bits +
                                      search_data_out_bits)) * (ndwl / 2 - 1) *
                         g_tp.wire_outside_mat.pitch) +
                        ((data_in_bits + data_out_bits) *
                         g_tp.wire_outside_mat.pitch * v)) / 2;
        } else if (ndwl > ndbl) {
            double excess_part = (_log2(ndwl / 2) - _log2(ndbl / 2));
            ht_temp = ((mat_height * ndbl / 2) +
                       ((add_bits + + (search_data_in_bits +
                                       search_data_out_bits)) *
                        ((ndbl / 2 - 1) + excess_part) *
                        g_tp.wire_outside_mat.pitch) +
                       (data_in_bits + data_out_bits) *
                       g_tp.wire_outside_mat.pitch *
                       (2 * (1 - pow(0.5, h - v)) + pow(0.5, v - h) * v)) / 2;
            len_temp = (mat_width * ndwl / 2 +
                        ((add_bits +
                          (search_data_in_bits + search_data_out_bits)) *
                         (ndwl / 2 - 1) * g_tp.wire_outside_mat.pitch) +
                        ((data_in_bits + data_out_bits) *
                         g_tp.wire_outside_mat.pitch * v)) / 2;
        } else {
            double excess_part = (_log2(ndbl / 2) - _log2(ndwl / 2));
            ht_temp = ((mat_height * ndbl / 2) +
                       ((add_bits +
                         (search_data_in_bits + search_data_out_bits)) *
                        ((ndwl / 2 - 1) + excess_part) *
                        g_tp.wire_outside_mat.pitch) +
                       ((data_in_bits + data_out_bits) *
                        g_tp.wire_outside_mat.pitch * h)
                      ) / 2;
            len_temp = (mat_width * ndwl / 2 +
                        ((add_bits +
                          (search_data_in_bits + search_data_out_bits)) *
                         ((ndwl / 2 - 1) + excess_part) *
                         g_tp.wire_outside_mat.pitch) +
                        (data_in_bits + data_out_bits) *
                        g_tp.wire_outside_mat.pitch *
                        (h + 2 * (1 - pow(0.5, v - h)))) / 2;
        }
    }

    area.h   = ht_temp * 2;
    area.w   = len_temp * 2;
    delay = 0;
    power.readOp.dynamic = 0;
    power.readOp.leakage = 0;
    power.searchOp.dynamic = 0;
    len = len_temp;
    ht  = ht_temp / 2;

    while (v > 0 || h > 0) {
        if (wtemp1) delete wtemp1;
        if (wtemp2) delete wtemp2;
        if (wtemp3) delete wtemp3;

        if (h > v) {
            //the iteration considers only one horizontal link
            wtemp1 = new Wire(wt, len); // hor
            wtemp2 = new Wire(wt, len / 2);  // ver
            len_temp = len;
            len /= 2;
            wtemp3 = 0;
            h--;
            option = 0;
        } else if (v > 0 && h > 0) {
            //considers one horizontal link and one vertical link
            wtemp1 = new Wire(wt, len); // hor
            wtemp2 = new Wire(wt, ht);  // ver
            wtemp3 = new Wire(wt, len / 2);  // next hor
            len_temp = len;
            ht_temp = ht;
            len /= 2;
            ht  /= 2;
            v--;
            h--;
            option = 1;
        } else {
            // considers only one vertical link
            assert(h == 0);
            wtemp1 = new Wire(wt, ht); // ver
            wtemp2 = new Wire(wt, ht / 2);  // hor
            ht_temp = ht;
            ht /= 2;
            wtemp3 = 0;
            v--;
            option = 2;
        }

        delay += wtemp1->delay;
        power.readOp.dynamic += wtemp1->power.readOp.dynamic;
        power.searchOp.dynamic += wtemp1->power.readOp.dynamic * wire_bw;
        power.readOp.leakage += wtemp1->power.readOp.leakage * wire_bw;
        power.readOp.gate_leakage += wtemp1->power.readOp.gate_leakage * wire_bw;
        if ((uca_tree == false && option == 2) || search_tree == true) {
            wire_bw *= 2;  // wire bandwidth doubles only for vertical branches
        }

        if (uca_tree == false) {
            if (len_temp > wtemp1->repeater_spacing) {
                s1 = wtemp1->repeater_size;
                l_eff = wtemp1->repeater_spacing;
            } else {
                s1 = (len_temp / wtemp1->repeater_spacing) *
                    wtemp1->repeater_size;
                l_eff = len_temp;
            }

            if (ht_temp > wtemp2->repeater_spacing) {
                s2 = wtemp2->repeater_size;
            } else {
                s2 = (len_temp / wtemp2->repeater_spacing) *
                    wtemp2->repeater_size;
            }
            // first level
            input_nand(s1, s2, l_eff);
        }


        if (option != 1) {
            continue;
        }

        // second level
        delay += wtemp2->delay;
        power.readOp.dynamic += wtemp2->power.readOp.dynamic;
        power.searchOp.dynamic += wtemp2->power.readOp.dynamic * wire_bw;
        power.readOp.leakage += wtemp2->power.readOp.leakage * wire_bw;
        power.readOp.gate_leakage += wtemp2->power.readOp.gate_leakage * wire_bw;

        if (uca_tree) {
            power.readOp.leakage += (wtemp2->power.readOp.leakage * wire_bw);
            power.readOp.gate_leakage +=
                wtemp2->power.readOp.gate_leakage * wire_bw;
        } else {
            power.readOp.leakage += (wtemp2->power.readOp.leakage * wire_bw);
            power.readOp.gate_leakage +=
                wtemp2->power.readOp.gate_leakage * wire_bw;
            wire_bw *= 2;

            if (ht_temp > wtemp3->repeater_spacing) {
                s3    = wtemp3->repeater_size;
                l_eff = wtemp3->repeater_spacing;
            } else {
                s3 = (len_temp / wtemp3->repeater_spacing) *
                    wtemp3->repeater_size;
                l_eff = ht_temp;
            }

            input_nand(s2, s3, l_eff);
        }
    }

    if (wtemp1) delete wtemp1;
    if (wtemp2) delete wtemp2;
    if (wtemp3) delete wtemp3;
}



/* a tristate buffer is used to handle fan-ins
 * The area of an unbalanced htree (rows != columns)
 * depends on how data is traversed.
 * In the following function, if ( no. of rows < no. of columns),
 * then data first traverse in excess hor. links until vertical
 * and horizontal nodes are same.
 * If no. of rows is bigger, then data traverse in
 * a hor. link followed by a ver. link in a repeated
 * fashion (similar to a balanced tree) until there are no
 * hor. links left. After this it goes through the remaining vertical
 * links.
 */
void Htree2::out_htree() {
    //temp var
    double s1 = 0, s2 = 0, s3 = 0;
    double l_eff = 0;
    Wire *wtemp1 = 0, *wtemp2 = 0, *wtemp3 = 0;
    double len = 0, ht = 0;
    int option = 0;

    int h = (int) _log2(ndwl / 2);
    int v = (int) _log2(ndbl / 2);
    double len_temp;
    double ht_temp;
    if (uca_tree) {
        ht_temp = (mat_height * ndbl / 2 +
                   /* since uca_tree models interbank tree,
                      mat_height => bank height */
                   ((add_bits + data_in_bits + data_out_bits +
                     (search_data_in_bits + search_data_out_bits)) *
                    g_tp.wire_outside_mat.pitch *
                    2 * (1 - pow(0.5, h)))) / 2;
        len_temp = (mat_width * ndwl / 2 +
                    ((add_bits + data_in_bits + data_out_bits +
                      (search_data_in_bits + search_data_out_bits)) *
                     g_tp.wire_outside_mat.pitch *
                     2 * (1 - pow(0.5, v)))) / 2;
    } else {
        if (ndwl == ndbl) {
            ht_temp = ((mat_height * ndbl / 2) +
                       ((add_bits + (search_data_in_bits +
                                     search_data_out_bits)) *
                        (ndbl / 2 - 1) * g_tp.wire_outside_mat.pitch) +
                       ((data_in_bits + data_out_bits) *
                        g_tp.wire_outside_mat.pitch * h)
                ) / 2;
            len_temp = (mat_width * ndwl / 2 +
                        ((add_bits + (search_data_in_bits +
                                      search_data_out_bits)) * (ndwl / 2 - 1) *
                         g_tp.wire_outside_mat.pitch) +
                        ((data_in_bits + data_out_bits) *
                         g_tp.wire_outside_mat.pitch * v)) / 2;

        } else if (ndwl > ndbl) {
            double excess_part = (_log2(ndwl / 2) - _log2(ndbl / 2));
            ht_temp = ((mat_height * ndbl / 2) +
                       ((add_bits +
                         (search_data_in_bits + search_data_out_bits)) *
                        ((ndbl / 2 - 1) + excess_part) *
                        g_tp.wire_outside_mat.pitch) +
                       (data_in_bits + data_out_bits) *
                       g_tp.wire_outside_mat.pitch *
                       (2 * (1 - pow(0.5, h - v)) + pow(0.5, v - h) * v)) / 2;
            len_temp = (mat_width * ndwl / 2 +
                        ((add_bits +
                          (search_data_in_bits + search_data_out_bits)) *
                         (ndwl / 2 - 1) * g_tp.wire_outside_mat.pitch) +
                        ((data_in_bits + data_out_bits) *
                         g_tp.wire_outside_mat.pitch * v)) / 2;
        } else {
            double excess_part = (_log2(ndbl / 2) - _log2(ndwl / 2));
            ht_temp = ((mat_height * ndbl / 2) +
                       ((add_bits +
                         (search_data_in_bits + search_data_out_bits)) *
                        ((ndwl / 2 - 1) + excess_part) *
                        g_tp.wire_outside_mat.pitch) +
                       ((data_in_bits + data_out_bits) *
                        g_tp.wire_outside_mat.pitch * h)
                      ) / 2;
            len_temp = (mat_width * ndwl / 2 +
                        ((add_bits + (search_data_in_bits +
                                      search_data_out_bits)) *
                         ((ndwl / 2 - 1) + excess_part) *
                         g_tp.wire_outside_mat.pitch) +
                        (data_in_bits + data_out_bits) *
                        g_tp.wire_outside_mat.pitch *
                        (h + 2 * (1 - pow(0.5, v - h)))) / 2;
        }
    }
    area.h = ht_temp * 2;
    area.w = len_temp * 2;
    delay = 0;
    power.readOp.dynamic = 0;
    power.readOp.leakage = 0;
    power.readOp.gate_leakage = 0;
    //cout<<"power.readOp.gate_leakage"<<power.readOp.gate_leakage<<endl;
    len = len_temp;
    ht = ht_temp / 2;

    while (v > 0 || h > 0) { //finds delay/power of each link in the tree
        if (wtemp1) delete wtemp1;
        if (wtemp2) delete wtemp2;
        if (wtemp3) delete wtemp3;

        if (h > v) {
            //the iteration considers only one horizontal link
            wtemp1 = new Wire(wt, len); // hor
            wtemp2 = new Wire(wt, len / 2);  // ver
            len_temp = len;
            len /= 2;
            wtemp3 = 0;
            h--;
            option = 0;
        } else if (v > 0 && h > 0) {
            //considers one horizontal link and one vertical link
            wtemp1 = new Wire(wt, len); // hor
            wtemp2 = new Wire(wt, ht);  // ver
            wtemp3 = new Wire(wt, len / 2);  // next hor
            len_temp = len;
            ht_temp = ht;
            len /= 2;
            ht /= 2;
            v--;
            h--;
            option = 1;
        } else {
            // considers only one vertical link
            assert(h == 0);
            wtemp1 = new Wire(wt, ht); // hor
            wtemp2 = new Wire(wt, ht / 2);  // ver
            ht_temp = ht;
            ht /= 2;
            wtemp3 = 0;
            v--;
            option = 2;
        }
        delay += wtemp1->delay;
        power.readOp.dynamic += wtemp1->power.readOp.dynamic;
        power.searchOp.dynamic += wtemp1->power.readOp.dynamic * init_wire_bw;
        power.readOp.leakage += wtemp1->power.readOp.leakage * wire_bw;
        power.readOp.gate_leakage += wtemp1->power.readOp.gate_leakage * wire_bw;
        if ((uca_tree == false && option == 2) || search_tree == true) {
            wire_bw *= 2;
        }

        if (uca_tree == false) {
            if (len_temp > wtemp1->repeater_spacing) {
                s1 = wtemp1->repeater_size;
                l_eff = wtemp1->repeater_spacing;
            } else {
                s1 = (len_temp / wtemp1->repeater_spacing) *
                    wtemp1->repeater_size;
                l_eff = len_temp;
            }
            if (ht_temp > wtemp2->repeater_spacing) {
                s2 = wtemp2->repeater_size;
            } else {
                s2 = (len_temp / wtemp2->repeater_spacing) *
                    wtemp2->repeater_size;
            }
            // first level
            output_buffer(s1, s2, l_eff);
        }


        if (option != 1) {
            continue;
        }

        // second level
        delay += wtemp2->delay;
        power.readOp.dynamic += wtemp2->power.readOp.dynamic;
        power.searchOp.dynamic += wtemp2->power.readOp.dynamic * init_wire_bw;
        power.readOp.leakage += wtemp2->power.readOp.leakage * wire_bw;
        power.readOp.gate_leakage += wtemp2->power.readOp.gate_leakage * wire_bw;
        //cout<<"power.readOp.gate_leakage"<<power.readOp.gate_leakage<<endl;
        if (uca_tree) {
            power.readOp.leakage += (wtemp2->power.readOp.leakage * wire_bw);
            power.readOp.gate_leakage +=
                wtemp2->power.readOp.gate_leakage * wire_bw;
        } else {
            power.readOp.leakage += (wtemp2->power.readOp.leakage * wire_bw);
            power.readOp.gate_leakage +=
                wtemp2->power.readOp.gate_leakage * wire_bw;
            wire_bw *= 2;

            if (ht_temp > wtemp3->repeater_spacing) {
                s3 = wtemp3->repeater_size;
                l_eff = wtemp3->repeater_spacing;
            } else {
                s3 = (len_temp / wtemp3->repeater_spacing) *
                    wtemp3->repeater_size;
                l_eff = ht_temp;
            }

            output_buffer(s2, s3, l_eff);
        }
        //cout<<"power.readOp.leakage"<<power.readOp.leakage<<endl;
        //cout<<"power.readOp.gate_leakage"<<power.readOp.gate_leakage<<endl;
        //cout<<"wtemp2->power.readOp.gate_leakage"<<wtemp2->power.readOp.gate_leakage<<endl;
    }

    if (wtemp1) delete wtemp1;
    if (wtemp2) delete wtemp2;
    if (wtemp3) delete wtemp3;
}

