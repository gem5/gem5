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



#include "router.h"

Router::Router(
    double flit_size_,
    double vc_buf, /* vc size = vc_buffer_size * flit_size */
    double vc_c,
    TechnologyParameter::DeviceType *dt,
    double I_,
    double O_,
    double M_
    ): flit_size(flit_size_),
        deviceType(dt),
        I(I_),
        O(O_),
       M(M_) {
    vc_buffer_size = vc_buf;
    vc_count = vc_c;
    min_w_pmos = deviceType->n_to_p_eff_curr_drv_ratio * g_tp.min_w_nmos_;
    double technology = g_ip->F_sz_um;

    Vdd = dt->Vdd;

    /*Crossbar parameters. Transmisson gate is employed for connector*/
    NTtr = 10 * technology * 1e-6 / 2; /*Transmission gate's nmos tr. length*/
    PTtr = 20 * technology * 1e-6 / 2; /* pmos tr. length*/
    wt = 15 * technology * 1e-6 / 2; /*track width*/
    ht = 15 * technology * 1e-6 / 2; /*track height*/
//  I = 5; /*Number of crossbar input ports*/
//  O = 5; /*Number of crossbar output ports*/
    NTi = 12.5 * technology * 1e-6 / 2;
    PTi = 25 * technology * 1e-6 / 2;

    NTid = 60 * technology * 1e-6 / 2; //m
    PTid = 120 * technology * 1e-6 / 2; // m
    NTod = 60 * technology * 1e-6 / 2; // m
    PTod = 120 * technology * 1e-6 / 2; // m

    calc_router_parameters();
}

Router::~Router() {}


double //wire cap with triple spacing
Router::Cw3(double length) {
    Wire wc(g_ip->wt, length, 1, 3, 3);
    return (wc.wire_cap(length));
}

/*Function to calculate the gate capacitance*/
double
Router::gate_cap(double w) {
    return (double) gate_C (w*1e6 /*u*/, 0);
}

/*Function to calculate the diffusion capacitance*/
double
Router::diff_cap(double w, int type /*0 for n-mos and 1 for p-mos*/,
                 double s /*number of stacking transistors*/) {
    return (double) drain_C_(w*1e6 /*u*/, type, (int) s, 1, g_tp.cell_h_def);
}


/*crossbar related functions */

// Model for simple transmission gate
double
Router::transmission_buf_inpcap() {
    return diff_cap(NTtr, 0, 1) + diff_cap(PTtr, 1, 1);
}

double
Router::transmission_buf_outcap() {
    return diff_cap(NTtr, 0, 1) + diff_cap(PTtr, 1, 1);
}

double
Router::transmission_buf_ctrcap() {
    return gate_cap(NTtr) + gate_cap(PTtr);
}

double
Router::crossbar_inpline() {
    return (Cw3(O*flit_size*wt) + O*transmission_buf_inpcap() + gate_cap(NTid) +
            gate_cap(PTid) + diff_cap(NTid, 0, 1) + diff_cap(PTid, 1, 1));
}

double
Router::crossbar_outline() {
    return (Cw3(I*flit_size*ht) + I*transmission_buf_outcap() + gate_cap(NTod) +
            gate_cap(PTod) + diff_cap(NTod, 0, 1) + diff_cap(PTod, 1, 1));
}

double
Router::crossbar_ctrline() {
    return (Cw3(0.5*O*flit_size*wt) + flit_size*transmission_buf_ctrcap() +
            diff_cap(NTi, 0, 1) + diff_cap(PTi, 1, 1) +
            gate_cap(NTi) + gate_cap(PTi));
}

double
Router::tr_crossbar_power() {
    return (crossbar_inpline()*Vdd*Vdd*flit_size / 2 +
            crossbar_outline()*Vdd*Vdd*flit_size / 2) * 2;
}

void Router::buffer_stats() {
    DynamicParameter dyn_p;
    dyn_p.is_tag      = false;
    dyn_p.pure_cam    = false;
    dyn_p.fully_assoc = false;
    dyn_p.pure_ram    = true;
    dyn_p.is_dram     = false;
    dyn_p.is_main_mem = false;
    dyn_p.num_subarrays = 1;
    dyn_p.num_mats = 1;
    dyn_p.Ndbl = 1;
    dyn_p.Ndwl = 1;
    dyn_p.Nspd = 1;
    dyn_p.deg_bl_muxing = 1;
    dyn_p.deg_senseamp_muxing_non_associativity = 1;
    dyn_p.Ndsam_lev_1 = 1;
    dyn_p.Ndsam_lev_2 = 1;
    dyn_p.Ndcm = 1;
    dyn_p.number_addr_bits_mat = 8;
    dyn_p.number_way_select_signals_mat = 1;
    dyn_p.number_subbanks_decode = 0;
    dyn_p.num_act_mats_hor_dir = 1;
    dyn_p.V_b_sense = Vdd; // FIXME check power calc.
    dyn_p.ram_cell_tech_type = 0;
    dyn_p.num_r_subarray = (int) vc_buffer_size;
    dyn_p.num_c_subarray = (int) flit_size * (int) vc_count;
    dyn_p.num_mats_h_dir = 1;
    dyn_p.num_mats_v_dir = 1;
    dyn_p.num_do_b_subbank = (int)flit_size;
    dyn_p.num_di_b_subbank = (int)flit_size;
    dyn_p.num_do_b_mat = (int) flit_size;
    dyn_p.num_di_b_mat = (int) flit_size;
    dyn_p.num_do_b_mat = (int) flit_size;
    dyn_p.num_di_b_mat = (int) flit_size;
    dyn_p.num_do_b_bank_per_port = (int) flit_size;
    dyn_p.num_di_b_bank_per_port = (int) flit_size;
    dyn_p.out_w = (int) flit_size;

    dyn_p.use_inp_params = 1;
    dyn_p.num_wr_ports = (unsigned int) vc_count;
    dyn_p.num_rd_ports = 1;//(unsigned int) vc_count;//based on Bill Dally's book
    dyn_p.num_rw_ports = 0;
    dyn_p.num_se_rd_ports = 0;
    dyn_p.num_search_ports = 0;



    dyn_p.cell.h = g_tp.sram.b_h + 2 * g_tp.wire_outside_mat.pitch * (dyn_p.num_wr_ports +
                   dyn_p.num_rw_ports - 1 + dyn_p.num_rd_ports);
    dyn_p.cell.w = g_tp.sram.b_w + 2 * g_tp.wire_outside_mat.pitch * (dyn_p.num_rw_ports - 1 +
                   (dyn_p.num_rd_ports - dyn_p.num_se_rd_ports) +
                   dyn_p.num_wr_ports) + g_tp.wire_outside_mat.pitch * dyn_p.num_se_rd_ports;

    Mat buff(dyn_p);
    buff.compute_delays(0);
    buff.compute_power_energy();
    buffer.power.readOp  = buff.power.readOp;
    buffer.power.writeOp = buffer.power.readOp; //FIXME
    buffer.area = buff.area;
}



void
Router::cb_stats () {
    if (1) {
        Crossbar c_b(I, O, flit_size);
        c_b.compute_power();
        crossbar.delay = c_b.delay;
        crossbar.power.readOp.dynamic = c_b.power.readOp.dynamic;
        crossbar.power.readOp.leakage = c_b.power.readOp.leakage;
        crossbar.power.readOp.gate_leakage = c_b.power.readOp.gate_leakage;
        crossbar.area = c_b.area;
//  c_b.print_crossbar();
    } else {
        crossbar.power.readOp.dynamic = tr_crossbar_power();
        crossbar.power.readOp.leakage = flit_size * I * O *
            cmos_Isub_leakage(NTtr * g_tp.min_w_nmos_, PTtr * min_w_pmos, 1, tg);
        crossbar.power.readOp.gate_leakage = flit_size * I * O *
            cmos_Ig_leakage(NTtr * g_tp.min_w_nmos_, PTtr * min_w_pmos, 1, tg);
    }
}

void
Router::get_router_power() {
    /* calculate buffer stats */
    buffer_stats();

    /* calculate cross-bar stats */
    cb_stats();

    /* calculate arbiter stats */
    Arbiter vcarb(vc_count, flit_size, buffer.area.w);
    Arbiter cbarb(I, flit_size, crossbar.area.w);
    vcarb.compute_power();
    cbarb.compute_power();
    arbiter.power.readOp.dynamic = vcarb.power.readOp.dynamic * I +
                                   cbarb.power.readOp.dynamic * O;
    arbiter.power.readOp.leakage = vcarb.power.readOp.leakage * I +
                                   cbarb.power.readOp.leakage * O;
    arbiter.power.readOp.gate_leakage = vcarb.power.readOp.gate_leakage * I +
                                        cbarb.power.readOp.gate_leakage * O;

//  arb_stats();
    power.readOp.dynamic = ((buffer.power.readOp.dynamic +
                             buffer.power.writeOp.dynamic) +
                            crossbar.power.readOp.dynamic +
                            arbiter.power.readOp.dynamic) * MIN(I, O) * M;
    double pppm_t[4]    = {1, I, I, 1};
    power = power + (buffer.power * pppm_t + crossbar.power + arbiter.power) *
        pppm_lkg;

}

void
Router::get_router_delay () {
    FREQUENCY = 5; // move this to config file --TODO
    cycle_time = (1 / (double)FREQUENCY) * 1e3; //ps
    delay = 4;
    max_cyc = 17 * g_tp.FO4; //s
    max_cyc *= 1e12; //ps
    if (cycle_time < max_cyc) {
        FREQUENCY = (1 / max_cyc) * 1e3; //GHz
    }
}

void
Router::get_router_area() {
    area.h = I * buffer.area.h;
    area.w = buffer.area.w + crossbar.area.w;
}

void
Router::calc_router_parameters() {
    /* calculate router frequency and pipeline cycles */
    get_router_delay();

    /* router power stats */
    get_router_power();

    /* area stats */
    get_router_area();
}

void
Router::print_router() {
    cout << "\n\nRouter stats:\n";
    cout << "\tRouter Area - " << area.get_area()*1e-6 << "(mm^2)\n";
    cout << "\tMaximum possible network frequency - " << (1 / max_cyc)*1e3
         << "GHz\n";
    cout << "\tNetwork frequency - " << FREQUENCY << " GHz\n";
    cout << "\tNo. of Virtual channels - " << vc_count << "\n";
    cout << "\tNo. of pipeline stages - " << delay << endl;
    cout << "\tLink bandwidth - " << flit_size << " (bits)\n";
    cout << "\tNo. of buffer entries per virtual channel -  "
         << vc_buffer_size << "\n";
    cout << "\tSimple buffer Area - " << buffer.area.get_area()*1e-6
         << "(mm^2)\n";
    cout << "\tSimple buffer access (Read) - "
         << buffer.power.readOp.dynamic * 1e9 << " (nJ)\n";
    cout << "\tSimple buffer leakage - " << buffer.power.readOp.leakage * 1e3
         << " (mW)\n";
    cout << "\tCrossbar Area - " << crossbar.area.get_area()*1e-6
         << "(mm^2)\n";
    cout << "\tCross bar access energy - "
         << crossbar.power.readOp.dynamic * 1e9 << " (nJ)\n";
    cout << "\tCross bar leakage power - "
         << crossbar.power.readOp.leakage * 1e3 << " (mW)\n";
    cout << "\tArbiter access energy (VC arb + Crossbar arb) - "
         << arbiter.power.readOp.dynamic * 1e9 << " (nJ)\n";
    cout << "\tArbiter leakage (VC arb + Crossbar arb) - "
         << arbiter.power.readOp.leakage * 1e3 << " (mW)\n";

}

