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

#include <stdio.h>

#include "power_router_init.hh"
#include "power_array.hh"
#include "power_arbiter.hh"
#include "power_crossbar.hh"
#include "power_ll.hh"
#include "parm_technology.hh"
#include "SIM_port.hh"
#include "power_static.hh"
#include "power_utils.hh"

/* -------------------------------------------------------------------------------------------- */
// Set buffer parameters
int buf_set_para(power_array_info *info, int is_fifo, unsigned n_read_port, unsigned n_write_port, unsigned n_entry, unsigned line_width, int outdrv)
{
        //general parameters
        info->share_rw = 0;
        info->read_ports = n_read_port;
        info->write_ports = n_write_port;
        info->n_set = n_entry;
        info->blk_bits = line_width;
        info->assoc = 1;
        info->data_width = line_width;
        info->data_end = PARM_data_end;

        //no array subpartition
        info->data_ndwl = 1;
        info->data_ndbl = 1;
        info->data_nspd = 1;

        info->data_n_share_amp =1;

        //MODEL parameters
        if(is_fifo) {
                info->row_dec_model = SIM_NO_MODEL;
                info->row_dec_pre_model = SIM_NO_MODEL;
        }
        else {
                info->row_dec_model = PARM_row_dec_model;
                info->row_dec_pre_model = PARM_row_dec_pre_model;
        }

        info->data_wordline_model = PARM_wordline_model;
        info->data_bitline_model = PARM_bitline_model;
        info->data_bitline_pre_model = PARM_bitline_pre_model;
        info->data_mem_model = PARM_mem_model;

        if(PARM_data_end == 2)
                info->data_amp_model = PARM_amp_model;
        else
                info->data_amp_model = SIM_NO_MODEL;
        if(outdrv)
                info->outdrv_model = PARM_outdrv_model;
        else
                info->outdrv_model = SIM_NO_MODEL;

        info->data_colsel_pre_model = SIM_NO_MODEL;
        info->col_dec_model = SIM_NO_MODEL;
        info->col_dec_pre_model = SIM_NO_MODEL;
        info->mux_model = SIM_NO_MODEL;

        //no tag array

        info->tag_wordline_model = SIM_NO_MODEL;
        info->tag_bitline_model = SIM_NO_MODEL;
        info->tag_bitline_pre_model = SIM_NO_MODEL;
        info->tag_mem_model = SIM_NO_MODEL;
        info->tag_attach_mem_model = SIM_NO_MODEL;
        info->tag_amp_model = SIM_NO_MODEL;
        info->tag_colsel_pre_model = SIM_NO_MODEL;
        info->comp_model = SIM_NO_MODEL;
        info->comp_pre_model = SIM_NO_MODEL;

        info->write_policy = 0; //no dirty bit

        //derived
        if(info->data_width != 0){
    info->n_item = info->blk_bits / info->data_width;
  }
  else{
    info->eff_data_cols = info->blk_bits * info->assoc * info->data_nspd;
  }

        return 0;
}
/* -------------------------------------------------------------------------------------------- */


/* --------------- Router init --------------------------------------*/


int power_router_init(power_router *router, power_router_info *info)
{
        int outdrv;
        double req_len = 0;

        //general
//      info->n_in = PARM_in_port;
        info->n_total_in = info->n_in;
//      info->n_out = PARM_out_port;
        info->n_total_out = info->n_out;
//      info->flit_width = PARM_flit_width;

        //vc
//      info->n_v_channel = MAX(PARM_v_channel, 1);
//      info->n_v_class = MAX(info->n_v_channel, PARM_v_class);

        if(info->n_v_class > 1) {
                info->in_share_buf = PARM_in_share_buf;
                info->out_share_buf = PARM_out_share_buf;
                info->in_share_switch = PARM_in_share_switch;
                info->out_share_switch = PARM_out_share_switch;
        }
        else {
                info->in_share_buf = 0;
                info->out_share_buf = 0;
                info->in_share_switch = 0;
                info->out_share_switch = 0;
        }

        //xbar
        info->crossbar_model = PARM_crossbar_model;
        info->degree = PARM_crsbar_degree;
        info->connect_type = PARM_connect_type;
        info->trans_type = PARM_trans_type;
        info->crossbar_in_len = PARM_crossbar_in_len;
        info->crossbar_out_len = PARM_crossbar_out_len;

        //input buffer
        info->in_buf = PARM_in_buf;
        outdrv = !info->in_share_buf && info->in_share_switch;
        buf_set_para(&info->in_buf_info, 0, PARM_in_buf_rport, 1, PARM_in_buf_set, info->flit_width, outdrv);

        //vc arbiter
        if(info->n_v_class > 1) {
                info->vc_in_arb_model = PARM_vc_in_arb_model;
                info->vc_out_arb_model = PARM_vc_out_arb_model;
                if(info->vc_in_arb_model == QUEUE_ARBITER) {
                        buf_set_para(&info->vc_in_arb_queue_info, 1, 1, 1, info->n_v_class, SIM_power_logtwo(info->n_v_class), 0);
                        info->vc_in_arb_ff_model = SIM_NO_MODEL;
                }
                else
                        info->vc_in_arb_ff_model = PARM_vc_in_arb_ff_model;

                if(info->vc_out_arb_model == QUEUE_ARBITER) {
                        buf_set_para(&info->vc_out_arb_queue_info, 1, 1, 1, info->n_total_in - 1, SIM_power_logtwo(info->n_total_in - 1), 0);
                        info->vc_out_arb_ff_model = SIM_NO_MODEL;
                }
                else
                        info->vc_out_arb_ff_model = PARM_vc_out_arb_ff_model;
        }
        else {
                info->vc_in_arb_model = SIM_NO_MODEL;
                info->vc_in_arb_ff_model = SIM_NO_MODEL;
                info->vc_out_arb_model = SIM_NO_MODEL;
                info->vc_out_arb_ff_model = SIM_NO_MODEL;
        }

        //switch arbiter
        if (info->n_in > 2) {
                info->sw_in_arb_model = PARM_sw_in_arb_model;
                info->sw_out_arb_model = PARM_sw_out_arb_model;
                if(info->sw_in_arb_model == QUEUE_ARBITER) {
                        buf_set_para(&info->sw_in_arb_queue_info, 1, 1, 1, info->n_v_class, SIM_power_logtwo(info->n_v_class), 0);
                        info->sw_in_arb_ff_model = SIM_NO_MODEL;
                }
                else
                        info->sw_in_arb_ff_model = PARM_sw_in_arb_ff_model;

                if(info->sw_out_arb_model == QUEUE_ARBITER) {
                        buf_set_para(&info->sw_out_arb_queue_info, 1, 1, 1, info->n_total_in - 1, SIM_power_logtwo(info->n_total_in - 1), 0);
                        info->sw_out_arb_ff_model = SIM_NO_MODEL;
                }
                else
                        info->sw_out_arb_ff_model = PARM_sw_out_arb_ff_model;
        }
        else {
                info->sw_in_arb_model = SIM_NO_MODEL;
                info->sw_in_arb_ff_model = SIM_NO_MODEL;
                info->sw_out_arb_model = SIM_NO_MODEL;
                info->sw_out_arb_ff_model = SIM_NO_MODEL;
        }

        if(info->in_buf) {
                if(info->in_share_buf)
                        info->in_n_switch = info->in_buf_info.read_ports;
                        else if(info->in_share_switch)
                                info->in_n_switch = 1;
                        else
                                info->in_n_switch = info->n_v_class;
        }
        else
                info->in_n_switch = 1;

        info->n_switch_in = info->n_in * info->in_n_switch;

        info->n_switch_out = info->n_out;

    //-------- call initialize functions -----------

        router->i_leakage = 0;

        //initialize crossbar
        power_crossbar_init(&router->crossbar, info->crossbar_model, info->n_switch_in, info->n_switch_out, info->flit_width, info->degree, info->connect_type, info->trans_type, info->crossbar_in_len, info->crossbar_out_len, &req_len);
        router->i_leakage += router->crossbar.i_leakage;
//      printf("xbar_leak %g", router->crossbar.i_leakage);

        //initialize input buffer
        if(info->in_buf) {
                power_array_init(&info->in_buf_info, &router->in_buf);
                router->i_leakage += router->in_buf.i_leakage * info->n_in;
//              printf("buffer_leak %g", router->in_buf.i_leakage);
        }
//    printf("initialize in buffer over\n");

        //initialize vc arbiter
        if(info->vc_in_arb_model)
                power_arbiter_init(&router->vc_in_arb, info->vc_in_arb_model, info->vc_in_arb_ff_model, PARM_VC_per_MC, 0, &info->vc_in_arb_queue_info);

        if(info->vc_out_arb_model)
                power_arbiter_init(&router->vc_out_arb, info->vc_out_arb_model, info->vc_out_arb_ff_model, info->n_total_in - 1, req_len, &info->vc_out_arb_queue_info);

        //initialize switch arbiter
        if(info->sw_in_arb_model)
                power_arbiter_init(&router->sw_in_arb, info->sw_in_arb_model, info->sw_in_arb_ff_model, info->n_v_class, 0, &info->sw_in_arb_queue_info);

        if(info->sw_out_arb_model)
                power_arbiter_init(&router->sw_out_arb, info->sw_out_arb_model, info->sw_out_arb_ff_model, info->n_total_in - 1, req_len, &info->sw_out_arb_queue_info);

        return 0;
}
