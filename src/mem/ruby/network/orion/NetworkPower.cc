/*
 * Copyright (c) 2010 Massachusetts Institute of Technology
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
 *
 * Authors: Chia-Hsin Owen Chen
 *          Tushar Krishna
 */

#include "mem/ruby/network/orion/NetworkPower.hh"
#include "mem/ruby/network/orion/OrionConfig.hh"
#include "mem/ruby/network/orion/OrionLink.hh"
#include "mem/ruby/network/orion/OrionRouter.hh"

void
Router_d::calculate_power()
{
    //Network Activities from garnet
    calculate_performance_numbers();
    double sim_cycles = curCycle() - g_ruby_start;

    // Number of virtual networks/message classes declared in Ruby
    // maybe greater than active virtual networks.
    // Estimate active virtual networks for correct power estimates
    int num_active_vclass = 0;
    std::vector<bool > active_vclass_ary;
    active_vclass_ary.resize(m_virtual_networks);

    std::vector<double > vc_local_arbit_count_active;
    std::vector<double > vc_global_arbit_count_active;
    std::vector<double > buf_read_count_active;
    std::vector<double > buf_write_count_active;

    for (int i =0; i < m_virtual_networks; i++) {

        active_vclass_ary[i] = (get_net_ptr())->validVirtualNetwork(i);
        if (active_vclass_ary[i]) {
            num_active_vclass++;
            vc_local_arbit_count_active.push_back(vc_local_arbit_count[i]);
            vc_global_arbit_count_active.push_back(vc_global_arbit_count[i]);
            buf_read_count_active.push_back(buf_read_count[i]);
            buf_write_count_active.push_back(buf_write_count[i]);
        }
        else {
            // Inactive vclass
            assert(vc_global_arbit_count[i] == 0);
            assert(vc_local_arbit_count[i] == 0);
        }
    }

    // Orion Initialization
    OrionConfig* orion_cfg_ptr;
    OrionRouter* orion_rtr_ptr;
    static double freq_Hz;

    const string cfg_fn = "src/mem/ruby/network/orion/router.cfg";
    orion_cfg_ptr = new OrionConfig(cfg_fn);
    freq_Hz = orion_cfg_ptr->get<double>("FREQUENCY");

    uint32_t num_in_port = m_input_unit.size();
    uint32_t num_out_port = m_output_unit.size();
    uint32_t num_vclass = num_active_vclass;
    std::vector<uint32_t > vclass_type_ary;

    for (int i = 0; i < m_virtual_networks; i++) {
        if (active_vclass_ary[i]) {
            int temp_vc = i*m_vc_per_vnet;
            vclass_type_ary.push_back((uint32_t) 
                m_network_ptr->get_vnet_type(temp_vc));
        }
    }
    assert(vclass_type_ary.size() == num_active_vclass);

    uint32_t num_vc_per_vclass = m_vc_per_vnet;
    uint32_t in_buf_per_data_vc = m_network_ptr->getBuffersPerDataVC();
    uint32_t in_buf_per_ctrl_vc = m_network_ptr->getBuffersPerCtrlVC();
    //flit width in bits
    uint32_t flit_width_bits = m_network_ptr->getNiFlitSize() * 8;

    orion_rtr_ptr = new OrionRouter(
        num_in_port,
        num_out_port,
        num_vclass,
        vclass_type_ary,
        num_vc_per_vclass,
        in_buf_per_data_vc,
        in_buf_per_ctrl_vc,
        flit_width_bits,
        orion_cfg_ptr
    );


    //Power Calculation
    double Pbuf_wr_dyn = 0.0;
    double Pbuf_rd_dyn = 0.0;
    double Pvc_arb_local_dyn = 0.0;
    double Pvc_arb_global_dyn = 0.0;
    double Psw_arb_local_dyn = 0.0;
    double Psw_arb_global_dyn = 0.0;
    double Pxbar_dyn = 0.0;

    double Pbuf_sta = 0.0;
    double Pvc_arb_sta = 0.0;
    double Psw_arb_sta = 0.0;
    double Pxbar_sta = 0.0;

    //Dynamic Power

    // Note: For each active arbiter in vc_arb or sw_arb of size T:1,
    // assuming half the requests (T/2) are high on average.
    // TODO: estimate expected value of requests from simulation.

    for (int i = 0; i < num_vclass; i++) {
        // Buffer Write
        Pbuf_wr_dyn +=
            orion_rtr_ptr->calc_dynamic_energy_buf(i, WRITE_MODE, false)*
                (buf_write_count_active[i]/sim_cycles)*freq_Hz;

        // Buffer Read
        Pbuf_rd_dyn +=
            orion_rtr_ptr->calc_dynamic_energy_buf(i, READ_MODE, false)*
                (buf_read_count_active[i]/sim_cycles)*freq_Hz;

        // VC arbitration local
        // Each input VC arbitrates for one output VC (in its vclass)
        // at its output port.
        // Arbiter size: num_vc_per_vclass:1
        Pvc_arb_local_dyn +=
            orion_rtr_ptr->calc_dynamic_energy_local_vc_arb(i,
                num_vc_per_vclass/2, false)*
                    (vc_local_arbit_count_active[i]/sim_cycles)*
                    freq_Hz;

        // VC arbitration global
        // Each output VC chooses one input VC out of all possible requesting
        // VCs (within vclass) at all input ports
        // Arbiter size: num_in_port*num_vc_per_vclass:1
        // Round-robin at each input VC for outvcs in the local stage will
        // try to keep outvc conflicts to the minimum.
        // Assuming conflicts due to request for same outvc from
        // num_in_port/2 requests.
        // TODO: use garnet to estimate this
        Pvc_arb_global_dyn +=
            orion_rtr_ptr->calc_dynamic_energy_global_vc_arb(i,
                num_in_port/2, false)*
                    (vc_global_arbit_count_active[i]/sim_cycles)*
                    freq_Hz;
    }

    // Switch Allocation Local
    // Each input port chooses one input VC as requestor
    // Arbiter size: num_vclass*num_vc_per_vclass:1
    Psw_arb_local_dyn =
        orion_rtr_ptr->calc_dynamic_energy_local_sw_arb(
            num_vclass*num_vc_per_vclass/2, false)*
            (sw_local_arbit_count/sim_cycles)*
            freq_Hz;

    // Switch Allocation Global
    // Each output port chooses one input port as winner
    // Arbiter size: num_in_port:1
    Psw_arb_global_dyn =
        orion_rtr_ptr->calc_dynamic_energy_global_sw_arb(
            num_in_port/2, false)*
                (sw_global_arbit_count/sim_cycles)*
                freq_Hz;

    // Crossbar
    Pxbar_dyn =
        orion_rtr_ptr->calc_dynamic_energy_xbar(false)*
            (crossbar_count/sim_cycles)*freq_Hz;

    // Total
    m_power_dyn = Pbuf_wr_dyn + Pbuf_rd_dyn +
                  Pvc_arb_local_dyn + Pvc_arb_global_dyn +
                  Psw_arb_local_dyn + Psw_arb_global_dyn +
                  Pxbar_dyn;

    // Clock Power
    m_clk_power = orion_rtr_ptr->calc_dynamic_energy_clock()*freq_Hz;

    // Static Power
    Pbuf_sta = orion_rtr_ptr->get_static_power_buf();
    Pvc_arb_sta = orion_rtr_ptr->get_static_power_va();
    Psw_arb_sta = orion_rtr_ptr->get_static_power_sa();
    Pxbar_sta = orion_rtr_ptr->get_static_power_xbar();

    m_power_sta =  Pbuf_sta + Pvc_arb_sta + Psw_arb_sta + Pxbar_sta;
}

void
NetworkLink_d::calculate_power(double sim_cycles)
{
    OrionConfig* orion_cfg_ptr;
    OrionLink* orion_link_ptr;
    static double freq_Hz;
    double link_length;

    // Initialization
    const string cfg_fn = "src/mem/ruby/network/orion/router.cfg";
    orion_cfg_ptr = new OrionConfig(cfg_fn);
    freq_Hz = orion_cfg_ptr->get<double>("FREQUENCY");

    link_length = orion_cfg_ptr->get<double>("LINK_LENGTH");

    int channel_width_bits = channel_width*8;

    orion_link_ptr = new OrionLink(
        link_length,
        channel_width_bits,
        orion_cfg_ptr);

    // Dynamic Power
    // Assume half the bits flipped on every link activity
    double link_dynamic_energy =
        orion_link_ptr->calc_dynamic_energy(channel_width_bits/2);
    m_power_dyn = link_dynamic_energy * (m_link_utilized  / sim_cycles) *
                  freq_Hz;


    // Static Power
    // Calculates number of repeaters needed in link, and their static power
    // For short links, like 1mm, no repeaters are needed so static power is 0
    m_power_sta = orion_link_ptr->get_static_power();

    delete orion_link_ptr;
    delete orion_cfg_ptr;
}
