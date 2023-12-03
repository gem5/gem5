/*
 * Copyright (c) 2011 Massachusetts Institute of Technology
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

/*
 * Official Tool Website: www.mit.edu/~kaisopos/FaultModel
 *
 * If you use our tool for academic research, we request that you cite:
 * Konstantinos Aisopos, Chia-Hsin Owen Chen, and Li-Shiuan Peh. Enabling
 * System-Level Modeling of Variation-Induced Faults in Networks-on-Chip.
 * Proceedings of the 48th Design Automation Conference (DAC'11)
 */

// C++ includes
#include <cassert>
#include <fstream>
#include <iostream>
#include <vector>

// GEM5 includes
#include "FaultModel.hh"
#include "base/logging.hh"

#define MAX(a, b) ((a > b) ? (a) : (b))

namespace gem5
{

namespace ruby
{

FaultModel::FaultModel(const Params &p) : SimObject(p)
{
    // read configurations into "configurations" vector
    // format: <buff/vc> <vcs> <10 fault types>
    bool more_records = true;
    for (int i = 0; more_records; i += (fields_per_conf_record)) {
        system_conf configuration;
        configuration.buff_per_vc =
            p.baseline_fault_vector_database[i + conf_record_buff_per_vc];
        configuration.vcs =
            p.baseline_fault_vector_database[i + conf_record_vcs];
        for (int fault_index = 0; fault_index < number_of_fault_types;
             fault_index++) {
            configuration.fault_type[fault_index] =
                p.baseline_fault_vector_database[i +
                                                 conf_record_first_fault_type +
                                                 fault_index] /
                100;
        }
        configurations.push_back(configuration);
        if (p.baseline_fault_vector_database[i + fields_per_conf_record] < 0) {
            more_records = false;
        }
    }

    // read temperature weights into "temperature_weights" vector
    // format: <temperature> <weight>
    more_records = true;
    for (int i = 0; more_records; i += (fields_per_temperature_record)) {
        int record_temperature =
            p.temperature_weights_database[i + temperature_record_temp];
        int record_weight =
            p.temperature_weights_database[i + temperature_record_weight];
        static int first_record = true;
        if (first_record) {
            for (int temperature = 0; temperature < record_temperature;
                 temperature++) {
                temperature_weights.push_back(0);
            }
            first_record = false;
        }
        assert(record_temperature == temperature_weights.size());
        temperature_weights.push_back(record_weight);
        if (p.temperature_weights_database[i + fields_per_temperature_record] <
            0) {
            more_records = false;
        }
    }
}

std::string
FaultModel::fault_type_to_string(int ft)
{
    if (ft == data_corruption__few_bits) {
        return "data_corruption__few_bits";
    } else if (ft == data_corruption__all_bits) {
        return "data_corruption__all_bits";
    } else if (ft == flit_conservation__flit_duplication) {
        return "flit_conservation__flit_duplication";
    } else if (ft == flit_conservation__flit_loss_or_split) {
        return "flit_conservation__flit_loss_or_split";
    } else if (ft == misrouting) {
        return "misrouting";
    } else if (ft == credit_conservation__credit_generation) {
        return "credit_conservation__credit_generation";
    } else if (ft == credit_conservation__credit_loss) {
        return "credit_conservation__credit_loss";
    } else if (ft == erroneous_allocation__VC) {
        return "erroneous_allocation__VC";
    } else if (ft == erroneous_allocation__switch) {
        return "erroneous_allocation__switch";
    } else if (ft == unfair_arbitration) {
        return "unfair_arbitration";
    } else if (ft == number_of_fault_types) {
        return "none";
    } else {
        return "none";
    }
}

int
FaultModel::declare_router(int number_of_inputs, int number_of_outputs,
                           int number_of_vcs_per_input,
                           int number_of_buff_per_data_vc,
                           int number_of_buff_per_ctrl_vc)
{
    // check inputs (are they legal?)
    if (number_of_inputs <= 0 || number_of_outputs <= 0 ||
        number_of_vcs_per_input <= 0 || number_of_buff_per_data_vc <= 0 ||
        number_of_buff_per_ctrl_vc <= 0) {
        fatal("Fault Model: ERROR in argument of FaultModel_declare_router!");
    }
    int number_of_buffers_per_vc =
        MAX(number_of_buff_per_data_vc, number_of_buff_per_ctrl_vc);
    int total_vcs = number_of_inputs * number_of_vcs_per_input;
    if (total_vcs > MAX_VCs) {
        fatal("Fault Model: ERROR! Number inputs*VCs (MAX_VCs) unsupported");
    }
    if (number_of_buffers_per_vc > MAX_BUFFERS_per_VC) {
        fatal("Fault Model: ERROR! buffers/VC (MAX_BUFFERS_per_VC) too high");
    }

    // link the router to a DB record
    int record_hit = -1;
    for (int record = 0; record < configurations.size(); record++) {
        if ((configurations[record].buff_per_vc == number_of_buffers_per_vc) &&
            (configurations[record].vcs == total_vcs)) {
            record_hit = record;
        }
    }
    if (record_hit == -1) {
        panic("Fault Model: ERROR! configuration not found in DB. BUG?");
    }

    // remember the router and return its ID
    routers.push_back(configurations[record_hit]);
    static int router_index = 0;
    return router_index++;
}

bool
FaultModel::fault_vector(int routerID, int temperature_input,
                         float fault_vector[])
{
    bool ok = true;

    // is the routerID recorded?
    if (routerID < 0 || routerID >= ((int)routers.size())) {
        warn("Fault Model: ERROR! unknown router ID argument.");
        fatal("Fault Model: Did you enable the fault model flag)?");
    }

    // is the temperature too high/too low?
    int temperature = temperature_input;
    if (temperature_input >= ((int)temperature_weights.size())) {
        ok = false;
        warn_once("Fault Model: Temperature exceeded simulated upper bound.");
        warn_once("Fault Model: The fault model is not accurate any more.");
        temperature = (temperature_weights.size() - 1);
    } else if (temperature_input < 0) {
        ok = false;
        warn_once("Fault Model: Temperature exceeded simulated lower bound.");
        warn_once("Fault Model: The fault model is not accurate any more.");
        temperature = 0;
    }

    // recover the router record and return its fault vector
    for (int i = 0; i < number_of_fault_types; i++) {
        fault_vector[i] = routers[routerID].fault_type[i] *
                          ((float)temperature_weights[temperature]);
    }
    return ok;
}

bool
FaultModel::fault_prob(int routerID, int temperature_input,
                       float *aggregate_fault_prob)
{
    *aggregate_fault_prob = 1.0;
    bool ok = true;

    // is the routerID recorded?
    if (routerID < 0 || routerID >= ((int)routers.size())) {
        warn("Fault Model: ERROR! unknown router ID argument.");
        fatal("Fault Model: Did you enable the fault model flag)?");
    }

    // is the temperature too high/too low?
    int temperature = temperature_input;
    if (temperature_input >= ((int)temperature_weights.size())) {
        ok = false;
        warn_once("Fault Model: Temperature exceeded simulated upper bound.");
        warn_once("Fault Model: The fault model is not accurate any more.");
        temperature = (temperature_weights.size() - 1);
    } else if (temperature_input < 0) {
        ok = false;
        warn_once("Fault Model: Temperature exceeded simulated lower bound.");
        warn_once("Fault Model: The fault model is not accurate any more.");
        temperature = 0;
    }

    // recover the router record and return its aggregate fault probability
    for (int i = 0; i < number_of_fault_types; i++) {
        *aggregate_fault_prob =
            *aggregate_fault_prob *
            (1.0 - (routers[routerID].fault_type[i] *
                    ((float)temperature_weights[temperature])));
    }
    *aggregate_fault_prob = 1.0 - *aggregate_fault_prob;
    return ok;
}

// this function is used only for debugging purposes
void
FaultModel::print(void)
{
    std::cout << "--- PRINTING configurations ---\n";
    for (int record = 0; record < configurations.size(); record++) {
        std::cout << "(" << record << ") ";
        std::cout << "VCs=" << configurations[record].vcs << " ";
        std::cout << "Buff/VC=" << configurations[record].buff_per_vc << " [";
        for (int fault_type_num = 0; fault_type_num < number_of_fault_types;
             fault_type_num++) {
            std::cout << (100 *
                          configurations[record].fault_type[fault_type_num]);
            std::cout << "% ";
        }
        std::cout << "]\n";
    }
    std::cout << "--- PRINTING temperature weights ---\n";
    for (int record = 0; record < temperature_weights.size(); record++) {
        std::cout << "temperature=" << record << " => ";
        std::cout << "weight=" << temperature_weights[record];
        std::cout << "\n";
    }
}

} // namespace ruby
} // namespace gem5
