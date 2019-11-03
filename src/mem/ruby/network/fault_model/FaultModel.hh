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
 *
 * Authors: Konstantinos Aisopos
 */

/*
 * Official Tool Website: www.mit.edu/~kaisopos/FaultModel
 *
 * If you use our tool for academic research, we request that you cite:
 * Konstantinos Aisopos, Chia-Hsin Owen Chen, and Li-Shiuan Peh. Enabling
 * System-Level Modeling of Variation-Induced Faults in Networks-on-Chip.
 * Proceedings of the 48th Design Automation Conference (DAC'11)
 */

#ifndef __MEM_RUBY_NETWORK_FAULT_MODEL_FAULTMODEL_HH__
#define __MEM_RUBY_NETWORK_FAULT_MODEL_FAULTMODEL_HH__

// tool limitations and fixed inputs
#define MAX_VCs 40
#define MAX_BUFFERS_per_VC 5
#define BASELINE_TEMPERATURE_CELCIUS 71

// C++ includes
#include <string>

// GEM5 includes
#include "params/FaultModel.hh"
#include "sim/sim_object.hh"

class FaultModel : public SimObject
{
  public:
    typedef FaultModelParams Params;
    FaultModel(const Params *p);
    const Params *params() const { return (const Params *)_params; }

    /************************************************************************/
    /**********  THE FAULT TYPES SUPPORTED BY THE FAULT MODEL ***************/
    /************************************************************************/

    enum fault_type
    {
        data_corruption__few_bits,
        data_corruption__all_bits,
        flit_conservation__flit_duplication,
        flit_conservation__flit_loss_or_split,
        misrouting,
        credit_conservation__credit_generation,
        credit_conservation__credit_loss,
        erroneous_allocation__VC,
        erroneous_allocation__switch,
        unfair_arbitration,
        number_of_fault_types
    };

    /************************************************************************/
    /********************  INTERFACE OF THE FAULT MODEL *********************/
    /************************************************************************/

    enum conf_record_format
    {
        conf_record_buff_per_vc,
        conf_record_vcs,
        conf_record_first_fault_type,
        conf_record_last_fault_type = conf_record_first_fault_type + number_of_fault_types - 1,
        fields_per_conf_record
    };

    enum temperature_record_format
    {
        temperature_record_temp,
        temperature_record_weight,
        fields_per_temperature_record
    };

    struct system_conf
    {
        int vcs;
        int buff_per_vc;
        float fault_type[number_of_fault_types];
    };

    int declare_router(int number_of_inputs,
                       int number_of_outputs,
                       int number_of_vcs_per_vnet,
                       int number_of_buff_per_data_vc,
                       int number_of_buff_per_ctrl_vc);

    std::string fault_type_to_string(int fault_type_index);

    // the following 2 functions are called at runtime, to get the probability
    // of each fault type (fault_vector) or the aggregate fault probability
    // (fault_prob). Note: the probability values are provided by reference
    // (in the variables fault_vector[] & aggregate_fault_prob respectively).
    // Both functions also return a success flag (which is always true if
    // temperature ranges from 0C to 125C)

    bool fault_vector(int routerID,
                      int temperature,
                      float fault_vector[]);

    bool fault_prob(int routerID,
                    int temperature,
                    float *aggregate_fault_prob);

    // for debugging purposes

    void print(void);

  private:
    std::vector <system_conf> configurations;
    std::vector <system_conf> routers;
    std::vector <int> temperature_weights;
};

#endif //__MEM_RUBY_NETWORK_FAULT_MODEL_FAULTMODEL_HH__
