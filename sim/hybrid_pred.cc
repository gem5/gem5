/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <string>
#include <sstream>

#include "hybrid_pred.hh"
#include "statistics.hh"
#include "sim_stats.hh"

using namespace std;

HybridPredictor::HybridPredictor(const char *_p_name, const char *_z_name,
                                 const char *_o_name,
                                 unsigned _index_bits, unsigned _counter_bits,
                                 unsigned _zero_change, unsigned _one_change,
                                 unsigned _thresh,
                                 unsigned _global_bits,
                                 unsigned _global_thresh,
                                 bool _reg_individual_stats)
{
    stringstream local_name, global_name;

    pred_name = _p_name;
    one_name  = _o_name;
    zero_name = _z_name;
    reg_individual_stats = _reg_individual_stats;

    local_name << pred_name.c_str() << ":L";
    local = new SaturatingCounterPred(local_name.str(), zero_name, one_name,
                                      _index_bits, _counter_bits,
                                      _zero_change, _one_change, _thresh);

    global_name << pred_name.c_str() << ":G";
    global = new SaturatingCounterPred(global_name.str(), zero_name, one_name,
                                       0, _global_bits, 1, 1, _global_thresh);
}

void HybridPredictor::regStats()
{
    using namespace Statistics;

    string p_name;
    stringstream description;

    if (reg_individual_stats)
        p_name = pred_name + ":A";
    else
        p_name = pred_name;


    //
    //  Number of predictions
    //
    stringstream num_zero_preds;
    num_zero_preds << p_name << ":" << zero_name << ":preds";
    description << "number of predictions of " << zero_name;
    pred_zero
        .name(num_zero_preds.str())
        .desc(description.str());
    description.str("");

    stringstream num_one_preds;
    num_one_preds << p_name << ":" << one_name << ":preds";
    description << "number of predictions of " << one_name;
    pred_one
        .name(num_one_preds.str())
        .desc(description.str())
        ;
    description.str("");

    //
    //  Count the number of correct predictions
    //
    stringstream num_zero_correct;
    num_zero_correct << p_name << ":" << zero_name << ":corr_preds";
    description << "number of correct " << zero_name << " preds" ;
    correct_pred_zero
        .name(num_zero_correct.str())
        .desc(description.str())
        ;
    description.str("");

    stringstream num_one_correct;
    num_one_correct << p_name << ":" << one_name << ":corr_preds";
    description << "number of correct " << one_name << " preds" ;
    correct_pred_one
        .name(num_one_correct.str())
        .desc(description.str())
        ;
    description.str("");


    //
    //  Number of predictor updates
    //
    stringstream num_zero_updates;
    num_zero_updates << p_name << ":" << zero_name << ":updates" ;
    description << "number of actual " << zero_name << "s" ;
    record_zero
        .name(num_zero_updates.str())
        .desc(description.str())
        ;
    description.str("");

    stringstream num_one_updates;
    num_one_updates << p_name << ":" << one_name << ":updates" ;
    description << "number of actual " << one_name << "s" ;
    record_one
        .name(num_one_updates.str())
        .desc(description.str())
        ;
    description.str("");

    //
    //  Local & Global predictor stats
    //
    if (reg_individual_stats) {
        local->regStats();
        global->regStats();
    }
}

void HybridPredictor::regFormulas()
{
    using namespace Statistics;

    string p_name;
    stringstream description;
    stringstream name;

    if (reg_individual_stats)
        p_name = pred_name + ":A";
    else
        p_name = pred_name;

    //
    //  Number of predictions
    //
    name << p_name << ":predictions" ;
    total_preds
        .name(name.str())
        .desc("total number of predictions made")
        ;
    total_preds = pred_one + pred_zero;
    name.str("");

    //
    //  Fraction of all predictions that are one or zero
    //
    name << p_name << ":" << zero_name << ":pred_frac";
    description << "fraction of all preds that were " << zero_name ;
    frac_preds_zero
        .name(name.str())
        .desc(description.str())
        ;
    frac_preds_zero = 100 * record_zero / total_preds;
    description.str("");
    name.str("");

    name << p_name << ":" << one_name << ":pred_frac";
    description << "fraction of all preds that were " << one_name ;
    frac_preds_one
        .name(name.str())
        .desc(description.str())
        ;
    frac_preds_one = 100 * record_one / total_preds;
    description.str("");
    name.str("");

    //
    //  Count the number of correct predictions
    //
    name << p_name << ":correct_preds" ;
    total_correct
        .name(name.str())
        .desc("total number of correct predictions made")
        ;
    total_correct = correct_pred_one + correct_pred_zero;
    name.str("");


    //
    //  Prediction accuracy rates
    //
    name << p_name << ":pred_rate";
    total_accuracy
        .name(name.str())
        .desc("fraction of all preds that were correct")
        ;
    total_accuracy = 100 * total_correct / total_preds;
    name.str("");

    name << p_name << ":" << zero_name << ":pred_rate" ;
    description << "fraction of "<< zero_name <<" preds that were correct";
    zero_accuracy
        .name(name.str())
        .desc(description.str())
        ;
    zero_accuracy = 100 * correct_pred_zero / pred_zero;
    description.str("");
    name.str("");

    name << p_name << ":" << one_name << ":pred_rate" ;
    description << "fraction of "<< one_name <<" preds that were correct";
    one_accuracy
        .name(name.str())
        .desc(description.str())
        ;
    one_accuracy = 100 * correct_pred_one / pred_one;
    description.str("");
    name.str("");

    //
    //  Coverage
    //
    name << p_name << ":" << zero_name << ":coverage";
    description << "fraction of " << zero_name
                << "s that were predicted correctly";
    zero_coverage
        .name(name.str())
        .desc(description.str())
        ;
    zero_coverage = 100 * correct_pred_zero / record_zero;
    description.str("");
    name.str("");

    name << p_name << ":" << one_name << ":coverage";
    description << "fraction of " << one_name
                << "s that were predicted correctly";
    one_coverage
        .name(name.str())
        .desc(description.str())
        ;
    one_coverage = 100 * correct_pred_one / record_one;
    description.str("");
    name.str("");

    //
    //  Local & Global predictor stats
    //
    if (reg_individual_stats) {
        local->regFormulas();
        global->regFormulas();
    }

}

