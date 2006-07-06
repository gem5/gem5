/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 *          Lisa Hsu
 */

#include <sstream>

#include "base/sat_counter.hh"
#include "base/statistics.hh"
#include "sim/stats.hh"


using namespace std;


SaturatingCounterPred::SaturatingCounterPred(string p_name,
                                             string z_name,
                                             string o_name,
                                             unsigned _index_bits,
                                             unsigned _counter_bits,
                                             unsigned _zero_change,
                                             unsigned _one_change,
                                             unsigned _thresh,
                                             unsigned _init_value)
{
    pred_name    = p_name;
    zero_name    = z_name;
    one_name     = o_name;

    index_bits   = _index_bits;
    counter_bits = _counter_bits;
    zero_change  = _zero_change;
    one_change   = _one_change;
    thresh       = _thresh;
    init_value   = _init_value;

    max_index = (1 << index_bits) - 1;
    max_value = (1 << counter_bits) - 1;

    table = new unsigned[max_index + 1];

    //  Initialize with the right parameters & clear the counter
    for (int i = 0; i <= max_index; ++i)
        table[i] = init_value;
}

void SaturatingCounterPred::regStats()
{
    using namespace Stats;
    stringstream name, description;

    //
    //  Number of predictions
    //
    name << pred_name << ":" << zero_name << ":preds";
    description << "number of predictions of " << zero_name;
    predicted_zero
        .name(name.str())
        .desc(description.str())
        ;
    description.str("");
    name.str("");

    name << pred_name << ":" << one_name << ":preds";
    description << "number of predictions of " << one_name;
    predicted_one
        .name(name.str())
        .desc(description.str())
        ;
    description.str("");
    name.str("");

    //
    //  Count the number of correct predictions
    //
    name << pred_name << ":" << zero_name << ":corr_preds";
    description << "number of correct " << zero_name << " preds";
    correct_pred_zero
        .name(name.str())
        .desc(description.str())
        ;
    description.str("");
    name.str("");

    name << pred_name << ":" << one_name << ":corr_preds";
    description << "number of correct " << one_name << " preds";
    correct_pred_one
        .name(name.str())
        .desc(description.str())
        ;
    description.str("");
    name.str("");

    //
    //  Number of predictor updates
    //
    name << pred_name << ":" << zero_name << ":updates";
    description << "number of actual " << zero_name << "s";
    record_zero
        .name(name.str())
        .desc(description.str())
        ;
    description.str("");
    name.str("");

    name << pred_name << ":" << one_name << ":updates";
    description << "number of actual " << one_name << "s";
    record_one
        .name(name.str())
        .desc(description.str())
        ;
    description.str("");
    name.str("");
}

void SaturatingCounterPred::regFormulas()
{
    using namespace Stats;
    stringstream name, description;

    //
    //  Number of predictions
    //
    name << pred_name << ":predictions";
    preds_total
        .name(name.str())
        .desc("total number of predictions made")
        ;
    preds_total = predicted_zero + predicted_one;
    name.str("");

    //
    //  Fraction of all predictions that are one or zero
    //
    name << pred_name << ":" << zero_name << ":pred_frac";
    description << "fraction of all preds that were " << zero_name;
    pred_frac_zero
        .name(name.str())
        .desc(description.str())
        ;
    pred_frac_zero = 100 * predicted_zero / preds_total;
    description.str("");
    name.str("");

    name << pred_name << ":" << one_name << ":pred_frac";
    description << "fraction of all preds that were " << one_name;
    pred_frac_one
        .name(name.str())
        .desc(description.str())
        ;
    pred_frac_one = 100 * predicted_one / preds_total;
    description.str("");
    name.str("");


    //
    //  Count the number of correct predictions
    //
    name << pred_name << ":correct_preds";
    correct_total
        .name(name.str())
        .desc("total correct predictions made")
        ;
    correct_total = correct_pred_one + correct_pred_zero;
    name.str("");

    //
    //  Number of predictor updates
    //
    name << pred_name << ":updates";
    updates_total
        .name(name.str())
        .desc("total number of updates")
        ;
    updates_total = record_zero + record_one;
    name.str("");

    //
    //  Prediction accuracy rates
    //
    name << pred_name << ":pred_rate";
    pred_rate
        .name(name.str())
        .desc("correct fraction of all preds")
        ;
    pred_rate = correct_total / updates_total;
    name.str("");

    name << pred_name << ":" << zero_name << ":pred_rate";
    description << "fraction of " << zero_name << " preds that were correct";
    frac_correct_zero
        .name(name.str())
        .desc(description.str())
        ;
    frac_correct_zero = 100 * correct_pred_zero /
        (correct_pred_zero + record_one - correct_pred_one);
    description.str("");
    name.str("");

    name << pred_name << ":" << one_name << ":pred_rate";
    description << "fraction of " << one_name << " preds that were correct";
    frac_correct_one
        .name(name.str())
        .desc(description.str())
        ;
    frac_correct_one = 100 * correct_pred_one /
        (correct_pred_one + record_zero - correct_pred_zero);
    description.str("");
    name.str("");

    //
    //  Coverage
    //
    name << pred_name << ":" << zero_name << ":coverage";
    description << "fraction of " << zero_name
                << "s that were predicted correctly";
    coverage_zero
        .name(name.str())
        .desc(description.str())
        ;
    coverage_zero = 100 * correct_pred_zero / record_zero;
    description.str("");
    name.str("");

    name << pred_name << ":" << one_name << ":coverage";
    description << "fraction of " << one_name
                << "s that were predicted correctly";
    coverage_one
        .name(name.str())
        .desc(description.str())
        ;
    coverage_one = 100 * correct_pred_one / record_one;
    description.str("");
    name.str("");
}










