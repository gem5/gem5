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

#ifndef __SAT_COUNTER_HH__
#define __SAT_COUNTER_HH__

#include <string>

#include "base/predictor.hh"

#include "base/statistics.hh"
#include "sim/stats.hh"

//
//
//  A simple saturating counter predictor
//
//
class SaturatingCounterPred : public GenericPredictor
{
  private:
    std::string   pred_name;
    std::string   zero_name;
    std::string   one_name;

    unsigned index_bits;
    unsigned counter_bits;
    unsigned zero_change;
    unsigned one_change;
    unsigned thresh;
    unsigned init_value;

    unsigned max_value;       // maximum counter value

    unsigned long max_index;  // also the index mask value
    unsigned *table;

    //  Statistics
    Stats::Scalar<> predicted_one;      // Total predictions of one, preds_one
    Stats::Scalar<> predicted_zero;     // Total predictions of zero, preds_zero
    Stats::Scalar<> correct_pred_one;   // Total correct predictions of one, correct_one
    Stats::Scalar<> correct_pred_zero;  // Total correct predictions of zero, correct_zero

    Stats::Scalar<> record_zero;        //updates_zero
    Stats::Scalar<> record_one;         //updates_one

    Stats::Formula preds_total;
    Stats::Formula pred_frac_zero;
    Stats::Formula pred_frac_one;
    Stats::Formula correct_total;
    Stats::Formula updates_total;
    Stats::Formula pred_rate;
    Stats::Formula frac_correct_zero;
    Stats::Formula frac_correct_one;
    Stats::Formula coverage_zero;
    Stats::Formula coverage_one;

  private:
    bool pred_one(unsigned &counter)  { return counter >  thresh; }
    bool pred_zero(unsigned &counter) { return counter <= thresh; }

    void update_one(unsigned &counter) {

        if (one_change)
            counter += one_change;
        else
            counter = 0;

        // check for wrap
        if (counter > max_value)
            counter = max_value;
    }

    void update_zero(unsigned &counter) {
        if (zero_change) {
            // check for wrap
            if (counter < zero_change)
                counter = 0;
            else
                counter -= zero_change;
        } else
            counter = 0;
    }


  public:

    SaturatingCounterPred(std::string p_name,
                          std::string z_name, std::string o_name,
                          unsigned _index_bits, unsigned _counter_bits = 2,
                          unsigned _zero_change = 1, unsigned _one_change = 1,
                          unsigned _thresh = 1, unsigned _init_value = 0);

    void clear() {
        for (int i = 0; i <= max_index; ++i)
            table[i] = init_value;
    }

    //  Record the ACTUAL result... and indicate whether the prediction
    //  corresponding to this event was correct
    void record(unsigned long _index, unsigned _val, unsigned _predicted,
                unsigned _pdata)
    {
        record(_index, _val, _predicted);
    }

    void record(unsigned long _index, unsigned _val, unsigned _predicted) {
        unsigned long index = _index & max_index;

        if (_val) {
            update_one(table[index]);
            ++record_one;

            if (_predicted)
                ++correct_pred_one;
        } else {
            update_zero(table[index]);
            ++record_zero;

            if (!_predicted)
                ++correct_pred_zero;
        }
    }

    unsigned value(unsigned long _index) {
        unsigned long index = _index & max_index;

        return table[index];
    }


    unsigned predict(unsigned long _index, unsigned &pdata) {
        return predict(_index);
    }

    unsigned predict(unsigned long _index) {
        unsigned long index = _index & max_index;

        if (pred_one(table[index])) {
            ++predicted_one;
            return 1;
        }

        ++predicted_zero;
        return 0;
    }

    //  No internal state is changed here
    unsigned peek(unsigned long _index) {
        unsigned long index = _index & max_index;

        if (pred_one(table[index]))
            return 1;

        return 0;
    }


    //=======================================================
    void regStats();
    void regFormulas();
};


#endif // __SAT_COUNTER_HH__
