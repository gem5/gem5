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

//==========================================================================
//
//  This predictor takes the AND of a "local" and a "global" predictor
//  in order to determine its prediction.
//
//
//
//

#ifndef __HYBRID_PRED_HH__
#define __HYBRID_PRED_HH__

#include <string>

#include "base/sat_counter.hh"
#include "base/statistics.hh"

class HybridPredictor : public GenericPredictor
{
  private:
    std::string pred_name;
    std::string one_name;
    std::string zero_name;
    bool reg_individual_stats;

    SaturatingCounterPred *local;
    SaturatingCounterPred *global;

    unsigned long max_index;

    //
    //  Stats
    //
    Statistics::Scalar<> pred_one; //num_one_preds
    Statistics::Scalar<> pred_zero; //num_zero_preds
    Statistics::Scalar<> correct_pred_one; //num_one_correct
    Statistics::Scalar<> correct_pred_zero; //num_zero_correct
    Statistics::Scalar<> record_one; //num_one_updates
    Statistics::Scalar<> record_zero; //num_zero_updates

    Statistics::Formula total_preds;
    Statistics::Formula frac_preds_zero;
    Statistics::Formula frac_preds_one;
    Statistics::Formula total_correct;
    Statistics::Formula total_accuracy;
    Statistics::Formula zero_accuracy;
    Statistics::Formula one_accuracy;
    Statistics::Formula zero_coverage;
    Statistics::Formula one_coverage;

  public:
    HybridPredictor(const char *_p_name, const char *_z_name,
                    const char *_o_name,
                    unsigned _index_bits, unsigned _counter_bits,
                    unsigned _zero_change, unsigned _one_change,
                    unsigned _thresh,
                    unsigned _global_bits, unsigned _global_thresh,
                    bool _reg_individual_stats = false);

    void clear() {
        global->clear();
        local->clear();
    }

    unsigned peek(unsigned long _index) {
        unsigned l = local->peek(_index);
        unsigned g = global->peek(_index);

        if (l && g)
            return 1;

        return 0;
    }

    unsigned value(unsigned long _index) {
        unsigned l = local->peek(_index);
        unsigned g = global->peek(_index);

        l = l & 0xFFFF;
        g = g & 0xFFFF;

        return  (l << 16) | g;

    }

    unsigned predict(unsigned long _index) {
        unsigned l = local->predict(_index);
        unsigned g = global->predict(_index);

        if (l && g) {
            ++pred_one;
            return 1;
        }

        ++pred_zero;
        return 0;
    }


    //
    //  This version need only be used if local/global statistics
    //  will be maintained
    //
    unsigned predict(unsigned long _index, unsigned &_pdata) {
        unsigned l = local->predict(_index);
        unsigned g = global->predict(_index);

        //
        //  bit 0 => local predictor result
        //  bit 1 => global predictor result
        //
        _pdata = 0;
        if (l)
            _pdata |= 1;
        if (g)
            _pdata |= 2;
        if (l && g) {
            ++pred_one;
            return 1;
        }

        ++pred_zero;
        return 0;
    }

    void record(unsigned long _index, unsigned _val, unsigned _predicted) {

        if (_val) {
            local->record(_index, _val, 0);
            global->record(_index, _val, 0);
            ++record_one;

            if (_val == _predicted) {
                ++correct_pred_one;
            }
        } else {
            local->record(_index, _val, 0);
            global->record(_index, _val, 0);
            ++record_zero;

            if (_val == _predicted)
                ++correct_pred_zero;
        }
    }

    void record(unsigned long _index, unsigned _val, unsigned _predicted,
                unsigned _pdata)
    {

        local->record(_index, _val, (_pdata & 1));
        global->record(_index, _val, ((_pdata & 2) ? 1 : 0));


        if (_val) {
            ++record_one;

            if (_val == _predicted)
                ++correct_pred_one;
        } else {
            ++record_zero;

            if (_val == _predicted)
                ++correct_pred_zero;
        }
    }

    void regStats();
    void regFormulas();
};


#endif  // _HYBRID_PRED_HH__

