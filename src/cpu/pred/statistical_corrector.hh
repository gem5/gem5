/*
 * Copyright (c) 2018 Metempsy Technology Consulting
 * All rights reserved.
 *
 * Copyright (c) 2006 INRIA (Institut National de Recherche en
 * Informatique et en Automatique  / French National Research Institute
 * for Computer Science and Applied Mathematics)
 *
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
 * Author: André Seznec, Pau Cabre, Javier Bueno
 *
 */

/*
 * Statistical corrector base class
 */

#ifndef __CPU_PRED_STATISTICAL_CORRECTOR_HH
#define __CPU_PRED_STATISTICAL_CORRECTOR_HH

#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "sim/sim_object.hh"

struct StatisticalCorrectorParams;

class StatisticalCorrector : public SimObject
{
  protected:
    template<typename T>
    inline void ctrUpdate(T & ctr, bool taken, int nbits) {
        assert(nbits <= sizeof(T) << 3);
        if (nbits > 0) {
            if (taken) {
                if (ctr < ((1 << (nbits - 1)) - 1))
                    ctr++;
            } else {
                if (ctr > -(1 << (nbits - 1)))
                    ctr--;
            }
        }
    }
    // histories used for the statistical corrector
    struct SCThreadHistory {
        SCThreadHistory() {
            bwHist = 0;
            numOrdinalHistories = 0;
            imliCount = 0;
        }
        int64_t bwHist;  // backward global history
        int64_t imliCount;

        void setNumOrdinalHistories(unsigned num)
        {
            numOrdinalHistories = num;
            assert(num > 0);
            shifts.resize(num);
            localHistories = new std::vector<int64_t> [num];
        }

        void initLocalHistory(int ordinal, int numHistories, int shift)
        {
            assert((ordinal >= 1) && (ordinal <= numOrdinalHistories));
            shifts[ordinal - 1] = shift;
            assert(isPowerOf2(numHistories));
            localHistories[ordinal - 1].resize(numHistories, 0);
        }

        int64_t getLocalHistory(int ordinal, Addr pc)
        {
            assert((ordinal >= 1) && (ordinal <= numOrdinalHistories));
            unsigned idx = ordinal - 1;
            return localHistories[idx][getEntry(pc, idx)];
        }

        void updateLocalHistory(
            int ordinal, Addr branch_pc, bool taken, Addr extraXor = 0)
        {
            assert((ordinal >= 1) && (ordinal <= numOrdinalHistories));
            unsigned idx = ordinal - 1;

            unsigned entry = getEntry(branch_pc, idx);
            int64_t hist =  (localHistories[idx][entry] << 1) + taken;

            if (extraXor) {
                hist = hist ^ extraXor;
            }

            localHistories[idx][entry] = hist;
        }

      private:
        std::vector<int64_t> * localHistories;
        std::vector<int> shifts;
        unsigned numOrdinalHistories;

        unsigned getEntry(Addr pc, unsigned idx)
        {
            return (pc ^ (pc >> shifts[idx])) & (localHistories[idx].size()-1);
        }
    };

    // For SC history we use global (i.e. not per thread) non speculative
    // histories, as some of the strucures needed are quite big and it is not
    // reasonable to make them per thread and it would be difficult to
    // rollback on miss-predictions
    SCThreadHistory * scHistory;

    const unsigned logBias;

    const unsigned logSizeUp;
    const unsigned logSizeUps;

    const unsigned numEntriesFirstLocalHistories;

    // global backward branch history GEHL
    const unsigned bwnb;
    const unsigned logBwnb;
    std::vector<int> bwm;
    std::vector<int8_t> * bwgehl;
    std::vector<int8_t> wbw;

    // First local history GEHL
    const unsigned lnb;
    const unsigned logLnb;
    std::vector<int> lm;
    std::vector<int8_t> * lgehl;
    std::vector<int8_t> wl;

    // IMLI GEHL
    const unsigned inb;
    const unsigned logInb;
    std::vector<int> im;
    std::vector<int8_t> * igehl;
    std::vector<int8_t> wi;

    std::vector<int8_t> bias;
    std::vector<int8_t> biasSK;
    std::vector<int8_t> biasBank;

    std::vector<int8_t> wb;

    int updateThreshold;
    std::vector<int> pUpdateThreshold;

    // The two counters used to choose between TAGE ang SC on High Conf
    // TAGE/Low Conf SC
    const unsigned chooserConfWidth;

    const unsigned updateThresholdWidth;
    const unsigned pUpdateThresholdWidth;

    const unsigned extraWeightsWidth;

    const unsigned scCountersWidth;

    int8_t firstH;
    int8_t secondH;

    struct StatisticalCorrectorStats : public Stats::Group {
        StatisticalCorrectorStats(Stats::Group *parent);
        Stats::Scalar correct;
        Stats::Scalar wrong;
    } stats;

  public:
    struct BranchInfo
    {
        BranchInfo() : lowConf(false), highConf(false), altConf(false),
              medConf(false), scPred(false), lsum(0), thres(0),
              predBeforeSC(false), usedScPred(false)
        {}

        // confidences calculated on tage and used on the statistical
        // correction
        bool lowConf;
        bool highConf;
        bool altConf;
        bool medConf;

        bool scPred;
        int lsum;
        int thres;
        bool predBeforeSC;
        bool usedScPred;
    };

    StatisticalCorrector(const StatisticalCorrectorParams *p);

    virtual BranchInfo *makeBranchInfo();
    virtual SCThreadHistory *makeThreadHistory();

    virtual void initBias();

    virtual bool scPredict(
        ThreadID tid, Addr branch_pc, bool cond_branch, BranchInfo* bi,
        bool prev_pred_taken, bool bias_bit, bool use_conf_ctr,
        int8_t conf_ctr, unsigned conf_bits, int hitBank, int altBank,
        int64_t phist, int init_lsum = 0);

    virtual unsigned getIndBias(Addr branch_pc, BranchInfo* bi, bool b) const;

    virtual unsigned getIndBiasSK(Addr branch_pc, BranchInfo* bi) const;

    virtual unsigned getIndBiasBank( Addr branch_pc, BranchInfo* bi,
        int hitBank, int altBank) const = 0;

    virtual unsigned getIndUpd(Addr branch_pc) const;
    unsigned getIndUpds(Addr branch_pc) const;

    virtual int gPredictions(ThreadID tid, Addr branch_pc, BranchInfo* bi,
        int & lsum, int64_t phist) = 0;

    int64_t gIndex(Addr branch_pc, int64_t bhist, int logs, int nbr, int i);

    virtual int gIndexLogsSubstr(int nbr, int i) = 0;

    int gPredict(
        Addr branch_pc, int64_t hist, std::vector<int> & length,
        std::vector<int8_t> * tab, int nbr, int logs,
        std::vector<int8_t> & w);

    virtual void gUpdate(
        Addr branch_pc, bool taken, int64_t hist, std::vector<int> & length,
        std::vector<int8_t> * tab, int nbr, int logs,
        std::vector<int8_t> & w, BranchInfo* bi);

    void initGEHLTable(
        unsigned numLenghts, std::vector<int> lengths,
        std::vector<int8_t> * & table, unsigned logNumEntries,
        std::vector<int8_t> & w, int8_t wInitValue);

    virtual void scHistoryUpdate(
        Addr branch_pc, const StaticInstPtr &inst , bool taken,
        BranchInfo * tage_bi, Addr corrTarget);

    virtual void gUpdates( ThreadID tid, Addr pc, bool taken, BranchInfo* bi,
        int64_t phist) = 0;

    void init() override;
    void updateStats(bool taken, BranchInfo *bi);

    virtual void condBranchUpdate(ThreadID tid, Addr branch_pc, bool taken,
                          BranchInfo *bi, Addr corrTarget, bool bias_bit,
                          int hitBank, int altBank, int64_t phist);

    virtual size_t getSizeInBits() const;
};
#endif//__CPU_PRED_STATISTICAL_CORRECTOR_HH
