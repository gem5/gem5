/*
 * Copyright 2019 Texas A&M University
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Daniel A. Jiménez
 *  Adapted to gem5 by: Javier Bueno Hedo
 *
 */

/*
 * Multiperspective Perceptron Predictor with TAGE (by Daniel A. Jiménez)
 */

#ifndef __CPU_PRED_MULTIPERSPECTIVE_PERCEPTRON_TAGE_HH__
#define __CPU_PRED_MULTIPERSPECTIVE_PERCEPTRON_TAGE_HH__

#include "cpu/pred/loop_predictor.hh"
#include "cpu/pred/multiperspective_perceptron.hh"
#include "cpu/pred/statistical_corrector.hh"
#include "cpu/pred/tage_base.hh"
#include "params/MPP_LoopPredictor.hh"
#include "params/MPP_StatisticalCorrector.hh"
#include "params/MPP_TAGE.hh"
#include "params/MultiperspectivePerceptronTAGE.hh"

namespace gem5
{

namespace branch_prediction
{

class MPP_TAGE : public TAGEBase
{
    std::vector<unsigned int> tunedHistoryLengths;
  public:
    struct BranchInfo : public TAGEBase::BranchInfo
    {
        BranchInfo(TAGEBase &tage) : TAGEBase::BranchInfo(tage)
        {}
        virtual ~BranchInfo()
        {}
    };

    MPP_TAGE(const MPP_TAGEParams &p) : TAGEBase(p),
        tunedHistoryLengths(p.tunedHistoryLengths)
    {}

    void calculateParameters() override;
    void handleTAGEUpdate(Addr branch_pc, bool taken, TAGEBase::BranchInfo* bi)
        override;
    void handleAllocAndUReset(bool alloc, bool taken, TAGEBase::BranchInfo* bi,
                              int nrand) override;
    void handleUReset() override;
    void resetUctr(uint8_t &u) override;
    int bindex(Addr pc_in) const override;
    bool isHighConfidence(TAGEBase::BranchInfo *bi) const override;

    unsigned getUseAltIdx(TAGEBase::BranchInfo* bi, Addr branch_pc) override;
    void adjustAlloc(bool & alloc, bool taken, bool pred_taken) override;
    void updateHistories(ThreadID tid, Addr branch_pc, bool taken,
                         TAGEBase::BranchInfo* b, bool speculative,
                         const StaticInstPtr &inst, Addr target) override;

    void updatePathAndGlobalHistory(ThreadHistory& tHist, int brtype,
                                    bool taken, Addr branch_pc, Addr target);
};

class MPP_LoopPredictor : public LoopPredictor
{
  public:
    MPP_LoopPredictor(const MPP_LoopPredictorParams &p) : LoopPredictor(p)
    {}

    bool calcConf(int index) const override;
    bool optionalAgeInc() const override;
};

class MPP_StatisticalCorrector : public StatisticalCorrector
{
  protected:
    int8_t thirdH;
    // global branch history variation GEHL
    const unsigned pnb;
    const unsigned logPnb;
    std::vector<int> pm;
    std::vector<int8_t> * pgehl;
    std::vector<int8_t> wp;

    // global branch history GEHL
    const unsigned gnb;
    const unsigned logGnb;
    std::vector<int> gm;
    std::vector<int8_t> * ggehl;
    std::vector<int8_t> wg;

    struct MPP_SCThreadHistory : public StatisticalCorrector::SCThreadHistory
    {
        MPP_SCThreadHistory() : globalHist(0), historyStack(16, 0),
            historyStackPointer(0) {}
        int64_t globalHist; // global history
        std::vector<int64_t> historyStack;
        unsigned int historyStackPointer;

        int64_t getHistoryStackEntry() const
        {
            return historyStack[historyStackPointer];
        }

        void updateHistoryStack(Addr target, bool taken, bool is_call,
                                bool is_return)
        {
            unsigned int truncated_target = target;
            historyStack[historyStackPointer] =
                (historyStack[historyStackPointer] << 1) ^ (truncated_target ^
                (truncated_target >> 5) ^ taken);
            if (is_return) {
                historyStackPointer = (historyStackPointer - 1) %
                    historyStack.size();
            }
            if (is_call) {
                int index = (historyStackPointer + 1) % historyStack.size();
                historyStack[index] = historyStack[historyStackPointer];
                historyStackPointer = index;
            }
        }
        unsigned int getPointer() const { return historyStackPointer; }
    };

  public:
    struct BranchInfo : public StatisticalCorrector::BranchInfo
    {
        virtual ~BranchInfo()
        {}
    };
    MPP_StatisticalCorrector(const MPP_StatisticalCorrectorParams &p);

    void initBias() override;
    unsigned getIndBias(Addr branch_pc, StatisticalCorrector::BranchInfo* bi,
                        bool bias) const override;
    unsigned getIndBiasSK(Addr branch_pc, StatisticalCorrector::BranchInfo* bi)
            const override;
    unsigned getIndBiasBank(Addr branch_pc,
                            StatisticalCorrector::BranchInfo* bi, int hitBank,
                            int altBank) const override;
    unsigned getIndUpd(Addr branch_pc) const override;
    int gIndexLogsSubstr(int nbr, int i) override;

    bool scPredict(ThreadID tid, Addr branch_pc, bool cond_branch,
                   StatisticalCorrector::BranchInfo* bi, bool prev_pred_taken,
                   bool bias_bit, bool use_conf_ctr, int8_t conf_ctr,
                   unsigned conf_bits, int hitBank, int altBank, int64_t phist,
                   int init_lsum) override;

    void condBranchUpdate(ThreadID tid, Addr branch_pc, bool taken,
                          StatisticalCorrector::BranchInfo *bi,
                          Addr corrTarget, bool b, int hitBank, int altBank,
                          int64_t phist) override;

    virtual void getBiasLSUM(Addr branch_pc,
            StatisticalCorrector::BranchInfo *bi, int &lsum) const = 0;

    void gUpdate(
        Addr branch_pc, bool taken, int64_t hist, std::vector<int> & length,
        std::vector<int8_t> * tab, int nbr, int logs,
        std::vector<int8_t> &w, StatisticalCorrector::BranchInfo* bi) override;
};

class MultiperspectivePerceptronTAGE : public MultiperspectivePerceptron
{
    TAGEBase *tage;
    LoopPredictor *loopPredictor;
    StatisticalCorrector *statisticalCorrector;

    /**
     * Branch information data type
     */
    struct MPPTAGEBranchInfo : public MPPBranchInfo
    {
        TAGEBase::BranchInfo *tageBranchInfo;
        LoopPredictor::BranchInfo *lpBranchInfo;
        StatisticalCorrector::BranchInfo *scBranchInfo;
        bool predictedTaken;
        MPPTAGEBranchInfo(Addr pc, int pcshift, bool cond, TAGEBase &tage,
                          LoopPredictor &loopPredictor,
                          StatisticalCorrector &statisticalCorrector)
          : MPPBranchInfo(pc, pcshift, cond),
            tageBranchInfo(tage.makeBranchInfo()),
            lpBranchInfo(loopPredictor.makeBranchInfo()),
            scBranchInfo(statisticalCorrector.makeBranchInfo()),
            predictedTaken(false)
        {}
        virtual ~MPPTAGEBranchInfo()
        {
            delete tageBranchInfo;
            delete lpBranchInfo;
            delete scBranchInfo;
        }
    };

    unsigned int getIndex(ThreadID tid, MPPTAGEBranchInfo &bi,
                          const HistorySpec &spec, int index) const;
    int computePartialSum(ThreadID tid, MPPTAGEBranchInfo &bi) const;
    void updatePartial(ThreadID tid, MPPTAGEBranchInfo &bi, bool taken);
    void updateHistories(ThreadID tid, MPPTAGEBranchInfo &bi, bool taken);

  public:
    MultiperspectivePerceptronTAGE(
        const MultiperspectivePerceptronTAGEParams &p);

    void init() override;

    bool lookup(ThreadID tid, Addr instPC, void * &bp_history) override;

    void update(ThreadID tid, Addr instPC, bool taken,
            void *bp_history, bool squashed,
            const StaticInstPtr & inst,
            Addr corrTarget) override;
    void uncondBranch(ThreadID tid, Addr pc, void * &bp_history) override;
    void squash(ThreadID tid, void *bp_history) override;

};

} // namespace branch_prediction
} // namespace gem5

#endif//__CPU_PRED_MULTIPERSPECTIVE_PERCEPTRON_TAGE_HH__
