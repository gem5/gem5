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
 * 8 KB version
 */

#ifndef __CPU_PRED_MULTIPERSPECTIVE_PERCEPTRON_TAGE_8KB_HH__
#define __CPU_PRED_MULTIPERSPECTIVE_PERCEPTRON_TAGE_8KB_HH__

#include "cpu/pred/multiperspective_perceptron_tage.hh"
#include "params/MPP_LoopPredictor_8KB.hh"
#include "params/MPP_StatisticalCorrector_8KB.hh"
#include "params/MPP_TAGE_8KB.hh"
#include "params/MultiperspectivePerceptronTAGE8KB.hh"

namespace gem5
{

namespace branch_prediction
{

class MPP_TAGE_8KB : public MPP_TAGE
{
  public:
    MPP_TAGE_8KB(const MPP_TAGE_8KBParams &p) : MPP_TAGE(p) {}
};

class MPP_LoopPredictor_8KB : public MPP_LoopPredictor
{
  public:
    MPP_LoopPredictor_8KB(const MPP_LoopPredictor_8KBParams &p) :
        MPP_LoopPredictor(p) {}
};

class MPP_StatisticalCorrector_8KB : public MPP_StatisticalCorrector
{
    StatisticalCorrector::SCThreadHistory *makeThreadHistory() override;
    int gPredictions(ThreadID tid, Addr branch_pc,
            StatisticalCorrector::BranchInfo* bi, int &lsum, int64_t phist)
            override;
    void getBiasLSUM(Addr branch_pc,
            StatisticalCorrector::BranchInfo *bi, int &lsum) const override;
    void gUpdates(ThreadID tid, Addr pc, bool taken,
            StatisticalCorrector::BranchInfo* bi, int64_t phist) override;
    void scHistoryUpdate(Addr branch_pc, const StaticInstPtr &inst, bool taken,
            StatisticalCorrector::BranchInfo *bi, Addr corrTarget) override;
  public:
    MPP_StatisticalCorrector_8KB(const MPP_StatisticalCorrector_8KBParams &p);
    size_t getSizeInBits() const override;
};

class MultiperspectivePerceptronTAGE8KB :
        public MultiperspectivePerceptronTAGE
{
    void createSpecs() override;
  public:
    MultiperspectivePerceptronTAGE8KB(
            const MultiperspectivePerceptronTAGE8KBParams &p);
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_MULTIPERSPECTIVE_PERCEPTRON_TAGE_8KB_HH__
