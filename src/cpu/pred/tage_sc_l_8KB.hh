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
 * 8KB TAGE-SC-L branch predictor (devised by Andre Seznec)
 */

#ifndef __CPU_PRED_TAGE_SC_L_8KB_HH__
#define __CPU_PRED_TAGE_SC_L_8KB_HH__

#include "cpu/pred/tage_sc_l.hh"
#include "params/TAGE_SC_L_8KB.hh"
#include "params/TAGE_SC_L_8KB_StatisticalCorrector.hh"
#include "params/TAGE_SC_L_TAGE_8KB.hh"

namespace gem5
{

namespace branch_prediction
{

class TAGE_SC_L_TAGE_8KB : public TAGE_SC_L_TAGE
{
  public:
    TAGE_SC_L_TAGE_8KB(const TAGE_SC_L_TAGE_8KBParams &p) : TAGE_SC_L_TAGE(p)
    {}

    void initFoldedHistories(ThreadHistory &history) override;
    int gindex_ext(int index, int bank) const override;

    uint16_t gtag(ThreadID tid, Addr pc, int bank) const override;

    void handleAllocAndUReset(bool alloc, bool taken, TAGEBase::BranchInfo *bi,
                              int nrand) override;

    void handleTAGEUpdate(Addr branch_pc, bool taken,
                          TAGEBase::BranchInfo *bi) override;

    void resetUctr(uint8_t &u) override;
};

class TAGE_SC_L_8KB_StatisticalCorrector : public StatisticalCorrector
{
    // global branch history GEHL
    const unsigned gnb;
    const unsigned logGnb;
    std::vector<int> gm;
    std::vector<int8_t> *ggehl;
    std::vector<int8_t> wg;

    struct SC_8KB_ThreadHistory : public SCThreadHistory
    {
        SC_8KB_ThreadHistory() { globalHist = 0; }

        int64_t globalHist; // global history
    };

    SCThreadHistory *makeThreadHistory() override;

  public:
    TAGE_SC_L_8KB_StatisticalCorrector(
        const TAGE_SC_L_8KB_StatisticalCorrectorParams &p);

    unsigned getIndBiasBank(Addr branch_pc, BranchInfo *bi, int hitBank,
                            int altBank) const override;

    int gPredictions(ThreadID tid, Addr branch_pc, BranchInfo *bi, int &lsum,
                     int64_t phist) override;

    int gIndexLogsSubstr(int nbr, int i) override;

    void scHistoryUpdate(Addr branch_pc, const StaticInstPtr &inst, bool taken,
                         BranchInfo *tage_bi, Addr corrTarget) override;

    void gUpdates(ThreadID tid, Addr pc, bool taken, BranchInfo *bi,
                  int64_t phist) override;
};

class TAGE_SC_L_8KB : public TAGE_SC_L
{
  public:
    TAGE_SC_L_8KB(const TAGE_SC_L_8KBParams &params);
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_TAGE_SC_L_8KB_HH__
