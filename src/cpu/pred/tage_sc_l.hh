/*
 * Copyright (c) 2022-2023 The University of Edinburgh
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 * TAGE-SC-L branch predictor base class (devised by Andre Seznec)
 * It consits of a TAGE + a statistical corrector (SC) + a loop predictor (L)
 */

#ifndef __CPU_PRED_TAGE_SC_L_HH__
#define __CPU_PRED_TAGE_SC_L_HH__

#include "cpu/pred/ltage.hh"
#include "cpu/pred/statistical_corrector.hh"
#include "params/TAGE_SC_L.hh"
#include "params/TAGE_SC_L_LoopPredictor.hh"
#include "params/TAGE_SC_L_TAGE.hh"

namespace gem5
{

namespace branch_prediction
{

class TAGE_SC_L_TAGE : public TAGEBase
{
    const unsigned firstLongTagTable;
    const unsigned longTagsSize;
    const unsigned shortTagsSize;

    const unsigned logTagTableSize;

    const unsigned shortTagsTageFactor;
    const unsigned longTagsTageFactor;

    const bool truncatePathHist;

  public:
    struct BranchInfo : public TAGEBase::BranchInfo
    {
        bool lowConf;
        bool highConf;
        bool altConf;
        bool medConf;
        BranchInfo(TAGEBase &tage) : TAGEBase::BranchInfo(tage),
            lowConf(false), highConf(false), altConf(false), medConf(false)
        {}
        virtual ~BranchInfo()
        {}
    };

    virtual TAGEBase::BranchInfo *makeBranchInfo() override;

    TAGE_SC_L_TAGE(const TAGE_SC_L_TAGEParams &p)
      : TAGEBase(p),
        firstLongTagTable(p.firstLongTagTable),
        longTagsSize(p.longTagsSize),
        shortTagsSize(p.shortTagsSize),
        logTagTableSize(p.logTagTableSize),
        shortTagsTageFactor(p.shortTagsTageFactor),
        longTagsTageFactor(p.longTagsTageFactor),
        truncatePathHist(p.truncatePathHist)
    {}

    void calculateParameters() override;

    void buildTageTables() override;

    void calculateIndicesAndTags(
        ThreadID tid, Addr branch_pc, TAGEBase::BranchInfo* bi) override;

    unsigned getUseAltIdx(TAGEBase::BranchInfo* bi, Addr branch_pc) override;

    void updateHistories(
        ThreadID tid, Addr branch_pc, bool taken, TAGEBase::BranchInfo* b,
        bool speculative, const StaticInstPtr &inst,
        Addr target) override;

    int bindex(Addr pc_in) const override;
    int gindex(ThreadID tid, Addr pc, int bank) const override;
    virtual int gindex_ext(int index, int bank) const = 0;
    int F(int phist, int size, int bank) const override;

    virtual uint16_t gtag(ThreadID tid, Addr pc, int bank) const override = 0;

    void squash(ThreadID tid, bool taken, TAGEBase::BranchInfo *bi,
                Addr target) override;

    void updatePathAndGlobalHistory(
        ThreadHistory & tHist, int brtype, bool taken,
        Addr branch_pc, Addr target);

    void adjustAlloc(bool & alloc, bool taken, bool pred_taken) override;

    virtual void handleAllocAndUReset(bool alloc, bool taken,
        TAGEBase::BranchInfo* bi, int nrand) override = 0;

    void handleUReset() override;

    virtual void handleTAGEUpdate(
        Addr branch_pc, bool taken, TAGEBase::BranchInfo* bi) override = 0;

    int calcDep(TAGEBase::BranchInfo* bi);

    bool getBimodePred(Addr branch_pc,
                       TAGEBase::BranchInfo* tage_bi) const override;

    void extraAltCalc(TAGEBase::BranchInfo* bi) override;

};

class TAGE_SC_L_LoopPredictor : public LoopPredictor
{
  public:
    TAGE_SC_L_LoopPredictor(const TAGE_SC_L_LoopPredictorParams &p)
      : LoopPredictor(p)
    {}

    virtual bool calcConf(int index) const override;
    virtual bool optionalAgeInc() const override;
};

class TAGE_SC_L: public LTAGE
{
    StatisticalCorrector *statisticalCorrector;
  public:
    TAGE_SC_L(const TAGE_SC_LParams &params);

    bool predict(
        ThreadID tid, Addr branch_pc, bool cond_branch, void* &b) override;

    void update(ThreadID tid, Addr branch_addr, bool taken, void * &bpHistory,
                bool squashed, const StaticInstPtr & inst,
                Addr corrTarget) override;

  protected:

    struct TageSCLBranchInfo : public LTageBranchInfo
    {
        StatisticalCorrector::BranchInfo *scBranchInfo;

        TageSCLBranchInfo(TAGEBase &tage, StatisticalCorrector &sc,
                          LoopPredictor &lp)
          : LTageBranchInfo(tage, lp), scBranchInfo(sc.makeBranchInfo())
        {}

        virtual ~TageSCLBranchInfo()
        {
            delete scBranchInfo;
        }
    };

    // more provider types
    enum
    {
        SC = LAST_LTAGE_PROVIDER_TYPE + 1
    };

};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_TAGE_SC_L_HH__
