/*
 * Copyright (c) 2014 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2006-2009 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

#ifndef __BASE__CP_ANNOTATE_HH__
#define __BASE__CP_ANNOTATE_HH__

#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/AnnotateQ.hh"
#include "config/cp_annotate.hh"
#include "config/the_isa.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"

#if CP_ANNOTATE
#include "params/CPA.hh"
#endif

class System;
class ThreadContext;


#if !CP_ANNOTATE
class CPA
{
  public:
    enum flags {
        FL_NONE     = 0x00,
        FL_HW       = 0x01,
        FL_BAD      = 0x02,
        FL_QOPP     = 0x04,
        FL_WAIT     = 0x08,
        FL_LINK     = 0x10,
        FL_RESET    = 0x20
    };

    static CPA *cpa()                                        { return NULL; }
    static bool available()                                 { return false; }
    bool enabled()                                          { return false; }
    void swSmBegin(ThreadContext *tc)                             { return; }
    void swSmEnd(ThreadContext *tc)                               { return; }
    void swExplictBegin(ThreadContext *tc)                        { return; }
    void swAutoBegin(ThreadContext *tc, Addr next_pc)             { return; }
    void swEnd(ThreadContext *tc)                                 { return; }
    void swQ(ThreadContext *tc)                                   { return; }
    void swDq(ThreadContext *tc)                                  { return; }
    void swPq(ThreadContext *tc)                                  { return; }
    void swRq(ThreadContext *tc)                                  { return; }
    void swWf(ThreadContext *tc)                                  { return; }
    void swWe(ThreadContext *tc)                                  { return; }
    void swSq(ThreadContext *tc)                                  { return; }
    void swAq(ThreadContext *tc)                                  { return; }
    void swLink(ThreadContext *tc)                                { return; }
    void swIdentify(ThreadContext *tc)                            { return; }
    uint64_t swGetId(ThreadContext *tc)                         { return 0; }
    void swSyscallLink(ThreadContext *tc)                         { return; }
    void hwBegin(flags f, System *sys, uint64_t frame, std::string sm, 
                 std::string st)                                  { return; }
    void hwQ(flags f, System *sys, uint64_t frame, std::string sm, 
             std::string q, uint64_t qid, System *q_sys = NULL, 
             int32_t count = 1)                                   { return; }
    void hwDq(flags f, System *sys, uint64_t frame, std::string sm, 
              std::string q, uint64_t qid, System *q_sys = NULL, 
              int32_t count = 1)                                  { return; }
    void hwPq(flags f, System *sys, uint64_t frame, std::string sm, 
              std::string q, uint64_t qid, System *q_sys = NULL, 
              int32_t count = 1)                                  { return; }
    void hwRq(flags f, System *sys, uint64_t frame, std::string sm, 
              std::string q, uint64_t qid, System *q_sys = NULL, 
              int32_t count = 1)                                  { return; }
    void hwWf(flags f, System *sys, uint64_t frame, std::string sm, 
              std::string q, uint64_t qid, System *q_sys = NULL, 
              int32_t count = 1)                                  { return; }
    void hwWe(flags f, System *sys, uint64_t frame, std::string sm, 
              std::string q, uint64_t qid, System *q_sys = NULL, 
              int32_t count = 1)                                  { return; }
};
#else

/**
 * Provide a hash function for the CPI Id type
 */
namespace std {
template <>
struct hash<std::pair<std::string, uint64_t> >
{

    size_t
    operator()(const std::pair<std::string, uint64_t>& x) const
    {
        return hash<std::string>()(x.first);
    }

};
}

class CPA : SimObject
{
  public:
    typedef CPAParams Params;
    
    /** The known operations that are written to the annotation output file. */
    enum ops {
        OP_BEGIN           = 0x01,
        OP_WAIT_EMPTY      = 0x02,
        OP_WAIT_FULL       = 0x03,
        OP_QUEUE           = 0x04,
        OP_DEQUEUE         = 0x05,
        OP_SIZE_QUEUE      = 0x08,
        OP_PEEK            = 0x09,
        OP_LINK            = 0x0A,
        OP_IDENT           = 0x0B,
        OP_RESERVE         = 0x0C
    };

    /** Flags for the various options.*/
    enum flags {
        /* no flags */
        FL_NONE     = 0x00,
        /* operation was done on hardware */
        FL_HW       = 0x01,
        /* operation should cause a warning when encountered */
        FL_BAD      = 0x02,
        /* Queue like a stack, not a queue */
        FL_QOPP     = 0x04,
        /* Mark HW state as waiting for some non-resource constraint
         * (e.g. wait because SM only starts after 10 items are queued) */ 
        FL_WAIT     = 0x08,
        /* operation is linking to another state machine */
        FL_LINK     = 0x10,
        /* queue should be completely cleared/reset before executing this
         * operation */
        FL_RESET    = 0x20
    };



  protected:
    const Params *
        params() const
        {
            return dynamic_cast<const Params *>(_params);
        }

    /* struct that is written to the annotation output file */
    struct AnnotateData : public Serializable {

        Tick time;
        uint32_t data;
        uint32_t orig_data;
        uint16_t sm;
        uint16_t stq;
        uint8_t  op;
        uint8_t  flag;
        uint8_t  cpu;
        bool dump;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    typedef std::shared_ptr<AnnotateData> AnnDataPtr;

    /* header for the annotation file */
    struct AnnotateHeader {
        uint64_t version;
        uint64_t num_recs;
        uint64_t key_off;
        uint64_t idx_off;
        uint32_t key_len;
        uint32_t idx_len;
    };

    AnnotateHeader ah;

    std::vector<uint64_t> annotateIdx;

    // number of state machines encountered in the simulation 
    int numSm;
    // number of states encountered in the simulation 
    int numSmt;
    // number of states/queues for a given state machine/system respectively
    std::vector<int> numSt, numQ;
    // number of systems in the simulation
    int numSys;
    // number of queues in the state machine
    int numQs;
    // maximum connection id assigned so far
    uint64_t conId;

    // Convert state strings into state ids
    typedef std::unordered_map<std::string, int> SCache;
    typedef std::vector<SCache> StCache;

    // Convert sm and queue name,id into queue id
    typedef std::pair<std::string, uint64_t> Id;
    typedef std::unordered_map<Id, int> IdHCache;
    typedef std::vector<IdHCache> IdCache;

    // Hold mapping of sm and queues to output python
    typedef std::vector<std::pair<int, Id> > IdMap;

    // System pointer to name,id
    typedef std::map<System*, std::pair<std::string, int> > NameCache;

    // array of systems each of which is a stack of running sm
    typedef std::pair<int, uint64_t> StackId;
    typedef std::map<StackId, std::vector<int> > SmStack;

    // map of each context and if it's currently in explict state mode
    // states are not automatically updated until it leaves
    typedef std::map<StackId, bool> SwExpl;

    typedef std::map<int,int> IMap;
    // List of annotate records have not been written/completed yet
    typedef std::list<AnnDataPtr> AnnotateList;

    // Maintain link state information
    typedef std::map<int, int> LinkMap;

    // SC Links
    typedef std::unordered_map<Id, AnnDataPtr> ScHCache;
    typedef std::vector<ScHCache> ScCache;


    AnnotateList data;

    // vector indexed by queueid to find current number of elements and bytes
    std::vector<int> qSize;
    std::vector<int32_t> qBytes;


    // Turn state machine string into state machine id (small int)
    // Used for outputting key to convert id back into string
    SCache smtCache;
    // Turn state machine id, state name into state id (small int)
    StCache stCache;
    // turn system, queue, and queue identify into qid (small int)
    // turn system, state, and context into state machine id (small int)
    IdCache qCache, smCache;
    //Link state machines accross system calls
    ScCache scLinks;
    // System pointer to name,id
    NameCache nameCache;
    // Stack of state machines currently nested (should unwind correctly)
    SmStack smStack;
    // Map of currently outstanding links 
    LinkMap lnMap;
    // If the state machine is currently exculding automatic changes
    SwExpl swExpl;
    // Last state that a given state machine was in
    IMap lastState;
    // Hold mapping of sm and queues to output python
    IdMap smMap, qMap;
    // Items still in queue, used for sanity checking 
    std::vector<AnnotateList> qData;

    void doDq(System *sys, int flags, int cpu, int sm, std::string q, int qi,
            int count);
    void doQ(System *sys, int flags, int cpu, int sm, std::string q, int qi,
            int count);

    void doSwSmEnd(System *sys, int cpuid, std::string sm, uint64_t frame);

    // Turn a system id, state machine string, state machine id into a small int
    // for annotation output
    int 
    getSm(int sysi, std::string si, uint64_t id)
    {
        int smi;
        Id smid = Id(si, id);

        smi = smCache[sysi-1][smid];
        if (smi == 0) {
            smCache[sysi-1][smid] = smi = ++numSm;
            assert(smi < 65535);
            smMap.push_back(std::make_pair(sysi, smid));
        }
        return smi;
    }

    // Turn a state machine string, state string into a small int
    // for annotation output
    int 
    getSt(std::string sm, std::string s)
    {
        int sti, smi;

        smi = smtCache[sm];
        if (smi == 0)
           smi = smtCache[sm] = ++numSmt;

        while (stCache.size() < smi) {
            //stCache.resize(sm);
            stCache.push_back(SCache());
            numSt.push_back(0);
        }
        //assert(stCache.size() == sm);
        //assert(numSt.size() == sm);
        sti = stCache[smi-1][s];
        if (sti == 0)
            stCache[smi-1][s] = sti = ++numSt[smi-1];
        return sti;
    }

    // Turn state machine pointer into a smal int for annotation output
    int 
    getSys(System *s)
    {
        NameCache::iterator i = nameCache.find(s);
        if (i == nameCache.end()) {
            nameCache[s] = std::make_pair(s->name(), ++numSys);
            i = nameCache.find(s);
            // might need to put smstackid into map here, but perhaps not
            //smStack.push_back(std::vector<int>());
            //swExpl.push_back(false);
            numQ.push_back(0);
            qCache.push_back(IdHCache());
            smCache.push_back(IdHCache());
            scLinks.push_back(ScHCache());
        }
        return i->second.second;
    }

    // Turn queue name, and queue context into small int for 
    // annotation output
    int 
    getQ(int sys, std::string q, uint64_t id)
    {
        int qi;
        Id qid = Id(q, id);

        qi = qCache[sys-1][qid];
        if (qi == 0) {
            qi = qCache[sys-1][qid] = ++numQs;
            assert(qi < 65535);
            qSize.push_back(0);
            qBytes.push_back(0);
            qData.push_back(AnnotateList());
            numQ[sys-1]++;
            qMap.push_back(std::make_pair(sys, qid));
        }
        return qi;
    }

    void swBegin(System *sys, int cpuid, std::string st, uint64_t frame, 
            bool expl = false, int flags = FL_NONE);

    AnnDataPtr add(int t, int f, int c, int sm, int stq, int32_t data=0);

    std::ostream *osbin;

    bool _enabled;

    /** Only allow one CPA object in a system. It doesn't make sense to have 
     * more that one per simulation because if a part of the system was
     * important it would have annotations and queues, and with more than one
     * object none of the sanity checking for queues will work. */
    static bool exists;
    static CPA *_cpa;


    std::map<std::string, SymbolTable*> userApp;

  public:
    static CPA *cpa() { return _cpa; }
    void swSmBegin(ThreadContext *tc);
    void swSmEnd(ThreadContext *tc);
    void swExplictBegin(ThreadContext *tc);
    void swAutoBegin(ThreadContext *tc, Addr next_pc);
    void swEnd(ThreadContext *tc);
    void swQ(ThreadContext *tc);
    void swDq(ThreadContext *tc);
    void swPq(ThreadContext *tc);
    void swRq(ThreadContext *tc);
    void swWf(ThreadContext *tc);
    void swWe(ThreadContext *tc);
    void swSq(ThreadContext *tc);
    void swAq(ThreadContext *tc);
    void swLink(ThreadContext *tc);
    void swIdentify(ThreadContext *tc);
    uint64_t swGetId(ThreadContext *tc);
    void swSyscallLink(ThreadContext *tc);

    inline void hwBegin(flags f, System *sys, uint64_t frame, std::string sm, 
            std::string st)
    {
        if (!enabled())
            return;

        int sysi = getSys(sys);
        int smi = getSm(sysi, sm, frame);
        add(OP_BEGIN, FL_HW | f, 0, smi, getSt(sm, st));
        if (f & FL_BAD)
            warn("BAD state encountered: at cycle %d: %s\n", curTick(), st);
    }

    inline void hwQ(flags f, System *sys, uint64_t frame, std::string sm, 
            std::string q, uint64_t qid, System *q_sys = NULL, int32_t count = 1)
    {
        if (!enabled())
            return;

        int sysi = getSys(sys);
        int qi = getQ(q_sys ?  getSys(q_sys) : sysi, q, qid);
        DPRINTFS(AnnotateQ, sys, 
                "hwQ: %s[%#x] cur size %d %d bytes: %d adding: %d\n",
                q, qid, qSize[qi-1], qData[qi-1].size(), qBytes[qi-1], count);
        doQ(sys, FL_HW | f, 0, getSm(sysi, sm, frame), q, qi, count);

    }

    inline void hwDq(flags f, System *sys, uint64_t frame, std::string sm, 
            std::string q, uint64_t qid, System *q_sys = NULL, int32_t count = 1)
    {
        if (!enabled())
            return;

        int sysi = getSys(sys);
        int qi = getQ(q_sys ?  getSys(q_sys) : sysi, q, qid);
        DPRINTFS(AnnotateQ, sys, 
                "hwDQ: %s[%#x] cur size %d %d bytes: %d removing: %d\n",
                q, qid, qSize[qi-1], qData[qi-1].size(), qBytes[qi-1], count);
        doDq(sys, FL_HW | f, 0, getSm(sysi,sm, frame), q, qi, count);
    }

    inline void hwPq(flags f, System *sys, uint64_t frame, std::string sm, 
            std::string q, uint64_t qid, System *q_sys = NULL, int32_t count = 1)
    {
        if (!enabled())
            return;

        int sysi = getSys(sys);
        int qi = getQ(q_sys ?  getSys(q_sys) : sysi, q, qid);
        DPRINTFS(AnnotateQ, sys, 
                "hwPQ: %s[%#x] cur size %d %d bytes: %d peeking: %d\n",
                q, qid, qSize[qi-1], qData[qi-1].size(), qBytes[qi-1], count);
        add(OP_PEEK, FL_HW | f, 0, getSm(sysi, sm, frame), qi, count);
    }

    inline void hwRq(flags f, System *sys, uint64_t frame, std::string sm, 
            std::string q, uint64_t qid, System *q_sys = NULL, int32_t count = 1)
    {
        if (!enabled())
            return;

        int sysi = getSys(sys);
        int qi = getQ(q_sys ?  getSys(q_sys) : sysi, q, qid);
        DPRINTFS(AnnotateQ, sys, 
                "hwRQ: %s[%#x] cur size %d %d bytes: %d reserving: %d\n",
                q, qid, qSize[qi-1], qData[qi-1].size(), qBytes[qi-1], count);
        add(OP_RESERVE, FL_HW | f, 0, getSm(sysi, sm, frame), qi, count);
    }

    inline void hwWf(flags f, System *sys, uint64_t frame, std::string sm, 
            std::string q, uint64_t qid, System *q_sys = NULL, int32_t count = 1)
    {
        if (!enabled())
            return;

        int sysi = getSys(sys);
        int qi = getQ(q_sys ?  getSys(q_sys) : sysi, q, qid);
        add(OP_WAIT_FULL, FL_HW | f, 0, getSm(sysi, sm, frame), qi, count);
    }

    inline void hwWe(flags f, System *sys, uint64_t frame, std::string sm, 
            std::string q, uint64_t qid, System *q_sys = NULL, int32_t count = 1) 
    {
        if (!enabled())
            return;

        int sysi = getSys(sys);
        int qi = getQ(q_sys ?  getSys(q_sys) : sysi, q, qid);
        add(OP_WAIT_EMPTY, FL_HW | f, 0, getSm(sysi, sm, frame), qi, count);
    }

  public:
    CPA(Params *p);
    void startup();

    uint64_t getFrame(ThreadContext *tc);

    static bool available()  { return true; }

    bool 
    enabled() 
    {   
        if (!this)
            return false;
        return _enabled;
    }
        
    void dump(bool all);
    void dumpKey();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};
#endif // !CP_ANNOTATE

#endif //__BASE__CP_ANNOTATE_HH__

