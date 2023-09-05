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
 * Multiperspective Perceptron Predictor (by Daniel A. Jiménez)
 */

#ifndef __CPU_PRED_MULTIPERSPECTIVE_PERCEPTRON_HH__
#define __CPU_PRED_MULTIPERSPECTIVE_PERCEPTRON_HH__

#include <array>
#include <vector>

#include "cpu/pred/bpred_unit.hh"
#include "params/MultiperspectivePerceptron.hh"

namespace gem5
{

namespace branch_prediction
{

class MultiperspectivePerceptron : public BPredUnit
{
  protected:
    /**
     * Branch information data
     */
    class MPPBranchInfo
    {
        /** pc of the branch */
        const unsigned int pc;
        /** pc of the branch, shifted 2 bits to the right */
        const unsigned short int pc2;
        /** pc of the branch, hashed */
        const unsigned short int hpc;
        /** Whether this is a conditional branch */
        const bool condBranch;

        /**
         * PC Hash functions
         */
        static inline unsigned int hash1(unsigned int a)
        {
            a = (a ^ 0xdeadbeef) + (a<<4);
            a = a ^ (a>>10);
            a = a + (a<<7);
            a = a ^ (a>>13);
            return a;
        }

        static inline unsigned int hash2(unsigned int key)
        {
            int c2 = 0x27d4eb2d; // a prime or an odd constant
            key = (key ^ 61) ^ (key >> 16);
            key = key + (key << 3);
            key = key ^ (key >> 4);
            key = key * c2;
            key = key ^ (key >> 15);
            return key;
        }

        static inline unsigned int hash(unsigned int key, unsigned int i)
        {
            return hash2(key) * i + hash1(key);
        }

        static inline unsigned int hashPC(unsigned int pc, int pcshift)
        {
            if (pcshift < 0) {
                return hash(pc, -pcshift);
            } else if (pcshift < 11) {
                unsigned int x = pc;
                x ^= (pc >> pcshift);
                return x;
            } else {
                return pc >> (pcshift-11);
            }
        }

      public:
        /** Whether this branch has been filtered by the prefetcher */
        bool filtered;
        /** Result of the prediction (true is taken) */
        bool prediction;
        /** Score of the perceptron */
        int yout;

        MPPBranchInfo(Addr _pc, int pcshift, bool cb) : pc((unsigned int)_pc),
        pc2(pc >> 2), hpc(hashPC(pc, pcshift)), condBranch(cb),
        filtered(false), prediction(false), yout(0)
        { }

        unsigned int getPC() const
        {
            return pc;
        }
        unsigned short int getPC2() const
        {
            return pc2;
        }
        unsigned short int getHPC() const
        {
            return hpc;
        }
        unsigned int getHashFilter(bool last_ghist_bit) const
        {
            return last_ghist_bit ^ hpc;
        }
        bool isUnconditional() const
        {
            return !condBranch;
        }
    };

    /**
     * Entry of the branch filter
     */
    struct FilterEntry
    {
        /** Has this branch been taken at least once? */
        bool seenTaken;
        /** Has this branch been not taken at least once? */
        bool seenUntaken;

        FilterEntry() : seenTaken(false), seenUntaken(false) {}

        /** Whether this branch has always been observed as not taken */
        bool alwaysNotTakenSoFar() const {
            return seenUntaken & !seenTaken;
        }
        /** Whether this branch has always been observed as taken */
        bool alwaysTakenSoFar() const {
            return seenTaken & !seenUntaken;
        }
        /** Whether this branch has been observed before */
        bool neverSeen() const {
            return !seenTaken && !seenUntaken;
        }
    };


    /**
     * Local history entries, each enty contains the history of directions
     * taken by a given branch.
     */
    class LocalHistories
    {
        /** The array of histories */
        std::vector<unsigned int> localHistories;
        /** Size in bits of each history entry */
        const int localHistoryLength;

        /** Index function given the pc of the branch */
        unsigned int index(Addr pc) const {
            return (pc >> 2) % localHistories.size();
        }
        public:
        LocalHistories(int nlocal_histories, int histo_len) :
            localHistories(nlocal_histories), localHistoryLength(histo_len) {}

        /** Obtains the local history entry of a given branch */
        unsigned int operator[](Addr pc) const
        {
            return localHistories[index(pc)];
        }

        /** Adds a history bit to the local history entry of a given branch */
        void update(Addr pc, bool value)
        {
            assert(localHistories.size() > 0);
            unsigned int &pos = localHistories[index(pc)];
            pos <<= 1;
            pos |= value;
            pos &= ((1<<localHistoryLength)-1);
        }

        /** Returns the number of bits of each local history entry */
        int getLocalHistoryLength() const
        {
            return localHistoryLength;
        }

        /** Size in bits required by all history entries */
        int getSize() const
        {
            return localHistoryLength * localHistories.size();
        }
    };

    /**
     * Base class to implement the predictor tables.
     */
    struct HistorySpec
    {
        /** First parameter */
        const int p1;
        /** Second parameter */
        const int p2;
        /** Third parameter */
        const int p3;
        /** Coefficient of the feature, models the accuracy of the feature */
        const double coeff;
        /** Pre-assigned size in bits assigned to this feature */
        const int size;
        /** Width of the table in bits  */
        const int width;
        /** Reference to the branch predictor class */
        MultiperspectivePerceptron &mpp;

        HistorySpec(int _p1, int _p2, int _p3, double _coeff, int _size,
                int _width, MultiperspectivePerceptron &_mpp) : p1(_p1),
        p2(_p2), p3(_p3), coeff(_coeff), size(_size), width(_width),
        mpp(_mpp)
        {}

        /**
         * Gets the hash to index the table, using the pc of the branch,
         * and the index of the table.
         * @param tid Thread ID of the branch
         * @param pc address of the branch
         * @param pc2 address of the branch shifted 2 bits to the right
         * @param t integer index of the table
         * @result resulting hash value that will be used to index the table
         */
        virtual unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t)
            const = 0;
        /**
         * Sets the size requirements of the table, used when initializing
         * to set the proper size of the tables
         */
        virtual void setBitRequirements() const {}
    };

    /** Predictor parameters */
    const int blockSize;
    const int pcshift;
    const int threshold;
    const int bias0;
    const int bias1;
    const int biasmostly0;
    const int biasmostly1;
    const int nbest;
    const int tunebits;
    const int hshift;
    const unsigned long long int imli_mask1;
    const unsigned long long int imli_mask4;
    const unsigned long long int recencypos_mask;
    const double fudge;
    const int n_sign_bits;
    const int pcbit;
    const int decay;
    const unsigned int record_mask;
    const bool hash_taken;
    const bool tuneonly;
    const int extra_rounds;
    const int speed;
    const int budgetbits;
    const bool speculative_update;

    /** Transfer function for 6-width tables */
    static int xlat[];
    /** Transfer function for 5-width tables */
    static int xlat4[];

    /** History data is kept for each thread */
    struct ThreadData
    {
        ThreadData(int num_filter, int n_local_histories,
            int local_history_length, int assoc,
            const std::vector<std::vector<int>> &blurrypath_bits,
            int path_length, int ghist_length, int block_size,
            const std::vector<std::vector<std::vector<bool>>> &acyclic_bits,
            const std::vector<int> &modhist_indices,
            const std::vector<int> &modhist_lengths,
            const std::vector<int> &modpath_indices,
            const std::vector<int> &modpath_lengths,
            const std::vector<int> &table_sizes, int n_sign_bits);

        std::vector<FilterEntry> filterTable;
        std::vector<std::vector<bool>> acyclic_histories;
        std::vector<std::vector<unsigned int>> acyclic2_histories;

        void updateAcyclic(bool hashed_taken, unsigned int hpc) {
            for (int i = 0; i < acyclic_histories.size(); i += 1) {
                if (acyclic_histories[i].size() > 0) {
                    acyclic_histories[i][hpc%(i+2)] = hashed_taken;
                    acyclic2_histories[i][hpc%(i+2)] = hpc;
                }
            }
        }

        std::vector<std::vector<unsigned int>> blurrypath_histories;
        std::vector<unsigned int> ghist_words;
        std::vector<std::vector<unsigned short int>> modpath_histories;
        std::vector<std::vector<bool>> mod_histories;
        std::vector<unsigned short int> path_history;
        std::vector<unsigned int> imli_counter;
        LocalHistories localHistories;
        std::vector<unsigned int short> recency_stack;

        void insertRecency(unsigned int pc, int assoc) {
            int i = 0;
            for (i = 0; i < assoc; i += 1) {
                if (recency_stack[i] == pc) {
                    break;
                }
            }
            if (i == assoc) {
                i = assoc-1;
                recency_stack[i] = pc;
            }
            int j;
            unsigned int b = recency_stack[i];
            for (j = i; j >= 1; j -= 1) {
                recency_stack[j] = recency_stack[j-1];
            }
            recency_stack[0] = b;
        }

        bool last_ghist_bit;
        int occupancy;

        std::vector<int> mpreds;
        std::vector<std::vector<short int>> tables;
        std::vector<std::vector<std::array<bool, 2>>> sign_bits;
    };
    std::vector<ThreadData *> threadData;

    /** Predictor tables */
    std::vector<HistorySpec *> specs;
    std::vector<int> table_sizes;

    /** runtime values and data used to count the size in bits */
    bool doing_local;
    bool doing_recency;
    int assoc;
    int ghist_length;
    int modghist_length;
    int path_length;
    int thresholdCounter;
    int theta;
    int extrabits;
    std::vector<int> imli_counter_bits;
    std::vector<int> modhist_indices;
    std::vector<int> modhist_lengths;
    std::vector<int> modpath_indices;
    std::vector<int> modpath_lengths;
    std::vector<std::vector<int>> blurrypath_bits;
    std::vector<std::vector<std::vector<bool>>> acyclic_bits;

    /** Auxiliary function for MODHIST and GHISTMODPATH features */
    void insertModhistSpec(int p1, int p2) {
        int j = insert(modhist_indices, p1);
        if (modhist_lengths.size() < (j + 1)) {
            modhist_lengths.resize(j + 1);
        }
        if (modhist_lengths[j] < p2 + 1) {
            modhist_lengths[j] = p2 + 1;
        }
        if (p2 >= modghist_length) {
            modghist_length = p2 + 1;
        }
    }

    /** Auxiliary function for MODPATH and GHISTMODPATH features */
    void insertModpathSpec(int p1, int p2) {
        int j = insert(modpath_indices, p1);
        if (modpath_lengths.size() < (j + 1)) {
            modpath_lengths.resize(j + 1);
        }
        if (modpath_lengths[j] < p2 + 1) {
            modpath_lengths[j] = p2 + 1;
        }
        if (p2 >= path_length) {
            path_length = p2 + 1;
        }
    }

    /** Auxiliary function used by insertModhistSpec and insertModpathSpec*/
    int insert(std::vector<int> &v, int x)
    {
        for (int i = 0; i < v.size(); i += 1) {
            if (v[i] == x) {
                return i;
            }
        }
        v.push_back(x);
        return v.size()-1;
    }

    /**
     * Computes the size in bits of the structures needed to keep track
     * of the history and the predictor tables and assigns the sizes of
     * those tables that did not had their size specified.
     * @param num_filter_entries number of entries of the filter
     * @param nlocal_histories number of local history entries
     * @param local_history_length size of each local history entry
     * @param ignore_path_size ignore the path length storage
     */
    void computeBits(int num_filter_entries, int nlocal_histories,
            int local_history_length, bool ignore_path_size);

    /**
     * Creates the tables of the predictor
     */
    virtual void createSpecs() = 0;

    /**
     * Get the position index of a predictor table
     * @param tid Thread ID of the branch
     * @param bi branch informaiton data
     * @param spec predictor table
     * @param index integer index of the predictor table
     * @result index to access the predictor table
     */
    unsigned int getIndex(ThreadID tid, const MPPBranchInfo &bi,
            const HistorySpec &spec, int index) const;
    /**
     * Finds the best subset of features to use in case of a low-confidence
     * branch, returns the result as an ordered vector of the indices to the
     * predictor tables
     * @param tid Thread ID of the branch
     * @param vector to write the ordered list of indices of the best tables
     */
    void findBest(ThreadID tid, std::vector<int> &best_preds) const;

    /**
     * Computes the output of the predictor for a given branch and the
     * resulting best value in case the prediction has low confidence
     * @param tid Thread ID of the branch
     * @param bi branch informaiton data
     * @return resulting sum for low-confidence branch
     */
    int computeOutput(ThreadID tid, MPPBranchInfo &bi);

    /**
     * Trains the branch predictor with the given branch and direction
     * @param tid Thread ID of the branch
     * @param bi branch informaiton data
     * @param taken whether the branch was taken
     */
    void train(ThreadID tid, MPPBranchInfo &bi, bool taken);

    /**
     * Auxiliary function to increase a table counter depending on the
     * direction of the branch
     * @param taken whether the branch was taken
     * @param sign current sign of the table
     * @param c current value of the table
     * @param max_weight maximum value of the counter
     */
    void satIncDec(bool taken, bool &sign, int &c, int max_weight) const;

    /** Add a table spec to the prefetcher */
    void addSpec(HistorySpec *spec)
    {
        specs.push_back(spec);
    }

    /** Available features */

    class GHIST : public HistorySpec
    {
      public:
        GHIST(int p1, int p2, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, p2, 0, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            return hash(mpp.threadData[tid]->ghist_words, mpp.blockSize, p1,
                        p2);
        }

        static unsigned int hash(const std::vector<unsigned int> &ghist_words,
                int block_size, int start_pos, int end_pos)
        {
            int a = start_pos;
            int b = end_pos;

            unsigned int x = 0;
            // am is the next multiple of block_size after a
            int am = (((a/block_size)*block_size)+block_size);
            // bm is the previous multiple of block_size before b
            int bm = (b/block_size)*block_size;

            // the 0th bit of ghist_words[a/block_size] is the most recent bit.
            // so the number of bits between a and am is the number to shift
            // right?

            // start out x as remainder bits from the beginning:
            // x = [ . . . . . b b b b b ]
            x += ghist_words[a / block_size] >> (a-am);
            // add in bits from the middle
            for (int i=am; i<bm; i+=block_size) {
                x += ghist_words[i / block_size];
            }
            // add in remainder bits from end:
            // x += [ b b b b b . . . . . ]
            unsigned int y = ghist_words[bm / block_size] & ((1<<(b - bm))-1);
            x += y << (block_size - (b - bm));
            return x;
        }
        void setBitRequirements() const override
        {
            if (mpp.ghist_length <= p2) {
                mpp.ghist_length = p2 + 1;
            }
        }
    };

    class ACYCLIC : public HistorySpec
    {
      public:
        ACYCLIC(int p1, int p2, int p3, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, p2, p3, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            int a = p1;
            int shift = p2;
            int style = p3;
            std::vector<std::vector<bool>> &acyclic_histories =
                mpp.threadData[tid]->acyclic_histories;
            std::vector<std::vector<unsigned int>> &acyclic2_histories =
                mpp.threadData[tid]->acyclic2_histories;

            unsigned int x = 0;
            if (style == -1) {
                unsigned int k = 0;
                for (int i = 0; i < a + 2; i += 1) {
                    x ^= acyclic_histories[a][i] << k;
                    k += 1;
                    k %= mpp.blockSize;
                }
            } else {
                for (int i = 0; i < a + 2; i += 1) {
                    x <<= shift;
                    x += acyclic2_histories[a][i];
                }
            }
            return x;
        }
        void setBitRequirements() const override
        {
            if (mpp.acyclic_bits.size() < (p1 + 1)) {
                mpp.acyclic_bits.resize(p1 + 1);
            }
            if (mpp.acyclic_bits[p1].size() < (p1 + 2)) {
                mpp.acyclic_bits[p1].resize(p1 + 2, std::vector<bool>(2));
            }
            for (int j = 0; j < p1 + 2; j += 1) {
                mpp.acyclic_bits[p1][j][!p3] = true;
            }
        }
    };

    class MODHIST : public HistorySpec
    {
      public:
        MODHIST(int p1, int p2, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, p2, 0, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            int a = p1;
            int b = p2;
            std::vector<std::vector<bool>> &mod_histories =
                mpp.threadData[tid]->mod_histories;

            unsigned int x = 0, k = 0;
            for (int i = 0; i < b; i += 1) {
                x ^= mod_histories[a][i] << k;
                k += 1;
                k %= mpp.blockSize;
            }
            return x;
        }
        void setBitRequirements() const override
        {
            mpp.insertModhistSpec(p1, p2);
        }
    };

    class BIAS : public HistorySpec
    {
      public:
        BIAS(double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(0, 0, 0, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            return 0;
        }
    };


    class RECENCY : public HistorySpec
    {
      public:
        RECENCY(int p1, int p2, int p3, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, p2, p3, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            int depth = p1;
            int shift = p2;
            int style = p3;
            std::vector<unsigned int short> &recency_stack =
                mpp.threadData[tid]->recency_stack;

            if (style == -1) {
                unsigned int x = 0;
                for (int i = 0; i < depth; i += 1) {
                    x <<= shift;
                    x += recency_stack[i];
                }
                return x;
            } else {
                unsigned int x = 0, k = 0;
                for (int i = 0; i < depth; i += 1) {
                    x ^= (!!(recency_stack[i] & (1 << shift))) << k;
                    k += 1;
                    k %= mpp.blockSize;
                }
                return x;
            }
        }
        void setBitRequirements() const override
        {
            if (mpp.assoc < p1) {
                mpp.assoc = p1;
            }
            mpp.doing_recency = true;
        }
    };

    class IMLI : public HistorySpec
    {
        public:
            IMLI(int p1, double coeff, int size, int width,
                    MultiperspectivePerceptron &mpp)
                : HistorySpec(p1, 0, 0, coeff, size, width, mpp)
            {}

            unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
                override
                {
                    assert(p1 >= 1);
                    assert(p1 <= 4);
                    return mpp.threadData[tid]->imli_counter[p1-1];
                }

            void setBitRequirements() const override
            {
                mpp.imli_counter_bits[p1 - 1] = 32;
            }
    };

    class PATH : public HistorySpec
    {
      public:
        PATH(int p1, int p2, int p3, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, p2, p3, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            int depth = p1;
            int shift = p2;
            int style = p3;
            std::vector<unsigned short int> &path_history =
                mpp.threadData[tid]->path_history;

            if (style == -1) {
                unsigned int x = 0;
                for (int i = 0; i < depth; i += 1) {
                    x <<= shift;
                    x += path_history[i];
                }
                return x;
            } else {
                unsigned int x = 0;
                int bm = (depth / mpp.blockSize) * mpp.blockSize;
                for (int i = 0; i < bm; i += mpp.blockSize) {
                    for (int j = 0; j < mpp.blockSize; j += 1) {
                        x ^= (!!(path_history[i + j] & (1 << shift))) << j;
                    }
                }
                int k = 0;
                for (int i = bm; i < depth; i += 1) {
                    x ^= (!!(path_history[i] & (1 << shift))) << k++;
                }
                return x;
            }
        }
        void setBitRequirements() const override
        {
            if (mpp.path_length <= p1) {
                mpp.path_length = p1 + 1;
            }
        }
    };

    class LOCAL : public HistorySpec
    {
      public:
        LOCAL(int p1, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, 0, 0, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            unsigned int x = mpp.threadData[tid]->localHistories[pc];
            if (p1 != -1) {
                x &= ((1 << p1) - 1);
            }
            return x;
        }
        void setBitRequirements() const override
        {
            mpp.doing_local = true;
        }
    };

    class MODPATH : public HistorySpec
    {
      public:
        MODPATH(int p1, int p2, int p3, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, p2, p3, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            int a = p1;
            int depth = p2;
            int shift = p3;

            unsigned int x = 0;
            for (int i=0; i<depth; i += 1) {
                x <<= shift;
                x += mpp.threadData[tid]->modpath_histories[a][i];
            }
            return x;
        }
        void setBitRequirements() const override
        {
            mpp.insertModpathSpec(p1, p2);
        }
    };

    class GHISTPATH : public HistorySpec
    {
      public:
        GHISTPATH(int p1, int p2, int p3, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, p2, p3, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            int depth = p1;
            int shift = p2;
            int style = p3;
            std::vector<unsigned int> &ghist_words =
                mpp.threadData[tid]->ghist_words;
            std::vector<unsigned short int> &path_history =
                mpp.threadData[tid]->path_history;

            if (style == -1) {
                unsigned int x = 0;
                int bm = (depth / mpp.blockSize) * mpp.blockSize;
                unsigned int w;
                for (int i = 0; i < bm; i += mpp.blockSize) {
                    w = ghist_words[i / mpp.blockSize];
                    for (int j = 0; j < mpp.blockSize; j += 1) {
                        x <<= shift;
                        x += (path_history[i + j] << 1) | (w & 1);
                        w >>= 1;
                    }
                }
                w = ghist_words[bm / mpp.blockSize];
                for (int i = bm; i < depth; i += 1) {
                    x <<= shift;
                    x += (path_history[i] << 1) | (w & 1);
                    w >>= 1;
                }
                return x;
            } else {
                unsigned int x = 0;
                int bm = (depth / mpp.blockSize) * mpp.blockSize;
                unsigned int w = 0;
                for (int i = 0; i < bm; i += mpp.blockSize) {
                    w = ghist_words[i / mpp.blockSize];
                    for (int j = 0; j < mpp.blockSize; j += 1) {
                        x ^= (!!(path_history[i + j] & (1 << shift))) << j;
                        x ^= (w & 1) << j;
                        w >>= 1;
                    }
                }
                w = ghist_words[bm/mpp.blockSize];
                int k = 0;
                for (int i = bm; i < depth; i += 1) {
                    x ^= (!!(path_history[i] & (1 << shift))) << k;
                    x ^= (w & 1) << k;
                    w >>= 1;
                    k += 1;
                }
                return x;
            }
        }

        void setBitRequirements() const override
        {
            if (mpp.ghist_length <= p1) {
                mpp.ghist_length = p1 + 1;
            }
            if (mpp.path_length <= p1) {
                mpp.path_length = p1 + 1;
            }
        }
    };

    class GHISTMODPATH : public HistorySpec
    {
      public:
        GHISTMODPATH(int p1, int p2, int p3, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, p2, p3, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            int a = p1;
            int depth = p2;
            int shift = p3;
            std::vector<std::vector<unsigned short int>> &modpath_histories =
                mpp.threadData[tid]->modpath_histories;
            std::vector<std::vector<bool>> &mod_histories =
                mpp.threadData[tid]->mod_histories;

            unsigned int x = 0;
            for (int i = 0; i < depth; i += 1) {
                x <<= shift;
                x += (modpath_histories[a][i] << 1) | mod_histories[a][i];
            }
            return x;
        }
        void setBitRequirements() const override
        {
            mpp.insertModhistSpec(p1, p2);
            mpp.insertModpathSpec(p1, p2);
        }
    };

    class BLURRYPATH : public HistorySpec
    {
      public:
        BLURRYPATH(int p1, int p2, int p3, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, p2, p3, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            int scale = p1;
            int depth = p2;
            int shiftdelta = p3;

            if (shiftdelta == -1) shiftdelta = 0;
            int sdint = shiftdelta >> 2;
            int sdfrac = shiftdelta & 3;
            unsigned int x = 0;
            int shift = 0;
            int count = 0;
            for (int i = 0; i < depth; i += 1) {
                x += mpp.threadData[tid]->blurrypath_histories[scale][i] >>
                    shift;
                count += 1;
                if (count == sdfrac) {
                    shift += sdint;
                    count = 0;
                }
            }
            return x;

        }
        void setBitRequirements() const override
        {
            if (mpp.blurrypath_bits.size() < (p1 + 1)) {
                mpp.blurrypath_bits.resize(p1 + 1);
            }
            if (mpp.blurrypath_bits[p1].size() < p2) {
                mpp.blurrypath_bits[p1].resize(p2);
            }
            for (int j = 0; j < p2; j += 1) {
                mpp.blurrypath_bits[p1][j] = 32 - p1;
            }
        }
    };

    class RECENCYPOS : public HistorySpec
    {
      public:
        RECENCYPOS(int p1, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, 0, 0, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            return hash(mpp.threadData[tid]->recency_stack, mpp.table_sizes,
                    pc2, p1, t);
        }

        static unsigned int hash(
            const std::vector<unsigned int short> &recency_stack,
            const std::vector<int> &table_sizes, unsigned short int pc, int l,
            int t)
        {
            // search for the PC

            for (int i = 0; i < l; i += 1) {
                if (recency_stack[i] == pc) {
                    return i * table_sizes[t] / l;
                }
            }

            // return last index in table on a miss

            return table_sizes[t] - 1;
        }

        void setBitRequirements() const override
        {
            if (mpp.assoc < p1) {
                mpp.assoc = p1;
            }
            mpp.doing_recency = true;
        }
    };

    class SGHISTPATH : public HistorySpec
    {
      public:
        SGHISTPATH(int p1, int p2, int p3, double coeff, int size, int width,
                MultiperspectivePerceptron &mpp)
            : HistorySpec(p1, p2, p3, coeff, size, width, mpp)
        {}

        unsigned int getHash(ThreadID tid, Addr pc, Addr pc2, int t) const
            override
        {
            int a = p1;
            int b = p2;
            int shift = p3;
            std::vector<unsigned int> &ghist_words =
                mpp.threadData[tid]->ghist_words;
            std::vector<unsigned short int> &path_history =
                mpp.threadData[tid]->path_history;

            unsigned int x = 0;
            int bm = (b / mpp.blockSize) * mpp.blockSize;
            unsigned int w;
            for (int i = a; i < bm; i += mpp.blockSize) {
                w = ghist_words[i / mpp.blockSize];
                for (int j = 0; j < mpp.blockSize; j += 1) {
                    x <<= shift;
                    x += (path_history[i+j] << 1) | (w & 1);
                    w >>= 1;
                }
            }
            w = ghist_words[bm / mpp.blockSize];
            for (int i = bm; i < b; i += 1) {
                x <<= shift;
                x += (path_history[i] << 1) | (w & 1);
                w >>= 1;
            }
            return x;
        }
    };

    public:
    MultiperspectivePerceptron(const MultiperspectivePerceptronParams &params);

    /**
     * Sets the starting number of storage bits to compute the number of
     * table entries
     * @param bits number of bits used
     */
    void setExtraBits(int bits);

    void init() override;

    // Base class methods.
    bool lookup(ThreadID tid, Addr branch_addr, void* &bpHistory) override;
    void updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                         Addr target,  void * &bpHistory) override;
    void update(ThreadID tid, Addr branch_addr, bool taken, void * &bpHistory,
                bool squashed, const StaticInstPtr & inst,
                Addr corrTarget) override;
    void squash(ThreadID tid, void * &bpHistory) override;
};

} // namespace branch_prediction
} // namespace gem5

#endif//__CPU_PRED_MULTIPERSPECTIVE_PERCEPTRON_HH__
