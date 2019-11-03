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
  * Multiperspective Perceptron Predictor (by Daniel A. Jiménez)
  */

#include "cpu/pred/multiperspective_perceptron.hh"

#include "base/random.hh"
#include "debug/Branch.hh"

int
MultiperspectivePerceptron::xlat[] =
    {1,3,4,5,7,8,9,11,12,14,15,17,19,21,23,25,27,29,32,34,37,41,45,49,53,58,63,
     69,76,85,94,106,};
int
MultiperspectivePerceptron::xlat4[] =
    {0,4,5,7,9,11,12,14,16,17,19,22,28,33,39,45,};

MultiperspectivePerceptron::ThreadData::ThreadData(int num_filters,
        int n_local_histories, int local_history_length, int assoc,
        const std::vector<std::vector<int>> &blurrypath_bits, int path_length,
        int ghist_length, int block_size,
        const std::vector<std::vector<std::vector<bool>>> &acyclic_bits,
        const std::vector<int> &modhist_indices,
        const std::vector<int> &modhist_lengths,
        const std::vector<int> &modpath_indices,
        const std::vector<int> &modpath_lengths,
        const std::vector<int> &table_sizes, int n_sign_bits)
      : filterTable(num_filters), acyclic_histories(acyclic_bits.size()),
        acyclic2_histories(acyclic_bits.size()),
        blurrypath_histories(blurrypath_bits.size()),
        ghist_words(ghist_length/block_size+1, 0),
        path_history(path_length, 0), imli_counter(4,0),
        localHistories(n_local_histories, local_history_length),
        recency_stack(assoc), last_ghist_bit(false), occupancy(0)
{
    for (int i = 0; i < blurrypath_bits.size(); i+= 1) {
        blurrypath_histories[i].resize(blurrypath_bits[i].size());
    }

    for (int i = 0; i < acyclic_bits.size(); i += 1) {
        acyclic_histories[i].resize(acyclic_bits[i].size());
        acyclic2_histories[i].resize(acyclic_bits[i].size());
    }

    int max_modhist_idx = -1;
    for (auto &elem : modhist_indices) {
        max_modhist_idx = (max_modhist_idx < elem) ? elem : max_modhist_idx;
    }
    if (max_modhist_idx >= 0) {
        mod_histories.resize(max_modhist_idx + 1);
    }
    for (int i = 0; i < modhist_lengths.size(); i+= 1) {
        mod_histories[modhist_indices[i]].resize(modhist_lengths[i]);
    }

    int max_modpath_idx = -1;
    for (auto &elem : modpath_indices) {
        max_modpath_idx = (max_modpath_idx < elem) ? elem : max_modpath_idx;
    }
    if (max_modpath_idx >= 0) {
        modpath_histories.resize(max_modpath_idx + 1);
    }
    for (int i = 0; i < modpath_lengths.size(); i+= 1) {
        modpath_histories[modpath_indices[i]].resize(modpath_lengths[i]);
    }

    for (int i = 0; i < table_sizes.size(); i += 1) {
        mpreds.push_back(0);
        tables.push_back(std::vector<short int>(table_sizes[i]));
        sign_bits.push_back(std::vector<std::array<bool, 2>>(table_sizes[i]));
        for (int j = 0; j < table_sizes[i]; j += 1) {
            for (int k = 0; k < n_sign_bits; k += 1) {
                sign_bits[i][j][k] = (i & 1) | (k & 1);
            }
        }
    }
}

MultiperspectivePerceptron::MultiperspectivePerceptron(
    const MultiperspectivePerceptronParams *p) : BPredUnit(p),
    blockSize(p->block_size), pcshift(p->pcshift), threshold(p->threshold),
    bias0(p->bias0), bias1(p->bias1), biasmostly0(p->biasmostly0),
    biasmostly1(p->biasmostly1), nbest(p->nbest), tunebits(p->tunebits),
    hshift(p->hshift), imli_mask1(p->imli_mask1), imli_mask4(p->imli_mask4),
    recencypos_mask(p->recencypos_mask), fudge(p->fudge),
    n_sign_bits(p->n_sign_bits), pcbit(p->pcbit), decay(p->decay),
    record_mask(p->record_mask), hash_taken(p->hash_taken),
    tuneonly(p->tuneonly), extra_rounds(p->extra_rounds), speed(p->speed),
    budgetbits(p->budgetbits), speculative_update(p->speculative_update),
    threadData(p->numThreads, nullptr), doing_local(false),
    doing_recency(false), assoc(0), ghist_length(p->initial_ghist_length),
    modghist_length(1), path_length(1), thresholdCounter(0),
    theta(p->initial_theta), extrabits(0), imli_counter_bits(4),
    modhist_indices(), modhist_lengths(), modpath_indices(), modpath_lengths()
{
    fatal_if(speculative_update, "Speculative update not implemented");
}

void
MultiperspectivePerceptron::setExtraBits(int bits)
{
    extrabits = bits;
}

void
MultiperspectivePerceptron::init()
{
    createSpecs();

    for (auto &spec : specs) {
        // initial assignation of values
        table_sizes.push_back(spec->size);
    }

    // Update bit requirements and runtime values
    for (auto &spec : specs) {
        spec->setBitRequirements();
    }
    const MultiperspectivePerceptronParams *p =
        static_cast<const MultiperspectivePerceptronParams *>(params());

    computeBits(p->num_filter_entries, p->num_local_histories,
                p->local_history_length, p->ignore_path_size);

    for (int i = 0; i < threadData.size(); i += 1) {
        threadData[i] = new ThreadData(p->num_filter_entries,
                                       p->num_local_histories,
                                       p->local_history_length, assoc,
                                       blurrypath_bits, path_length,
                                       ghist_length, blockSize, acyclic_bits,
                                       modhist_indices, modhist_lengths,
                                       modpath_indices, modpath_lengths,
                                       table_sizes, n_sign_bits);
    }
}

void
MultiperspectivePerceptron::computeBits(int num_filter_entries,
        int nlocal_histories, int local_history_length, bool ignore_path_size)
{
    int totalbits = extrabits;
    for (auto &imli_bits : imli_counter_bits) {
        totalbits += imli_bits;
    }
    totalbits += ghist_length;
    if (!ignore_path_size) {
        totalbits += path_length * 16;
    }
    totalbits += (threshold >= 0) ? (tunebits * specs.size()) : 0;
    for (auto &len : modhist_lengths) {
        totalbits += len;
    }
    if (!ignore_path_size) {
        for (auto &len : modpath_lengths) {
            totalbits += 16 * len;
        }
    }
    totalbits += doing_local ? (nlocal_histories * local_history_length) : 0;
    totalbits += doing_recency ? (assoc * 16) : 0;

    for (auto &bv : blurrypath_bits) {
        for (auto &bve : bv) {
            totalbits += bve;
        }
    }
    totalbits += num_filter_entries * 2;

    for (auto &abi : acyclic_bits) {
        for (auto &abj : abi) {
            for (auto abk : abj) {
                totalbits += abk;
            }
        }
    }

    int remaining = budgetbits - totalbits;

    // count the tables that have already been assigned sizes
    int num_sized = 0;
    for (int i = 0; i < specs.size(); i +=1) {
        if (table_sizes[i] != 0) {
            int sz = table_sizes[i] * (specs[i]->width + (n_sign_bits - 1));
            totalbits += sz;
            remaining -= sz;
            num_sized += 1;
        }
    }

    // whatever is left, we divide among the rest of the tables
    int table_size_bits = (remaining / (specs.size()-num_sized));
    for (int i = 0; i < specs.size(); i += 1) {
        // if a table doesn't have a size yet, give it one and count those bits
        if (!table_sizes[i]) {
            int my_table_size = table_size_bits /
                (specs[i]->width + (n_sign_bits - 1)); // extra sign bits
            table_sizes[i] = my_table_size;
            totalbits += my_table_size * (specs[i]->width + (n_sign_bits - 1));
        }
    }

    DPRINTF(Branch, "%d bits of metadata so far, %d left out of "
            "%d total budget\n", totalbits, remaining, budgetbits);
    DPRINTF(Branch, "table size is %d bits, %d entries for 5 bit, %d entries "
            "for 6 bit\n", table_size_bits,
            table_size_bits / (5 + (n_sign_bits - 1)),
            table_size_bits / (6 + (n_sign_bits - 1)));
    DPRINTF(Branch, "%d total bits (%0.2fKB)\n", totalbits,
            totalbits / 8192.0);
}

void
MultiperspectivePerceptron::findBest(ThreadID tid,
                                     std::vector<int> &best_preds) const
{
    if (threshold < 0) {
        return;
    }
    struct BestPair {
        int index;
        int mpreds;
        bool operator<(BestPair const &bp) const
        {
            return mpreds < bp.mpreds;
        }
    };
    std::vector<BestPair> pairs(best_preds.size());
    for (int i = 0; i < best_preds.size(); i += 1) {
        pairs[i].index = i;
        pairs[i].mpreds = threadData[tid]->mpreds[i];
    }
    std::sort(pairs.begin(), pairs.end());
    for (int i = 0; i < (std::min(nbest, (int) best_preds.size())); i += 1) {
        best_preds[i] = pairs[i].index;
    }
}

unsigned int
MultiperspectivePerceptron::getIndex(ThreadID tid, const MPPBranchInfo &bi,
                                     const HistorySpec &spec, int index) const
{
    unsigned int g = spec.getHash(tid, bi.getPC(), bi.getPC2(), index);
    unsigned long long int h = g;
     // shift the hash from the feature to xor with the hashed PC
    if (hshift < 0) {
        h <<= -hshift;
        h ^= bi.getPC2();
    } else {
        h <<= hshift;
        h ^= bi.getHPC();
    }
    // xor in the imli counter(s) and/or recency position based on the masks
    if ((1ull<<index) & imli_mask1) {
        h ^= threadData[tid]->imli_counter[0];
    }
    if ((1ull<<index) & imli_mask4) {
        h ^= threadData[tid]->imli_counter[3];
    }
    if (doing_recency) {
        if ((1ull<<index) & recencypos_mask) {
            h ^= RECENCYPOS::hash(threadData[tid]->recency_stack, table_sizes,
                    bi.getPC2(), 31, index);
        }
    }
    h %= table_sizes[index];
    return h;
}

int
MultiperspectivePerceptron::computeOutput(ThreadID tid, MPPBranchInfo &bi)
{
    // list of best predictors
    std::vector<int> best_preds(specs.size(), -1);

    // initialize sum
    bi.yout = 0;

    // bias the prediction by whether the local history is
    // one of four distinctive patterns
    int lhist = threadData[tid]->localHistories[bi.getPC()];
    int history_len = threadData[tid]->localHistories.getLocalHistoryLength();
    if (lhist == 0) {
        bi.yout = bias0;
    } else if (lhist == ((1<<history_len)-1)) {
        bi.yout = bias1;
    } else if (lhist == (1<<(history_len-1))) {
        bi.yout = biasmostly0;
    } else if (lhist == ((1<<(history_len-1))-1)) {
        bi.yout = biasmostly1;
    }
    // find the best subset of features to use in case of a low-confidence
    // branch
    findBest(tid, best_preds);

    // begin computation of the sum for low-confidence branch
    int bestval = 0;

    for (int i = 0; i < specs.size(); i += 1) {
        HistorySpec const &spec = *specs[i];
        // get the hash to index the table
        unsigned int hashed_idx = getIndex(tid, bi, spec, i);
        // add the weight; first get the weight's magnitude
        int counter = threadData[tid]->tables[i][hashed_idx];
        // get the sign
        bool sign =
          threadData[tid]->sign_bits[i][hashed_idx][bi.getHPC() % n_sign_bits];
        // apply the transfer function and multiply by a coefficient
        int weight = spec.coeff * ((spec.width == 5) ?
                                   xlat4[counter] : xlat[counter]);
        // apply the sign
        int val = sign ? -weight : weight;
        // add the value
        bi.yout += val;
        // if this is one of those good features, add the value to bestval
        if (threshold >= 0) {
            for (int j = 0;
                 j < std::min(nbest, (int) best_preds.size());
                 j += 1)
            {
                if (best_preds[j] == i) {
                    bestval += val;
                    break;
                }
            }
        }
    }
    // apply a fudge factor to affect when training is triggered
    bi.yout *= fudge;
    return bestval;
}

void
MultiperspectivePerceptron::satIncDec(bool taken, bool &sign, int &counter,
                                      int max_weight) const
{
    if (taken) {
        // increment sign/magnitude
        if (sign) {
            // go toward 0 away from negative max weight
            if (counter == 0) {
                sign = false; // moved to positive 0
            } else {
                counter -= 1;
            }
        } else {
            // go toward max weight away from 0
            if (counter < max_weight) {
                counter += 1;
            }
        }
    } else {
        // decrement sign/magnitude
        if (sign) {
            // go toward negative max weight down from 0
            if (counter < max_weight) {
                counter += 1;
            }
        } else {
            // go toward 0 away from max weight
            if (counter == 0) {
                sign = true; // negative 0
            } else {
                counter -= 1;
            }
        }
    }
}

void
MultiperspectivePerceptron::train(ThreadID tid, MPPBranchInfo &bi, bool taken)
{
    std::vector<std::vector<short int>> &tables = threadData[tid]->tables;
    std::vector<std::vector<std::array<bool, 2>>> &sign_bits =
            threadData[tid]->sign_bits;
    std::vector<int> &mpreds = threadData[tid]->mpreds;
    // was the prediction correct?
    bool correct = (bi.yout >= 1) == taken;
    // what is the magnitude of yout?
    int abs_yout = abs(bi.yout);
    // keep track of mispredictions per table
    if (threshold >= 0) if (!tuneonly || (abs_yout <= threshold)) {
        bool halve = false;

        // for each table, figure out if there was a misprediction
        for (int i = 0; i < specs.size(); i += 1) {
            HistorySpec const &spec = *specs[i];
            // get the hash to index the table
            unsigned int hashed_idx = getIndex(tid, bi, spec, i);
            bool sign = sign_bits[i][hashed_idx][bi.getHPC() % n_sign_bits];
            int counter = tables[i][hashed_idx];
            int weight = spec.coeff * ((spec.width == 5) ?
                                       xlat4[counter] : xlat[counter]);
            if (sign) weight = -weight;
            bool pred = weight >= 1;
            if (pred != taken) {
                mpreds[i] += 1;
                if (mpreds[i] == (1 << tunebits) - 1) {
                    halve = true;
                }
            }
        }
        // if we reach the maximum counter value, halve all the counters
        if (halve) {
            for (int i = 0; i < specs.size(); i += 1) {
                mpreds[i] /= 2;
            }
        }
    }
    // if the branch was predicted incorrectly or the correct
    // prediction was weak, update the weights
    bool do_train = !correct || (abs_yout <= theta);
    if (!do_train) return;

    // adaptive theta training, adapted from O-GEHL
    if (!correct) {
        thresholdCounter += 1;
        if (thresholdCounter >= speed) {
            theta += 1;
            thresholdCounter = 0;
        }
    }
    if (correct && abs_yout < theta) {
        thresholdCounter -= 1;
        if (thresholdCounter <= -speed) {
            theta -= 1;
            thresholdCounter = 0;
        }
    }

    // train the weights, computing what the value of yout
    // would have been if these updates had been applied before
    int newyout = 0;
    for (int i = 0; i < specs.size(); i += 1) {
        HistorySpec const &spec = *specs[i];
        // get the magnitude
        unsigned int hashed_idx = getIndex(tid, bi, spec, i);
        int counter = tables[i][hashed_idx];
        // get the sign
        bool sign = sign_bits[i][hashed_idx][bi.getHPC() % n_sign_bits];
        // increment/decrement if taken/not taken
        satIncDec(taken, sign, counter, (1 << (spec.width - 1)) - 1);
        // update the magnitude and sign
        tables[i][hashed_idx] = counter;
        sign_bits[i][hashed_idx][bi.getHPC() % n_sign_bits] = sign;
        int weight = ((spec.width == 5) ? xlat4[counter] : xlat[counter]);
        // update the new version of yout
        if (sign) {
            newyout -= weight;
        } else {
            newyout += weight;
        }
    }

    // if the prediction still would have been incorrect even
    // with the updated weights, update some more weights to
    // try to fix the problem
    if ((newyout >= 1) != taken) {
        if (extra_rounds != -1) {
            int round_counter = 0;
            bool found;
            do {
                // udpate a random weight
                int besti = -1;
                int nrand = random_mt.random<int>() % specs.size();
                int pout;
                found = false;
                for (int j = 0; j < specs.size(); j += 1) {
                    int i = (nrand + j) % specs.size();
                    HistorySpec const &spec = *specs[i];
                    unsigned int hashed_idx = getIndex(tid, bi, spec, i);
                    int counter = tables[i][hashed_idx];
                    bool sign =
                        sign_bits[i][hashed_idx][bi.getHPC() % n_sign_bits];
                    int weight = ((spec.width == 5) ?
                            xlat4[counter] : xlat[counter]);
                    int signed_weight = sign ? -weight : weight;
                    pout = newyout - signed_weight;
                    if ((pout >= 1) == taken) {
                        // we have found a weight that if we blow
                        // it away will help!
                        besti = i;
                        break;
                    }
                }
                if (besti != -1) {
                    int i = besti;
                    HistorySpec const &spec = *specs[i];
                    unsigned int hashed_idx = getIndex(tid, bi, spec, i);
                    int counter = tables[i][hashed_idx];
                    bool sign =
                        sign_bits[i][hashed_idx][bi.getHPC() % n_sign_bits];
                    if (counter > 1) {
                        counter--;
                        tables[i][hashed_idx] = counter;
                    }
                    int weight = ((spec.width == 5) ?
                            xlat4[counter] : xlat[counter]);
                    int signed_weight = sign ? -weight : weight;
                    int out = pout + signed_weight;
                    round_counter += 1;
                    if ((out >= 1) != taken) {
                        found = true;
                    }
                }
            } while (found && round_counter < extra_rounds);
        }
    }
}

void
MultiperspectivePerceptron::uncondBranch(ThreadID tid, Addr pc,
                                         void * &bp_history)
{
    MPPBranchInfo *bi = new MPPBranchInfo(pc, pcshift, false);
    std::vector<unsigned int> &ghist_words = threadData[tid]->ghist_words;

    bp_history = (void *)bi;
    unsigned short int pc2 = pc >> 2;
    bool ab = !(pc & (1<<pcbit));
    for (int i = 0; i < ghist_length / blockSize + 1; i += 1) {
        bool ab_new = (ghist_words[i] >> (blockSize - 1)) & 1;
        ghist_words[i] <<= 1;
        ghist_words[i] |= ab;
        ghist_words[i] &= (1 << blockSize) - 1;
        ab = ab_new;
    }
    memmove(&threadData[tid]->path_history[1],
            &threadData[tid]->path_history[0],
            sizeof(unsigned short int) * (path_length - 1));
    threadData[tid]->path_history[0] = pc2;
}

bool
MultiperspectivePerceptron::lookup(ThreadID tid, Addr instPC,
                                   void * &bp_history)
{
    MPPBranchInfo *bi = new MPPBranchInfo(instPC, pcshift, true);
    bp_history = (void *)bi;

    bool use_static = false;

    if (!threadData[tid]->filterTable.empty()) {
        unsigned int findex =
            bi->getHashFilter(threadData[tid]->last_ghist_bit) %
            threadData[tid]->filterTable.size();
        FilterEntry &f = threadData[tid]->filterTable[findex];
        if (f.alwaysNotTakenSoFar()) {
            bi->filtered = true;
            bi->prediction = false;
            return false;
        } else if (f.alwaysTakenSoFar()) {
            bi->filtered = true;
            bi->prediction = true;
            return true;
        }
        if (f.neverSeen()) {
            use_static = true;
        }
    }

    int bestval = computeOutput(tid, *bi);
    if (use_static) {
        bi->prediction = false;
    } else {
        if (abs(bi->yout) <= threshold) {
            bi->prediction = (bestval >= 1);
        } else {
            bi->prediction = (bi->yout >= 1);
        }
    }

    return bi->prediction;
}

void
MultiperspectivePerceptron::update(ThreadID tid, Addr instPC, bool taken,
                                   void *bp_history, bool squashed,
                                   const StaticInstPtr & inst,
                                   Addr corrTarget)
{
    assert(bp_history);
    MPPBranchInfo *bi = static_cast<MPPBranchInfo*>(bp_history);
    assert(corrTarget != MaxAddr);
    if (squashed) {
        //delete bi;
        return;
    }

    if (bi->isUnconditional()) {
        delete bi;
        return;
    }

    bool do_train = true;

    if (!threadData[tid]->filterTable.empty()) {
        int findex = bi->getHashFilter(threadData[tid]->last_ghist_bit) %
                                       threadData[tid]->filterTable.size();
        FilterEntry &f = threadData[tid]->filterTable[findex];

        // compute this first, so we don't not train on the
        // first time a branch is seen.
        bool transition = false;
        if (f.alwaysNotTakenSoFar() || f.alwaysTakenSoFar()) {
            do_train = false;
        }
        if (taken) {
            if (f.alwaysNotTakenSoFar()) {
                transition = true;
            }
            f.seenTaken = true;
        } else {
            if (f.alwaysTakenSoFar()) {
                transition = true;
            }
            f.seenUntaken = true;
        }
        // is this the first time time the branch has gone both ways?
        if (transition) {
            threadData[tid]->occupancy += 1;
        }
        // for every new dynamic branch, when there ar
        // more than 'decay' number of branches in the
        // filter, blow a random filter entry away
        if (decay && transition &&
            ((threadData[tid]->occupancy > decay) || (decay == 1))) {
            int rnd = random_mt.random<int>() %
                      threadData[tid]->filterTable.size();
            FilterEntry &frand = threadData[tid]->filterTable[rnd];
            if (frand.seenTaken && frand.seenUntaken) {
                threadData[tid]->occupancy -= 1;
            }
            frand.seenTaken = false;
            frand.seenUntaken = false;
        }
    }

    if (do_train) {
        train(tid, *bi, taken);
    }

#define RECORD_FILTERED_IMLI      1
#define RECORD_FILTERED_GHIST     2
#define RECORD_FILTERED_PATH      4
#define RECORD_FILTERED_ACYCLIC   8
#define RECORD_FILTERED_MOD      16
#define RECORD_FILTERED_BLURRY   32
// should never record a filtered local branch - duh!
#define RECORD_FILTERED_LOCAL    64
#define RECORD_FILTERED_RECENCY 128

    // four different styles of IMLI
    if (!bi->filtered || (record_mask & RECORD_FILTERED_IMLI)) {
        unsigned int target = corrTarget;
        if (target < bi->getPC()) {
            if (taken) {
                threadData[tid]->imli_counter[0] += 1;
            } else {
                threadData[tid]->imli_counter[0] = 0;
            }
            if (!taken) {
                threadData[tid]->imli_counter[1] += 1;
            } else {
                threadData[tid]->imli_counter[1] = 0;
            }
        } else {
            if (taken) {
                threadData[tid]->imli_counter[2] += 1;
            } else {
                threadData[tid]->imli_counter[2] = 0;
            }
            if (!taken) {
                threadData[tid]->imli_counter[3] += 1;
            } else {
                threadData[tid]->imli_counter[3] = 0;
            }
        }
    }

    bool hashed_taken = hash_taken ? (taken ^ !!(bi->getPC() & (1<<pcbit)))
                                   : taken;
    // record into ghist
    if (!bi->filtered || (record_mask & RECORD_FILTERED_GHIST)) {
        bool ab = hashed_taken;
        assert(threadData[tid]->ghist_words.size() > 0);
        for (int i = 0; i < ghist_length / blockSize + 1; i += 1) {
            unsigned int a = threadData[tid]->ghist_words[i];
            bool ab_new = (a >> (blockSize - 1)) & 1;
            a <<= 1;
            a |= ab;
            ab = ab_new;
            a &= (1 << blockSize) - 1;
            threadData[tid]->ghist_words[i] = a;
        }
    }

    // record into path history
    if (!bi->filtered || (record_mask & RECORD_FILTERED_PATH)) {
        assert(threadData[tid]->path_history.size() > 0);
        memmove(&threadData[tid]->path_history[1],
                &threadData[tid]->path_history[0],
                sizeof(unsigned short int) * (path_length - 1));
        threadData[tid]->path_history[0] = bi->getPC2();
    }

    // record into acyclic history
    if (!bi->filtered || (record_mask & RECORD_FILTERED_ACYCLIC)) {
        threadData[tid]->updateAcyclic(hashed_taken, bi->getHPC());
    }

    // record into modulo path history
    if (!bi->filtered || (record_mask & RECORD_FILTERED_MOD)) {
        for (int ii = 0; ii < modpath_indices.size(); ii += 1) {
            int i = modpath_indices[ii];
            if (bi->getHPC() % (i + 2) == 0) {
                memmove(&threadData[tid]->modpath_histories[i][1],
                        &threadData[tid]->modpath_histories[i][0],
                        sizeof(unsigned short int) * (modpath_lengths[ii]-1));
                threadData[tid]->modpath_histories[i][0] = bi->getPC2();
            }
        }
    }

    // update blurry history
    if (!bi->filtered || (record_mask & RECORD_FILTERED_BLURRY)) {
        std::vector<std::vector<unsigned int>> &blurrypath_histories =
             threadData[tid]->blurrypath_histories;

        for (int i = 0; i < blurrypath_histories.size(); i += 1)
        {
            if (blurrypath_histories[i].size() > 0) {
                unsigned int z = bi->getPC() >> i;
                if (blurrypath_histories[i][0] != z) {
                    memmove(&blurrypath_histories[i][1],
                            &blurrypath_histories[i][0],
                            sizeof(unsigned int) *
                            (blurrypath_histories[i].size() - 1));
                    blurrypath_histories[i][0] = z;
                }
            }
        }
    }

    // record into modulo pattern history
    if (!bi->filtered || (record_mask & RECORD_FILTERED_MOD)) {
        for (int ii = 0; ii < modhist_indices.size(); ii += 1) {
            int i = modhist_indices[ii];
            if (bi->getHPC() % (i + 2) == 0) {
                for (int j = modhist_lengths[ii] - 1; j > 0; j -= 1) {
                    threadData[tid]->mod_histories[i][j] =
                        threadData[tid]->mod_histories[i][j-1];
                }
                threadData[tid]->mod_histories[i][0] = hashed_taken;
            }
        }
    }

    // insert this PC into the recency stack
    if (doing_recency) {
        if (!bi->filtered || (record_mask & RECORD_FILTERED_RECENCY)) {
            threadData[tid]->insertRecency(bi->getPC2(), assoc);
        }
    }

    // record into a local history
    if (!bi->filtered || (record_mask & RECORD_FILTERED_LOCAL)) {
        threadData[tid]->localHistories.update(bi->getPC(), hashed_taken);
    }

    // update last ghist bit, used to index filter
    threadData[tid]->last_ghist_bit = taken;

    delete bi;
}

void
MultiperspectivePerceptron::btbUpdate(ThreadID tid, Addr branch_pc,
                                      void* &bp_history)
{
}

void
MultiperspectivePerceptron::squash(ThreadID tid, void *bp_history)
{
    assert(bp_history);
    MPPBranchInfo *bi = static_cast<MPPBranchInfo*>(bp_history);
    delete bi;
}
