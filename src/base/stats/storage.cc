/*
 * Copyright (c) 2021 Daniel R. Carvalho
 * Copyright (c) 2019 Arm Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "base/stats/storage.hh"

#include <cmath>

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Stats, statistics);
namespace statistics
{

void
DistStor::sample(Counter val, int number)
{
    assert(bucket_size > 0);
    if (val < min_track)
        underflow += number;
    else if (val > max_track)
        overflow += number;
    else {
        cvec[std::floor((val - min_track) / bucket_size)] += number;
    }

    if (val < min_val)
        min_val = val;

    if (val > max_val)
        max_val = val;

    sum += val * number;
    squares += val * val * number;
    samples += number;
}

void
HistStor::growOut()
{
    int size = cvec.size();
    int zero = size / 2; // round down!
    int top_half = zero + (size - zero + 1) / 2; // round up!
    int bottom_half = (size - zero) / 2; // round down!

    // grow down
    int low_pair = zero - 1;
    for (int i = zero - 1; i >= bottom_half; i--) {
        cvec[i] = cvec[low_pair];
        if (low_pair - 1 >= 0)
            cvec[i] += cvec[low_pair - 1];
        low_pair -= 2;
    }
    assert(low_pair == 0 || low_pair == -1 || low_pair == -2);

    for (int i = bottom_half - 1; i >= 0; i--)
        cvec[i] = Counter();

    // grow up
    int high_pair = zero;
    for (int i = zero; i < top_half; i++) {
        cvec[i] = cvec[high_pair];
        if (high_pair + 1 < size)
            cvec[i] += cvec[high_pair + 1];
        high_pair += 2;
    }
    assert(high_pair == size || high_pair == size + 1);

    for (int i = top_half; i < size; i++)
        cvec[i] = Counter();

    max_bucket *= 2;
    min_bucket *= 2;
    bucket_size *= 2;
}

void
HistStor::growDown()
{
    const int size = cvec.size();
    const int zero = size / 2; // round down!
    const bool even = ((size - 1) % 2) == 0;

    // Make sure that zero becomes the lower bound of the middle bucket. On
    // an even number of buckets the last bucket does not change its lower
    // bound, therefore it does not need to absorb any other bucket
    int pair = size - 1;
    if (even) {
        pair--;
    }
    for (int i = pair; i >= zero; --i) {
        cvec[i] = cvec[pair];
        if (pair - 1 >= 0)
            cvec[i] += cvec[pair - 1];
        pair -= 2;
    }

    for (int i = zero - 1; i >= 0; i--)
        cvec[i] = Counter();

    // Double the range by using the negative of the lower bound of the last
    // bucket as the new lower bound of the first bucket
    min_bucket = -max_bucket;

    // A special case must be handled when there is an odd number of
    // buckets so that zero is kept as the lower bound of the middle bucket
    if (!even) {
        min_bucket -= bucket_size;
        max_bucket -= bucket_size;
    }

    // Only update the bucket size once the range has been updated
    bucket_size *= 2;
}

void
HistStor::growUp()
{
    int size = cvec.size();
    int half = (size + 1) / 2; // round up!

    int pair = 0;
    for (int i = 0; i < half; i++) {
        cvec[i] = cvec[pair];
        if (pair + 1 < size)
            cvec[i] += cvec[pair + 1];
        pair += 2;
    }
    assert(pair == size || pair == size + 1);

    for (int i = half; i < size; i++)
        cvec[i] = Counter();

    max_bucket *= 2;
    bucket_size *= 2;
}

void
HistStor::sample(Counter val, int number)
{
    assert(min_bucket < max_bucket);
    if (val < min_bucket) {
        if (min_bucket == 0)
            growDown();

        while (val < min_bucket)
            growOut();
    } else if (val >= max_bucket + bucket_size) {
        if (min_bucket == 0) {
            while (val >= max_bucket + bucket_size)
                growUp();
        } else {
            while (val >= max_bucket + bucket_size)
                growOut();
        }
    }

    assert(bucket_size > 0);
    size_type index =
        (int64_t)std::floor((val - min_bucket) / bucket_size);

    assert(index < size());
    cvec[index] += number;

    sum += val * number;
    squares += val * val * number;
    logs += std::log(val) * number;
    samples += number;
}

void
HistStor::add(HistStor *hs)
{
    int b_size = hs->size();
    assert(size() == b_size);
    assert(min_bucket == hs->min_bucket);

    sum += hs->sum;
    logs += hs->logs;
    squares += hs->squares;
    samples += hs->samples;

    while (bucket_size > hs->bucket_size)
        hs->growUp();
    while (bucket_size < hs->bucket_size)
        growUp();

    for (uint32_t i = 0; i < b_size; i++)
        cvec[i] += hs->cvec[i];
}

} // namespace statistics
} // namespace gem5
