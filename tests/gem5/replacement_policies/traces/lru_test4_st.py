# Copyright (c) 2022 Jarvis Jia, Jing Qu, Matt Sinclair, & Mingyuan Xiang
# All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# This test is targeting stores
# Access pattern: A, C, E, G, E, G, A, C, I, E
# Each letter represents a 64-byte address range.

# The [] indicate two different sets, and each set has four ways.
# [set0way0, set0way1, set0way2, set0way3],
# [set1way0, set1way1, set1way2, set1way3],
# If you have a 512B cache with 4-way associativity, and each cache
# line is 64B with LRU replacement policy, you will observe:
# m, m, m, m, h, h, h, h, m, m, where 'h' means
# hit and 'm' means miss.

# Explanation of this result:
# A, C, E, G are misses, now the cache stores ([A*, C, E, G],[ , , ,]).
# A is marked as the LRU address range.
# E and G then hits, and the cache stores ([A*, C, E, G],[ , , ,]).
# A and C then hits, and the cache stores ([A, C, E*, G],[ , , ,]).
# I searches for a victim and selects E. Now the cache stores ([A, C, I, G*],[ , , ,]).
# E searches for a victim and selects G. Now the cache stores ([A*, C, I, E],[ , , ,]).

from m5.objects.ReplacementPolicies import LRURP as rp


def python_generator(generator):
    yield generator.createLinear(60000, 0, 63, 64, 30000, 30000, 0, 0)
    yield generator.createLinear(60000, 128, 191, 64, 30000, 30000, 0, 0)
    yield generator.createLinear(60000, 256, 319, 64, 30000, 30000, 0, 0)
    yield generator.createLinear(60000, 384, 447, 64, 30000, 30000, 0, 0)
    yield generator.createLinear(60000, 256, 319, 64, 30000, 30000, 0, 0)
    yield generator.createLinear(60000, 384, 447, 64, 30000, 30000, 0, 0)
    yield generator.createLinear(60000, 0, 63, 64, 30000, 30000, 0, 0)
    yield generator.createLinear(60000, 128, 191, 64, 30000, 30000, 0, 0)
    yield generator.createLinear(60000, 512, 575, 64, 30000, 30000, 0, 0)
    yield generator.createLinear(60000, 256, 319, 64, 30000, 30000, 0, 0)
    yield generator.createLinear(30000, 0, 0, 0, 30000, 30000, 0, 0)

    yield generator.createExit(0)
