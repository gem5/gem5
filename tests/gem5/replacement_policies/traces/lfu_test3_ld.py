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
# This test is targeting loads.
# Access pattern: A, A, C, C, C, E, G, G, I, E, E, E, E, K,
# A, A, A, A, M, G, G, G, G, O, C
# Each letter represents a 64-byte address range.
# The [] indicate two different sets, and each set has four ways.
# [set0way0, set0way1, set0way2, set0way3],
# [set1way0, set1way1, set1way2, set1way3],
# If you have a 512B cache with 4-way associativity, and each cache
# line is 64B, with LFU replacement policy, you will observe:
# m, h, m, h, h, m, m, h, m, m, h, h, h, m, m, h, h, h, m, m, h, h,
# h, m, m, where 'm' means miss, and 'h' means hit.
# Explanation of this result:
# After two A accesses, three C accesses, one E access and two G accesses,
# the cache stores ([A2,C3,E1,G2],[ , , ,]). The numbers after each letter are the
# counts of accesses to that address range.
# I searches a victim and selects E. Now the cache stores ([A2,C3,I1,G2],[ , , ,]).
# E searches a victim and selects I. Now the cache stores ([A2,C3,E1,G2],[ , , ,]).
# Three E accesses are hits. Now the cache stores ([A2,C3,E4,G2],[ , , ,]).
# K searches a victim and selects A. Now the cache stores ([K1,C3,E4,G2],[ , , ,]).
# A searches a victim and selects K. Now the cache stores ([A1,C3,E4,G2],[ , , ,]).
# Three A accesses are hits. Now the cache stores ([A4,C3,E4,G2],[ , , ,]).
# M searches a victim and selects G. Now the cache stores ([A4,C3,E4,M1],[ , , ,]).
# G searches a victim and selects M. Now the cache stores ([A4,C3,E4,G1],[ , , ,]).
# Three G accesses are hits. Now the cache stores ([A4,C3,E4,G4],[ , , ,]).
# O searches a victim and selects C. Now the cache stores ([A4,O1,E4,G4],[ , , ,]).
# C searches a victim and selects O. Now the cache stores ([A4,C1,E4,G4],[ , , ,]).
from m5.objects.ReplacementPolicies import LFURP as rp


def python_generator(generator):
    yield generator.createLinear(60000, 0, 63, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 0, 63, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 128, 191, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 128, 191, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 128, 191, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 256, 319, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 384, 447, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 384, 447, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 512, 575, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 256, 319, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 256, 319, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 256, 319, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 256, 319, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 640, 703, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 0, 63, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 0, 63, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 0, 63, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 0, 63, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 768, 831, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 384, 447, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 384, 447, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 384, 447, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 384, 447, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 896, 959, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(60000, 128, 191, 64, 30000, 30000, 100, 0)
    yield generator.createLinear(30000, 0, 0, 0, 30000, 30000, 100, 0)

    yield generator.createExit(0)
