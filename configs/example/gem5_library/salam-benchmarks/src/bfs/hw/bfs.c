/*
 * Copyright (c) 2014-2015, the President and Fellows of Harvard College.
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains code derived from gem5-SALAM:
 * https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Harvard University nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
Implementations based on:
Harish and Narayanan.
"Accelerating large graph algorithms on the GPU using CUDA." HiPC, 2007.
Hong, Oguntebi, Olukotun.
"Efficient Parallel Graph Exploration on Multi-Core CPU and GPU." PACT, 2011.
*/

#include "hw_defines.h"

void
bfs(node_index_t starting_node)
{
    volatile node_t *nodes = (node_t *)NODES;
    volatile edge_t *edges = (edge_t *)EDGES;
    volatile level_t *level = (level_t *)LEVELS;
    volatile edge_index_t *level_counts = (edge_index_t *)LEVELCOUNTS;

    node_index_t n;
    edge_index_t e;
    level_t horizon;
    edge_index_t cnt;

    level[starting_node] = 0;
    level_counts[0] = 1;
#pragma nounroll
    for (horizon = 0; horizon < N_LEVELS; horizon++) {
        cnt = 0;
// Add unmarked neighbors of the current horizon to the next horizon
#pragma nounroll
        for (n = 0; n < N_NODES; n++) {
            if (level[n] == horizon) {
                edge_index_t tmp_begin = nodes[n].edge_begin;
                edge_index_t tmp_end = nodes[n].edge_end;
#pragma nounroll
                for (e = tmp_begin; e < tmp_end; e++) {
                    node_index_t tmp_dst = edges[e].dst;
                    level_t tmp_level = level[tmp_dst];

                    if (tmp_level == MAX_LEVEL) { // Unmarked
                        level[tmp_dst] = horizon + 1;
                        ++cnt;
                    }
                }
            }
        }
        if ((level_counts[horizon + 1] = cnt) == 0) {
            break;
        }
    }
}
