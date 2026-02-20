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
