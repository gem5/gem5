/*
Implementations based on:
Harish and Narayanan.
"Accelerating large graph algorithms on the GPU using CUDA." HiPC, 2007.
Hong, Oguntebi, Olukotun.
"Efficient Parallel Graph Exploration on Multi-Core CPU and GPU." PACT, 2011.
*/

#include "../defines.h"

typedef struct edge_t_struct
{
    node_index_t dst;
} edge_t;

typedef struct node_t_struct
{
    edge_index_t edge_begin;
    edge_index_t edge_end;
} node_t;

void bfs(node_t nodes[N_NODES], edge_t edges[N_EDGES],
         node_index_t starting_node, level_t level[N_NODES],
         edge_index_t level_counts[N_LEVELS]);
