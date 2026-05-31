#include "hw_defines.h"

void
top(uint64_t nodes_addr, uint64_t edges_addr, uint64_t levels_addr,
    uint64_t level_counts_addr, node_index_t starting_node)
{
    // Define Device MMRs
    volatile uint8_t *BFSFlags = (uint8_t *)(BFS);
    volatile uint8_t *BFSConfig = (uint8_t *)(BFS + 1);
    volatile uint8_t *DmaFlags = (uint8_t *)(DMA_Flags);
    volatile uint64_t *DmaRdAddr = (uint64_t *)(DMA_RdAddr);
    volatile uint64_t *DmaWrAddr = (uint64_t *)(DMA_WrAddr);
    volatile uint32_t *DmaCopyLen = (uint32_t *)(DMA_CopyLen);

    // Transfer Input Matrices
    // Transfer Nodes
    *DmaRdAddr = nodes_addr;
    *DmaWrAddr = NODES;
    *DmaCopyLen = NODESSIZE;
    *DmaFlags = DEV_INIT;
    // Poll DMA for finish
    while ((*DmaFlags & DEV_INTR) != DEV_INTR)
        ;
    // Transfer Edges
    *DmaRdAddr = edges_addr;
    *DmaWrAddr = EDGES;
    *DmaCopyLen = EDGESSIZE;
    *DmaFlags = DEV_INIT;
    // Poll DMA for finish
    while ((*DmaFlags & DEV_INTR) != DEV_INTR)
        ;
    // Transfer Levels
    *DmaRdAddr = levels_addr;
    *DmaWrAddr = LEVELS;
    *DmaCopyLen = LEVELSIZE;
    *DmaFlags = DEV_INIT;
    // Poll DMA for finish
    while ((*DmaFlags & DEV_INTR) != DEV_INTR)
        ;

    // Start the accelerated function
    *(node_index_t *)BFSConfig = starting_node;
    *BFSFlags = DEV_INIT;
    // Poll function for finish
    while ((*BFSFlags & DEV_INTR) != DEV_INTR)
        ;

    // Transfer level_counts
    *DmaRdAddr = LEVELCOUNTS;
    *DmaWrAddr = level_counts_addr;
    *DmaCopyLen = LVLCNTSIZE;
    *DmaFlags = DEV_INIT;
    // Poll DMA for finish
    while ((*DmaFlags & DEV_INTR) != DEV_INTR)
        ;
    return;
}
