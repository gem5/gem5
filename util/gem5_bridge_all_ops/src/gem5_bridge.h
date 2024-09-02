#ifndef GEM5_BRIDGE_H
#define GEM5_BRIDGE_H

/* Physical address for base of gem5ops MMIO range */
#ifndef GEM5OPS_BASE
#define GEM5OPS_BASE 0xffff0000 /* default, also used for x86 */
#endif

/* Size of gem5ops MMIO range */
#ifndef GEM5OPS_SIZE
#define GEM5OPS_SIZE 0x10000
#endif

/* MMIO virtual address for poking out of KVM */
extern void __iomem *gem5_bridge_mmio;

#endif /* GEM5_BRIDGE_H */
