#include <machine/asm.h>
#include <regdef.h>
		
#define m5_op 0x01
#define arm_func 0x00
#define quiesce_func 0x01
#define ivlb_func 0x10
#define ivle_func 0x11
#define m5exit_func 0x20
#define initparam_func 0x30
#define resetstats_func 0x40
	
#define INST(op, ra, rb, func) \
	.long (((op) << 26) | ((ra) << 21) | ((rb) << 16) | (func))
	
#define	ARM(reg) INST(m5_op, reg, 0, arm_func)
#define QUIESCE() INST(m5_op, 0, 0, quiesce_func)
#define IVLB(reg) INST(m5_op, reg, 0, ivlb_func)
#define IVLE(reg) INST(m5_op, reg, 0, ivle_func)
#define M5_EXIT() INST(m5_op, 0, 0, m5exit_func)
#define INITPARAM(reg) INST(m5_op, reg, 0, initparam_func)
#define RESETSTATS() INST(m5_op, 0, 0, resetstats_func)

	.set noreorder

	.align 4
LEAF(arm)
	ARM(16)
	RET
END(arm)

	.align 4
LEAF(quiesce)
	QUIESCE()
	RET
END(quiesce)

	.align 4
LEAF(ivlb)
	IVLB(16)
	RET
END(ivlb)

	.align 4
LEAF(ivle)
	IVLE(16)
	RET
END(ivle)

	.align 4
LEAF(m5exit)
	M5_EXIT()
	RET
END(m5exit)

    .align 4
LEAF(initparam)
    INITPARAM(0)
    RET
END(initparam)

    .align 4
LEAF(resetstats)
    RESETSTATS()
    RET
END(resetstats)