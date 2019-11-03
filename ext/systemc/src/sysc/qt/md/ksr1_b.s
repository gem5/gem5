/*
 * QuickThreads -- Threads-building toolkit.
 * Copyright (c) 1993 by David Keppel
 *
 * Permission to use, copy, modify and distribute this software and
 * its documentation for any purpose and without fee is hereby
 * granted, provided that the above copyright notice and this notice
 * appear in all copies.  This software is provided as a
 * proof-of-concept and for demonstration purposes; there is no
 * representation about the suitability of this software for any
 * purpose.
 */

	.file   "ksr1_b.s"
	.def	.debug;	.endef

        .globl b_call_reg$TXT
	.globl b_call_reg
	.globl b_call_imm$TXT
	.globl b_call_imm
	.globl b_add$TXT
	.globl b_add
	.globl b_load$TXT
	.globl b_load


b_call_reg:
b_call_imm:
b_add:
b_load:
	.word	b_call_reg$TXT
	.word	qt_error
	.word	qt_error$TXT


b_call_reg$TXT:
b_call_imm$TXT:
b_add$TXT:
b_load$TXT:
	finop			; cxnop
	finop			; cxnop
	finop			; ld8 16(%cp),%c4
        finop			; ld8 8(%cp),%cp
	finop			; cxnop
	finop			; cxnop
        finop			; jsr %c4,0(%c4)
	finop			; cxnop
	finop			; cxnop

