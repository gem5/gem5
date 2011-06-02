/*
 * Copyright (c) 1992, 1993 The Regents of the University of California
 * All rights reserved.
 *
 * This software was developed by the Computer Systems Engineering group
 * at Lawrence Berkeley Laboratory under DARPA contract BG 91-66 and
 * contributed to Berkeley.
 *
 * All advertising materials mentioning features or use of this software
 * must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Lawrence Berkeley Laboratories.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)remote-sl.h	8.1 (Berkeley) 6/11/93
 */

/*	$NetBSD: kgdb.h,v 1.4 1998/08/13 02:10:59 eeh Exp $	*/

#ifndef __KGDB_H__
#define __KGDB_H__

/*
 * Message types.
 */
#define KGDB_SIGNAL		'?'	// last sigal
#define KGDB_SET_BAUD		'b'	// set baud (deprecated)
#define KGDB_SET_BREAK		'B'	// set breakpoint (deprecated)
#define KGDB_CONT		'c'	// resume
#define KGDB_ASYNC_CONT		'C'	// continue with signal
#define KGDB_DEBUG		'd'	// toggle debug flags (deprecated)
#define KGDB_DETACH		'D'	// detach remote gdb
#define KGDB_REG_R		'g'	// read general registers
#define KGDB_REG_W		'G'	// write general registers
#define KGDB_SET_THREAD		'H'	// set thread
#define KGDB_CYCLE_STEP		'i'	// step a single cycle
#define KGDB_SIG_CYCLE_STEP	'I'	// signal then single cycle step
#define KGDB_KILL		'k'	// kill program
#define KGDB_MEM_R		'm'	// read memory
#define KGDB_MEM_W		'M'	// write memory
#define KGDB_READ_REG		'p'	// read register
#define KGDB_SET_REG		'P'	// write register
#define KGDB_QUERY_VAR		'q'	// query variable
#define KGDB_SET_VAR		'Q'	// set variable
#define KGDB_RESET		'r'	// reset system.  (Deprecated)
#define KGDB_STEP		's'	// step
#define KGDB_ASYNC_STEP		'S'	// signal and step
#define KGDB_THREAD_ALIVE	'T'	// find out if the thread is alive.
#define KGDB_TARGET_EXIT	'W'	// target exited
#define KGDB_BINARY_DLOAD	'X'	// write memory
#define KGDB_CLR_HW_BKPT	'z'	// remove breakpoint or watchpoint
#define KGDB_SET_HW_BKPT	'Z'	// insert breakpoint or watchpoint

/*
 * start of frame/end of frame
 */
#define KGDB_START	'$'
#define KGDB_END	'#'
#define KGDB_GOODP	'+'
#define KGDB_BADP	'-'

/*
 * Stuff for KGDB.
 */
#define	KGDB_NUMREGS	66	/* from tm-alpha.h, NUM_REGS */
#define	KGDB_REG_V0	0
#define	KGDB_REG_T0	1
#define	KGDB_REG_T1	2
#define	KGDB_REG_T2	3
#define	KGDB_REG_T3	4
#define	KGDB_REG_T4	5
#define	KGDB_REG_T5	6
#define	KGDB_REG_T6	7
#define	KGDB_REG_T7	8
#define	KGDB_REG_S0	9
#define	KGDB_REG_S1	10
#define	KGDB_REG_S2	11
#define	KGDB_REG_S3	12
#define	KGDB_REG_S4	13
#define	KGDB_REG_S5	14
#define	KGDB_REG_S6	15	/* FP */
#define	KGDB_REG_A0	16
#define	KGDB_REG_A1	17
#define	KGDB_REG_A2	18
#define	KGDB_REG_A3	19
#define	KGDB_REG_A4	20
#define	KGDB_REG_A5	21
#define	KGDB_REG_T8	22
#define	KGDB_REG_T9	23
#define	KGDB_REG_T10	24
#define	KGDB_REG_T11	25
#define	KGDB_REG_RA	26
#define	KGDB_REG_T12	27
#define	KGDB_REG_AT	28
#define	KGDB_REG_GP	29
#define	KGDB_REG_SP	30
#define	KGDB_REG_ZERO	31
#define	KGDB_REG_F0	32
#define	KGDB_REG_F1	33
#define	KGDB_REG_F2	34
#define	KGDB_REG_F3	35
#define	KGDB_REG_F4	36
#define	KGDB_REG_F5	37
#define	KGDB_REG_F6	38
#define	KGDB_REG_F7	39
#define	KGDB_REG_F8	40
#define	KGDB_REG_F9	41
#define	KGDB_REG_F10	42
#define	KGDB_REG_F11	43
#define	KGDB_REG_F12	44
#define	KGDB_REG_F13	45
#define	KGDB_REG_F14	46
#define	KGDB_REG_F15	47
#define	KGDB_REG_F16	48
#define	KGDB_REG_F17	49
#define	KGDB_REG_F18	50
#define	KGDB_REG_F19	51
#define	KGDB_REG_F20	52
#define	KGDB_REG_F21	53
#define	KGDB_REG_F22	54
#define	KGDB_REG_F23	55
#define	KGDB_REG_F24	56
#define	KGDB_REG_F25	57
#define	KGDB_REG_F26	58
#define	KGDB_REG_F27	59
#define	KGDB_REG_F28	60
#define	KGDB_REG_F29	61
#define	KGDB_REG_F30	62
#define	KGDB_REG_F31	63
#define	KGDB_REG_PC	64
#define	KGDB_REG_VFP	65

/* Too much?  Must be large enough for register transfer. */
#define	KGDB_BUFLEN	1024

#endif /* __KGDB_H__ */
