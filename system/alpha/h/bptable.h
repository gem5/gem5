#ifndef __BPTABLE_H_LOADED
#define __BPTABLE_H_LOADED
/*****************************************************************************

       Copyright © 1993, 1994 Digital Equipment Corporation,
                       Maynard, Massachusetts.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its
documentation for any purpose and without fee is hereby granted, provided
that the copyright notice and this permission notice appear in all copies
of software and supporting documentation, and that the name of Digital not
be used in advertising or publicity pertaining to distribution of the software
without specific, written prior permission. Digital grants this permission
provided that you prominently mark, as not part of the original, any
modifications made to this software or documentation.

Digital Equipment Corporation disclaims all warranties and/or guarantees
with regard to this software, including all implied warranties of fitness for
a particular purpose and merchantability, and makes no representations
regarding the use of, or the results of the use of, the software and
documentation in terms of correctness, accuracy, reliability, currentness or
otherwise; and you rely on the software, documentation and results solely at
your own risk.

******************************************************************************/

/*
 * This file defines the interface to the breakpoint table.
 */

#include "lib.h"
#include "server_t.h"


#define BPTMAX 100

#define SUCCESS  1
#define TRUE     1
#define FALSE    0

#define BPTFULL  (-1)  /* breakpoint table is full */
#define BPTDUP   (-2)  /* duplicate breakpoint */
#define BPTNOBRK (-3)  /* no such breakpoint */
#define BPTINV   (-4)  /* this breakpoint number not in use */
#define BPTILL   (-5)  /* illegal breakpoint number */

/* bptinitialize - Initialize the breakpoint table.
 */
void bptinitialize(void);

/* bptinsert - add a breakpoint to the breakpoint table
 *
 * Arguments:
 *    addr      - address of breakpoint
 *    savedinst - the instruction at that address
 *
 * Return value:
 *    SUCCESS if successful, otherwise a negative error code.
 */
int bptinsert(address_value addr, instruction_value savedinst);

/* bptremote - remove a breakpoint from the breakpoint table
 *
 * Arguments:
 *     addr      - address of breakpoint
 *     savedinst - (returned) saved instruction
 * Return value:
 *    SUCCESS if successful, otherwise a negative error code.
 */
int bptremove(address_value addr, instruction_value *savedinst);

/* bptgetn - get a breakpoint by number
 *
 * Arguments:
 *     n         - breakpoint number
 *     addr      - (returned) address of breakpoint
 *     savedinst - (returned) saved instruction
 * Return value:
 *    SUCCESS if successful, otherwise a negative error code.
 */
int bptgetn(int n, address_value *addr, instruction_value *savedinst);

/* bptgeta - get a breakpoint by address
 *
 * Arguments:
 *     addr      - address of breakpoint
 *     savedinst - (returned) saved instruction
 * Return value:
 *    SUCCESS if successful, otherwise a negative error code.
 */
int bptgeta(address_value addr, instruction_value *savedinst);

/* bptisbreakat - checks whether there is a breakpoint at the given address
 *
 * Argument:
 *     addr      - address to be checked
 * Return value:
 *     TRUE if there is a breakpoint at that address; FALSE if not.
 */
int bptisbreakat(address_value addr);

/* bptfull - checks whether the breakpoint table is full
 *
 * Return value:
 *      TRUE if full; FALSE if not.
 */
int bptfull(void);

#endif /* __BPTABLE_H_LOADED */
