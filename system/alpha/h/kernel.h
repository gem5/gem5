#ifndef __KERNEL_H_LOADED
#define __KERNEL_H_LOADED
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
 *  $Id: kernel.h,v 1.1.1.1 1997/10/30 23:27:16 verghese Exp $;
 */

/*
 * Derived from EB64 version; history of EB64 version:
 *
 * $Log: kernel.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:16  verghese
 * current 10/29/97
 *
 * Revision 1.7  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.6  1994/06/28  20:08:21  fdh
 * Modified filenames and build precedure to fit into a FAT filesystem.
 *
 * Revision 1.5  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.4  1994/03/09  12:48:33  berent
 * Made NT compilable and tidied up
 *
 * Revision 1.4  1993/10/01  15:47:00  berent
 * Added saved_user_pc; used to avoid need for special ethernet PAL code
 *
 * Revision 1.3  1993/08/09  11:43:38  berent
 * Correct return types of some functions
 *
 * Revision 1.2  1993/06/08  22:32:06  berent
 * Changed to improve ladbx server communications
 *
 * Revision 1.1  1993/06/08  19:56:36  fdh
 * Initial revision
 *
 */

#include "system.h"
#include "server_t.h"


/* kload_implemented - check whether the kernel will load new processes.
 *
 * Returns TRUE if this kernel supports the loading of new processes,
 * FALSE if not.
 */
extern kload_implemented(void);

/* kload - load a new process.
 *
 * Arguments:
 *   name - file name of new process.
 *   argv - argument array for new process, NULL terminated.
 *   standardIn - file name of standard input.
 *   standardOut - file name of standard output.
 *   standardError - file name of standard error.
 *   loadAddress - address at which client expects process to be loaded; or
 *                 all bits 1 if unknown.
 *   startAddress - address at which client expects process to start executing.
 *                  ignored if the load address is not set.
 *
 *   The format and interpretation of file name arguments is kernel dependent except
 *   in that an empty string always means use the default (if any).  The
 *   standard input, standard output and standard error file names may be ignored by
 *   some kernels.  Kernels will only use the load and start addresses if they have
 *   direct control over where programs are loaded or started.
 *
 * Return Value:
 *   TRUE if successful, FALSE if not.
 */
extern int kload(char * name,
          char * argv[],
          char * standardIn,
          char * standardOut,
          char * standardError,
          address_value loadAddress,
          address_value startAddress);

/* kconnect_implemented - check whether the kernel will connect to existing processes.
 *
 * Returns TRUE if this kernel supports connecting to existing  processes,
 * FALSE if not.
 */

extern int kconnect_implemented(void);

/* kconnect - connect to an existing process
 *
 * Argument:
 *    pid - process id of process to which the kernel is to be connected. The interpretation
 *          of this value is kernel dependent.  It may be ignored by kernels running
 *          in a non multiprocessing environment.
 *
 * Return Value:
 *   TRUE if successful, FALSE if not.
 */
extern int kconnect(ui pid);

/* kkill_possible - checks whether this kernel can kill the current process.
 *
 * Return Value:
 *    TRUE if possible; false if not.
 */
extern int kkill_possible(void);

/* kkill - kill the current process.
 */
extern void kkill(void);

/* kdetach_possible - checks whether this kernel can detach from the current process
 *                    without killing it.
 *
 * Return Value:
 *    TRUE if possible; false if not.
 */
extern int kdetach_possible(void);

/* kdetach - detach from the current process without killing it. If possible the kernel will
 *           not remove any breakpoints or continue the process if it is stopped ; but
 *           there may be kernels on which detaching can only be implemented such that
 *           it does one or both of these.
 */
extern void kdetach(void);

/* kpid - return the pid of the current process.
 */
extern ui kpid(void);

/* kgo - run the current process until it hits a breakpoint or stops for some
 *       other reason.
 */
extern void kgo(void);

/* kstop - stop the current process as soon as possible.
 */
extern void kstop(void);

/* kaddressok - check whether an address is readable
 *
 * Argument:
 *    address - the address to be checked.
 *
 * Return value:
 *    TRUE if readable, FALSE if not.
 */
extern int kaddressok(address_value address);

/* kcexamine - get a value from memory.
 *
 * Argument:
 *    address - the address from which the value is to be fetched. Must be
 *              8 byte alligned.
 *
 * Return value:
 *    The 8 byte value at address.  If there is a breakpoint within this 8 byte
 *    region the value returned is the value that would be at that address if the
 *    breakpoint were removed.
 */
extern ul kcexamine(address_value address);


/* kcdeposit - deposit a value in memory.
 *
 * Arguments:
 *    address - the address at which the value is to be deposited. Must be
 *              8 byte alligned.
 *    value   - the 8 byte value to be deposited.  If there is a breakpoint within
 *              the 8 byte region the new value should not overwrite any breakpoint
 *              instruction; but instead should change the value that will be written
 *              back when any such instruction is removed.
 *
 * Return value:
 *    TRUE if successful. FALSE if the kernel was unable to deposit the data.
 */
extern int kcdeposit(address_value address, ul value);

/* kstep - step one instruction.  If there is a breakpoint at the current program counter
 *          the instruction that would be at that address if the breakpoint were removed is
 *          executed.
 */
extern void kstep(void);

/* kpc - get the current program counter.
 *
 * Return value:
 *    current program counter.
 */
extern address_value kpc(void);

/* ksetpc - update the program counter.
 *
 * Argument:
 *    address - new value of program counter.
 */
extern void ksetpc(address_value address);

/* kregister - get the contents of a register
 *
 * Argument:
 *    reg - the register number. If reg is in the range 0 to 31 the function fetches the
 *          contents of fixed point register reg.  If reg is the range 32 to 63 it
 *          fetches (as an 8 byte bit pattern) the contents of floating point register
 *          reg-32.
 *
 * Return value:
 *    The 8 byte bit pattern in the selected register.
 */
extern register_value kregister(int reg);

/* ksetreg - writes a bit pattern to a register
 *
 * Arguments:
 *     reg - the register number, as for kregister.
 *     value - the 8 byte bit pattern to be written.
 */
extern void ksetreg(int reg, register_value value);

/* kbreak - sets a breakpoint.
 *
 * Argument:
 *     address - the address at which a breakpoint is to be set. Must be 4 byte alligned.
 *
 * Return value:
 *     The result that should be sent back to the client.
 */
extern short int kbreak(address_value address);

/* kremovebreak - sets a breakpoint.
 *
 * Argument:
 *     address - the address of the breakpoint. Must be 4 byte alligned.
 *
 * Return value:
 *     The result that should be sent back to the client.
 */
extern short int kremovebreak(address_value address);

/* kpoll - get the state of the current process.
 *
 * Return value:
 *     PROCESS_STATE_PROCESS_RUNNING  - the process is running,
 *     PROCESS_STATE_PROCESS_AT_BREAK - the process has stopped at a breakpoint,
 *     PROCESS_STATE_PROCESS_SUSPENDED - the process has stopped elsewhere,
 *     PROCESS_STATE_PROCESS_TERMINATED - the process no longer exists,
 *     PROCESS_STATE_PROCESS_LOADING - the process is loading,
 *     PROCESS_STATE_LOAD_FAILED - the process failed to load.
 */
extern int kpoll(void);

#endif /* __KERNEL_H_LOADED */
