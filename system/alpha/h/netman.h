#ifndef __NETMAN_H_LOADED
#define __NETMAN_H_LOADED
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

/* "$Id: netman.h,v 1.1.1.1 1997/10/30 23:27:16 verghese Exp $" */

/*
 * This module provides functions for selecting which ethernet device is used and
 * for initialising it when neccessary.
 *
 * $Log: netman.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:16  verghese
 * current 10/29/97
 *
 * Revision 1.3  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.2  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.1  1993/08/11  10:18:33  berent
 * Initial revision
 *
 */

extern int monitor_ether_device;

/* netman_setup - initialise the networking modules and register the devices
 */
extern void netman_setup(void);

/* netman_set_monitor_device - set up the device to be used by the monitor
 *
 * Argument:
 *    device_no - The monitor device number.
 *
 * Return value:
 *    TRUE - successful
 *    FALSE - bad device number
 */
extern int netman_set_monitor_device(int device_no);

/* netman_start_monitor_device - start the monitor device and all protocol handlers for it
 */
extern void netman_start_monitor_device(void);

/* netman_monitor_device_started  - tells other modules whether the monitor device has been started
 *
 * Return value:
 *    TRUE - it has been initialised before
 *    FALSE - it has never been initialised
 */
extern int netman_monitor_device_started(void);

#endif /* __NETMAN_H_LOADED */
