#ifndef __ARP_H_LOADED
#define __ARP_H_LOADED
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
 * $Id: arp.h,v 1.1.1.1 1997/10/30 23:27:13 verghese Exp $
 */

/*
 * MODULE DESCRIPTION:
 *
 *     Arp interface for EB64 monitor
 *
 * HISTORY:
 *
 * $Log: arp.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:13  verghese
 * current 10/29/97
 *
 * Revision 1.6  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.5  1994/06/28  20:08:21  fdh
 * Modified filenames and build precedure to fit into a FAT filesystem.
 *
 * Revision 1.4  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.3  1994/01/21  09:45:59  rusling
 * Added #ifdef <filename>_H around the module.
 * Additionally, any included files are not ifdef'd
 * *before* they're included (ie address.h).
 *
 * Revision 1.2  1994/01/20  15:46:47  rusling
 * Added new routine, arp_show(), to print out
 * the contents of the current arp table.
 *
 * Revision 1.1  1993/08/06  10:25:46  berent
 * Initial revision
 *
 *
 */

#include "address.h"


/* arp_init - initialises the arp module; and registers it with the
 * ethernet protocol handler.
 *
 * Argument:
 *    device_no - device number on which arp should be registered.
 */
extern void arp_init(int device_no);

/* arp_set_device_addr - tells arp a device's ip address.
 *
 * Arguments:
 *     device_no - device number
 *     ip        - ip address
 */
extern void arp_set_device_addr(int device_no,ip_addr ip);

/* arp_resolve - converts an ip address into an ethernet address
 *
 * Arguments:
 *     device_no - device on which the message is to be sent
 *     ip        - ip address
 *     mac       - returned ethernet MAC address
 *
 * Returned value:
 *     TRUE      - address resolved; eaddr valid
 *     FALSE     - Unable to resolve address.
 */
extern int arp_resolve(int device_no, ip_addr ip, mac_addr mac);

/* arp_init_module - initialise the module
 */
extern void arp_init_module(void);
/*
 * arp_show - show all arp entries
 */
extern void arp_show(void);

#endif /* __ARP_H_LOADED */
