#ifndef __BOOTP_H_LOADED
#define __BOOTP_H_LOADED
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
 * $Id: bootp.h,v 1.1.1.1 1997/10/30 23:27:14 verghese Exp $
 */

/*
 * MODULE DESCRIPTION:
 *
 *     BOOTP protocol interface for EB64 monitor
 *
 * HISTORY:
 *
 * $Log: bootp.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:14  verghese
 * current 10/29/97
 *
 * Revision 1.5  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.4  1994/06/28  20:08:21  fdh
 * Modified filenames and build precedure to fit into a FAT filesystem.
 *
 * Revision 1.3  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.2  1994/01/21  09:45:59  rusling
 * Added #ifdef <filename>_H around the module.
 * Additionally, any included files are not ifdef'd
 * *before* they're included (ie address.h).
 *
 * Revision 1.1  1993/08/06  10:27:32  berent
 * Initial revision
 *
 *
 */

#include "address.h"


/* bootp_boot - performs a bootp transaction
 *
 * Arguments:
 *     device_no       - device on which request should be sent
 *     input_file_name - file name supplied by caller; or null for default. Null terminated;
 *     server_addr     - returned server IP address
 *     boot_file_name  - returned boot file name supplied by server
 *
 * Returned value:
 *     TRUE   - bootp successful; server_addr and boot_file_name valid;
 *     FALSE  - bootp failed; server_addr and boot_file_name invalid.
 */
extern int bootp_boot(int device_no,
                      char input_file_name[128],
                      ip_addr server_addr,
                      char bootp_file_name[128]);

/* bootp_init_module - initialise the module
 */
extern void bootp_init_module(void);

#endif /* __BOOTP_H_LOADED */

