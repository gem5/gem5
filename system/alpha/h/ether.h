#ifndef __ETHER_H_LOADED
#define __ETHER_H_LOADED
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
 *  $Id: ether.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $;
 */

/*
 * $Log: ether.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
 * current 10/29/97
 *
 * Revision 1.4  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.3  1994/06/28  20:08:21  fdh
 * Modified filenames and build precedure to fit into a FAT filesystem.
 *
 * Revision 1.2  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.1  1994/06/20  12:54:45  fdh
 * Initial revision
 *
 */

#include "address.h"
#include "arp.h"
#include "base.h"
#include "bootp.h"
#include "buffer.h"
#include "edevice.h"
#include "ethernet.h"
#include "ip.h"
#include "isa_buff.h"
#include "net_buff.h"
#include "netman.h"
#include "tftp.h"
#include "udp.h"
#include "am79c960.h"
#include "DEC21040.h"


/* /ether/arp.c */
extern void arp_show();

/* /ether/edevice.c */
extern void ether_device_show();
extern void ether_device_clear_interrupts(int device_no);
extern int ethernet_process_one_packet(int device_no , ub input_buffer[1600] , ub * * output_buffer);

/* /ether/tftp.c */
extern void tftp_init_module(void);

/* /ether/bootp.c */
extern void bootp_init_module(void);

/* ladbx/server_init_module */
extern void ladbx_server_init_module(void);

/* /ether/ethernet.c */
extern int ethernet_process_one_packet(int, ub *, ub **);

/* /ether/eaddr.c */
extern void get_eaddr(void);
extern int ethernet_address(ui argc , ub * s);

/* /ether/netboot.c */
extern int netboot(int argc , char * file);

#endif /* __ETHER_H_LOADED */
