#ifndef __ADDRESSES_H_LOADED
#define __ADDRESSES_H_LOADED
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
 * $Id: address.h,v 1.1.1.1 1997/10/30 23:27:13 verghese Exp $
 */

/*
 * MODULE DESCRIPTION:
 *
 *     Network addresses, and other parameters to service interface, for EB64 monitor
 *
 * HISTORY:
 *
 * $Log: address.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:13  verghese
 * current 10/29/97
 *
 * Revision 1.7  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.6  1994/06/28  20:08:21  fdh
 * Modified filenames and build precedure to fit into a FAT filesystem.
 *
 * Revision 1.5  1994/06/22  15:10:11  rusling
 * Fixed up WNT compile warnings.
 *
 * Revision 1.4  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.3  1994/01/21  09:45:59  rusling
 * Added #ifdef <filename>_H around the module.
 * Additionally, any included files are not ifdef'd
 * *before* they're included (ie address.h).
 *
 * Revision 1.2  1994/01/20  17:56:30  rusling
 * Added #ifdef ADDRESSES_H around this include file.
 *
 * Revision 1.1  1993/08/06  10:26:13  berent
 * Initial revision
 *
 *
 */


/* Note that all the address and protocol types in this file are held in network byte order */

/* MAC address */

#define MAC_ADDRESS_SIZE 6

typedef unsigned char mac_addr[MAC_ADDRESS_SIZE];

/* Broadcast MAC address */
static mac_addr broadcast_mac_address = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* Ethernet protocol ids */

#define ETHERNET_PROTOCOL_ID_SIZE 2

typedef unsigned char ethernet_protocol_id[ETHERNET_PROTOCOL_ID_SIZE];

/* Standard ethernet protocol ids - from RFC 1340 */
static ethernet_protocol_id ether_protocol_ip = {0x08, 0x00};
static ethernet_protocol_id ether_protocol_arp = {0x08, 0x06};

/* IP addresses */

#define IP_ADDRESS_SIZE 4

typedef unsigned char ip_addr[IP_ADDRESS_SIZE];

static ip_addr own_ip_addr_unknown_address = {0,0,0,0};
static ip_addr local_ip_broadcast_address = {0xFF,0xFF,0xFF,0xFF};

/* IP protocol ids */

typedef unsigned char ip_protocol_id;

/* UDP port ids */

#define UDP_PORT_SIZE 2

typedef unsigned char udp_port[UDP_PORT_SIZE];

/* Well known UDP port numbers - mainly from RFC 1340 */

static udp_port udp_port_remote_debug_connect={410 >> 8, 410 & 0xff}; /* New assignment by IANA  -
                                                                        Ladebug remote debug protocol = 410 */

/* Special for compatibility with existing remote debug implementation */
static udp_port udp_port_old_debug={0x54, 0x07};

#define ETHER_BUFFER_SIZE 1600

#endif /* __ADDRESSES_H_LOADED */
