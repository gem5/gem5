#ifndef __IP_H_LOADED
#define __IP_H_LOADED
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
 * $Id: ip.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $
 */

/*
 * MODULE DESCRIPTION:
 *
 *     IP protocol interface for EB64 monitor
 *
 * HISTORY:
 *
 * $Log: ip.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
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
 * Revision 1.1  1993/08/06  10:30:54  berent
 * Initial revision
 *
 *
 */

#include "address.h"


/* ip_init - initialise ip protocol for an ethernet device
 *
 * Argument:
 *    device_no - device for which IP is to be initialised.
 */
extern void ip_init(int device_no);

/* ip_set_device_addr - tells ip a device's ip address.
 *
 * Arguments:
 *     device_no - device number
 *     ip        - ip address
 */
extern void ip_set_device_addr(int device_no,ip_addr ip);

/*
 * Prototype for packet processing functions
 *
 * Arguments to packet processing functions:
 *     protocol_data - protocol data passed to module when protocol registered
 *     device_no     - device on which packet was received;
 *     packet        - packet, with IP header removed;
 *     size          - size of packet;
 *     source        - source IP address of packet;
 *     destination   - IP address to which the packet was addressed; this is needed by UDP.
 *     frame_buffer  - original buffer containing packet.
 *     balance_buffer- buffer returned to maintain buffer balance.
 *
 * DESCRIPTION:
 *     The packet processing functions process a received protocol packet. To avoid
 *     buffer shortages they are always passed the buffer containing the packet in
 *     frame_buffer.  They must return either this buffer, or another buffer, in
 *     balance buffer. The ip module will never access frame_buffer after
 *     calling a packet processing function; and the protocol modules must not
 *     access the returned balance_buffer after returning from this function.
 *
 */
typedef void (* ip_packet_handler)(void * protocol_data,
                                   int device_no,
                                   unsigned char * packet,
                                   int size,
                                   ip_addr source,
                                   ip_addr destination,
                                   unsigned char frame_buffer[ETHER_BUFFER_SIZE],
                                   unsigned char ** balance_buffer);

/* ip_register_protocol - registers an IP protocol to be recognised in received frames by this module
 *
 * Arguments:
 *
 *    device_no             - device number on which this protocol can be received
 *    protocol_id           - protocol id to be recognised by this module
 *    processing_function   - function to process received packets; if NULL cancels registration of protocol.
 *    protocol_data         - arbitrary data to be passed back when a packet is received
 *
 * Returned value:
 *    TRUE   - protocol registered
 *    FALSE  - unable to register protocol
 */
extern int ip_register_protocol(int device_no,
                                ip_protocol_id protocol_id,
                                ip_packet_handler processing_function,
                                void * protocol_data);

/* ip_write - sends a packet on an IP network
 *
 * Arguments:
 *
 *    device_no - device on which to send the packet
 *    packet    - pointer to data packet to be sent
 *    destination - destination address for packet
 *    protocol_id - protocol id for packet
 *    size        - size of packet
 *    frame_buffer - buffer containing packet
 *    balance_buffer - buffer returned to maintain buffer balance
 *
 * Return values:
 *
 *    TRUE  - send successfull
 *    FALSE - Error occured
 *
 * DESCRIPTION:
 *    This function writes an ip data packet to the device  To maintain the caller's
 *    pool of buffers the driver must give the caller an unused buffer (balance_buffer)
 *    in return for the buffer containing the frame to be transmitted.
 *
 *    If the send succeeds then frame_buffer must not be accessed by the caller after this
 *    call.  If the send can't be queued then frame_buffer will remain valid and
 *    the returned value of balance_buffer is undefined and must not be used.
 *
 */
extern int ip_write(int device_no,
                    unsigned char * packet,
                    ip_addr destination,
                    ip_protocol_id protocol_id,
                    int size,
                    unsigned char frame_buffer[ETHER_BUFFER_SIZE],
                    unsigned char ** balance_buffer);


/* ip_data_offset - returns the offset at which ip data should be placed in a new ethernet frame
 *
 * Return value:
 *    Offset.
 */
extern int ip_data_offset(void);

/* ip_get_address - gets the address that will be used as the source address of transmitted packets.
 *
 * Arguments:
 *     device_no - device number of device on which packet is to be sent.
 *     ip        - returned ip address
 *
 * Note:
 *     This function exists so that UDP can calculate header checksums.
 */
extern void ip_get_device_address(int device_no, ip_addr ip);

/* ip_printaddress - utility to print an ethernet address
 *
 * Argument:
 *    address - IP address to be printed
 */
extern void ip_printaddress(mac_addr address);

#endif /* __IP_H_LOADED */
