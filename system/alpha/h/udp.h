#ifndef UDP_H_LOADED
#define UDP_H_LOADED
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
 * $Id: udp.h,v 1.1.1.1 1997/10/30 23:27:18 verghese Exp $
 */

/*
 * MODULE DESCRIPTION:
 *
 *     UDP protocol interface for EB64 monitor
 *
 * HISTORY:
 *
 * $Log: udp.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:18  verghese
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
 * Revision 1.1  1993/08/06  10:46:45  berent
 * Initial revision
 *
 *
 */

#include "address.h"


/* udp_init - initialise udp protocol module
 *
 * Argument:
 *    device_no - device number on which udp should be enabled.
 */
extern void udp_init(int device_no);

/*
 * Prototype for packet processing functions
 *
 * Arguments to packet processing functions:
 *     protocol_data - protocol data passed to module when protocol registered
 *     device_no     - device on which packet was received;
 *     packet        - packet, with UDP header removed;
 *     size          - size of packet;
 *     source_port   - source udp port of packet;
 *     source_address- source ip address of packet;
 *     frame_buffer  - original buffer containing packet;
 *     balance_buffer- buffer returned to maintain buffer balance.
 *
 * DESCRIPTION:
 *     The packet processing functions process a received protocol packet. To avoid
 *     buffer shortages they are always passed the buffer containing the packet in
 *     frame_buffer.  They must return either this buffer, or another buffer, in
 *     balance buffer. The udp module will never access frame_buffer after
 *     calling a packet processing function; and the protocol modules must not
 *     access the returned balance_buffer after returning from this function.
 *
 */
typedef void (* udp_packet_handler)(void * protocol_data,
                                   int device_no,
                                   unsigned char * packet,
                                   int size,
                                   udp_port source_port,
                                   ip_addr source_address,
                                   unsigned char frame_buffer[ETHER_BUFFER_SIZE],
                                   unsigned char ** balance_buffer);

/* udp_register_well_known_port - tells the udp module what to do with packets received for a particular
 *                                well known port number.
 *
 * Arguments:
 *
 *    device_no             - device number on which port should be registered.
 *    port                  - port to register.
 *    processing_function   - function to process received packets; if NULL cancels registration of protocol.
 *    protocol_data         - arbitrary data to be passed back when a packet is received
 *
 * Returned value:
 *    TRUE   - protocol registered
 *    FALSE  - unable to register protocol
 */
extern int udp_register_well_known_port(int device_no,
                                        udp_port port,
                                        udp_packet_handler processing_function,
                                        void * protocol_data);

/* udp_create_new_port - tells the udp module to create a new port; and what to do with data received
 *                       on that port.  The created port number will be outside the well known port
 *                       number range.
 *
 * Arguments:
 *
 *    device_no             - device number on which port should be created.
 *    processing_function   - function to process received packets
 *    protocol_data         - arbitrary data to be passed back when a packet is received
 *    port                  - returned port number
 *
 * Returned value:
 *    TRUE   - protocol registered
 *    FALSE  - unable to register protocol
 */
extern int udp_create_port(int device_no,
                           udp_packet_handler processing_function,
                           void * protocol_data,
                           udp_port port);

/* udp_delete_port - deletes a previously created port
 *
 * Argument:
 *    device_no   - device number on which port is registered.
 *    port        - port to be deleted.
 */
extern void udp_delete_port(int device_no, udp_port port);

/* udp_write - sends a packet on a UDP port
 *
 * Arguments:
 *
 *    device_no        - device on which to send the packet
 *    packet           - pointer to data packet to be sent
 *    destination_port - UDP port to which the packet should be sent
 *    destination_addr - IP address to which the packet should be sent
 *    source_port      - UDP source port
 *    size             - size of packet
 *    frame_buffer     - buffer containing packet
 *    balance_buffer   - buffer returned to maintain buffer balance
 *
 * Return values:
 *
 *    TRUE  - send successfull
 *    FALSE - Error occured
 *
 * DESCRIPTION:
 *    This function writes an udp data packet to the device.  To maintain the caller's
 *    pool of buffers the driver must give the caller an unused buffer (balance_buffer)
 *    in return for the buffer containing the frame to be transmitted.
 *
 *    If the send succeeds then frame_buffer must not be accessed by the caller after this
 *    call.  If the send can't be queued then frame_buffer will remain valid and
 *    the returned value of balance_buffer is undefined and must not be used.
 *
 */
extern int udp_write(int device_no,
                     unsigned char * packet,
                     udp_port destination_port,
                     ip_addr destination_addr,
                     udp_port source_port,
                     int size,
                     unsigned char frame_buffer[ETHER_BUFFER_SIZE],
                     unsigned char ** balance_buffer);


/* udp_data_offset - returns the offset at which udp data should be placed in a new ethernet frame
 *
 * Return value:
 *    Offset.
 */
extern int udp_data_offset(void);

#endif /* UDP_H_LOADED */
