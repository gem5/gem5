#ifndef __ETHERNET_H_LOADED
#define __ETHERNET_H_LOADED
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
 * $Id: ethernet.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $
 */

/*
 * MODULE DESCRIPTION:
 *
 *     Ethernet protocol interface for EB64 monitor
 *
 * HISTORY:
 *
 * $Log: ethernet.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
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
 * Revision 1.2  1993/10/01  16:05:45  berent
 * made interface to ethernet_init public
 *
 * Revision 1.1  1993/08/06  10:30:23  berent
 * Initial revision
 *
 *
 */

#include "address.h"


/*
 * Prototype for packet processing functions
 *
 * Arguments to packet processing functions:
 *     protocol_data - protocol data passed to module when protocol registered
 *     device_no     - device on which packet was received;
 *     packet        - packet, with ethernet header removed;
 *     size          - size of packet;
 *     source        - source MAC address of packet;
 *     frame_buffer  - original buffer containing packet.
 *     balance_buffer- buffer returned to maintain buffer balance buffer.
 *
 * DESCRIPTION:
 *     The packet processing functions process a received protocol packet. To avoid
 *     buffer shortages they are always passed the buffer containing the packet in
 *     frame_buffer.  They must return either this buffer, or another buffer, in
 *     balance buffer. The ethernet module will never access frame_buffer after
 *     calling a packet processing function; and the protocol modules must not
 *     access the returned balance_buffer after returning from this function.
 *
 */
typedef void (* ethernet_packet_handler)(void * protocol_data,
                                int device_no,
                                unsigned char * packet,
                                int size,
                                mac_addr source,
                                unsigned char frame_buffer[ETHER_BUFFER_SIZE],
                                unsigned char ** balance_buffer);

/* ethernet_register_protocol - registers an ethernet protocol to be recognised in received frames by this module
 *
 * Arguments:
 *
 *    device_no             - device on which the protocol is to be registered
 *    protocol_id           - protocol id to be recognised by this module
 *    processing_function   - function to process received packets; if NULL cancels registration of protocol.
 *    protocol_data         - arbitrary data to be passed back when a packet is received
 *
 * Returned value:
 *    TRUE   - protocol registered
 *    FALSE  - unable to register protocol
 */
extern int ethernet_register_protocol(int device_no,
                                      ethernet_protocol_id protocol_id,
                                      ethernet_packet_handler processing_function,
                                      void * protocol_data);

/* ethernet_process_one_packet - reads and processes one packet from the ethernet
 *
 * Arguments:
 *
 *    device_no - device to poll, -1 means poll all devices
 *    input_buffer - free buffer usable by the ethernet module
 *    output_buffer - returned free buffer that may be used by the caller.
 *
 * Return value:
 *    TRUE - packet received and processed
 *    FALSE - nothing to receive.
 *
 * DESCRIPTION:
 *    This function checks whether there are any packets available to be processed on
 *    the ethernet device. If there are it reads and processes one packet. The caller
 *    must give it a buffer to work with (input_buffer) and will be returned a, possibly
 *    different, buffer (output_buffer) Note that this returned buffer does not contain the
 *    received packet, or any other meaningfull data; it is returned simply to ensure that caller
 *    does not run short of buffers.  By calling this function the caller gives up ownership
 *    of input_buffer. The caller must not access input_buffer after calling this function.
 *
 */
extern int ethernet_process_one_packet(int device_no,
                                       unsigned char input_buffer[ETHER_BUFFER_SIZE],
                                       unsigned char ** output_buffer );

/* ethernet_input - processes the packet passed to it as an ethernet frame
 *
 * Arguments:
 *
 *    device_no - device on which frame was received
 *    frame - the frame to be processed
 *    size - size of frame
 *    input_buffer - the buffer containing the frame
 *    balance_buffer - returned free buffer that may be used by the caller.
 *
 */
extern void ethernet_input(int device_no,
                           unsigned char * frame,
                           int size,
                           unsigned char input_buffer[ETHER_BUFFER_SIZE],
                           unsigned char ** balance_buffer );

/* ethernet_write - sends a packet on an ethernet
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
 *    This function writes an ethernet data packet to the device  To maintain the caller's
 *    pool of buffers the driver must give the caller an unused buffer (balance_buffer)
 *    in return for the buffer containing the frame to be transmitted.
 *
 *    If the send succeeds then frame_buffer must not be accessed by the caller after this
 *    call.  If the send can't be queued then frame_buffer will remain valid and
 *    the returned value of balance_buffer is undefined and must not be used.
 *
 */
extern int ethernet_write(int device_no,
                          unsigned char * packet,
                          mac_addr destination,
                          ethernet_protocol_id protocol_id,
                          int size,
                          unsigned char frame_buffer[ETHER_BUFFER_SIZE],
                          unsigned char ** balance_buffer);


/* ethernet_data_offset - returns the offset of the data in an ethernet frame
 *
 * Return value:
 *    Offset.
 */
extern int ethernet_data_offset(void);

/* ethernet_printpacket - print the contents of an ethernet frame
 *
 * Arguments:
 *    p          - the frame
 *    frame_size - its size
 */
extern void ethernet_printpacket(unsigned char *p, int frame_size);

/* ethernet_printaddress - utility to print an ethernet address
 *
 * Argument:
 *    address - MAC address to be printed
 */
extern void ethernet_printaddress(mac_addr address);

#endif /* __ETHERNET_H_LOADED */
