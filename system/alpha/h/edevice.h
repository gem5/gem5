#ifndef __ETHER_DEVICE_H_LOADED
#define __ETHER_DEVICE_H_LOADED
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
 * $Id: edevice.h,v 1.1.1.1 1997/10/30 23:27:15 verghese Exp $
 */

/*
 * MODULE DESCRIPTION:
 *
 *     Ethernet device interface for EB64 monitor
 *
 * HISTORY:
 *
 * $Log: edevice.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:15  verghese
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
 * Revision 1.4  1994/01/21  09:45:59  rusling
 * Added #ifdef <filename>_H around the module.
 * Additionally, any included files are not ifdef'd
 * *before* they're included (ie address.h).
 *
 * Revision 1.3  1993/11/22  13:17:13  rusling
 * Merged with PCI/21040 changes.
 *
 * Revision 1.2  1993/10/01  16:05:27  berent
 * Added module initialisation function
 *
 * Revision 1.1  1993/08/06  10:30:01  berent
 * Initial revision
 *
 *
 */

#include "address.h"


/* ether_device_init - initialises an ethernet device
 *
 * Arguments:
 *
 *    device_no    - device number
 *
 * Returned value
 *    TRUE         - initialised succesfully
 *    FALSE        - error on initialisation
 */
extern int ether_device_init(int device_no);

/* ether_device_read - reads a frame from an ethernet device
 *
 * Arguments:
 *
 *    device_no      - device number
 *    balance_buffer - spare buffer to maintain buffer balance
 *    frame_buffer   - returned buffer containing frame
 *
 * Returned value:
 *
 *    positive     - size of frame read; frame copied into buffer if <= size.
 *       0         - nothing to read
 *    negative     - error on read
 *
 * DESCRIPTION:
 *    This function reads a frame from the device if there is one ready to be read. The
 *    frame read is returned in frame_buffer.  To maintain the driver's pool of buffers
 *    the caller must give the driver an unused buffer (balance_buffer) before the
 *    driver returns the frame.  Once this function has been called balance_buffer must
 *    not be used by the calling software.
 *
 *    On return frame_buffer will always point to a valid buffer.  If the read has failed
 *    (result <= 0) the contents of this buffer will not be valid.  Note that whether or
 *    not the read succeeds it is undefined whether the same buffer will be returned as
 *    frame_buffer as was passed down as balance_buffer.
 */
extern int ether_device_read(int device_no,
                             unsigned char balance_buffer[ETHER_BUFFER_SIZE],
                             unsigned char ** frame_buffer);

/* ether_device_write - queue a frame to be sent on an ethernet device
 *
 * Arguments:
 *
 *    device_no         - device number
 *    frame_buffer      - buffer containing frame
 *    frame             - the frame itself
 *    size              - size of frame
 *    balance_buffer    - returned buffer to maintain buffer balance
 *
 * Returned value:
 *
 *    TRUE -  succefully queued for sending
 *    FALSE - unable to send
 *
 * DESCRIPTION:
 *    This function writes a frame to the device  To maintain the caller's pool of buffers
 *    the driver must give the caller an unused buffer (balance_buffer) in return for
 *    the buffer containing the frame to be transmitted.
 *
 *    If the send succeeds then frame_buffer must not be accessed by the caller after this
 *    call.  If the send can't be queued then frame_buffer will remain valid and
 *    the returned value of balance_buffer is undefined and must not be used.
 *
 */
extern int ether_device_write(int device_no,
                              unsigned char * frame,
                              int size,
                              unsigned char frame_buffer[ETHER_BUFFER_SIZE],
                              unsigned char ** balance_buffer);

/* ether_device_flush - wait for all writes on an ethernet device to complete
 *
 * Argument:
 *
 *    device_no - device to be flushed
 */
extern int ether_device_flush(int device_no);

/* ether_device_print_stats - print device statistics
 *
 * Argument:
 *
 *     device_no - device number
 */
extern void ether_device_print_stats(int device_no);

/* ether_device_preg - print device registers
 *
 * Argument:
 *
 *     device_no - device number
 */
extern void ether_device_preg(int device_no);


/* ether_device_clear_interrupts - clear all interrupts from an ethernet device
 *
 * Argument:
 *
 *     device_no - device number, -1 means all devices
 */
extern void ether_device_clear_interrupts(int device_no);

/* ether_device_get_hw_address - gets the hardware mac address of the device
 *
 * Arguments:
 *
 *     device_no - device_number
 *     hw_address - returned hardware address
 */
extern void ether_device_get_hw_address(int device_no, mac_addr hw_address);

/* ether_device_next - get the next valid device number
 *
 * Argument:
 *
 *    previous_device_number - previous device number; or -1 if no previous device.
 *
 * Result:
 *
 *    next valid device number or -1 if no more devices.
 *
 * Description:
 *    The purpose of this function it to allow the device table to be scanned.
 *    If it called initially with -1 and then repeatedly called with its previous
 *    return value as its argument it will return each valid device number
 *    precisely once before returning -1.
 *
 * Notes:
 *    1) The device numbers will not neccesary be returned in assending order
 *    2) If previous_device_number is not the number of a valid device (or -1) the
 *        result is undefined.
 */
extern int ether_device_number(int previous_device_number);

/* ether_device_register - register a new ethernet device
 *
 * Arguments:
 *     device_no              - device_number
 *     init_func              - initialisation function
 *     read_func              - read function
 *     write_func             - write function
 *     flush_func             - flush function
 *     stats_func             - print statistics function
 *     preg_func              - print device registers function
 *     clear_interrupts_func  - clear interrupts function
 *     hw_addr_func           - get hardware address function
 *     description            - description of the device.
 */
extern void ether_device_register(int device_no,
                                  int (* init_func)(int device_no),
                                  int (* read_func)(int device_no,
                                                    unsigned char balance_buffer[ETHER_BUFFER_SIZE],
                                                    unsigned char ** frame_buffer),
                                  int (* write_func)(int device_no,
                                                     unsigned char * frame,
                                                     int size,
                                                     unsigned char frame_buffer[ETHER_BUFFER_SIZE],
                                                     unsigned char ** balance_buffer),
                                   int (* flush_func)(int device_no),
                                  void (* stats_func)(int device_no),
                                  void (* preg_func)(int device_no),
                                  void (* clear_interrupts_func)(int device_no),
                                  void (* hw_addr_func)(int device_no, mac_addr hw_address) ,
                                  char *description
                                  );

/* ether_device_init_module - initialise or reinitialise the ether_device module
 */
extern void ether_device_init_module(void);

extern void ether_device_show();

#endif /* __ETHER_DEVICE_H_LOADED */
