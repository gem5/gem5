/*
Copyright 1993 Hewlett-Packard Development Company, L.P.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef __EB164_H_LOADED
#define __EB164_H_LOADED

/*
 *  $Id: eb164.h,v 1.1.1.1 1997/10/30 23:27:14 verghese Exp $;
 */

/*
 * $Log: eb164.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:14  verghese
 * current 10/29/97
 *
 * Revision 1.3  1995/02/10  02:21:36  fdh
 * Corrected EB164 banner.
 *
 * Revision 1.2  1994/12/07  21:24:58  cruz
 * Added constant defining the maximum size of the BCache
 *
 * Revision 1.1  1994/11/23  19:47:30  cruz
 * Initial revision
 *
 *
 */

#ifdef EB164

#define BANNER "DECchip 21164 Evaluation Board (EB164) Debug Monitor"
#define PROMPT "EB164> "

/****************************************************************************
 * Basic                                                                    *
 ****************************************************************************/

#define NEEDPCI
#define NEEDDEBUGGER
#define NEEDFLOPPY

/****************************************************************************
 * CIA (Part of CIA) definitions	                                    *
 ****************************************************************************/
#include "cia.h"

/****************************************************************************
 * Plug in cards, what does it have?                                        *
 ****************************************************************************/
/*
 * The Allied Telesis card is an ISA card based on the am79c960
 * chip.
 */
#define ALLIED_TELESIS_AT_1500T

/*
 *  The DEC Etherworks ISA card is a LANCE like device.
 */
#define DEC_ETHERWORKS

#define NEED21040

/****************************************************************************
 * ISA Address Space                                                        *
 ****************************************************************************/

#define RTCBASE                             0x70
#define LEDPORT                             0x80  /* JDR Microdevices P.O.S.T. Diagnostics Card. */


/*
 * Allied telesis is optional.  We treat it like an optional
 * am79c960 as far as the driver is concerned, see /h/am79c960.h
 * for more definitions and /ether/am79c960_device.c for the device
 * driver.
 */
#ifdef ALLIED_TELESIS_AT_1500T
#define OPTIONAL_AM79C960		/* warning only define this once - kmc	*/
#define NEEDAM79C960			/* same for this 			*/
#define OPTIONAL_AM79C960_BASE              0x360
#define OPTIONAL_AM79C960_INT               9

/*
 * This parameter determines the
 * DMA Mask for the DMA2 controller.
 *       Mask     Channel
 *       ----     -------
 *        1          5
 *        2          6
 *        3          7
 *        4          8
 */
#define OPTIONAL_AM79C960_DMA               1 /* Selects DMA Channel 5 */
#endif

#ifdef DEC_ETHERWORKS
#define DEC_ETHERWORKS_BASE                 0x300
#endif

#ifdef DEC_ETHERWORKS
#define DEC_ETHERWORKS_BASE                 0x300
#endif


/****************************************************************************
 * System Address Space                                                     *
 ****************************************************************************/

#define MINIMUM_SYSTEM_MEMORY		    0x1000000 /* 16MB */
#define MAXIMUM_SYSTEM_CACHE     	    0x800000  /* 8MB */
#define DMA_BUFF_BASE                       0x100000


/****************************************************************************
 * PCI I/O Address Space                                                    *
 ****************************************************************************/
/*
 * Definitions for the windows mapping PCI addresses into
 * system addresses
 *
 * 16 megabyte window starting at CPU address = 0.
 */

#define PCI_BASE_1_USED                     1
#define PCI_BASE_1                          0x000100000
#define PCI_MASK_1                          0x000000000
#define PCI_TBASE_1                         0x000100000

#define PCI_BASE_2_USED                     0
#define PCI_BASE_2                          0x000000000
#define PCI_MASK_2                          0x000000000
#define PCI_TBASE_2                         0x000000000

#define PCI_BASE_3_USED                     0
#define PCI_BASE_3                          0x000000000
#define PCI_MASK_3                          0x000000000
#define PCI_TBASE_3                         0x000000000

#define PCI_BASE_4_USED                     0
#define PCI_BASE_4                          0x000000000
#define PCI_MASK_4                          0x00000000
#define PCI_TBASE_4                         0x000000000

/*
 * Each mask translates to a number of these units.  For
 * APECS this unit is 1Kbyte.
 */
#define PCI_MASK_UNIT                       0x100000

/*
 * Where do we start allocating addresses from in PCI I/O space?
 */
#define PCI_IO_BASE                         0xB000

/*
 * ROM definitions.
 */
#define NEEDFLASHMEMORY
#define INTEL_28F008SA
#define ROMBASE 0xFFF80000
#define ROMSIZE 0x100000
#define ROMINC 0x1

#endif /* EB164 */
#endif /* __EB164_H_LOADED */

