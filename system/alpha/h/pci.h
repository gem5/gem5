#ifndef __PCI_H_LOADED
#define __PCI_H_LOADED
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
 *  $Id:
 *
 *  This file contains all of the PCI specific definitions required for
 *  the PCI bus support within the debug monitor.
 */

/*
 * $Log: pci.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:17  verghese
 * current 10/29/97
 *
 * Revision 1.1  1996/04/15 16:26:54  berc
 * Initial revision
 *
 * Revision 1.11  1994/11/01  11:30:18  rusling
 * Added PCI-PCI bridge support.
 *
 * Revision 1.10  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * $Log: pci.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:17  verghese
 * current 10/29/97
 *
 * Revision 1.1  1996/04/15 16:26:54  berc
 * Initial revision
 *
 * Revision 1.11  1994/11/01  11:30:18  rusling
 * Added PCI-PCI bridge support.
 *
 * Revision 1.9  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.8  1994/06/19  15:48:04  fdh
 * Changed PCI_H to __PCI_H_LOADED
 *
 * Revision 1.7  1994/06/17  19:34:01  fdh
 * Clean-up...
 *
 * Revision 1.6  1994/01/19  10:22:28  rusling
 * Ported to ALpha Windows NT.
 *
 * Revision 1.5  1993/11/29  14:58:42  rusling
 * Added PCI address checking and conversion routines,
 * PCIMapAddress() and PCIValidAddress().
 *
 * Revision 1.4  1993/11/24  15:23:29  rusling
 * Added PCI_CFG_REG_VENDOR_DEVICE for offset to
 * first longword of the PCI configuration header.
 *
 * Revision 1.3  1993/11/23  10:43:50  rusling
 * Remove per system information.
 *
 * Revision 1.2  1993/11/22  13:17:13  rusling
 * Merged with PCI/21040 changes.
 *
 * Revision 1.1  1993/11/22  12:16:56  rusling
 * Initial revision
 *
 *
 */

#ifdef NEEDPCI


/*
 *  Define the PCI device data structure.
 */
typedef struct PCIDevice {
    struct PCIBus *parent;		/* parent bus */
    struct PCIDevice *next;		/* in device chain */
    struct PCIDevice *sibling;		/* for this bus */
    ui slot;
    uw vendor;
    uw device;
    ui class;

    ui PCI_IO_Reg;
    ui PCI_IO_Base;
    ui PCI_IO_Size;
    ui PCI_Mem_Reg;
    ui PCI_Mem_Base;
    ui PCI_Mem_Size;

    void (*print)(struct PCIDevice *device);
    struct {
        ui IO : 1;
        ui MEM : 1;
        ui bridge : 1;
        ui allocated : 1;
    } flags;
} PCIDevice_t;

typedef struct PCIBus {
    struct PCIBus *parent;
    struct PCIBus *next;
    struct PCIDevice *bridge;
    struct PCIDevice *devices;
    struct PCIBus *children;

    ui PCI_IO_Reg;
    ui PCI_IO_Base;
    ui PCI_IO_Size;
    ui PCI_Mem_Reg;
    ui PCI_Mem_Base;
    ui PCI_Mem_Size;

    unsigned char number;
    unsigned char primary;
    unsigned char secondary;
    unsigned char subordinate;
} PCIBus_t;


/*
 *  Define some macros for getting at fields in the
 *  PCIDevice_t and PCIBus_t typedefs.
 */
#define _PCI_IO_Base(device)  (device)->PCI_IO_Base
#define _PCI_IO_Size(device)  (device)->PCI_IO_Size
#define _PCI_Mem_Base(device) (device)->PCI_Mem_Base
#define _PCI_Mem_Size(device) (device)->PCI_Mem_Size
#define _PCI_Slot(device)     (device)->slot
#define _PCI_Print(device)    (device)->print
#define _PCI_Bus(device)      (device)->parent->number

/*
 *  Declare some maximums.
 */
#define PCI_MAX_DEVICES   	6	/* bits 31:11 of type 0 */
#define PCI_21050_MAX_DEVICES 	16	/* bits 14:11 of type 1 */
#define PCI_MAX_BRIDGES	   	6

/*
 *  The PCI configuration registers.
 */
#define PCI_CFG_REG_VENDOR_DEVICE         0x0
#define PCI_CFG_REG_STATUS_COMMAND        0x4
                                        /* LONG */
#define PCI_CFG_REG_REVISION_ID           0x8
                                        /* BYTE */
#define PCI_CFG_REG_CLASS_CODE		  0x9
#define PCI_CFG_REG_LATENCY_TIMER         0xD
                                        /* BYTE */
#define PCI_CFG_REG_BAR0                  0x10
                                        /* Long */
#define PCI_CFG_REG_BRIDGE_PRIMARY	  0x18
#define PCI_CFG_REG_BRIDGE_SECONDARY	  0x19
#define PCI_CFG_REG_BRIDGE_SUBORDINATE	  0x1a

#define PCI_CFG_REG_EROM_BASE             0x30
                                        /* Long */

/*
 *  PCI Vendors and devices.
 */
#define DIGITAL                        0x1011
#define DECCHIP_21040                  0x0002
#define DECCHIP_21030                  0x0004

#define PCI_BRIDGE_CLASS		       0x060400
/*
 *  Declare all of the externally visible routines in pci.c
 */
extern void PCIShow(void);
extern ui PCIValidAddress(ub * address);
extern ui PCIMapAddress(ub * address);
extern void PCIInit(void);
extern PCIDevice_t *PCIDeviceFind(uw vendor, uw device);
extern PCIDevice_t *PCIDeviceFindNext(PCIDevice_t *device);
extern void PCISetDeviceConfig(PCIDevice_t *device);

#endif /* NEEDPCI */
#endif /* __PCI_H_LOADED */
