/* $Id: tlaserreg.h,v 1.3 2002/10/27 14:28:17 binkertn Exp $ */
/*
 * Copyright (C) 1998 by the Board of Trustees
 *    of Leland Stanford Junior University.
 * Copyright (C) 1998 Digital Equipment Corporation
 *
 * This file is part of the SimOS distribution.
 * See LICENSE file for terms of the license.
 *
 */

#ifndef __TLASERREG_H__
#define __TLASERREG_H__

///////////////////////////////////////////////////////////////////////
//
// This file is also included to build the palcode
//

// Common module defines
#define TLDEV_REG		0x00
#define TLBER_REG		0x40
#define TLCNR_REG		0x80
#define TLFADR0_REG		0x600
#define TLFADR1_REG		0x640
#define TLESR0_REG		0x680
#define TLESR1_REG		0x6c0
#define TLESR2_REG		0x700
#define TLESR3_REG		0x740

// I/O Module defines
#define TLMMR0_REG		0x200
#define TLMMR1_REG		0x240
#define TLMMR2_REG		0x280
#define TLMMR3_REG		0x2c0
#define TLMMR4_REG		0x300
#define TLMMR5_REG		0x340
#define TLMMR6_REG		0x380
#define TLMMR7_REG		0x3c0
#define TLCPUMASK_REG		0x0b00
#define TLILID0_REG		0xa00
#define TLILID1_REG		0xa40
#define TLILID2_REG		0xa80
#define TLILID3_REG		0xac0
#define TLMBPR_REG		0xc00
#define ICCMSR_REG		0x2000
#define ICCMTR_REG		0x20c0
#define ICCWTR_REG		0x2100
#define ICCNSE_REG		0x2040
#define IDPNSE0_REG		0x2a40
#define IDPNSE1_REG		0x2140
#define IDPNSE2_REG		0x2240
#define IDPNSE3_REG		0x2340
#define IDPVR_REG		0x2b40
#define IDPDR0_REG		0x2a80
#define IDPDR1_REG		0x2180
#define IDPDR2_REG		0x2280
#define IDPDR3_REG		0x2380

// CPU Module defines
#define CPU0_OFFSET		0x0000
#define CPU1_OFFSET		0x0040
#define TLVID_REG		0x00c0
#define TLDIAG_REG		0x1000
#define TLMODCONFIG_REG		0x10c0
#define TLINTRMASK0_REG		0x1100
#define TLINTRMASK1_REG		0x1140
#define TLINTRSUM0_REG		0x1180
#define TLINTRSUM1_REG		0x11c0
#define TLEPAERR_REG		0x1500
#define TLEPDERR_REG		0x1540
#define TLEPMERR_REG		0x1580
#define TLEP_VMG_REG		0x15c0
#define TLEPWERR_REG		0x1600

// Memory Module defines
#define MCR_REG			0x1880
#define MIR_REG			0x1840
#define MDRA_REG		0x1980
#define MER_REG			0x1940
#define DDR0_REG		0x10140
#define DDR1_REG		0x14140
#define DDR2_REG		0x18140
#define DDR3_REG		0x1c140

// Broadcast defines
#define BROADCAST_NODE		0x18
#define TLIPINTR_REG		0x40

// GBUS defines
#define GBUS_BASE		ULL(0xfffffcff90000000)
#define GBUS_BIT_SHIFT		0x06
#define FLASH_BASE		0x07000000
#define UART_BASE		0x10000000
#define WATCH_CSR_BASE		0x20000000
#define WHATAMI_REG		0x30000000
#define MISCR_REG		0x34000000
#define SERNUM_REG		0x37000000

// RTC defines
#define RTC_SECOND		0	// second of minute [0..59]
#define RTC_SECOND_ALARM	1	// seconds to alarm
#define RTC_MINUTE		2	// minute of hour [0..59]
#define RTC_MINUTE_ALARM	3	// minutes to alarm
#define RTC_HOUR		4	// hour of day [0..23]
#define RTC_HOUR_ALARM		5	// hours to alarm
#define RTC_DAY_OF_WEEK		6	// day of week [1..7]
#define RTC_DAY_OF_MONTH	7	// day of month [1..31]
#define RTC_MONTH		8	// month of year [1..12]
#define RTC_YEAR		9	// year [00..99]
#define RTC_CONTROL_REGISTERA	10	// control register A
#define RTC_CONTROL_REGISTERB	11	// control register B
#define RTC_CONTROL_REGISTERC	12	// control register C
#define RTC_CONTROL_REGISTERD	13	// control register D
#define RTC_REGNUMBER_RTC_CR1	0x6A	// control register 1

// Other defines
#define DEVICE_TYPE_TIOP	0x2000
#define DEVICE_TYPE_MEM		0x4000
#define DEVICE_TYPE_CPU		0x8000


///////////////////////////////////////////////////////////////////////
//
// litterals used in the platform_tlaser.s file.
//
// -DBUILD_PALCODE is only defined then. The compilation does include
// this file from the simulation source tree.
//
// It is NOT an obsolete compilation option
//

#ifdef BUILD_PALCODE

#define  tlep_lintrsum0_offset   0x1180
#define  tlep_lintrsum1_offset   0x11c0
#define  tlep_tlintrsum0_offset  tlep_lintrsum0_offset
#define  tlep_tlintrsum1_offset  tlep_lintrsum1_offset
#define  tlep_watch_csrc_offset  (RTC_CONTROL_REGISTERC <<GBUS_BIT_SHIFT)

#define  tlsb_tlber              TLBER_REG
#define  tlsb_tlber_offset       tlsb_tlber /* ??? */
#define  tlsb_tldev              TLDEV_REG
#define  tlsb_tlesr0             TLESR0_REG
#define  tlsb_tlesr1             TLESR1_REG
#define  tlsb_tlesr2             TLESR2_REG
#define  tlsb_tlesr3             TLESR3_REG


#define  tlsb_tlilid0_offset     TLILID0_REG
#define  tlsb_tlilid1_offset     TLILID1_REG
#define  tlsb_tlilid2_offset     TLILID2_REG
#define  tlsb_tlilid3_offset     TLILID3_REG

#define TLSB_TLIPINTR_OFFSET     TLIPINTR_REG

#endif // BUILD_PALCODE



///////////////////////////////////////////////////////////////////////
//
// Codes used to probe/clear the TLINTRSUM register
//

#define TLASER_INTRSUM_UART	1	// uart
#define TLASER_INTRSUM_IPI	0x20	// IPI
#define TLASER_INTRSUM_INTIM	0x40	// clock

#endif // __TLASERREG_H__

