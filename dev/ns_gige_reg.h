/*
 * Copyright (c) 2004 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Portions of code taken from: */

/* ns83820.c by Benjamin LaHaise with contributions.
 *
 * Questions/comments/discussion to linux-ns83820@kvack.org.
 *
 * $Revision: 1.34.2.23 $
 *
 * Copyright 2001 Benjamin LaHaise.
 * Copyright 2001, 2002 Red Hat.
 *
 * Mmmm, chocolate vanilla mocha...
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* @file
 * Ethernet device register definitions for the National
 * Semiconductor DP83820 Ethernet controller
 */

#ifndef __DEV_NS_GIGE_REG_H__
#define __DEV_NS_GIGE_REG_H__

/*
 * Configuration Register Map
 */
#define NS_ID                   0x00    /* identification register */
#define NS_CS                   0x04    /* command and status register */
#define NS_RID                  0x08    /* revision ID register */
#define NS_LAT			0x0C	/* latency timer register */
#define NS_IOA			0x10	/* IO base address register */
#define NS_MA			0x14	/* memory address register */
#define NS_MA1			0x18	/* memory address high dword register */
#define NS_SID	                0x2C	/* subsystem identification register */
#define NS_ROM	                0x30	/* boot ROM configuration register */
#define NS_CAPPTR		0x34	/* number of tx descriptors */
#define NS_INT	                0x3C	/* interrupt select register */
#define NS_PMCAP 		0x40	/* power mgmt capabilities register */
#define NS_PMCS    		0x44	/* power mgmt control and status
                                           register */
/* Operational Register Map */
#define CR		0x00
#define CFG		0x04
#define MEAR		0x08
#define PTSCR		0x0c
#define	ISR		0x10
#define	IMR		0x14
#define	IER		0x18
#define	IHR		0x1c
#define TXDP		0x20
#define TXDP_HI		0x24
#define TXCFG		0x28
#define GPIOR		0x2c
#define RXDP		0x30
#define RXDP_HI		0x34
#define RXCFG		0x38
#define PQCR		0x3c
#define WCSR		0x40
#define PCR		0x44
#define RFCR		0x48
#define RFDR		0x4c
#define BRAR            0x50
#define BRDR            0x54
#define SRR		0x58
#define MIBC            0x5c
#define MIB_START       0x60
#define MIB_END         0x88
#define VRCR		0xbc
#define VTCR		0xc0
#define VDR		0xc4
#define CCSR		0xcc
#define TBICR		0xe0
#define TBISR		0xe4
#define TANAR		0xe8
#define TANLPAR		0xec
#define TANER		0xf0
#define TESR		0xf4
#define LAST            0xf4
#define RESERVED        0xfc

/* chip command register */
#define CR_TXE		0x00000001
#define CR_TXD		0x00000002
#define CR_RXE		0x00000004
#define CR_RXD		0x00000008
#define CR_TXR		0x00000010
#define CR_RXR		0x00000020
#define CR_SWI		0x00000080
#define CR_RST		0x00000100

/* configuration register */
#define CFG_LNKSTS	0x80000000
#define CFG_SPDSTS	0x60000000
#define CFG_SPDSTS1	0x40000000
#define CFG_SPDSTS0	0x20000000
#define CFG_DUPSTS	0x10000000
#define CFG_TBI_EN	0x01000000
#define CFG_RESERVED    0x0e000000
#define CFG_MODE_1000	0x00400000
#define CFG_AUTO_1000	0x00200000
#define CFG_PINT_CTL	0x001c0000
#define CFG_PINT_DUPSTS	0x00100000
#define CFG_PINT_LNKSTS	0x00080000
#define CFG_PINT_SPDSTS	0x00040000
#define CFG_TMRTEST	0x00020000
#define CFG_MRM_DIS	0x00010000
#define CFG_MWI_DIS	0x00008000
#define CFG_T64ADDR	0x00004000
#define CFG_PCI64_DET	0x00002000
#define CFG_DATA64_EN	0x00001000
#define CFG_M64ADDR	0x00000800
#define CFG_PHY_RST	0x00000400
#define CFG_PHY_DIS	0x00000200
#define CFG_EXTSTS_EN	0x00000100
#define CFG_REQALG	0x00000080
#define CFG_SB		0x00000040
#define CFG_POW		0x00000020
#define CFG_EXD		0x00000010
#define CFG_PESEL	0x00000008
#define CFG_BROM_DIS	0x00000004
#define CFG_EXT_125	0x00000002
#define CFG_BEM		0x00000001

/* EEPROM access register */
#define MEAR_EEDI             	0x00000001
#define MEAR_EEDO		0x00000002
#define MEAR_EECLK		0x00000004
#define MEAR_EESEL		0x00000008
#define MEAR_MDIO		0x00000010
#define MEAR_MDDIR		0x00000020
#define MEAR_MDC		0x00000040

/* PCI test control register */
#define PTSCR_EEBIST_FAIL       0x00000001
#define PTSCR_EEBIST_EN         0x00000002
#define PTSCR_EELOAD_EN         0x00000004
#define PTSCR_RBIST_FAIL        0x000001b8
#define PTSCR_RBIST_DONE        0x00000200
#define PTSCR_RBIST_EN          0x00000400
#define PTSCR_RBIST_RST         0x00002000
#define PTSCR_RBIST_RDONLY      0x000003f9

/* interrupt status register */
#define ISR_RESERVE     0x80000000
#define ISR_TXDESC3	0x40000000
#define ISR_TXDESC2	0x20000000
#define ISR_TXDESC1	0x10000000
#define ISR_TXDESC0	0x08000000
#define ISR_RXDESC3	0x04000000
#define ISR_RXDESC2	0x02000000
#define ISR_RXDESC1	0x01000000
#define ISR_RXDESC0	0x00800000
#define ISR_TXRCMP	0x00400000
#define ISR_RXRCMP	0x00200000
#define ISR_DPERR	0x00100000
#define ISR_SSERR	0x00080000
#define ISR_RMABT	0x00040000
#define ISR_RTABT	0x00020000
#define ISR_RXSOVR	0x00010000
#define ISR_HIBINT	0x00008000
#define ISR_PHY		0x00004000
#define ISR_PME		0x00002000
#define ISR_SWI		0x00001000
#define ISR_MIB		0x00000800
#define ISR_TXURN	0x00000400
#define ISR_TXIDLE	0x00000200
#define ISR_TXERR	0x00000100
#define ISR_TXDESC	0x00000080
#define ISR_TXOK	0x00000040
#define ISR_RXORN	0x00000020
#define ISR_RXIDLE	0x00000010
#define ISR_RXEARLY	0x00000008
#define ISR_RXERR	0x00000004
#define ISR_RXDESC	0x00000002
#define ISR_RXOK	0x00000001
#define ISR_ALL         0x7FFFFFFF
#define ISR_NODELAY	(ISR_ALL & ~(ISR_RXOK|ISR_RXDESC|ISR_TXOK|ISR_TXDESC))
#define ISR_NOIMPL	(~(ISR_SWI|ISR_TXIDLE|ISR_TXDESC|ISR_TXOK|ISR_RXORN| \
                           ISR_RXIDLE|ISR_RXDESC|ISR_RXOK))

/* transmit configuration register */
#define TXCFG_CSI	0x80000000
#define TXCFG_HBI	0x40000000
#define TXCFG_MLB	0x20000000
#define TXCFG_ATP	0x10000000
#define TXCFG_ECRETRY	0x00800000
#define TXCFG_BRST_DIS	0x00080000
#define TXCFG_MXDMA1024	0x00000000
#define TXCFG_MXDMA512	0x00700000
#define TXCFG_MXDMA256	0x00600000
#define TXCFG_MXDMA128	0x00500000
#define TXCFG_MXDMA64	0x00400000
#define TXCFG_MXDMA32	0x00300000
#define TXCFG_MXDMA16	0x00200000
#define TXCFG_MXDMA8	0x00100000
#define TXCFG_MXDMA     0x00700000

#define TXCFG_FLTH_MASK 0x0000ff00
#define TXCFG_DRTH_MASK 0x000000ff

/*general purpose I/O control register */
#define GPIOR_GP5_OE		0x00000200
#define GPIOR_GP4_OE		0x00000100
#define GPIOR_GP3_OE		0x00000080
#define GPIOR_GP2_OE		0x00000040
#define GPIOR_GP1_OE		0x00000020
#define GPIOR_GP3_OUT		0x00000004
#define GPIOR_GP1_OUT		0x00000001

/* receive configuration register */
#define RXCFG_AEP	0x80000000
#define RXCFG_ARP	0x40000000
#define RXCFG_STRIPCRC	0x20000000
#define RXCFG_RX_FD	0x10000000
#define RXCFG_ALP	0x08000000
#define RXCFG_AIRL	0x04000000
#define RXCFG_MXDMA512	0x00700000
#define RXCFG_MXDMA     0x00700000
#define RXCFG_DRTH	0x0000003e
#define RXCFG_DRTH0	0x00000002

/* pause control status register */
#define PCR_PSEN	(1 << 31)
#define PCR_PS_MCAST	(1 << 30)
#define PCR_PS_DA	(1 << 29)
#define PCR_STHI_8	(3 << 23)
#define PCR_STLO_4	(1 << 23)
#define PCR_FFHI_8K	(3 << 21)
#define PCR_FFLO_4K	(1 << 21)
#define PCR_PAUSE_CNT	0xFFFE

/*receive filter/match control register */
#define RFCR_RFEN	0x80000000
#define RFCR_AAB	0x40000000
#define RFCR_AAM	0x20000000
#define RFCR_AAU	0x10000000
#define RFCR_APM	0x08000000
#define RFCR_APAT	0x07800000
#define RFCR_APAT3	0x04000000
#define RFCR_APAT2	0x02000000
#define RFCR_APAT1	0x01000000
#define RFCR_APAT0	0x00800000
#define RFCR_AARP	0x00400000
#define RFCR_MHEN	0x00200000
#define RFCR_UHEN	0x00100000
#define RFCR_ULM	0x00080000
#define RFCR_RFADDR     0x000003ff

/* receive filter/match data register */
#define RFDR_BMASK      0x00030000
#define RFDR_RFDATA0    0x000000ff
#define RFDR_RFDATA1    0x0000ff00

/* management information base control register */
#define MIBC_MIBS	0x00000008
#define MIBC_ACLR	0x00000004
#define MIBC_FRZ	0x00000002
#define MIBC_WRN	0x00000001

/* VLAN/IP receive control register */
#define VRCR_RUDPE	0x00000080
#define VRCR_RTCPE	0x00000040
#define VRCR_RIPE	0x00000020
#define VRCR_IPEN	0x00000010
#define VRCR_DUTF	0x00000008
#define VRCR_DVTF	0x00000004
#define VRCR_VTREN	0x00000002
#define VRCR_VTDEN	0x00000001

/* VLAN/IP transmit control register */
#define VTCR_PPCHK	0x00000008
#define VTCR_GCHK	0x00000004
#define VTCR_VPPTI	0x00000002
#define VTCR_VGTI	0x00000001

/* Clockrun Control/Status Register */
#define CCSR_CLKRUN_EN  0x00000001

/* TBI control register */
#define TBICR_MR_LOOPBACK       0x00004000
#define TBICR_MR_AN_ENABLE	0x00001000
#define TBICR_MR_RESTART_AN	0x00000200

/* TBI status register */
#define TBISR_MR_LINK_STATUS	0x00000020
#define TBISR_MR_AN_COMPLETE	0x00000004

/* TBI auto-negotiation advertisement register */
#define TANAR_PS2 		0x00000100
#define TANAR_PS1 		0x00000080
#define TANAR_HALF_DUP 		0x00000040
#define TANAR_FULL_DUP 		0x00000020

/*
 * descriptor format currently assuming link and bufptr
 * are set for 32 bits,( may be wrong ) ASSUME32
 */
struct ns_desc {
  uint32_t link;    /* link field to next descriptor in linked list */
  uint32_t bufptr;  /* pointer to the first fragment or buffer */
  uint32_t cmdsts;  /* command/status field */
  uint32_t extsts;  /* extended status field for VLAN and IP info */
};

/* cmdsts flags for descriptors */
#define CMDSTS_OWN	0x80000000
#define CMDSTS_MORE	0x40000000
#define CMDSTS_INTR	0x20000000
#define CMDSTS_ERR	0x10000000
#define CMDSTS_OK	0x08000000
#define CMDSTS_LEN_MASK	0x0000ffff

#define CMDSTS_DEST_MASK	0x01800000
#define CMDSTS_DEST_SELF	0x00800000
#define CMDSTS_DEST_MULTI	0x01000000

/* extended flags for descriptors */
#define EXTSTS_UDPERR   0x00400000
#define EXTSTS_UDPPKT	0x00200000
#define EXTSTS_TCPERR   0x00100000
#define EXTSTS_TCPPKT	0x00080000
#define EXTSTS_IPERR    0x00040000
#define EXTSTS_IPPKT	0x00020000


/* speed status */
#define SPDSTS_POLARITY	(CFG_SPDSTS1 | CFG_SPDSTS0 | CFG_DUPSTS | (lnksts ? CFG_LNKSTS : 0))

#endif /* __DEV_NS_GIGE_REG_H__ */
