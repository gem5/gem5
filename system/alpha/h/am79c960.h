#ifndef __AM79C960_H_LOADED
#define __AM79C960_H_LOADED
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
 * $Id: am79c960.h,v 1.1.1.1 1997/10/30 23:27:13 verghese Exp $
 */

/*
 * MODULE DESCRIPTION:
 *
 *    Parameters and logicals for AM79C960 drivers in EB64 monitor
 *
 * HISTORY:
 *
 * $Log: am79c960.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:13  verghese
 * current 10/29/97
 *
 * Revision 1.8  1994/12/12  21:31:59  cruz
 * Fixed bug in TMD3 constants
 *
 * Revision 1.7  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.6  1994/06/28  20:08:21  fdh
 * Modified filenames and build precedure to fit into a FAT filesystem.
 *
 * Revision 1.5  1994/06/24  15:26:49  rusling
 * Fixed initialization alignment problems.
 *
 * Revision 1.4  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.3  1994/06/17  19:34:01  fdh
 * Clean-up...
 *
 * Revision 1.2  1994/06/03  20:11:11  fdh
 * Replaced shorts declared in bitfields to ints.
 *
 * Revision 1.1  1993/08/06  10:10:05  berent
 * Initial revision
 *
 *
 */



/*  The embedded Ethernet controller on the EB64 is an AMD Am79C960 device.
 *
 *  This device integrates an Am7990 LANCE Ethernet controller with an ISA
 *  bus interface plus all the serial interface logic bar some discrete
 *  termination and isolation transformers for AUI (10base5) and twisted
 *  pair (10baseT) connections.
 *
 *  The AUI port on EB64 is connected to an Am7996 thinwire transceiver to
 *  provide a 10base2 port (BNC) as well as the twisted pair (MMJ)
 *  connectivity.
 *
 *  The device is programmed in a similar but not identical fashion to the
 *  LANCE. The following programming differences and features are noted:
 *
 *  A RAP/RDP mechanism is used for register access like the LANCE, but with
 *  a much larger register set:
 *
 *        Initialisation block now part of on-chip register set with new
 *        ...features added to the mode register. This can be set up in
 *        ...memory and copied by the device using the csr0_INIT bit, or
 *        ...programmed directly to the device using the extended CSR map.
 *
 *        CSR3 completely changed - interrupt mask and enabling bits for
 *        ...new features
 *
 *        CSR3 can be accessed while the "LANCE" running
 *
 *        A register added for controlling the ISA bus DMA profile eg. FIFO
 *        ...watermarks for DMA loads and stores, TX transmission start
 *        ...point wrt data in TX FIFO, DMA burstsizes.
 *
 *        Registers to overrule ring sizes in initialisation blocks allowing
 *        ...ring sizes from 1-64k entries. CSR76 for RX; CSR78 for TX
 *
 *         Timer and ID registers
 *
 *         Missed pkt and RX collision counters
 *
 * A similar RAP/IDP mechanism is used to program ISA DMA signal timing, ISA
 * bus configuration, bits, and the functionality for the three LED drives
 * provided.
 *
 * The Am79C960 *must* use a 16-bit DMA path, but can use an 8-bit path for
 * CSR (IO) access. It also supports access to a sixteen byte area for
 * keeping the MAC address in PROM. This is always byte-wide access, as is a
 * shared memory mode to a larger local address space, not used on EB64.
 *
 * EB64 accesses to the 79C960 are through the VL82C486 ISA controller. All
 * CSR accesses should be done as 16-bit words, with the VL82C486 taking care
 * of the packing/unpacking issue. Enet PROM accesses are done as byte
 * accesses. The EB64 memory controller will perform read_modify_write cycles
 * into main memory to correctly merge the bytes/words, and preserve longword
 * parity.
 *
 *
 *  Please see the Am7990 and Am79C960 literature for definitive lists and
 *  further detail.
 *
 *  PLEASE NOTE: LANCE is used in many definitions for historical reasons.
 */


        /* Maximum number of AMD79C960's supported in the EB64 monitor */

#define AM79C960_MAX_DEVICES	2


        /* Base addresses supported for 79C960 in ISA address space */

#define EMBEDDED_79C960_BASE 	0x360	    /* IOAM pins = 0b11 */
#define OPTION_79C960_BASE 	0x300	    /* IOAM pins = 0b00 */

        /* Assigned interrupt numbers for 8259 controller */

#define EMBEDDED_79C960_INT 	9
#define OPTION_79C960_INT	12

        /* Assigned mask numbers for enabling appropriate DMA channel */

#define EMBEDDED_79C960_DMA 	1
#define OPTION_79C960_DMA 	2


             /* CSR access offsets for 79C960 */

#define HW_ADDR_OFFSET  0    /* The hardware address is at the base address for the device */
#define	RDP_OFFSET	0x10
#define	RAP_OFFSET 	0x12
#define	RESET_OFFSET	0x14
#define	IDP_OFFSET	0x16
#define	VSW_OFFSET	0x18

#define LANCE_CSR0  0		/* Main csr */
#define LANCE_IADRL 1		/* Init block low address bits */
#define LANCE_IADRH 2		/* Init block high address bits */
#define LANCE_CSR3  3		/* Mask and function bits */
#define LANCE_CSR4  4           /* Additional mask and function bits, only exists on amd 79c960 */

    /* CSR bit definitions - CSR1 and 2 have no bit fields */

             /* CSR0 Bit definitions */

#define csr0_INIT	0x0001		/* Initialize - can be set with STRT on 79C960*/
#define csr0_STRT	0x0002		/* Start operation */
#define csr0_STOP	0x0004		/* Stop LANCE */
#define csr0_TDMD	0x0008		/* Transmit demand */
#define csr0_TXON	0x0010		/* Transmitter ON */
#define csr0_RXON	0x0020		/* Receiver ON */
#define csr0_IENA	0x0040		/* Interrupt enable */
#define csr0_INTR	0x0080		/* Interrupt received */
#define csr0_IDON	0x0100		/* Initialization done */
#define csr0_TINT	0x0200		/* Transmitter interrupt */
#define csr0_RINT	0x0400		/* Receiver interrupt */
#define csr0_MERR	0x0800		/* Memory error */
#define csr0_MISS	0x1000		/* Missed packet */
#define csr0_CERR	0x2000		/* Collision error */
#define csr0_BABL	0x4000		/* Babbling on the wire */
#define csr0_ERR	0x8000		/* Error logical OR */


             /* CSR3 Bit definitions - all undefined bits reserved */

#define csr3_EMBA	0x0008		/* Enable Alternate Backoff Algorithm - 79C960 */
#define csr3_DXMT2PD	0x0010		/* Disable 2-part TX deferral - 79C960 */
#define csr3_IDONM	0x0100		/* Initialise Done interrupt mask */
#define csr3_TINT	0x0200		/* TX interrupt mask */
#define csr3_RINT	0x0400		/* RX interrupt mask */
#define csr3_MERM	0x0800		/* Memory error interrupt mask */
#define csr3_MISSM	0x1000		/* Missed pkt interrupt mask */
#define csr3_BABLM	0x4000		/* Babble interrupt mask */
#define csr3_MASKALL    0x5F00          /* Mask all CSR3 maskable interrupts */

             /* CSR4 Bit definitions - all undefined bits reserved */
                   /* This register is unique to the 79C960 */

#define csr4_JABM	0x0001		/* Jabber interrupt mask
#define csr4_JAB	0x0002		/* set if T-MAU detects a jabber condition
#define csr4_TXSTRTM	0x0004		/* TX start (of packet) interrupt mask */
#define csr4_TXSTRT	0x0008		/*    */
#define csr4_RCVCCOM	0x0010		/* RX Collision Counter Overflow interrupt mask */
#define csr4_RCVCCO	0x0020		/* RX Collision Counter Overflow bit */
#define csr4_MPCOM	0x0100		/* Missed Pkt counter overflow interrupt mask */
#define csr4_MPCO	0x0200		/* Missed Pkt counter overflow bit */
#define csr4_ASTRPRCV	0x0400		/* Enable Automatic Pad Stripping on RX - IEEE802.3 */
#define csr4_APADXMT	0x0800		/* Enable Automatic Padding on TX - IEEE802.3 */
#define csr4_DPOLL	0x1000		/* Disable polling - TX rings */
#define csr4_TIMER	0x2000		/*    */
#define csr4_DMAPLUS	0x4000		/* Disable CSR80 DMA burst counter */
#define csr4_ENTST	0x8000		/* Enable Test mode */



             /* Receive Message Descriptor */
       /* RMD0, RMD2 and RMD3 have no bit fields */

     /* Bits 0 -> 7 of RMD1 part of buffer address */

#define rmd1_ENP	0x0100		/* End of packet */
#define rmd1_STP	0x0200		/* Start of packet */
#define rmd1_BUFF	0x0400		/* Buffer error */
#define rmd1_CRC	0x0800		/* CRC error */
#define rmd1_OFLO	0x1000		/* Overflow error */
#define rmd1_FRAM	0x2000		/* Framing error */
#define rmd1_ERR	0x4000		/* Error logical OR */
#define rmd1_OWN	0x8000		/* Who owns it */



             /* Transmit Message Descriptor */
           /* TMD0 and TMD2 have no bit fields */

     /* Bits 0 -> 7 of TMD1 part of buffer address */

#define tmd1_ENP	0x0100		/* End of packet */
#define tmd1_STP	0x0200		/* Start of packet */
#define tmd1_DEF	0x0400		/* Had to defer */
#define tmd1_ONE	0x0800		/* One eretry needed */
#define tdm1_MORE	0x1000		/* More than one retry */
#define tmd1_ADDFCS	0x2000		/* Am79C960 specific - add FCS on pkt basis */
#define tmd1_ERR	0x4000		/* Error logical OR */
#define tmd1_OWN	0x8000		/* Who owns it */


  /* Bits 0 -> 9 of TMD3 provide a TDR counter for finding cable faults */

#define tmd3_RTRY	0x0400		/* Too many retries */
#define tmd3_LCAR	0x0800		/* Loss of Carrier */
#define tmd3_LCOL	0x1000		/* Late Collision on send */
#define tmd3_RES	0x2000		/* RESERVED */
#define tmd3_UFLO	0x4000		/* Underflow Error */
#define tmd3_BUFF	0x8000		/* Buffer Error */


             /* For init block - operating modes */

#define mode_DRX	0x0001		/* Disable receiver */
#define mode_DTX	0x0002		/* Disable transmitter */
#define mode_LOOP 	0x0004		/* Loopback mode */
#define mode_DTCR	0x0008		/* Disable transmit CRC */
#define mode_COLL	0x0010		/* Force collision */
#define mode_DRTY	0x0020		/* Disable retries */
#define mode_INTL  	0x0040		/* Internal loopback */
#define mode_PORTSEL0	0x0080		/* 0 = AUI; 1 = T-MAU */
#define mode_PORTSEL1	0x0100		/* MBZ */
#define mode_LRT_TSEL	0x0200		/* Low RX threshold/TX mode select */
#define mode_MENDECL	0x0400		/* MENDEC loopback mode */
#define mode_DAPC	0x0800		/* T-MAU: disable automatic polarity correction */
#define mode_DLNKTST	0x1000		/* Disable link status */
#define mode_DRCVPA	0x2000		/* Disable RX physical address */
#define mode_DRCVBC	0x4000		/* Disable RX broadcast */
#define mode_PROM	0x8000		/* Promiscuous mode */



             /* Initialization block. */

#define LANCE_INIT_BLOCK_SIZE 		24
#define LANCE_INIT_BLOCK_MODE_OFFSET 	0
#define LANCE_INIT_BLOCK_PADR_OFFSET 	2
#define LANCE_INIT_BLOCK_LADRF_OFFSET 	8
#define LANCE_INIT_BLOCK_RD_OFFSET	16
#define LANCE_INIT_BLOCK_TD_OFFSET	20


typedef struct {
    unsigned
      radr : 24,	/* Ring address.			*/
           :  5,
      rlen :  3;	/* Entries (0=1, 1=2, 2=4, 3=8...)	*/
} INIT_BLOCK_DESC;

             /* Receive buffer descriptor. */

typedef struct{
  unsigned int
    ladr : 24,		/* Buffer address.			*/
    enp	 : 1,		/* End of packet.			*/
    stp	 : 1,		/* Start of packet.			*/
    buff : 1,		/* Buffer error.			*/
    crc  : 1,		/* CRC error.				*/
    oflo : 1,		/* Overflow error.			*/
    fram : 1,		/* Framing error.			*/
    err  : 1,		/* Error summary: buff + crc + oflo + fram */
    own  : 1;		/* Buffer owner: host=0, lance=1.	*/
  uw bcnt;		/* Buffer size.				*/
  uw mcnt;		/* Message size.			*/
} LANCE_RECV_BD;

             /* Transmit buffer descriptor. */

typedef struct{
  unsigned int
    ladr : 24,		/* Buffer address.			*/
    enp	 : 1,		/* End of packet.			*/
    stp	 : 1,		/* Start of packet.			*/
    def	 : 1,		/* Deferred.				*/
    one	 : 1,		/* One retry.				*/
    more : 1,		/* More retries.			*/
    add_fcs : 1,        /* Add FCS on packet basis - 79C960 specific feature */
    err	 : 1,		/* Error summary: lcol + lcar + uflo + rtry */
    own	 : 1;		/* Buffer owner: host=0, lance=1.	*/
  uw bcnt;		/* Buffer size.				*/
  uw tmd3;		/* TMD 3				*/
} LANCE_SEND_BD;

#define LANCE_OWNS 1
#define HOST_OWNS  0

             /* Lance counters. */

typedef struct{
  time_t zeroed;
  int    b_recv;
  int    b_sent;
  int    f_recv;
  int    f_sent;
  int    mc_b_recv;
  int    mc_f_recv;
  int    f_sent_def;
  int    f_sent_sgl_col;
  int    f_sent_mult_col;
  int    send_fail_exs_col;
  int    send_fail_cc;
  int    send_fail_short;
  int    send_fail_open;
  int    send_fail_flen;
  int    send_fail_defer;
  int    recv_fail_fcs;
  int    recv_fail_f_err;
  int    recv_fail_flen;
  int    data_overrun;
  int    sbuf_unav;
  int    ubuf_unav;
  int    cc_fail;
} LANCE_CTRS;

typedef struct{
  int	base_addr;
  int	interrupt_line;
  int	dma_mask;
} AM79C960_CONFIG;

#define AM79C960_FLUSH_WATCHDOG	    5000    /* watchdog timeout  - in ms - for TX flush routine */

/* /ether/am79c960.c */
extern int am79c960_device_register(int device_no);
extern void am79c960_device_init_module(void );

#endif /* __AM79C960_H_LOADED */
