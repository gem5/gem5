/*
 *      VID: [T1.2] PT: [Fri Apr 21 16:47:18 1995] SF: [platform.h]
 *       TI: [/sae_users/cruz/bin/vice -iplatform.s -l// -p# -DEB164 -h -m -aeb164 ]
 */
#define	__PLATFORM_LOADED	1
/*
*****************************************************************************
**                                                                          *
**  Copyright © 1993, 1994	       					    *
**  by Digital Equipment Corporation, Maynard, Massachusetts.		    *
**                                                                          *
**  All Rights Reserved							    *
**                                                                          *
**  Permission  is  hereby  granted  to  use, copy, modify and distribute   *
**  this  software  and  its  documentation,  in  both  source  code  and   *
**  object  code  form,  and without fee, for the purpose of distribution   *
**  of this software  or  modifications  of this software within products   *
**  incorporating  an  integrated   circuit  implementing  Digital's  AXP   *
**  architecture,  regardless  of the  source of such integrated circuit,   *
**  provided that the  above copyright  notice and this permission notice   *
**  appear  in  all copies,  and  that  the  name  of  Digital  Equipment   *
**  Corporation  not  be  used  in advertising or publicity pertaining to   *
**  distribution of the  document  or  software without specific, written   *
**  prior permission.							    *
**                                                                          *
**  Digital  Equipment  Corporation   disclaims  all   warranties  and/or   *
**  guarantees  with  regard  to  this  software,  including  all implied   *
**  warranties of fitness for  a  particular purpose and merchantability,   *
**  and makes  no  representations  regarding  the use of, or the results   *
**  of the use of, the software and documentation in terms of correctness,  *
**  accuracy,  reliability,  currentness  or  otherwise;  and you rely on   *
**  the software, documentation and results solely at your own risk.	    *
**                                                                          *
**  AXP is a trademark of Digital Equipment Corporation.		    *
**                                                                          *
*****************************************************************************
**
**  FACILITY:
**
**	DECchip 21164 OSF/1 PALcode
**
**  MODULE:
**
**	platform.h
**
**  MODULE DESCRIPTION:
**
**      Platform specific definitions.
**
**  AUTHOR: Lance Berc (taken from EB164 code)
**
**  CREATION DATE:  14-Jun-1995
**
**  $Id: platform.h,v 1.1.1.1 1997/10/30 23:27:20 verghese Exp $
**
**  MODIFICATION HISTORY:
**
**  $Log: platform.h,v $
**  Revision 1.1.1.1  1997/10/30 23:27:20  verghese
**  current 10/29/97
**
 * Revision 1.1  1995/06/14  18:50:42  berc
 * Initial revision
 *
*/

#if !defined(CONSOLE_ENTRY)
#define CONSOLE_ENTRY	        0x10000
#endif /* CONSOLE_ENTRY */

#define DEBUGDEATH(c) \
        lda	a0, c(zero) ; \
        br DebugDeath

#define DEBUGSTORE(c) \
        stq_p	t0,0(zero) ; \
        stq_p	t1,8(zero) ; \
        lda	t0, 0x400(zero) ; \
        sll	t0, 29, t0 ; \
        ldah	t0, 0x280(t0) ; \
9:	lda	t1, 0x140(t0) ; \
        ldl_p	t1, 0(t1) ; \
        srl	t1, 16, t1 ; \
        and	t1, 0x20, t1 ; \
        beq	t1, 9b ; \
        lda	t1, c(zero) ; \
        stl_p	t1, 0(t0) ; \
        mb ; \
        ldq_p	t1, 8(zero) ; \
        ldq_p	t0, 0(zero)


/*
** IPL translation table definitions:
**
** EB164 specific IRQ pins are
**
**  Line   IPL  Source			        OSF/1 IPL
**  ----   ---	------			        ---------
**  IRQ0   20	Corrected ECC error	        7
**  IRQ1   21	PCI/ISA                         3
**  IRQ2   22	Real Time Clock 	        5
**  IRQ3   23	SIO NMI, CIA errors	        7
**
**  The mask contains one byte for each IPL level, with IPL0 in the
**  least significant (right-most) byte and IPL7 in the most
**  significant (left-most) byte.  Each byte in the mask maps the
**  OSF/1 IPL to the DC21164 IPL.
**
**  OSF/1 IPL	IPL
**  ---------	---
**	0	0
**	1	1
**	2	2
**	3	21 (to account for PCI/ISA at IPL 21)
**	4	21
**	5	22 (to account for clock at IPL 21)
**	6	30 (to account for powerfail)
**	7	31
*/

#define INT_K_MASK_HIGH         0x1F1E1615
#define INT_K_MASK_LOW          0x15020100

#define BYTE_ENABLE_SHIFT	5

/*
** Dallas DS1287A Real-Time Clock (RTC) Definitions:
*/
#define RTCADD 0x160000
#define RTCDAT 0x170000


/*
** Serial Port (COM) Definitions:
*/

#define DLA_K_BRG		12	/* Baud Rate Divisor = 9600 */

#define LSR_V_THRE		5	/* Xmit Holding Register Empty Bit */

#define LCR_M_WLS		3	/* Word Length Select Mask */
#define LCR_M_STB		4	/* Number Of Stop Bits Mask */
#define LCR_M_PEN		8	/* Parity Enable Mask */
#define LCR_M_DLAB		128	/* Divisor Latch Access Bit Mask */

#define LCR_K_INIT	      	(LCR_M_WLS | LCR_M_STB)

#define MCR_M_DTR		1	/* Data Terminal Ready Mask */
#define MCR_M_RTS		2	/* Request To Send Mask */
#define MCR_M_OUT1		4	/* Output 1 Control Mask */
#define MCR_M_OUT2		8	/* UART Interrupt Mask Enable */

#define MCR_K_INIT	      	(MCR_M_DTR  | \
                                 MCR_M_RTS  | \
                                 MCR_M_OUT1 | \
                                 MCR_M_OUT2)

/*    CPU Adr[39:29]=0x500 select PCI Mem.	*/
#define PCI_MEM   0x400
#define SLOT_D_COM1                (0x140000)
#define SLOT_D_COM2                (0x150000)

#define COM1_RBR (SLOT_D_COM1 | (0x0 << 1)) /* Receive Buffer Register Offset */
#define COM1_THR (SLOT_D_COM1 | (0x0 << 1)) /* Xmit Holding Register Offset */
#define COM1_IER (SLOT_D_COM1 | (0x1 << 1)) /* Interrupt Enable Register Offset */
#define COM1_IIR (SLOT_D_COM1 | (0x2 << 1)) /* Interrupt ID Register Offset */
#define COM1_LCR (SLOT_D_COM1 | (0x3 << 1)) /* Line Control Register Offset */
#define COM1_MCR (SLOT_D_COM1 | (0x4 << 1)) /* Modem Control Register Offset */
#define COM1_LSR (SLOT_D_COM1 | (0x5 << 1)) /* Line Status Register Offset */
#define COM1_MSR (SLOT_D_COM1 | (0x6 << 1)) /* Modem Status Register Offset */
#define COM1_SCR (SLOT_D_COM1 | (0x7 << 1)) /* Scratch Register Offset */
#define COM1_DLL (SLOT_D_COM1 | (0x8 << 1)) /* Divisor Latch (LS) Offset */
#define COM1_DLH (SLOT_D_COM1 | (0x9 << 1)) /* Divisor Latch (MS) Offset */

#define COM2_RBR (SLOT_D_COM2 | (0x0 << 1))
#define COM2_THR (SLOT_D_COM2 | (0x0 << 1))
#define COM2_IER (SLOT_D_COM2 | (0x1 << 1))
#define COM2_IIR (SLOT_D_COM2 | (0x2 << 1))
#define COM2_LCR (SLOT_D_COM2 | (0x3 << 1))
#define COM2_MCR (SLOT_D_COM2 | (0x4 << 1))
#define COM2_LSR (SLOT_D_COM2 | (0x5 << 1))
#define COM2_MSR (SLOT_D_COM2 | (0x6 << 1))
#define COM2_SCR (SLOT_D_COM2 | (0x7 << 1))
#define COM2_DLL (SLOT_D_COM2 | (0x8 << 1))
#define COM2_DLH (SLOT_D_COM2 | (0x9 << 1))


/*
** Macro to define a port address
*/
#define IO_MASK 	0x7FFFFFF

/* NOTE ON ADDITIONAL PORT DEFINITION:
**
** We also need to set bit 39! Since the span between bit 39
** and the byte enable field is more than 32, we set bit 39 in the
** port macros.
*/

/*
** Macro to write a byte literal to a specified port
*/
#define OutPortByte(port,val,tmp0,tmp1) \
        LDLI	(tmp0, port); \
        sll	tmp0, BYTE_ENABLE_SHIFT, tmp0; \
        lda	tmp1, PCI_MEM(zero); \
        sll	tmp1, 29, tmp1; \
        bis	tmp0, tmp1, tmp0; \
        lda	tmp1, (val)(zero); \
        sll	tmp1, 8*(port & 3), tmp1; \
        stl_p	tmp1, 0x00(tmp0); \
        mb

/*
** Macro to write a byte from a register to a specified port
*/
#define OutPortByteReg(port,reg,tmp0,tmp1) \
        LDLI	(tmp0, port); \
        sll	tmp0, BYTE_ENABLE_SHIFT, tmp0; \
        lda	tmp1, PCI_MEM(zero); \
        sll	tmp1, 29, tmp1; \
        bis	tmp0, tmp1, tmp0; \
        sll	reg, 8*(port & 3), tmp1; \
        stl_p	tmp1, 0x00(tmp0); \
        mb

/*
** Macro to write a longword from a register to a specified port
*/
#define OutPortLongReg(port,reg,tmp0,tmp1) \
        LDLI	(tmp0, port); \
        sll	tmp0, BYTE_ENABLE_SHIFT, tmp0; \
        lda	tmp1, PCI_MEM(zero); \
        sll	tmp1, 29, tmp1; \
        bis	tmp0, tmp1, tmp0; \
        stl_p	tmp1, 0x18(tmp0); \
        mb

/*
** Macro to read a byte from a specified port
*/
#define InPortByte(port,tmp0,tmp1) \
        LDLI	(tmp0, port); \
        sll	tmp0, BYTE_ENABLE_SHIFT, tmp0; \
        lda	tmp1, PCI_MEM(zero); \
        sll	tmp1, 29, tmp1; \
        bis	tmp0, tmp1, tmp0; \
        ldl_p	tmp0, 0x00(tmp0); \
        srl	tmp0, (8 * (port & 3)), tmp0; \
        zap	tmp0, 0xfe, tmp0
