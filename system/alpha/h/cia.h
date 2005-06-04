/*
Copyright 1993, 1994, 1995 Hewlett-Packard Development Company, L.P.

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



#ifndef __CIA_H_LOADED
#define __CIA_H_LOADED

/*
 *  $Id: cia.h,v 1.1.1.1 1997/10/30 23:27:14 verghese Exp $;
 */

/*
 * $Log: cia.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:14  verghese
 * current 10/29/97
 *
 * Revision 1.3  1995/04/20  16:37:22  cruz
 * Made some deletions and modifications.
 *
 * Revision 1.2  1994/12/19  18:33:02  cruz
 * Added some new constants.
 *
 * Revision 1.1  1994/11/23  19:49:39  cruz
 * Initial revision
 *
 *
 */

/*
 ******************************** WARNING *******************************
 *	BE EXTREMELY CAREFUL WHEN USING OFFSETS LARGER THAN 0x8000  WITH
 *	AN "LDA" INSTRUCTION BECAUSE THEY WILL CAUSE AN UNWANTED SIGN
 *	EXTENSION.
 ******************************** WARNING *******************************
*/

/* ======================================================================
 * =			BASE ADDRESSES					=
 * ======================================================================
 *
 *	Define the base address for registers accessing the memory
 *	controller
*/
#define MC_GCR_BASE		0x874	/* General Control Register	*/
#define MC_GCR_BASE_SHIFT 	28	/* Shift base by this # of bits */
#define MC_DIAG_BASE		0x874	/* Diagnostic Registers		*/
#define MC_DIAG_BASE_SHIFT 	28
#define MC_PERFMON_BASE		0x874	/* Performance monitoring regs 	*/
#define MC_PERFMON_BASE_SHIFT 	28
#define MC_ERROR_BASE		0x874	/* Error Registers		*/
#define MC_ERROR_BASE_SHIFT 	28
#define MC_MCR_BASE		0x875	/* Memory Configuration Registers*/
#define MC_MCR_BASE_SHIFT 	28
#define MC_PCI_BASE		0x876	/* PCI Configuration Registers	*/
#define MC_PCI_BASE_SHIFT 	28


/* ======================================================================
 * =	  OFFSETS FOR THE GENERAL CONTROL REGISTERS (87.4000.0000)	=
 * ======================================================================
*/
#define MC_PCI_REV		0x80	/* Revision of CIA ASIC.	*/
#define MC_PCI_LAT		0xC0	/* PCI Master Latency Timer.	*/
#define MC_CIA_CTRL		0x100	/* CIA Control Register.	*/
#define MC_CIA_CNFG		0x200	/* CIA Configuration Register.	*/
#define MC_HAE_MEM		0x400   /* HW Address Extension (Sparse)*/
#define MC_HAE_IO		0x440	/* HW Addr. Ext. for sparse IO.	*/
#define MC_CFG			0x480	/* Configuration reg for bridges*/
#define MC_CACK_EN		0x600	/* Acknowledgement Control Reg.	*/


/* ======================================================================
 * =	    OFFSETS FOR THE DIAGNOSTIC REGISTERS (87.4000.0000)		=
 * ======================================================================
*/
#define MC_CIA_DIAG		0x2000	/* Diagnostic control register	*/
#define MC_DIAG_CHECK		0x3000	/* Diagnostic check register	*/


/* ======================================================================
 * =   OFFSETS FOR THE PERFORMANCE MONITORING REGISTERS (87.4000.0000)	=
 * ======================================================================
*/
#define MC_PERF_MONITOR		0x4000	/* Performance monitor register	*/
#define MC_PERF_CONTROL		0x4040	/* Perf. Mon. Control register	*/


/* ======================================================================
 * =	      OFFSETS FOR THE ERROR REGISTERS (87.4000.0000)		=
 * ======================================================================
*/
#define MC_CPU_ERR0		0x8000	/* CPU Error Info Register 0.	*/
#define MC_CPU_ERR1		0x8040	/* CPU Error Info Register 1.	*/
#define MC_CIA_ERR		0x8200	/* CIA Error Register.		*/
#define MC_CIA_STAT		0x8240	/* CIA Status Register.		*/
#define MC_ERR_MASK		0x8280	/* CIA Error Mask Register.	*/
#define MC_CIA_SYN		0x8300	/* CIA Syndrome Register.	*/
#define MC_MEM_ERR0		0x8400	/* Memory Port Status Register 0*/
#define MC_MEM_ERR1		0x8440	/* Memory Port Status Register 1*/
#define MC_PCI_ERR0		0x8800	/* PCI Error Status Register 0.	*/
#define MC_PCI_ERR1		0x8840	/* PCI Error Status Register 1.	*/


/* ======================================================================
 * =   OFFSETS FOR THE MEMORY CONFIGURATION REGISTERS (87.5000.0000)	=
 * ======================================================================
*/
#define MC_MCR			0x0	/* Memory Configuration Register*/
#define MC_MBA0			0x600	/* Memory Base Address Register	*/
#define MC_MBA2			0x680	/* Memory Base Address Register	*/
#define MC_MBA4			0x700	/* Memory Base Address Register	*/
#define MC_MBA6			0x780	/* Memory Base Address Register	*/
#define MC_MBA8			0x800	/* Memory Base Address Register	*/
#define MC_MBAA			0x880	/* Memory Base Address Register	*/
#define MC_MBAC			0x900	/* Memory Base Address Register	*/
#define MC_MBAE			0x980	/* Memory Base Address Register	*/
#define MC_TMG0			0xB00	/* Memory Timing Register	*/
#define MC_TMG1			0xB40	/* Memory Timing Register	*/
#define MC_TMG2			0xB80	/* Memory Timing Register	*/


/* ======================================================================
 * =     OFFSETS FOR THE PCI CONFIGURATION REGISTERS (87.6000.0000)	=
 * ======================================================================
*/
#define MC_TBIA			0x100	/* S/G Translation Buffer Inval.*/
#define MC_W0_BASE		0x400	/* Window Base 0.		*/
#define MC_W0_MASK		0x440	/* Window Mask 0.		*/
#define MC_T0_BASE		0x480	/* Translated Base 0.		*/
#define MC_W1_BASE		0x500	/* Window Base 1.		*/
#define MC_W1_MASK		0x540	/* Window Mask 1.		*/
#define MC_T1_BASE		0x580	/* Translated Base 1.		*/
#define MC_W2_BASE		0x600	/* Window Base 2.		*/
#define MC_W2_MASK		0x640	/* Window Mask 2.		*/
#define MC_T2_BASE		0x680	/* Translated Base 2.		*/
#define MC_W3_BASE		0x700	/* Window Base 3.		*/
#define MC_W3_MASK		0x740	/* Window Mask 3.		*/
#define MC_T3_BASE		0x780	/* Translated Base 3.		*/
#define MC_DAC			0x7C0	/* Window DAC Base.		*/
#define MC_LTB_TAG0		0x800	/* Lockable Translation Buffer.	*/
#define MC_LTB_TAG1		0x840	/* Lockable Translation Buffer.	*/
#define MC_LTB_TAG2		0x880	/* Lockable Translation Buffer.	*/
#define MC_LTB_TAG3		0x8C0	/* Lockable Translation Buffer.	*/
#define MC_TB_TAG0		0x900	/* Translation Buffer Tag.	*/
#define MC_TB_TAG1		0x940	/* Translation Buffer Tag.	*/
#define MC_TB_TAG2		0x980	/* Translation Buffer Tag.	*/
#define MC_TB_TAG3		0x9C0	/* Translation Buffer Tag.	*/
#define MC_TB0_PAGE0		0x1000	/* Translation Buffer 0 Page.	*/
#define MC_TB0_PAGE1		0x1040	/* Translation Buffer 0 Page.	*/
#define MC_TB0_PAGE2		0x1080	/* Translation Buffer 0 Page.	*/
#define MC_TB0_PAGE3		0x10C0	/* Translation Buffer 0 Page.	*/
#define MC_TB1_PAGE0		0x1100	/* Translation Buffer 1 Page.	*/
#define MC_TB1_PAGE1		0x1140	/* Translation Buffer 1 Page.	*/
#define MC_TB1_PAGE2		0x1180	/* Translation Buffer 1 Page.	*/
#define MC_TB1_PAGE3		0x11C0	/* Translation Buffer 1 Page.	*/
#define MC_TB2_PAGE0		0x1200	/* Translation Buffer 2 Page.	*/
#define MC_TB2_PAGE1		0x1240	/* Translation Buffer 2 Page.	*/
#define MC_TB2_PAGE2		0x1280	/* Translation Buffer 2 Page.	*/
#define MC_TB2_PAGE3		0x12C0	/* Translation Buffer 2 Page.	*/
#define MC_TB3_PAGE0		0x1300	/* Translation Buffer 3 Page.	*/
#define MC_TB3_PAGE1		0x1340	/* Translation Buffer 3 Page.	*/
#define MC_TB3_PAGE2		0x1380	/* Translation Buffer 3 Page.	*/
#define MC_TB3_PAGE3		0x13C0	/* Translation Buffer 3 Page.	*/
#define MC_TB4_PAGE0		0x1400	/* Translation Buffer 4 Page.	*/
#define MC_TB4_PAGE1		0x1440	/* Translation Buffer 4 Page.	*/
#define MC_TB4_PAGE2		0x1480	/* Translation Buffer 4 Page.	*/
#define MC_TB4_PAGE3		0x14C0	/* Translation Buffer 4 Page.	*/
#define MC_TB5_PAGE0		0x1500	/* Translation Buffer 5 Page.	*/
#define MC_TB5_PAGE1		0x1540	/* Translation Buffer 5 Page.	*/
#define MC_TB5_PAGE2		0x1580	/* Translation Buffer 5 Page.	*/
#define MC_TB5_PAGE3		0x15C0	/* Translation Buffer 5 Page.	*/
#define MC_TB6_PAGE0		0x1600	/* Translation Buffer 6 Page.	*/
#define MC_TB6_PAGE1		0x1640	/* Translation Buffer 6 Page.	*/
#define MC_TB6_PAGE2		0x1680	/* Translation Buffer 6 Page.	*/
#define MC_TB6_PAGE3		0x16C0	/* Translation Buffer 6 Page.	*/
#define MC_TB7_PAGE0		0x1700	/* Translation Buffer 7 Page.	*/
#define MC_TB7_PAGE1		0x1740	/* Translation Buffer 7 Page.	*/
#define MC_TB7_PAGE2		0x1780	/* Translation Buffer 7 Page.	*/
#define MC_TB7_PAGE3		0x17C0	/* Translation Buffer 7 Page.	*/


/* ======================================================================
 * =   		BIT EXTENT FOR CIA_CTRL REGISTER (87.4000.0100)		=
 * ======================================================================
*/
#define MC_CIA_CTRL_V_PCI_EN		0
#define MC_CIA_CTRL_M_PCI_EN 		(1 << MC_CIA_CTRL_V_PCI_EN)
#define MC_CIA_CTRL_V_PCI_LOCK_EN	1
#define MC_CIA_CTRL_M_PCI_LOCK_EN 	(1 << MC_CIA_CTRL_V_PCI_LOCK_EN)
#define MC_CIA_CTRL_V_PCI_LOOP_EN	2
#define MC_CIA_CTRL_M_PCI_LOOP_EN 	(1 << MC_CIA_CTRL_V_PCI_LOOP_EN)
#define MC_CIA_CTRL_V_FST_BB_EN		3
#define MC_CIA_CTRL_M_FST_BB_EN 	(1 << MC_CIA_CTRL_V_FST_BB_EN)
#define MC_CIA_CTRL_V_PCI_MST_EN	4
#define MC_CIA_CTRL_M_PCI_MST_EN 	(1 << MC_CIA_CTRL_V_PCI_MST_EN)
#define MC_CIA_CTRL_V_PCI_MEM_EN	5
#define MC_CIA_CTRL_M_PCI_MEM_EN 	(1 << MC_CIA_CTRL_V_PCI_MEM_EN)
#define MC_CIA_CTRL_V_PCI_REQ64_EN	6
#define MC_CIA_CTRL_M_PCI_REQ64_EN 	(1 << MC_CIA_CTRL_V_PCI_REQ64_EN)
#define MC_CIA_CTRL_V_PCI_ACK64_EN	7
#define MC_CIA_CTRL_M_PCI_ACK64_EN 	(1 << MC_CIA_CTRL_V_PCI_ACK64_EN)
#define MC_CIA_CTRL_V_ADDR_PE_EN	8
#define MC_CIA_CTRL_M_ADDR_PE_EN 	(1 << MC_CIA_CTRL_V_ADDR_PE_EN)
#define MC_CIA_CTRL_V_PERR_EN		9
#define MC_CIA_CTRL_M_PERR_EN 		(1 << MC_CIA_CTRL_V_PERR_EN)
#define MC_CIA_CTRL_V_FILLERR_EN	10
#define MC_CIA_CTRL_M_FILLERR_EN 	(1 << MC_CIA_CTRL_V_FILLERR_EN)
#define MC_CIA_CTRL_V_MCHKERR_EN	11
#define MC_CIA_CTRL_M_MCHKERR_EN 	(1 << MC_CIA_CTRL_V_MCHKERR_EN)
#define MC_CIA_CTRL_V_ECC_CHK_EN	12
#define MC_CIA_CTRL_M_ECC_CHK_EN 	(1 << MC_CIA_CTRL_V_ECC_CHK_EN)
#define MC_CIA_CTRL_V_ASSERT_IDLE_BC	13
#define MC_CIA_CTRL_M_ASSERT_IDLE_BC 	(1 << MC_CIA_CTRL_V_ASSERT_IDLE_BC)
#define MC_CIA_CTRL_V_CON_IDLE_BC	14
#define MC_CIA_CTRL_M_CON_IDLE_BC 	(1 << MC_CIA_CTRL_V_CON_IDLE_BC)
#define MC_CIA_CTRL_V_CSR_IOA_BYPASS 	15
#define MC_CIA_CTRL_M_CSR_IOA_BYPASS  	(1 << MC_CIA_CTRL_V_CSR_IOA_BYPASS )
#define MC_CIA_CTRL_V_IO_FLUSHREQ_EN 	16
#define MC_CIA_CTRL_M_IO_FLUSHREQ_EN	(1 << MC_CIA_CTRL_V_IO_FLUSHREQ_EN )
#define MC_CIA_CTRL_V_CPU_FLUSHREQ_EN 	17
#define MC_CIA_CTRL_M_CPU_FLUSHREQ_EN 	(1 << MC_CIA_CTRL_V_CPU_FLUSHREQ_EN )
#define MC_CIA_CTRL_V_ARB_EV5_EN	18
#define MC_CIA_CTRL_M_ARB_EV5_EN 	(1 << MC_CIA_CTRL_V_ARB_EV5_EN)
#define MC_CIA_CTRL_V_EN_ARB_LINK	19
#define MC_CIA_CTRL_M_EN_ARB_LINK 	(1 << MC_CIA_CTRL_V_EN_ARB_LINK)
#define MC_CIA_CTRL_V_RD_TYPE		20
#define MC_CIA_CTRL_M_RD_TYPE 		(3 << MC_CIA_CTRL_V_RD_TYPE)
#define MC_CIA_CTRL_V_RL_TYPE		24
#define MC_CIA_CTRL_M_RL_TYPE 		(3 << MC_CIA_CTRL_V_RL_TYPE)
#define MC_CIA_CTRL_V_RM_TYPE		28
#define MC_CIA_CTRL_M_RM_TYPE 		(3 << MC_CIA_CTRL_V_RM_TYPE)


/* ======================================================================
 * =   		BIT EXTENT FOR CACK_EN REGISTER (87.4000.0600)		=
 * ======================================================================
*/
#define MC_CACK_EN_V_MEM_LOCK	0x0	/* Controls LOCK enable.	*/
#define MC_CACK_EN_V_MB		0x1	/* Controls MB enable.		*/
#define MC_CACK_EN_V_SET_DIRTY	0x2	/* Controls SET_DIRTY enable.	*/
#define MC_CACK_EN_V_BC_VICTIM	0x3	/* Controls BC_VICTIM enable.	*/


/* ======================================================================
 * =   		BIT SIZES FOR CACK_EN REGISTER (87.4000.0600)		=
 * ======================================================================
*/
#define MC_CACK_EN_S_MEM_LOCK	1	/* Controls LOCK enable.	*/
#define MC_CACK_EN_S_MB		1	/* Controls MB enable.		*/
#define MC_CACK_EN_S_SET_DIRTY	1	/* Controls SET_DIRTY enable.	*/
#define MC_CACK_EN_S_BC_VICTIM	1	/* Controls BC_VICTIM enable.	*/

/* ======================================================================
 * =   		BIT MASKS FOR CACK_EN REGISTER (87.4000.0600)		=
 * ======================================================================
*/
#define MC_CACK_EN_M_MEM_LOCK	(((1<<MC_CACK_EN_S_MEM_LOCK)-1) << MC_CACK_EN_V_MEM_LOCK)
#define MC_CACK_EN_M_MB		(((1<<MC_CACK_EN_S_MB)-1) << MC_CACK_EN_V_MB)
#define MC_CACK_EN_M_SET_DIRTY	(((1<<MC_CACK_EN_S_SET_DIRTY)-1) << MC_CACK_EN_V_SET_DIRTY)
#define MC_CACK_EN_M_BC_VICTIM	(((1<<MC_CACK_EN_S_BC_VICTIM)-1) << MC_CACK_EN_V_BC_VICTIM)


/* ======================================================================
 * =   		BIT EXTENT FOR MCR REGISTER (87.5000.0000)		=
 * ======================================================================
*/
#define MC_MCR_V_MEM_SIZE	0x0	/* 1 = 256-bit data, 0 = 128-bit*/
#define MC_MCR_V_CACHE_SIZE	0x4	/* Set to match CPU's cache size*/
#define MC_MCR_V_REF_RATE	0x8	/* Refresh rate counter.	*/
#define MC_MCR_V_REF_BURST	0x18	/* Refresh burst.		*/
#define MC_MCR_V_TMG_R0		0x20	/* Row address setup.		*/
#define MC_MCR_V_LONG_CBR_CAS	0x22	/* Refresh CAS pulse width.	*/
#define MC_MCR_V_DLY_IDLE_BC	0x26	/* ??.				*/
#define MC_MCR_V_EARLY_IDLE_BC	0x29	/* ??.				*/


/* ======================================================================
 * =   		  BIT SIZES FOR MCR REGISTER (87.5000.0000)		=
 * ======================================================================
*/
#define MC_MCR_S_MEM_SIZE	1
#define MC_MCR_S_CACHE_SIZE	3
#define MC_MCR_S_REF_RATE	10
#define MC_MCR_S_REF_BURST	2
#define MC_MCR_S_TMG_R0		2
#define MC_MCR_S_LONG_CBR_CAS	1
#define MC_MCR_S_DLY_IDLE_BC	2
#define MC_MCR_S_EARLY_IDLE_BC	1

/* ======================================================================
 * =   		 BIT MASKS FOR MCR REGISTER (87.5000.0000)		=
 * ======================================================================
*/
#define MC_MCR_M_MEM_SIZE	(((1<<MC_MCR_S_MEM_SIZE)-1) << MC_MCR_V_MEM_SIZE)
#define MC_MCR_M_CACHE_SIZE	(((1<<MC_MCR_S_CACHE_SIZE)-1) << MC_MCR_V_CACHE_SIZE)
#define MC_MCR_M_REF_RATE	(((1<<MC_MCR_S_REF_RATE)-1) << MC_MCR_V_REF_RATE)
#define MC_MCR_M_REF_BURST	(((1<<MC_MCR_S_REF_BURST)-1) << MC_MCR_V_REF_BURST)
#define MC_MCR_M_TMG_R0		(((1<<MC_MCR_S_TMG_R0)-1) << MC_MCR_V_TMG_R0)
#define MC_MCR_M_LONG_CBR_CAS	(((1<<MC_MCR_S_LONG_CBR_CAS)-1) << MC_MCR_V_LONG_CBR_CAS)
#define MC_MCR_M_DLY_IDLE_BC	(((1<<MC_MCR_S_DLY_IDLE_BC)-1) << MC_MCR_V_DLY_IDLE_BC)
#define MC_MCR_M_EARLY_IDLE_BC	(((1<<MC_MCR_S_EARLY_IDLE_BC)-1) << MC_MCR_V_EARLY_IDLE_BC)

/* ======================================================================
 * =   		BIT EXTENT FOR TIMING REGISTERS 			=
 * ======================================================================
*/
#define MC_TMG_V_R1	0		/*  [1:0] row address hold  	*/
#define MC_TMG_V_R2	2		/*  [3:2] row address hold	*/
#define MC_TMG_V_R3	4		/*  [5:4] read, cycle time 	*/
#define MC_TMG_V_R4	6		/*  [7:6] read, delay from 	*/
                                        /*  MEM_RD_RDY to CAS assertion	*/
#define MC_TMG_V_R5	8		/*  [9:8] read, cas pulse width	*/
#define MC_TMG_V_R6	10		/*  [11:10] read, column address hold*/
#define MC_TMG_V_W1	12		/*  [13:12] writes, non-linked 	*/
                                        /* victim, delay from start to MEM_WR */
#define MC_TMG_V_W4	14		/*[16:14] writes, MEM_WR to CAS assert*/
#define MC_TMG_V_PRE	17		/* [17] RAS Pre-charge delay 	*/
                                        /* __0=no delay, 1=delay 30 ns	*/
#define MC_TMG_V_V3	18		/*  [19:18] write, cycle time 	*/
#define MC_TMG_V_V4	20		/*  [22:20] linked victim,  	*/
                                        /*  MEM_WR to CAS assertion	*/
#define MC_TMG_V_V5	24		/*  [25:24] write, victim, CAS 	*/
                                        /*  pulse width			*/
#define MC_TMG_V_V6	26		/*  [27:26] victim/write, column*/
                                        /*  address hold		*/
#define MC_TMG_V_RV	28		/*  [29:28] read-victim, delay 	*/
                                        /*  from MEM_RD_RDY de-assertion*/
                                        /*  to MEM_WR			*/
#define MC_TMG_V_RD_DLY	30		/*  [31:30] select clk the dsw 	*/
                                        /*  will use to capture memory 	*/
                                        /*  data 0=CLK, 1=CLK2X		*/


/* ======================================================================
 * =   		BIT EXTENT FOR MBA REGISTERS (87.5000.0600 + n*80)	=
 * ======================================================================
*/
#define MC_MBA_V_S0_VALID	0	/* Side 0 for the bank is valid	*/
#define MC_MBA_V_ROW_TYPE	1       /* Row and Column configuration	*/
#define MC_MBA_V_MASK		4	/* Indicates size of mem SIMMs	*/
#define MC_MBA_V_S1_VALID	15   	/* Side 1 for the bank is valid	*/
#define MC_MBA_V_PATTERN	16      /* Base address of the bank	*/
#define MC_MBA_V_TMG_SEL	28      /* Select timing register.	*/

/* ======================================================================
 * =   	  BIT FIELD VALUES FOR MBA REGISTERS (87.5000.0600 + n*80)	=
 * ======================================================================
*/
#define MC_MBA_V_ROW_TYPE_10X10	0       /* 10 row, 10 column		*/
#define MC_MBA_V_ROW_TYPE_12X10	1       /* 12 row, 10 column		*/
#define MC_MBA_V_ROW_TYPE_11X11	1       /* 11 row, 11 column		*/
#define MC_MBA_V_ROW_TYPE_13X11	2       /* 13 row, 11 column		*/
#define MC_MBA_V_ROW_TYPE_12X12	2       /* 12 row, 12 column		*/

#define MC_MBA_V_TMG_SEL_TMG0	0	/* Select timing register 0.	*/
#define MC_MBA_V_TMG_SEL_TMG1	1	/* Select timing register 1.	*/
#define MC_MBA_V_TMG_SEL_TMG2	2	/* Select timing register 2.	*/


/* ======================================================================
 * =   		BIT SIZES FOR MBA REGISTERS (87.5000.0600 + n*80)	=
 * ======================================================================
*/
#define MC_MBA_S_S0_VALID	1
#define MC_MBA_S_ROW_TYPE	2
#define MC_MBA_S_MASK		5
#define MC_MBA_S_S1_VALID	1
#define MC_MBA_S_PATTERN	10
#define MC_MBA_S_TMG_SEL	2

/* ======================================================================
 * =   		 BIT MASKS FOR MBA REGISTERS (87.5000.0600 + n*80)	=
 * ======================================================================
*/
#define MC_MBA_M_S0_VALID	(((1<<MC_MBA_S_S0_VALID)-1) << MC_MBA_V_S0_VALID)
#define MC_MBA_M_ROW_TYPE	(((1<<MC_MBA_S_ROW_TYPE)-1) << MC_MBA_V_ROW_TYPE)
#define MC_MBA_M_MASK		(((1<<MC_MBA_S_MASK)-1) << MC_MBA_V_MASK)
#define MC_MBA_M_S1_VALID	(((1<<MC_MBA_S_S1_VALID)-1) << MC_MBA_V_S1_VALID)
#define MC_MBA_M_PATTERN	(((1<<MC_MBA_S_PATTERN)-1) << MC_MBA_V_PATTERN)
#define MC_MBA_M_TMG_SEL	(((1<<MC_MBA_S_TMG_SEL)-1) << MC_MBA_V_TMG_SEL)



#define cia_k_main_csr_base 	0x0874	/* CIA General Control Register Base*/
#define cia_v_main_csr_base 	28	/* Shift base by this # of bits */

/* Offsets from CIA control base register. 				*/

#define hae_mem			0x0400
#define hae_io			0x0440

#define cia_err			0x8200
#define cia_err_mask		0x8280
#define cia_err_v_rcvd_mas_abt	7
#define cia_err_m_rcvd_mas_abt (1 << cia_err_v_rcvd_mas_abt)
#define cia_err_mask_v_mas_abt	7
#define cia_err_mask_m_mas_abt	(1 << cia_err_mask_v_mas_abt)

#define cia_k_addr_trans	0x876	/* CIA's PCI Address Translation Regs*/
#define cia_v_addr_trans	28	/* Shift base by this # of bits */

/* Offsets from PCI address translation base register.	Must be < 0x8000 */
/* if they are to be used with LDA instructions!			*/
#define w_base0			0x0400
#define w_mask0			0x0440
#define t_base0			0x0480
#define w_base1			0x0500
#define w_mask1			0x0540
#define t_base1			0x0580
#define w_base2			0x0600
#define w_mask2			0x0640
#define t_base2			0x0680
#define w_base3			0x0700
#define w_mask3			0x0740
#define t_base3			0x0780

#define w_v_en			0x0
#define w_m_en			(1 << w_v_en)



/* CIA Memory Control Register. */
#define mcr			0
#define mba			0x600
#define mba_v_disp		0x7
#define mba_k_disp		0x80 	/* 1 << mba_v_disp */



/* The following constants define which bits are provided by the HAE_MEM*/
/* register for each of the three sparse regions.			*/
#define hae_sp1_bits		0xE0000000
#define hae_sp2_bits		0xF8000000
#define hae_sp3_bits		0xFC000000

#endif /* __CIA_H_LOADED */
