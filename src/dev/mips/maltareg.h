/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

/** @file
 * List of Tsunami CSRs
 */

// NEEDS TO BE ADJUSTED FOR MALTA BOARD

#ifndef __MALTAREG_H__
#define __MALTAREG_H__

#define ALPHA_K0SEG_BASE  ULL(0xfffffc0000000000)

// CChip Registers
#define TSDEV_CC_CSR    0x00
#define TSDEV_CC_MTR    0x01
#define TSDEV_CC_MISC   0x02

#define TSDEV_CC_AAR0   0x04
#define TSDEV_CC_AAR1   0x05
#define TSDEV_CC_AAR2   0x06
#define TSDEV_CC_AAR3   0x07
#define TSDEV_CC_DIM0   0x08
#define TSDEV_CC_DIM1   0x09
#define TSDEV_CC_DIR0   0x0A
#define TSDEV_CC_DIR1   0x0B
#define TSDEV_CC_DRIR   0x0C
#define TSDEV_CC_PRBEN  0x0D
#define TSDEV_CC_IIC0   0x0E
#define TSDEV_CC_IIC1   0x0F
#define TSDEV_CC_MPR0   0x10
#define TSDEV_CC_MPR1   0x11
#define TSDEV_CC_MPR2   0x12
#define TSDEV_CC_MPR3   0x13

#define TSDEV_CC_DIM2   0x18
#define TSDEV_CC_DIM3   0x19
#define TSDEV_CC_DIR2   0x1A
#define TSDEV_CC_DIR3   0x1B
#define TSDEV_CC_IIC2   0x1C
#define TSDEV_CC_IIC3   0x1D

// BigTsunami Registers
#define TSDEV_CC_BDIMS  0x1000000
#define TSDEV_CC_BDIRS  0x2000000
#define TSDEV_CC_IPIQ   0x20  //0xf01a000800
#define TSDEV_CC_IPIR   0x21  //0xf01a000840
#define TSDEV_CC_ITIR   0x22  //0xf01a000880


// PChip Registers
#define TSDEV_PC_WSBA0      0x00
#define TSDEV_PC_WSBA1      0x01
#define TSDEV_PC_WSBA2      0x02
#define TSDEV_PC_WSBA3      0x03
#define TSDEV_PC_WSM0       0x04
#define TSDEV_PC_WSM1       0x05
#define TSDEV_PC_WSM2       0x06
#define TSDEV_PC_WSM3       0x07
#define TSDEV_PC_TBA0       0x08
#define TSDEV_PC_TBA1       0x09
#define TSDEV_PC_TBA2       0x0A
#define TSDEV_PC_TBA3       0x0B
#define TSDEV_PC_PCTL       0x0C
#define TSDEV_PC_PLAT       0x0D
#define TSDEV_PC_RES        0x0E
#define TSDEV_PC_PERROR     0x0F
#define TSDEV_PC_PERRMASK   0x10
#define TSDEV_PC_PERRSET    0x11
#define TSDEV_PC_TLBIV      0x12
#define TSDEV_PC_TLBIA      0x13
#define TSDEV_PC_PMONCTL    0x14
#define TSDEV_PC_PMONCNT    0x15

#define TSDEV_PC_SPST       0x20


// DChip Registers
#define TSDEV_DC_DSC        0x20
#define TSDEV_DC_STR        0x21
#define TSDEV_DC_DREV       0x22
#define TSDEV_DC_DSC2       0x23

// I/O Ports
#define TSDEV_PIC1_MASK     0x21
#define TSDEV_PIC2_MASK     0xA1
#define TSDEV_PIC1_ISR      0x20
#define TSDEV_PIC2_ISR      0xA0
#define TSDEV_PIC1_ACK      0x20
#define TSDEV_PIC2_ACK      0xA0
#define TSDEV_DMA1_RESET    0x0D
#define TSDEV_DMA2_RESET    0xDA
#define TSDEV_DMA1_MODE     0x0B
#define TSDEV_DMA2_MODE     0xD6
#define TSDEV_DMA1_MASK     0x0A
#define TSDEV_DMA2_MASK     0xD4
#define TSDEV_CTRL_PORTB    0x61
#define TSDEV_TMR0_DATA     0x40
#define TSDEV_TMR1_DATA     0x41
#define TSDEV_TMR2_DATA     0x42
#define TSDEV_TMR_CTRL      0x43
#define TSDEV_KBD           0x64
#define TSDEV_DMA1_CMND     0x08
#define TSDEV_DMA1_STAT     TSDEV_DMA1_CMND
#define TSDEV_DMA2_CMND     0xD0
#define TSDEV_DMA2_STAT     TSDEV_DMA2_CMND
#define TSDEV_DMA1_MMASK    0x0F
#define TSDEV_DMA2_MMASK    0xDE

// Added for keyboard accesses /
#define TSDEV_KBD           0x64

// Added for ATA PCI DMA /
#define ATA_PCI_DMA         0x00
#define ATA_PCI_DMA2        0x02
#define ATA_PCI_DMA3        0x16
#define ATA_PCI_DMA4        0x17
#define ATA_PCI_DMA5        0x1a
#define ATA_PCI_DMA6        0x11
#define ATA_PCI_DMA7        0x14

#define TSDEV_RTC_ADDR      0x70
#define TSDEV_RTC_DATA      0x71

#define PCHIP_PCI0_MEMORY       ULL(0x00000000000)
#define PCHIP_PCI0_IO           ULL(0x001FC000000)
#define TSUNAMI_UNCACHABLE_BIT  ULL(0x80000000000)
#define TSUNAMI_PCI0_MEMORY     TSUNAMI_UNCACHABLE_BIT + PCHIP_PCI0_MEMORY
#define TSUNAMI_PCI0_IO         TSUNAMI_UNCACHABLE_BIT + PCHIP_PCI0_IO


// UART Defines
//Relates to whether the kernel wants an interrupt when data is available
#define UART_IER_RDI            0x01
#define UART_IER_THRI           0x02
#define UART_IER_RLSI           0x04


#define UART_LSR_TEMT   0x40
#define UART_LSR_THRE   0x20
#define UART_LSR_DR     0x01

#define UART_MCR_LOOP   0x10

// System Control PortB Status Bits
#define PORTB_SPKR_HIGH 0x20

#endif // __MALTAREG_H__
