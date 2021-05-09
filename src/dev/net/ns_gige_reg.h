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
 * Ethernet device register definitions for the National
 * Semiconductor DP83820 Ethernet controller
 */

#ifndef __DEV_NS_GIGE_REG_H__
#define __DEV_NS_GIGE_REG_H__

namespace gem5
{

/* Device Register Address Map */
enum DeviceRegisterAddress
{
    CR =                0x00,
    CFGR =              0x04,
    MEAR =              0x08,
    PTSCR =             0x0c,
    ISR =               0x10,
    IMR =               0x14,
    IER =               0x18,
    IHR =               0x1c,
    TXDP =              0x20,
    TXDP_HI =           0x24,
    TX_CFG =            0x28,
    GPIOR =             0x2c,
    RXDP =              0x30,
    RXDP_HI =           0x34,
    RX_CFG =            0x38,
    PQCR =              0x3c,
    WCSR =              0x40,
    PCR =               0x44,
    RFCR =              0x48,
    RFDR =              0x4c,
    BRAR =              0x50,
    BRDR =              0x54,
    SRR =               0x58,
    MIBC =              0x5c,
    MIB_START =         0x60,
    MIB_END =           0x88,
    VRCR =              0xbc,
    VTCR =              0xc0,
    VDR =               0xc4,
    CCSR =              0xcc,
    TBICR =             0xe0,
    TBISR =             0xe4,
    TANAR =             0xe8,
    TANLPAR =           0xec,
    TANER =             0xf0,
    TESR =              0xf4,
    M5REG =             0xf8,
    LAST =              0xf8,
    RESERVED =          0xfc
};

/* Chip Command Register */
enum ChipCommandRegister
{
     CR_TXE =           0x00000001,
     CR_TXD =           0x00000002,
     CR_RXE =           0x00000004,
     CR_RXD =           0x00000008,
     CR_TXR =           0x00000010,
     CR_RXR =           0x00000020,
     CR_SWI =           0x00000080,
     CR_RST =           0x00000100
};

/* configuration register */
enum ConfigurationRegisters
{
     CFGR_ZERO =        0x00000000,
     CFGR_LNKSTS =      0x80000000,
     CFGR_SPDSTS =      0x60000000,
     CFGR_SPDSTS1 =     0x40000000,
     CFGR_SPDSTS0 =     0x20000000,
     CFGR_DUPSTS =      0x10000000,
     CFGR_TBI_EN =      0x01000000,
     CFGR_RESERVED =    0x0e000000,
     CFGR_MODE_1000 =   0x00400000,
     CFGR_AUTO_1000 =   0x00200000,
     CFGR_PINT_CTL =    0x001c0000,
     CFGR_PINT_DUPSTS = 0x00100000,
     CFGR_PINT_LNKSTS = 0x00080000,
     CFGR_PINT_SPDSTS = 0x00040000,
     CFGR_TMRTEST =     0x00020000,
     CFGR_MRM_DIS =     0x00010000,
     CFGR_MWI_DIS =     0x00008000,
     CFGR_T64ADDR =     0x00004000,
     CFGR_PCI64_DET =   0x00002000,
     CFGR_DATA64_EN =   0x00001000,
     CFGR_M64ADDR =     0x00000800,
     CFGR_PHY_RST =     0x00000400,
     CFGR_PHY_DIS =     0x00000200,
     CFGR_EXTSTS_EN =   0x00000100,
     CFGR_REQALG =      0x00000080,
     CFGR_SB =          0x00000040,
     CFGR_POW =         0x00000020,
     CFGR_EXD =         0x00000010,
     CFGR_PESEL =       0x00000008,
     CFGR_BROM_DIS =    0x00000004,
     CFGR_EXT_125 =     0x00000002,
     CFGR_BEM =         0x00000001
};

/* EEPROM access register */
enum EEPROMAccessRegister
{
     MEAR_EEDI =        0x00000001,
     MEAR_EEDO =        0x00000002,
     MEAR_EECLK =       0x00000004,
     MEAR_EESEL =       0x00000008,
     MEAR_MDIO =        0x00000010,
     MEAR_MDDIR =       0x00000020,
     MEAR_MDC =         0x00000040,
};

/* PCI test control register */
enum PCITestControlRegister
{
     PTSCR_EEBIST_FAIL =        0x00000001,
     PTSCR_EEBIST_EN =          0x00000002,
     PTSCR_EELOAD_EN =          0x00000004,
     PTSCR_RBIST_FAIL =         0x000001b8,
     PTSCR_RBIST_DONE =         0x00000200,
     PTSCR_RBIST_EN =           0x00000400,
     PTSCR_RBIST_RST =          0x00002000,
     PTSCR_RBIST_RDONLY =       0x000003f9
};

/* interrupt status register */
enum InterruptStatusRegister
{
     ISR_RESERVE =      0x80000000,
     ISR_TXDESC3 =      0x40000000,
     ISR_TXDESC2 =      0x20000000,
     ISR_TXDESC1 =      0x10000000,
     ISR_TXDESC0 =      0x08000000,
     ISR_RXDESC3 =      0x04000000,
     ISR_RXDESC2 =      0x02000000,
     ISR_RXDESC1 =      0x01000000,
     ISR_RXDESC0 =      0x00800000,
     ISR_TXRCMP =       0x00400000,
     ISR_RXRCMP =       0x00200000,
     ISR_DPERR =        0x00100000,
     ISR_SSERR =        0x00080000,
     ISR_RMABT =        0x00040000,
     ISR_RTAB =         0x00020000,
     ISR_RXSOVR =       0x00010000,
     ISR_HIBINT =       0x00008000,
     ISR_PHY =          0x00004000,
     ISR_PME =          0x00002000,
     ISR_SWI =          0x00001000,
     ISR_MIB =          0x00000800,
     ISR_TXURN =        0x00000400,
     ISR_TXIDLE =       0x00000200,
     ISR_TXERR =        0x00000100,
     ISR_TXDESC =       0x00000080,
     ISR_TXOK =         0x00000040,
     ISR_RXORN =        0x00000020,
     ISR_RXIDLE =       0x00000010,
     ISR_RXEARLY =      0x00000008,
     ISR_RXERR =        0x00000004,
     ISR_RXDESC =       0x00000002,
     ISR_RXOK =         0x00000001,
     ISR_ALL =          0x7FFFFFFF,
     ISR_DELAY =        (ISR_TXIDLE|ISR_TXDESC|ISR_TXOK|
                         ISR_RXIDLE|ISR_RXDESC|ISR_RXOK),
     ISR_NODELAY =      (ISR_ALL & ~ISR_DELAY),
     ISR_IMPL =         (ISR_SWI|ISR_TXIDLE|ISR_TXDESC|ISR_TXOK|ISR_RXORN|
                         ISR_RXIDLE|ISR_RXDESC|ISR_RXOK),
     ISR_NOIMPL =       (ISR_ALL & ~ISR_IMPL)
};

/* transmit configuration register */
enum TransmitConfigurationRegister
{
     TX_CFG_CSI =       0x80000000,
     TX_CFG_HBI =       0x40000000,
     TX_CFG_MLB =       0x20000000,
     TX_CFG_ATP =       0x10000000,
     TX_CFG_ECRETRY =   0x00800000,
     TX_CFG_BRST_DIS =  0x00080000,
     TX_CFG_MXDMA1024 = 0x00000000,
     TX_CFG_MXDMA512 =  0x00700000,
     TX_CFG_MXDMA256 =  0x00600000,
     TX_CFG_MXDMA128 =  0x00500000,
     TX_CFG_MXDMA64 =   0x00400000,
     TX_CFG_MXDMA32 =   0x00300000,
     TX_CFG_MXDMA16 =   0x00200000,
     TX_CFG_MXDMA8 =    0x00100000,
     TX_CFG_MXDMA =     0x00700000,

     TX_CFG_FLTH_MASK = 0x0000ff00,
     TX_CFG_DRTH_MASK = 0x000000ff
};

/*general purpose I/O control register */
enum GeneralPurposeIOControlRegister
{
     GPIOR_UNUSED =     0xffff8000,
     GPIOR_GP5_IN =     0x00004000,
     GPIOR_GP4_IN =     0x00002000,
     GPIOR_GP3_IN =     0x00001000,
     GPIOR_GP2_IN =     0x00000800,
     GPIOR_GP1_IN =     0x00000400,
     GPIOR_GP5_OE =     0x00000200,
     GPIOR_GP4_OE =     0x00000100,
     GPIOR_GP3_OE =     0x00000080,
     GPIOR_GP2_OE =     0x00000040,
     GPIOR_GP1_OE =     0x00000020,
     GPIOR_GP5_OUT =    0x00000010,
     GPIOR_GP4_OUT =    0x00000008,
     GPIOR_GP3_OUT =    0x00000004,
     GPIOR_GP2_OUT =    0x00000002,
     GPIOR_GP1_OUT =    0x00000001
};

/* receive configuration register */
enum ReceiveConfigurationRegister
{
     RX_CFG_AEP =       0x80000000,
     RX_CFG_ARP =       0x40000000,
     RX_CFG_STRIPCRC =  0x20000000,
     RX_CFG_RX_FD =     0x10000000,
     RX_CFG_ALP =       0x08000000,
     RX_CFG_AIRL =      0x04000000,
     RX_CFG_MXDMA512 =  0x00700000,
     RX_CFG_MXDMA =     0x00700000,
     RX_CFG_DRTH =      0x0000003e,
     RX_CFG_DRTH0 =     0x00000002
};

/* pause control status register */
enum PauseControlStatusRegister
{
     PCR_PSEN =         (1 << 31),
     PCR_PS_MCAST =     (1 << 30),
     PCR_PS_DA =        (1 << 29),
     PCR_STHI_8 =       (3 << 23),
     PCR_STLO_4 =       (1 << 23),
     PCR_FFHI_8K =      (3 << 21),
     PCR_FFLO_4K =      (1 << 21),
     PCR_PAUSE_CNT =    0xFFFE
};

/*receive filter/match control register */
enum ReceiveFilterMatchControlRegister
{
     RFCR_RFEN =        0x80000000,
     RFCR_AAB =         0x40000000,
     RFCR_AAM =         0x20000000,
     RFCR_AAU =         0x10000000,
     RFCR_APM =         0x08000000,
     RFCR_APAT =        0x07800000,
     RFCR_APAT3 =       0x04000000,
     RFCR_APAT2 =       0x02000000,
     RFCR_APAT1 =       0x01000000,
     RFCR_APAT0 =       0x00800000,
     RFCR_AARP =        0x00400000,
     RFCR_MHEN =        0x00200000,
     RFCR_UHEN =        0x00100000,
     RFCR_ULM =         0x00080000,
     RFCR_RFADDR =      0x000003ff
};

/* receive filter/match data register */
enum ReceiveFilterMatchDataRegister
{
     RFDR_BMASK =       0x00030000,
     RFDR_RFDATA0 =     0x000000ff,
     RFDR_RFDATA1 =     0x0000ff00
};

/* management information base control register */
enum ManagementInformationBaseControlRegister
{
     MIBC_MIBS =        0x00000008,
     MIBC_ACLR =        0x00000004,
     MIBC_FRZ =         0x00000002,
     MIBC_WRN =         0x00000001
};

/* VLAN/IP receive control register */
enum VLANIPReceiveControlRegister
{
     VRCR_RUDPE =       0x00000080,
     VRCR_RTCPE =       0x00000040,
     VRCR_RIPE =        0x00000020,
     VRCR_IPEN =        0x00000010,
     VRCR_DUTF =        0x00000008,
     VRCR_DVTF =        0x00000004,
     VRCR_VTREN =       0x00000002,
     VRCR_VTDEN =       0x00000001
};

/* VLAN/IP transmit control register */
enum VLANIPTransmitControlRegister
{
     VTCR_PPCHK =       0x00000008,
     VTCR_GCHK =        0x00000004,
     VTCR_VPPTI =       0x00000002,
     VTCR_VGTI =        0x00000001
};

/* Clockrun Control/Status Register */
enum ClockrunControlStatusRegister
{
     CCSR_CLKRUN_EN =   0x00000001
};

/* TBI control register */
enum TBIControlRegister
{
     TBICR_MR_LOOPBACK =        0x00004000,
     TBICR_MR_AN_ENABLE =       0x00001000,
     TBICR_MR_RESTART_AN =      0x00000200
};

/* TBI status register */
enum TBIStatusRegister
{
     TBISR_MR_LINK_STATUS =     0x00000020,
     TBISR_MR_AN_COMPLETE =     0x00000004
};

/* TBI auto-negotiation advertisement register */
enum TBIAutoNegotiationAdvertisementRegister
{
     TANAR_NP =         0x00008000,
     TANAR_RF2 =        0x00002000,
     TANAR_RF1 =        0x00001000,
     TANAR_PS2 =        0x00000100,
     TANAR_PS1 =        0x00000080,
     TANAR_HALF_DUP =   0x00000040,
     TANAR_FULL_DUP =   0x00000020,
     TANAR_UNUSED =     0x00000E1F
};

/* M5 control register */
enum M5ControlRegister
{
     M5REG_RESERVED =   0xfffffffc,
     M5REG_RSS =        0x00000004,
     M5REG_RX_THREAD =  0x00000002,
     M5REG_TX_THREAD =  0x00000001
};

struct ns_desc32
{
    uint32_t link;    /* link field to next descriptor in linked list */
    uint32_t bufptr;  /* pointer to the first fragment or buffer */
    uint32_t cmdsts;  /* command/status field */
    uint32_t extsts;  /* extended status field for VLAN and IP info */
};

struct ns_desc64
{
    uint64_t link;    /* link field to next descriptor in linked list */
    uint64_t bufptr;  /* pointer to the first fragment or buffer */
    uint32_t cmdsts;  /* command/status field */
    uint32_t extsts;  /* extended status field for VLAN and IP info */
};

/* cmdsts flags for descriptors */
enum CMDSTSFlatsForDescriptors
{
     CMDSTS_OWN =       0x80000000,
     CMDSTS_MORE =      0x40000000,
     CMDSTS_INTR =      0x20000000,
     CMDSTS_ERR =       0x10000000,
     CMDSTS_OK =        0x08000000,
     CMDSTS_LEN_MASK =  0x0000ffff,

     CMDSTS_DEST_MASK = 0x01800000,
     CMDSTS_DEST_SELF = 0x00800000,
     CMDSTS_DEST_MULTI = 0x01000000
};

/* extended flags for descriptors */
enum ExtendedFlagsForDescriptors
{
     EXTSTS_UDPERR =    0x00400000,
     EXTSTS_UDPPKT =    0x00200000,
     EXTSTS_TCPERR =    0x00100000,
     EXTSTS_TCPPKT =    0x00080000,
     EXTSTS_IPERR =     0x00040000,
     EXTSTS_IPPKT =     0x00020000
};

/* speed status */
static inline int
SPDSTS_POLARITY(int lnksts)
{
    return (CFGR_SPDSTS1 | CFGR_SPDSTS0 | CFGR_DUPSTS |
            (lnksts ? CFGR_LNKSTS : CFGR_ZERO));
}

} // namespace gem5

#endif /* __DEV_NS_GIGE_REG_H__ */
