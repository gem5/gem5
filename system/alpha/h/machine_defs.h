/*
 * Copyright (C) 1998 by the Board of Trustees
 *    of Leland Stanford Junior University.
 * Copyright (C) 1998 Digital Equipment Corporation
 *
 * This file is part of the SimOS distribution.
 * See LICENSE file for terms of the license.
 *
 */

  /***********************************************************************

     machine_defs.h

   ***********************************************************************/

  /*************************************************************************
   *                                                                       *
   *               Copyright (C) 1993-1996 Stanford University             *
   *                                                                       *
   *  These coded instructions, statements, and computer programs contain  *
   *  unpublished proprietary information of Stanford University, and      *
   *  are protected by Federal copyright law.  They may not be disclosed   *
   *  to third parties or copied or duplicated in any form, in whole or    *
   *  in part, without the prior written consent of Stanford University.   *
   *                                                                       *
   *************************************************************************/

#ifndef __DPGCC__
#ifndef _HEADER_STACK_
#define _HEADER_STACK_
#endif
#endif

#ifndef _MACHINE_DEFS_H
#define _MACHINE_DEFS_H

/*
 * Created by: Dan Teodosiu, 07/96
 *
 * This header file defines the OS view of the MAGIC service address space
 * and of the device registers.
 *
 * The service address space addresses used by the OS are VIRTUAL addresses.
 * Depending on whether we simulate a 32 bit or a 64 bit processor,
 * the service address mappings will be different. MAGIC physical addresses
 * are 40 bits wide, so in 32 bit mode (current SimOS) we had to squeeze
 * the service address space into less address bits.
 *
 * There are two kinds of macros in this file:
 *
 *  - MAGIC definitions, pertaining to the services offered by the
 *    MAGIC node controller. All these macros start with MAGIC_...,
 *    Those definitions are further subdivided into ones which do not
 *    depend on the address mappings (such as MAGIC register numbers,
 *    error codes, etc.), and ones which do (such as the macros that help
 *    construct MAGIC PPR addresses).
 *    The current simulated MAGIC only supports doubleword accesses.
 *
 *  - devices register definitions, of the form DEV_... These defs
 *    describe the various device registers. Devices are accessed by
 *    performing uncached word (32bit) reads and writes to their registers.
 *
 * Notes:
 *    The macro SIMOS64BIT selects the 64 bit version of those definitions;
 *    by default, you get the 32 bit version. 64 bit is currently not
 *    implemented.
 *
 * Related documents:
 *  - FLASH: Physical Address Layout
 *  - FLASH: PP Software Services
 *  - SimOS to FLASH Physical Address Mapping
 *
 */


/***************************************************************************
 MAGIC defs which do not depend on the virtual -> physical address mapping
 ***************************************************************************/

/** zone numbering for service address space **/

#define MAGIC_ZONE_FRAM_ALIAS       0
#define MAGIC_ZONE_PPR_ALIAS        1
#define MAGIC_ZONE_PPC_ALIAS        2
#define MAGIC_ZONE_FIREWALL         3
#define MAGIC_ZONE_DMAMAP           4
#define MAGIC_ZONE_SWTLB            5
#define MAGIC_ZONE_MISSCNT          6
#define MAGIC_ZONE_NODEMAP          7
#define MAGIC_ZONE_PPR              8
#define MAGIC_ZONE_PPC              9
#define MAGIC_ZONE_MILO_ALIAS       10
#define MAGIC_ZONE_EV5_ALIAS        10
#define MAGIC_ZONE_NODECOMM         11        /* node communication zone */
                                              /* do not def zone 12: see below why */
#define MAGIC_ZONE_BDOOR_DEV        14        /* backdoor area for devices */
#define MAGIC_ZONE_FPROM_ALIAS      15

#ifndef SIMOS64BIT
#undef  MAGIC_ZONE_FPROM_ALIAS                /* In 32bit Simos, 0xbfc00000 maps to... */
#define MAGIC_ZONE_FPROM_ALIAS      12        /* ... zone 12 */
#endif

/** PPR numbering **/

/* Individual MAGIC registers */

#define MAGIC_PPR_IECHIGH            0x0000    /* r */
#define MAGIC_PPR_ACKINTERNAL        0x0001    /* w */
#define MAGIC_PPR_IECENABLE          0x0002    /* r/w */
#define MAGIC_PPR_SENDIPI            0x0003    /* w */
#define MAGIC_PPR_OPSPACE            0x0004    /* r/w */
#define MAGIC_PPR_ASID               0x0005    /* r/w */
#define MAGIC_PPR_TLBINVAL           0x0006    /* w */
#define MAGIC_PPR_TLBINUSE           0x0007    /* r */
#define MAGIC_PPR_MSGTAG             0x0008    /* r/w */
#define MAGIC_PPR_STALLOSPC          0x0009    /* r/w */
#define MAGIC_PPR_CYCLECOUNT         0x000a    /* r */
#define MAGIC_PPR_NETMSGTIME         0x000b    /* r */
#define MAGIC_PPR_RESERVED_C         0x000c    /* */
#define MAGIC_PPR_RESERVED_D         0x000d    /* */
#define MAGIC_PPR_RESERVED_E         0x000e    /* */
#define MAGIC_PPR_RESERVED_F         0x000f    /* */

#define MAGIC_PPR_UNUSED10           0x0010    /* r */
#define MAGIC_PPR_PROTVERSION        0x0011    /* r */
#define MAGIC_PPR_HWVERSION          0x0012    /* r */
#define MAGIC_PPR_REMAPMASK          0x0013    /* r/w */
#define MAGIC_PPR_PROTCONTROL        0x0014    /* r/w */
#define MAGIC_PPR_RESERVED_15        0x0015    /* */
#define MAGIC_PPR_RESERVED_16        0x0016    /* */
#define MAGIC_PPR_RESERVED_17        0x0017    /* */
#define MAGIC_PPR_OUTOFRANGE         0x0018    /* r */
#define MAGIC_PPR_INTERVAL           0x0019    /* r/w */
#define MAGIC_PPR_SLOTMAP            0x001a    /* r/w */
#define    MAGIC_SLOTMAP_SLOT0_OFFS         0
#define    MAGIC_SLOTMAP_SLOT0_MASK         0x00000000000000FFLL
#define    MAGIC_SLOTMAP_SLOT1_OFFS         8
#define    MAGIC_SLOTMAP_SLOT1_MASK         0x000000000000FF00LL
#define    MAGIC_SLOTMAP_SLOT2_OFFS         16
#define    MAGIC_SLOTMAP_SLOT2_MASK         0x0000000000FF0000LL
#define    MAGIC_SLOTMAP_SLOT3_OFFS         24
#define    MAGIC_SLOTMAP_SLOT3_MASK         0x00000000FF000000LL
#define MAGIC_PPR_FWSHIFT            0x001b    /* r/w */
#define MAGIC_PPR_RECOVERYSYNC       0x001c    /* r */
#define    MAGIC_RECOVERYSYNC_PHASE_MASK    0xF000000000000000LL;
#define    MAGIC_RECOVERYSYNC_PHASE_SHIFT   60
#define    MAGIC_RECOVERYSYNC_PHASE_ZERO    (0x0LL << MAGIC_RECOVERYSYNC_PHASE_SHIFT)
#define    MAGIC_RECOVERYSYNC_PHASE_ONE     (0x1LL << MAGIC_RECOVERYSYNC_PHASE_SHIFT)
#define    MAGIC_RECOVERYSYNC_PHASE_TWO     (0x2LL << MAGIC_RECOVERYSYNC_PHASE_SHIFT)
#define    MAGIC_RECOVERYSYNC_PHASE_THREE   (0x3LL << MAGIC_RECOVERYSYNC_PHASE_SHIFT)
#define    MAGIC_RECOVERYSYNC_TIMESTAMP_MASK  0x0FFFFFFFFFFFFFFFLL
#define    MAGIC_RECOVERYSYNC_TIMESTAMP_SHIFT 0
#define MAGIC_PPR_REPORT_DIAG_RESULT 0x001d    /* w */
#define    MAGIC_REPORT_PASS_DIAG           0  /* other values indicate fail */
#define MAGIC_PPR_RESERVED_1E        0x001e    /* w */
#define MAGIC_PPR_DRAIN_POLL         0x001f    /* r */

#define MAGIC_PPR_NODECONFIG         0x0020    /* r */
#define    MAGIC_NODECONFIG_THISNODE_OFFS    0
#define    MAGIC_NODECONFIG_THISNODE_MASK    0x0000000000000fffLL
#define    MAGIC_NODECONFIG_FIRSTNODE_OFFS   12
#define    MAGIC_NODECONFIG_FIRSTNODE_MASK   0x0000000000fff000LL
#define    MAGIC_NODECONFIG_NODESINCELL_OFFS 24
#define    MAGIC_NODECONFIG_NODESINCELL_MASK 0x0000000fff000000LL
#define    MAGIC_NODECONFIG_THISCELL_OFFS    36
#define    MAGIC_NODECONFIG_THISCELL_MASK    0x0000fff000000000LL
#define    MAGIC_NODECONFIG_NCELLS_OFFS      48
#define    MAGIC_NODECONFIG_NCELLS_MASK      0x0fff000000000000LL

#define    MAGIC_NODECONFIG_THISNODE(val)                                  \
 (((val)&MAGIC_NODECONFIG_THISNODE_MASK)>>MAGIC_NODECONFIG_THISNODE_OFFS)
#define    MAGIC_NODECONFIG_FIRSTNODE(val)                                 \
 (((val)&MAGIC_NODECONFIG_FIRSTNODE_MASK)>>MAGIC_NODECONFIG_FIRSTNODE_OFFS)
#define    MAGIC_NODECONFIG_NODESINCELL(val)                               \
 (((val)&MAGIC_NODECONFIG_NODESINCELL_MASK)>>MAGIC_NODECONFIG_NODESINCELL_OFFS)
#define    MAGIC_NODECONFIG_THISCELL(val)                                  \
 (((val)&MAGIC_NODECONFIG_THISCELL_MASK)>>MAGIC_NODECONFIG_THISCELL_OFFS)
#define    MAGIC_NODECONFIG_NCELLS(val)                                    \
 (((val)&MAGIC_NODECONFIG_NCELLS_MASK)>>MAGIC_NODECONFIG_NCELLS_OFFS)

#define MAGIC_PPR_ADDRCONFIG         0x0021    /* r */
#define    MAGIC_ADDRCONFIG_PAGES_OFFS       0
#define    MAGIC_ADDRCONFIG_PAGES_MASK       0x0000ffffffffLL
#define    MAGIC_ADDRCONFIG_NNBITS_OFFS      32
#define    MAGIC_ADDRCONFIG_NNBITS_MASK      0x00ff00000000LL
#define    MAGIC_ADDRCONFIG_MASBITS_OFFS     40
#define    MAGIC_ADDRCONFIG_MASBITS_MASK     0xff0000000000LL

#define    MAGIC_ADDRCONFIG_PAGES(val) \
 (((val)&MAGIC_ADDRCONFIG_PAGES_MASK)>>MAGIC_ADDRCONFIG_PAGES_OFFS)
#define    MAGIC_ADDRCONFIG_NNBITS(val)                                    \
 (((val)&MAGIC_ADDRCONFIG_NNBITS_MASK)>>MAGIC_ADDRCONFIG_NNBITS_OFFS)
#define    MAGIC_ADDRCONFIG_MASBITS(val)                                   \
 (((val)&MAGIC_ADDRCONFIG_MASBITS_MASK)>>MAGIC_ADDRCONFIG_MASBITS_OFFS)

/* OSPC mirror in uncached space (used by FPROM) */
#define MAGIC_PPR_OSPC               0x1000    /* r */


/** PPC error codes **/

#define MAGIC_PPC_NOT_SUCCESSFUL_BIT   0x8000000000000000LL /* set for error */
/* these return values should be ORed with the "not successful" bit */
#define MAGIC_PPC_RETRY_CODE           0x00   /* please retry request (default) */
#define MAGIC_PPC_BADGROUP             0x01   /* PPC group was invalid */
#define MAGIC_PPC_BADOPCODE            0x02   /* PPC opcode was invalid */
#define MAGIC_PPC_ARGOUTOFRANGE        0x03   /* some arg to the PPC was bad */
#define MAGIC_PPC_BUSY                 0x04   /* operation needed some
                                               * resource that was unavail */
/* these results indicate the request cannot be serviced, and it should
   not be retried.  The interpretation of these is protocol-dependent
   One example is: the physical pages are remote.  The application
   can't know this, but the sequence was otherwise valid.  If a protocol
   can't handle remote pages, this is a possibility. So, e.g. fmemcpy
   uses PROT_FAIL1 to indicate it can't handle the pages.  See the
   individual _interface files to describe the interpretation*/
#define MAGIC_PPC_PROT_FAIL1           0x11
#define MAGIC_PPC_PROT_FAIL2           0x12
#define MAGIC_PPC_PROT_FAIL3           0x13
#define MAGIC_PPC_RETRY                (MAGIC_PPC_NOT_SUCCESSFUL_BIT|MAGIC_PPC_RETRY_CODE)

/** PPC groups **/

#define MAGIC_PPC_GROUP_KERNEL       0x000
#define MAGIC_PPC_GROUP_MSG          0x001

/** kernel group opcodes **/

#define MAGIC_PPC_OP_SIPSLO          0x000
#define MAGIC_PPC_OP_SIPSHI          0x001
#define MAGIC_PPC_OP_MEMCPY          0x002
#define MAGIC_PPC_OP_IBITWRITE       0x003
#define MAGIC_PPC_OP_IBITREAD        0x004
#define MAGIC_PPC_OP_DONATE          0x005
#define MAGIC_PPC_OP_RESETPOOL       0x006
#define MAGIC_PPC_OP_LOADSTATE       0x007
#define MAGIC_PPC_OP_STORESTATE      0x008
#define MAGIC_PPC_OP_MEMRESET        0x009
#define MAGIC_PPC_OP_VECTORPKT       0x00A
#define MAGIC_PPC_OP_BZERO           0x00B
/* HLL/Diag opcodes within kernel region */
#define MAGIC_PPC_OP_PRINTF1         0x00C
/* config info opcodes within kernel region */
/* note: result has same format as MAGIC_PPR_NODECONFIG and
 *       MAGIC_PPR_ADDRCONFIG PPR's.
 */
#define MAGIC_PPC_OP_CELLNODECONFIG  0x010
#define MAGIC_PPC_OP_NODEADDRCONFIG  0x011
#define MAGIC_PPC_OP_STARTSLAVENODE  0x012

/** msg group opcodes **/
#define MAGIC_PPC_OP_MEMCPY_V        0x000
#define MAGIC_PPC_OP_SIPS_V          0x001
#define MAGIC_PPC_OP_BZERO_V         0x002


/** OSPC defs **/

/* OSPC opcodes.
 *
 * Note:
 * In the future we will probably add more structure to the header dword...
 * for example, in SIPS it would be nice to have the sender CPU from the
 * message header there, which authenticates the sender at a lower level
 * than the sender cell info in the SIPS header.  Because this will happen
 * eventually, these opcodes are not defined as 0x01LL.
 */
#define MAGIC_OSPC_LO_NONE           0xff     /* no OSPC pending */
#define MAGIC_OSPC_LO_SIPSREQ        0x01     /* lopri SIPS pending */
#define MAGIC_OSPC_LO_CACHECTR       0x02     /* ??? */

#define MAGIC_OSPC_HI_NONE           0xff     /* no OSPC pending */
#define MAGIC_OSPC_HI_SIPSREPLY      0x81     /* hipri SIPS pending */
#define MAGIC_OSPC_HI_TLBMISS        0x82     /* ??? */

/* For vector packet payload retrieval... in the future we'll move to some
   method other than OSPCs for this. */
#define MAGIC_OSPC_VEC_REQ_NONE 0xff
#define MAGIC_OSPC_VEC_REQ      0xd1
#define MAGIC_OSPC_VEC_REP_NONE 0xff
#define MAGIC_OSPC_VEC_REP      0xe1

/* OSPC offsets */
#define MAGIC_OSPC_SIZE              128      /* OSPC size == 1 cache line */

#define MAGIC_OSPC_LO_OFFS           0*MAGIC_OSPC_SIZE /* lopri SIPS */
#define MAGIC_OSPC_HI_OFFS           1*MAGIC_OSPC_SIZE /* hipri SIPS */
#define MAGIC_OSPC_VEC_REQ_OFFS      2*MAGIC_OSPC_SIZE /* VP request */
#define MAGIC_OSPC_VEC_REP_OFFS      3*MAGIC_OSPC_SIZE /* VP reply   */

/***************************************************************************
 MAGIC defs which reflect the virtual -> physical address mapping
 ***************************************************************************/

/* auxiliary defs -- ONLY FOR INTERNAL USE IN THIS FILE */

#ifndef SIMOS64BIT     /* 32 bit address space (current SimOS version) */

/* virtual addresses for start of FPROM and FRAM */
#define FPROM_BASE _SEXT(0xbfc00000)
#define FRAM_BASE  _SEXT(0xa0000000)

#ifdef _KERNEL
#define __MAGIC_BASE                COMPAT_K1BASE /* KSEG1 */
#define __MAGIC_BASE_32             COMPAT_K1BASE_32 /* KSEG1 */
#define __MAGIC_BASE_ACC            COMPAT_K1BASE /* still KSEG1 */
#else
#define __MAGIC_BASE                K1BASE /* KSEG1 */
#define __MAGIC_BASE_32             K1BASE_32 /* KSEG1 */
#define __MAGIC_BASE_ACC            K1BASE /* still KSEG1 */
#endif
#define __MAGIC_OSPC_BASE           (K0BASE+0x1000) /* OSPC right after remap */
#define __MAGIC_OSPC_END            (K0BASE+0x2000) /* 1 page-alias for each node */

#define __MAGIC_NODE_BITS           5  /* max. 32 nodes */
#define __MAGIC_NODE_OFFS           24

#define __MAGIC_ZONE_BITS           4  /* 16 zones / node */
#define __MAGIC_ZONE_OFFS           20 /* 1MB / zone */

#define __MAGIC_REG_BITS            17
#define __MAGIC_REG_OFFS            3  /* registers are 64bit */

#define __MAGIC_PPC_SEQ_BITS        7  /* one cache line */
#define __MAGIC_PPC_SEQ_OFFS        0
#define __MAGIC_PPC_OPC_BITS        5  /* group corresponds to a 4K page */
#define __MAGIC_PPC_OPC_OFFS        __MAGIC_PPC_SEQ_BITS
#define __MAGIC_PPC_GRP_BITS        8
#define __MAGIC_PPC_GRP_OFFS        (__MAGIC_PPC_SEQ_BITS+__MAGIC_PPC_OPC_BITS)

#define __MAGIC_ZONE(node, nbits, zone)                                    \
   ( __MAGIC_BASE | ((node) << __MAGIC_NODE_OFFS)                          \
                  | ((zone) << __MAGIC_ZONE_OFFS) )
#define __MAGIC_ZONE_32(node, nbits, zone)                                    \
   ( __MAGIC_BASE_32 | ((node) << __MAGIC_NODE_OFFS)                          \
                     | ((zone) << __MAGIC_ZONE_OFFS) )
#define __MAGIC_ZONE_ACC(node, nbits, zone)                                \
   ( __MAGIC_BASE_ACC | ((node) << __MAGIC_NODE_OFFS)                      \
                      | ((zone) << __MAGIC_ZONE_OFFS) )
#define __MAGIC_ZONE_ALIAS(zone)                                           \
   ( __MAGIC_BASE | ((zone) << __MAGIC_ZONE_OFFS) )
#define __MAGIC_ZONE_ALIAS_ACC(zone)                                       \
   ( __MAGIC_BASE_ACC | ((zone) << __MAGIC_ZONE_OFFS) )
#define __MAGIC_OSPC_RANGE(node, nbits)                                    \
   ( __MAGIC_OSPC_BASE )

#define MAGIC_MAX_REMAP_PAGES       1  /* max. sz of remap area (limited
                                        * by OSPC in following page) */

/* offsets in bdoor zone of simulated devices (64KB each) */
#define __MAGIC_BDOOR_CLOCK_OFFS        0x00000000 /* CMOS rt clock */
#define __MAGIC_BDOOR_CNSLE_OFFS        0x00001000 /* console interface */
#define __MAGIC_BDOOR_ETHER_OFFS        0x00002000 /* ethernet controller */
#define __MAGIC_BDOOR_DISKS_OFFS        0x00010000 /* scsi disk controller */

#ifdef TORNADO
#define __MAGIC_BDOOR_GIZMO_OFFS	                                       \
    ((__MAGIC_BDOOR_DISKS_OFFS + sizeof(DevDiskRegisters)*DEV_DISK_MAX_DISKS + \
      0x1000 - 1) & ~(0x1000-1))		   /* gizmo interface */
#endif

#else /* SIMOS64BIT */ /* 64 bit address space */
not yet implemented, will not compile;
#endif /* SIMOS64BIT */

#ifdef LANGUAGE_ASSEMBLY
#define MagicRegister int
#define MAGICREGP_CAST
#else
typedef uint64 MagicRegister;
#define MAGICREGP_CAST (MagicRegister *)
#endif

/* PPR access */
#define MAGIC_PPR(node, nbits, reg)                                        \
   ((MagicRegister*)                                                       \
    (__MAGIC_ZONE(node,nbits,MAGIC_ZONE_PPR) | ((reg) << __MAGIC_REG_OFFS)))
#define MAGIC_PPR_ALIAS(reg)                                               \
   (MAGICREGP_CAST                                                       \
    (__MAGIC_ZONE_ALIAS(MAGIC_ZONE_PPR_ALIAS) | ((reg) << __MAGIC_REG_OFFS)))
#define MAGIC_PPR_NODE(addr, nbits)                                        \
   ( ((addr) >> __MAGIC_NODE_OFFS) & ((1LL << (nbits)) - 1) )
#define MAGIC_PPR_ZONE(addr)                                               \
   ( ((addr) >> __MAGIC_ZONE_OFFS) & ((1LL << __MAGIC_ZONE_BITS) - 1) )
#define MAGIC_PPR_REG(addr)                                                \
   ( ((addr) >> __MAGIC_REG_OFFS) & ((1LL << __MAGIC_REG_BITS) - 1) )

/* PPC access */
#define MAGIC_PPC(node, nbits, grp, opc)                                   \
   ((MagicRegister*)                                                       \
    ( __MAGIC_ZONE(node,nbits,MAGIC_ZONE_PPC) |                            \
      ((grp) << __MAGIC_PPC_GRP_OFFS) |                                    \
      ((opc) << __MAGIC_PPC_OPC_OFFS) ))
#define MAGIC_PPC_ACC(node, nbits, grp, opc)                               \
   ((MagicRegister*)                                                       \
    ( __MAGIC_ZONE_ACC(node,nbits,MAGIC_ZONE_PPC) |                        \
      ((grp) << __MAGIC_PPC_GRP_OFFS) |                                    \
      ((opc) << __MAGIC_PPC_OPC_OFFS) ))
#define MAGIC_PPC_ALIAS(grp, opc)                                          \
   (MAGICREGP_CAST                                                         \
    ( __MAGIC_ZONE_ALIAS(MAGIC_ZONE_PPC_ALIAS)                             \
                   | ((grp) << __MAGIC_PPC_GRP_OFFS)                       \
                   | ((opc) << __MAGIC_PPC_OPC_OFFS) ))
#define MAGIC_PPC_ALIAS_ACC(grp, opc)                                      \
   ((MagicRegister*)                                                       \
    ( __MAGIC_ZONE_ALIAS_ACC(MAGIC_ZONE_PPC_ALIAS)                         \
                   | ((grp) << __MAGIC_PPC_GRP_OFFS)                       \
                   | ((opc) << __MAGIC_PPC_OPC_OFFS) ))

#define MAGIC_PPC_NODE(addr,nbits)                                         \
   ( ((addr) >> __MAGIC_NODE_OFFS) & ((1LL << (nbits)) - 1) )
#define MAGIC_PPC_ZONE(addr)                                               \
   ( ((addr) >> __MAGIC_ZONE_OFFS) & ((1LL << __MAGIC_ZONE_BITS) - 1) )
#define MAGIC_PPC_GRP(addr)                                                \
   ( ((addr) >> __MAGIC_PPC_GRP_OFFS) & ((1LL << __MAGIC_PPC_GRP_BITS) - 1) )
#define MAGIC_PPC_OPC(addr)                                                \
   ( ((addr) >> __MAGIC_PPC_OPC_OFFS) & ((1LL << __MAGIC_PPC_OPC_BITS) - 1) )

/* Nodemap access */
#define MAGIC_NODEMAP(node, nbits, reg)                                    \
   ((MagicRegister*)                                                       \
    (__MAGIC_ZONE(node,nbits,MAGIC_ZONE_NODEMAP) | ((reg) << __MAGIC_REG_OFFS)))

/* Nodecomm access */
#define MAGIC_NODECOMM(node, nbits, n)                                     \
   ((MagicRegister*)                                                       \
    (__MAGIC_ZONE(node,nbits,MAGIC_ZONE_NODECOMM) | ((n) << __MAGIC_REG_OFFS)))

/* OSPC access */
#define MAGIC_OSPC(node, nbits, ospc)                                      \
   ( (MagicRegister*)(__MAGIC_OSPC_RANGE(node, nbits) + (ospc)) )

#define MAGIC_UNCACHED_OSPC(node, nbits, offs)                             \
   MAGIC_PPR(node, nbits, MAGIC_PPR_OSPC + ((offs) >> __MAGIC_REG_OFFS))

#define MAGIC_UNCACHED_OSPC_ALIAS(offs)                                    \
   MAGIC_PPR_ALIAS(MAGIC_PPR_OSPC + ((offs) >> __MAGIC_REG_OFFS))

#define MAGIC_OSPC_OPCODE(val) ((int)((val) & ((MagicRegister)0xff)))

/* firewall access */
#define MAGIC_FW_RANGE(node, nbits)                                        \
   ((MagicRegister*) __MAGIC_ZONE(node,nbits,MAGIC_ZONE_FIREWALL))


/***************************************************************************
 definitions of the simulated devices and of various parameters
 which depend on the simulator.
 ***************************************************************************/

#define MAGIC_MAX_CPUS                32 /* max no of nodes in a simulation */
#define MAGIC_MAX_CELLS               32 /* max no of cells in a simulation */


/* needed so assembly files can include machine_defs */
#ifndef LANGUAGE_ASSEMBLY


typedef unsigned int DevRegister;

/* disk device:
 * there is currently one controller per cell. The controller
 * can be accessed through the backdoor zone of the first node
 * in the cell.
 */

#define DEV_DISK_CMD_SIZE            12
#define DEV_DISK_MAX_DMA_LENGTH      1024 /* max no of pages for one request */
#define DEV_DISK_MAX_DISKS           128  /* max disks per controller (cell) */

typedef struct DevDiskRegisters {
    DevRegister intr_pending;     /* r:int posted for this disk / w:ack int */
    DevRegister errnoVal;         /* status of last i/o */
    DevRegister bytesTransferred; /* bytes transferred during last i/o */

    DevRegister interruptNode;    /* node to interrupt upon completion */
    DevRegister k0Addr[DEV_DISK_MAX_DMA_LENGTH]; /* page addresses */
    DevRegister offset;           /* page offset for first page */
    DevRegister command[DEV_DISK_CMD_SIZE]; /* i/o command */

    DevRegister startIO;          /* write here causes i/o initiation */
    DevRegister doneIO;           /* tells when i/o complete */

    DevRegister filler[16];       /* filler: resize when you add new regs */
} DevDiskRegisters;

#define DEV_DISK_REGISTERS(node, nbits, disk)                               \
   ( ((volatile DevDiskRegisters*)                                          \
        (__MAGIC_ZONE(node, nbits, MAGIC_ZONE_BDOOR_DEV) +                  \
         __MAGIC_BDOOR_DISKS_OFFS)) +                                       \
     (disk) )


/* console device:
 * there is currently one console per cell. The console registers
 * can be accessed through the backdoor zone of the first node
 * in the cell. The console always interrupts this node.
 */

#define DEV_CNSLE_TX_INTR            0x01 /* intr enable / state bits */
#define DEV_CNSLE_RX_INTR            0x02

typedef struct DevConsoleRegisters {
    DevRegister intr_status;      /* r: intr state / w: intr enable */
    DevRegister data;             /* r: current char / w: send char */
} DevConsoleRegisters;

#define DEV_CONSOLE_REGISTERS(node, nbits)                                  \
   ( ((volatile DevConsoleRegisters*)                                       \
        (__MAGIC_ZONE(node, nbits, MAGIC_ZONE_BDOOR_DEV) +                  \
         __MAGIC_BDOOR_CNSLE_OFFS)) )


/* ethernet device:
 * there is currently one ether interface per cell. The ether registers
 * can be accessed through the backdoor zone of the first node
 * in the cell. The ether interface always interrupts this node.
 */

#define DEV_ETHER_MAX_RCV_ENTRIES    64
#define DEV_ETHER_MAX_SND_ENTRIES    64
#define DEV_ETHER_MAX_SND_CHUNKS     128

#define DEV_ETHER_MAX_TRANSFER_SIZE  1800

typedef struct DevEtherRegisters {
  DevRegister etheraddr[6];       /* controller tells OS its ethernet addr */
  DevRegister numRcvEntries;      /* read by OS, indicates how many receive
                                   * ring buffer entries will be used. OS must
                                   * allocate a receive buffer for each of
                                   * these entries */
  DevRegister numSndEntries;      /* read by OS, indicates how many send ring
                                   * buffer entries will be used so it knows
                                   * when to wrap its index pointer */
  DevRegister numSndChunks;       /* same as numSndEntries */

  struct {
    DevRegister pAddr;
    DevRegister maxLen;
    DevRegister len;
    DevRegister flag;
  } rcvEntries[ DEV_ETHER_MAX_RCV_ENTRIES ];

  struct {
    DevRegister firstChunk;
    DevRegister lastChunk;
    DevRegister flag; /* triggers send */
  } sndEntries[ DEV_ETHER_MAX_SND_ENTRIES ];

  struct {
    DevRegister pAddr;
    DevRegister len;
  } sndChunks[ DEV_ETHER_MAX_SND_CHUNKS ];

  /* note: sndChunks is last because we might extend the number of
   * send chunks in the future and don't want to break OS compatibility when
   * we do it */
} DevEtherRegisters;

/* values for flag field */
#define DEV_ETHER_OS_OWNED           (DevRegister)1
#define DEV_ETHER_CONTROLLER_OWNED   (DevRegister)2

#define DEV_ETHER_REGISTERS(node, nbits)                                  \
   ( ((volatile DevEtherRegisters*)                                       \
        (__MAGIC_ZONE(node, nbits, MAGIC_ZONE_BDOOR_DEV) +                \
         __MAGIC_BDOOR_ETHER_OFFS)) )


/* CMOS RT clock device:
 * This simulates a very primitive CMOS clock. This device only
 * has one register that contains the time since January 1, 1970
 * (same as the Unix gettimeofday() result).
 */

typedef struct DevClockRegisters {
  DevRegister ctime;              /* current time */
} DevClockRegisters;

#define DEV_CLOCK_REGISTERS(node, nbits)                                  \
   ( ((volatile DevClockRegisters*)                                       \
        (__MAGIC_ZONE(node, nbits, MAGIC_ZONE_BDOOR_DEV) +                \
         __MAGIC_BDOOR_CLOCK_OFFS)) )


#endif /* LANGUAGE_ASSEMBLY */


/* Interrupt bit assignments:
 *
 * There are 64 external interrupt lines coming into MAGIC. The
 * following defines show to what interrupt line each device is
 * connected.
 *
 * NOTE: when MAGIC posts an interrupt, the IEChigh register will
 * contain the bit number of the highest level interrupt pending,
 * so the bit numbers are also IEC's (Interrupt Exception Codes).
 */

#define DEV_IEC_SCSI                0x08    /* scsi disk controller */
#define DEV_IEC_ETHER               0x09    /* ether controller */
#define DEV_IEC_OSPC_LO             0x0a    /* low-priority SIPS */
#define DEV_IEC_VEC_REQ             0x0b    /* vector packet request */
#define DEV_IEC_KEYBDMOUSE          0x10    /* console */
#define DEV_IEC_DUART               0x11    /* serial line on FLASH board */
#define DEV_IEC_OSPC_HI             0x12    /* high-priority SIPS */
#define DEV_IEC_RECOVERY            0x13    /* recov int (posted by MAGIC) */
#define DEV_IEC_VEC_REPLY           0x14    /* vector packet reply */
#define DEV_IEC_CLOCK               0x18    /* clock */
#define DEV_IEC_IPI                 0x20    /* inter-processor interrupt */
#define DEV_IEC_DEBUG               0x21    /* ??? */
#define DEV_IEC_PROFTIM             0x28    /* prof timer (currently unused) */
#define DEV_IEC_MAGICWARN           0x29    /* ??? */
#define DEV_IEC_MAGICERR            0x31    /* ??? */
#define DEV_IEC_POWERFAIL           0x38    /* ??? */

#define DEV_IEC_MAX                 0x3f    /* 64 bits */


/* PCI slot assignments:
 *
 * NOTE:
 * On the real system these slot assignments wouldn't be fixed (you
 * could plug a card into any slot on the I/O bus) but this isn't
 * particularly interesting to model.
 */

#define DEV_PCI_DISK_SLOT           0
#define DEV_PCI_ETHER_SLOT          1
#define DEV_PCI_CONSOLE_SLOT        2


#endif /* _MACHINE_DEFS_H */
