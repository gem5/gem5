/*
 * Copyright (c) 1990 The Hewlett-Packard Development Company
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
 *
 */

/*
 * Copyright (c) 1994, 1995, 1996 Carnegie-Mellon University.
 * All rights reserved.
 *
 * Author: Keith Bostic, Chris G. Demetriou
 *
 * Permission to use, copy, modify and distribute this software and
 * its documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND
 * FOR ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
 */

/*
 * Defines for the architected startup addresses.
 */
#define HWRPB_ADDR	0x10000000	/* 256 MB */
#define BOOT_ADDR	0x20000000	/* 512 MB */
#define PGTBL_ADDR	0x40000000	/*   1 GB */

/*
 * Values for the "haltcode" field in the per-cpu portion of the HWRPB
 *
 * Bit defines for the "sysvar" field in the HWRPB.
 * Each platform has different values for SYSBOARD and IOBOARD bits.
 */
#define HALT_PWRUP	0		/* power up */
#define HALT_OPR	1		/* operator issued halt cmd */
#define HALT_KSTK	2		/* kernel stack not valid */
#define HALT_SCBB	3		/* invalid SCBB */
#define HALT_PTBR	4		/* invalid PTBR */
#define HALT_EXE	5		/* kernel executed halt instruction */
#define HALT_DBLE	6		/* double error abort */

/*
 * Bit defines for the "state" field in the per-cpu portion of the HWRPB
 */
#define STATE_BIP	0x00000001	/* bootstrap in progress */
#define STATE_RC	0x00000002	/* restart capable */
#define STATE_PA	0x00000004	/* processor available to OS */
#define STATE_PP	0x00000008	/* processor present */
#define STATE_OH	0x00000010	/* operator halted */
#define STATE_CV	0x00000020	/* context valid */
#define STATE_PV	0x00000040	/* PALcode valid */
#define STATE_PMV	0x00000080	/* PALcode memory valid */
#define STATE_PL	0x00000100	/* PALcode loaded */
#define STATE_HALT_MASK	0x00ff0000	/* Mask for Halt Requested field */
#define STATE_DEFAULT	0x00000000	/* Default (no specific action) */
#define STATE_SVRS_TERM	0x00010000	/* SAVE_TERM/RESTORE_TERM Exit */
#define STATE_COLD_BOOT	0x00020000	/* Cold Bootstrap Requested */
#define STATE_WARM_BOOT	0x00030000	/* Warm Bootstrap Requested */
#define STATE_HALT	0x00040000	/* Remain halted (no restart) */


#define SV_PF_RSVD	0x00000000	/* RESERVED */
#define SV_RESERVED	0x00000000	/* All STS bits; 0 for back compat */
#define SV_MPCAP	0x00000001	/* MP capable */
#define SV_PF_UNITED	0x00000020	/* United */
#define SV_PF_SEPARATE	0x00000040	/* Separate */
#define SV_PF_FULLBB	0x00000060	/* Full battery backup */
#define SV_POWERFAIL	0x000000e0	/* Powerfail implementation */
#define SV_PF_RESTART	0x00000100	/* Powerfail restart */

#define SV_GRAPHICS	0x00000200	/* Embedded graphics processor */

#define SV_STS_MASK	0x0000fc00	/* STS bits - system and I/O board */
#define SV_SANDPIPER	0x00000400	/* others define system platforms */
#define SV_FLAMINGO	0x00000800	/* STS BIT SETTINGS */
#define SV_HOTPINK	0x00000c00	/* STS BIT SETTINGS */
#define SV_FLAMINGOPLUS	0x00001000	/* STS BIT SETTINGS */
#define SV_ULTRA	0x00001400	/* STS BIT SETTINGS */
#define SV_SANDPLUS	0x00001800	/* STS BIT SETTINGS */
#define SV_SANDPIPER45	0x00001c00	/* STS BIT SETTINGS */
#define SV_FLAMINGO45	0x00002000	/* STS BIT SETTINGS */

#define SV_SABLE	0x00000400	/* STS BIT SETTINGS */

#define SV_KN20AA	0x00000400	/* STS BIT SETTINGS */

/*
 * Values for the "console type" field in the CTB portion of the HWRPB
 */
#define CONS_NONE	0		/* no console present */
#define CONS_SRVC	1		/* console is service processor */
#define CONS_DZ		2		/* console is dz/dl VT device */
#define CONS_GRPH	3		/* cons is gfx dev w/ dz/dl keybd*/
#define CONS_REM	4		/* cons is remote, protocal enet/MOP */

/*
 * PALcode variants that we're interested in.
 * Used as indices into the "palrev_avail" array in the per-cpu portion
 * of the HWRPB.
 */
#define PALvar_reserved	0
#define PALvar_OpenVMS	1
#define PALvar_OSF1	2

/*
 * The Alpha restart parameter block, which is a page or 2 in low memory
 */
struct rpb {
    struct rpb *rpb_selfref;	/* 000: physical self-reference */
    long  rpb_string;		/* 008: contains string "HWRPB" */
    long  rpb_vers;		/* 010: HWRPB version number */
    ulong rpb_size;		/* 018: bytes in RPB perCPU CTB CRB MEMDSC */
    ulong rpb_cpuid;		/* 020: primary cpu id */
    ulong rpb_pagesize;		/* 028: page size in bytes */
    ulong rpb_addrbits;		/* 030: number of phys addr bits */
    ulong rpb_maxasn;		/* 038: max valid ASN */
    char  rpb_ssn[16];		/* 040: system serial num: 10 ascii chars */
    ulong grpb_systype;		/* 050: system type */
    long  rpb_sysvar;		/* 058: system variation */
    long  rpb_sysrev;		/* 060: system revision */
    ulong rpb_clock;		/* 068: scaled interval clock intr freq */
    ulong rpb_counter;		/* 070: cycle counter frequency */
    ulong rpb_vptb;		/* 078: virtual page table base */
    long  rpb_res1;		/* 080: reserved */
    ulong rpb_trans_off;	/* 088: offset to translation buffer hint */
    ulong rpb_numprocs;		/* 090: number of processor slots */
    ulong rpb_slotsize;		/* 098: per-cpu slot size */
    ulong rpb_percpu_off;	/* 0A0: offset to per_cpu slots */
    ulong rpb_num_ctb;		/* 0A8: number of CTBs */
    ulong rpb_ctb_size;		/* 0B0: bytes in largest CTB */
    ulong rpb_ctb_off;		/* 0B8: offset to CTB (cons term block) */
    ulong rpb_crb_off;		/* 0C0: offset to CRB (cons routine block) */
    ulong rpb_mdt_off;		/* 0C8: offset to memory descriptor table */
    ulong rpb_config_off;	/* 0D0: offset to config data block */
    ulong rpb_fru_off;		/* 0D8: offset to FRU table */
    void  (*rpb_saveterm)();	/* 0E0: virt addr of save term routine */
    long  rpb_saveterm_pv;	/* 0E8: proc value for save term routine */
    void  (*rpb_rstrterm)();	/* 0F0: virt addr of restore term routine */
    long  rpb_rstrterm_pv;	/* 0F8: proc value for restore term routine */
    void  (*rpb_restart)();	/* 100: virt addr of CPU restart routine */
    long  rpb_restart_pv;	/* 108: proc value for CPU restart routine */
    long  rpb_software;		/* 110: used to determine presence of kdebug */
    long  rpb_hardware;		/* 118: reserved for hardware */
    long  rpb_checksum;		/* 120: checksum of prior entries in rpb */
    long  rpb_rxrdy;		/* 128: receive ready bitmask */
    long  rpb_txrdy;		/* 130: transmit ready bitmask */
    ulong rpb_dsr_off;		/* 138: Dynamic System Recog. offset */
};

#define rpb_kdebug rpb_software

#define OSF_HWRPB_ADDR	((vm_offset_t)(-1L << 23))

/*
 * This is the format for the boot/restart HWPCB.  It must match the
 * initial fields of the pcb structure as defined in pcb.h, but must
 * additionally contain the appropriate amount of padding to line up
 * with formats used by other palcode types.
 */
struct bootpcb {
    long rpb_ksp;		/* 000: kernel stack pointer */
    long rpb_usp;		/* 008: user stack pointer */
    long rpb_ptbr;		/* 010: page table base register */
    int  rpb_cc;		/* 018: cycle counter */
    int  rpb_asn;		/* 01C: address space number */
    long rpb_proc_uniq;		/* 020: proc/thread unique value */
    long rpb_fen;		/* 028: floating point enable */
    long rpb_palscr[2];		/* 030: pal scratch area */
    long rpb_pcbpad[8];		/* 040: padding for fixed size */
};

/*
 * Inter-Console Communications Buffer
 * Used for the primary processor to communcate with the console
 * of secondary processors.
 */
struct iccb {
    uint iccb_rxlen;		/* receive length in bytes      */
    uint iccb_txlen;		/* transmit length in bytes     */
    char iccb_rxbuf[80];	/* receive buffer               */
    char iccb_txbuf[80];	/* transmit buffer              */
};

/*
 * The per-cpu portion of the Alpha HWRPB.
 * Note that the main portion of the HWRPB is of variable size,
 * hence this must be a separate structure.
 *
 */
struct rpb_percpu {
    struct bootpcb rpb_pcb;	/* 000: boot/restart HWPCB */
    long rpb_state;		/* 080: per-cpu state bits */
    long rpb_palmem;		/* 088: palcode memory length */
    long rpb_palscratch;	/* 090: palcode scratch length */
    long rpb_palmem_addr;	/* 098: phys addr of palcode mem space */
    long rpb_palscratch_addr;	/* 0A0: phys addr of palcode scratch space */
    long rpb_palrev;		/* 0A8: PALcode rev required */
    long rpb_proctype;		/* 0B0: processor type */
    long rpb_procvar;		/* 0B8: processor variation */
    long rpb_procrev;		/* 0C0: processor revision */
    char rpb_procsn[16];	/* 0C8: proc serial num: 10 ascii chars */
    long rpb_logout;		/* 0D8: phys addr of logout area */
    long rpb_logout_len;	/* 0E0: length in bytes of logout area */
    long rpb_haltpb;		/* 0E8: halt pcb base */
    long rpb_haltpc;		/* 0F0: halt pc */
    long rpb_haltps;		/* 0F8: halt ps */
    long rpb_haltal;		/* 100: halt arg list (R25) */
    long rpb_haltra;		/* 108: halt return address (R26) */
    long rpb_haltpv;		/* 110: halt procedure value (R27) */
    long rpb_haltcode;		/* 118: reason for halt */
    long rpb_software;		/* 120: for software */
    struct iccb	rpb_iccb;       /* 128: inter-console communications buffer */
    long rpb_palrev_avail[16];	/* 1D0: PALcode revs available */
    long rpb_pcrsvd[6];		/* 250: reserved for arch use */
/* the dump stack grows from the end of the rpb page not to reach here */
};

/* The firmware revision is in the (unused) first entry of palrevs available */
#define rpb_firmrev rpb_palrev_avail[0]

/*
 * The memory cluster descriptor.
 */
struct rpb_cluster {
    long rpb_pfn;		/* 000: starting PFN of this cluster */
    long rpb_pfncount;		/* 008: count of PFNs in this cluster */
    long rpb_pfntested;		/* 010: count of tested PFNs in cluster */
    long rpb_va;		/* 018: va of bitmap */
    long rpb_pa;		/* 020: pa of bitmap */
    long rpb_checksum;		/* 028: checksum of bitmap */
    long rpb_usage;		/* 030: usage of cluster */
};
#define CLUSTER_USAGE_OS	((long)0)
#define CLUSTER_USAGE_PAL	((long)1)
#define CLUSTER_USAGE_NVRAM	((long)2)

/*
 * The "memory descriptor table" portion of the HWRPB.
 * Note that the main portion of the HWRPB is of variable size and there is a
 * variable number of per-cpu slots, hence this must be a separate structure.
 * Also note that the memory descriptor table contains a fixed portion plus
 * a variable number of "memory cluster descriptors" (one for each "cluster"
 * of memory).
 */
struct rpb_mdt {
    long rpb_checksum;		/* 000: checksum of entire mem desc table */
    long rpb_impaddr;		/* 008: PA of implementation dep info */
    long rpb_numcl;		/* 010: number of clusters */
    struct rpb_cluster rpb_cluster[1];	/* first instance of a cluster */
};

/*
 * The "Console Terminal Block" portion of the HWRPB, for serial line
 * UART console device.
 */
struct ctb_tt {

    long ctb_type;               /*   0: always 4 */
    long ctb_unit;               /*   8: */
    long ctb_reserved;           /*  16: */
    long ctb_len;                /*  24: bytes of info */
    long ctb_ipl;                /*  32: console ipl level */
    long ctb_tintr_vec;          /*  40: transmit vec (0x800) */
    long ctb_rintr_vec;          /*  48: receive vec (0x800) */
#define CTB_GRAPHICS       3     /* graphics device */
#define CTB_NETWORK     0xC0     /* network device */
#define CTB_PRINTERPORT    2     /* printer port on the SCC */
    long ctb_term_type;          /*  56: terminal type */
    long ctb_keybd_type;         /*  64: keyboard nationality */
    long ctb_keybd_trans;        /*  72: trans. table addr */
    long ctb_keybd_map;          /*  80: map table addr */
    long ctb_keybd_state;        /*  88: keyboard flags */
    long ctb_keybd_last;         /*  96: last key entered */
    long ctb_font_us;            /* 104: US font table addr */
    long ctb_font_mcs;           /* 112: MCS font table addr */
    long ctb_font_width;         /* 120: font width, height */
    long ctb_font_height;        /* 128:         in pixels */
    long ctb_mon_width;          /* 136: monitor width, height */
    long ctb_mon_height;         /* 144:         in pixels */
    long ctb_dpi;                /* 152: monitor dots per inch */
    long ctb_planes;             /* 160: # of planes */
    long ctb_cur_width;          /* 168: cursor width, height */
    long ctb_cur_height;         /* 176:         in pixels */
    long ctb_head_cnt;           /* 184: # of heads */
    long ctb_opwindow;           /* 192: opwindow on screen */
    long ctb_head_offset;        /* 200: offset to head info */
    long ctb_putchar;            /* 208: output char to TURBO */
    long ctb_io_state;           /* 216: I/O flags */
    long ctb_listen_state;       /* 224: listener flags */
    long ctb_xaddr;              /* 232: extended info addr */
    long ctb_turboslot;          /* 248: TURBOchannel slot # */
    long ctb_server_off;         /* 256: offset to server info */
    long ctb_line_off;           /* 264: line parameter offset */
    char ctb_csd;                /* 272: console specific data */
};

/*
 * The "Console Terminal Block" portion of the HWRPB.
 */
struct rpb_ctb {
    long rpb_type;		/* 000: console type */
    long rpb_unit;		/* 008: console unit */
    long rpb_resv;		/* 010: reserved */
    long rpb_length;		/* 018: byte length of device dep portion */
    long rpb_first;		/* 000: first field of device dep portion */
};

/*
 * The physical/virtual map for the console routine block.
 */
struct rpb_map {
    long rpb_virt;		/* virtual address for map entry */
    long rpb_phys;		/* phys address for map entry */
    long rpb_pgcount;		/* page count for map entry */
};

/*
 * The "Console Routine Block" portion of the HWRPB.
 * Note: the "offsets" are all relative to the start of the HWRPB (HWRPB_ADDR).
 */
struct rpb_crb {
    long rpb_va_disp;		/* va of call-back dispatch rtn */
    long rpb_pa_disp;		/* pa of call-back dispatch rtn */
    long rpb_va_fixup;		/* va of call-back fixup rtn */
    long rpb_pa_fixup;		/* pa of call-back fixup rtn */
    long rpb_num;		/* number of entries in phys/virt map */
    long rpb_mapped_pages;	/* Number of pages to be mapped */
    struct rpb_map rpb_map[1];	/* first instance of a map entry */
};

/*
 * These macros define where within the HWRPB the CTB and CRB are located.
 */
#define CTB_SETUP \
    ((struct rpb_ctb *) ((long)hwrpb_addr + (long)(hwrpb_addr->rpb_ctb_off)))

#define CRB_SETUP \
    ((struct rpb_crb *) ((long)hwrpb_addr + (long)(hwrpb_addr->rpb_crb_off)))

/*
 * The "Dynamic System Recognition" portion of the HWRPB.
 * It is used to obtain the platform specific data need to allow
 * the platform define the platform name, the platform SMM and LURT
 * data for software licensing
 */
struct rpb_dsr {
    long rpb_smm;		/* SMM nubber used by LMF	*/
    ulong rpb_lurt_off;		/* offset to LURT table		*/
    ulong rpb_sysname_off;	/* offset to sysname char count	*/
    int	lurt[10];		/* XXM has one LURT entry	*/
};
