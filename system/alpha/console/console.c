/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
 * Copyright (c) 1993 The Hewlett-Packard Development Company
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

/* ******************************************
 * M5 Console
 * ******************************************/

#include <linux/stddef.h>
#include <sys/types.h>

#define CONSOLE
#include "access.h"
#include "cserve.h"
#include "rpb.h"

#define CONS_INT_TX   0x01  /* interrupt enable / state bits */
#define CONS_INT_RX   0x02

#define PAGE_SIZE (8192)

#define KSTACK_REGION_VA 0x20040000

#define KSEG   0xfffffc0000000000
#define K1BASE 0xfffffc8000000000
#define KSEG_TO_PHYS(x) (((ulong)x) & ~KSEG)

#define ROUNDUP8(x) ((ulong)(((ulong)x)+7) & ~7)
#define ROUNDUP128(x) ((ulong)(((ulong)x) + 127) & ~127)
#define ROUNDUP8K(x) ((ulong)(((ulong)(x)) + 8191) & ~8191)

#define FIRST(x)  ((((ulong)(x)) >> 33) & 0x3ff)
#define SECOND(x) ((((ulong)(x)) >> 23) & 0x3ff)
#define THIRD(x) ((((ulong)(x)) >> 13) & 0x3ff)
#define THIRD_XXX(x)  ((((ulong)(x)) >> 13) & 0xfff)
#define PFN(x)  ((((ulong)(x) & ~KSEG) >> 13))

/* Kernel write | kernel read | valid */
#define KPTE(x) ((ulong)((((ulong)(x)) << 32) | 0x1101))

#define HWRPB_PAGES 16

#define NUM_KERNEL_THIRD (4)

#define printf_lock(args...)		\
    do {				\
        SpinLock(&theLock);		\
        printf(args);			\
        SpinUnlock(&theLock);		\
    } while (0)


void unixBoot(int argc, char **argv);
void JToKern(char *bootadr, ulong rpb_percpu, ulong free_pfn, ulong k_argc,
             char **k_argv, char **envp);
void JToPal(ulong bootadr);
void SlaveLoop(int cpu);

volatile struct AlphaAccess *m5AlphaAccess;
struct AlphaAccess m5Conf;

ulong theLock;

extern void SpinLock(ulong *lock);
#define SpinUnlock(_x) *(_x) = 0;

struct _kernel_params {
    char *bootadr;
    ulong rpb_percpu;
    ulong free_pfn;
    ulong argc;
    ulong argv;
    ulong envp; /* NULL */
};

extern consoleCallback[];
extern consoleFixup[];
long CallBackDispatcher();
long CallBackFixup();

/*
 * m5 console output
 */

void
InitConsole()
{
}

char
GetChar()
{
    return m5AlphaAccess->inputChar;
}

void
PutChar(char c)
{
    m5AlphaAccess->outputChar = c;
}

int
passArgs(int argc)
{
    return 0;
}

int
main(int argc, char **argv)
{
    int x, i;
    uint *k1ptr, *ksegptr;

    InitConsole();
    printf_lock("M5 console: m5AlphaAccess @ 0x%x\n", m5AlphaAccess);

    /*
     * get configuration from backdoor
     */
    m5Conf.last_offset = m5AlphaAccess->last_offset;
    printf_lock("Got Configuration %d\n", m5Conf.last_offset);

    m5Conf.last_offset = m5AlphaAccess->last_offset;
    m5Conf.version = m5AlphaAccess->version;
    m5Conf.numCPUs = m5AlphaAccess->numCPUs;
    m5Conf.intrClockFrequency = m5AlphaAccess->intrClockFrequency;
    m5Conf.cpuClock = m5AlphaAccess->cpuClock;
    m5Conf.mem_size = m5AlphaAccess->mem_size;
    m5Conf.kernStart = m5AlphaAccess->kernStart;
    m5Conf.kernEnd = m5AlphaAccess->kernEnd;
    m5Conf.entryPoint = m5AlphaAccess->entryPoint;
    m5Conf.diskUnit = m5AlphaAccess->diskUnit;
    m5Conf.diskCount = m5AlphaAccess->diskCount;
    m5Conf.diskPAddr = m5AlphaAccess->diskPAddr;
    m5Conf.diskBlock = m5AlphaAccess->diskBlock;
    m5Conf.diskOperation = m5AlphaAccess->diskOperation;
    m5Conf.outputChar = m5AlphaAccess->outputChar;
    m5Conf.inputChar = m5AlphaAccess->inputChar;

    if (m5Conf.version != ALPHA_ACCESS_VERSION)  {
        panic("Console version mismatch. Console expects %d. has %d \n",
              ALPHA_ACCESS_VERSION, m5Conf.version);
    }

    /*
     * setup arguments to kernel
     */
    unixBoot(argc, argv);

    panic("unix failed to boot\n");
    return 1;
}

/*
 * BOOTING
 */
struct rpb m5_rpb = {
    NULL,		/* 000: physical self-reference */
    ((long)'H') | (((long)'W') << 8) | (((long)'R') << 16) |
    ((long)'P' << 24) | (((long)'B') << 32),  /* 008: contains "HWRPB" */
    6,			/* 010: HWRPB version number */
    /* the byte count is wrong, but who needs it? - lance */
    0,			/* 018: bytes in RPB perCPU CTB CRB MEDSC */
    0,			/* 020: primary cpu id */
    PAGE_SIZE,		/* 028: page size in bytes */
    43,			/* 030: number of phys addr bits */
    127,		/* 038: max valid ASN */
    {'0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','1'},
                        /* 040: system serial num: 10 ascii chars */
    0, /* OVERRIDDEN */
    (1<<10),		/* 058: system variation */
    'c'|('o'<<8)|('o'<<16)|('l'<< 24),	/* 060: system revision */
    1024*4096,		/* 068: scaled interval clock intr freq */
    0,			/* 070: cycle counter frequency */
    0x200000000,	/* 078: virtual page table base */
    0,			/* 080: reserved */
    0,			/* 088: offset to translation buffer hint */
    1,			/* 090: number of processor slots OVERRIDDEN*/
    sizeof(struct rpb_percpu),	/* 098: per-cpu slot size. OVERRIDDEN */
    0,			/* 0A0: offset to per_cpu slots */
    1,			/* 0A8: number of CTBs */
    sizeof(struct ctb_tt),
    0,			/* 0B8: offset to CTB (cons term block) */
    0,			/* 0C0: offset to CRB (cons routine block) */
    0,			/* 0C8: offset to memory descriptor table */
    0,			/* 0D0: offset to config data block */
    0,			/* 0D8: offset to FRU table */
    0,			/* 0E0: virt addr of save term routine */
    0,			/* 0E8: proc value for save term routine */
    0,			/* 0F0: virt addr of restore term routine */
    0,			/* 0F8: proc value for restore term routine */
    0,			/* 100: virt addr of CPU restart routine */
    0,			/* 108: proc value for CPU restart routine */
    0,			/* 110: used to determine presence of kdebug */
    0,			/* 118: reserved for hardware */
/* the checksum is wrong, but who needs it? - lance */
    0,			/* 120: checksum of prior entries in rpb */
    0,			/* 128: receive ready bitmask */
    0,			/* 130: transmit ready bitmask */
    0,			/* 138: Dynamic System Recog. offset */
};

ulong m5_tbb[] = { 0x1e1e1e1e1e1e1e1e, 0x1e1e1e1e1e1e1e1e,
                   0x1e1e1e1e1e1e1e1e, 0x1e1e1e1e1e1e1e1e,
                   0x1e1e1e1e1e1e1e1e, 0x1e1e1e1e1e1e1e1e,
                   0x1e1e1e1e1e1e1e1e, 0x1e1e1e1e1e1e1e1e };

struct rpb_percpu m5_rpb_percpu = {
    {0,0,0,0,0,0,1,{0,0},{0,0,0,0,0,0,0,0}}, /* 000: boot/restart HWPCB */
    (STATE_PA | STATE_PP | STATE_CV |
     STATE_PV | STATE_PMV | STATE_PL), 	/* 080: per-cpu state bits */
    0xc000,				/* 088: palcode memory length */
    0x2000,				/* 090: palcode scratch length */
    0x4000,				/* 098: paddr of pal mem space */
    0x2000,				/* 0A0: paddr of pal scratch space */
    (2 << 16) | (5 << 8) | 1,		/* 0A8: PALcode rev required */
    11 | (2L  << 32),			/* 0B0: processor type */
    7,					/* 0B8: processor variation */
    'M'|('5'<<8)|('A'<<16)|('0'<<24),	/* 0C0: processor revision */
    {'M','5','/','A','l','p','h','a','0','0','0','0','0','0','0','0'},
                                        /* 0C8: proc serial num: 10 chars */
    0,					/* 0D8: phys addr of logout area */
    0,					/* 0E0: len in bytes of logout area */
    0,					/* 0E8: halt pcb base */
    0,					/* 0F0: halt pc */
    0,					/* 0F8: halt ps */
    0,					/* 100: halt arg list (R25) */
    0,					/* 108: halt return address (R26) */
    0,					/* 110: halt procedure value (R27) */
    0,		       			/* 118: reason for halt */
    0,		       			/* 120: for software */
    {0},				/* 128: inter-console comm buffer */
    {1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0},	/* 1D0: PALcode revs available */
    0					/* 250: reserved for arch use */
/* the dump stack grows from the end of the rpb page not to reach here */
};

struct _m5_rpb_mdt {
    long   rpb_checksum;	/* 000: checksum of entire mem desc table */
    long   rpb_impaddr;		/* 008: PA of implementation dep info */
    long   rpb_numcl;		/* 010: number of clusters */
    struct rpb_cluster rpb_cluster[3];	/* first instance of a cluster */
};

struct _m5_rpb_mdt m5_rpb_mdt = {
    0,			/* 000: checksum of entire mem desc table */
    0,			/* 008: PA of implementation dep info */
    0,			/* 010: number of clusters */
    {{	0,		/* 000: starting PFN of this cluster */
        0,		/* 008: count of PFNs in this cluster */
        0,		/* 010: count of tested PFNs in cluster */
        0,		/* 018: va of bitmap */
        0,		/* 020: pa of bitmap */
        0,		/* 028: checksum of bitmap */
        1		/* 030: usage of cluster */
     },
     {   0,		/* 000: starting PFN of this cluster */
         0,		/* 008: count of PFNs in this cluster */
         0,		/* 010: count of tested PFNs in cluster */
         0,		/* 018: va of bitmap */
         0,		/* 020: pa of bitmap */
         0,		/* 028: checksum of bitmap */
         0		/* 030: usage of cluster */
     },
     {   0,		/* 000: starting PFN of this cluster */
         0,		/* 008: count of PFNs in this cluster */
         0,		/* 010: count of tested PFNs in cluster */
         0,		/* 018: va of bitmap */
         0,		/* 020: pa of bitmap */
         0,		/* 028: checksum of bitmap */
         0		/* 030: usage of cluster */
     }}
};

/* constants for slotinfo bus_type subfield */
#define SLOTINFO_TC	0
#define SLOTINFO_ISA	1
#define SLOTINFO_EISA	2
#define SLOTINFO_PCI	3

struct rpb_ctb m5_rpb_ctb = {
    CONS_DZ,	/* 000: console type */
    0,		/* 008: console unit */
    0,		/* 010: reserved */
    0		/* 018: byte length of device dep portion */
};

/* we don't do any fixup (aka relocate the console) - we hope */
struct rpb_crb m5_rpb_crb = {
    0,		/* va of call-back dispatch rtn */
    0,		/* pa of call-back dispatch rtn */
    0,		/* va of call-back fixup rtn */
    0,		/* pa of call-back fixup rtn */
    0,		/* number of entries in phys/virt map */
    0		/* Number of pages to be mapped */
};

struct _rpb_name {
    ulong length;
    char name[16];
};

extern struct _rpb_name m5_name;

struct rpb_dsr m5_rpb_dsr = {
    0,
    0,
    0,
};

struct _rpb_name m5_name = {
    16,
    {'U','M','I','C','H',' ','M','5','/','A','L','P','H','A',' ',0},
};

/*
 * M5 has one LURT entry:
 *   1050 is for workstations
 *   1100 is servers (and is needed for CXX)
 */
long m5_lurt[10] = { 9, 12, -1, -1, -1, -1, -1, -1, 1100, 1100 };

ulong unix_boot_mem;
ulong bootadr;

char **kargv;
int kargc;
ulong free_pfn;
struct rpb_percpu *rpb_percpu;

char *
unix_boot_alloc(int pages)
{
    char *ret = (char *)unix_boot_mem;
    unix_boot_mem += (pages * PAGE_SIZE);
    return ret;
}

ulong *first = 0;
ulong *third_rpb = 0;
ulong *reservedFixup = 0;

int strcpy(char *dst, char *src);

struct rpb *rpb;
extern ulong _end;

void
unixBoot(int argc, char **argv)
{
    ulong *second,  *third_kernel, ptr, *tbb, size, *percpu_logout;
    unsigned char *mdt_bitmap;
    long *lp1, *lp2, sum;
    int i, cl;
    ulong kern_first_page;
    ulong mem_size = m5Conf.mem_size;

    ulong mem_pages = mem_size / PAGE_SIZE, cons_pages;
    ulong mdt_bitmap_pages = mem_pages / (PAGE_SIZE*8);

    ulong kernel_bytes, ksp, kernel_end, *unix_kernel_stack, bss,
        ksp_bottom, ksp_top;
    struct rpb_ctb *rpb_ctb;
    struct ctb_tt *ctb_tt;
    struct rpb_dsr *rpb_dsr;
    struct rpb_crb *rpb_crb;
    struct _m5_rpb_mdt *rpb_mdt;
    int *rpb_lurt;
    char *rpb_name;
    ulong nextPtr;

    printf_lock("memsize %x pages %x \n", mem_size, mem_pages);

    /* Allocate:
     *   two pages for the HWRPB
     *   five page table pages:
     *     1: First level page table
     *     1: Second level page table
     *     1: Third level page table for HWRPB
     *     2: Third level page table for kernel (for up to 16MB)
     * set up the page tables
     * load the kernel at the physical address 0x230000
     * build the HWRPB
     *   set up memory descriptor table to give up the
     *   physical memory between the end of the page
     *   tables and the start of the kernel
     * enable kseg addressing
     * jump to the kernel
     */

    unix_boot_mem = ROUNDUP8K(&_end);

    printf_lock("First free page after ROM 0x%x\n", unix_boot_mem);

    rpb = (struct rpb *)unix_boot_alloc(HWRPB_PAGES);

    mdt_bitmap = (unsigned char *)unix_boot_alloc(mdt_bitmap_pages); 
    first = (ulong *)unix_boot_alloc(1);
    second = (ulong *)unix_boot_alloc(1);
    third_rpb = (ulong *)unix_boot_alloc(1);
    reservedFixup = (ulong*) unix_boot_alloc(1);
    third_kernel = (ulong *)unix_boot_alloc(NUM_KERNEL_THIRD);
    percpu_logout = (ulong*)unix_boot_alloc(1);

    cons_pages = KSEG_TO_PHYS(unix_boot_mem) / PAGE_SIZE;

    /* Set up the page tables */
    bzero((char *)first, PAGE_SIZE);
    bzero((char *)second, PAGE_SIZE);
    bzero((char *)reservedFixup, PAGE_SIZE);
    bzero((char *)third_rpb, HWRPB_PAGES * PAGE_SIZE);
    bzero((char *)third_kernel, PAGE_SIZE * NUM_KERNEL_THIRD);

    first[0] = KPTE(PFN(second));
    first[1] = KPTE(PFN(first)); /* Region 3 */

    /* Region 0 */
    second[SECOND(0x10000000)] = KPTE(PFN(third_rpb));

    for (i = 0; i < NUM_KERNEL_THIRD; i++) {
        /* Region 1 */
        second[SECOND(0x20000000) + i] = KPTE(PFN(third_kernel) + i);
    }

    /* Region 2 */
    second[SECOND(0x40000000)] = KPTE(PFN(second));


    /* For some obscure reason, Dec Unix's database read
     * from /etc/sysconfigtab is written to this fixed
     * mapped memory location. Go figure, since it is
     * not initialized by the console. Maybe it is
     * to look at the database from the console
     * after a boot/crash.
     *
     * Black magic to estimate the max size. SEGVs on overflow
     * bugnion
     */

#define DATABASE_BASE           0x20000000
#define DATABASE_END            0x20020000

    ulong *dbPage = (ulong*)unix_boot_alloc(1);
    bzero(dbPage, PAGE_SIZE);
    second[SECOND(DATABASE_BASE)] = KPTE(PFN(dbPage));
    for (i = DATABASE_BASE; i < DATABASE_END ; i += PAGE_SIZE) {
        ulong *db = (ulong*)unix_boot_alloc(1);
        dbPage[THIRD(i)] = KPTE(PFN(db));
    }

    /* Region 0 */
    /* Map the HWRPB */
    for (i = 0; i < HWRPB_PAGES; i++)
        third_rpb[i] = KPTE(PFN(rpb) + i);

    /* Map the MDT bitmap table */
    for (i = 0; i < mdt_bitmap_pages; i++) {
        third_rpb[HWRPB_PAGES + i] = KPTE(PFN(mdt_bitmap) + i);
    }

    /* Protect the PAL pages */
    for (i = 1; i < PFN(first); i++)
        third_rpb[HWRPB_PAGES + mdt_bitmap_pages + i] = KPTE(i);

   /* Set up third_kernel after it's loaded, when we know where it is */
    kern_first_page = (KSEG_TO_PHYS(m5Conf.kernStart)/PAGE_SIZE);
    kernel_end = ROUNDUP8K(m5Conf.kernEnd);
    bootadr = m5Conf.entryPoint;

    printf_lock("HWRPB 0x%x l1pt 0x%x l2pt 0x%x l3pt_rpb 0x%x l3pt_kernel 0x%x"
                " l2reserv 0x%x\n",
                rpb, first, second, third_rpb, third_kernel, reservedFixup);
    if (kernel_end - m5Conf.kernStart > (0x800000*NUM_KERNEL_THIRD)) {
        printf_lock("Kernel is more than 8MB 0x%x - 0x%x = 0x%x\n",
                    kernel_end, m5Conf.kernStart,
                    kernel_end - m5Conf.kernStart );
        panic("kernel too big\n");
    }
    printf_lock("kstart = 0x%x, kend = 0x%x, kentry = 0x%x, numCPUs = 0x%x\n", m5Conf.kernStart, m5Conf.kernEnd, m5Conf.entryPoint, m5Conf.numCPUs);
    ksp_bottom = (ulong)unix_boot_alloc(1);
    ksp_top = ksp_bottom + PAGE_SIZE;
    ptr = (ulong) ksp_bottom;
    bzero((char *)ptr, PAGE_SIZE);
    dbPage[THIRD(KSTACK_REGION_VA)] = 0;		          /* Stack Guard Page */
    dbPage[THIRD(KSTACK_REGION_VA + PAGE_SIZE)] = KPTE(PFN(ptr)); /* Kernel Stack Page */
    dbPage[THIRD(KSTACK_REGION_VA + 2*PAGE_SIZE)] = 0;		  /* Stack Guard Page */

    /* put argv into the bottom of the stack - argv starts at 1 because
     * the command thatr got us here (i.e. "unixboot) is in argv[0].
     */
    ksp = ksp_top - 8;			/* Back up one longword */
    ksp -= argc * sizeof(char *);	/* Make room for argv */
    kargv = (char **) ksp;
    for (i = 1; i < argc; i++) {	/* Copy arguments to stack */
        ksp -= ((strlen(argv[i]) + 1) + 7) & ~0x7;
        kargv[i-1] = (char *) ksp;
        strcpy(kargv[i - 1], argv[i]);
    }
    kargc = i - 1;
    kargv[kargc] = NULL;	/* just to be sure; doesn't seem to be used */
    ksp -= sizeof(char *);	/* point above last arg for no real reason */

    free_pfn = PFN(kernel_end);

    bcopy((char *)&m5_rpb, (char *)rpb, sizeof(struct rpb));

    rpb->rpb_selfref = (struct rpb *) KSEG_TO_PHYS(rpb);
    rpb->rpb_string = 0x0000004250525748;

    tbb = (ulong *) (((char *) rpb) + ROUNDUP8(sizeof(struct rpb)));
    rpb->rpb_trans_off = (ulong)tbb - (ulong)rpb;
    bcopy((char *)m5_tbb, (char *)tbb, sizeof(m5_tbb));

    /*
     * rpb_counter. Use to determine timeouts in OS.
     * XXX must be patched after a checkpoint restore (I guess)
     */

    printf_lock("CPU Clock at %d MHz IntrClockFrequency=%d \n",
                m5Conf.cpuClock, m5Conf.intrClockFrequency);
    rpb->rpb_counter = m5Conf.cpuClock * 1000 * 1000;

    /*
     * By definition, the rpb_clock is scaled by 4096 (in hz)
     */
    rpb->rpb_clock = m5Conf.intrClockFrequency * 4096;

    /*
     * Per CPU Slots. Multiprocessor support.
     */
    int percpu_size = ROUNDUP128(sizeof(struct rpb_percpu));

    printf_lock("Booting with %d processor(s) \n", m5Conf.numCPUs);

    rpb->rpb_numprocs = m5Conf.numCPUs;
    rpb->rpb_slotsize = percpu_size;
    rpb_percpu = (struct rpb_percpu *)
        ROUNDUP128(((ulong)tbb) + (sizeof(m5_tbb)));

    rpb->rpb_percpu_off = (ulong)rpb_percpu - (ulong)rpb;

    for (i = 0; i < m5Conf.numCPUs; i++) {
        struct rpb_percpu *thisCPU = (struct rpb_percpu*)
            ((ulong)rpb_percpu + percpu_size * i);

        bzero((char *)thisCPU, percpu_size);
        bcopy((char *)&m5_rpb_percpu, (char *)thisCPU,
              sizeof(struct rpb_percpu));

        thisCPU->rpb_pcb.rpb_ksp = (KSTACK_REGION_VA + 2*PAGE_SIZE - (ksp_top - ksp));
        thisCPU->rpb_pcb.rpb_ptbr = PFN(first);

        thisCPU->rpb_logout = KSEG_TO_PHYS(percpu_logout);
        thisCPU->rpb_logout_len = PAGE_SIZE;

        printf_lock("KSP: 0x%x PTBR 0x%x\n",
                    thisCPU->rpb_pcb.rpb_ksp, thisCPU->rpb_pcb.rpb_ptbr);
    }

    nextPtr = (ulong)rpb_percpu + percpu_size * m5Conf.numCPUs;

    /*
     * Console Terminal Block
     */
    rpb_ctb = (struct rpb_ctb *) nextPtr;
    ctb_tt = (struct ctb_tt*) rpb_ctb;

    rpb->rpb_ctb_off = ((ulong)rpb_ctb) - (ulong)rpb;
    rpb->rpb_ctb_size  = sizeof(struct rpb_ctb);

    bzero((char *)rpb_ctb, sizeof(struct ctb_tt));

    rpb_ctb->rpb_type = CONS_DZ;
    rpb_ctb->rpb_length = sizeof(ctb_tt) - sizeof(rpb_ctb);

    /*
     * uart initizliation
     */
    ctb_tt->ctb_tintr_vec = 0x6c0;  /* matches tlaser pal code */
    ctb_tt->ctb_rintr_vec = 0x680;  /* matches tlaser pal code */
    ctb_tt->ctb_term_type = CTB_GRAPHICS;

    rpb_crb = (struct rpb_crb *) (((ulong)rpb_ctb) + sizeof(struct ctb_tt));
    rpb->rpb_crb_off = ((ulong)rpb_crb) - (ulong)rpb;

    bzero((char *)rpb_crb, sizeof(struct rpb_crb));

    /*
     * console callback stuff (m5)
     */
    rpb_crb->rpb_num = 1;
    rpb_crb->rpb_mapped_pages = HWRPB_PAGES;
    rpb_crb->rpb_map[0].rpb_virt = 0x10000000;
    rpb_crb->rpb_map[0].rpb_phys = KSEG_TO_PHYS(((ulong)rpb) & ~0x1fff);
    rpb_crb->rpb_map[0].rpb_pgcount = HWRPB_PAGES;

    printf_lock("Console Callback at 0x%x, fixup at 0x%x, crb offset: 0x%x\n",
                rpb_crb->rpb_va_disp, rpb_crb->rpb_va_fixup, rpb->rpb_crb_off);

    rpb_mdt = (struct _m5_rpb_mdt *)((ulong)rpb_crb + sizeof(struct rpb_crb));
    rpb->rpb_mdt_off = (ulong)rpb_mdt - (ulong)rpb;
    bcopy((char *)&m5_rpb_mdt, (char *)rpb_mdt, sizeof(struct _m5_rpb_mdt));


    cl = 0;
    rpb_mdt->rpb_cluster[cl].rpb_pfncount = kern_first_page;
    cl++;

    rpb_mdt->rpb_cluster[cl].rpb_pfn = kern_first_page;
    rpb_mdt->rpb_cluster[cl].rpb_pfncount = mem_pages - kern_first_page;
    rpb_mdt->rpb_cluster[cl].rpb_pfntested =
        rpb_mdt->rpb_cluster[cl].rpb_pfncount;
    rpb_mdt->rpb_cluster[cl].rpb_pa = KSEG_TO_PHYS(mdt_bitmap);
    rpb_mdt->rpb_cluster[cl].rpb_va = 0x10000000 + HWRPB_PAGES * PAGE_SIZE;
    cl++;

    rpb_mdt->rpb_numcl = cl;

    for (i = 0; i < cl; i++)
        printf_lock("Memory cluster %d [%d - %d]\n", i,
                    rpb_mdt->rpb_cluster[i].rpb_pfn,
                    rpb_mdt->rpb_cluster[i].rpb_pfncount);

    /* Checksum the rpb for good luck */
    sum = 0;
    lp1 = (long *)&rpb_mdt->rpb_impaddr;
    lp2 = (long *)&rpb_mdt->rpb_cluster[cl];
    while (lp1 < lp2) sum += *lp1++;
    rpb_mdt->rpb_checksum = sum;

    /* XXX should checksum the cluster descriptors */
    bzero((char *)mdt_bitmap, mdt_bitmap_pages * PAGE_SIZE);
    for (i = 0; i < mem_pages/8; i++)
        ((unsigned char *)mdt_bitmap)[i] = 0xff;

    printf_lock("Initalizing mdt_bitmap addr 0x%x mem_pages %x \n",
                (long)mdt_bitmap,(long)mem_pages);

    m5_rpb.rpb_config_off = 0;
    m5_rpb.rpb_fru_off = 0;

    rpb_dsr = (struct rpb_dsr *)((ulong)rpb_mdt + sizeof(struct _m5_rpb_mdt));
    rpb->rpb_dsr_off = (ulong)rpb_dsr - (ulong)rpb;
    bzero((char *)rpb_dsr, sizeof(struct rpb_dsr));
    rpb_dsr->rpb_smm = 1578; /* Official XXM SMM number as per SRM */
    rpb_dsr->rpb_smm = 1089; /* Official Alcor SMM number as per SRM */

    rpb_lurt = (int *) ROUNDUP8((ulong)rpb_dsr + sizeof(struct rpb_dsr));
    rpb_dsr->rpb_lurt_off = ((ulong) rpb_lurt) - (ulong) rpb_dsr;
    bcopy((char *)m5_lurt, (char *)rpb_lurt, sizeof(m5_lurt));

    rpb_name = (char *) ROUNDUP8(((ulong)rpb_lurt) + sizeof(m5_lurt));
    rpb_dsr->rpb_sysname_off = ((ulong) rpb_name) - (ulong) rpb_dsr;
#define THENAME "             M5/Alpha       "
    sum = sizeof(THENAME);
    bcopy(THENAME, rpb_name, sum);
    *(ulong *)rpb_name = sizeof(THENAME); /* put in length field */

    /* calculate size of rpb */
    rpb->rpb_size = ((ulong) &rpb_name[sum]) - (ulong)rpb;

    if (rpb->rpb_size > PAGE_SIZE * HWRPB_PAGES) {
        panic("HWRPB_PAGES=%d too small for HWRPB !!! \n");
    }

    ulong *rpbptr = (ulong*)((char*)rpb_dsr + sizeof(struct rpb_dsr));
    rpb_crb->rpb_pa_disp = KSEG_TO_PHYS(rpbptr);
    rpb_crb->rpb_va_disp = 0x10000000 +
        (((ulong)rpbptr - (ulong)rpb) & (0x2000 * HWRPB_PAGES - 1));
    printf_lock("ConsoleDispatch at virt %x phys %x val %x\n",
                rpb_crb->rpb_va_disp, rpb_crb->rpb_pa_disp, consoleCallback);
    *rpbptr++ = 0;
    *rpbptr++ = (ulong) consoleCallback;
    rpb_crb->rpb_pa_fixup = KSEG_TO_PHYS(rpbptr);
    rpb_crb->rpb_va_fixup = 0x10000000 +
        (((ulong)rpbptr - (ulong)rpb) & (0x2000 * HWRPB_PAGES - 1));
    *rpbptr++ = 0;

    *rpbptr++ = (ulong) consoleFixup;

    /* Checksum the rpb for good luck */
    sum = 0;
    lp1 = (long *)rpb;
    lp2 = &rpb->rpb_checksum;
    while (lp1 < lp2)
        sum += *lp1++;
    *lp2 = sum;

  /*
   * MP bootstrap
   */
    for (i = 1; i < m5Conf.numCPUs; i++) {
        ulong stack = (ulong)unix_boot_alloc(1);
        printf_lock("Bootstraping CPU %d with sp=0x%x\n", i, stack);
        m5AlphaAccess->cpuStack[i] = stack;
    }

    /*
     * Make sure that we are not stepping on the kernel
     */
    if ((ulong)unix_boot_mem >= (ulong)m5Conf.kernStart) {
        panic("CONSOLE: too much memory. Smashing kernel\n");
    } else {
        printf_lock("unix_boot_mem ends at %x \n", unix_boot_mem);
    }

    JToKern((char *)bootadr, (ulong)rpb_percpu, free_pfn, kargc, kargv, NULL);
}


void
JToKern(char *bootadr, ulong rpb_percpu, ulong free_pfn, ulong k_argc,
        char **k_argv, char **envp)
{
    extern ulong palJToKern[];

    struct _kernel_params *kernel_params = (struct _kernel_params *) KSEG;
    int i;

    printf_lock("k_argc = %d ", k_argc);
    for (i = 0; i < k_argc; i++) {
        printf_lock("'%s' ", k_argv[i]);
    }
    printf_lock("\n");

    kernel_params->bootadr = bootadr;
    kernel_params->rpb_percpu = KSEG_TO_PHYS(rpb_percpu);
    kernel_params->free_pfn = free_pfn;
    kernel_params->argc = k_argc;
    kernel_params->argv = (ulong)k_argv;
    kernel_params->envp = (ulong)envp;
    printf_lock("jumping to kernel at 0x%x, (PCBB 0x%x pfn %d)\n",
                bootadr, rpb_percpu, free_pfn);
    JToPal(KSEG_TO_PHYS(palJToKern));
    printf_lock("returned from JToPal. Looping\n");
    while (1)
        continue;
}

void
JToPal(ulong bootadr)
{
    cServe(bootadr, 0, CSERVE_K_JTOPAL);

    /*
     * Make sure that floating point is enabled incase
     * it was disabled by the user program.
     */
    wrfen(1);
}

int
strcpy(char *dst, char *src)
{
    int i = 0;
    while (*src) {
        *dst++ = *src++;
        i++;
    }
    return i;
}

/*
 * Console I/O
 *
 */

int numOpenDevices = 11;
struct {
    char name[128];
} deviceState[32];

#define BOOTDEVICE_NAME "SCSI 1 0 0 1 100 0"

void
DeviceOperation(long op, long channel, long count, long address, long block)
{
    long pAddr;

    if (strcmp(deviceState[channel].name, BOOTDEVICE_NAME )) {
        panic("DeviceRead: only implemented for root disk \n");
    }
    pAddr = KSEG_TO_PHYS(address);
    if (pAddr + count > m5Conf.mem_size) {
        panic("DeviceRead: request out of range \n");
    }

    m5AlphaAccess->diskCount = count;
    m5AlphaAccess->diskPAddr = pAddr;
    m5AlphaAccess->diskBlock = block;
    m5AlphaAccess->diskOperation = op; /* launch */
}

/*
 * M5 Console callbacks
 *
 */

/* AXP manual 2-31 */
#define CONSCB_GETC 0x1
#define CONSCB_PUTS 0x2
#define CONSCB_RESET_TERM 0x3
#define CONSCB_SET_TERM_INT 0x4
#define CONSCB_SET_TERM_CTL 0x5
#define CONSCB_PROCESS_KEY 0x6
#define CONSCB_OPEN_CONSOLE 0x7
#define CONSCB_CLOSE_CONSOLE 0x8

#define CONSCB_OPEN 0x10
#define CONSCB_CLOSE 0x11
#define CONSCB_READ 0x13

#define CONSCB_GETENV 0x22

/* AXP manual 2-26 */
#define	ENV_AUTO_ACTION		0X01
#define	ENV_BOOT_DEV		0X02
#define	ENV_BOOTDEF_DEV		0X03
#define	ENV_BOOTED_DEV		0X04
#define	ENV_BOOT_FILE		0X05
#define	ENV_BOOTED_FILE		0X06
#define	ENV_BOOT_OSFLAGS	0X07
#define	ENV_BOOTED_OSFLAGS	0X08
#define	ENV_BOOT_RESET		0X09
#define	ENV_DUMP_DEV		0X0A
#define	ENV_ENABLE_AUDIT	0X0B
#define	ENV_LICENSE		0X0C
#define	ENV_CHAR_SET		0X0D
#define	ENV_LANGUAGE		0X0E
#define	ENV_TTY_DEV		0X0F
#define	ENV_SCSIID		0X42
#define	ENV_SCSIFAST		0X43
#define	ENV_COM1_BAUD		0X44
#define	ENV_COM1_MODEM		0X45
#define	ENV_COM1_FLOW		0X46
#define	ENV_COM1_MISC		0X47
#define	ENV_COM2_BAUD		0X48
#define	ENV_COM2_MODEM		0X49
#define	ENV_COM2_FLOW		0X4A
#define	ENV_COM2_MISC		0X4B
#define	ENV_PASSWORD		0X4C
#define	ENV_SECURE		0X4D
#define	ENV_LOGFAIL		0X4E
#define	ENV_SRM2DEV_ID		0X4F

#define MAX_ENVLEN 32

char env_auto_action[MAX_ENVLEN]	= "BOOT";
char env_boot_dev[MAX_ENVLEN]		= "";
char env_bootdef_dev[MAX_ENVLEN]	= "";
char env_booted_dev[MAX_ENVLEN]		= BOOTDEVICE_NAME;
char env_boot_file[MAX_ENVLEN]		= "";
char env_booted_file[MAX_ENVLEN]	= "";
char env_boot_osflags[MAX_ENVLEN]	= "";
char env_booted_osflags[MAX_ENVLEN]	= "";
char env_boot_reset[MAX_ENVLEN]		= "";
char env_dump_dev[MAX_ENVLEN]		= "";
char env_enable_audit[MAX_ENVLEN]	= "";
char env_license[MAX_ENVLEN]		= "";
char env_char_set[MAX_ENVLEN]		= "";
char env_language[MAX_ENVLEN]		= "";
char env_tty_dev[MAX_ENVLEN]		= "0";
char env_scsiid[MAX_ENVLEN]		= "";
char env_scsifast[MAX_ENVLEN]		= "";
char env_com1_baud[MAX_ENVLEN]		= "";
char env_com1_modem[MAX_ENVLEN]		= "";
char env_com1_flow[MAX_ENVLEN]		= "";
char env_com1_misc[MAX_ENVLEN]		= "";
char env_com2_baud[MAX_ENVLEN]		= "";
char env_com2_modem[MAX_ENVLEN]		= "";
char env_com2_flow[MAX_ENVLEN]		= "";
char env_com2_misc[MAX_ENVLEN]		= "";
char env_password[MAX_ENVLEN]		= "";
char env_secure[MAX_ENVLEN]		= "";
char env_logfail[MAX_ENVLEN]		= "";
char env_srm2dev_id[MAX_ENVLEN]		= "";

#define MAX_ENV_INDEX 100
char *envptr[MAX_ENV_INDEX] = {
    0,					/* 0x00 */
    env_auto_action,			/* 0x01 */
    env_boot_dev,			/* 0x02 */
    env_bootdef_dev,			/* 0x03 */
    env_booted_dev,			/* 0x04 */
    env_boot_file,			/* 0x05 */
    env_booted_file,			/* 0x06 */
    env_boot_osflags,			/* 0x07 */
    env_booted_osflags,			/* 0x08 */
    env_boot_reset,			/* 0x09 */
    env_dump_dev,			/* 0x0A */
    env_enable_audit,			/* 0x0B */
    env_license,			/* 0x0C */
    env_char_set,			/* 0x0D */
    (char *)&env_language,		/* 0x0E */
    env_tty_dev,			/* 0x0F */
    0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,	/* 0x10 - 0x1F */
    0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,	/* 0x20 - 0x2F */
    0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,	/* 0x30 - 0x3F */
    0,					/* 0x40 */
    0,					/* 0x41 */
    env_scsiid,				/* 0x42 */
    env_scsifast,			/* 0x43 */
    env_com1_baud,			/* 0x44 */
    env_com1_modem,			/* 0x45 */
    env_com1_flow,			/* 0x46 */
    env_com1_misc,			/* 0x47 */
    env_com2_baud,			/* 0x48 */
    env_com2_modem,			/* 0x49 */
    env_com2_flow,			/* 0x4A */
    env_com2_misc,			/* 0x4B */
    env_password,			/* 0x4C */
    env_secure,				/* 0x4D */
    env_logfail,			/* 0x4E */
    env_srm2dev_id,			/* 0x4F */
    0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,	/* 0x50 - 0x5F */
    0,					/* 0x60 */
    0,					/* 0x61 */
    0,					/* 0x62 */
    0,					/* 0x63 */
};

long
CallBackDispatcher(long a0, long a1, long a2, long a3, long a4)
{
    long i;
    switch (a0) {
      case CONSCB_GETC:
        return GetChar();

      case CONSCB_PUTS:
        for (i = 0; i < a3; i++)
            PutChar(*((char *)a2 + i));
        return a3;

      case CONSCB_GETENV:
        if (a1 >= 0 && a1 < MAX_ENV_INDEX && envptr[a1] != 0 && *envptr[a1]) {
            i = strcpy((char*)a2, envptr[a1]);
        } else {
            strcpy((char*)a2, "");
            i = (long)0xc000000000000000;
            if (a1 >= 0 && a1 < MAX_ENV_INDEX)
                printf_lock("GETENV unsupported option %d (0x%x)\n", a1, a1);
            else
                printf_lock("GETENV unsupported option %s\n", a1);
        }

        if (i > a3)
            panic("CONSCB_GETENV overwrote buffer\n");
        return i;

      case CONSCB_OPEN:
        bcopy((char*)a1, deviceState[numOpenDevices].name, a2);
        deviceState[numOpenDevices].name[a2] = '\0';
        printf_lock("CONSOLE OPEN : %s --> success \n",
                    deviceState[numOpenDevices].name);
        return numOpenDevices++;

      case CONSCB_READ:
        DeviceOperation(a0, a1, a2, a3, a4);
        break;

      case CONSCB_CLOSE:
        break;

      case CONSCB_OPEN_CONSOLE:
        printf_lock("CONSOLE OPEN\n");
        return 0; /* success */
        break; /* not reached */

      case CONSCB_CLOSE_CONSOLE:
        printf_lock("CONSOLE CLOSE\n");
        return 0; /* success */
        break; /* not reached */

      default:
        panic("CallBackDispatcher(%x,%x,%x,%x,%x)\n", a0, a1, a2, a3, a4);
    }

    return 0;
}

long
CallBackFixup(int a0, int a1, int a2)
{
    long temp;
    /*
     * Linux uses r8 for the current pointer (pointer to data
     * structure contating info about currently running process). It
     * is set when the kernel starts and is expected to remain
     * there... Problem is that the unlike the kernel, the console
     * does not prevent the assembler from using r8. So here is a work
     * around. So far this has only been a problem in CallBackFixup()
     * but any other call back functions couldd cause a problem at
     * some point
     */

    /* save off the current pointer to a temp variable */
    asm("bis $8, $31, %0" : "=r" (temp));

    /* call original code */
    printf_lock("CallbackFixup %x %x, t7=%x\n", a0, a1, temp);

    /* restore the current pointer */
    asm("bis %0, $31, $8" : : "r" (temp) : "$8");

    return 0;
}

void
SlaveCmd(int cpu, struct rpb_percpu *my_rpb)
{
    extern ulong palJToSlave[];

    printf_lock("Slave CPU %d console command %s", cpu,
                my_rpb->rpb_iccb.iccb_rxbuf);

    my_rpb->rpb_state |= STATE_BIP;
    my_rpb->rpb_state &= ~STATE_RC;

    printf_lock("SlaveCmd: restart %x %x vptb %x my_rpb %x my_rpb_phys %x\n",
                rpb->rpb_restart, rpb->rpb_restart_pv, rpb->rpb_vptb, my_rpb,
                KSEG_TO_PHYS(my_rpb));

    cServe(KSEG_TO_PHYS(palJToSlave), (ulong)rpb->rpb_restart,
           CSERVE_K_JTOPAL, rpb->rpb_restart_pv, rpb->rpb_vptb,
           KSEG_TO_PHYS(my_rpb));

    panic("SlaveCmd returned \n");
}

void
SlaveLoop(int cpu)
{
    int size = ROUNDUP128(sizeof(struct rpb_percpu));
    struct rpb_percpu *my_rpb = (struct rpb_percpu*)
        ((ulong)rpb_percpu + size * cpu);

    if (cpu == 0) {
        panic("CPU 0 entering slaveLoop. Reenetering the console. HOSED\n");
    } else {
        printf_lock("Entering slaveloop for cpu %d my_rpb=%x\n", cpu, my_rpb);
    }

    // swap the processors context to the one in the
    // rpb_percpu struct very carefully (i.e. no stack usage)
    // so that linux knows which processor ends up in __smp_callin
    // and we don't trash any data is the process
    SlaveSpin(cpu, my_rpb, &my_rpb->rpb_iccb.iccb_rxlen);
}
