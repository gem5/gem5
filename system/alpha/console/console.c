

/* ******************************************
 * SimOS SRM  Console
 *
 * Derived from Lance Berc's SRM console
 * for the SRC XXM Machine
 * ******************************************/


typedef unsigned long long uint64_t;
typedef unsigned long long uint64;
typedef unsigned int uint32_t;
typedef unsigned int uint32;

#define CONSOLE
#include "alpha_access.h"
#include "machine_defs.h"

#if 0
#include "new_aouthdr.h"
#include "srcmax.h"
#endif

/* from ../h */
#include "lib.h"
#include "rpb.h"
#include "cserve.h"


#define CONS_INT_TX   0x01  /* interrupt enable / state bits */
#define CONS_INT_RX   0x02

#define KSEG   0xfffffc0000000000
#define K1BASE 0xfffffc8000000000
#define KSEG_TO_PHYS(x)(((ul)x) & ~KSEG)

#define CDR ((volatile DevConsoleRegisters *) \
             (__MAGIC_ZONE(0, 0, MAGIC_ZONE_BDOOR_DEV) + __MAGIC_BDOOR_CNSLE_OFFS))


#define PHYS_TO_K1(_x) (K1BASE|(_x))

#define AOUT_LOAD_ADDR (KSEG|0xf000)

#define ROUNDUP8(x) ((ul)(((ul)x)+7) & ~7)
#define ROUNDUP128(x) ((ul)(((ul)x)+127) & ~127)
#define ROUNDUP8K(x) ((ul)(((ul)(x))+8191) & ~8191)

#define FIRST(x)  ((((ul)(x)) >> 33) & 0x3ff)
#define SECOND(x) ((((ul)(x)) >> 23) & 0x3ff)
#define THIRD(x) ((((ul)(x)) >> 13) & 0x3ff)
#define THIRD_XXX(x)  ((((ul)(x)) >> 13) & 0xfff)
#define PFN(x)  ((((ul)(x) & ~KSEG) >> 13))

/* Kernel write | kernel read | valid */
#define KPTE(x) ((ul)((((ul)(x)) << 32) | 0x1101))

#define HWRPB_PAGES 4
#define MDT_BITMAP_PAGES  4

#define CSERVE_K_JTOKERN       0x18

#define NUM_KERNEL_THIRD (4)


static unixBoot(int go, int argc, char **argv);
void jToPal(ul bootadr);
void SlaveLoop(int cpu);


struct AlphaAccess simosConf;

/* **************************************************************
 * Console callbacks use VMS calling conventions
 * read AXP manual, 2-64.
 * ***************************************************************/
typedef struct OpenVMSFunc {
   long dummy;
   long func;
}OpenVMSFunc;

OpenVMSFunc callbackFunc, fixupFunc;




ul theLock;


extern void SpinLock(ul *lock);
#define SpinUnlock(_x) *(_x) = 0;

struct _kernel_params {
   char *bootadr;
   ul rpb_percpu;
   ul free_pfn;
   ul argc;
   ul argv;
   ul envp; /* NULL */
};


extern consoleCallback[];
extern consoleFixup[];
long CallBackDispatcher();
long CallBackFixup();

/*
 * simos console output
 */

void InitConsole(void)
{
#if 0
   CDR->intr_status =(DevRegister)(DEV_CNSLE_RX_INTR |DEV_CNSLE_TX_INTR);
#endif
}

char GetChar()
{
   struct AlphaAccess *k1Conf = (struct AlphaAccess *)(__MAGIC_ZONE(0, 0, MAGIC_ZONE_EV5_ALIAS));
   return k1Conf->inputChar;
}

void PutChar(char c)
{
#if 0
   CDR->data = c;
#endif
#if 0
   *(int*) PHYS_TO_K1(SLOT_D_COM1<<5) = c;
#endif
   struct AlphaAccess *k1Conf = (struct AlphaAccess *)(__MAGIC_ZONE(0, 0, MAGIC_ZONE_EV5_ALIAS));
   k1Conf->outputChar = c;

}


int
passArgs(int argc)
{ return 0; }

int
main(int argc, char **argv)
{
   int x,i;
   struct AlphaAccess *k1Conf = (struct AlphaAccess *)(__MAGIC_ZONE(0, 0, MAGIC_ZONE_EV5_ALIAS));
   ui *k1ptr,*ksegptr;


   InitConsole();
   printf("SimOS console \n");
   /*
    * get configuration from backdoor
    */
   simosConf.last_offset = k1Conf->last_offset;
   printf(" Got simosConfiguration %d \n",simosConf.last_offset);

   for (i=1;i<=simosConf.last_offset/4;i++) {
      ui *k1ptr = (ui*)k1Conf + i;
      ui *ksegptr = (ui*)(&simosConf.last_offset)+i;
      *ksegptr = *k1ptr;

   }

   if (simosConf.version != ALPHA_ACCESS_VERSION)  {
      panic("Console version mismatch. Console expects %d. SimOS has %d \n",
            ALPHA_ACCESS_VERSION,simosConf.version);
   }


   /*
    * setup arguments to kernel
    */
   unixBoot(1,argc,argv);

   x = *(volatile int *)(K1BASE-4);
   while(1) continue;
   return x;
}

/*
 * BOOTING
 */
struct rpb xxm_rpb = {
   NULL,		/* 000: physical self-reference */
   ((long)'H') | (((long)'W') << 8) | (((long)'R') << 16) |
   ((long)'P' << 24) | (((long)'B') << 32),  /* 008: contains string "HWRPB" */
   6,			/* 010: HWRPB version number */
   /* the byte count is wrong, but who needs it? - lance */
   0,			/* 018: bytes in RPB perCPU CTB CRB MEDSC */
   0,			/* 020: primary cpu id */
   8192,		/* 028: page size in bytes */
   43,		/* 030: number of phys addr bits */
   127,		/* 038: max valid ASN */
   {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1'},	/* 040: system serial num: 10 ascii chars */
#ifdef undef
/* To be legitimate, the following system type and variation are correct for the XXM.
   But there are too many #ifdefs etc to deal with in Unix, so we tell the kernel
   that we're an Avanti, which is similar enough.
   */
   31,		/* 050: system type - XXM is now in the Alpha SRM */
   (1 << 10) | (2<<1),/* 058: system variation - XXM w/EV5 & embeded console */
#endif
#if 0
   0x12,		/* 050: system type - masquarade as some random 21064 */
#endif
   12, /* masquerade a DEC_3000_500 (bugnion) */
   (2<<1),		/* 058: system variation */
   'c'|('o'<<8)|('o'<<16)|('l'<< 24),		/* 060: system revision */
   1024*4096,		/* 068: scaled interval clock intr freq  OVERRIDEN*/
   0,			/* 070: cycle counter frequency */
   0x200000000,	/* 078: virtual page table base */
   0,			/* 080: reserved */
   0,			/* 088: offset to translation buffer hint */
   1,			/* 090: number of processor slots OVERRIDDEN*/
   sizeof(struct rpb_percpu),	/* 098: per-cpu slot size. OVERRIDDEN */
   0,			/* 0A0: offset to per_cpu slots */
   1,			/* 0A8: number of CTBs */
#ifdef bugnion_gone
   sizeof(struct rpb_ctb),	/* 0B0: bytes in largest CTB */
#else
   sizeof(struct ctb_tt),
#endif
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

ul xxm_tbb[] = { 0x1e1e1e1e1e1e1e1e, 0x1e1e1e1e1e1e1e1e, 0x1e1e1e1e1e1e1e1e, 0x1e1e1e1e1e1e1e1e,
                 0x1e1e1e1e1e1e1e1e, 0x1e1e1e1e1e1e1e1e, 0x1e1e1e1e1e1e1e1e, 0x1e1e1e1e1e1e1e1e};

struct rpb_percpu xxm_rpb_percpu = {
   {0,0,0,0,0,0,0,{0,0},{0,0,0,0,0,0,0,0}},				/* 000: boot/restart HWPCB */
   (STATE_PA | STATE_PP | STATE_CV | STATE_PV | STATE_PMV | STATE_PL), 	/* 080: per-cpu state bits */
   0xc000,				/* 088: palcode memory length */
   0x2000,				/* 090: palcode scratch length */
   0x4000,				/* 098: phys addr of palcode mem space */
   0x2000,				/* 0A0: phys addr of palcode scratch space */
   (2 << 16) | (5 << 8) | 1,		/* 0A8: PALcode rev required */
   5 | (2L  << 32),				/* 0B0: processor type */
   7,					/* 0B8: processor variation */
   'D'|('a'<<8)|('v'<<16)|('e'<<24),	/* 0C0: processor revision */
   {'D','a','v','e','C','o','n','r','o','y',0,0,0,0,0,0},	/* 0C8: proc serial num: 10 ascii chars */
   0,					/* 0D8: phys addr of logout area */
   0,					/* 0E0: length in bytes of logout area */
   0,					/* 0E8: halt pcb base */
   0,					/* 0F0: halt pc */
   0,					/* 0F8: halt ps */
   0,					/* 100: halt arg list (R25) */
   0,					/* 108: halt return address (R26) */
   0,					/* 110: halt procedure value (R27) */
   0,		       			/* 118: reason for halt */
   0,		       			/* 120: for software */
   {0},				/* 128: inter-console communications buffer */
   {1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0},	/* 1D0: PALcode revs available */
   0					/* 250: reserved for arch use */
/* the dump stack grows from the end of the rpb page not to reach here */
};

struct _xxm_rpb_mdt {
   long   rpb_checksum;	/* 000: checksum of entire mem desc table */
   long   rpb_impaddr;		/* 008: PA of implementation dep info */
   long   rpb_numcl;		/* 010: number of clusters */
   struct rpb_cluster rpb_cluster[3];	/* first instance of a cluster */
};

struct _xxm_rpb_mdt xxm_rpb_mdt = {
   0,		/* 000: checksum of entire mem desc table */
   0,		/* 008: PA of implementation dep info */
   0,		/* 010: number of clusters */
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

struct rpb_ctb xxm_rpb_ctb = {
   CONS_DZ,	/* 000: console type */
   0,		/* 008: console unit */
   0,		/* 010: reserved */
   0		/* 018: byte length of device dep portion */
};

/* we don't do any fixup (aka relocate the console) - we hope */
struct rpb_crb xxm_rpb_crb = {
   0,		/* va of call-back dispatch rtn */
   0,		/* pa of call-back dispatch rtn */
   0,		/* va of call-back fixup rtn */
   0,		/* pa of call-back fixup rtn */
   0,		/* number of entries in phys/virt map */
   0		/* Number of pages to be mapped */
};

struct _rpb_name {
   unsigned long length;
   char name[16];
};

extern struct _rpb_name xxm_name;

struct rpb_dsr xxm_rpb_dsr = {
   0,
   0,
   0,
};

struct _rpb_name xxm_name = {
   16,
   {'D','E','C',' ','S','R','C',' ','X','X','M',' ','D','G','C',0},
};

/* XXM has one LURT entry - 1050 is for workstations, 1100 is servers (and is needed for CXX) */
long xxm_lurt[10] = { 9, 12, -1, -1, -1, -1, -1, -1, 1100, 1100 };

ul unix_boot_mem;
unsigned long bootadr;
#if 0
unsigned long  aout_bss_addr, aout_bss_size, aout_entry, aout_text_start, aout_data_addr;
#endif
char **kargv;
int kargc;
ul free_pfn;
struct rpb_percpu *rpb_percpu;


#define MAX_CPUS 32

ul bootStrapImpure[MAX_CPUS];


char *unix_boot_alloc(int pages)
{
   char *ret = (char *) unix_boot_mem;
   unix_boot_mem += (pages * 8192);
   return ret;
}

ul *first = 0;
ul *third_rpb = 0;
ul *reservedFixup = 0;

int strcpy(char *dst, char *src);

struct rpb *rpb;

unixBoot(int go, int argc, char **argv)
{
   ul *second,  *third_kernel, ptr, *tbb, size, *percpu_logout;
   unsigned char *mdt_bitmap;
   long *lp1, *lp2, sum;
   int i, cl;
   int kern_first_page;
   int mem_size = simosConf.mem_size;

   int mem_pages = mem_size / 8192, cons_pages;
   ul kernel_bytes, ksp, kernel_end, *unix_kernel_stack, bss, ksp_bottom, ksp_top;
   struct rpb_ctb *rpb_ctb;
   struct ctb_tt *ctb_tt;
   struct rpb_dsr *rpb_dsr;
   struct rpb_crb *rpb_crb;
   struct _xxm_rpb_mdt *rpb_mdt;
   int *rpb_lurt;
   char *rpb_name;
   ul nextPtr;

   printf( "memsize %x pages %x \n",mem_size,mem_pages);



#ifdef notnow
   if (unixArgs()) return;
#endif

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

   printf("First free page after ROM 0x%x\n", unix_boot_mem);

   rpb = (struct rpb *) unix_boot_alloc( HWRPB_PAGES);

   mdt_bitmap =  (unsigned char *) unix_boot_alloc(MDT_BITMAP_PAGES);
   first = (ul *)unix_boot_alloc(1);
   second = (ul *)unix_boot_alloc(1);
   third_rpb = (ul *)unix_boot_alloc(1);
   reservedFixup = (ul*) unix_boot_alloc(1);
   third_kernel = (ul *)unix_boot_alloc(NUM_KERNEL_THIRD);
   percpu_logout = (ul*)unix_boot_alloc(1);


   cons_pages = KSEG_TO_PHYS(unix_boot_mem) / 8192;

   /* Set up the page tables */
   bzero((char *)first, 8192);
   bzero((char *)second, 8192);
   bzero((char *)reservedFixup,8192);
   bzero((char *)third_rpb, HWRPB_PAGES * 8192);
   bzero((char *)third_kernel, 8192 * NUM_KERNEL_THIRD);

   first[0] = KPTE(PFN(second));
   first[1] = KPTE(PFN(first)); /* Region 3 */

   second[SECOND(0x10000000)] = KPTE(PFN(third_rpb));	/* Region 0 */
   for (i=0;i<NUM_KERNEL_THIRD;i++) {
      second[SECOND(0x20000000)+i] = KPTE(PFN(third_kernel)+i);	/* Region 1 */
   }
   second[SECOND(0x40000000)] = KPTE(PFN(second));	/* Region 2 */


   {

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
#ifdef not_not
#define DATABASE_END            0x20230000 /* don't need all that */
#endif

#define DATABASE_END            0x20020000

      int i;
      ul *dbPage = (ul*)unix_boot_alloc(1);
      second[SECOND(DATABASE_BASE)] = KPTE(PFN(dbPage));
      for (i=DATABASE_BASE; i <DATABASE_END ; i+= 8096) {
         ul *db = (ul*)unix_boot_alloc(1);
         dbPage[THIRD(i)] = KPTE(PFN(db));
      }
   }

   /* Region 0 */
   /* Map the HWRPB */
   for (i = 0; i < HWRPB_PAGES; i++) third_rpb[i] = KPTE(PFN(rpb) + i);

   /* Map the MDT bitmap table */
   for (i=0;i<MDT_BITMAP_PAGES;i++) {
      third_rpb[HWRPB_PAGES+i] = KPTE(PFN(mdt_bitmap)+i);
   }

   /* Protect the PAL pages */
   for (i = 1; i < PFN(first); i++) third_rpb[HWRPB_PAGES + MDT_BITMAP_PAGES + i] = KPTE(i);

   /* Set up third_kernel after it's loaded, when we know where it is */

#ifdef original__xxm
   if (unixLoadKernel(AOUT_LOAD_ADDR, argv[1]) == -1) return;
   aoutfixup(AOUT_LOAD_ADDR);
#else
   /* aoutfixup(simosConf.kernelFileHdr); */
#endif
#if 0
   bss = aout_bss_addr;

   kern_first_page = (KSEG_TO_PHYS(aout_text_start) / 8192);
   kernel_end = ksp_top = ROUNDUP8K(aout_bss_addr + aout_bss_size);
   bootadr = aout_entry;
#endif

   kern_first_page = (KSEG_TO_PHYS(simosConf.kernStart)/8192);
   kernel_end = ksp_top = ROUNDUP8K(simosConf.kernEnd);
   bootadr = simosConf.entryPoint;


   printf("HWRPB 0x%x l1pt 0x%x l2pt 0x%x l3pt_rpb 0x%x l3pt_kernel 0x%x l2reserv 0x%x\n",
          rpb, first, second, third_rpb, third_kernel,reservedFixup);
   if (kernel_end - simosConf.kernStart > (0x800000*NUM_KERNEL_THIRD)) {
      printf("Kernel is more than 8MB 0x%x - 0x%x = 0x%x\n",
             kernel_end, simosConf.kernStart,
             kernel_end -simosConf.kernStart );
      panic("kernel too big\n");

   }
   /* Map the kernel's pages into the third level of region 2 */

   for (ptr = simosConf.kernStart; ptr < kernel_end; ptr += 8192) {

      third_kernel[THIRD_XXX(ptr)] = KPTE(PFN(ptr));
   }
   /* blow 2 pages of phys mem for guards since it maintains 1-to-1 mapping */
   ksp = ksp_top + (3 * 8192);
   if (ksp - simosConf.kernStart > (0x800000*NUM_KERNEL_THIRD)) {
      printf("Kernel stack pushd us over 8MB\n");
      panic("ksp too big\n");
   }
   if (THIRD_XXX((ul)ksp_top) >  NUM_KERNEL_THIRD * 1024) {
      panic("increase NUM_KERNEL_THIRD, and change THIRD_XXX\n");
   }
   ptr = (ul) ksp_top;
   bzero((char *)ptr, 8192 * 2);
   third_kernel[THIRD_XXX(ptr)] = 0;			/* Stack Guard Page */
   ptr += 8192;
   third_kernel[THIRD_XXX(ptr)] = KPTE(PFN(ptr));	/* Kernel Stack Pages */
   ptr += 8192;
   third_kernel[THIRD_XXX(ptr)] = KPTE(PFN(ptr));
   ptr += 8192;
   third_kernel[THIRD_XXX(ptr)] = 0;			/* Stack Guard Page */

   /* put argv into the bottom of the stack - argv starts at 1 because
    * the command thatr got us here (i.e. "unixboot) is in argv[0].
    */
   ksp -= 8;			/* Back up one longword */
   ksp -= argc * sizeof(char *);	/* Make room for argv */
   kargv = (char **) ksp;
   for (i = 1; i < argc; i++) {	/* Copy arguments to stack */
      ksp -= ((strlen(argv[i]) + 1) + 7) & ~0x7;
      kargv[i-1] = (char *) ksp;
      strcpy(kargv[i-1], argv[i]);
   }
   kargc = i - 1;
   kargv[kargc] = NULL;		/* just to be sure; doesn't seem to be used */
   ksp -= sizeof(char *);	/* point above last arg for no real reason */

   free_pfn = PFN(ptr);

   bcopy((char *)&xxm_rpb, (char *)rpb, sizeof(struct rpb));

   rpb->rpb_selfref = (struct rpb *) KSEG_TO_PHYS(rpb);
   rpb->rpb_string = 0x0000004250525748;

   tbb = (ul *) (((char *) rpb) + ROUNDUP8(sizeof(struct rpb)));
   rpb->rpb_trans_off = (ul)tbb - (ul)rpb;
   bcopy((char *)xxm_tbb, (char *)tbb, sizeof(xxm_tbb));


   /*
    * rpb_counter. Use to determine timeouts in OS.
    * XXX must be patched after a checkpoint restore (I guess)
    */

   printf("CPU Clock at %d MHz IntrClockFrequency=%d \n", simosConf.cpuClock,simosConf.intrClockFrequency);
   rpb->rpb_counter = simosConf.cpuClock * 1000 * 1000;

   /*
    * By definition, the rpb_clock is scaled by 4096 (in hz)
    */
   rpb->rpb_clock = simosConf.intrClockFrequency * 4096;



   /*
    * Per CPU Slots. Multiprocessor support.
    */
   {
      int i;
      int size = ROUNDUP128(sizeof(struct rpb_percpu));

      printf("Booting with %d processor(s) \n",simosConf.numCPUs);

      rpb->rpb_numprocs = simosConf.numCPUs;
      rpb->rpb_slotsize = size;
      rpb_percpu = (struct rpb_percpu *)
         ROUNDUP128(((ul) tbb) +(sizeof(xxm_tbb)));

      rpb->rpb_percpu_off = (ul)rpb_percpu - (ul)rpb;

      for (i=0;i<simosConf.numCPUs;i++) {
         struct rpb_percpu *thisCPU = (struct rpb_percpu*)
            ((ul)rpb_percpu + size*i);

         bzero((char *)thisCPU, size);
         bcopy((char *)&xxm_rpb_percpu,
               (char *)thisCPU,
               sizeof(struct rpb_percpu));

         thisCPU->rpb_pcb.rpb_ksp = ksp;
         thisCPU->rpb_pcb.rpb_ptbr = PFN(first);

         thisCPU->rpb_logout = KSEG_TO_PHYS(percpu_logout);
         thisCPU->rpb_logout_len = 8192;

/*  thisCPU->rpb_pcb.rpb_ptbr = PFN(second);*/

         printf("KSP: 0x%x PTBR 0x%x\n", thisCPU->rpb_pcb.rpb_ksp, thisCPU->rpb_pcb.rpb_ptbr);

         if (i) {
            bootStrapImpure[i] = (ul)unix_boot_alloc(1);
         }

      }

      nextPtr = (ul)rpb_percpu + size*simosConf.numCPUs;
   }

   /*
    * Console Terminal Block
    */


      rpb_ctb = (struct rpb_ctb *) nextPtr;
      ctb_tt = (struct ctb_tt*) rpb_ctb;

      rpb->rpb_ctb_off = ((ul)rpb_ctb) - (ul)rpb;
      rpb->rpb_ctb_size  = sizeof(struct rpb_ctb);

   bzero((char *)rpb_ctb, sizeof(struct ctb_tt));

#ifdef original_xxm
   if (tga_slot == -1)
      rpb_ctb->rpb_type = CONS_DZ;
  else {
    rpb_ctb->rpb_type = CONS_GRPH;
    rpb_ctb->rpb_unit = (SLOTINFO_PCI << 16) | (0 << 8) | tga_slot;
  }
#else
  rpb_ctb->rpb_type = CONS_DZ;
#endif

  rpb_ctb->rpb_length = sizeof(ctb_tt)-sizeof(rpb_ctb);

  /*
   * uart initizliation
   */
  ctb_tt->ctb_csr = 0;
  ctb_tt->ctb_tivec = 0x6c0;  /* matches tlaser pal code */
  ctb_tt->ctb_rivec = 0x680;  /* matches tlaser pal code */
  ctb_tt->ctb_baud = 9600;
  ctb_tt->ctb_put_sts = 0;
  ctb_tt->ctb_get_sts = 0;


  rpb_crb = (struct rpb_crb *) (((ul)rpb_ctb) + sizeof(struct ctb_tt));
  rpb->rpb_crb_off = ((ul)rpb_crb) - (ul)rpb;

  bzero((char *)rpb_crb, sizeof(struct rpb_crb));
  /*
   * console callback stuff (simos)
   */

  rpb_crb->rpb_num = 1;
  rpb_crb->rpb_mapped_pages = HWRPB_PAGES;
  rpb_crb->rpb_map[0].rpb_virt = 0x10000000;
  rpb_crb->rpb_map[0].rpb_phys = ((ul)rpb) & ~0x1fff;
  rpb_crb->rpb_map[0].rpb_pgcount = HWRPB_PAGES;


  printf("Console Callback at 0x%x, fixup at 0x%x \n",
          rpb_crb->rpb_va_disp,
          rpb_crb->rpb_va_fixup );

  rpb_mdt = (struct _xxm_rpb_mdt *) (((ul)rpb_crb) + sizeof(struct rpb_crb));
  rpb->rpb_mdt_off = (ul)rpb_mdt - (ul)rpb;
  bcopy((char *)&xxm_rpb_mdt, (char *)rpb_mdt, sizeof(struct _xxm_rpb_mdt));


  cl = 0;
#ifdef undef
  /* Until Digital Unix can handle it, account all pages below the kernel
   * as "console" memory. */
  rpb_mdt->rpb_cluster[cl].rpb_pfncount = cons_pages;
#endif
  rpb_mdt->rpb_cluster[cl].rpb_pfncount = kern_first_page;
  cl++;

  rpb_mdt->rpb_cluster[cl].rpb_pfn = kern_first_page;
  rpb_mdt->rpb_cluster[cl].rpb_pfncount = mem_pages - kern_first_page;
  rpb_mdt->rpb_cluster[cl].rpb_pfntested=rpb_mdt->rpb_cluster[cl].rpb_pfncount;
  rpb_mdt->rpb_cluster[cl].rpb_pa = KSEG_TO_PHYS(mdt_bitmap);
  rpb_mdt->rpb_cluster[cl].rpb_va = 0x10000000 + HWRPB_PAGES * 8192;
  cl++;

#ifdef undef
  /* The stupid Unix kernel needs to have all mdt clusters in ascending
   * order, and the last cluster is used to compute the top of memory.
   * It can't make use of memory between the console and the kernel.
   */
  rpb_mdt->rpb_cluster[cl].rpb_pfn = cons_pages;
  rpb_mdt->rpb_cluster[cl].rpb_pfncount = kern_first_page - cons_pages;
  rpb_mdt->rpb_cluster[cl].rpb_pfntested=rpb_mdt->rpb_cluster[cl].rpb_pfncount;
  rpb_mdt->rpb_cluster[cl].rpb_pa = KSEG_TO_PHYS(mdt_bitmap);
  rpb_mdt->rpb_cluster[cl].rpb_va = 0x10000000 + HWRPB_PAGES * 8192;
  cl++;
#endif

  rpb_mdt->rpb_numcl = cl;

  for (i = 0; i < cl; i++)
    printf("Memory cluster %d [%d - %d]\n", i, rpb_mdt->rpb_cluster[i].rpb_pfn, rpb_mdt->rpb_cluster[i].rpb_pfncount);



  /* Checksum the rpb for good luck */
  sum = 0;
  lp1 = (long *)&rpb_mdt->rpb_impaddr;
  lp2 = (long *)&rpb_mdt->rpb_cluster[cl];
  while (lp1 < lp2) sum += *lp1++;
  rpb_mdt->rpb_checksum = sum;

  /* XXX should checksum the cluster descriptors */

  bzero((char *)mdt_bitmap, MDT_BITMAP_PAGES * 8192);
  for (i = 0; i < mem_pages/8; i++) ((unsigned char *)mdt_bitmap)[i] = 0xff;

  printf("Initalizing mdt_bitmap addr 0x%x mem_pages %x \n",
         (long)mdt_bitmap,(long)mem_pages);

  xxm_rpb.rpb_config_off = 0;
  xxm_rpb.rpb_fru_off = 0;

  rpb_dsr = (struct rpb_dsr *) (((ul)rpb_mdt) + sizeof(struct _xxm_rpb_mdt));
  rpb->rpb_dsr_off = ((ul)rpb_dsr) - (ul)rpb;
  bzero((char *)rpb_dsr, sizeof(struct rpb_dsr));
  rpb_dsr->rpb_smm = 1578; /* Official XXM SMM number as per SRM */
  rpb_dsr->rpb_smm = 1089; /* Official Alcor SMM number as per SRM */

  rpb_lurt = (int *) ROUNDUP8(((ul)rpb_dsr) + sizeof(struct rpb_dsr));
  rpb_dsr->rpb_lurt_off = ((ul) rpb_lurt) - (ul) rpb_dsr;
  bcopy((char *)xxm_lurt, (char *)rpb_lurt, sizeof(xxm_lurt));

  rpb_name = (char *) ROUNDUP8(((ul)rpb_lurt) + sizeof(xxm_lurt));
  rpb_dsr->rpb_sysname_off = ((ul) rpb_name) - (ul) rpb_dsr;
#define THENAME "             SimOS ALPHA/EV5"
  sum = sizeof(THENAME);
  bcopy(THENAME, rpb_name, sum);
  *(ul *)rpb_name = sizeof(THENAME); /* put in length field */

  /* calculate size of rpb */
  rpb->rpb_size = ((ul) &rpb_name[sum]) - (ul)rpb;

  if (rpb->rpb_size > 8192*HWRPB_PAGES) {
     panic("HWRPB_PAGES=%d too small for HWRPB !!! \n");
  }


 {
     ul *ptr = (ul*)((char*)rpb_dsr + sizeof(struct rpb_dsr ));
     rpb_crb->rpb_pa_disp = KSEG_TO_PHYS(ptr);
#if 0
     rpb_crb->rpb_va_disp = 0x10000000 + ((ul)ptr&(0x2000*HWRPB_PAGES-1));
#else
     rpb_crb->rpb_va_disp = 0x10000000 + ((ul)ptr & 0x1fff);
#endif
     printf("ConsoleDispatch at virt %x phys %x val %x\n",
             rpb_crb->rpb_va_disp,
            rpb_crb->rpb_pa_disp,
            consoleCallback);
     *ptr++ = 0;
     *ptr++ = (ul) consoleCallback;
     rpb_crb->rpb_pa_fixup = KSEG_TO_PHYS(ptr);
#if 0
     rpb_crb->rpb_va_fixup = 0x10000000 + ((ul)ptr& (0x2000*HWRPB_PAGES-1));
#else
     rpb_crb->rpb_va_fixup = 0x10000000 + ((ul)ptr & 0x1fff);
#endif
     *ptr++ = 0;
     *ptr++ = (ul) consoleFixup;
  }


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

  {
     int i;
     for (i=1;i<simosConf.numCPUs;i++) {
        volatile struct AlphaAccess *k1Conf = (volatile struct AlphaAccess *)
           (__MAGIC_ZONE(0, 0, MAGIC_ZONE_EV5_ALIAS));
        SpinLock(&theLock);
        printf("Bootstraping CPU %d with sp=0x%x \n",
               i,bootStrapImpure[i]);
        SpinUnlock(&theLock);
        k1Conf->bootStrapImpure = bootStrapImpure[i];
        k1Conf->bootStrapCPU = i;
     }
  }

  /*
   * Make sure that we are not stepping on the kernel
   */
  if ((ul)unix_boot_mem >= (ul)simosConf.kernStart) {
     panic("CONSOLE: too much memory. Smashing kernel  \n");
  } else {
     SpinLock(&theLock);
     printf("unix_boot_mem ends at %x \n",unix_boot_mem);
     SpinUnlock(&theLock);
  }


#ifdef undef
#define CSERVE_K_JTOKERN	0x18
  cServe(bootadr, (ul) rpb_percpu, CSERVE_K_JTOKERN, free_pfn);
#endif

  if (go) JToKern(bootadr, rpb_percpu, free_pfn, kargc, kargv, NULL);
}


#if 0
aoutfixup(char *p)
{
  int i;
  unsigned long rem, len, off, dst;


  struct new_aouthdr *ao = (struct new_aouthdr *) &p[NEW_FILHSZ];
#if 0
  struct scnhdr *s = (struct scnhdr *) &p[FILHSZ + AOUTHSZ];
  struct scnhdr *t, *d, *b;
  printf("aoutfixup: %d sections \n",fh->f_nscns);
#endif


  aout_text_start = ((ul)ao->text_start_hi<<32) + ao->text_start;
  aout_data_addr = ((ul)ao->data_start_hi<<32) + ao->data_start;
  aout_bss_addr = ((ul)ao->bss_start_hi<<32) + ao->bss_start;
  aout_bss_size = ((ul)ao->bsize_hi<<32) +  ao->bsize;
  aout_entry = ((ul)ao->entry_hi<<32) + ao->entry;

  printf("_text 0x%16x %8d @ %08d\n", aout_text_start, ao->tsize,0 /* t->s_scnptr*/);
  printf("_data 0x%16x %8d @ %08d\n", aout_data_addr, ao->dsize,0/* d->s_scnptr*/);
  printf("_bss  0x%16x %8d\n", aout_bss_addr,  ao->bsize);
  printf("entry 0x%16x\n", aout_entry);
#if 0
  for (i = 0; i < fh->f_nscns; i++) {
     printf("section %d %s \n",i,s[i].s_name);
    if (!strcmp(s[i].s_name, ".text")) t = &s[i];
    else if (!strcmp(s[i].s_name, ".data")) d = &s[i];
    else if (!strcmp(s[i].s_name, ".bss")) b = &s[i];
  }
  bcopy(&p[t->s_scnptr], (char *)ao->text_start, ao->tsize);
  bcopy(&p[d->s_scnptr], (char *)ao->data_start, ao->dsize);
#endif
}
#endif

extern ui palJToKern[];

JToKern(bootadr, rpb_percpu, free_pfn, k_argc, k_argv, envp)
char * bootadr;
ul rpb_percpu;
ul free_pfn;
ul k_argc;
char **k_argv;
char **envp;
{
  struct _kernel_params *kernel_params = (struct _kernel_params *) KSEG;
  int i;

  printf("k_argc = %d ", k_argc);
  for (i = 0; i < k_argc; i++) {
    printf("'%s' ", k_argv[i]);
  }
  printf("\n");

/*  rpb_percpu |= 0xfffffc0000000000;*/
  kernel_params->bootadr = bootadr;
  kernel_params->rpb_percpu = KSEG_TO_PHYS(rpb_percpu);
  kernel_params->free_pfn = free_pfn;
  kernel_params->argc = k_argc;
  kernel_params->argv = (ul)k_argv;
  kernel_params->envp = (ul)envp;
  printf("jumping to kernel at 0x%x, (PCBB 0x%x pfn %d)\n", bootadr, rpb_percpu, free_pfn);
  jToPal(KSEG_TO_PHYS((ul)palJToKern));
  printf("returned from jToPal. Looping\n");
  while(1) continue;
}


void jToPal(ul bootadr)
{
  cServe(bootadr, 0, CSERVE_K_JTOPAL);

/*
 * Make sure that floating point is enabled incase
 * it was disabled by the user program.
 */
  wrfen(1);
}


int strcpy(char *dst, char *src)
{
   int i=0;
   while(*src) {
      *dst++ = *src++;
      i++;
   }
   return i;
}




/* *****************************************
 * Console I/O
 * ******************************************/

int numOpenDevices = 11;
struct {
   char name[128];
} deviceState[32];

#define BOOTDEVICE_NAME "SCSI 1 0 0 1 100 0"

void
DeviceOperation(long op, long channel, long count, long address, long block)
{
   struct AlphaAccess *k1Conf = (struct AlphaAccess *)
      (__MAGIC_ZONE(0, 0, MAGIC_ZONE_EV5_ALIAS));

   long pAddr;

#if 0
   printf("Console::DeviceRead count=0x%x address=0x%x block=0x%x\n",
          count,address,block);
#endif

   if (strcmp(deviceState[channel].name, BOOTDEVICE_NAME )) {
      panic("DeviceRead: only implemented for root disk \n");
   }
   pAddr = KSEG_TO_PHYS(address);
   if (pAddr + count > simosConf.mem_size) {
      panic("DeviceRead: request out of range \n");
   }

   k1Conf->diskCount = count;
   k1Conf->diskPAddr = pAddr;
   k1Conf->diskBlock = block;
   k1Conf->diskOperation = op; /* launch */
}



/* *************************************************************************
 * SimoS Console callbacks
 * **************************************************/

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

char	env_auto_action[MAX_ENVLEN]	= "BOOT";
char	env_boot_dev[MAX_ENVLEN]	= "";
char	env_bootdef_dev[MAX_ENVLEN]	= "";
char	env_booted_dev[MAX_ENVLEN]	= BOOTDEVICE_NAME;
char	env_boot_file[MAX_ENVLEN]	= "";
char	env_booted_file[MAX_ENVLEN]	= "";
char	env_boot_osflags[MAX_ENVLEN]	= "";
char	env_booted_osflags[MAX_ENVLEN]	= "";
char	env_boot_reset[MAX_ENVLEN]	= "";
char	env_dump_dev[MAX_ENVLEN]	= "";
char	env_enable_audit[MAX_ENVLEN]	= "";
char	env_license[MAX_ENVLEN]		= "";
char	env_char_set[MAX_ENVLEN]	= "";
char	env_language[MAX_ENVLEN]	= "";
char	env_tty_dev[MAX_ENVLEN]		= "0";
char	env_scsiid[MAX_ENVLEN]		= "";
char	env_scsifast[MAX_ENVLEN]	= "";
char	env_com1_baud[MAX_ENVLEN]	= "";
char	env_com1_modem[MAX_ENVLEN]	= "";
char	env_com1_flow[MAX_ENVLEN]	= "";
char	env_com1_misc[MAX_ENVLEN]	= "";
char	env_com2_baud[MAX_ENVLEN]	= "";
char	env_com2_modem[MAX_ENVLEN]	= "";
char	env_com2_flow[MAX_ENVLEN]	= "";
char	env_com2_misc[MAX_ENVLEN]	= "";
char	env_password[MAX_ENVLEN]	= "";
char	env_secure[MAX_ENVLEN]		= "";
char	env_logfail[MAX_ENVLEN]		= "";
char	env_srm2dev_id[MAX_ENVLEN]	= "";

#define MAX_ENV_INDEX 100
char *env_ptr[MAX_ENV_INDEX] =
{
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
      for(i = 0; i < a3; i++)
         PutChar(*((char *)a2+i));
      return a3;

   case CONSCB_GETENV:
     if (a1 >= 0 && a1 < MAX_ENV_INDEX && env_ptr[a1] != 0 && *env_ptr[a1]) {
         i = strcpy((char*)a2, env_ptr[a1]);
     } else {
         strcpy((char*)a2, "");
         i = (long)0xc000000000000000;
         if (a1 >= 0 && a1 < MAX_ENV_INDEX)
             printf ("GETENV unsupported option %d (0x%x)\n", a1, a1);
         else
             printf ("GETENV unsupported option %s\n", a1);
     }

     if (i > a3)
         panic("CONSCB_GETENV overwrote buffer\n");
     return i;

   case CONSCB_OPEN:
      bcopy((char*)a1,deviceState[numOpenDevices].name,a2);
      deviceState[numOpenDevices].name[a2] = '\0';
      printf("CONSOLE OPEN : %s --> success \n",
             deviceState[numOpenDevices].name);
      return numOpenDevices++;

   case CONSCB_READ:
      DeviceOperation(a0,a1,a2,a3,a4);
      break;

   case CONSCB_CLOSE:
      break;
   case CONSCB_OPEN_CONSOLE:
      printf("CONSOLE OPEN\n");
      return 0; /* success */
      break; /* not rearched */
   case CONSCB_CLOSE_CONSOLE:
      printf("CONSOLE CLOSE\n");
      return 0; /* success */
      break; /* not reached */

   default:
      panic("cher (%x,%x,%x,%x)\n", a0, a1, a2, a3);
   }

   return 0;
}

long CallBackFixup(int a0, int a1, int a2)
{
   printf("CallbackFixup %x %x \n",a0,a1);

#if 0
  if (first[FIRST(a1)]==0) {
      first[FIRST(a1)] = KPTE(PFN(reservedFixup));
   } else {
      panic("CallBakcfixup\n");
   }
   second[SECOND(a1)] = KPTE(PFN(third_rpb));	/* Region 0 */
   printf("Fixup: FISRT(a1)=0x%x SECOND(a1)=0x%x THIRD(a1)=0x%x\n",
          FIRST(a1),SECOND(a1),THIRD(a1));

#endif
   return 0;
}





void SlaveCmd(int cpu, struct rpb_percpu *my_rpb)
{
/*   extern void palJToSlave[]; */
   extern unsigned int palJToSlave[];


   my_rpb->rpb_state |= STATE_BIP;
   my_rpb->rpb_state &= ~STATE_RC;

   SpinLock(&theLock);
   printf("SlaveCmd: restart %x %x vptb %x my_rpb %x my_rpb_phys %x\n",
          rpb->rpb_restart,
          rpb->rpb_restart_pv,
          rpb->rpb_vptb, my_rpb,
          KSEG_TO_PHYS(my_rpb));
   SpinUnlock(&theLock);

   cServe(KSEG_TO_PHYS((ul)palJToSlave),
          (ul)rpb->rpb_restart,
          CSERVE_K_JTOPAL,
          rpb->rpb_restart_pv,
          rpb->rpb_vptb,
          KSEG_TO_PHYS(my_rpb));
}

void SlaveLoop( int cpu)
{
   int size = ROUNDUP128(sizeof(struct rpb_percpu));
   struct rpb_percpu *my_rpb = (struct rpb_percpu*)
      ((ul)rpb_percpu + size*cpu);


   SpinLock(&theLock);
   if (cpu==0) {
      panic("CPU 0 entering slaveLoop. Reenetering the console. HOSED \n");
   } else {
      printf("Entering slaveloop for cpu %d my_rpb=%x \n",cpu,my_rpb);
   }
   SpinUnlock(&theLock);
   while(1) {
      int i;
      for (i=0; i < 1000000 ; i++) {
         if (my_rpb->rpb_iccb.iccb_rxlen) {
            SpinLock(&theLock);
            printf("Slave CPU %d console command %s",
                   cpu,my_rpb->rpb_iccb.iccb_rxbuf);
            SpinUnlock(&theLock);
            SlaveCmd(cpu,my_rpb);
            panic("SlaveCmd returned \n");
         }
      }
      printf("*");
   }
}

