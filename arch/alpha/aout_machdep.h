/* $Id$ */

#ifndef __AOUT_MACHDEP_H__
#define __AOUT_MACHDEP_H__

///
/// Funky Alpha 64-bit a.out header used for PAL code.
///
struct aout_exechdr {
    uint16_t	magic;
    uint16_t	vstamp;
    uint16_t	bldrev;
    uint16_t	padcell;
    uint64_t	tsize;		// text segment size
    uint64_t	dsize;		// data segment size
    uint64_t	bsize;		// bss segment size
    uint64_t	entry;		// entry point
    uint64_t	text_start;	// text base address
    uint64_t	data_start;	// data base address
    uint64_t	bss_start;	// bss base address
    uint32_t	gprmask;
    uint32_t	fprmask;
    uint64_t	gp_value;
};

#define AOUT_LDPGSZ	8192

#define N_GETMAGIC(ex)	((ex).magic)

#define N_BADMAX

#define N_TXTADDR(ex)	((ex).text_start)
#define N_DATADDR(ex)	((ex).data_start)
#define N_BSSADDR(ex)	((ex).bss_start)

#define N_TXTOFF(ex)	\
        (N_GETMAGIC(ex) == ZMAGIC ? 0 : sizeof(struct aout_exechdr))

#define N_DATOFF(ex)	N_ALIGN(ex, N_TXTOFF(ex) + (ex).tsize)

#endif /* !__AOUT_MACHDEP_H__*/
