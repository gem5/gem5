#ifndef __LIB_H_LOADED
#define __LIB_H_LOADED
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
 *  $Id: lib.h,v 1.1.1.1 1997/10/30 23:27:16 verghese Exp $;
 */

/*
 * $Log: lib.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:16  verghese
 * current 10/29/97
 *
 * Revision 1.1  1995/06/23  00:18:57  berc
 * Initial revision
 *
 * Revision 1.29  1995/04/05  21:46:10  cruz
 * Added EB164 prototype for wr_bcache().
 *
 * Revision 1.28  1995/04/03  18:35:32  cruz
 * Extended the prototype definition of cserv.
 *
 * Revision 1.27  1995/03/05  04:17:23  fdh
 * Corrected prototype for inrom().
 *
 * Revision 1.26  1995/02/28  03:05:53  fdh
 * Moved rom.c prototypes to rom.h
 *
 * Revision 1.25  1995/02/27  19:21:27  fdh
 * Added prototypes for Ctype routines.
 *
 * Revision 1.24  1995/02/23  21:48:31  fdh
 * Added prototypes for set_haxr() and atoi().
 *
 * Revision 1.23  1995/02/22  22:01:10  fdh
 * Changed types for tolower() and toupper().
 *
 * Revision 1.22  1995/02/19  17:46:01  fdh
 * Changed one of the arguments to read_rom().
 *
 * Revision 1.21  1995/02/10  02:23:20  fdh
 * Added prototype for set_romboot().
 *
 * Revision 1.20  1994/11/19  03:30:51  fdh
 * Added support for romlist command.
 *
 * Revision 1.19  1994/11/18  19:05:31  fdh
 * swpipl returns the current ipl.
 *
 * Revision 1.18  1994/11/08  21:39:20  fdh
 * Added declaration for the flash routine.
 *
 * Revision 1.17  1994/11/01  11:30:01  rusling
 * Changed following PCI-PCI bridge support.
 *
 * Revision 1.16  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.15  1994/08/03  19:44:23  fdh
 * Fixups around the linker defined symbols _edata and _end.
 * Protect time_t definition with #ifndef _TIME_T.
 *
 * Revision 1.14  1994/07/22  21:02:46  fdh
 * Added extern void rtcBaseInit(void); for EB64 builds.
 *
 * Revision 1.13  1994/07/21  18:10:06  fdh
 * >> Added EnableBCache(), DisableBCache(), and CleanBCache().
 *
 * Revision 1.12  1994/07/13  14:17:07  fdh
 * Added data structure for holding pointers to SROM interface
 * parameters.
 *
 * Revision 1.11  1994/06/28  20:08:21  fdh
 * Modified filenames and build precedure to fit into a FAT filesystem.
 *
 * Revision 1.10  1994/06/22  15:10:34  rusling
 * Fixed up WNT compile warnings.
 *
 * Revision 1.9  1994/06/21  15:27:47  rusling
 * Removed putFloat() prototype.
 *
 * Revision 1.8  1994/06/21  14:18:05  rusling
 * changed definition of loadHeader() in rom.c
 *
 * Revision 1.7  1994/06/21  10:38:53  rusling
 * Added strncmp() for WNT.
 *
 * Revision 1.5  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.4  1994/06/17  19:34:01  fdh
 * Clean-up...
 *
 * Revision 1.3  1994/06/13  15:54:35  fdh
 * Added definitions of unsigned amounts as defined in system.h
 * Definitions are also placed here to make the lib subdirectory
 * free-standing from the rest of the source tree.
 *
 * Revision 1.2  1994/06/03  20:18:38  fdh
 * Added protypes for all routines in /lib.
 *
 * Revision 1.1  1994/01/19  10:33:21  rusling
 * Initial revision
 *
 * Revision 1.1  1993/06/08  19:56:14  fdh
 * Initial revision
 *
 */

#include <stddef.h>

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*
 * Definitions of unsigned amounts
 */
#define ub unsigned char
#define uw unsigned short int

#ifdef _WIN32
#define ul unsigned __int64
#define sl __int64
#define ui unsigned int
#else
#define ul unsigned long
#define sl long
#define ui unsigned int
#endif

typedef struct {
  ul *abox_ctl;
#ifdef DC21064
  ul *biu_ctl;
#endif /* DC21064 */
#ifdef DC21066
  ui *bcr0;
  ui *bcr1;
  ui *bcr2;
  ui *bcr3;
  ui *bmr0;
  ui *bmr1;
  ui *bmr2;
  ui *bmr3;
#endif /* DC21066 */
  ul *srom_rev;
  ul *proc_id;
  ul *mem_size;
  ul *cycle_cnt;
  ul *signature;
  ul *proc_mask;
  ul *sysctx;
  int valid;
} sysdata_t;

#ifdef _WIN32
extern ul end;
extern ul edata;
#define _END end
#define _EDATA edata
#else
extern ul _end;
extern ul _edata;
#define _END _end
#define _EDATA _edata
#endif

#ifndef _TIME_T
#define _TIME_T
/* typedef int time_t; */
typedef ui time_t;
#endif

/*::::::::::::::
rw.c
::::::::::::::*/
/* Here B = 8 bits, W = 16 bits, L = 32 bits, Q = 64 bits */
extern ub ReadB(ub * adr);
extern uw ReadW(uw * adr);
extern void WriteB(ub * adr , ub data);
extern void WriteW(uw * adr , uw data);

/*::::::::::::::
beep.c
::::::::::::::*/
extern void tone(int period , int time);
extern void dummy(int x);
extern void Beep(int msec , int freq);
extern void msleep(int ms);

/*::::::::::::::
c8514.c
::::::::::::::*/
extern int c8514init(void);
extern void c8514erase(void);
extern void c8514putc(char c);
extern void c8514show(void);
extern void c8514hide(void);
extern void c8514insfontchar(int code , ub * s);
extern void initfont(void);

/*::::::::::::::
date.c
::::::::::::::*/
extern void printDate(void);
extern void setDate(ub * date);
extern ui gettime(void);
extern time_t time(void);
extern void CheckDate(void);

/*::::::::::::::
initdata.c
::::::::::::::*/
extern void doinitdata(void );

/*::::::::::::::
ebxx_io.c
::::::::::::::*/
extern void outportb(ul p , ui d);
extern void outportw(ul p , ui d);
extern void outportt(ul p , ui d);
extern void outportl(ul p , ui d);
extern void outportbxt(ul p , ui d);
extern void outport(ul p , ui d);
extern void outcfgb(ui bus, ui dev, ui reg, ui data);
extern void outcfgw(ui bus, ui dev, ui reg, ui data);
extern void outcfgl(ui bus, ui dev, ui reg, ui data);
extern void outmembxt(ul p , ui d);
extern void outmemwat(ul p , ui d);
extern ui inportb(ul p);
extern ui inportw(ul p);
extern ui inportt(ul p);
extern ui inportl(ul p);
extern ui inportwat(ul p);
extern ui inportbxt(ul p);
extern ui inport(ul p);
extern ui incfgb(ui bus, ui dev, ui reg);
extern ui incfgw(ui bus, ui dev, ui reg);
extern ui incfgl(ui bus, ui dev, ui reg);
extern ui inmembat(ul p);
extern ui inmembxt(ul p);
extern void IOBusInit(void);
extern ul IOPCIClearNODEV(void);
extern void outLed(ui d);
extern void out_ioc_csr(ui p , ul d);
extern ul in_ioc_csr(ui p);
extern void outmemb(ul p , ui d);
extern void outmemw(ul p , ui d);
extern void outmemt(ul p , ui d);
extern void outmeml(ul p , ui d);
extern void outmem(ul p , ui d);
extern ui inmemb(ul p);
extern ui inmemw(ul p);
extern ui inmemt(ul p);
extern ul inmeml(ul p);
extern ui inmemwat(ul p);
extern ui inmem(ul p);
extern void set_haxr(unsigned int addr);
extern void outVti(ul p , ui d);
extern ui inVti(ul p);
extern ub inrom(ul p);
extern ui insctl(void);
extern void outsctl(ui d);
extern ui inIack(void);
#ifdef EB64
extern void rtcBaseInit(void);
#endif

/*::::::::::::::
ebxx_mem.c
::::::::::::::*/
extern void memdetect(void);
extern int check_mem_esr(int silent);
#ifdef EB66
extern void out_mem_csr(ul p , ul d);
extern ul in_mem_csr(ui p);
#elif EB64P
extern void out_mem_csr(ui p , ui d);
extern ui in_mem_csr(ui p);
#endif

extern void EnableBCache(void);
extern void DisableBCache(void);

#ifdef EB164
void wr_bcache (ui argc, ul arg1, ul arg2, ui select);
#endif

/*::::::::::::::
ffcsubs.c
::::::::::::::*/
extern int strlen(char * s);
extern void bzero(char * s , int count);
extern void bcopy(char * from , char * to , int count);
extern int tolower(int c);
extern int toupper(int c);
extern int IsAlpha(char c);
extern int IsDigit(char c);

/*::::::::::::::
fftty.c
::::::::::::::*/
extern void UnGetChar(char c);
extern char MonGetChar(void);
extern void FlushLine(void);
extern ul ReadHex(void);
extern void ReadString(char * s);
extern char RawMonGetChar(void);
extern int kbdcontinue(void );

/*::::::::::::::
floppy.c
::::::::::::::*/
extern int floppyRead(int loadtype , char * file2load);

/*::::::::::::::
gpchar.c
::::::::::::::*/
extern void PutChar(char c);
extern char GetChar(void);
extern int CharAv(void);
extern void WaitUs(int usecs);

/*::::::::::::::
ident.c
::::::::::::::*/
extern int ident(ul first , ul last);

/*::::::::::::::
int.c
::::::::::::::*/
extern void intr_enable(int int_level);

/*::::::::::::::
kbd.c
::::::::::::::*/
extern void kbd_error(char * error_msg , int s1);
extern int kbd_init(void);
extern void kbd_reset_state(void);
extern int kbd_charav(void);
extern int kbd_getc(void);

/*::::::::::::::
leds.c
::::::::::::::*/
extern void sethdled(int v);
extern int isturbo(void);
extern int kbd_locked(void);

/*::::::::::::::
memtest.c
::::::::::::::*/
extern ul do_memtest(char * llim , char * hlim , int inc);
extern void memtest1(char * llim , char * hlim , int inc , int pattern);
extern void memtest2(char * llim , char * hlim , int inc , int seed);
extern void memtest3(char * llim , char * hlim , int inc);
extern void memtest(char * min , char * max , int inc);

/*::::::::::::::
p8514.c
::::::::::::::*/
extern void pwgaFillSolid(ui fg , ui alu , ui planemask , int xDst , int yDst , int wDst , int hDst);
extern void pwgaDrawColorImage(int xDst , int yDst , int wDst , int hDst , ub * pSrc , int widthSrc , ui alu , ui planemask);
extern void pwgaBlit(int xSrc , int ySrc , int wSrc , int hSrc , int xDst , int yDst , ui alu , ui planemask);
extern int DisplayOpen(int mode1024);
extern void DisplayClose(void);
extern void byteoutport(int p , int d);
extern void short_delay(void);
extern void InitLUT(void);
extern int pwgaExists(void);
extern ui pwgaHWInit(int mode1024);
extern void outwords(short * wSrc , int wcount);
extern void outwblock(ub * pSrc , int w , int h , int widthSrc);

/*::::::::::::::
pr.c
::::::::::::::*/
extern void PQ(ul x);
extern void PL(ui x);
extern void PW(uw x);
extern void PB(ub x);
extern void PutSpace(void);
extern void PutCR(void);

/*::::::::::::::
printf.c
::::::::::::::*/
extern void PutString(char * s);
extern void printf(char * f , ...);

/*::::::::::::::
search.c
::::::::::::::*/
extern int search(ul first , ul last , int size , char * valstr , int inverse);
extern void ParseVal(char * s , ul * val , ul * mask , int size);

/*::::::::::::::
sniff.c
::::::::::::::*/
extern int find_first(int map);
extern int sniff_eisa(int id , int mask , int num_slots);

/*::::::::::::::
uart.c
::::::::::::::*/
extern int uart_charav(int port);
extern char uart_getchar(int port);
extern void uart_putchar(int port , char c);
extern void putcLpt(int c);
extern void uart_init_line(int line , int baud);
extern int uart_init(void);

/*::::::::::::::
vga.c
::::::::::::::*/
extern void vgaerase(void);
extern void vgalcgen(void);
extern void vgasetloc(void);
extern void vgaputc(register int c);
extern void vgastl(ul a , int d);
extern int vgaldl(ul a);
extern ub readreg(ui sel , ui off);
extern void writereg(ui sel , ui off , ub dat);
extern void dumpvga(void);
extern void vgainit(void);

/*::::::::::::::
libc.c
::::::::::::::*/
extern int memcmp(const void * pcs , const void * pct , size_t n);
extern void * memset(void * ps , char c , size_t n);
extern void * memmove(void * ps , const void * pct , size_t n);
extern void * memcpy(void * ps , const void * pct , size_t n);
extern int atoi(const char *nptr);
extern int isalnum(int c);
extern int isalpha(int c);
extern int isascii(int c);
extern int iscntrl(int c);
extern int isdigit(int c);
extern int isgraph(int c);
extern int islower(int c);
extern int isprint(int c);
extern int ispunct(int c);
extern int isspace(int c);
extern int isupper(int c);
extern int isxdigit(int c);

/*::::::::::::::
flash.c
::::::::::::::*/
extern int flash_main(ui src , ui segnum, ui segcnt);
/*::::::::::::::
asmstuff.s
::::::::::::::*/
extern void mb(void);
extern ul GetSP(void);
extern ul cServe(ul, ul, ul, ...);
extern void wrfen(ui);
extern void swppal(ul, ul, ul, ul);
extern void halt(void);
extern void wait_cycles(ui cycles);
extern int swpipl(ui);
extern void CleanBCache(ul);
/* Here B = 8 bits, W = 16 bits, L = 32 bits, Q = 64 bits */
#ifdef _WIN32
extern ui ReadL(ul);
extern void WriteL(ul, ui);
extern ul ReadQ(ul);
extern void WriteQ(ul, ul);
#else
#define WriteL(address,value)	(*(ui *)(address)=(ui)(value))
#define ReadL(address)		(*(ui *)(address))
#define WriteQ(address,value)	(*(ul *)(address)=(ul)(value))
#define	ReadQ(address)		(*(ul *)(address))
#endif

/*::::::::::::::::
host specific library definitions
::::::::::::::::*/
#ifdef _WIN32
extern int strcmp(char *cs, char *ct);
extern char *strcpy(char *s, char *ct);
extern int strncmp(char *cs, char *ct, int n);
extern char *strncpy(char *s, char *ct, int n);
extern int rand(void);
#endif

#endif /* __LIB_H_LOADED */
