#ifndef __SSB_H_LOADED
#define __SSB_H_LOADED
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
 *  $Id: ssb.h,v 1.1.1.1 1997/10/30 23:27:17 verghese Exp $;
 */

/*
 * $Log: ssb.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:17  verghese
 * current 10/29/97
 *
 * Revision 1.3  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.2  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.1  1993/06/08  19:56:16  fdh
 * Initial revision
 *
 */



/* ssbinit 						*/
/* Set up SSB definitions				*/
/* Last modified on 1/3/91 by L. Stewart		*/

#define SSB_stcreset1  0x0800	/*  CR_DIR */
#define SSB_stcreset2  0x3820	/* CR_CABEN | CR_AUTO | CR_DIR */
                                        /*  |  CR_FIFOMR */

#define SSB_TagCable	 0
#define SSB_TagSlot	 3
#define SSB_TagTVA	 2
#define SSB_TagRFA	 1
#define SSB_TagORF	 5
#define SSB_TagStartTV	 4
#define SSB_TagTVStat	 6
#define SSB_TagLocalStat 7

#define SSB_AdrLocalStat 0x4000
#define SSB_AdrSRA 0x8000
#define SSB_AdrSRB 0x8001

#define SSB_LCRenable	0x1
#define SSB_LCRclrstat	0x2
#define SSB_LCRfifomr	0x4
#define SSB_LCRreset	0x8
#define SSB_LCRled	0x10

#define SSB_LSTfifoff	 0x100
#define SSB_LSTfifohf	 0x200
#define SSB_LSTfifoef	 0x400
#define SSB_LSTinst	 0x800
#define SSB_LSTid	 0x1000
#define SSB_LSTotherinst 0x2000
#define SSB_LSTcnfg0	 0x4000
#define SSB_LSTcnfg1     0x8000
#define SSB_LSTnode	 0xf000

#define SSB_CRrParityS	 0x01
#define SSB_CRdataWP	 0x02
#define SSB_CRadrWP	 0x04
#define SSB_CRerrEnable	 0x08
#define SSB_CRreset	 0x10
#define SSB_CRdoConfig	 0x20
#define SSB_CRnodeClock	 0x40

#define SSB_SRAshared	 0x00000080
#define SSB_SRAdirty	 0x00100000
#define SSB_SRAnType	 0x80000000

#define SSB_PSRinst	 0x001
#define SSB_PSRmerror	 0x002
#define SSB_PSRreset	 0x004
#define SSB_PSRerror	 0x008
#define SSB_PSRserror	 0x010
#define SSB_PSRisr	 0x7e0

struct SSB_lsrtype {
        unsigned	sequence : 8;
        unsigned	slot : 8;
        unsigned	mix : 4;
        unsigned	inst : 1;
        unsigned	ef : 1;
        unsigned	hf : 1;
        unsigned	ff : 1;
        unsigned	crpad : 3;
        unsigned	crled : 1;
        unsigned	crreset : 1;
        unsigned	crfifomr : 1;
        unsigned	crclrstat : 1;
        unsigned	crenable : 1;
};

struct SSB_pstattype {
        unsigned	pad : 21;
        unsigned	isr : 6;
        unsigned	serror : 1;
        unsigned	error : 1;
        unsigned	reset : 1;
        unsigned	merror : 1;
        unsigned	inst : 1;
};

struct SSB_sratype {
        unsigned	nodeType : 1;
        unsigned	p0stat: 11;
        unsigned	dirty : 1;
        unsigned	p1stat : 11;
        unsigned	shared : 1;
        unsigned	cr : 7;
};

struct SSB_srbtype {
        unsigned	p1count : 8;
        unsigned	p0count : 8;
        unsigned	arbcount : 16;
};

extern int ssb_sequence;
extern int ssb_localcr;
extern int ssb_maincr;
extern int ssb_port;
extern int ssb_node;
extern int ssb_config1;
extern int ssb_config0;
extern int ssb_lastsra;
extern int ssb_lastsrb;
extern int ssb_lastlsr;
extern int ssb_lastieecc;
extern struct SSB_lsrtype ssb_lsr;
extern int ssb_mix;
extern struct SSB_sratype ssb_sra;
extern struct SSB_srbtype ssb_srb;
extern int ssb_resid;
extern struct SSB_pstattype ssb_portstatus;


/* ssb routines */
#define SSB_CR  SSB_stcreset2
#define SSBCH   1020

#define SSB_CABLE(x) (wfifo(x, SSB_TagCable))
#define SSB_LDSLOT(x) (wfifo(x, SSB_TagSlot))
#define SSB_LDTVA(x, w) (wfifo((x) | ((w) ? 0x80000000 : 0), SSB_TagTVA))
#define SSB_LDRFA(a, c) (wfifo(((255 - c) << 16) + a, SSB_TagRFA))
#define SSB_LDORF(x) (wfifo(x, SSB_TagORF))
#define SSB_STARTTV(a, c, w) (wfifo(( (w << 15) | ((255 - c) << 16) | a), SSB_TagStartTV ))
#define SSB_TVSTAT(x) (wfifo(x, SSB_TagTVStat))
#define SSB_LOCALSTAT(x) (wfifo(x, SSB_TagLocalStat))

#define SSB_CFF() ( STC_SETCR(tvp->crmask & ~STC_CRFIFOMR), \
    STC_SETCR(tvp->crmask | STC_CRFIFOMR) )

#endif /* __SSB_H_LOADED */
