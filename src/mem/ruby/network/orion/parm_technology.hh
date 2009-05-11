/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#ifndef _PARM_TECHNOLOGY_H
#define _PARM_TECHNOLOGY_H

/******* LEAKAGE current values for different tech points are in power_static.c *******/
/******* for LINKS: metal cap is assumed to be CC2metal and repeater size is assumed to be n_size=Lambda*10, p_size=n_size*2 *******/

#define SWITCHING_FACTOR 0.5 /*used for offline calculations*/

/********************* Technology point **********/
#define PARM_TECH_POINT 10 /* 100nm */
#define PARM_Freq 3e9
/*************************************************/


/*********** Transistor parasitics ******************/
#define PARM_Cndiffside (2.750000e-16)
#define PARM_Cpdiffovlp (1.380000e-16)
#define PARM_Cgatepass (1.450000e-15)
#define PARM_Cpdiffarea (3.430000e-16)
#define PARM_Cpdiffside (2.750000e-16)
#define PARM_Cpoxideovlp (3.380000e-16)
#define PARM_Cndiffovlp (1.380000e-16)
#define PARM_Cndiffarea (1.370000e-16)
#define PARM_Cgate (1.950000e-15)
#define PARM_Cnoxideovlp (2.630000e-16)
/*************************************************/



/************* Scaling factors for different technology points ******************/
#if ( PARM_TECH_POINT == 10 )   /* 100nm */
#define CSCALE          (84.2172)       /* wire capacitance scaling factor */
                                                    /* linear: 51.7172, predicted: 84.2172 */
#define RSCALE          (80.0000)       /* wire resistance scaling factor */
#define LSCALE          0.1250          /* length (feature) scaling factor */
#define ASCALE          (LSCALE*LSCALE) /* area scaling factor */
#define VSCALE          0.38            /* voltage scaling factor */
#define VTSCALE         0.49            /* threshold voltage scaling factor */
#define SSCALE          0.80            /* sense voltage scaling factor */
/* FIXME: borrowed from 0.11u technology */
#define MCSCALE         5.2277          /* metal coupling capacitance scaling factor */
#define MCSCALE2        3               /* metal coupling capacitance scaling factor (2X) */
#define MCSCALE3        1.5             /* metal coupling capacitance scaling factor (3X) */
#define GEN_POWER_SCALE (1/PARM_GEN_POWER_FACTOR)
/* copied from TECH_POINT 10 except LSCALE */
#elif ( PARM_TECH_POINT == 11 ) /* 110nm */
#define CSCALE          (84.2172)       /* wire capacitance scaling factor */
#define RSCALE          (80.0000)       /* wire resistance scaling factor */
#define LSCALE          0.1375          /* length (feature) scaling factor */
#define ASCALE          (LSCALE*LSCALE) /* area scaling factor */
#define VSCALE          0.38            /* voltage scaling factor */
#define VTSCALE         0.49            /* threshold voltage scaling factor */
#define SSCALE          0.80            /* sense voltage scaling factor */
#define MCSCALE         5.2277          /* metal coupling capacitance scaling factor */
#define MCSCALE2        3               /* metal coupling capacitance scaling factor (2X) */
#define MCSCALE3        1.5             /* metal coupling capacitance scaling factor (3X) */
#define GEN_POWER_SCALE (1/PARM_GEN_POWER_FACTOR)
#elif ( PARM_TECH_POINT == 18 ) /* 180nm */
#define CSCALE          (19.7172)       /* wire capacitance scaling factor */
#define RSCALE          (20.0000)       /* wire resistance scaling factor */
#define LSCALE          0.2250          /* length (feature) scaling factor */
#define ASCALE          (LSCALE*LSCALE) /* area scaling factor */
#define VSCALE          0.4             /* voltage scaling factor */
#define VTSCALE         0.5046          /* threshold voltage scaling factor */
#define SSCALE          0.85            /* sense voltage scaling factor */
#define MCSCALE         4.1250          /* metal coupling capacitance scaling factor */
#define MCSCALE2        2.4444          /* metal coupling capacitance scaling factor (2X) */
#define MCSCALE3        1.2             /* metal coupling capacitance scaling factor (3X) */
#define GEN_POWER_SCALE 1
#elif ( PARM_TECH_POINT == 25 ) /* 250nm */
#define CSCALE          (10.2197)       /* wire capacitance scaling factor */
#define RSCALE          (10.2571)       /* wire resistance scaling factor */
#define LSCALE          0.3571          /* length (feature) scaling factor */
#define ASCALE          (LSCALE*LSCALE) /* area scaling factor */
#define VSCALE          0.45            /* voltage scaling factor */
#define VTSCALE         0.5596          /* threshold voltage scaling factor */
#define SSCALE          0.90            /* sense voltage scaling factor */
#define MCSCALE         1.0             /* metal coupling capacitance scaling factor */
#define MCSCALE2        1.0             /* metal coupling capacitance scaling factor (2X) */
#define MCSCALE3        1.0             /* metal coupling capacitance scaling factor (3X) */
#define GEN_POWER_SCALE PARM_GEN_POWER_FACTOR
#elif ( PARM_TECH_POINT == 35 ) /* 350nm */
#define CSCALE          (5.2197)        /* wire capacitance scaling factor */
#define RSCALE          (5.2571)        /* wire resistance scaling factor */
#define LSCALE          0.4375          /* length (feature) scaling factor */
#define ASCALE          (LSCALE*LSCALE) /* area scaling factor */
#define VSCALE          0.5             /* voltage scaling factor */
#define VTSCALE         0.6147          /* threshold voltage scaling factor */
#define SSCALE          0.95            /* sense voltage scaling factor */
#define MCSCALE         1.0             /* metal coupling capacitance scaling factor */
#define MCSCALE2        1.0             /* metal coupling capacitance scaling factor (2X) */
#define MCSCALE3        1.0             /* metal coupling capacitance scaling factor (3X) */
#define GEN_POWER_SCALE (PARM_GEN_POWER_FACTOR*PARM_GEN_POWER_FACTOR)
#elif ( PARM_TECH_POINT == 40 ) /* 400nm */
#define CSCALE          1.0             /* wire capacitance scaling factor */
#define RSCALE          1.0             /* wire resistance scaling factor */
#define LSCALE          0.5             /* length (feature) scaling factor */
#define ASCALE          (LSCALE*LSCALE) /* area scaling factor */
#define VSCALE          1.0             /* voltage scaling factor */
#define VTSCALE         1.0             /* threshold voltage scaling factor */
#define SSCALE          1.0             /* sense voltage scaling factor */
#define MCSCALE         1.0             /* metal coupling capacitance scaling factor */
#define MCSCALE2        1.0             /* metal coupling capacitance scaling factor (2X) */
#define MCSCALE3        1.0             /* metal coupling capacitance scaling factor (3X) */
#define GEN_POWER_SCALE (PARM_GEN_POWER_FACTOR*PARM_GEN_POWER_FACTOR*PARM_GEN_POWER_FACTOR)
#else /* ( PARM_TECH_POINT == 80 ) */ /* 800nm */
#define CSCALE          1.0             /* wire capacitance scaling factor */
#define RSCALE          1.0             /* wire resistance scaling factor */
#define LSCALE          1.0             /* length (feature) scaling factor */
#define ASCALE          (LSCALE*LSCALE) /* area scaling factor */
#define VSCALE          1.0             /* voltage scaling factor */
#define VTSCALE         1.0             /* threshold voltage scaling factor */
#define SSCALE          1.0             /* sense voltage scaling factor */
#define MCSCALE         1.0             /* metal coupling capacitance scaling factor */
#define MCSCALE2        1.0             /* metal coupling capacitance scaling factor (2X) */
#define MCSCALE3        1.0             /* metal coupling capacitance scaling factor (3X) */
#define GEN_POWER_SCALE (PARM_GEN_POWER_FACTOR*PARM_GEN_POWER_FACTOR*PARM_GEN_POWER_FACTOR*PARM_GEN_POWER_FACTOR)
#endif

#define MSCALE  (LSCALE * .624 / .2250)
#define PARM_NORMALIZE_SCALE (6.488730e-10)

/* scaling factors from 0.1u to 0.07u, 0.05u and 0.035u */
/* if technology point is 0.07u, o.05u or 0.035u set PARM_TECH_POINT to 10 and
 * set TEST_LENGTH to 70, 50 or 35 respectively(TEST_LENGTH =100 for all other technologies) */
#define TEST_LENGTH (70)

#if (TEST_LENGTH == 70)
#define SCALE_T (0.5489156157)
#define SCALE_M (0.6566502462)
#define SCALE_S (1.4088071075)
#elif (TEST_LENGTH == 50)
#define SCALE_T (0.3251012552)
#define SCALE_M (0.4426460239)
#define SCALE_S (2.8667111607)
#elif (TEST_LENGTH == 35)
#define SCALE_T (0.2016627474)
#define SCALE_M (0.2489788586)
#define SCALE_S (8.7726826878)
#else
#define SCALE_T (1)
#define SCALE_M (1)
#define SCALE_S (1)
#endif  /* TEST_LENGTH */
/*************************************************/



/********************** technology point related ***************************/
/* um */
#define Leff          (0.8 * LSCALE)
/* length unit in um */
#define Lamda         (Leff * 0.5)

#ifndef Vdd
#define Vdd             (5 * VSCALE)
#endif  /* Vdd */

#define Period          ((double)1/(double)PARM_Freq)
/*************************************************/


/**** for SRAM - decoder, mem cell, wordline, bitline, output driver ****/
/*
 * CMOS 0.8um model parameters
 *   - from Appendix II of Cacti tech report
 */
/* corresponds to 8um of m3 @ 225ff/um */
#define Cwordmetal    (1.8e-15 * (CSCALE * ASCALE) * SCALE_M)

/* corresponds to 16um of m2 @ 275ff/um */
#define Cbitmetal     (4.4e-15 * (CSCALE * ASCALE) * SCALE_M)

/* corresponds to 1um of m2 @ 275ff/um */
#define Cmetal        (Cbitmetal/16)
#define CM2metal      (Cbitmetal/16)
#define CM3metal      (Cbitmetal/16)

/* minimal spacing metal cap per unit length */
#define CCmetal       (Cmetal * MCSCALE)
#define CCM2metal     (CM2metal * MCSCALE)
#define CCM3metal     (CM3metal * MCSCALE)
/* 2x minimal spacing metal cap per unit length */
#define CC2metal      (Cmetal * MCSCALE2)
#define CC2M2metal    (CM2metal * MCSCALE2)
#define CC2M3metal    (CM3metal * MCSCALE2)
/* 3x minimal spacing metal cap per unit length */
#define CC3metal      (Cmetal * MCSCALE3)
#define CC3M2metal    (CM2metal * MCSCALE3)
#define CC3M3metal    (CM3metal * MCSCALE3)

/* fF/um */
#define Cpolywire       (0.25e-15 * CSCALE * LSCALE)

/* ohms*um of channel width */
#define Rnchannelstatic (25800 * LSCALE)

/* ohms*um of channel width */
#define Rpchannelstatic (61200 * LSCALE)

#define Rnchannelon     (9723 * LSCALE)

#define Rpchannelon     (22400 * LSCALE)

/* corresponds to 16um of m2 @ 48mO/sq */
#define Rbitmetal       (0.320 * (RSCALE * ASCALE))

/* corresponds to  8um of m3 @ 24mO/sq */
#define Rwordmetal      (0.080 * (RSCALE * ASCALE))

#define Powerfactor     (PARM_Freq)*Vdd*Vdd
#define EnergyFactor    (Vdd*Vdd)

#define SensePowerfactor3 (PARM_Freq)*(Vbitsense)*(Vbitsense) //
#define SensePowerfactor2 (PARM_Freq)*(Vbitpre-Vbitsense)*(Vbitpre-Vbitsense) //
#define SensePowerfactor  (PARM_Freq)*Vdd*(Vdd/2) //
#define SenseEnergyFactor (Vdd*Vdd/2)

/* transistor widths in um (as described in tech report, appendix 1) */
#define Wdecdrivep      (57.0 * LSCALE)
#define Wdecdriven      (40.0 * LSCALE)
#define Wdec3to8n       (14.4 * LSCALE)
#define Wdec3to8p       (14.4 * LSCALE)
#define WdecNORn        (5.4 * LSCALE)
#define WdecNORp        (30.5 * LSCALE)
#define Wdecinvn        (5.0 * LSCALE)
#define Wdecinvp        (10.0  * LSCALE)

#define Wworddrivemax   (100.0 * LSCALE)
#define Wmemcella       (2.4 * LSCALE) /* AMIT memory cell inverter PMOS transistor width */
#define Wmemcellr       (4.0 * LSCALE) /* AMIT memory cell read access transistor width */
#define Wmemcellw       (2.1 * LSCALE) /* AMIT memory cell write access transistor width */
#define Wmemcellbscale  2               /* AMIT (mem cell inverter NMOS trans width = Wmemcella * Wmemcellbscale) means 2x bigger than Wmemcella */
#define Wbitpreequ      (10.0 * LSCALE) //

#define Wbitmuxn        (10.0 * LSCALE)
#define WsenseQ1to4     (4.0 * LSCALE)
#define Wcompinvp1      (10.0 * LSCALE)
#define Wcompinvn1      (6.0 * LSCALE)
#define Wcompinvp2      (20.0 * LSCALE)
#define Wcompinvn2      (12.0 * LSCALE)
#define Wcompinvp3      (40.0 * LSCALE)
#define Wcompinvn3      (24.0 * LSCALE)
#define Wevalinvp       (20.0 * LSCALE)
#define Wevalinvn       (80.0 * LSCALE)

#define Wcompn          (20.0 * LSCALE)
#define Wcompp          (30.0 * LSCALE)
#define Wcomppreequ     (40.0 * LSCALE)
#define Wmuxdrv12n      (30.0 * LSCALE)
#define Wmuxdrv12p      (50.0 * LSCALE)
#define WmuxdrvNANDn    (20.0 * LSCALE)
#define WmuxdrvNANDp    (80.0 * LSCALE)
#define WmuxdrvNORn     (60.0 * LSCALE)
#define WmuxdrvNORp     (80.0 * LSCALE)
#define Wmuxdrv3n       (200.0 * LSCALE)
#define Wmuxdrv3p       (480.0 * LSCALE)
#define Woutdrvseln     (12.0 * LSCALE)
#define Woutdrvselp     (20.0 * LSCALE)
#define Woutdrvnandn    (24.0 * LSCALE)
#define Woutdrvnandp    (10.0 * LSCALE)
#define Woutdrvnorn     (6.0 * LSCALE)
#define Woutdrvnorp     (40.0 * LSCALE)
#define Woutdrivern     (48.0 * LSCALE)
#define Woutdriverp     (80.0 * LSCALE)
#define Wbusdrvn        (48.0 * LSCALE)
#define Wbusdrvp        (80.0 * LSCALE)

#define Wcompcellpd2    (2.4 * LSCALE) //
#define Wcompdrivern    (400.0 * LSCALE)
#define Wcompdriverp    (800.0 * LSCALE)
#define Wcomparen2      (40.0 * LSCALE)
#define Wcomparen1      (20.0 * LSCALE)
#define Wmatchpchg      (10.0 * LSCALE)
#define Wmatchinvn      (10.0 * LSCALE)
#define Wmatchinvp      (20.0 * LSCALE)
#define Wmatchnandn     (20.0 * LSCALE)
#define Wmatchnandp     (10.0 * LSCALE)
#define Wmatchnorn      (20.0 * LSCALE)
#define Wmatchnorp      (10.0 * LSCALE)

#define WSelORn         (10.0 * LSCALE) //
#define WSelORprequ     (40.0 * LSCALE) //
#define WSelPn          (10.0 * LSCALE) //
#define WSelPp          (15.0 * LSCALE) //
#define WSelEnn         (5.0 * LSCALE) //
#define WSelEnp         (10.0 * LSCALE) //

#define Wsenseextdrv1p  (40.0*LSCALE) //
#define Wsenseextdrv1n  (24.0*LSCALE) //
#define Wsenseextdrv2p  (200.0*LSCALE) //
#define Wsenseextdrv2n  (120.0*LSCALE) //

/* bit width of RAM cell in um */
#define BitWidth        (16.0 * LSCALE)

/* bit height of RAM cell in um */
#define BitHeight       (16.0 * LSCALE)

#define Cout            (0.5e-12 * LSCALE)

/* Sizing of cells and spacings */
#define RatCellHeight    (40.0 * LSCALE) //
#define RatCellWidth     (70.0 * LSCALE) //
#define RatShiftRegWidth (120.0 * LSCALE) //
#define RatNumShift      4 //
#define BitlineSpacing   (6.0 * LSCALE)
#define WordlineSpacing  (6.0 * LSCALE)

#define RegCellHeight    (16.0 * LSCALE) /* AMIT memory cell height */
#define RegCellWidth     (8.0  * LSCALE) /* AMIT memory cell width */

#define CamCellHeight    (40.0 * LSCALE)
#define CamCellWidth     (25.0 * LSCALE)
#define MatchlineSpacing (6.0 * LSCALE)
#define TaglineSpacing   (6.0 * LSCALE)

#define CrsbarCellHeight (6.0 * LSCALE)
#define CrsbarCellWidth  (6.0 * LSCALE)

/* Link length in um */
#define PARM_link_length 1000


/*************************************************/

/******* LEAKAGE current values for different tech points are in power_static.c *******/
/******* for LINKS: metal cap is assumed to be CC2metal and repeater size is assumed to be n_size=Lambda*10, p_size=n_size*2 *******/

/* -------------------------Miscellaneous---------------------------- */

#define SIM_NO_MODEL    0

#define MAX_ENERGY      1
#define AVG_ENERGY      0

#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif
#ifndef MIN
#define MIN(a,b) (((a)>(b))?(b):(a))
#endif

#define NEXT_DEPTH(d)   ((d) > 0 ? (d) - 1 : (d))

#define BIGNUM  1e30
#define BIGONE  ((unsigned long int)1)
#define BIGNONE ((unsigned long int)-1)
#define HAMM_MASK(w)    ((unsigned)w < (sizeof(unsigned long int) << 3) ? (BIGONE << w) - 1 : BIGNONE)

/* Used to communicate with the horowitz model */
#define RISE 1
#define FALL 0
#define NCH  1
#define PCH  0
/* --------------------------------------------------- */



/* ---------------------- not used ------------------------------- */

#define krise           (0.4e-9 * LSCALE) //
#define tsensedata      (5.8e-10 * LSCALE) //
#define tsensetag       (2.6e-10 * LSCALE) //
#define tfalldata       (7e-10 * LSCALE) //
#define tfalltag        (7e-10 * LSCALE) //
#define Vbitpre         (3.3 * SSCALE) //
#define Vt              (1.09 * VTSCALE)
#define Vbitsense       (0.10 * SSCALE) //


#define PARM_AF (5.000000e-01) //
#define PARM_MAXN (8) //
#define PARM_MAXSUBARRAYS (8) //
#define PARM_MAXSPD (8) //
#define PARM_VTHSENSEEXTDRV (4.370000e-01) //
#define PARM_VTHOUTDRNOR (4.310000e-01) //
#define PARM_res_fpalu (4) //
#define PARM_VTHCOMPINV (4.370000e-01) //
#define PARM_MD_NUM_IREGS (32) //
#define PARM_die_length (1.800000e-02) //
#define PARM_BITOUT (64) //

#define PARM_ruu_decode_width (4) //
#define PARM_ruu_issue_width (4) // AMIT used only for result bus
#define PARM_amp_Idsat (5.000000e-04) //used in amp_energy
#define PARM_AF_TYPE (1) //
#define PARM_VSINV (4.560000e-01) //used in driver size calculations

#define PARM_GEN_POWER_FACTOR (1.310000e+00) //
#define PARM_res_memport (2) //
#define PARM_VTHNAND60x90 (5.610000e-01) //

#define PARM_opcode_length (8) //
#define PARM_MD_NUM_FREGS (32) //
#define PARM_FUDGEFACTOR (1.000000e+00) //
#define PARM_ruu_commit_width (4) //

#define PARM_VTHOUTDRIVE (4.250000e-01) //

#define PARM_VTHMUXDRV1 (4.370000e-01) //
#define PARM_inst_length (32) //
#define PARM_VTHMUXDRV2 (4.860000e-01) //

#define PARM_ras_size (8) //
#define PARM_VTHMUXDRV3 (4.370000e-01) //
#define PARM_ADDRESS_BITS (64) //
#define PARM_RUU_size (16) //

#define PARM_VTHNOR12x4x1 (5.030000e-01) //
#define PARM_VTHNOR12x4x2 (4.520000e-01) //
#define PARM_VTHOUTDRINV (4.370000e-01) //
#define PARM_VTHNOR12x4x3 (4.170000e-01) //
#define PARM_VTHEVALINV (2.670000e-01) //
#define PARM_crossover_scaling (1.200000e+00) //
#define PARM_VTHNOR12x4x4 (3.900000e-01) //
#define PARM_turnoff_factor (1.000000e-01) //
#define PARM_res_ialu (4) //

#define PARM_VTHOUTDRNAND (4.410000e-01) //
#define PARM_VTHINV100x60 (4.380000e-01) //
#define PARM_LSQ_size (8) //

/* ALU POWER NUMBERS for .18um 733Mhz */
/* normalize .18um cap to other gen's cap, then xPowerfactor */
#define POWER_SCALE    (GEN_POWER_SCALE * PARM_NORMALIZE_SCALE * Powerfactor) //
#define I_ADD          ((.37 - .091)*POWER_SCALE) //
#define I_ADD32        (((.37 - .091)/2)*POWER_SCALE) //
#define I_MULT16       ((.31-.095)*POWER_SCALE) //
#define I_SHIFT        ((.21-.089)*POWER_SCALE) //
#define I_LOGIC        ((.04-.015)*POWER_SCALE) //
#define F_ADD          ((1.307-.452)*POWER_SCALE) //
#define F_MULT         ((1.307-.452)*POWER_SCALE) //

#define I_ADD_CLOCK    (.091*POWER_SCALE) //
#define I_MULT_CLOCK   (.095*POWER_SCALE) //
#define I_SHIFT_CLOCK  (.089*POWER_SCALE) //
#define I_LOGIC_CLOCK  (.015*POWER_SCALE) //
#define F_ADD_CLOCK    (.452*POWER_SCALE) //
#define F_MULT_CLOCK   (.452*POWER_SCALE) //


/* ----------------------------------------------------- */


#endif
