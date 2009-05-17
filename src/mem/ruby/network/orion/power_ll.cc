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

/*------------------------------------------------------------
 *  Copyright 1994 Digital Equipment Corporation and Steve Wilton
 *                         All Rights Reserved
 *
 * Permission to use, copy, and modify this software and its documentation is
 * hereby granted only under the following terms and conditions.  Both the
 * above copyright notice and this permission notice must appear in all copies
 * of the software, derivative works or modified versions, and any portions
 * thereof, and both notices must appear in supporting documentation.
 *
 * Users of this software agree to the terms and conditions set forth herein,
 * and hereby grant back to Digital a non-exclusive, unrestricted, royalty-
 * free right and license under any changes, enhancements or extensions
 * made to the core functions of the software, including but not limited to
 * those affording compatibility with other hardware or software
 * environments, but excluding applications which incorporate this software.
 * Users further agree to use their best efforts to return to Digital any
 * such changes, enhancements or extensions that they make and inform Digital
 * of noteworthy uses of this software.  Correspondence should be provided
 * to Digital at:
 *
 *                       Director of Licensing
 *                       Western Research Laboratory
 *                       Digital Equipment Corporation
 *                       100 Hamilton Avenue
 *                       Palo Alto, California  94301
 *
 * This software may be distributed (but not offered for sale or transferred
 * for compensation) to third parties, provided such third parties agree to
 * abide by the terms and conditions of this notice.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND DIGITAL EQUIPMENT CORP. DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS.   IN NO EVENT SHALL DIGITAL EQUIPMENT
 * CORPORATION BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
 * SOFTWARE.
 *------------------------------------------------------------*/

#include <cassert>
#include <cmath>

#include "mem/ruby/network/orion/parm_technology.hh"
#include "mem/ruby/network/orion/SIM_port.hh"
#include "mem/ruby/network/orion/power_static.hh"
#include "mem/ruby/network/orion/power_ll.hh"

/*----------------------------------------------------------------------*/

double SIM_power_gatecap(double width,double wirelength) /* returns gate capacitance in Farads */
//double width;         /* gate width in um (length is Leff) */
//double wirelength;    /* poly wire length going to gate in lambda */
{

  double overlapCap;
  double gateCap;
  double l = 0.1525;

#if defined(Pdelta_w)
  overlapCap = (width - 2*Pdelta_w) * PCov;
  gateCap  = ((width - 2*Pdelta_w) * (l * LSCALE - 2*Pdelta_l) *
                PCg) + 2.0 * overlapCap;

  return gateCap;
#endif
  return(width*Leff*PARM_Cgate+wirelength*Cpolywire*Leff * SCALE_T);
  /* return(width*Leff*PARM_Cgate); */
  /* return(width*CgateLeff+wirelength*Cpolywire*Leff);*/
}


double SIM_power_gatecappass(double width,double wirelength) /* returns gate capacitance in Farads */
//double width;           /* gate width in um (length is Leff) */
//double wirelength;      /* poly wire length going to gate in lambda */
{
  return(SIM_power_gatecap(width,wirelength));
  /* return(width*Leff*PARM_Cgatepass+wirelength*Cpolywire*Leff); */
}


/*----------------------------------------------------------------------*/

/* Routine for calculating drain capacitances.  The draincap routine
 * folds transistors larger than 10um */
double SIM_power_draincap(double width,int nchannel,int stack)  /* returns drain cap in Farads */
//double width;         /* um */
//int nchannel;         /* whether n or p-channel (boolean) */
//int stack;            /* number of transistors in series that are on */
{
  double Cdiffside,Cdiffarea,Coverlap,cap;

  double overlapCap;
  double swAreaUnderGate;
  double area_peri;
  double diffArea;
  double diffPeri;
  double l = 0.4 * LSCALE;


  diffArea = l * width;
  diffPeri = 2 * l + 2 * width;

#if defined(Pdelta_w)
  if(nchannel == 0) {
    overlapCap = (width - 2 * Pdelta_w) * PCov;
    swAreaUnderGate = (width - 2 * Pdelta_w) * PCjswA;
    area_peri = ((diffArea * PCja)
                 +  (diffPeri * PCjsw));

    return(stack*(area_peri + overlapCap + swAreaUnderGate));
  }
  else {
    overlapCap = (width - 2 * Ndelta_w) * NCov;
    swAreaUnderGate = (width - 2 * Ndelta_w) * NCjswA;
    area_peri = ((diffArea * NCja * LSCALE)
                 +  (diffPeri * NCjsw * LSCALE));

    return(stack*(area_peri + overlapCap + swAreaUnderGate));
  }
#endif

        Cdiffside = (nchannel) ? PARM_Cndiffside : PARM_Cpdiffside;
        Cdiffarea = (nchannel) ? PARM_Cndiffarea : PARM_Cpdiffarea;
        Coverlap = (nchannel) ? (PARM_Cndiffovlp+PARM_Cnoxideovlp) :
                                (PARM_Cpdiffovlp+PARM_Cpoxideovlp);
        /* calculate directly-connected (non-stacked) capacitance */
        /* then add in capacitance due to stacking */
        if (width >= 10) {
            cap = 3.0*Leff*width/2.0*Cdiffarea + 6.0*Leff*Cdiffside +
                width*Coverlap;
            cap += (double)(stack-1)*(Leff*width*Cdiffarea +
                4.0*Leff*Cdiffside + 2.0*width*Coverlap);
        } else {
            cap = 3.0*Leff*width*Cdiffarea + (6.0*Leff+width)*Cdiffside +
                width*Coverlap;
            cap += (double)(stack-1)*(Leff*width*Cdiffarea +
                2.0*Leff*Cdiffside + 2.0*width*Coverlap);
        }
        return(cap * SCALE_T);
}


/*----------------------------------------------------------------------*/

/* The following routines estimate the effective resistance of an
   on transistor as described in the tech report.  The first routine
   gives the "switching" resistance, and the second gives the
   "full-on" resistance */
double SIM_power_transresswitch(double width,int nchannel,int stack)  /* returns resistance in ohms */
//double width;         /* um */
//int nchannel;         /* whether n or p-channel (boolean) */
//int stack;            /* number of transistors in series */
{
        double restrans;
        restrans = (nchannel) ? (Rnchannelstatic):
                                (Rpchannelstatic);
        /* calculate resistance of stack - assume all but switching trans
           have 0.8X the resistance since they are on throughout switching */
        return((1.0+((stack-1.0)*0.8))*restrans/width);
}


/*----------------------------------------------------------------------*/

double SIM_power_transreson(double width,int nchannel,int stack)  /* returns resistance in ohms */
//double width;           /* um */
//int nchannel;           /* whether n or p-channel (boolean) */
//int stack;              /* number of transistors in series */
{
        double restrans;
        restrans = (nchannel) ? Rnchannelon : Rpchannelon;

      /* calculate resistance of stack.  Unlike transres, we don't
           multiply the stacked transistors by 0.8 */
        return(stack*restrans/width);
}


/*----------------------------------------------------------------------*/

/* This routine operates in reverse: given a resistance, it finds
 * the transistor width that would have this R.  It is used in the
 * data wordline to estimate the wordline driver size. */
double SIM_power_restowidth(double res,int nchannel)  /* returns width in um */
//double res;            /* resistance in ohms */
//int nchannel;          /* whether N-channel or P-channel */
{
   double restrans;

        restrans = (nchannel) ? Rnchannelon : Rpchannelon;

   return(restrans/res);
}


/*----------------------------------------------------------------------*/

double SIM_power_horowitz(double inputramptime,double tf,double vs1,double vs2,int rise)
//double inputramptime,    /* input rise time */
  //     tf,               /* time constant of gate */
    //   vs1,vs2;          /* threshold voltages */
//int rise;                /* whether INPUT rise or fall (boolean) */
{
    double a,b,td;

    a = inputramptime/tf;
    if (rise==RISE) {
       b = 0.5;
       td = tf*sqrt(fabs( log(vs1)*log(vs1)+2*a*b*(1.0-vs1))) +
            tf*(log(vs1)-log(vs2));
    } else {
       b = 0.4;
       td = tf*sqrt(fabs( log(1.0-vs1)*log(1.0-vs1)+2*a*b*(vs1))) +
            tf*(log(1.0-vs1)-log(1.0-vs2));
    }

    return(td);
}




double SIM_power_driver_size(double driving_cap, double desiredrisetime)
{
  double nsize, psize;
  double Rpdrive;

  Rpdrive = desiredrisetime/(driving_cap*log(PARM_VSINV)*-1.0);
  psize = SIM_power_restowidth(Rpdrive,PCH);
  nsize = SIM_power_restowidth(Rpdrive,NCH);
  if (psize > Wworddrivemax) {
    psize = Wworddrivemax;
  }
  if (psize < 4.0 * LSCALE)
    psize = 4.0 * LSCALE;

  return (psize);
}


