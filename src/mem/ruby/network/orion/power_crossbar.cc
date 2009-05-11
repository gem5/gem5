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

#include <stdio.h>
#include <math.h>

#include "power_ll.hh"
#include "power_crossbar.hh"
#include "parm_technology.hh"
#include "SIM_port.hh"
#include "power_static.hh"
#include "power_utils.hh"

/*-------------------- CROSSBAR power model -------------------*/

static double crossbar_in_cap(double wire_cap, unsigned n_out, int connect_type, int trans_type, double *Nsize)
{
  double Ctotal = 0, Ctrans = 0, psize, nsize, Cdriver = 0;

  /* part 1: wire cap */
  Ctotal += wire_cap;
  //printf("CROSSBAR_INTERNAL: input wire cap = %g\n", wire_cap);

  /* part 2: drain cap of transmission gate or gate cap of tri-state gate */
  if (connect_type == TRANS_GATE) {
    /* FIXME: resizing strategy */
    nsize = Nsize ? *Nsize : Wmemcellr;
    psize = nsize * Wdecinvp / Wdecinvn;
    Ctrans = SIM_power_draincap(nsize, NCH, 1);
    if (trans_type == NP_GATE)
      Ctrans += SIM_power_draincap(psize, PCH, 1);
  }
  else if (connect_type == TRISTATE_GATE) {
    Ctrans = SIM_power_gatecap(Woutdrvnandn + Woutdrvnandp, 0) +
             SIM_power_gatecap(Woutdrvnorn + Woutdrvnorp, 0);
  }
  else {/* some error handler */}

  //printf("CROSSBAR_INTERNAL: input connector cap = %g\n", (n_out * Ctrans));
  Ctotal += n_out * Ctrans;

  /* part 3: input driver */
  /* FIXME: how to specify timing? */
  psize = SIM_power_driver_size(Ctotal, Period / 3);
  nsize = psize * Wdecinvn / Wdecinvp;
  Cdriver = SIM_power_draincap(nsize, NCH, 1) + SIM_power_draincap(psize, PCH, 1) +
            SIM_power_gatecap(nsize + psize, 0);

  //printf("CROSSBAR_INTERNAL: input driver cap = %g\n", Cdriver);

  Ctotal += Cdriver;

  return Ctotal / 2;
}


static double crossbar_out_cap(double length, unsigned n_in, int connect_type, int trans_type, double *Nsize)
{
  double Ctotal = 0, Cwire = 0, Ctrans = 0, Cdriver = 0, psize, nsize;

  /* part 1: wire cap */
  Cwire += CC3metal * length;
  //printf("CROSSBAR_INTERNAL: output wire cap = %g\n", Cwire);

  Ctotal += Cwire;

  /* part 2: drain cap of transmission gate or tri-state gate */
  if (connect_type == TRANS_GATE) {
    /* FIXME: resizing strategy */
    if (Nsize) {
      /* FIXME: how to specify timing? */
      psize = SIM_power_driver_size(Ctotal, Period / 3);
      *Nsize = nsize = psize * Wdecinvn / Wdecinvp;
    }
    else {
      nsize = Wmemcellr;
      psize = nsize * Wdecinvp / Wdecinvn;
    }
    Ctrans = SIM_power_draincap(nsize, NCH, 1);
    if (trans_type == NP_GATE)
      Ctrans += SIM_power_draincap(psize, PCH, 1);
  }
  else if (connect_type == TRISTATE_GATE) {
    Ctrans = SIM_power_draincap(Woutdrivern, NCH, 1) + SIM_power_draincap(Woutdriverp, PCH, 1);
  }
  else {/* some error handler */}

  //printf("CROSSBAR_INTERNAL: output connector cap = %g\n", (n_in * Ctrans));
  Ctotal += n_in * Ctrans;

  /* part 3: output driver */
  Cdriver += SIM_power_draincap(Woutdrivern, NCH, 1) + SIM_power_draincap(Woutdriverp, PCH, 1) +
            SIM_power_gatecap(Woutdrivern + Woutdriverp, 0);

  //printf("CROSSBAR_INTERNAL: output driver cap = %g\n", Cdriver);

  Ctotal += Cdriver;

  return Ctotal / 2;
}


/* cut-through crossbar only supports 4x4 now */
static double crossbar_io_cap(double length)
{
  double Ctotal = 0, psize, nsize;

  /* part 1: wire cap */
  Ctotal += CC3metal * length;

  /* part 2: gate cap of tri-state gate */
  Ctotal += 2 * (SIM_power_gatecap(Woutdrvnandn + Woutdrvnandp, 0) +
                 SIM_power_gatecap(Woutdrvnorn + Woutdrvnorp, 0));

  /* part 3: drain cap of tri-state gate */
  Ctotal += 2 * (SIM_power_draincap(Woutdrivern, NCH, 1) + SIM_power_draincap(Woutdriverp, PCH, 1));

  /* part 4: input driver */
  /* FIXME: how to specify timing? */
  psize = SIM_power_driver_size(Ctotal, Period * 0.8);
  nsize = psize * Wdecinvn / Wdecinvp;
  Ctotal += SIM_power_draincap(nsize, NCH, 1) + SIM_power_draincap(psize, PCH, 1) +
            SIM_power_gatecap(nsize + psize, 0);

  /* part 5: output driver */
  Ctotal += SIM_power_draincap(Woutdrivern, NCH, 1) + SIM_power_draincap(Woutdriverp, PCH, 1) +
            SIM_power_gatecap(Woutdrivern + Woutdriverp, 0);

  /* HACK HACK HACK */
  /* this HACK is to count a 1:4 mux and a 4:1 mux, so we have a 5x5 crossbar */
  return Ctotal / 2 * 1.32;
}


static double crossbar_int_cap(unsigned degree, int connect_type, int trans_type)
{
  double Ctotal = 0, Ctrans;

  if (connect_type == TRANS_GATE) {
    /* part 1: drain cap of transmission gate */
    /* FIXME: Wmemcellr and resize */
    Ctrans = SIM_power_draincap(Wmemcellr, NCH, 1);
    if (trans_type == NP_GATE)
      Ctrans += SIM_power_draincap(Wmemcellr * Wdecinvp / Wdecinvn, PCH, 1);
    Ctotal += (degree + 1) * Ctrans;
  }
  else if (connect_type == TRISTATE_GATE) {
    /* part 1: drain cap of tri-state gate */
    Ctotal += degree * (SIM_power_draincap(Woutdrivern, NCH, 1) + SIM_power_draincap(Woutdriverp, PCH, 1));

    /* part 2: gate cap of tri-state gate */
    Ctotal += SIM_power_gatecap(Woutdrvnandn + Woutdrvnandp, 0) +
              SIM_power_gatecap(Woutdrvnorn + Woutdrvnorp, 0);
  }
  else {/* some error handler */}

  return Ctotal / 2;
}


/* FIXME: segment control signals are not handled yet */
static double crossbar_ctr_cap(double length, unsigned data_width, int prev_ctr, int next_ctr, unsigned degree, int connect_type, int trans_type)
{
  double Ctotal = 0, Cgate;

  /* part 1: wire cap */
  Ctotal = Cmetal * length;

  /* part 2: gate cap of transmission gate or tri-state gate */
  if (connect_type == TRANS_GATE) {
    /* FIXME: Wmemcellr and resize */
    Cgate = SIM_power_gatecap(Wmemcellr, 0);
    if (trans_type == NP_GATE)
      Cgate += SIM_power_gatecap(Wmemcellr * Wdecinvp / Wdecinvn, 0);
  }
  else if (connect_type == TRISTATE_GATE) {
    Cgate = SIM_power_gatecap(Woutdrvnandn + Woutdrvnandp, 0) +
            SIM_power_gatecap(Woutdrvnorn + Woutdrvnorp, 0);
  }
  else {/* some error handler */}

  Ctotal += data_width * Cgate;

  /* part 3: inverter */
  if (!(connect_type == TRANS_GATE && trans_type == N_GATE && !prev_ctr))
    /* FIXME: need accurate size, use minimal size for now */
    Ctotal += SIM_power_draincap(Wdecinvn, NCH, 1) + SIM_power_draincap(Wdecinvp, PCH, 1) +
              SIM_power_gatecap(Wdecinvn + Wdecinvp, 0);

  /* part 4: drain cap of previous level control signal */
  if (prev_ctr)
    /* FIXME: need actual size, use decoder data for now */
    Ctotal += degree * SIM_power_draincap(WdecNORn, NCH, 1) + SIM_power_draincap(WdecNORp, PCH, degree);

  /* part 5: gate cap of next level control signal */
  if (next_ctr)
    /* FIXME: need actual size, use decoder data for now */
    Ctotal += SIM_power_gatecap(WdecNORn + WdecNORp, degree * 40 + 20);

  return Ctotal;
}


int power_crossbar_init(power_crossbar *crsbar, int model, unsigned n_in, unsigned n_out, unsigned data_width, unsigned degree, int connect_type, int trans_type, double in_len, double out_len, double *req_len)
{
  double in_length, out_length, ctr_length, Nsize, in_wire_cap, i_leakage;

  if ((crsbar->model = model) && model < CROSSBAR_MAX_MODEL) {
    crsbar->n_in = n_in;
    crsbar->n_out = n_out;
    crsbar->data_width = data_width;
    crsbar->degree = degree;
    crsbar->connect_type = connect_type;
    crsbar->trans_type = trans_type;
    /* redundant field */
    crsbar->mask = HAMM_MASK(data_width);

    crsbar->n_chg_in = crsbar->n_chg_int = crsbar->n_chg_out = crsbar->n_chg_ctr = 0;

    switch (model) {
      case MATRIX_CROSSBAR:

           /* FIXME: need accurate spacing */
           in_length = n_out * data_width * CrsbarCellWidth;
           out_length = n_in * data_width * CrsbarCellHeight;
           if (in_length < in_len) in_length = in_len;
           if (out_length < out_len) out_length = out_len;
           ctr_length = in_length / 2;
           if (req_len) *req_len = in_length;

           in_wire_cap = in_length * CC3metal;

           crsbar->e_chg_out = crossbar_out_cap(out_length, n_in, connect_type, trans_type, &Nsize) * EnergyFactor;
           crsbar->e_chg_in = crossbar_in_cap(in_wire_cap, n_out, connect_type, trans_type, &Nsize) * EnergyFactor;
           /* FIXME: wire length estimation, really reset? */
           /* control signal should reset after transmission is done, so no 1/2 */
           crsbar->e_chg_ctr = crossbar_ctr_cap(ctr_length, data_width, 0, 0, 0, connect_type, trans_type) * EnergyFactor;
           crsbar->e_chg_int = 0;

           /* static power */
           i_leakage = 0;
           /* tri-state buffers */
           i_leakage += ((Woutdrvnandp * (NAND2_TAB[0] + NAND2_TAB[1] + NAND2_TAB[2]) + Woutdrvnandn * NAND2_TAB[3]) / 4 +
                        (Woutdrvnorp * NOR2_TAB[0] + Woutdrvnorn * (NOR2_TAB[1] + NOR2_TAB[2] + NOR2_TAB[3])) / 4 +
                         Woutdrivern * NMOS_TAB[0] + Woutdriverp * PMOS_TAB[0]) * n_in * n_out * data_width;
           /* input driver */
           i_leakage += (Wdecinvn * NMOS_TAB[0] + Wdecinvp * PMOS_TAB[0]) * n_in * data_width;
           /* output driver */
           i_leakage += (Woutdrivern * NMOS_TAB[0] + Woutdriverp * PMOS_TAB[0]) * n_out * data_width;
           /* control signal inverter */
           i_leakage += (Wdecinvn * NMOS_TAB[0] + Wdecinvp * PMOS_TAB[0]) * n_in * n_out;
           crsbar->i_leakage = i_leakage / PARM_TECH_POINT * 100;
           break;

      case MULTREE_CROSSBAR:
           /* input wire horizontal segment length */
           in_length = n_in * data_width * CrsbarCellWidth * (n_out / 2);
           in_wire_cap = in_length * CCmetal;
           /* input wire vertical segment length */
           in_length = n_in * data_width * (5 * Lamda) * (n_out / 2);
           in_wire_cap += in_length * CC3metal;

           ctr_length = n_in * data_width * CrsbarCellWidth * (n_out / 2) / 2;

           crsbar->e_chg_out = crossbar_out_cap(0, degree, connect_type, trans_type, NULL) * EnergyFactor;
           crsbar->e_chg_in = crossbar_in_cap(in_wire_cap, n_out, connect_type, trans_type, NULL) * EnergyFactor;
           crsbar->e_chg_int = crossbar_int_cap(degree, connect_type, trans_type) * EnergyFactor;

           /* redundant field */
           crsbar->depth = (unsigned)ceil(log(n_in) / log(degree));

           /* control signal should reset after transmission is done, so no 1/2 */
           if (crsbar->depth == 1)
             /* only one level of control signal */
             crsbar->e_chg_ctr = crossbar_ctr_cap(ctr_length, data_width, 0, 0, degree, connect_type, trans_type) * EnergyFactor;
           else {
             /* first level and last level control signals */
             crsbar->e_chg_ctr = crossbar_ctr_cap(ctr_length, data_width, 0, 1, degree, connect_type, trans_type) * EnergyFactor +
                                 crossbar_ctr_cap(0, data_width, 1, 0, degree, connect_type, trans_type) * EnergyFactor;
             /* intermediate control signals */
             if (crsbar->depth > 2)
               crsbar->e_chg_ctr += (crsbar->depth - 2) * crossbar_ctr_cap(0, data_width, 1, 1, degree, connect_type, trans_type) * EnergyFactor;
           }

           /* static power */
           i_leakage = 0;
           /* input driver */
           i_leakage += (Wdecinvn * NMOS_TAB[0] + Wdecinvp * PMOS_TAB[0]) * n_in * data_width;
           /* output driver */
           i_leakage += (Woutdrivern * NMOS_TAB[0] + Woutdriverp * PMOS_TAB[0]) * n_out * data_width;
           /* mux */
           i_leakage += (WdecNORp * NOR2_TAB[0] + WdecNORn * (NOR2_TAB[1] + NOR2_TAB[2] + NOR2_TAB[3])) / 4 * (2 * n_in - 1) * n_out * data_width;
           /* control signal inverter */
           i_leakage += (Wdecinvn * NMOS_TAB[0] + Wdecinvp * PMOS_TAB[0]) * n_in * n_out;
           crsbar->i_leakage = i_leakage / PARM_TECH_POINT * 100;
           break;

      default:  break;/* some error handler */
    }

    return 0;
  }
  else
    return -1;
}


/* FIXME: MULTREE_CROSSBAR record missing */
int crossbar_record(power_crossbar *xb, int io, unsigned long int new_data, unsigned long int old_data, unsigned new_port, unsigned old_port)
{
  switch (xb->model) {
    case MATRIX_CROSSBAR:
         if (io)        /* input port */
           xb->n_chg_in += SIM_power_Hamming(new_data, old_data, xb->mask);
         else {         /* output port */
           xb->n_chg_out += SIM_power_Hamming(new_data, old_data, xb->mask);
           xb->n_chg_ctr += new_port != old_port;
         }
         break;

    case MULTREE_CROSSBAR:
         break;

    default:    break;/* some error handler */
  }

  return 0;
}


double crossbar_report(power_crossbar *crsbar)
{
  return (crsbar->n_chg_in * crsbar->e_chg_in + crsbar->n_chg_out * crsbar->e_chg_out +
          crsbar->n_chg_int * crsbar->e_chg_int + crsbar->n_chg_ctr * crsbar->e_chg_ctr);
}

/* ---------- crossbar model ---------- */



