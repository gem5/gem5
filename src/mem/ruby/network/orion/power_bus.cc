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

#include "mem/ruby/network/orion/power_bus.hh"
#include "mem/ruby/network/orion/power_ll.hh"
#include "mem/ruby/network/orion/parm_technology.hh"
#include "mem/ruby/network/orion/SIM_port.hh"
#include "mem/ruby/network/orion/power_static.hh"
#include "mem/ruby/network/orion/power_utils.hh"

/* ------- bus(link) model ---------- */

static int SIM_bus_bitwidth(int encoding, unsigned data_width, unsigned grp_width)
{
  if (encoding && encoding < BUS_MAX_ENC)
    switch (encoding) {
      case IDENT_ENC:
      case TRANS_ENC:   return data_width;
      case BUSINV_ENC:  return data_width + data_width / grp_width + (data_width % grp_width ? 1:0);
      default:  return 0;/* some error handler */
    }
  else
    return -1;
}


/*
 * this function is provided to upper layers to compute the exact binary bus representation
 * only correct when grp_width divides data_width
 */
unsigned long int SIM_bus_state(power_bus *bus, unsigned long int old_data, unsigned long int old_state, unsigned long int new_data)
{
  unsigned long int mask_bus, mask_data;
  unsigned long int new_state = 0;
  unsigned done_width = 0;

  switch (bus->encoding) {
    case IDENT_ENC:     return new_data;
    case TRANS_ENC:     return new_data ^ old_data;

    case BUSINV_ENC:
         /* FIXME: this function should be re-written for boundary checking */
         mask_data = (BIGONE << bus->grp_width) - 1;
         mask_bus = (mask_data << 1) + 1;

         while (bus->data_width > done_width) {
           if (SIM_power_Hamming(old_state & mask_bus, new_data & mask_data, mask_bus) > bus->grp_width / 2)
             new_state += (~(new_data & mask_data) & mask_bus) << done_width + done_width / bus->grp_width;
           else
             new_state += (new_data & mask_data) << done_width + done_width / bus->grp_width;

           done_width += bus->grp_width;
           old_state >>= bus->grp_width + 1;
           new_data >>= bus->grp_width;
         }

         return new_state;

    default:    return 0;/* some error handler */
  }
}


static double SIM_resultbus_cap(void)
{
  double Cline, reg_height;

  /* compute size of result bus tags */
  reg_height = PARM_RUU_size * (RegCellHeight + WordlineSpacing * 3 * PARM_ruu_issue_width);

  /* assume num alu's = ialu */
  /* FIXME: generate a more detailed result bus network model */
  /* WHS: 3200 should go to PARM */
  /* WHS: use minimal pitch for buses */
  Cline = CCmetal * (reg_height + 0.5 * PARM_res_ialu * 3200 * LSCALE);

  /* or use result bus length measured from 21264 die photo */
  // Cline = CCmetal * 3.3 * 1000;

  return Cline;
}


static double SIM_generic_bus_cap(unsigned n_snd, unsigned n_rcv, double length, double time)
{
  double Ctotal = 0;
  double n_size, p_size;

  /* part 1: wire cap */
  /* WHS: use minimal pitch for buses */
  Ctotal += CC2metal * length;

  if ((n_snd == 1) && (n_rcv == 1)) {
    /* directed bus if only one sender and one receiver */

    /* part 2: repeater cap */
    /* FIXME: ratio taken from Raw, does not scale now */
    n_size = Lamda * 10;
    p_size = n_size * 2;

    Ctotal += SIM_power_gatecap(n_size + p_size, 0) + SIM_power_draincap(n_size, NCH, 1) + SIM_power_draincap(p_size, PCH, 1);

    n_size *= 2.5;
    p_size *= 2.5;

    Ctotal += SIM_power_gatecap(n_size + p_size, 0) + SIM_power_draincap(n_size, NCH, 1) + SIM_power_draincap(p_size, PCH, 1);
  }
  else {
    /* otherwise, broadcasting bus */

    /* part 2: input cap */
    /* WHS: no idea how input interface is, use an inverter for now */
    Ctotal += n_rcv * SIM_power_gatecap(Wdecinvn + Wdecinvp, 0);

    /* part 3: output driver cap */
    if (time) {
      p_size = SIM_power_driver_size(Ctotal, time);
      n_size = p_size / 2;
    }
    else {
      p_size = Wbusdrvp;
      n_size = Wbusdrvn;
    }

    Ctotal += n_snd * (SIM_power_draincap(Wdecinvn, NCH, 1) + SIM_power_draincap(Wdecinvp, PCH, 1));
  }

  return Ctotal;
}


/*
 * n_snd -> # of senders
 * n_rcv -> # of receivers
 * time  -> rise and fall time, 0 means using default transistor sizes
 * grp_width only matters for BUSINV_ENC
 */
int power_bus_init(power_bus *bus, int model, int encoding, unsigned width, unsigned grp_width, unsigned n_snd, unsigned n_rcv, double length, double time)
{
  if ((bus->model = model) && model < BUS_MAX_MODEL) {
    bus->data_width = width;
    bus->grp_width = grp_width;
    bus->n_switch = 0;

    switch (model) {
      case RESULT_BUS:
           /* assume result bus uses identity encoding */
           bus->encoding = IDENT_ENC;
           bus->e_switch = SIM_resultbus_cap() / 2 * EnergyFactor;
           break;

      case GENERIC_BUS:
           if ((bus->encoding = encoding) && encoding < BUS_MAX_ENC) {
             bus->e_switch = SIM_generic_bus_cap(n_snd, n_rcv, length, time) / 2 * EnergyFactor;
             /* sanity check */
             if (!grp_width || grp_width > width)
               bus->grp_width = width;
           }
           else return -1;

      default:  break;/* some error handler */
    }

    bus->bit_width = SIM_bus_bitwidth(bus->encoding, width, bus->grp_width);
    bus->bus_mask = HAMM_MASK(bus->bit_width);

    return 0;
  }
  else
    return -1;
}


int bus_record(power_bus *bus, unsigned long int old_state, unsigned long int new_state)
{
  bus->n_switch += SIM_power_Hamming(new_state, old_state, bus->bus_mask);
  return 0;
}


double bus_report(power_bus *bus)
{
  return (bus->n_switch * bus->e_switch);
}

/* ------- bus(link) model ---------- */


