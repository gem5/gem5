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

#include "mem/ruby/network/orion/power_array.hh"
#include "mem/ruby/network/orion/power_ll.hh"
#include "mem/ruby/network/orion/parm_technology.hh"
#include "mem/ruby/network/orion/SIM_port.hh"
#include "mem/ruby/network/orion/power_static.hh"
#include "mem/ruby/network/orion/power_utils.hh"

/* local macros */

#define IS_DIRECT_MAP( info )           ((info)->assoc == 1)
#define IS_FULLY_ASSOC( info )          ((info)->n_set == 1 && (info)->assoc > 1)
#define IS_WRITE_THROUGH( info )        (! (info)->write_policy)
#define IS_WRITE_BACK( info )           ((info)->write_policy)

/* sufficient (not necessary) condition */
#define HAVE_TAG( info )                ((info)->tag_mem_model)
#define HAVE_USE_BIT( info )            ((info)->use_bit_width)
#define HAVE_COL_DEC( info )            ((info)->col_dec_model)
#define HAVE_COL_MUX( info )            ((info)->mux_model)


/* ----------------------------- CAM ---------------------------------- */
/*============================== wordlines ==============================*/

/* each time one wordline 1->0, another wordline 0->1, so no 1/2 */
double SIM_cam_wordline_cap( unsigned cols, double wire_cap, double tx_width )
{
  double Ctotal, Cline, psize, nsize;

  /* part 1: line cap, including gate cap of pass tx's and metal cap */
  Ctotal = Cline = SIM_power_gatecappass( tx_width, 2 ) * cols + wire_cap;

  /* part 2: input driver */
  psize = SIM_power_driver_size( Cline, Period / 8 );
  nsize = psize * Wdecinvn / Wdecinvp;
  /* WHS: 20 should go to PARM */
  Ctotal += SIM_power_draincap( nsize, NCH, 1 ) + SIM_power_draincap( psize, PCH, 1 ) +
            SIM_power_gatecap( nsize + psize, 20 );

  return Ctotal;
}

/*============================== wordlines ==============================*/



/*============================== tag comparator ==============================*/

/* tag and tagbar switch simultaneously, so no 1/2 */
double SIM_cam_comp_tagline_cap( unsigned rows, double taglinelength )
{
  double Ctotal;

  /* part 1: line cap, including drain cap of pass tx's and metal cap */
  Ctotal = rows * SIM_power_gatecap( Wcomparen2, 2 ) + CC3M2metal * taglinelength;

  /* part 2: input driver */
  Ctotal += SIM_power_draincap( Wcompdrivern, NCH, 1 ) + SIM_power_draincap( Wcompdriverp, PCH, 1 ) +
            SIM_power_gatecap( Wcompdrivern + Wcompdriverp, 1 );

  return Ctotal;
}


/* upon mismatch, matchline 1->0, then 0->1 on next precharging, so no 1/2 */
double SIM_cam_comp_mismatch_cap( unsigned n_bits, unsigned n_pre, double matchline_len )
{
  double Ctotal;

  /* part 1: drain cap of precharge tx */
  Ctotal = n_pre * SIM_power_draincap( Wmatchpchg, PCH, 1 );

  /* part 2: drain cap of comparator tx */
  Ctotal += n_bits * ( SIM_power_draincap( Wcomparen1, NCH, 1 ) + SIM_power_draincap( Wcomparen1, NCH, 2 ));

  /* part 3: metal cap of matchline */
  Ctotal += CC3M3metal * matchline_len;

  /* FIXME: I don't understand the Wattch code here */
  /* part 4: nor gate of valid output */
  Ctotal += SIM_power_gatecap( Wmatchnorn + Wmatchnorp, 10 );

  return Ctotal;
}


/* WHS: subtle difference of valid output between cache and inst window:
 *   fully-associative cache: nor all matchlines of the same port
 *   instruction window:      nor all matchlines of the same tag line */
/* upon miss, valid output switches twice in one cycle, so no 1/2 */
double SIM_cam_comp_miss_cap( unsigned assoc )
{
  /* drain cap of valid output */
  return ( assoc * SIM_power_draincap( Wmatchnorn, NCH, 1 ) + SIM_power_draincap( Wmatchnorp, PCH, assoc ));
}

/*============================== tag comparator ==============================*/



/*============================== memory cell ==============================*/

/* WHS: use Wmemcella and Wmemcellbscale to compute tx width of memory cell */
double SIM_cam_tag_mem_cap( unsigned read_ports, unsigned write_ports, int share_rw, unsigned end, int only_write )
{
  double Ctotal;

  /* part 1: drain capacitance of pass transistors */
  if ( only_write )
    Ctotal = SIM_power_draincap( Wmemcellw, NCH, 1 ) * write_ports;
  else {
    Ctotal = SIM_power_draincap( Wmemcellr, NCH, 1 ) * read_ports * end / 2;
    if ( ! share_rw )
      Ctotal += SIM_power_draincap( Wmemcellw, NCH, 1 ) * write_ports;
  }

  /* has coefficient ( 1/2 * 2 ) */
  /* part 2: drain capacitance of memory cell */
  Ctotal += SIM_power_draincap( Wmemcella, NCH, 1 ) + SIM_power_draincap( Wmemcella * Wmemcellbscale, PCH, 1 );

  /* has coefficient ( 1/2 * 2 ) */
  /* part 3: gate capacitance of memory cell */
  Ctotal += SIM_power_gatecap( Wmemcella, 1 ) + SIM_power_gatecap( Wmemcella * Wmemcellbscale, 1 );

  /* has coefficient ( 1/2 * 2 ) */
  /* part 4: gate capacitance of comparator */
  Ctotal += SIM_power_gatecap( Wcomparen1, 2 ) * read_ports;

  return Ctotal;
}


double SIM_cam_data_mem_cap( unsigned read_ports, unsigned write_ports )
{
  double Ctotal;

  /* has coefficient ( 1/2 * 2 ) */
  /* part 1: drain capacitance of pass transistors */
  Ctotal = SIM_power_draincap( Wmemcellw, NCH, 1 ) * write_ports;

  /* has coefficient ( 1/2 * 2 ) */
  /* part 2: drain capacitance of memory cell */
  Ctotal += SIM_power_draincap( Wmemcella, NCH, 1 ) + SIM_power_draincap( Wmemcella * Wmemcellbscale, PCH, 1 );

  /* has coefficient ( 1/2 * 2 ) */
  /* part 3: gate capacitance of memory cell */
  Ctotal += SIM_power_gatecap( Wmemcella, 1 ) + SIM_power_gatecap( Wmemcella * Wmemcellbscale, 1 );

  /* part 4: gate capacitance of output driver */
  Ctotal += ( SIM_power_gatecap( Woutdrvnandn, 1 ) + SIM_power_gatecap( Woutdrvnandp, 1 ) +
              SIM_power_gatecap( Woutdrvnorn, 1 ) + SIM_power_gatecap( Woutdrvnorp, 1 )) / 2 * read_ports;

  return Ctotal;
}

/*============================== memory cell ==============================*/







/* ---------- buffer model ------------ */

// ------- Decoder begin

/*#
 * compute switching cap when decoder changes output (select signal)
 *
 * Parameters:
 *   n_input -- fanin of 1 gate of last level decoder
 *
 * Return value: switching cap
 *
 * NOTES: 2 select signals switch, so no 1/2
 */
static double SIM_array_dec_select_cap( unsigned n_input )
{
  double Ctotal = 0;

  /* FIXME: why? */
  // if ( numstack > 5 ) numstack = 5;

  /* part 1: drain cap of last level decoders */
  Ctotal = n_input * SIM_power_draincap( WdecNORn, NCH, 1 ) + SIM_power_draincap( WdecNORp, PCH, n_input );

  /* part 2: output inverter */
  /* WHS: 20 should go to PARM */
  Ctotal += SIM_power_draincap( Wdecinvn, NCH, 1 ) + SIM_power_draincap( Wdecinvp, PCH, 1) +
            SIM_power_gatecap( Wdecinvn + Wdecinvp, 20 );

  return Ctotal;
}


/*#
 * compute switching cap when 1 input bit of decoder changes
 *
 * Parameters:
 *   n_gates -- fanout of 1 addr signal
 *
 * Return value: switching cap
 *
 * NOTES: both addr and its complement change, so no 1/2
 */
static double SIM_array_dec_chgaddr_cap( unsigned n_gates )
{
  double Ctotal;

  /* stage 1: input driver */
  Ctotal = SIM_power_draincap( Wdecdrivep, PCH, 1 ) + SIM_power_draincap( Wdecdriven, NCH, 1 ) +
           SIM_power_gatecap( Wdecdrivep, 1 ) + SIM_power_gatecap( Wdecdriven, 1 );
  /* inverter to produce complement addr, this needs 1/2 */
  /* WHS: assume Wdecinv(np) for this inverter */
  Ctotal += ( SIM_power_draincap( Wdecinvp, PCH, 1 ) + SIM_power_draincap( Wdecinvn, NCH, 1 ) +
              SIM_power_gatecap( Wdecinvp, 1 ) + SIM_power_gatecap( Wdecinvn, 1 )) / 2;

  /* stage 2: gate cap of level-1 decoder */
  /* WHS: 10 should go to PARM */
  Ctotal += n_gates * SIM_power_gatecap( Wdec3to8n + Wdec3to8p, 10 );

  return Ctotal;
}


/*#
 * compute switching cap when 1st-level decoder changes output
 *
 * Parameters:
 *   n_in_1st -- fanin of 1 gate of 1st-level decoder
 *   n_in_2nd -- fanin of 1 gate of 2nd-level decoder
 *   n_gates  -- # of gates of 2nd-level decoder, i.e.
 *               fanout of 1 gate of 1st-level decoder
 *
 * Return value: switching cap
 *
 * NOTES: 2 complementary signals switch, so no 1/2
 */
static double SIM_array_dec_chgl1_cap( unsigned n_in_1st, unsigned n_in_2nd, unsigned n_gates )
{
  double Ctotal;

  /* part 1: drain cap of level-1 decoder */
  Ctotal = n_in_1st * SIM_power_draincap( Wdec3to8p, PCH, 1 ) + SIM_power_draincap( Wdec3to8n, NCH, n_in_1st );

  /* part 2: gate cap of level-2 decoder */
  /* WHS: 40 and 20 should go to PARM */
  Ctotal += n_gates * SIM_power_gatecap( WdecNORn + WdecNORp, n_in_2nd * 40 + 20 );

  return Ctotal;
}


static int SIM_array_dec_clear_stat(power_decoder *dec)
{
  dec->n_chg_output = dec->n_chg_l1 = dec->n_chg_addr = 0;

  return 0;
}


/*#
 * initialize decoder
 *
 * Parameters:
 *   dec    -- decoder structure
 *   model  -- decoder model type
 *   n_bits -- decoder width
 *
 * Side effects:
 *   initialize dec structure if model type is valid
 *
 * Return value: -1 if model type is invalid
 *               0 otherwise
 */
static int SIM_array_dec_init(power_decoder *dec, int model, unsigned n_bits )
{
  if ((dec->model = model) && model < DEC_MAX_MODEL) {
    dec->n_bits = n_bits;
    /* redundant field */
    dec->addr_mask = HAMM_MASK(n_bits);

    SIM_array_dec_clear_stat(dec);
    dec->e_chg_output = dec->e_chg_l1 = dec->e_chg_addr = 0;

    /* compute geometry parameters */
    if ( n_bits >= 4 ) {                /* 2-level decoder */
      /* WHS: inaccurate for some n_bits */
      dec->n_in_1st = ( n_bits == 4 ) ? 2:3;
      dec->n_out_0th = BIGONE << ( dec->n_in_1st - 1 );
      dec->n_in_2nd = (unsigned)ceil((double)n_bits / dec->n_in_1st );
      dec->n_out_1st = BIGONE << ( n_bits - dec->n_in_1st );
    }
    else if ( n_bits >= 2 ) {   /* 1-level decoder */
      dec->n_in_1st = n_bits;
      dec->n_out_0th = BIGONE << ( n_bits - 1 );
      dec->n_in_2nd = dec->n_out_1st = 0;
    }
    else {                      /* no decoder basically */
      dec->n_in_1st = dec->n_in_2nd = dec->n_out_0th = dec->n_out_1st = 0;
    }

    /* compute energy constants */
    if ( n_bits >= 2 ) {
      dec->e_chg_l1 = SIM_array_dec_chgl1_cap( dec->n_in_1st, dec->n_in_2nd, dec->n_out_1st ) * EnergyFactor;
      if ( n_bits >= 4 )
        dec->e_chg_output = SIM_array_dec_select_cap( dec->n_in_2nd ) * EnergyFactor;
    }
    dec->e_chg_addr = SIM_array_dec_chgaddr_cap( dec->n_out_0th ) * EnergyFactor;

    return 0;
  }
  else
    return -1;
}


/*#
 * record decoder power stats
 *
 * Parameters:
 *   dec       -- decoder structure
 *   prev_addr -- previous input
 *   curr_addr -- current input
 *
 * Side effects:
 *   update counters in dec structure
 *
 * Return value: 0
 */
int SIM_array_dec_record(power_decoder *dec, unsigned long int prev_addr, unsigned long int curr_addr )
{
  unsigned n_chg_bits, n_chg_l1 = 0, n_chg_output = 0;
  unsigned i;
  unsigned long int mask;

  /* compute Hamming distance */
  n_chg_bits = SIM_power_Hamming( prev_addr, curr_addr, dec->addr_mask );
  if ( n_chg_bits ) {
    if ( dec->n_bits >= 4 ) {           /* 2-level decoder */
      /* WHS: inaccurate for some n_bits */
      n_chg_output ++;
      /* count addr group changes */
      mask = HAMM_MASK(dec->n_in_1st);
      for ( i = 0; i < dec->n_in_2nd; i ++ ) {
        if ( SIM_power_Hamming( prev_addr, curr_addr, mask ))
          n_chg_l1 ++;
        mask = mask << dec->n_in_1st;
      }
    }
    else if ( dec->n_bits >= 2 ) {      /* 1-level decoder */
      n_chg_l1 ++;
    }

    dec->n_chg_addr += n_chg_bits;
    dec->n_chg_l1 += n_chg_l1;
    dec->n_chg_output += n_chg_output;
  }

  return 0;
}


/*#
 * report decoder power stats
 *
 * Parameters:
 *   dec -- decoder structure
 *
 * Return value: total energy consumption of this decoder
 *
 * TODO: add more report functionality, currently only total energy is reported
 */
double SIM_array_dec_report(power_decoder *dec )
{
  double Etotal;

  Etotal = dec->n_chg_output * dec->e_chg_output + dec->n_chg_l1 * dec->e_chg_l1 +
           dec->n_chg_addr * dec->e_chg_addr;

  /* bonus energy for dynamic decoder :) */
  //if ( is_dynamic_dec( dec->model )) Etotal += Etotal;

  return Etotal;
}

// ------- Decoder end



// ------- Wordlines begin

/*#
 * compute wordline switching cap
 *
 * Parameters:
 *   cols           -- # of pass transistors, i.e. # of bitlines
 *   wordlinelength -- length of wordline
 *   tx_width       -- width of pass transistor
 *
 * Return value: switching cap
 *
 * NOTES: upon address change, one wordline 1->0, another 0->1, so no 1/2
 */
static double SIM_array_wordline_cap( unsigned cols, double wire_cap, double tx_width )
{
  double Ctotal, Cline, psize, nsize;

  /* part 1: line cap, including gate cap of pass tx's and metal cap */
  Ctotal = Cline = SIM_power_gatecappass( tx_width, BitWidth / 2 - tx_width ) * cols + wire_cap;

  /* part 2: input driver */
  psize = SIM_power_driver_size( Cline, Period / 16 );
  nsize = psize * Wdecinvn / Wdecinvp;
  /* WHS: 20 should go to PARM */
  Ctotal += SIM_power_draincap( nsize, NCH, 1 ) + SIM_power_draincap( psize, PCH, 1 ) +
            SIM_power_gatecap( nsize + psize, 20 );

  return Ctotal;
}


static int SIM_array_wordline_clear_stat(power_wordline *wordline)
{
  wordline->n_read = wordline->n_write = 0;

  return 0;
}


/*#
 * initialize wordline
 *
 * Parameters:
 *   wordline -- wordline structure
 *   model    -- wordline model type
 *   share_rw -- 1 if shared R/W wordlines, 0 if separate R/W wordlines
 *   cols     -- # of array columns, NOT # of bitlines
 *   wire_cap -- wordline wire capacitance
 *   end      -- end of bitlines
 *
 * Return value: -1 if invalid model type
 *               0 otherwise
 *
 * Side effects:
 *   initialize wordline structure if model type is valid
 *
 * TODO: add error handler
 */
static int SIM_array_wordline_init(power_wordline *wordline, int model, int share_rw, unsigned cols, double wire_cap, unsigned end )
{
  if ((wordline->model = model) && model < WORDLINE_MAX_MODEL) {
    SIM_array_wordline_clear_stat(wordline);

    switch ( model ) {
      case CAM_RW_WORDLINE:
           wordline->e_read = SIM_cam_wordline_cap( cols * end, wire_cap, Wmemcellr ) * EnergyFactor;
           if ( wordline->share_rw = share_rw )
             wordline->e_write = wordline->e_read;
           else
             /* write bitlines are always double-ended */
             wordline->e_write = SIM_cam_wordline_cap( cols * 2, wire_cap, Wmemcellw ) * EnergyFactor;
           break;

      case CAM_WO_WORDLINE:     /* only have write wordlines */
           wordline->share_rw = 0;
           wordline->e_read = 0;
           wordline->e_write = SIM_cam_wordline_cap( cols * 2, wire_cap, Wmemcellw ) * EnergyFactor;
           break;

      case CACHE_WO_WORDLINE:   /* only have write wordlines */
           wordline->share_rw = 0;
           wordline->e_read = 0;
           wordline->e_write = SIM_array_wordline_cap( cols * 2, wire_cap, Wmemcellw ) * EnergyFactor;
           break;

      case CACHE_RW_WORDLINE:
           wordline->e_read = SIM_array_wordline_cap( cols * end, wire_cap, Wmemcellr ) * EnergyFactor;
           if ( wordline->share_rw = share_rw )
             wordline->e_write = wordline->e_read;
           else
             wordline->e_write = SIM_array_wordline_cap( cols * 2, wire_cap, Wmemcellw ) * EnergyFactor;

           /* static power */
           /* input driver */
           wordline->i_leakage = (Woutdrivern * NMOS_TAB[0] + Woutdriverp * PMOS_TAB[0]) / PARM_TECH_POINT * 100;
           break;

      default:  break;/* some error handler */
    }

    return 0;
  }
  else
    return -1;
}


/*#
 * record wordline power stats
 *
 * Parameters:
 *   wordline -- wordline structure
 *   rw       -- 1 if write operation, 0 if read operation
 *   n_switch -- switching times
 *
 * Return value: 0
 *
 * Side effects:
 *   update counters of wordline structure
 */
int SIM_array_wordline_record(power_wordline *wordline, int rw, unsigned long int n_switch )
{
  if ( rw ) wordline->n_write += n_switch;
  else wordline->n_read += n_switch;

  return 0;
}


/*#
 * report wordline power stats
 *
 * Parameters:
 *   wordline -- wordline structure
 *
 * Return value: total energy consumption of all wordlines of this array
 *
 * TODO: add more report functionality, currently only total energy is reported
 */
double SIM_array_wordline_report(power_wordline *wordline )
{
  return ( wordline->n_read * wordline->e_read +
           wordline->n_write * wordline->e_write );
}

// ------- Wordlines end



// ------- Bitlines begin

/*#
 * compute switching cap of reading 1 separate bitline column
 *
 * Parameters:
 *   rows          -- # of array rows, i.e. # of wordlines
 *   wire_cap      -- bitline wire capacitance
 *   end           -- end of bitlines
 *   n_share_amp   -- # of columns who share one sense amp
 *   n_bitline_pre -- # of precharge transistor drains for 1 bitline column
 *   n_colsel_pre  -- # of precharge transistor drains for 1 column selector, if any
 *   pre_size      -- width of precharge transistors
 *   outdrv_model  -- output driver model type
 *
 * Return value: switching cap
 *
 * NOTES: one bitline 1->0, then 0->1 on next precharging, so no 1/2
 */
static double SIM_array_column_read_cap(unsigned rows, double wire_cap, unsigned end, unsigned n_share_amp, unsigned n_bitline_pre, unsigned n_colsel_pre, double pre_size, int outdrv_model)
{
  double Ctotal=0, Cprecharge=0, Cpass=0, Cwire=0, Ccol_sel=0, Csense=0;

  /* part 1: drain cap of precharge tx's */
  Cprecharge = n_bitline_pre * SIM_power_draincap( pre_size, PCH, 1 );
//  printf("Precharge = %g\n", Cprecharge);
  Ctotal = Cprecharge;

  /* part 2: drain cap of pass tx's */
  Cpass = rows * SIM_power_draincap( Wmemcellr, NCH, 1 );
//  printf("Pass = %g\n", Cpass);
  Ctotal += Cpass;

  /* part 3: metal cap */
  Cwire = wire_cap;
//  printf("Wire = %g\n", Cwire);
  Ctotal += Cwire;

  /* part 4: column selector or bitline inverter */
  if ( end == 1 ) {             /* bitline inverter */
    /* FIXME: magic numbers */
    Ccol_sel = SIM_power_gatecap( MSCALE * ( 29.9 + 7.8 ), 0 ) +
              SIM_power_gatecap( MSCALE * ( 47.0 + 12.0), 0 );
  }
  else if ( n_share_amp > 1 ) { /* column selector */
    /* drain cap of pass tx's */
    Ccol_sel = ( n_share_amp + 1 ) * SIM_power_draincap( Wbitmuxn, NCH, 1 );
    /* drain cap of column selector precharge tx's */
    Ccol_sel += n_colsel_pre * SIM_power_draincap( pre_size, PCH, 1 );
    /* FIXME: no way to count activity factor on gates of column selector */
  }
//  printf("Col selector = %g\n", Ccol_sel);

  Ctotal += Ccol_sel;

  /* part 5: gate cap of sense amplifier or output driver */
  if (end == 2)                 /* sense amplifier */
    Csense = 2 * SIM_power_gatecap( WsenseQ1to4, 10 );
  else if (outdrv_model)        /* end == 1, output driver */
    Csense = SIM_power_gatecap( Woutdrvnandn, 1 ) + SIM_power_gatecap( Woutdrvnandp, 1 ) +
              SIM_power_gatecap( Woutdrvnorn, 1 ) + SIM_power_gatecap( Woutdrvnorp, 1 );
//  printf("Sense = %g\n", Csense);
  Ctotal += Csense;

  return Ctotal;
}


/*#
 * compute switching cap of selecting 1 column selector
 *
 * Parameters:
 *
 * Return value: switching cap
 *
 * NOTES: select one, deselect another, so no 1/2
 */
static double SIM_array_column_select_cap( void )
{
  return SIM_power_gatecap( Wbitmuxn, 1 );
}


/*#
 * compute switching cap of writing 1 separate bitline column
 *
 * Parameters:
 *   rows          -- # of array rows, i.e. # of wordlines
 *   wire_cap      -- bitline wire capacitance
 *
 * Return value: switching cap
 *
 * NOTES: bit and bitbar switch simultaneously, so no 1/2
 */
static double SIM_array_column_write_cap( unsigned rows, double wire_cap )
{
  double Ctotal=0, Cwire=0, Cpass=0, Cdriver=0, psize, nsize;

  Cwire = wire_cap;
//  printf("WRITE wire cap = %g\n", Cwire);
  Ctotal = Cwire;

  /* part 1: line cap, including drain cap of pass tx's and metal cap */
  Cpass = rows * SIM_power_draincap( Wmemcellw, NCH, 1 );
//  printf("WRITE pass tx cap = %g\n", Cpass);
  Ctotal += Cpass;


  /* part 2: write driver */
  psize = SIM_power_driver_size( Ctotal, Period / 8 );
  nsize = psize * Wdecinvn / Wdecinvp;
  Cdriver = SIM_power_draincap( psize, PCH, 1 ) + SIM_power_draincap( nsize, NCH, 1 ) +
            SIM_power_gatecap( psize + nsize, 1 );
//  printf("WRITE driver cap = %g\n", Cdriver);
  Ctotal += Cdriver;

  return Ctotal;
}


/* one bitline switches twice in one cycle, so no 1/2 */
static double SIM_array_share_column_write_cap( unsigned rows, double wire_cap, unsigned n_share_amp, unsigned n_bitline_pre, double pre_size )
{
  double Ctotal, psize, nsize;

  /* part 1: drain cap of precharge tx's */
  Ctotal = n_bitline_pre * SIM_power_draincap( pre_size, PCH, 1 );

  /* part 2: drain cap of pass tx's */
  Ctotal += rows * SIM_power_draincap( Wmemcellr, NCH, 1 );

  /* part 3: metal cap */
  Ctotal += wire_cap;

  /* part 4: column selector or sense amplifier */
  if ( n_share_amp > 1 ) Ctotal += SIM_power_draincap( Wbitmuxn, NCH, 1 );
  else Ctotal += 2 * SIM_power_gatecap( WsenseQ1to4, 10 );

  /* part 5: write driver */
  psize = SIM_power_driver_size( Ctotal, Period / 8 );
  nsize = psize * Wdecinvn / Wdecinvp;
  /* WHS: omit gate cap of driver due to modeling difficulty */
  Ctotal += SIM_power_draincap( psize, PCH, 1 ) + SIM_power_draincap( nsize, NCH, 1 );

  return Ctotal;
}


/* one bitline switches twice in one cycle, so no 1/2 */
static double SIM_array_share_column_read_cap( unsigned rows, double wire_cap, unsigned n_share_amp, unsigned n_bitline_pre, unsigned n_colsel_pre, double pre_size )
{
  double Ctotal;

  /* part 1: same portion as write */
  Ctotal = SIM_array_share_column_write_cap( rows, wire_cap, n_share_amp, n_bitline_pre, pre_size );

  /* part 2: column selector and sense amplifier */
  if ( n_share_amp > 1 ) {
    /* bottom part of drain cap of pass tx's */
    Ctotal += n_share_amp * SIM_power_draincap( Wbitmuxn, NCH, 1 );
    /* drain cap of column selector precharge tx's */
    Ctotal += n_colsel_pre * SIM_power_draincap( pre_size, PCH, 1 );

    /* part 3: gate cap of sense amplifier */
    Ctotal += 2 * SIM_power_gatecap( WsenseQ1to4, 10 );
  }

  return Ctotal;
}


static int SIM_array_bitline_clear_stat(power_bitline *bitline)
{
  bitline->n_col_write = bitline->n_col_read = bitline->n_col_sel = 0;

  return 0;
}


static int SIM_array_bitline_init(power_bitline *bitline, int model, int share_rw, unsigned end, unsigned rows, double wire_cap, unsigned n_share_amp, unsigned n_bitline_pre, unsigned n_colsel_pre, double pre_size, int outdrv_model)
{
  if ((bitline->model = model) && model < BITLINE_MAX_MODEL) {
    bitline->end = end;
    SIM_array_bitline_clear_stat(bitline);

    switch ( model ) {
      case RW_BITLINE:
           if ( end == 2 )
             bitline->e_col_sel = SIM_array_column_select_cap() * EnergyFactor;
           else         /* end == 1 implies register file */
             bitline->e_col_sel = 0;
//         printf("BUFFER INTERNAL bitline sel energy = %g\n", bitline->e_col_sel);

           if ( bitline->share_rw = share_rw ) {
             /* shared bitlines are double-ended, so SenseEnergyFactor */
             bitline->e_col_read = SIM_array_share_column_read_cap( rows, wire_cap, n_share_amp, n_bitline_pre, n_colsel_pre, pre_size ) * SenseEnergyFactor;
             bitline->e_col_write = SIM_array_share_column_write_cap( rows, wire_cap, n_share_amp, n_bitline_pre, pre_size ) * EnergyFactor;
           }
           else {
             bitline->e_col_read = SIM_array_column_read_cap(rows, wire_cap, end, n_share_amp, n_bitline_pre, n_colsel_pre, pre_size, outdrv_model) * (end == 2 ? SenseEnergyFactor : EnergyFactor);
//               printf("BUFFER INTERNAL bitline read energy = %g\n", bitline->e_col_read);
             bitline->e_col_write = SIM_array_column_write_cap( rows, wire_cap ) * EnergyFactor;
//               printf("BUFFER INTERNAL bitline write energy = %g\n", bitline->e_col_write);

             /* static power */
             bitline->i_leakage = 2 * (Wdecinvn * NMOS_TAB[0] + Wdecinvp * PMOS_TAB[0]) / PARM_TECH_POINT * 100;
//               printf("BUFFER INTERNAL bitline leakage current = %g\n", bitline->i_leakage);
           }

           break;

      case WO_BITLINE:  /* only have write bitlines */
           bitline->share_rw = 0;
           bitline->e_col_sel = bitline->e_col_read = 0;
           bitline->e_col_write = SIM_array_column_write_cap( rows, wire_cap ) * EnergyFactor;
           break;

      default:  break;/* some error handler */
    }

    return 0;
  }
  else
    return -1;
}


static int is_rw_bitline( int model )
{
  return ( model == RW_BITLINE );
}


/* WHS: no way to count activity factor on column selector gates */
int SIM_array_bitline_record(power_bitline *bitline, int rw, unsigned cols, unsigned long int old_value, unsigned long int new_value )
{
  /* FIXME: should use variable rather than computing each time */
  unsigned long int mask = HAMM_MASK(cols);

  if ( rw ) {   /* write */
    if ( bitline->share_rw )    /* share R/W bitlines */
      bitline->n_col_write += cols;
    else                        /* separate R/W bitlines */
      bitline->n_col_write += SIM_power_Hamming( old_value, new_value, mask );
  }
  else {        /* read */
    if ( bitline->end == 2 )    /* double-ended bitline */
      bitline->n_col_read += cols;
    else                        /* single-ended bitline */
      /* WHS: read ~new_value due to the bitline inverter */
      bitline->n_col_read += SIM_power_Hamming( mask, ~new_value, mask );
  }

  return 0;
}


double SIM_array_bitline_report(power_bitline *bitline )
{
  return ( bitline->n_col_write * bitline->e_col_write +
           bitline->n_col_read * bitline->e_col_read +
           bitline->n_col_sel * bitline->e_col_sel );
}

// ------- Bitlines end



// ------- Sense amplifier begin

/* estimate senseamp power dissipation in cache structures (Zyuban's method) */
static double SIM_array_amp_energy( void )
{
  return ( (double)Vdd / 8.0 * (double )(Period) * (double )(PARM_amp_Idsat));
}


static int SIM_array_amp_clear_stat(power_amp *amp)
{
  amp->n_access = 0;

  return 0;
}


static int SIM_array_amp_init(power_amp *amp, int model )
{
  if ((amp->model = model) && model < AMP_MAX_MODEL) {
    SIM_array_amp_clear_stat(amp);
    amp->e_access = SIM_array_amp_energy();

    return 0;
  }
  else
    return -1;
}


int SIM_array_amp_record(power_amp *amp, unsigned cols )
{
  amp->n_access += cols;

  return 0;
}


double SIM_array_amp_report(power_amp *amp )
{
  return ( amp->n_access * amp->e_access );
}

// ------- Sense amplifier end


// ------- Tag comparator begin

/* eval switches twice per cycle, so no 1/2 */
/* WHS: assume eval = 1 when no cache operation */
static double SIM_array_comp_base_cap( void )
{
  /* eval tx's: 4 inverters */
  return ( SIM_power_draincap( Wevalinvp, PCH, 1 ) + SIM_power_draincap( Wevalinvn, NCH, 1 ) +
           SIM_power_gatecap( Wevalinvp, 1 ) + SIM_power_gatecap( Wevalinvn, 1 ) +
           SIM_power_draincap( Wcompinvp1, PCH, 1 ) + SIM_power_draincap( Wcompinvn1, NCH, 1 ) +
           SIM_power_gatecap( Wcompinvp1, 1 ) + SIM_power_gatecap( Wcompinvn1, 1 ) +
           SIM_power_draincap( Wcompinvp2, PCH, 1 ) + SIM_power_draincap( Wcompinvn2, NCH, 1 ) +
           SIM_power_gatecap( Wcompinvp2, 1 ) + SIM_power_gatecap( Wcompinvn2, 1 ) +
           SIM_power_draincap( Wcompinvp3, PCH, 1 ) + SIM_power_draincap( Wcompinvn3, NCH, 1 ) +
           SIM_power_gatecap( Wcompinvp3, 1 ) + SIM_power_gatecap( Wcompinvn3, 1 ));
}


/* no 1/2 for the same reason with SIM_array_comp_base_cap */
static double SIM_array_comp_match_cap( unsigned n_bits )
{
  return ( n_bits * ( SIM_power_draincap( Wcompn, NCH, 1 ) + SIM_power_draincap( Wcompn, NCH, 2 )));
}


/* upon mismatch, select signal 1->0, then 0->1 on next precharging, so no 1/2 */
static double SIM_array_comp_mismatch_cap( unsigned n_pre )
{
  double Ctotal;

  /* part 1: drain cap of precharge tx */
  Ctotal = n_pre * SIM_power_draincap( Wcomppreequ, PCH, 1 );

  /* part 2: nor gate of valid output */
  Ctotal += SIM_power_gatecap( WdecNORn, 1 ) + SIM_power_gatecap( WdecNORp, 3 );

  return Ctotal;
}


/* upon miss, valid output switches twice in one cycle, so no 1/2 */
static double SIM_array_comp_miss_cap( unsigned assoc )
{
  /* drain cap of valid output */
  return ( assoc * SIM_power_draincap( WdecNORn, NCH, 1 ) + SIM_power_draincap( WdecNORp, PCH, assoc ));
}


/* no 1/2 for the same reason as base_cap */
static double SIM_array_comp_bit_match_cap( void )
{
  return ( 2 * ( SIM_power_draincap( Wcompn, NCH, 1 ) + SIM_power_draincap( Wcompn, NCH, 2 )));
}


/* no 1/2 for the same reason as base_cap */
static double SIM_array_comp_bit_mismatch_cap( void )
{
  return ( 3 * SIM_power_draincap( Wcompn, NCH, 1 ) + SIM_power_draincap( Wcompn, NCH, 2 ));
}


/* each addr bit drives 2 nmos pass transistors, so no 1/2 */
static double SIM_array_comp_chgaddr_cap( void )
{
  return ( SIM_power_gatecap( Wcompn, 1 ));
}


static int SIM_array_comp_clear_stat(power_comp *comp)
{
  comp->n_access = comp->n_miss = comp->n_chg_addr = comp->n_match = 0;
  comp->n_mismatch = comp->n_bit_match = comp->n_bit_mismatch = 0;

  return 0;
}


static int SIM_array_comp_init(power_comp *comp, int model, unsigned n_bits, unsigned assoc, unsigned n_pre, double matchline_len, double tagline_len )
{
  if ((comp->model = model) && model < COMP_MAX_MODEL) {
    comp->n_bits = n_bits;
    comp->assoc = assoc;
    /* redundant field */
    comp->comp_mask = HAMM_MASK(n_bits);

    SIM_array_comp_clear_stat(comp);

    switch ( model ) {
      case CACHE_COMPONENT:
           comp->e_access = SIM_array_comp_base_cap() * EnergyFactor;
           comp->e_match = SIM_array_comp_match_cap( n_bits ) * EnergyFactor;
           comp->e_mismatch = SIM_array_comp_mismatch_cap( n_pre ) * EnergyFactor;
           comp->e_miss = SIM_array_comp_miss_cap( assoc ) * EnergyFactor;
           comp->e_bit_match = SIM_array_comp_bit_match_cap() * EnergyFactor;
           comp->e_bit_mismatch = SIM_array_comp_bit_mismatch_cap() * EnergyFactor;
           comp->e_chg_addr = SIM_array_comp_chgaddr_cap() * EnergyFactor;
           break;

      case CAM_COMP:
           comp->e_access = comp->e_match = comp->e_chg_addr = 0;
           comp->e_bit_match = comp->e_bit_mismatch = 0;
           /* energy consumption of tagline */
           comp->e_chg_addr = SIM_cam_comp_tagline_cap( assoc, tagline_len ) * EnergyFactor;
           comp->e_mismatch = SIM_cam_comp_mismatch_cap( n_bits, n_pre, matchline_len ) * EnergyFactor;
           comp->e_miss = SIM_cam_comp_miss_cap( assoc ) * EnergyFactor;
           break;

      default:  break;/* some error handler */
    }

    return 0;
  }
  else
    return -1;
}


int SIM_array_comp_global_record(power_comp *comp, unsigned long int prev_value, unsigned long int curr_value, int miss )
{
  if ( miss ) comp->n_miss ++;

  switch ( comp->model ) {
    case CACHE_COMPONENT:
         comp->n_access ++;
         comp->n_chg_addr += SIM_power_Hamming( prev_value, curr_value, comp->comp_mask ) * comp->assoc;
         break;

    case CAM_COMP:
         comp->n_chg_addr += SIM_power_Hamming( prev_value, curr_value, comp->comp_mask );
         break;

    default:    break;/* some error handler */
  }

  return 0;
}


/* recover means prev_tag will recover on next cycle, e.g. driven by sense amplifier */
/* return value: 1 if miss, 0 if hit */
int SIM_array_comp_local_record(power_comp *comp, unsigned long int prev_tag, unsigned long int curr_tag, unsigned long int input, int recover )
{
  unsigned H_dist;
  int mismatch;

  if ( mismatch = ( curr_tag != input )) comp->n_mismatch ++;

  /* for cam, input changes are reflected in memory cells */
  if ( comp->model == CACHE_COMPONENT ) {
    if ( recover )
      comp->n_chg_addr += 2 * SIM_power_Hamming( prev_tag, curr_tag, comp->comp_mask );
    else
      comp->n_chg_addr += SIM_power_Hamming( prev_tag, curr_tag, comp->comp_mask );

    if ( mismatch ) {
      H_dist = SIM_power_Hamming( curr_tag, input, comp->comp_mask );
      comp->n_bit_mismatch += H_dist;
      comp->n_bit_match += comp->n_bits - H_dist;
    }
    else comp->n_match ++;
  }

  return mismatch;
}


double SIM_array_comp_report(power_comp *comp )
{
  return ( comp->n_access * comp->e_access + comp->n_match * comp->e_match +
           comp->n_mismatch * comp->e_mismatch + comp->n_miss * comp->e_miss +
           comp->n_bit_match * comp->e_bit_match + comp->n_chg_addr * comp->e_chg_addr +
           comp->n_bit_mismatch * comp->e_bit_mismatch );
}

// ------- Tag comparator end



// ------- Multiplexor begin

/* upon mismatch, 1 output of nor gates 1->0, then 0->1 on next cycle, so no 1/2 */
static double SIM_array_mux_mismatch_cap( unsigned n_nor_gates )
{
  double Cmul;

  /* stage 1: inverter */
  Cmul = SIM_power_draincap( Wmuxdrv12n, NCH, 1 ) + SIM_power_draincap( Wmuxdrv12p, PCH, 1 ) +
         SIM_power_gatecap( Wmuxdrv12n, 1 ) + SIM_power_gatecap( Wmuxdrv12p, 1 );

  /* stage 2: nor gates */
  /* gate cap of nor gates */
  Cmul += n_nor_gates * ( SIM_power_gatecap( WmuxdrvNORn, 1 ) + SIM_power_gatecap( WmuxdrvNORp, 1 ));
  /* drain cap of nor gates, only count 1 */
  Cmul += SIM_power_draincap( WmuxdrvNORp, PCH, 2 ) + 2 * SIM_power_draincap( WmuxdrvNORn, NCH, 1 );

  /* stage 3: output inverter */
  Cmul += SIM_power_gatecap( Wmuxdrv3n, 1 ) + SIM_power_gatecap( Wmuxdrv3p, 1 ) +
          SIM_power_draincap( Wmuxdrv3n, NCH, 1 ) + SIM_power_draincap( Wmuxdrv3p, PCH, 1 );

  return Cmul;
}


/* 2 nor gates switch gate signals, so no 1/2 */
/* WHS: assume address changes won't propagate until matched or mismatched */
static double SIM_array_mux_chgaddr_cap( void )
{
  return ( SIM_power_gatecap( WmuxdrvNORn, 1 ) + SIM_power_gatecap( WmuxdrvNORp, 1 ));
}


static int SIM_array_mux_clear_stat(power_mux *mux)
{
  mux->n_mismatch = mux->n_chg_addr = 0;

  return 0;
}


static int SIM_array_mux_init(power_mux *mux, int model, unsigned n_gates, unsigned assoc )
{
  if ((mux->model = model) && model < MUX_MAX_MODEL) {
    mux->assoc = assoc;

    SIM_array_mux_clear_stat(mux);

    mux->e_mismatch = SIM_array_mux_mismatch_cap( n_gates ) * EnergyFactor;
    mux->e_chg_addr = SIM_array_mux_chgaddr_cap() * EnergyFactor;

    return 0;
  }
  else
    return -1;
}


int SIM_array_mux_record(power_mux *mux, unsigned long int prev_addr, unsigned long int curr_addr, int miss )
{
  if ( prev_addr != curr_addr )
    mux->n_chg_addr += mux->assoc;

  if ( miss )
    mux->n_mismatch += mux->assoc;
  else
    mux->n_mismatch += mux->assoc - 1;

  return 0;
}


double SIM_array_mux_report(power_mux *mux )
{
  return ( mux->n_mismatch * mux->e_mismatch + mux->n_chg_addr * mux->e_chg_addr );
}

// ------- Multiplexor end


// ------- Output driver begin

/* output driver should be disabled somehow when no access occurs, so no 1/2 */
static double SIM_array_outdrv_select_cap( unsigned data_width )
{
  double Ctotal;

  /* stage 1: inverter */
  Ctotal = SIM_power_gatecap( Woutdrvseln, 1 ) + SIM_power_gatecap( Woutdrvselp, 1 ) +
           SIM_power_draincap( Woutdrvseln, NCH, 1 ) + SIM_power_draincap( Woutdrvselp, PCH, 1 );

  /* stage 2: gate cap of nand gate and nor gate */
  /* only consider 1 gate cap because another and drain cap switch depends on data value */
  Ctotal += data_width *( SIM_power_gatecap( Woutdrvnandn, 1 ) + SIM_power_gatecap( Woutdrvnandp, 1 ) +
                          SIM_power_gatecap( Woutdrvnorn, 1 ) + SIM_power_gatecap( Woutdrvnorp, 1 ));

  return Ctotal;
}


/* WHS: assume data changes won't propagate until enabled */
static double SIM_array_outdrv_chgdata_cap( void )
{
  return (( SIM_power_gatecap( Woutdrvnandn, 1 ) + SIM_power_gatecap( Woutdrvnandp, 1 ) +
            SIM_power_gatecap( Woutdrvnorn, 1 ) + SIM_power_gatecap( Woutdrvnorp, 1 )) / 2 );
}


/* no 1/2 for the same reason as outdrv_select_cap */
static double SIM_array_outdrv_outdata_cap( unsigned value )
{
  double Ctotal;

  /* stage 1: drain cap of nand gate or nor gate */
  if ( value )
    /* drain cap of nand gate */
    Ctotal = SIM_power_draincap( Woutdrvnandn, NCH, 2 ) + 2 * SIM_power_draincap( Woutdrvnandp, PCH, 1 );
  else
    /* drain cap of nor gate */
    Ctotal = 2 * SIM_power_draincap( Woutdrvnorn, NCH, 1 ) + SIM_power_draincap( Woutdrvnorp, PCH, 2 );

  /* stage 2: gate cap of output inverter */
  if ( value )
    Ctotal += SIM_power_gatecap( Woutdriverp, 1 );
  else
    Ctotal += SIM_power_gatecap( Woutdrivern, 1 );

  /* drain cap of output inverter should be included into bus cap */
  return Ctotal;
}


static int SIM_array_outdrv_clear_stat(power_out *outdrv)
{
  outdrv->n_select = outdrv->n_chg_data = 0;
  outdrv->n_out_0 = outdrv->n_out_1 = 0;

  return 0;
}


static int SIM_array_outdrv_init(power_out *outdrv, int model, unsigned item_width )
{
  if ((outdrv->model = model) && model < OUTDRV_MAX_MODEL) {
    outdrv->item_width = item_width;
    /* redundant field */
    outdrv->out_mask = HAMM_MASK(item_width);

    SIM_array_outdrv_clear_stat(outdrv);

    outdrv->e_select = SIM_array_outdrv_select_cap( item_width ) * EnergyFactor;
    outdrv->e_out_1 = SIM_array_outdrv_outdata_cap( 1 ) * EnergyFactor;
    outdrv->e_out_0 = SIM_array_outdrv_outdata_cap( 0 ) * EnergyFactor;

    switch ( model ) {
      case CACHE_OUTDRV:
           outdrv->e_chg_data = SIM_array_outdrv_chgdata_cap() * EnergyFactor;
           break;

      case CAM_OUTDRV:
           /* input changes are reflected in memory cells */
      case REG_OUTDRV:
           /* input changes are reflected in bitlines */
           outdrv->e_chg_data = 0;
           break;

      default:  break;/* some error handler */
    }

    return 0;
  }
  else
    return -1;
}


int SIM_array_outdrv_global_record(power_out *outdrv, unsigned long int data )
{
  unsigned n_1;

  outdrv->n_select ++;

  n_1 = SIM_power_Hamming( data, 0, outdrv->out_mask );

  outdrv->n_out_1 += n_1;
  outdrv->n_out_0 += outdrv->item_width - n_1;

  return 0;
}


/* recover means prev_data will recover on next cycle, e.g. driven by sense amplifier */
/* NOTE: this function SHOULD not be called by a fully-associative cache */
int SIM_array_outdrv_local_record(power_out *outdrv, unsigned long int prev_data, unsigned long int curr_data, int recover )
{
  if ( recover )
    outdrv->n_chg_data += 2 * SIM_power_Hamming( prev_data, curr_data, outdrv->out_mask );
  else
    outdrv->n_chg_data += SIM_power_Hamming( prev_data, curr_data, outdrv->out_mask );

  return 0;
}


double SIM_array_outdrv_report(power_out *outdrv )
{
  return ( outdrv->n_select * outdrv->e_select + outdrv->n_chg_data * outdrv->e_chg_data +
           outdrv->n_out_1 * outdrv->e_out_1 + outdrv->n_out_0 * outdrv->e_out_0 );
}

// ------- Output driver end



// ------- Memcory cell begin

/* WHS: use Wmemcella and Wmemcellbscale to compute tx width of memory cell */
static double SIM_array_mem_cap( unsigned read_ports, unsigned write_ports, int share_rw, unsigned end )
{
  double Ctotal;

  /* part 1: drain capacitance of pass transistors */
  Ctotal = SIM_power_draincap( Wmemcellr, NCH, 1 ) * read_ports * end / 2;
  if ( ! share_rw )
    Ctotal += SIM_power_draincap( Wmemcellw, NCH, 1 ) * write_ports;

  /* has coefficient ( 1/2 * 2 ) */
  /* part 2: drain capacitance of memory cell */
  Ctotal += SIM_power_draincap( Wmemcella, NCH, 1 ) + SIM_power_draincap( Wmemcella * Wmemcellbscale, PCH, 1 );

  /* has coefficient ( 1/2 * 2 ) */
  /* part 3: gate capacitance of memory cell */
  Ctotal += SIM_power_gatecap( Wmemcella, 1 ) + SIM_power_gatecap( Wmemcella * Wmemcellbscale, 1 );

  return Ctotal;
}


static int SIM_array_mem_clear_stat(power_mem *mem)
{
  mem->n_switch = 0;

  return 0;
}


static int SIM_array_mem_init(power_mem *mem, int model, unsigned read_ports, unsigned write_ports, int share_rw, unsigned end )
{
  double i_leakage;

  if ((mem->model = model) && model < MEM_MAX_MODEL) {
    mem->end = end;
    SIM_array_mem_clear_stat(mem);

    switch ( model ) {
      case CAM_TAG_RW_MEM:
           mem->e_switch = SIM_cam_tag_mem_cap( read_ports, write_ports, share_rw, end, SIM_ARRAY_RW ) * EnergyFactor;
           break;

      /* FIXME: it's only an approximation using CAM_TAG_WO_MEM to emulate CAM_ATTACH_MEM */
      case CAM_ATTACH_MEM:
      case CAM_TAG_WO_MEM:
           mem->e_switch = SIM_cam_tag_mem_cap( read_ports, write_ports, share_rw, end, SIM_ARRAY_WO ) * EnergyFactor;
           break;

      case CAM_DATA_MEM:
           mem->e_switch = SIM_cam_data_mem_cap( read_ports, write_ports ) * EnergyFactor;
           break;

      default:  /* NORMAL_MEM */
           mem->e_switch = SIM_array_mem_cap( read_ports, write_ports, share_rw, end ) * EnergyFactor;

           /* static power */
           i_leakage = 0;
           /* memory cell */
           i_leakage += (Wmemcella * NMOS_TAB[0] + Wmemcella * Wmemcellbscale * PMOS_TAB[0]) * 2;
           /* read port pass tx */
           i_leakage += Wmemcellr * NMOS_TAB[0] * end * read_ports;
           /* write port pass tx */
           if (! share_rw)
             i_leakage += Wmemcellw * NMOS_TAB[0] * 2 * write_ports;

           mem->i_leakage = i_leakage / PARM_TECH_POINT * 100;
    }

    return 0;
  }
  else
    return -1;
}


int SIM_array_mem_record(power_mem *mem, unsigned long int prev_value, unsigned long int curr_value, unsigned width )
{
  mem->n_switch += SIM_power_Hamming( prev_value, curr_value, HAMM_MASK(width));

  return 0;
}


double SIM_array_mem_report(power_mem *mem )
{
  return ( mem->n_switch * mem->e_switch );
}

// ------- Memcory cell end



// ------- Precharge begin

/* consider charge then discharge, so no 1/2 */
static double SIM_array_pre_cap( double width, double length )
{
  return SIM_power_gatecap( width, length );
}


/* return # of precharging gates per column */
static unsigned n_pre_gate( int model )
{
  switch ( model ) {
    case SINGLE_BITLINE:        return 2;
    case EQU_BITLINE:           return 3;
    case SINGLE_OTHER:          return 1;
    default:    break;/* some error handler */
  }

  return 0;
}


/* return # of precharging drains per line */
static unsigned n_pre_drain( int model )
{
  switch ( model ) {
    case SINGLE_BITLINE:        return 1;
    case EQU_BITLINE:           return 2;
    case SINGLE_OTHER:          return 1;
    default:    break;/* some error handler */
  }

  return 0;
}


static int SIM_array_pre_clear_stat(power_arr_pre *pre)
{
  pre->n_charge = 0;

  return 0;
}


static int SIM_array_pre_init(power_arr_pre *pre, int model, double pre_size )
{
  unsigned n_gate;

  n_gate = n_pre_gate(model);

  if ((pre->model = model) && model < PRE_MAX_MODEL) {
    SIM_array_pre_clear_stat(pre);

    /* WHS: 10 should go to PARM */
    pre->e_charge = SIM_array_pre_cap( pre_size, 10 ) * n_gate * EnergyFactor;

    /* static power */
    pre->i_leakage = n_gate * pre_size * PMOS_TAB[0] / PARM_TECH_POINT * 100;

    return 0;
  }
  else
    return -1;
}


int SIM_array_pre_record(power_arr_pre *pre, unsigned long int n_charge )
{
  pre->n_charge += n_charge;

  return 0;
}


double SIM_array_pre_report(power_arr_pre *pre )
{
  return ( pre->n_charge * pre->e_charge );
}

// ------- Precharge end

/* ---------- buffer model end ------------ */








/***** from SIM_array_internal_m.c *********/


/* for now we simply initialize all fields to 0, which should not
 * add too much error if the program runtime is long enough :) */
int SIM_array_port_state_init(power_array_info *info, SIM_array_port_state_t *port )
{
  //if ( IS_FULLY_ASSOC( info ) || !(info->share_rw))
    //bzero( port->data_line, port->data_line_size );

  port->tag_line = 0;
  port->row_addr = 0;
  port->col_addr = 0;
  port->tag_addr = 0;

  return 0;
}


int SIM_array_set_state_init( power_array_info *info, SIM_array_set_state_t *set )
{
  set->entry = NULL;
  set->entry_set = NULL;

  if ( IS_FULLY_ASSOC( info )) {
    set->write_flag = 0;
    set->write_back_flag = 0;
  }

  /* no default value for other fields */
  return 0;
}


/* record row decoder and wordline activity */
/* only used by non-fully-associative array, but we check it anyway */
int SIM_power_array_dec( power_array_info *info, power_array *arr, SIM_array_port_state_t *port, unsigned long int row_addr, int rw )
{
  if ( ! IS_FULLY_ASSOC( info )) {
    /* record row decoder stats */
    if (info->row_dec_model) {
      SIM_array_dec_record( &arr->row_dec, port->row_addr, row_addr );

      /* update state */
      port->row_addr = row_addr;
    }

    /* record wordline stats */
    SIM_array_wordline_record( &arr->data_wordline, rw, info->data_ndwl );
    if ( HAVE_TAG( info ))
      SIM_array_wordline_record( &arr->tag_wordline, rw, info->tag_ndwl );

    return 0;
  }
  else
    return -1;
}


/* record read data activity (including bitline and sense amplifier) */
/* only used by non-fully-associative array, but we check it anyway */
/* data only used by RF array */
int SIM_power_array_data_read( power_array_info *info, power_array *arr, unsigned long int data )
{
  if (info->data_end == 1) {
    SIM_array_bitline_record( &arr->data_bitline, SIM_ARRAY_READ, info->eff_data_cols, 0, data );

    return 0;
  }
  else if ( ! IS_FULLY_ASSOC( info )) {
    SIM_array_bitline_record( &arr->data_bitline, SIM_ARRAY_READ, info->eff_data_cols, 0, 0 );
    SIM_array_amp_record( &arr->data_amp, info->eff_data_cols );

    return 0;
  }
  else
    return -1;
}


/* record write data bitline and memory cell activity */
/* assume no alignment restriction on write, so (char *) */
/* set only used by fully-associative array */
/* data_line only used by fully-associative or RF array */
int SIM_power_array_data_write( power_array_info *info, power_array *arr, SIM_array_set_state_t *set, unsigned n_item, char *data_line, char *old_data, char *new_data )
{
  unsigned i;

  /* record bitline stats */
  if ( IS_FULLY_ASSOC( info )) {
    /* wordline should be driven only once */
    if ( ! set->write_flag ) {
      SIM_array_wordline_record( &arr->data_wordline, SIM_ARRAY_WRITE, 1 );
      set->write_flag = 1;
    }

    /* for fully-associative array, data bank has no read
     * bitlines, so bitlines not written have no activity */
    for ( i = 0; i < n_item; i ++ ) {
      SIM_array_bitline_record( &arr->data_bitline, SIM_ARRAY_WRITE, 8, data_line[i], new_data[i] );
      /* update state */
      data_line[i] = new_data[i];
    }
  }
  else if (info->share_rw) {
    /* there is some subtlety here: write width may not be as wide as block size,
     * bitlines not written are actually read, but column selector should be off,
     * so read energy per bitline is the same as write energy per bitline */
    SIM_array_bitline_record( &arr->data_bitline, SIM_ARRAY_WRITE, info->eff_data_cols, 0, 0 );

    /* write in all sub-arrays if direct-mapped, which implies 1 cycle write latency,
     * in those sub-arrays wordlines are not driven, so only n items columns switch */
    if ( IS_DIRECT_MAP( info ) && info->data_ndbl > 1 )
        SIM_array_bitline_record( &arr->data_bitline, SIM_ARRAY_WRITE, n_item * 8 * ( info->data_ndbl - 1 ), 0, 0 );
  }
  else {        /* separate R/W bitlines */
    /* same arguments as in the previous case apply here, except that when we say
     * read_energy = write_energy, we omit the energy of write driver gate cap */
    for ( i = 0; i < n_item; i ++ ) {
      SIM_array_bitline_record( &arr->data_bitline, SIM_ARRAY_WRITE, 8, data_line[i], new_data[i] );
      /* update state */
      data_line[i] = new_data[i];
    }
  }

  /* record memory cell stats */
  for ( i = 0; i < n_item; i ++ )
    SIM_array_mem_record( &arr->data_mem, old_data[i], new_data[i], 8 );

  return 0;
}


/* record read tag activity (including bitline and sense amplifier) */
/* only used by non-RF array */
/* set only used by fully-associative array */
int SIM_power_array_tag_read( power_array_info *info, power_array *arr, SIM_array_set_state_t *set )
{
  if ( IS_FULLY_ASSOC( info )) {
    /* the only reason to read a fully-associative array tag is writing back */
    SIM_array_wordline_record( &arr->tag_wordline, SIM_ARRAY_READ, 1 );
    set->write_back_flag = 1;
  }

  SIM_array_bitline_record( &arr->tag_bitline, SIM_ARRAY_READ, info->eff_tag_cols, 0, 0 );
  SIM_array_amp_record( &arr->tag_amp, info->eff_tag_cols );

  return 0;
}


/* record write tag bitline and memory cell activity */
/* WHS: assume update of use bit, valid bit, dirty bit and tag will be coalesced */
/* only used by non-RF array */
/* port only used by fully-associative array */
//int SIM_power_array_tag_update( power_array_info *info, power_array *arr, SIM_array_port_state_t *port, SIM_array_set_state_t *set )
//{
  //unsigned i;
  //unsigned long int curr_tag;
  //power_mem *tag_attach_mem;

  /* get current tag */
  //if ( set->entry )
    //curr_tag = (*info->get_entry_tag)( set->entry );

 // if ( IS_FULLY_ASSOC( info ))
  //  tag_attach_mem = &arr->tag_attach_mem;
  //else
    //tag_attach_mem = &arr->tag_mem;

  /* record tag bitline stats */
  //if ( IS_FULLY_ASSOC( info )) {
   // if ( set->entry && curr_tag != set->tag_bak ) {
    //  /* shared wordline should be driven only once */
     // if ( ! set->write_back_flag )
      //  SIM_array_wordline_record( &arr->tag_wordline, SIM_ARRAY_WRITE, 1 );

      /* WHS: value of tag_line doesn't matter if not write_through */
      //SIM_array_bitline_record( &arr->tag_bitline, SIM_ARRAY_WRITE, info->eff_tag_cols, port->tag_line, curr_tag );
      /* update state */
      //if ( IS_WRITE_THROUGH( info ))
       // port->tag_line = curr_tag;
    //}
  //}
  //else {
    /* tag update cannot occur at the 1st cycle, so no other sub-arrays */
   // SIM_array_bitline_record( &arr->tag_bitline, SIM_ARRAY_WRITE, info->eff_tag_cols, 0, 0 );
  //}

  /* record tag memory cell stats */
  //if ( HAVE_USE_BIT( info ))
   // for ( i = 0; i < info->assoc; i ++ )
    //  SIM_array_mem_record( tag_attach_mem, set->use_bak[i], (*info->get_set_use_bit)( set->entry_set, i ), info->use_bit_width );

  //if ( set->entry ) {
   // SIM_array_mem_record( tag_attach_mem, set->valid_bak, (*info->get_entry_valid_bit)( set->entry ), info->valid_bit_width );
    //SIM_array_mem_record( &arr->tag_mem, set->tag_bak, curr_tag, info->tag_addr_width );

    //if ( IS_WRITE_BACK( info ))
     // SIM_array_mem_record( tag_attach_mem, set->dirty_bak, (*info->get_entry_dirty_bit)( set->entry ), 1 );
  //}

  //return 0;
//}


/* record tag compare activity (including tag comparator, column decoder and multiplexor) */
/* NOTE: this function may be called twice during ONE array operation, remember to update
 *       states at the end so that call to *_record won't add erroneous extra energy */
/* only used by non-RF array */
//int SIM_power_array_tag_compare( power_array_info *info, power_array *arr, SIM_array_port_state_t *port, unsigned long int tag_input, unsigned long int col_addr, SIM_array_set_state_t *set )
//{
  //int miss = 0;
  //unsigned i;

  /* record tag comparator stats */
  //for ( i = 0; i < info->assoc; i ++ ) {
    /* WHS: sense amplifiers output 0 when idle */
    //if ( SIM_array_comp_local_record( &arr->comp, 0, (*info->get_set_tag)( set->entry_set, i ), tag_input, SIM_ARRAY_RECOVER ))
    //  miss = 1;
  //}

  //SIM_array_comp_global_record( &arr->comp, port->tag_addr, tag_input, miss );

  /* record column decoder stats */
  //if ( HAVE_COL_DEC( info ))
    //SIM_array_dec_record( &arr->col_dec, port->col_addr, col_addr );

  /* record multiplexor stats */
  //if ( HAVE_COL_MUX( info ))
    //SIM_array_mux_record( &arr->mux, port->col_addr, col_addr, miss );

  /* update state */
  //port->tag_addr = tag_input;
  //if ( HAVE_COL_DEC( info ))
    //port->col_addr = col_addr;

  //return 0;
//}


/* record output driver activity */
/* assume alignment restriction on read, so specify data_size */
/* WHS: it's really a mess to use data_size to specify data type */
/* data_all only used by non-RF and non-fully-associative array */
/* WHS: don't support 128-bit or wider integer */
//int SIM_power_array_output( power_array_info *info, power_array *arr, unsigned data_size, unsigned length, void *data_out, void *data_all )
//{
 // unsigned i, j;

  /* record output driver stats */
  //for ( i = 0; i < length; i ++ ) {
   // switch ( data_size ) {
    //  case 1: SIM_array_outdrv_global_record( &arr->outdrv, ((unsigned8_t *)data_out)[i] );
         //     break;
      //case 2: SIM_array_outdrv_global_record( &arr->outdrv, ((unsigned16_t *)data_out)[i] );
           //   break;
      //case 4: SIM_array_outdrv_global_record( &arr->outdrv, ((unsigned32_t *)data_out)[i] );
           //   break;
      //case 8: SIM_array_outdrv_global_record( &arr->outdrv, ((unsigned64_t *)data_out)[i] );
           //   break;
      //default:        /* some error handler */
    //}
  //}

  //if ( ! IS_FULLY_ASSOC( info )) {
    //for ( i = 0; i < info->assoc; i ++ )
      //for ( j = 0; j < info->n_item; j ++ )
        /* sense amplifiers output 0 when idle */
        //switch ( data_size ) {
          //case 1: SIM_array_outdrv_local_record( &arr->outdrv, 0, ((unsigned8_t **)data_all)[i][j], SIM_ARRAY_RECOVER );
                  //break;
          //case 2: SIM_array_outdrv_local_record( &arr->outdrv, 0, ((unsigned16_t **)data_all)[i][j], SIM_ARRAY_RECOVER );
                  //break;
          //case 4: SIM_array_outdrv_local_record( &arr->outdrv, 0, ((unsigned32_t **)data_all)[i][j], SIM_ARRAY_RECOVER );
                  //break;
          //case 8: SIM_array_outdrv_local_record( &arr->outdrv, 0, ((unsigned64_t **)data_all)[i][j], SIM_ARRAY_RECOVER );
                  //break;
          //default:    /* some error handler */
        //}
  //}

  //return 0;
//}


/********* end from SIM_array_internal_m.c **********/


// ------- Array init


int power_array_init(power_array_info *info, power_array *arr )
{
  unsigned rows, cols, ports, dec_width, n_bitline_pre, n_colsel_pre;
  double wordline_len, bitline_len, tagline_len, matchline_len;
  double wordline_cmetal, bitline_cmetal;
  double Cline, pre_size, comp_pre_size;

  arr->i_leakage = 0;

  /* sanity check */
  if ( info->read_ports == 0 ) info->share_rw = 0;
  if ( info->share_rw ) { //AMIT are read and write ports shared?
    info->data_end = 2;
    info->tag_end = 2;
  }

  if ( info->share_rw ) ports = info->read_ports;
  else ports = info->read_ports + info->write_ports;

  /* data array unit length wire cap */
  if (ports > 1) {
    /* 3x minimal spacing */
    wordline_cmetal = CC3M3metal;
    bitline_cmetal = CC3M2metal;
  }
  else if (info->data_end == 2) {
    /* wordline infinite spacing, bitline 3x minimal spacing */
    wordline_cmetal = CM3metal;
    bitline_cmetal = CC3M2metal;
  }
  else {
    /* both infinite spacing */
    wordline_cmetal = CM3metal;
    bitline_cmetal = CM2metal;
  }

  info->data_arr_width = 0;
  info->tag_arr_width = 0;
  info->data_arr_height = 0;
  info->tag_arr_height = 0;

  /* BEGIN: data array power initialization */
  if (dec_width = SIM_power_logtwo(info->n_set)) { //AMIT: not fully associative, n->sets!=1
    /* row decoder power initialization */
    SIM_array_dec_init( &arr->row_dec, info->row_dec_model, dec_width );

    /* row decoder precharging power initialization */
    //if ( is_dynamic_dec( info->row_dec_model ))
      /* FIXME: need real pre_size */
      //SIM_array_pre_init( &arr->row_dec_pre, info->row_dec_pre_model, 0 );

    rows = info->n_set / info->data_ndbl / info->data_nspd; //AMIT: n_set is the number of sets(fully associative n_sets=1)
    cols = info->blk_bits * info->assoc * info->data_nspd / info->data_ndwl; //AMIT: blk_bits is the line size

    bitline_len = rows * ( RegCellHeight + ports * WordlineSpacing );
    if ( info->data_end == 2 )
      wordline_len = cols * ( RegCellWidth + 2 * ports * BitlineSpacing );
    else                /* info->data_end == 1 */
      wordline_len = cols * ( RegCellWidth + ( 2 * ports - info->read_ports ) * BitlineSpacing );
    info->data_arr_width = wordline_len;
    info->data_arr_height = bitline_len;

    /* compute precharging size */
    /* FIXME: should consider n_pre and pre_size simultaneously */
    Cline = rows * SIM_power_draincap( Wmemcellr, NCH, 1 ) + bitline_cmetal * bitline_len;
    pre_size = SIM_power_driver_size( Cline, Period / 8 );
    /* WHS: ?? compensate for not having an nmos pre-charging */
    pre_size += pre_size * Wdecinvn / Wdecinvp;

    /* bitline power initialization */
    n_bitline_pre = n_pre_drain( info->data_bitline_pre_model );
    n_colsel_pre = ( info->data_n_share_amp > 1 ) ? n_pre_drain( info->data_colsel_pre_model ) : 0;
    SIM_array_bitline_init(&arr->data_bitline, info->data_bitline_model, info->share_rw, info->data_end, rows, bitline_len * bitline_cmetal, info->data_n_share_amp, n_bitline_pre, n_colsel_pre, pre_size, info->outdrv_model);
    /* static power */
    arr->i_leakage += arr->data_bitline.i_leakage * cols * info->write_ports;

    /* bitline precharging power initialization */
    SIM_array_pre_init( &arr->data_bitline_pre, info->data_bitline_pre_model, pre_size );
    /* static power */
    arr->i_leakage += arr->data_bitline_pre.i_leakage * cols * info->read_ports;
    /* bitline column selector precharging power initialization */
    if ( info->data_n_share_amp > 1 )
      SIM_array_pre_init( &arr->data_colsel_pre, info->data_colsel_pre_model, pre_size );

    /* sense amplifier power initialization */
    SIM_array_amp_init( &arr->data_amp, info->data_amp_model );
  }
  else {
    /* info->n_set == 1 means this array is fully-associative */
    rows = info->assoc;
    cols = info->blk_bits;

    /* WHS: no read wordlines or bitlines */
    bitline_len = rows * ( RegCellHeight + info->write_ports * WordlineSpacing );
    wordline_len = cols * ( RegCellWidth + 2 * info->write_ports * BitlineSpacing );
    info->data_arr_width = wordline_len;
    info->data_arr_height = bitline_len;

    /* bitline power initialization */
    SIM_array_bitline_init(&arr->data_bitline, info->data_bitline_model, 0, info->data_end, rows, bitline_len * bitline_cmetal, 1, 0, 0, 0, info->outdrv_model);
  }

  /* wordline power initialization */
  SIM_array_wordline_init( &arr->data_wordline, info->data_wordline_model, info->share_rw, cols, wordline_len * wordline_cmetal, info->data_end );
  /* static power */
  arr->i_leakage += arr->data_wordline.i_leakage * rows * ports;

  if (dec_width = SIM_power_logtwo(info->n_item)) {
    /* multiplexor power initialization */
    SIM_array_mux_init( &arr->mux, info->mux_model, info->n_item, info->assoc );

    /* column decoder power initialization */
    SIM_array_dec_init( &arr->col_dec, info->col_dec_model, dec_width );

    /* column decoder precharging power initialization */
    //if ( is_dynamic_dec( info->col_dec_model ))
      /* FIXME: need real pre_size */
      //SIM_array_pre_init( &arr->col_dec_pre, info->col_dec_pre_model, 0 );
  }

  /* memory cell power initialization */
  SIM_array_mem_init( &arr->data_mem, info->data_mem_model, info->read_ports, info->write_ports, info->share_rw, info->data_end );
  /* static power */
  arr->i_leakage += arr->data_mem.i_leakage * rows * cols;

  /* output driver power initialization */
  SIM_array_outdrv_init( &arr->outdrv, info->outdrv_model, info->data_width );
  /* END: data array power initialization */


  /* BEGIN: tag array power initialization */
  /* assume a tag array must have memory cells */
  if ( info->tag_mem_model ) {
    if ( info->n_set > 1 ) {
      /* tag array unit length wire cap */
      if (ports > 1) {
        /* 3x minimal spacing */
        wordline_cmetal = CC3M3metal;
        bitline_cmetal = CC3M2metal;
      }
      else if (info->data_end == 2) {
        /* wordline infinite spacing, bitline 3x minimal spacing */
        wordline_cmetal = CM3metal;
        bitline_cmetal = CC3M2metal;
      }
      else {
        /* both infinite spacing */
        wordline_cmetal = CM3metal;
        bitline_cmetal = CM2metal;
      }

      rows = info->n_set / info->tag_ndbl / info->tag_nspd;
      cols = info->tag_line_width * info->assoc * info->tag_nspd / info->tag_ndwl;

      bitline_len = rows * ( RegCellHeight + ports * WordlineSpacing );
      if ( info->tag_end == 2 )
        wordline_len = cols * ( RegCellWidth + 2 * ports * BitlineSpacing );
      else              /* info->tag_end == 1 */
        wordline_len = cols * ( RegCellWidth + ( 2 * ports - info->read_ports ) * BitlineSpacing );
      info->tag_arr_width = wordline_len;
      info->tag_arr_height = bitline_len;

      /* compute precharging size */
      /* FIXME: should consider n_pre and pre_size simultaneously */
      Cline = rows * SIM_power_draincap( Wmemcellr, NCH, 1 ) + bitline_cmetal * bitline_len;
      pre_size = SIM_power_driver_size( Cline, Period / 8 );
      /* WHS: ?? compensate for not having an nmos pre-charging */
      pre_size += pre_size * Wdecinvn / Wdecinvp;

      /* bitline power initialization */
      n_bitline_pre = n_pre_drain( info->tag_bitline_pre_model );
      n_colsel_pre = ( info->tag_n_share_amp > 1 ) ? n_pre_drain( info->tag_colsel_pre_model ) : 0;
      SIM_array_bitline_init(&arr->tag_bitline, info->tag_bitline_model, info->share_rw, info->tag_end, rows, bitline_len * bitline_cmetal, info->tag_n_share_amp, n_bitline_pre, n_colsel_pre, pre_size, SIM_NO_MODEL);

      /* bitline precharging power initialization */
      SIM_array_pre_init( &arr->tag_bitline_pre, info->tag_bitline_pre_model, pre_size );
      /* bitline column selector precharging power initialization */
      if ( info->tag_n_share_amp > 1 )
        SIM_array_pre_init( &arr->tag_colsel_pre, info->tag_colsel_pre_model, pre_size );

      /* sense amplifier power initialization */
      SIM_array_amp_init( &arr->tag_amp, info->tag_amp_model );

      /* prepare for comparator initialization */
      tagline_len = matchline_len = 0;
      comp_pre_size = Wcomppreequ;
    }
    else {      /* info->n_set == 1 */
      /* cam cells are big enough, so infinite spacing */
      wordline_cmetal = CM3metal;
      bitline_cmetal = CM2metal;

      rows = info->assoc;
      /* FIXME: operations of valid bit, use bit and dirty bit are not modeled */
      cols = info->tag_addr_width;

      bitline_len = rows * ( CamCellHeight + ports * WordlineSpacing + info->read_ports * MatchlineSpacing );
      if ( info->tag_end == 2 )
        wordline_len = cols * ( CamCellWidth + 2 * ports * BitlineSpacing + 2 * info->read_ports * TaglineSpacing );
      else              /* info->tag_end == 1 */
        wordline_len = cols * ( CamCellWidth + ( 2 * ports - info->read_ports ) * BitlineSpacing + 2 * info->read_ports * TaglineSpacing );
      info->tag_arr_width = wordline_len;
      info->tag_arr_height = bitline_len;

      if ( is_rw_bitline ( info->tag_bitline_model )) {
        /* compute precharging size */
        /* FIXME: should consider n_pre and pre_size simultaneously */
        Cline = rows * SIM_power_draincap( Wmemcellr, NCH, 1 ) + bitline_cmetal * bitline_len;
        pre_size = SIM_power_driver_size( Cline, Period / 8 );
        /* WHS: ?? compensate for not having an nmos pre-charging */
        pre_size += pre_size * Wdecinvn / Wdecinvp;

        /* bitline power initialization */
        n_bitline_pre = n_pre_drain( info->tag_bitline_pre_model );
        SIM_array_bitline_init(&arr->tag_bitline, info->tag_bitline_model, info->share_rw, info->tag_end, rows, bitline_len * bitline_cmetal, 1, n_bitline_pre, 0, pre_size, SIM_NO_MODEL);

        /* bitline precharging power initialization */
        SIM_array_pre_init( &arr->tag_bitline_pre, info->tag_bitline_pre_model, pre_size );

        /* sense amplifier power initialization */
        SIM_array_amp_init( &arr->tag_amp, info->tag_amp_model );
      }
      else {
        /* bitline power initialization */
        SIM_array_bitline_init(&arr->tag_bitline, info->tag_bitline_model, 0, info->tag_end, rows, bitline_len * bitline_cmetal, 1, 0, 0, 0, SIM_NO_MODEL);
      }

      /* memory cell power initialization */
      SIM_array_mem_init( &arr->tag_attach_mem, info->tag_attach_mem_model, info->read_ports, info->write_ports, info->share_rw, info->tag_end );

      /* prepare for comparator initialization */
      tagline_len = bitline_len;
      matchline_len = wordline_len;
      comp_pre_size = Wmatchpchg;
    }

    /* wordline power initialization */
    SIM_array_wordline_init( &arr->tag_wordline, info->tag_wordline_model, info->share_rw, cols, wordline_len * wordline_cmetal, info->tag_end );

    /* comparator power initialization */
    SIM_array_comp_init( &arr->comp, info->comp_model, info->tag_addr_width, info->assoc, n_pre_drain( info->comp_pre_model ), matchline_len, tagline_len );

    /* comparator precharging power initialization */
    SIM_array_pre_init( &arr->comp_pre, info->comp_pre_model, comp_pre_size );

    /* memory cell power initialization */
    SIM_array_mem_init( &arr->tag_mem, info->tag_mem_model, info->read_ports, info->write_ports, info->share_rw, info->tag_end );
  }
  /* END: tag array power initialization */

  return 0;
}

double array_report(power_array_info *info, power_array *arr)
{
  double epart, etotal = 0;

  if (info->row_dec_model) {
    epart = SIM_array_dec_report(&arr->row_dec);
    //fprintf(stderr, "row decoder: %g\n", epart);
    etotal += epart;
  }
  if (info->col_dec_model) {
    epart = SIM_array_dec_report(&arr->col_dec);
    //fprintf(stderr, "col decoder: %g\n", epart);
    etotal += epart;
  }
  if (info->data_wordline_model) {
    epart = SIM_array_wordline_report(&arr->data_wordline);
    //fprintf(stderr, "data wordline: %g\n", epart);
    etotal += epart;
  }
  if (info->tag_wordline_model) {
    epart = SIM_array_wordline_report(&arr->tag_wordline);
    //fprintf(stderr, "tag wordline: %g\n", epart);
    etotal += epart;
  }
  if (info->data_bitline_model) {
    epart = SIM_array_bitline_report(&arr->data_bitline);
    //fprintf(stderr, "data bitline: %g\n", epart);
    etotal += epart;
  }
  if (info->data_bitline_pre_model) {
    epart = SIM_array_pre_report(&arr->data_bitline_pre);
    //fprintf(stderr, "data bitline precharge: %g\n", epart);
    etotal += epart;
  }
  if (info->tag_bitline_model) {
    epart = SIM_array_bitline_report(&arr->tag_bitline);
    //fprintf(stderr, "tag bitline: %g\n", epart);
    etotal += epart;
  }
  if (info->data_mem_model) {
    epart = SIM_array_mem_report(&arr->data_mem);
    //fprintf(stderr, "data memory: %g\n", epart);
    etotal += epart;
  }
  if (info->tag_mem_model) {
    epart = SIM_array_mem_report(&arr->tag_mem);
    //fprintf(stderr, "tag memory: %g\n", epart);
    etotal += epart;
  }
  if (info->data_amp_model) {
    epart = SIM_array_amp_report(&arr->data_amp);
    //fprintf(stderr, "data amp: %g\n", epart);
    etotal += epart;
  }
  if (info->tag_amp_model) {
    epart = SIM_array_amp_report(&arr->tag_amp);
    //fprintf(stderr, "tag amp: %g\n", epart);
    etotal += epart;
  }
  if (info->comp_model) {
    epart = SIM_array_comp_report(&arr->comp);
    //fprintf(stderr, "comparator: %g\n", epart);
    etotal += epart;
  }
  if (info->mux_model) {
    epart = SIM_array_mux_report(&arr->mux);
    //fprintf(stderr, "multiplexor: %g\n", epart);
    etotal += epart;
  }
  if (info->outdrv_model) {
    epart = SIM_array_outdrv_report(&arr->outdrv);
    //fprintf(stderr, "output driver: %g\n", epart);
    etotal += epart;
  }
  /* ignore other precharging for now */

  //fprintf(stderr, "total energy: %g\n", etotal);

  return etotal;
}

/* ==================== buffer (wrapper/record functions) ==================== */

/* record read data activity */
int SIM_buf_power_data_read(power_array_info *info, power_array *arr, unsigned long int data)
{
  /* precharge */
  SIM_array_pre_record(&arr->data_bitline_pre, info->blk_bits);
  /* drive the wordline */
  SIM_power_array_dec(info, arr, NULL, 0, SIM_ARRAY_READ);
  /* read data */
  SIM_power_array_data_read(info, arr, data);

  return 0;
}


/* record write data bitline and memory cell activity */
int SIM_buf_power_data_write(power_array_info *info, power_array *arr, char *data_line, char *old_data, char *new_data)
{
#define N_ITEM  (PARM_flit_width / 8 + (PARM_flit_width % 8 ? 1:0))
  /* drive the wordline */
  SIM_power_array_dec(info, arr, NULL, 0, SIM_ARRAY_WRITE);
  /* write data */
  SIM_power_array_data_write(info, arr, NULL, N_ITEM, data_line, old_data, new_data);

  return 0;
}

/* WHS: missing data output wrapper function */

/* ==================== buffer (wrapper/record functions) end ==================== */





int SIM_array_clear_stat(power_array *arr)
{
  SIM_array_dec_clear_stat(&arr->row_dec);
  SIM_array_dec_clear_stat(&arr->col_dec);
  SIM_array_wordline_clear_stat(&arr->data_wordline);
  SIM_array_wordline_clear_stat(&arr->tag_wordline);
  SIM_array_bitline_clear_stat(&arr->data_bitline);
  SIM_array_bitline_clear_stat(&arr->tag_bitline);
  SIM_array_mem_clear_stat(&arr->data_mem);
  SIM_array_mem_clear_stat(&arr->tag_mem);
  SIM_array_mem_clear_stat(&arr->tag_attach_mem);
  SIM_array_amp_clear_stat(&arr->data_amp);
  SIM_array_amp_clear_stat(&arr->tag_amp);
  SIM_array_comp_clear_stat(&arr->comp);
  SIM_array_mux_clear_stat(&arr->mux);
  SIM_array_outdrv_clear_stat(&arr->outdrv);
  SIM_array_pre_clear_stat(&arr->row_dec_pre);
  SIM_array_pre_clear_stat(&arr->col_dec_pre);
  SIM_array_pre_clear_stat(&arr->data_bitline_pre);
  SIM_array_pre_clear_stat(&arr->tag_bitline_pre);
  SIM_array_pre_clear_stat(&arr->data_colsel_pre);
  SIM_array_pre_clear_stat(&arr->tag_colsel_pre);
  SIM_array_pre_clear_stat(&arr->comp_pre);

  return 0;
}





