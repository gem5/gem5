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

#include <cassert>
#include <cmath>
#include <cstdio>

#include "mem/ruby/network/orion/parm_technology.hh"
#include "mem/ruby/network/orion/power_utils.hh"

/* ----------- from SIM_power_util.c ------------ */

/* Hamming distance table */
static char h_tab[256] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8};


static unsigned SIM_power_Hamming_slow( unsigned long int old_val, unsigned long int new_val, unsigned long int mask )
{
  /* old slow code, I don't understand the new fast code though */
  /* unsigned long int dist;
  unsigned Hamming = 0;

  dist = ( old_val ^ new_val ) & mask;
  mask = (mask >> 1) + 1;

  while ( mask ) {
    if ( mask & dist ) Hamming ++;
    mask = mask >> 1;
  }

  return Hamming; */

#define TWO(k) (BIGONE << (k))
#define CYCL(k) (BIGNONE/(1 + (TWO(TWO(k)))))
#define BSUM(x,k) ((x)+=(x) >> TWO(k), (x) &= CYCL(k))
  unsigned long int x;

  x = (old_val ^ new_val) & mask;
  x = (x & CYCL(0)) + ((x>>TWO(0)) & CYCL(0));
  x = (x & CYCL(1)) + ((x>>TWO(1)) & CYCL(1));
  BSUM(x,2);
  BSUM(x,3);
  BSUM(x,4);
  BSUM(x,5);

  return x;
}


int SIM_power_init(void)
{
  unsigned i;

  /* initialize Hamming distance table */
  for (i = 0; i < 256; i++)
    h_tab[i] = SIM_power_Hamming_slow(i, 0, 0xFF);

  return 0;
}


/* assume unsigned long int is unsigned64_t */
unsigned SIM_power_Hamming(unsigned long int old_val, unsigned long int new_val, unsigned long int mask)
{
  union {
          unsigned long int x;
          char id[8];
        } u;
  unsigned rval;

  u.x = (old_val ^ new_val) & mask;

  rval = h_tab[u.id[0]];
  rval += h_tab[u.id[1]];
  rval += h_tab[u.id[2]];
  rval += h_tab[u.id[3]];
  rval += h_tab[u.id[4]];
  rval += h_tab[u.id[5]];
  rval += h_tab[u.id[6]];
  rval += h_tab[u.id[7]];

  return rval;
}


unsigned SIM_power_Hamming_group(unsigned long int d1_new, unsigned long int d1_old, unsigned long int d2_new, unsigned long int d2_old, unsigned width, unsigned n_grp)
{
  unsigned rval = 0;
  unsigned long int g1_new, g1_old, g2_new, g2_old, mask;

  mask = HAMM_MASK(width);

  while (n_grp--) {
    g1_new = d1_new & mask;
    g1_old = d1_old & mask;
    g2_new = d2_new & mask;
    g2_old = d2_old & mask;

    if (g1_new != g1_old || g2_new != g2_old)
      rval ++;

    d1_new >>= width;
    d1_old >>= width;
    d2_new >>= width;
    d2_old >>= width;
  }

  return rval;
}

/* ---------------------------------------------- */

/* ----------------------------------------------- */




double logtwo(double x)
{
  assert(x > 0);
  return log10(x)/log10(2);
}

unsigned SIM_power_logtwo(unsigned long int x)
{
  unsigned rval = 0;

  while (x >> rval && rval < sizeof(unsigned long int) << 3) rval++;
  if (x == (BIGONE << rval - 1)) rval--;

  return rval;
}

