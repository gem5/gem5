/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  sc_nbexterns.cpp -- External functions for both sc_signed and sc_unsigned
                      classes. These functions work on two parameters u and
                      v, and copy the result to the first parameter u. This
                      is also the reason that they are suffixed with _on_help.
 
  Original Author: Ali Dasdan, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// $Log: sc_nbexterns.cpp,v $
// Revision 1.2  2011/02/18 20:19:15  acg
//  Andy Goodrich: updating Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:49:32  acg
// Added $Log command so that CVS check in comments are reproduced in the
// source.
//

#include "sysc/datatypes/int/sc_nbexterns.h"
#include "sysc/kernel/sc_macros.h"


namespace sc_dt
{

// ----------------------------------------------------------------------------
//  SECTION: External functions for PLUS operators.
// ----------------------------------------------------------------------------

// Handles the cases 3 and 4 and returns the result in u.
void
add_on_help(small_type &us, int /* unb */, int und,
            sc_digit *ud, 
            small_type vs, int /* vnb */, int vnd,
            const sc_digit *vd)
{

  vnd = vec_skip_leading_zeros(vnd, vd);

  if (us == vs) {  // case 3

    if (und >= vnd)
      vec_add_on(und, ud, vnd, vd);
    else
      vec_add_on2(und, ud, vnd, vd);

  }
  else {  // case 4

    // vec_cmp expects that und is the number of non-zero digits in ud.
    int new_und = vec_skip_leading_zeros(und, ud); 
    int cmp_res = vec_cmp(new_und, ud, vnd, vd);

    if (cmp_res == 0)  { // u == v
      us = SC_ZERO;
      vec_zero(und, ud);
      return;
    }

    if (cmp_res > 0) // u > v
      vec_sub_on(und, ud, vnd, vd);

    else { // u < v
      us = -us;
      vec_sub_on2(und, ud, vnd, vd);
    }

  }
}


// ----------------------------------------------------------------------------

/* 

mul_on_help_signed and mul_on_help_unsigned have the same body except
that CONVERT_SM_to_2C_to_SM and COPY_DIGITS are defined for signed and
unsigned, respectively.  This comment also applies to the
signed/unsigned versions of div_on_help and mod_on_help. It is
possible to take COPY_DIGITS out of these functions and create a
single version of each of these helper functions; however, this will
impose an onverhead on performance. In the versions below, any change
in the signed version of a helper function must be carried to a
corresponding change in the unsigned verion of the same function or
vice versa.

*/


// ----------------------------------------------------------------------------
//  SECTION: External functions of MULTIPLICATION operators.
// ----------------------------------------------------------------------------

void
mul_on_help_signed(small_type &us, 
                   int unb, int und, 
                   sc_digit *ud, 
                   int vnb, int vnd,
                   const sc_digit *vd)
{
#define CONVERT_SM_to_2C_to_SM convert_signed_SM_to_2C_to_SM
#define COPY_DIGITS copy_digits_signed

  {  // Body of mul_on_help

    int old_und = und;

    und = vec_skip_leading_zeros(und, ud);
    vnd = vec_skip_leading_zeros(vnd, vd);
    
    sc_digit ud0 = (*ud);
    sc_digit vd0 = (*vd);
    
    if ((vnd == 1) && (vd0 == 1)) {
      us = CONVERT_SM_to_2C_to_SM(us, unb, old_und, ud);
      return;
    }
    
    if ((und == 1) && (ud0 == 1)) {
      COPY_DIGITS(us, unb, old_und, ud, vnb, vnd, vd);
      return;
    }
    
    if ((und == 1) && (vnd == 1) && 
        (ud0 < HALF_DIGIT_RADIX) && (vd0 < HALF_DIGIT_RADIX)) {
      
      sc_digit d = ud0 * vd0;
      COPY_DIGITS(us, unb, old_und, ud, unb + vnb, 1, &d);
      return;
      
    }
    
    int nd = und + vnd;
    
#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS];
#else
    sc_digit *d = new sc_digit[nd];
#endif
  
    vec_zero(nd, d);
    
    if ((und == 1) && (ud0 < HALF_DIGIT_RADIX))
      vec_mul_small(vnd, vd, ud0, d);
    
    else if ((vnd == 1) && (vd0 < HALF_DIGIT_RADIX))
      vec_mul_small(und, ud, vd0, d);
    
    else if (vnd < und)
      vec_mul(und, ud, vnd, vd, d);
    
    else
      vec_mul(vnd, vd, und, ud, d);
    
    COPY_DIGITS(us, unb, old_und, ud, unb + vnb, nd, d);
    
#ifndef SC_MAX_NBITS
    delete [] d;
#endif
    
  }

#undef COPY_DIGITS
#undef CONVERT_SM_to_2C_to_SM

}


void
mul_on_help_unsigned(small_type &us, 
                     int unb, int und, 
                     sc_digit *ud, 
                     int vnb, int vnd,
                     const sc_digit *vd)
{
#define CONVERT_SM_to_2C_to_SM convert_unsigned_SM_to_2C_to_SM
#define COPY_DIGITS copy_digits_unsigned

  {  // Body of mul_on_help

    int old_und = und;

    und = vec_skip_leading_zeros(und, ud);
    vnd = vec_skip_leading_zeros(vnd, vd);
    
    sc_digit ud0 = (*ud);
    sc_digit vd0 = (*vd);
    
    if ((vnd == 1) && (vd0 == 1)) {
      us = CONVERT_SM_to_2C_to_SM(us, unb, old_und, ud);
      return;
    }
    
    if ((und == 1) && (ud0 == 1)) {
      COPY_DIGITS(us, unb, old_und, ud, vnb, vnd, vd);
      return;
    }
    
    if ((und == 1) && (vnd == 1) && 
        (ud0 < HALF_DIGIT_RADIX) && (vd0 < HALF_DIGIT_RADIX)) {
      
      sc_digit d = ud0 * vd0;
      COPY_DIGITS(us, unb, old_und, ud, unb + vnb, 1, &d);
      return;
      
    }
    
    int nd = und + vnd;
    
#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS];
#else
    sc_digit *d = new sc_digit[nd];
#endif
  
    vec_zero(nd, d);
    
    if ((und == 1) && (ud0 < HALF_DIGIT_RADIX))
      vec_mul_small(vnd, vd, ud0, d);
    
    else if ((vnd == 1) && (vd0 < HALF_DIGIT_RADIX))
      vec_mul_small(und, ud, vd0, d);
    
    else if (vnd < und)
      vec_mul(und, ud, vnd, vd, d);
    
    else
      vec_mul(vnd, vd, und, ud, d);
    
    COPY_DIGITS(us, unb, old_und, ud, unb + vnb, nd, d);
    
#ifndef SC_MAX_NBITS
    delete [] d;
#endif
    
  }

#undef COPY_DIGITS
#undef CONVERT_SM_to_2C_to_SM

}


// ----------------------------------------------------------------------------
//  SECTION: External functions for DIVISION operators.
// ----------------------------------------------------------------------------

void
div_on_help_signed(small_type &us, 
                   int unb, int und, 
                   sc_digit *ud, 
                   int vnb, int vnd,
                   const sc_digit *vd)
{
#define CONVERT_SM_to_2C_to_SM convert_signed_SM_to_2C_to_SM
#define COPY_DIGITS copy_digits_signed

  {  // Body of div_on_help

    int old_und = und;
    
    und = vec_skip_leading_zeros(und, ud);
    vnd = vec_skip_leading_zeros(vnd, vd);
    
    int cmp_res = vec_cmp(und, ud, vnd, vd);
    
    if (cmp_res < 0) { // u < v => u / v = 0 - case 4
      us = SC_ZERO;
      vec_zero(old_und, ud);
      return;
    }
    
    sc_digit vd0 = (*vd);
    
    if ((cmp_res > 0) && (vnd == 1) && (vd0 == 1))  {
      us = CONVERT_SM_to_2C_to_SM(us, unb, old_und, ud);
      return;
    }
    
    // One extra digit for d is allocated to simplify vec_div_*().
    int nd = sc_max(und, vnd) + 1;
    
#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS + 1];
#else
    sc_digit *d = new sc_digit[nd];
#endif
    
    vec_zero(nd, d);
    
    // u = v => u / v = 1 - case 3
    if (cmp_res == 0)
      d[0] = 1;
    
    else if ((vnd == 1) && (und == 1))
      d[0] = (*ud) / vd0;
    
    else if ((vnd == 1) && (vd0 < HALF_DIGIT_RADIX))
      vec_div_small(und, ud, vd0, d);
    
    else
      vec_div_large(und, ud, vnd, vd, d);
    
    COPY_DIGITS(us, unb, old_und, ud, sc_max(unb, vnb), nd - 1, d);
    
#ifndef SC_MAX_NBITS
    delete [] d;
#endif
    
  }
  
#undef COPY_DIGITS
#undef CONVERT_SM_to_2C_to_SM

}


void
div_on_help_unsigned(small_type &us, 
                     int unb, int und, 
                     sc_digit *ud, 
                     int vnb, int vnd,
                     const sc_digit *vd)
{
#define CONVERT_SM_to_2C_to_SM convert_unsigned_SM_to_2C_to_SM
#define COPY_DIGITS copy_digits_unsigned

  {  // Body of div_on_help

    int old_und = und;
    
    und = vec_skip_leading_zeros(und, ud);
    vnd = vec_skip_leading_zeros(vnd, vd);
    
    int cmp_res = vec_cmp(und, ud, vnd, vd);
    
    if (cmp_res < 0) { // u < v => u / v = 0 - case 4
      us = SC_ZERO;
      vec_zero(old_und, ud);
      return;
    }
    
    sc_digit vd0 = (*vd);
    
    if ((cmp_res > 0) && (vnd == 1) && (vd0 == 1))  {
      us = CONVERT_SM_to_2C_to_SM(us, unb, old_und, ud);
      return;
    }
    
    // One extra digit for d is allocated to simplify vec_div_*().
    int nd = sc_max(und, vnd) + 1;
    
#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS + 1];
#else
    sc_digit *d = new sc_digit[nd];
#endif
    
    vec_zero(nd, d);
    
    // u = v => u / v = 1 - case 3
    if (cmp_res == 0)
      d[0] = 1;
    
    else if ((vnd == 1) && (und == 1))
      d[0] = (*ud) / vd0;
    
    else if ((vnd == 1) && (vd0 < HALF_DIGIT_RADIX))
      vec_div_small(und, ud, vd0, d);
    
    else
      vec_div_large(und, ud, vnd, vd, d);
    
    COPY_DIGITS(us, unb, old_und, ud, sc_max(unb, vnb), nd - 1, d);
    
#ifndef SC_MAX_NBITS
    delete [] d;
#endif
    
  }
  
#undef COPY_DIGITS
#undef CONVERT_SM_to_2C_to_SM

}


// ----------------------------------------------------------------------------
//  SECTION: External functions for MOD operators.
// ----------------------------------------------------------------------------

void
mod_on_help_signed(small_type &us, 
                   int unb, int und, 
                   sc_digit *ud, 
                   int /* vnb */, int vnd,
                   const sc_digit *vd)
{

#define COPY_DIGITS copy_digits_signed

  { // Body of mod_on_help

    int old_und = und;
    
    und = vec_skip_leading_zeros(und, ud);
    vnd = vec_skip_leading_zeros(vnd, vd);
    
    int cmp_res = vec_cmp(und, ud, vnd, vd);
    
    // u < v => u % v = u - case 4
    if (cmp_res < 0) 
      return;
    
    // u = v => u % v = 0 - case 3
    if (cmp_res == 0) { 
      us = SC_ZERO;
      vec_zero(old_und, ud);
      return;
    }
    
    // else if u > v - case 5
    
    sc_digit vd0 = (*vd);
    
    if ((vnd == 1) && (vd0 == 1)) {
      us = SC_ZERO;
      vec_zero(old_und, ud);
      return;
    }
    
    // One extra digit for d is allocated to simplify vec_div_*().
    int nd = sc_max(und, vnd) + 1;
    
#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS + 1];
#else
    sc_digit *d = new sc_digit[nd];
#endif
    
    vec_zero(nd, d);
    
    if ((vnd == 1) && (und == 1))
      d[0] = (*ud) % vd0;
    
    if ((vnd == 1) && (vd0 < HALF_DIGIT_RADIX))
      d[0] = vec_rem_small(und, ud, vd0);
    
    else
      vec_rem_large(und, ud, vnd, vd, d);
    
    us = check_for_zero(us, nd - 1, d);
    
    if (us == SC_ZERO)
      vec_zero(old_und, ud);
    else
      COPY_DIGITS(us, unb, old_und, ud, sc_min(unb, vnd), nd - 1, d);
    
#ifndef SC_MAX_NBITS
    delete [] d;
#endif
    
  }
  
#undef COPY_DIGITS
  
}


void
mod_on_help_unsigned(small_type &us, 
                     int unb, int und, 
                     sc_digit *ud, 
                     int /* vnb */, int vnd,
                     const sc_digit *vd)
{

#define COPY_DIGITS copy_digits_unsigned

  { // Body of mod_on_help

    int old_und = und;
    
    und = vec_skip_leading_zeros(und, ud);
    vnd = vec_skip_leading_zeros(vnd, vd);
    
    int cmp_res = vec_cmp(und, ud, vnd, vd);
    
    // u < v => u % v = u - case 4
    if (cmp_res < 0) 
      return;
    
    // u = v => u % v = 0 - case 3
    if (cmp_res == 0) { 
      us = SC_ZERO;
      vec_zero(old_und, ud);
      return;
    }
    
    // else if u > v - case 5
    
    sc_digit vd0 = (*vd);
    
    if ((vnd == 1) && (vd0 == 1)) {
      us = SC_ZERO;
      vec_zero(old_und, ud);
      return;
    }
    
    // One extra digit for d is allocated to simplify vec_div_*().
    int nd = sc_max(und, vnd) + 1;
    
#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS + 1];
#else
    sc_digit *d = new sc_digit[nd];
#endif
    
    vec_zero(nd, d);
    
    if ((vnd == 1) && (und == 1))
      d[0] = (*ud) % vd0;
    
    if ((vnd == 1) && (vd0 < HALF_DIGIT_RADIX))
      d[0] = vec_rem_small(und, ud, vd0);
    
    else
      vec_rem_large(und, ud, vnd, vd, d);
    
    us = check_for_zero(us, nd - 1, d);
    
    if (us == SC_ZERO)
      vec_zero(old_und, ud);
    else
      COPY_DIGITS(us, unb, old_und, ud, sc_min(unb, vnd), nd - 1, d);
    
#ifndef SC_MAX_NBITS
    delete [] d;
#endif
    
  }
  
#undef COPY_DIGITS
  
}


// ----------------------------------------------------------------------------
//  SECTION: External functions for AND operators.
// ----------------------------------------------------------------------------

// Handles the cases 2-5 and returns the result in u.
void
and_on_help(small_type us, 
            int /* unb */, int und, 
            sc_digit *ud, 
            small_type vs,
            int /* vnb */, int vnd,
            const sc_digit *vd)
{

  sc_digit *x = ud;
  const sc_digit *y = vd;
  int xnd = und;
  int ynd = vnd;

  // Truncate y.
  if (xnd < ynd)
    ynd = xnd;

  const sc_digit *xend = (x + xnd);
  const sc_digit *yend = (y + ynd);

  // x is longer than y.

  small_type s = mul_signs(us, vs);

  if (s > 0) {

    if (us > 0) { // case 2

      while (y < yend)
        (*x++) &= (*y++);

      while (x < xend)
        (*x++) = 0;

    }
    else {  // case 3

      sc_digit xcarry = 1;
      sc_digit ycarry = 1;

      while (y < yend) {
        xcarry += (~(*x) & DIGIT_MASK);
        ycarry += (~(*y++) & DIGIT_MASK);
        (*x++) = (xcarry & ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        xcarry += (~(*x) & DIGIT_MASK);
        ycarry += DIGIT_MASK;
        (*x++) = (xcarry & ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }

    }
  }
  else {

    if (us > 0) { // case 4

      sc_digit ycarry = 1;

      while (y < yend) {
        ycarry += (~(*y++) & DIGIT_MASK);
        (*x++) &= ycarry & DIGIT_MASK;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        ycarry += DIGIT_MASK;
        (*x++) &= ycarry & DIGIT_MASK;
        ycarry >>= BITS_PER_DIGIT;
      }

    }
    else {  // case 5

      sc_digit xcarry = 1;

      while (y < yend) {
        xcarry += (~(*x) & DIGIT_MASK);
        (*x++) = (xcarry & (*y++)) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
      }

      while (x < xend)
        (*x++) = 0;

    }
  }
}


// ----------------------------------------------------------------------------
//  SECTION: External functions for OR operators.
// ----------------------------------------------------------------------------

// Handles the cases 3-5 and returns the result in u.
void
or_on_help(small_type us, 
           int /* unb */, int und, 
           sc_digit *ud, 
           small_type vs,
           int /* vnb */, int vnd,
           const sc_digit *vd)
{
  
  sc_digit *x = ud;
  const sc_digit *y = vd;
  int xnd = und;
  int ynd = vnd;

  if (xnd < ynd)
    ynd = xnd;

  const sc_digit *xend = (x + xnd);
  const sc_digit *yend = (y + ynd);

  // x is longer than y.

  small_type s = mul_signs(us, vs);

  if (s > 0) {

    if (us > 0) { // case 3

      while (y < yend)
        (*x++) |= (*y++);

      // No change for the rest of x.

    }
    else {  // case 4

      sc_digit xcarry = 1;
      sc_digit ycarry = 1;

      while (y < yend) {
        xcarry += (~(*x) & DIGIT_MASK);
        ycarry += (~(*y++) & DIGIT_MASK);
        (*x++) = (xcarry | ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        xcarry += (~(*x) & DIGIT_MASK);
        ycarry += DIGIT_MASK;
        (*x++) = (xcarry | ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }
    }

  }
  else {

    if (us > 0) { // case 5

      sc_digit ycarry = 1;

      while (y < yend) {
        ycarry += (~(*y++) & DIGIT_MASK);
        (*x) = ((*x) | ycarry) & DIGIT_MASK;
        x++;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        ycarry += DIGIT_MASK;
        (*x) = ((*x) | ycarry) & DIGIT_MASK;
        x++;
        ycarry >>= BITS_PER_DIGIT;
      }

    }
    else {  // case 6

      sc_digit xcarry = 1;

      while (y < yend) {
        xcarry += (~(*x) & DIGIT_MASK);
        (*x++) = (xcarry | (*y++)) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        xcarry += (~(*x) & DIGIT_MASK);
        (*x++) = xcarry & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
      }
    }
  }
}


// ----------------------------------------------------------------------------
//  SECTION: External functions for XOR operators.
// ----------------------------------------------------------------------------

// Handles the cases 3-5 and returns the result in u.
void
xor_on_help(small_type us, 
            int /* unb */, int und, 
            sc_digit *ud, 
            small_type vs,
            int /* vnb */, int vnd,
            const sc_digit *vd)
{
  
  sc_digit *x = ud;
  const sc_digit *y = vd;
  int xnd = und;
  int ynd = vnd;

  if (xnd < ynd)
    ynd = xnd;

  const sc_digit *xend = (x + xnd);
  const sc_digit *yend = (y + ynd);

  // x is longer than y.

  small_type s = mul_signs(us, vs);

  if (s > 0) {

    if (us > 0) { // case 3

      while (y < yend) {
        (*x) = ((*x) ^ (*y)) & DIGIT_MASK;
        x++;
        y++;
      }

      // No change for the rest of x.

    }
    else {  // case 4

      sc_digit xcarry = 1;
      sc_digit ycarry = 1;

      while (y < yend) {
        xcarry += (~(*x) & DIGIT_MASK);
        ycarry += (~(*y++) & DIGIT_MASK);
        (*x++) = (xcarry ^ ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        xcarry += (~(*x) & DIGIT_MASK);
        ycarry += DIGIT_MASK;
        (*x++) = (xcarry ^ ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }
    }
  }
  else {

    if (us > 0) { // case 5

      sc_digit ycarry = 1;

      while (y < yend) {
        ycarry += (~(*y++) & DIGIT_MASK);
        (*x) = ((*x) ^ ycarry) & DIGIT_MASK;
        x++;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        ycarry += DIGIT_MASK;
        (*x) = ((*x) ^ ycarry) & DIGIT_MASK;
        x++;
        ycarry >>= BITS_PER_DIGIT;
      }

    }
    else {  // case 6

      sc_digit xcarry = 1;

      while (y < yend) {
        xcarry += (~(*x) & DIGIT_MASK);
        (*x++) = (xcarry ^ (*y++)) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        xcarry += (~(*x) & DIGIT_MASK);
        (*x++) = xcarry & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
      }
    }
  }
}

} // namespace sc_dt


// End of file
