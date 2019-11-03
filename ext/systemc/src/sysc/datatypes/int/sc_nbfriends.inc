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

  sc_nbfriends.inc -- Friend functions for both sc_signed and sc_unsigned
                      classes. The vec_* functions are called through either
                      these functions or those in sc_nbexterns.cpp. These
                      functions perform their work on two inputs u and v, and
                      return the result object. The functions in
                      sc_nbexterns.cpp perform their work on one of their
                      inputs.

                      The functions here try to use faster algorithms in case
                      the input numbers are small. The bitwise functions (and,
                      or, and xor) need the 2's complement representations of
                      their inputs. Instead of complementing their inputs
                      first and then processing, they complement their inputs
                      while processing without allocating extra temporary
                      memory.

  Original Author: Ali Dasdan, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// ----------------------------------------------------------------------------
//  Naming conventions:
//  For sc_signed or sc_unsigned number u:
//    us : u's sign, unb : u's number of bits, 
//    und : u's number of digits, ud : u's digits array.
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
//  SECTION: Friend functions for PLUS operators.
// ----------------------------------------------------------------------------

// Handles cases 3 and 4 and returns the result.
CLASS_TYPE
ADD_HELPER(small_type us, int unb, int und,
           const sc_digit *ud, 
           small_type vs, int vnb, int vnd,
           const sc_digit *vd)
{

  und = vec_skip_leading_zeros(und, ud);
  vnd = vec_skip_leading_zeros(vnd, vd);

  int nb = sc_max(unb, vnb);
  int nd = sc_max(und, vnd) + 1;

#ifdef SC_MAX_NBITS
  test_bound(nb);
  sc_digit d[MAX_NDIGITS];
#else
  sc_digit *d = new sc_digit[nd];
#endif
  
  d[nd - 1] = d[nd - 2] = 0;

  // case 3
  if (us == vs) {

    ++nb;

    if ((und == 1) && (vnd == 1)) {
      sc_digit carry = (*ud) + (*vd);
      d[0] = carry & DIGIT_MASK;
      d[1] = carry >> BITS_PER_DIGIT;
    }

    else if (und >= vnd)
      vec_add(und, ud, vnd, vd, d);

    else
      vec_add(vnd, vd, und, ud, d);

  }

  // case 4
  else {

    int cmp_res = vec_cmp(und, ud, vnd, vd);
    
    if (cmp_res == 0) { // u == v
#ifndef SC_MAX_NBITS
      delete[] d;
#endif
      return CLASS_TYPE();
    }
    
    if (cmp_res > 0)  { // u > v

      if ((und == 1) && (vnd == 1))
        d[0] = (*ud) - (*vd);
      else
        vec_sub(und, ud, vnd, vd, d);

    }
    else { // u < v

      us = -us;

      if ((und == 1) && (vnd == 1))
        d[0] = (*vd) - (*ud);
      else
        vec_sub(vnd, vd, und, ud, d);

    }
  }
  
  return CLASS_TYPE(us, nb, nd, d);
  
}


// ----------------------------------------------------------------------------
//  SECTION: Friend functions of MULTIPLICATION operators.
// ----------------------------------------------------------------------------

// Handles the case 4 and returns the result.
CLASS_TYPE 
MUL_HELPER(small_type s, 
           int unb, int und, 
           const sc_digit *ud, 
           int vnb, int vnd,
           const sc_digit *vd)
{

  und = vec_skip_leading_zeros(und, ud);
  vnd = vec_skip_leading_zeros(vnd, vd);

  int nb = unb + vnb;
  int nd = und + vnd;
  
#ifdef SC_MAX_NBITS
  test_bound(nb);
  sc_digit d[MAX_NDIGITS];
#else
  sc_digit *d = new sc_digit[nd];
#endif

  vec_zero(nd, d);
    
  sc_digit ud0 = (*ud);
  sc_digit vd0 = (*vd);

  if ((vnd == 1) && (vd0 == 1))
    vec_copy(und, d, ud);

  else if ((und == 1) && (ud0 == 1))
    vec_copy(vnd, d, vd);

  else if ((und == 1) && (vnd == 1) && 
      (ud0 < HALF_DIGIT_RADIX) && (vd0 < HALF_DIGIT_RADIX))
    d[0] = ud0 * vd0;
  
  else if ((und == 1) && (ud0 < HALF_DIGIT_RADIX))
    vec_mul_small(vnd, vd, ud0, d);

  else if ((vnd == 1) && (vd0 < HALF_DIGIT_RADIX))
    vec_mul_small(und, ud, vd0, d);

  else if (vnd < und)
    vec_mul(und, ud, vnd, vd, d);

  else
    vec_mul(vnd, vd, und, ud, d);
  
  return CLASS_TYPE(s, nb, nd, d);

}


// ----------------------------------------------------------------------------
//  SECTION: Friend functions for DIVISION operators.
// ----------------------------------------------------------------------------

// Handles the cases 3-4 and returns the result.
CLASS_TYPE 
DIV_HELPER(small_type s, 
           int unb, int und, 
           const sc_digit *ud, 
           int vnb, int vnd,
           const sc_digit *vd)
{

  und = vec_skip_leading_zeros(und, ud);
  vnd = vec_skip_leading_zeros(vnd, vd);

  int cmp_res = vec_cmp(und, ud, vnd, vd);

  // u < v => u / v = 0 - case 4
  if (cmp_res < 0) 
    return CLASS_TYPE();

  // One extra digit for d is allocated to simplify vec_div_*().
  int nd = sc_max(und, vnd) + 1;

#ifdef SC_MAX_NBITS
  sc_digit d[MAX_NDIGITS + 1];
#else
  sc_digit *d = new sc_digit[nd];
#endif

  vec_zero(nd, d);

  sc_digit vd0 = (*vd);

  // u = v => u / v = 1 - case 3
  if (cmp_res == 0)
    d[0] = 1;

  // else if u > v - case 5

  else if ((vnd == 1) && (vd0 == 1))
    vec_copy(und, d, ud);

  else if ((vnd == 1) && (und == 1))
    d[0] = (*ud) / vd0;
 
  else if ((vnd == 1) && (vd0 < HALF_DIGIT_RADIX))
    vec_div_small(und, ud, vd0, d);

  else
    vec_div_large(und, ud, vnd, vd, d);

  return CLASS_TYPE(s, sc_max(unb, vnb), nd - 1, d);
  
}


// ----------------------------------------------------------------------------
//  SECTION: Friend functions for MOD operators.
// ----------------------------------------------------------------------------

// Handles the cases 3-4 and returns the result.
CLASS_TYPE 
MOD_HELPER(small_type us, 
           int unb, int und, 
           const sc_digit *ud, 
           int vnb, int vnd,
           const sc_digit *vd)
{

  und = vec_skip_leading_zeros(und, ud);
  vnd = vec_skip_leading_zeros(vnd, vd);

  int cmp_res = vec_cmp(und, ud, vnd, vd);

  // u = v => u % v = 0 - case 3
  if (cmp_res == 0) 
    return CLASS_TYPE();

  sc_digit vd0 = (*vd);

  if ((cmp_res > 0) && (vnd == 1) && (vd0 == 1))
    return CLASS_TYPE();

  // One extra digit for d is allocated to simplify vec_div_*().
  int nd = sc_max(und, vnd) + 1;

#ifdef SC_MAX_NBITS
  sc_digit d[MAX_NDIGITS + 1];
#else
  sc_digit *d = new sc_digit[nd];
#endif

  vec_zero(nd, d);

  // u < v => u % v = u - case 4
  if (cmp_res < 0) 
    vec_copy(und, d, ud);

  // else if u > v - case 5

  else if ((vnd == 1) && (und == 1))
    d[0] = (*ud) % vd0;

  else if ((vnd == 1) && (vd0 < HALF_DIGIT_RADIX))
    d[0] = vec_rem_small(und, ud, vd0);

  else
    vec_rem_large(und, ud, vnd, vd, d);

  us = check_for_zero(us, nd - 1, d);

  if (us == SC_ZERO) {
#ifndef SC_MAX_NBITS
    delete[] d;
#endif
    return CLASS_TYPE();
  } else
    return CLASS_TYPE(us, sc_min(unb, vnb), nd - 1, d);

}


// ----------------------------------------------------------------------------
//  SECTION: Friend functions for AND operators.
// ----------------------------------------------------------------------------

// Handles the cases 2-5 and returns the result.
CLASS_TYPE 
AND_HELPER(small_type us, 
           int unb, int und, 
           const sc_digit *ud, 
           small_type vs,
           int vnb, int vnd,
           const sc_digit *vd)
{

  int nb = sc_max(unb, vnb);
  int nd = sc_max(und, vnd);

#ifdef SC_MAX_NBITS
  sc_digit dbegin[MAX_NDIGITS];
#else
  sc_digit *dbegin = new sc_digit[nd];
#endif

  sc_digit *d = dbegin;

  const sc_digit *x;
  const sc_digit *y;
  int xnd;
  int ynd;
  small_type xs;
  small_type ys;

  if (und >= vnd) {
    x = ud;
    y = vd;
    xnd = und;
    ynd = vnd;
    xs = us;
    ys = vs;
  }
  else {
    y = ud;
    x = vd;
    ynd = und;
    xnd = vnd;
    ys = us;
    xs = vs;
  }

  const sc_digit *xend = (x + xnd);
  const sc_digit *yend = (y + ynd);

  // x is longer than y.

  small_type s = mul_signs(xs, ys);

  if (s > 0) {

    if (xs > 0) { // case 2

      while (y < yend)
        (*d++) = (*x++) & (*y++);

      while (x++ < xend) 
        (*d++) = 0;

    }
    else {  // case 3

      sc_digit xcarry = 1;
      sc_digit ycarry = 1;

      while (y < yend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        ycarry += (~(*y++) & DIGIT_MASK);
        (*d++) = (xcarry & ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        ycarry += DIGIT_MASK;
        (*d++) = (xcarry & ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }

    }
  }
  else {

    if (xs > 0) { // case 4

      sc_digit ycarry = 1;

      while (y < yend) {
        ycarry += (~(*y++) & DIGIT_MASK);
        (*d++) = ((*x++) & ycarry) & DIGIT_MASK;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        ycarry += DIGIT_MASK;
        (*d++) = ((*x++) & ycarry) & DIGIT_MASK;
        ycarry >>= BITS_PER_DIGIT;
      }

    }
    else {  // case 5

      sc_digit xcarry = 1;

      while (y < yend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        (*d++) = (xcarry & (*y++)) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
      }

      while (x++ < xend)
        (*d++) = 0;

    }
  }

  s = convert_signed_2C_to_SM(nb, nd, dbegin);

  return CLASS_TYPE(s, nb, nd, dbegin);  

}


// ----------------------------------------------------------------------------
//  SECTION: Friend functions for OR operators.
// ----------------------------------------------------------------------------

// Handles the cases 3-5 and returns the result.
CLASS_TYPE 
OR_HELPER(small_type us, 
          int unb, int und, 
          const sc_digit *ud, 
          small_type vs,
          int vnb, int vnd,
          const sc_digit *vd)
{
  
  int nb = sc_max(unb, vnb);
  int nd = sc_max(und, vnd);

#ifdef SC_MAX_NBITS
  sc_digit dbegin[MAX_NDIGITS];
#else
  sc_digit *dbegin = new sc_digit[nd];
#endif

  sc_digit *d = dbegin;

  const sc_digit *x;
  const sc_digit *y;
  int xnd;
  int ynd;
  small_type xs;
  small_type ys;

  if (und >= vnd) {
    x = ud;
    y = vd;
    xnd = und;
    ynd = vnd;
    xs = us;
    ys = vs;
  }
  else {
    y = ud;
    x = vd;
    ynd = und;
    xnd = vnd;
    ys = us;
    xs = vs;
  }

  const sc_digit *xend = (x + xnd);
  const sc_digit *yend = (y + ynd);

  // x is longer than y.

  small_type s = mul_signs(xs, ys);

  if (s > 0) {

    if (xs > 0) { // case 3

      while (y < yend)
        (*d++) = (*x++) | (*y++);

      while (x < xend)
        (*d++) = (*x++);

    }
    else {  // case 4

      sc_digit xcarry = 1;
      sc_digit ycarry = 1;

      while (y < yend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        ycarry += (~(*y++) & DIGIT_MASK);
        (*d++) = (xcarry | ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        ycarry += DIGIT_MASK;
        (*d++) = (xcarry | ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }

    }

  }
  else {

    if (xs > 0) { // case 5

      sc_digit ycarry = 1;

      while (y < yend) {
        ycarry += (~(*y++) & DIGIT_MASK);
        (*d++) = ((*x++) | ycarry) & DIGIT_MASK;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        ycarry += DIGIT_MASK;
        (*d++) = ((*x++) | ycarry) & DIGIT_MASK;
        ycarry >>= BITS_PER_DIGIT;
      }

    }
    else {  // case 6

      sc_digit xcarry = 1;

      while (y < yend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        (*d++) = (xcarry | (*y++)) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        (*d++) = xcarry & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
      }
    }

  }

  s = convert_signed_2C_to_SM(nb, nd, dbegin);

  return CLASS_TYPE(s, nb, nd, dbegin);

}


// ----------------------------------------------------------------------------
//  SECTION: Friend functions for XOR operators.
// ----------------------------------------------------------------------------

// Handles the cases 3-5 and returns the result.
CLASS_TYPE 
XOR_HELPER(small_type us, 
           int unb, int und, 
           const sc_digit *ud, 
           small_type vs,
           int vnb, int vnd,
           const sc_digit *vd)
{
  
  int nb = sc_max(unb, vnb);
  int nd = sc_max(und, vnd);

#ifdef SC_MAX_NBITS
  sc_digit dbegin[MAX_NDIGITS];
#else
  sc_digit *dbegin = new sc_digit[nd];
#endif

  sc_digit *d = dbegin;

  const sc_digit *x;
  const sc_digit *y;
  int xnd;
  int ynd;
  small_type xs;
  small_type ys;

  if (und >= vnd) {
    x = ud;
    y = vd;
    xnd = und;
    ynd = vnd;
    xs = us;
    ys = vs;
  }
  else {
    y = ud;
    x = vd;
    ynd = und;
    xnd = vnd;
    ys = us;
    xs = vs;
  }

  const sc_digit *xend = (x + xnd);
  const sc_digit *yend = (y + ynd);

  // x is longer than y.

  small_type s = mul_signs(xs, ys);

  if (s > 0) {

    if (xs > 0) { // case 3

      while (y < yend)
        (*d++) = ((*x++) ^ (*y++)) & DIGIT_MASK;

      while (x < xend)
        (*d++) = (*x++);

    }
    else {  // case 4

      sc_digit xcarry = 1;
      sc_digit ycarry = 1;

      while (y < yend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        ycarry += (~(*y++) & DIGIT_MASK);
        (*d++) = (xcarry ^ ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        ycarry += DIGIT_MASK;
        (*d++) = (xcarry ^ ycarry) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
        ycarry >>= BITS_PER_DIGIT;
      }

    }
  }
  else {

    if (xs > 0) { // case 5

      sc_digit ycarry = 1;

      while (y < yend) {
        ycarry += (~(*y++) & DIGIT_MASK);
        (*d++) = ((*x++) ^ ycarry) & DIGIT_MASK;
        ycarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        ycarry += DIGIT_MASK;
        (*d++) = ((*x++) ^ ycarry) & DIGIT_MASK;
        ycarry >>= BITS_PER_DIGIT;
      }

    }
    else {  // case 6

      sc_digit xcarry = 1;

      while (y < yend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        (*d++) = (xcarry ^ (*y++)) & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
      }

      while (x < xend) {
        xcarry += (~(*x++) & DIGIT_MASK);
        (*d++) = xcarry & DIGIT_MASK;
        xcarry >>= BITS_PER_DIGIT;
      }
    }
  }

  s = convert_signed_2C_to_SM(nb, nd, dbegin);

  return CLASS_TYPE(s, nb, nd, dbegin);

}

// End of file.

