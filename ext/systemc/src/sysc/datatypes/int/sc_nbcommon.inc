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

  sc_nbcommon.cpp -- Functions common to both sc_signed and sc_unsigned.
                     This file is included in sc_signed.cpp and
                     sc_unsigned.cpp after the macros are defined accordingly.
                     For example, sc_signed.cpp will first define CLASS_TYPE
                     as sc_signed before including this file. This file like
                     sc_nbfriends.cpp and sc_nbexterns.cpp is created in order
                     to ensure only one version of each function, regardless
                     of the class that they interface to.
 
  Original Author: Ali Dasdan, Synopsys, Inc.
  
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// ----------------------------------------------------------------------------
//  SECTION : Public members
// ----------------------------------------------------------------------------

// Create a CLASS_TYPE number with nb bits.
CLASS_TYPE::CLASS_TYPE( int nb ) :
    sc_value_base(), sgn(), nbits(), ndigits(), digit()
{
    sgn = default_sign();
    if( nb > 0 ) {
	nbits = num_bits( nb );
    } else {
	char msg[BUFSIZ];
	std::sprintf( msg, "%s::%s( int nb ) : nb = %d is not valid",
		 CLASS_TYPE_STR, CLASS_TYPE_STR, nb );
	SC_REPORT_ERROR( sc_core::SC_ID_INIT_FAILED_, msg );
    }
    ndigits = DIV_CEIL(nbits);
#ifdef SC_MAX_NBITS
    test_bound(nb);
#else
    digit = new sc_digit[ndigits];
#endif
    makezero();
}


// Create a copy of v with sgn s. v is of the same type.
CLASS_TYPE::CLASS_TYPE(const CLASS_TYPE& v) :
    sc_value_base(v), sgn(v.sgn), nbits(v.nbits), ndigits(v.ndigits), digit()
{
#ifndef SC_MAX_NBITS
  digit = new sc_digit[ndigits];
#endif

  vec_copy(ndigits, digit, v.digit);
}


// Create a copy of v where v is of the different type.
CLASS_TYPE::CLASS_TYPE(const OTHER_CLASS_TYPE& v) :
    sc_value_base(v), sgn(v.sgn), nbits(num_bits(v.nbits)), ndigits(), digit()
{
#if (IF_SC_SIGNED == 1)
  ndigits = v.ndigits;
#else
  ndigits = DIV_CEIL(nbits);
#endif

#ifndef SC_MAX_NBITS
  digit = new sc_digit[ndigits];
#endif

  copy_digits(v.nbits, v.ndigits, v.digit);
}

// Create a copy of v where v is an sign-less instance.
CLASS_TYPE::CLASS_TYPE(const sc_bv_base& v) :
    sc_value_base(), sgn(), nbits(), ndigits(), digit()
{
    int nb = v.length();
    sgn = default_sign();
    if( nb > 0 ) {
        nbits = num_bits( nb );
    } else {
        char msg[BUFSIZ];
        std::sprintf( msg, "%s::%s( sc_bv_base ) : nb = %d is not valid",
                 CLASS_TYPE_STR, CLASS_TYPE_STR, nb );
        SC_REPORT_ERROR( sc_core::SC_ID_INIT_FAILED_, msg );
    }
    ndigits = DIV_CEIL(nbits);
#   ifdef SC_MAX_NBITS
        test_bound(nb);
#    else
        digit = new sc_digit[ndigits];
#    endif
    makezero();
    *this = v;
}

CLASS_TYPE::CLASS_TYPE(const sc_lv_base& v) : 
    sc_value_base(), sgn(), nbits(), ndigits(), digit()
{
    int nb = v.length();
    sgn = default_sign();
    if( nb > 0 ) {
        nbits = num_bits( nb );
    } else {
        char msg[BUFSIZ];
        std::sprintf( msg, "%s::%s( sc_lv_base ) : nb = %d is not valid",
                 CLASS_TYPE_STR, CLASS_TYPE_STR, nb );
        SC_REPORT_ERROR( sc_core::SC_ID_INIT_FAILED_, msg );
    }
    ndigits = DIV_CEIL(nbits);
#   ifdef SC_MAX_NBITS
        test_bound(nb);
#    else
        digit = new sc_digit[ndigits];
#    endif
    makezero();
    *this = v;
}

CLASS_TYPE::CLASS_TYPE(const sc_int_subref_r& v) :
    sc_value_base(v), sgn(), nbits(), ndigits(), digit()
{
    int nb = v.length();
    sgn = default_sign();
    if( nb > 0 ) {
        nbits = num_bits( nb );
    } else {
        char msg[BUFSIZ];
        std::sprintf( msg, "%s::%s( sc_int_subref ) : nb = %d is not valid",
                 CLASS_TYPE_STR, CLASS_TYPE_STR, nb );
        SC_REPORT_ERROR( sc_core::SC_ID_INIT_FAILED_, msg );
    }
    ndigits = DIV_CEIL(nbits);
#   ifdef SC_MAX_NBITS
        test_bound(nb);
#    else
        digit = new sc_digit[ndigits];
#    endif
    makezero();
    *this = v.to_uint64();
}

CLASS_TYPE::CLASS_TYPE(const sc_uint_subref_r& v) :
    sc_value_base(v), sgn(), nbits(), ndigits(), digit()
{
    int nb = v.length();
    sgn = default_sign();
    if( nb > 0 ) {
        nbits = num_bits( nb );
    } else {
        char msg[BUFSIZ];
        std::sprintf( msg, "%s::%s( sc_uint_subref ) : nb = %d is not valid",
                 CLASS_TYPE_STR, CLASS_TYPE_STR, nb );
        SC_REPORT_ERROR( sc_core::SC_ID_INIT_FAILED_, msg );
    }
    ndigits = DIV_CEIL(nbits);
#   ifdef SC_MAX_NBITS
        test_bound(nb);
#    else
        digit = new sc_digit[ndigits];
#    endif
    makezero();
    *this = v.to_uint64();
}

CLASS_TYPE::CLASS_TYPE(const sc_signed_subref_r& v) :
    sc_value_base(v), sgn(), nbits(), ndigits(), digit()
{
    int nb = v.length();
    sgn = default_sign();
    if( nb > 0 ) {
        nbits = num_bits( nb );
    } else {
        char msg[BUFSIZ];
        std::sprintf( msg, "%s::%s( sc_signed_subref ) : nb = %d is not valid",
                 CLASS_TYPE_STR, CLASS_TYPE_STR, nb );
        SC_REPORT_ERROR( sc_core::SC_ID_INIT_FAILED_, msg );
    }
    ndigits = DIV_CEIL(nbits);
#   ifdef SC_MAX_NBITS
        test_bound(nb);
#    else
        digit = new sc_digit[ndigits];
#    endif
    makezero();
    *this = sc_unsigned(v.m_obj_p, v.m_left, v.m_right);
}

CLASS_TYPE::CLASS_TYPE(const sc_unsigned_subref_r& v) :
    sc_value_base(v), sgn(), nbits(), ndigits(), digit()
{
    int nb = v.length();
    sgn = default_sign();
    if( nb > 0 ) {
        nbits = num_bits( nb );
    } else {
        char msg[BUFSIZ];
        std::sprintf( msg, "%s::%s( sc_unsigned_subref ) : nb = %d is not valid",
                 CLASS_TYPE_STR, CLASS_TYPE_STR, nb );
        SC_REPORT_ERROR( sc_core::SC_ID_INIT_FAILED_, msg );
    }
    ndigits = DIV_CEIL(nbits);
#   ifdef SC_MAX_NBITS
        test_bound(nb);
#    else
        digit = new sc_digit[ndigits];
#    endif
    makezero();
    *this = sc_unsigned(v.m_obj_p, v.m_left, v.m_right);
}

// ----------------------------------------------------------------------------
//  SECTION: Public members - Concatenation support.
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
//  SECTION: Public members - Assignment operators.
// ----------------------------------------------------------------------------

// Assignment from v of the same type.
const CLASS_TYPE&
CLASS_TYPE::operator=(const CLASS_TYPE& v)
{
  if (this != &v) {

    sgn = v.sgn;

    if (sgn == SC_ZERO)
      vec_zero(ndigits, digit);

    else
      copy_digits(v.nbits, v.ndigits, v.digit);

  }

  return *this;
}


// Assignment from v of the different type.
const CLASS_TYPE&
CLASS_TYPE::operator=(const OTHER_CLASS_TYPE& v)
{
  sgn = v.sgn;

  if (sgn == SC_ZERO)
    vec_zero(ndigits, digit);

  else
    copy_digits(v.nbits, v.ndigits, v.digit);
    
  return *this;
}


// Assignment from an sc_unsigned_subref_r
const CLASS_TYPE& 
CLASS_TYPE::operator=(const sc_unsigned_subref_r& v)
{
  return operator=(sc_unsigned(v));
}


// Assignment from an sc_signed_subref_r
const CLASS_TYPE& 
CLASS_TYPE::operator=(const sc_signed_subref_r& v)
{
  return operator=(sc_unsigned(v));
}


// ----------------------------------------------------------------------------
//  SECTION: Input and output operators
// ----------------------------------------------------------------------------

void
CLASS_TYPE::scan( ::std::istream& is )
{
    std::string s;
    is >> s;
    *this = s.c_str();
}


// ----------------------------------------------------------------------------
//  SECTION: PLUS operators: +, +=, ++
// ----------------------------------------------------------------------------

// Cases to consider when computing u + v:
// 1. 0 + v = v
// 2. u + 0 = u
// 3. if sgn(u) == sgn(v)
//    3.1 u + v = +(u + v) = sgn(u) * (u + v) 
//    3.2 (-u) + (-v) = -(u + v) = sgn(u) * (u + v)
// 4. if sgn(u) != sgn(v)
//    4.1 u + (-v) = u - v = sgn(u) * (u - v)
//    4.2 (-u) + v = -(u - v) ==> sgn(u) * (u - v)
//
// Specialization of above cases for computing ++u or u++: 
// 1. 0 + 1 = 1
// 3. u + 1 = u + 1 = sgn(u) * (u + 1)
// 4. (-u) + 1 = -(u - 1) = sgn(u) * (u - 1)

const CLASS_TYPE&
CLASS_TYPE::operator+=(const CLASS_TYPE& v)
{
  // u = *this

  if (sgn == SC_ZERO) // case 1
    return (*this = v);

  if (v.sgn == SC_ZERO) // case 2
    return *this;

  // cases 3 and 4
  add_on_help(sgn, nbits, ndigits, digit,
              v.sgn, v.nbits, v.ndigits, v.digit);

  convert_SM_to_2C_to_SM();

  return *this; 
}


const CLASS_TYPE&
CLASS_TYPE::operator+=(const OTHER_CLASS_TYPE& v)
{
  // u = *this

  if (sgn == SC_ZERO) // case 1
    return (*this = v);

  if (v.sgn == SC_ZERO) // case 2
    return *this;

  // cases 3 and 4
  add_on_help(sgn, nbits, ndigits, digit,
              v.sgn, v.nbits, v.ndigits, v.digit);

  convert_SM_to_2C_to_SM();

  return *this; 
}


CLASS_TYPE&
CLASS_TYPE::operator++() // prefix
{
    *this = *this + 1;
    return *this;   
}


const CLASS_TYPE
CLASS_TYPE::operator++(int) // postfix
{
  // Copy digit into d.

#ifdef SC_MAX_NBITS
  sc_digit d[MAX_NDIGITS];
#else
  sc_digit *d = new sc_digit[ndigits];
#endif

  small_type s = sgn;

  vec_copy(ndigits, d, digit);

  *this = *this + 1;

  return CLASS_TYPE(s, nbits, ndigits, d);
}


const CLASS_TYPE&
CLASS_TYPE::operator+=(int64 v)
{
  // u = *this

  if (sgn == SC_ZERO)  // case 1
    return (*this = v);

  if (v == 0) // case 2
    return *this;

  CONVERT_INT64(v);

  // cases 3 and 4
  add_on_help(sgn, nbits, ndigits, digit,
              vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  convert_SM_to_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator+=(uint64 v)
{
  // u = *this

  if (sgn == SC_ZERO)  // case 1
    return (*this = v);

  if (v == 0)  // case 2
    return *this;

  CONVERT_INT64(v);

  // cases 3 and 4
  add_on_help(sgn, nbits, ndigits, digit,
              vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  convert_SM_to_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator+=(long v)
{
  // u = *this

  if (sgn == SC_ZERO)  // case 1
    return (*this = v);

  if (v == 0) // case 2
    return *this;

  CONVERT_LONG(v);

  // cases 3 and 4
  add_on_help(sgn, nbits, ndigits, digit,
              vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  convert_SM_to_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator+=(unsigned long v)
{
  // u = *this

  if (sgn == SC_ZERO)  // case 1
    return (*this = v);

  if (v == 0)  // case 2
    return *this;

  CONVERT_LONG(v);

  // cases 3 and 4
  add_on_help(sgn, nbits, ndigits, digit,
              vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  convert_SM_to_2C_to_SM();

  return *this;
}


// ----------------------------------------------------------------------------
//  SECTION: MINUS operators: -, -=, --
// ----------------------------------------------------------------------------

// Cases to consider when computing u + v:
// 1. u - 0 = u 
// 2. 0 - v = -v
// 3. if sgn(u) != sgn(v)
//    3.1 u - (-v) = u + v = sgn(u) * (u + v)
//    3.2 (-u) - v = -(u + v) ==> sgn(u) * (u + v)
// 4. if sgn(u) == sgn(v)
//    4.1 u - v = +(u - v) = sgn(u) * (u - v) 
//    4.2 (-u) - (-v) = -(u - v) = sgn(u) * (u - v)
//
// Specialization of above cases for computing --u or u--: 
// 1. 0 - 1 = -1
// 3. (-u) - 1 = -(u + 1) = sgn(u) * (u + 1)
// 4. u - 1 = u - 1 = sgn(u) * (u - 1)

const CLASS_TYPE&
CLASS_TYPE::operator-=(const CLASS_TYPE& v)
{
  // u = *this

  if (v.sgn == SC_ZERO)  // case 1
    return *this;

  if (sgn == SC_ZERO) { // case 2

    sgn = -v.sgn;
    copy_digits(v.nbits, v.ndigits, v.digit);   

  }
  else {
    
    // cases 3 and 4
    add_on_help(sgn, nbits, ndigits, digit, 
                -v.sgn, v.nbits, v.ndigits, v.digit);

    convert_SM_to_2C_to_SM();

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator-=(const OTHER_CLASS_TYPE& v)
{
  // u = *this

  if (v.sgn == SC_ZERO)  // case 1
    return *this;

  if (sgn == SC_ZERO) { // case 2

    sgn = -v.sgn;
    copy_digits(v.nbits, v.ndigits, v.digit);   

  }
  else {

    // cases 3 and 4
    add_on_help(sgn, nbits, ndigits, digit, 
                -v.sgn, v.nbits, v.ndigits, v.digit);

    convert_SM_to_2C_to_SM();
    
  }
  
  return *this;
}


CLASS_TYPE&
CLASS_TYPE::operator--() // prefix
{
    *this = *this - 1;
    return *this;   
}


const CLASS_TYPE
CLASS_TYPE::operator--(int) // postfix
{
  // Copy digit into d.

#ifdef SC_MAX_NBITS
  sc_digit d[MAX_NDIGITS];
#else
  sc_digit *d = new sc_digit[ndigits];
#endif

  small_type s = sgn;

  vec_copy(ndigits, d, digit);

  *this = *this - 1;

  return CLASS_TYPE(s, nbits, ndigits, d);
}


const CLASS_TYPE&
CLASS_TYPE::operator-=(int64 v)
{
  // u = *this

  if (v == 0) // case 1
    return *this;

  if (sgn == SC_ZERO) // case 2
    return (*this = -v);

  CONVERT_INT64(v);

  // cases 3 and 4
  add_on_help(sgn, nbits, ndigits, digit,
              -vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  convert_SM_to_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator-=(uint64 v)
{
  // u = *this

  if (v == 0) // case 1
    return *this;

  int64 v2 = (int64) v;

  if (sgn == SC_ZERO) // case 2
    return (*this = -v2);

  CONVERT_INT64(v);

  // cases 3 and 4
  add_on_help(sgn, nbits, ndigits, digit,
              -vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  convert_SM_to_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator-=(long v)
{
  // u = *this

  if (v == 0) // case 1
    return *this;

  if (sgn == SC_ZERO) // case 2
    return (*this = -v);

  CONVERT_LONG(v);

  // cases 3 and 4
  add_on_help(sgn, nbits, ndigits, digit,
              -vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  convert_SM_to_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator-=(unsigned long v)
{
  // u = *this

  if (v == 0) // case 1
    return *this;

  long v2 = (long) v;

  if (sgn == SC_ZERO) // case 2
    return (*this = -v2);

  CONVERT_LONG(v);

  // cases 3 and 4
  add_on_help(sgn, nbits, ndigits, digit,
              -vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  convert_SM_to_2C_to_SM();

  return *this;
}


// ----------------------------------------------------------------------------
//  SECTION: MULTIPLICATION operators: *, *=
// ----------------------------------------------------------------------------

// Cases to consider when computing u * v:
// 1. u * 0 = 0 * v = 0
// 2. 1 * v = v and -1 * v = -v
// 3. u * 1 = u and u * -1 = -u
// 4. u * v = u * v

const CLASS_TYPE&
CLASS_TYPE::operator*=(const CLASS_TYPE& v)
{
  // u = *this

  sgn = mul_signs(sgn, v.sgn);

  if (sgn == SC_ZERO) // case 1
    vec_zero(ndigits, digit);

  else
    // cases 2-4
    MUL_ON_HELPER(sgn,  nbits, ndigits, digit,
                  v.nbits, v.ndigits, v.digit);

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator*=(const OTHER_CLASS_TYPE& v)
{
  // u = *this

  sgn = mul_signs(sgn, v.sgn);

  if (sgn == SC_ZERO) // case 1
    vec_zero(ndigits, digit);

  else 
    // cases 2-4
    MUL_ON_HELPER(sgn,  nbits, ndigits, digit,
                  v.nbits, v.ndigits, v.digit);

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator*=(int64 v)
{
  // u = *this

  sgn = mul_signs(sgn, get_sign(v));

  if (sgn == SC_ZERO) // case 1
    vec_zero(ndigits, digit);

  else {  // cases 2-4

    CONVERT_INT64_2(v);

    MUL_ON_HELPER(sgn,  nbits, ndigits, digit,
                  BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator*=(uint64 v)
{
  // u = *this

  sgn = mul_signs(sgn, get_sign(v));

  if (sgn == SC_ZERO) // case 1
    vec_zero(ndigits, digit);

  else { // cases 2-4

    CONVERT_INT64_2(v);

    MUL_ON_HELPER(sgn,  nbits, ndigits, digit,
                  BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator*=(long v)
{
  // u = *this

  sgn = mul_signs(sgn, get_sign(v));

  if (sgn == SC_ZERO) // case 1
    vec_zero(ndigits, digit);

  else {   // cases 2-4

    CONVERT_LONG_2(v);

    MUL_ON_HELPER(sgn,  nbits, ndigits, digit,
                  BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator*=(unsigned long v)
{
  // u = *this

  sgn = mul_signs(sgn, get_sign(v));

  if (sgn == SC_ZERO) // case 1
    vec_zero(ndigits, digit);

  else {  // cases 2-4

    CONVERT_LONG_2(v);

    MUL_ON_HELPER(sgn,  nbits, ndigits, digit,
                  BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  }

  return *this;
}


// ----------------------------------------------------------------------------
//  SECTION: DIVISION operators: /, /=
// ----------------------------------------------------------------------------

// Cases to consider when finding the quotient q = floor(u/v):
// Note that u = q * v + r for r < q.
// 1. 0 / 0 or u / 0 => error
// 2. 0 / v => 0 = 0 * v + 0
// 3. u / v && u = v => u = 1 * u + 0  - u or v can be 1 or -1
// 4. u / v && u < v => u = 0 * v + u  - u can be 1 or -1
// 5. u / v && u > v => u = q * v + r  - v can be 1 or -1

const CLASS_TYPE&
CLASS_TYPE::operator/=(const CLASS_TYPE& v)
{
  sgn = mul_signs(sgn, v.sgn);

  if (sgn == SC_ZERO) {

    div_by_zero(v.sgn); // case 1
    vec_zero(ndigits, digit);  // case 2

  }
  else  // other cases
    DIV_ON_HELPER(sgn, nbits, ndigits, digit,
                  v.nbits, v.ndigits, v.digit);

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator/=(const OTHER_CLASS_TYPE& v)
{
  sgn = mul_signs(sgn, v.sgn);

  if (sgn == SC_ZERO) {

    div_by_zero(v.sgn); // case 1
    vec_zero(ndigits, digit);  // case 2

  }
  else  // other cases
    DIV_ON_HELPER(sgn, nbits, ndigits, digit,
                  v.nbits, v.ndigits, v.digit);

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator/=(int64 v)
{
  // u = *this

  sgn = mul_signs(sgn, get_sign(v));

  if (sgn == SC_ZERO) {

    div_by_zero(v);  // case 1
    vec_zero(ndigits, digit); // case 2

  }
  else {

    CONVERT_INT64_2(v);

    // other cases
    DIV_ON_HELPER(sgn, nbits, ndigits, digit,
                  BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator/=(uint64 v)
{
  // u = *this

  sgn = mul_signs(sgn, get_sign(v));

  if (sgn == SC_ZERO) {

    div_by_zero(v);  // case 1
    vec_zero(ndigits, digit);  // case 2

  }
  else {

    CONVERT_INT64_2(v);

    // other cases
    DIV_ON_HELPER(sgn, nbits, ndigits, digit,
                  BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator/=(long v)
{
  // u = *this

  sgn = mul_signs(sgn, get_sign(v));

  if (sgn == SC_ZERO) {

    div_by_zero(v);  // case 1
    vec_zero(ndigits, digit); // case 2

  }
  else {

    CONVERT_LONG_2(v);

    // other cases
    DIV_ON_HELPER(sgn,  nbits, ndigits, digit,
                  BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator/=(unsigned long v)
{
  // u = *this

  sgn = mul_signs(sgn, get_sign(v));

  if (sgn == SC_ZERO) {

    div_by_zero(v);  // case 1
    vec_zero(ndigits, digit);  // case 2

  }
  else {
    
    CONVERT_LONG_2(v);

    // other cases
    DIV_ON_HELPER(sgn,  nbits, ndigits, digit,
                  BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  }

  return *this;
}


// ----------------------------------------------------------------------------
//  SECTION: MOD operators: %, %=.
// ----------------------------------------------------------------------------

// Cases to consider when finding the remainder r = u % v:
// Note that u = q * v + r for r < q.
// 1. 0 % 0 or u % 0 => error
// 2. 0 % v => 0 = 0 * v + 0
// 3. u % v && u = v => u = 1 * u + 0  - u or v can be 1 or -1
// 4. u % v && u < v => u = 0 * v + u  - u can be 1 or -1
// 5. u % v && u > v => u = q * v + r  - v can be 1 or -1

const CLASS_TYPE&
CLASS_TYPE::operator%=(const CLASS_TYPE& v)
{
  if ((sgn == SC_ZERO) || (v.sgn == SC_ZERO)) {

    div_by_zero(v.sgn);  // case 1
    vec_zero(ndigits, digit);  // case 2

  }
  else // other cases
    MOD_ON_HELPER(sgn, nbits, ndigits, digit,
                  v.nbits, v.ndigits, v.digit);

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator%=(const OTHER_CLASS_TYPE& v)
{
  if ((sgn == SC_ZERO) || (v.sgn == SC_ZERO)) {

    div_by_zero(v.sgn);  // case 1
    vec_zero(ndigits, digit);  // case 2

  }
  else  // other cases
    MOD_ON_HELPER(sgn, nbits, ndigits, digit,
                  v.nbits, v.ndigits, v.digit);

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator%=(int64 v)
{
  small_type vs = get_sign(v);

  if ((sgn == SC_ZERO) || (vs == SC_ZERO)) {

    div_by_zero(v);  // case 1
    vec_zero(ndigits, digit);  // case 2

  }
  else {
    
    CONVERT_INT64_2(v);

    // other cases
    MOD_ON_HELPER(sgn, nbits, ndigits, digit,
                  BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator%=(uint64 v)
{
  if ((sgn == SC_ZERO) || (v == 0)) {

    div_by_zero(v);  // case 1
    vec_zero(ndigits, digit);  // case 2

  }
  else {
    
    CONVERT_INT64_2(v);

    // other cases
    MOD_ON_HELPER(sgn, nbits, ndigits, digit,
                  BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator%=(long v)
{
  small_type vs = get_sign(v);

  if ((sgn == SC_ZERO) || (vs == SC_ZERO)) {

    div_by_zero(v);  // case 1
    vec_zero(ndigits, digit);  // case 2

  }
  else {
    
    CONVERT_LONG_2(v);

    // other cases
    MOD_ON_HELPER(sgn, nbits, ndigits, digit,
                  BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator%=(unsigned long v)
{
  if ((sgn == SC_ZERO) || (v == 0)) {

    div_by_zero(v);  // case 1
    vec_zero(ndigits, digit);  // case 2

  }
  else {
    
    CONVERT_LONG_2(v);

    // other cases
    MOD_ON_HELPER(sgn, nbits, ndigits, digit,
                  BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  }

  return *this;
}


// ----------------------------------------------------------------------------
//  SECTION: Bitwise AND operators: &, &=
// ----------------------------------------------------------------------------

// Cases to consider when computing u & v:
// 1. u & 0 = 0 & v = 0
// 2. u & v => sgn = +
// 3. (-u) & (-v) => sgn = -
// 4. u & (-v) => sgn = +
// 5. (-u) & v => sgn = +

const CLASS_TYPE&
CLASS_TYPE::operator&=(const CLASS_TYPE& v)
{
  // u = *this

  if ((sgn == SC_ZERO) || (v.sgn == SC_ZERO)) // case 1
    makezero();

  else  {  // other cases

    and_on_help(sgn, nbits, ndigits, digit,
                v.sgn, v.nbits, v.ndigits, v.digit);

    convert_2C_to_SM();

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator&=(const OTHER_CLASS_TYPE& v)
{
  // u = *this

  if ((sgn == SC_ZERO) || (v.sgn == SC_ZERO)) // case 1
    makezero();

  else {  // other cases
    
    and_on_help(sgn, nbits, ndigits, digit,
                v.sgn, v.nbits, v.ndigits, v.digit);

    convert_2C_to_SM();

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator&=(int64 v)
{
  // u = *this

  if ((sgn == SC_ZERO) || (v == 0)) // case 1
    makezero();

  else {    // other cases

    CONVERT_INT64(v);

    and_on_help(sgn, nbits, ndigits, digit,
                vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

    convert_2C_to_SM();

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator&=(uint64 v)
{
  // u = *this

  if ((sgn == SC_ZERO) || (v == 0))  // case 1
    makezero();

  else {  // other cases

    CONVERT_INT64(v);

    and_on_help(sgn, nbits, ndigits, digit,
                vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

    convert_2C_to_SM();
    
  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator&=(long v)
{
  // u = *this

  if ((sgn == SC_ZERO) || (v == 0))  // case 1
    makezero();

  else {      // other cases

    CONVERT_LONG(v);

    and_on_help(sgn, nbits, ndigits, digit,
                vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

    convert_2C_to_SM();

  }

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator&=(unsigned long v)
{
  // u = *this

  if ((sgn == SC_ZERO) || (v == 0))  // case 1
    makezero();

  else {  // other cases

    CONVERT_LONG(v);

    and_on_help(sgn, nbits, ndigits, digit,
                vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

    convert_2C_to_SM();

  }

  return *this;
}


// ----------------------------------------------------------------------------
//  SECTION: Bitwise OR operators: |, |=
// ----------------------------------------------------------------------------

// Cases to consider when computing u | v:
// 1. u | 0 = u
// 2. 0 | v = v
// 3. u | v => sgn = +
// 4. (-u) | (-v) => sgn = -
// 5. u | (-v) => sgn = -
// 6. (-u) | v => sgn = -

const CLASS_TYPE&
CLASS_TYPE::operator|=(const CLASS_TYPE& v)
{
  if (v.sgn == SC_ZERO)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases
  or_on_help(sgn, nbits, ndigits, digit,
             v.sgn, v.nbits, v.ndigits, v.digit);

  convert_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator|=(const OTHER_CLASS_TYPE& v)
{
  if (v.sgn == SC_ZERO)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases
  or_on_help(sgn, nbits, ndigits, digit,
             v.sgn, v.nbits, v.ndigits, v.digit);

  convert_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator|=(int64 v)
{
  if (v == 0)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases

  CONVERT_INT64(v);

  or_on_help(sgn, nbits, ndigits, digit,
             vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  convert_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator|=(uint64 v)
{
  if (v == 0)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases

  CONVERT_INT64(v);

  or_on_help(sgn, nbits, ndigits, digit,
             vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  convert_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator|=(long v)
{
  if (v == 0)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases

  CONVERT_LONG(v);

  or_on_help(sgn, nbits, ndigits, digit,
             vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  convert_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator|=(unsigned long v)
{
  if (v == 0)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases

  CONVERT_LONG(v);

  or_on_help(sgn, nbits, ndigits, digit,
             vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  convert_2C_to_SM();

  return *this;
}


// ----------------------------------------------------------------------------
//  SECTION: Bitwise XOR operators: ^, ^=
// ----------------------------------------------------------------------------

// Cases to consider when computing u ^ v:
// Note that  u ^ v = (~u & v) | (u & ~v).
// 1. u ^ 0 = u
// 2. 0 ^ v = v
// 3. u ^ v => sgn = +
// 4. (-u) ^ (-v) => sgn = -
// 5. u ^ (-v) => sgn = -
// 6. (-u) ^ v => sgn = +

const CLASS_TYPE&
CLASS_TYPE::operator^=(const CLASS_TYPE& v)
{
  // u = *this

  if (v.sgn == SC_ZERO)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases
  xor_on_help(sgn, nbits, ndigits, digit,
              v.sgn, v.nbits, v.ndigits, v.digit);

  convert_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator^=(const OTHER_CLASS_TYPE& v)
{
  // u = *this

  if (v.sgn == SC_ZERO)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases
  xor_on_help(sgn, nbits, ndigits, digit,
              v.sgn, v.nbits, v.ndigits, v.digit);

  convert_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator^=(int64 v)
{
  if (v == 0)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases

  CONVERT_INT64(v);

  xor_on_help(sgn, nbits, ndigits, digit,
              vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd);

  convert_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator^=(uint64 v)
{
  if (v == 0)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases

  CONVERT_INT64(v);

  xor_on_help(sgn, nbits, ndigits, digit,
              vs, BITS_PER_UINT64, DIGITS_PER_UINT64, vd);
  
  convert_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator^=(long v)
{
  if (v == 0)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases

  CONVERT_LONG(v);

  xor_on_help(sgn, nbits, ndigits, digit,
              vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd);

  convert_2C_to_SM();

  return *this;
}


const CLASS_TYPE&
CLASS_TYPE::operator^=(unsigned long v)
{
  if (v == 0)  // case 1
    return *this;

  if (sgn == SC_ZERO)  // case 2
    return (*this = v);

  // other cases

  CONVERT_LONG(v);

  xor_on_help(sgn, nbits, ndigits, digit,
              vs, BITS_PER_ULONG, DIGITS_PER_ULONG, vd);
  
  convert_2C_to_SM();

  return *this;
}


// ----------------------------------------------------------------------------
//  SECTION: Bitwise NOT operator: ~
// ----------------------------------------------------------------------------

CLASS_TYPE
operator~(const CLASS_TYPE& u)
{
  small_type s = u.sgn;

  if (s == SC_ZERO) {

    sc_digit d = 1;
    return CLASS_TYPE(SC_NEG, u.nbits, 1, &d, false);

  }

  int nd = u.ndigits;

#ifdef SC_MAX_NBITS
  sc_digit d[MAX_NDIGITS];
#else
  sc_digit *d = new sc_digit[nd];
#endif

  vec_copy(nd, d, u.digit);

  if (s == SC_POS) {

    s = SC_NEG;
    vec_add_small_on(nd, d, 1);

  }
  else {

    s = SC_POS;
    vec_sub_small_on(nd, d, 1);

    if (check_for_zero(nd, d))
      s = SC_ZERO;

  }

  return CLASS_TYPE(s, u.nbits, nd, d);
}


// ----------------------------------------------------------------------------
//  SECTION: LEFT SHIFT operators: <<, <<=
// ----------------------------------------------------------------------------

CLASS_TYPE
operator<<(const CLASS_TYPE& u, const CLASS_TYPE& v)
{
  if (v.sgn == SC_ZERO) 
    return CLASS_TYPE(u);

#ifdef SC_SIGNED
  if (v.sgn == SC_NEG)
    return CLASS_TYPE(u);
#endif

  return operator<<(u, v.to_ulong());
}


const CLASS_TYPE&
CLASS_TYPE::operator<<=(const CLASS_TYPE& v)
{
  if (v.sgn == SC_ZERO)
    return *this;

#ifdef SC_SIGNED
  if (v.sgn == SC_NEG)
    return *this;
#endif 

  return operator<<=(v.to_ulong());
}


const CLASS_TYPE&
CLASS_TYPE::operator<<=(const OTHER_CLASS_TYPE& v)
{
  if (v.sgn == SC_ZERO)
    return *this;

#ifdef SC_UNSIGNED
  if (v.sgn == SC_NEG)
    return *this;
#endif

  return operator<<=(v.to_ulong());
}


CLASS_TYPE
operator<<(const CLASS_TYPE& u, int64 v)
{
  if (v <= 0)
    return CLASS_TYPE(u);

  return operator<<(u, (unsigned long) v);
}


CLASS_TYPE
operator<<(const CLASS_TYPE& u, uint64 v)
{
  if (v == 0)
    return CLASS_TYPE(u);

  return operator<<(u, (unsigned long) v);
}


const CLASS_TYPE&
CLASS_TYPE::operator<<=(int64 v)
{
  if (v <= 0)
    return *this;

  return operator<<=((unsigned long) v);
}


const CLASS_TYPE&
CLASS_TYPE::operator<<=(uint64 v)
{
  if (v == 0)
    return *this;

  return operator<<=((unsigned long) v);
}


CLASS_TYPE
operator<<(const CLASS_TYPE& u, long v)
{
  if (v <= 0)
    return CLASS_TYPE(u);

  return operator<<(u, (unsigned long) v);
}

CLASS_TYPE
operator<<(const CLASS_TYPE& u, unsigned long v)
{
  if (v == 0)
    return CLASS_TYPE(u);

  if (u.sgn == SC_ZERO)
    return CLASS_TYPE(u);

  int nb = u.nbits + v;
  int nd = DIV_CEIL(nb);

#ifdef SC_MAX_NBITS
  test_bound(nb);
  sc_digit d[MAX_NDIGITS];
#else
  sc_digit *d = new sc_digit[nd];
#endif  

  vec_copy_and_zero(nd, d, u.ndigits, u.digit);

  convert_SM_to_2C(u.sgn, nd, d);

  vec_shift_left(nd, d, v);

  small_type s = convert_signed_2C_to_SM(nb, nd, d);

  return CLASS_TYPE(s, nb, nd, d);
}


const CLASS_TYPE&
CLASS_TYPE::operator<<=(long v)
{
  if (v <= 0)
    return *this;

  return operator<<=((unsigned long) v);
}


const CLASS_TYPE&
CLASS_TYPE::operator<<=(unsigned long v)
{
  if (v == 0)
    return *this;

  if (sgn == SC_ZERO)
    return *this;

  convert_SM_to_2C();

  vec_shift_left(ndigits, digit, v);

  convert_2C_to_SM();

  return *this;
}


// ----------------------------------------------------------------------------
//  SECTION: RIGHT SHIFT operators: >>, >>=
// ----------------------------------------------------------------------------

CLASS_TYPE
operator>>(const CLASS_TYPE& u, const CLASS_TYPE& v)
{
  if (v.sgn == SC_ZERO)
    return CLASS_TYPE(u);

#ifdef SC_SIGNED
  if (v.sgn == SC_NEG)
    return CLASS_TYPE(u);
#endif

  return operator>>(u, v.to_long());
}


const CLASS_TYPE&
CLASS_TYPE::operator>>=(const CLASS_TYPE& v)
{
  if (v.sgn == SC_ZERO)
    return *this;

#ifdef SC_SIGNED
  if (v.sgn == SC_NEG)
    return *this;
#endif

  return operator>>=(v.to_long());
}


const CLASS_TYPE&
CLASS_TYPE::operator>>=(const OTHER_CLASS_TYPE& v)
{
  if (v.sgn == SC_ZERO)
    return *this;

#ifdef SC_UNSIGNED
  if (v.sgn == SC_NEG)
    return *this;
#endif

  return operator>>=(v.to_ulong());
}


CLASS_TYPE
operator>>(const CLASS_TYPE& u, int64 v)
{
  if (v <= 0)
    return CLASS_TYPE(u);

  return operator>>(u, (unsigned long) v);
}


CLASS_TYPE
operator>>(const CLASS_TYPE& u, uint64 v)
{
  if (v == 0)
    return CLASS_TYPE(u);

  return operator>>(u, (unsigned long) v);
}

const CLASS_TYPE&
CLASS_TYPE::operator>>=(int64 v)
{
  if (v <= 0)
    return *this;

  return operator>>=((unsigned long) v);
}


const CLASS_TYPE&
CLASS_TYPE::operator>>=(uint64 v)
{
  if (v == 0)
    return *this;

  return operator>>=((unsigned long) v);
}


CLASS_TYPE
operator>>(const CLASS_TYPE& u, long v)
{
  if (v <= 0)
    return CLASS_TYPE(u);

  return operator>>(u, (unsigned long) v);
}


CLASS_TYPE
operator>>(const CLASS_TYPE& u, unsigned long v)
{
  if (v == 0)
    return CLASS_TYPE(u);

  if (u.sgn == SC_ZERO)
    return CLASS_TYPE(u);

  int nb = u.nbits;
  int nd = u.ndigits;

#ifdef SC_MAX_NBITS
  sc_digit d[MAX_NDIGITS];
#else
  sc_digit *d = new sc_digit[nd];
#endif  

  vec_copy(nd, d, u.digit);

  convert_SM_to_2C(u.sgn, nd, d);

  if (u.sgn == SC_NEG)
    vec_shift_right(nd, d, v, DIGIT_MASK);
  else
    vec_shift_right(nd, d, v, 0);
    
  small_type s = convert_signed_2C_to_SM(nb, nd, d);

  return CLASS_TYPE(s, nb, nd, d);
}


const CLASS_TYPE&
CLASS_TYPE::operator>>=(long v)
{
  if (v <= 0)
    return *this;

  return operator>>=((unsigned long) v);
}


const CLASS_TYPE&
CLASS_TYPE::operator>>=(unsigned long v)
{
  if (v == 0)
    return *this;

  if (sgn == SC_ZERO)
    return *this;

  convert_SM_to_2C();

  if (sgn == SC_NEG)
    vec_shift_right(ndigits, digit, v, DIGIT_MASK);
  else 
    vec_shift_right(ndigits, digit, v, 0);

  convert_2C_to_SM();

  return *this;
}


// ----------------------------------------------------------------------------
//  SECTION: EQUAL TO operator: ==
// ----------------------------------------------------------------------------

// Defined in the sc_signed.cpp and sc_unsigned.cpp.


// ----------------------------------------------------------------------------
//  SECTION: NOT_EQUAL operator: !=
// ----------------------------------------------------------------------------

bool
operator!=(const CLASS_TYPE& u, const CLASS_TYPE& v)
{
  return (! operator==(u, v));
}


bool
operator!=(const CLASS_TYPE& u, int64 v)
{
  return (! operator==(u, v));
}


bool
operator!=(int64 u, const CLASS_TYPE& v)
{
  return (! operator==(u, v));  
}


bool
operator!=(const CLASS_TYPE& u, uint64 v)
{
  return (! operator==(u, v));  
}


bool
operator!=(uint64 u, const CLASS_TYPE& v)
{
  return (! operator==(u, v));  
}


bool
operator!=(const CLASS_TYPE& u, long v)
{
  return (! operator==(u, v));
}


bool
operator!=(long u, const CLASS_TYPE& v)
{
  return (! operator==(u, v));  
}


bool
operator!=(const CLASS_TYPE& u, unsigned long v)
{
  return (! operator==(u, v));  
}


bool
operator!=(unsigned long u, const CLASS_TYPE& v)
{
  return (! operator==(u, v));  
}


// ----------------------------------------------------------------------------
//  SECTION: LESS THAN operator: <
// ----------------------------------------------------------------------------

// Defined in the sc_signed.cpp and sc_unsigned.cpp.


// ----------------------------------------------------------------------------
//  SECTION: LESS THAN or EQUAL operator: <=
// ----------------------------------------------------------------------------

bool
operator<=(const CLASS_TYPE& u, const CLASS_TYPE& v)
{
  return (operator<(u, v) || operator==(u, v));
}


bool
operator<=(const CLASS_TYPE& u, int64 v)
{
  return (operator<(u, v) || operator==(u, v));
}


bool
operator<=(int64 u, const CLASS_TYPE& v)
{
  return (operator<(u, v) || operator==(u, v));
}


bool
operator<=(const CLASS_TYPE& u, uint64 v)
{
  return (operator<(u, v) || operator==(u, v));
}


bool
operator<=(uint64 u, const CLASS_TYPE& v)
{
  return (operator<(u, v) || operator==(u, v));
}


bool
operator<=(const CLASS_TYPE& u, long v)
{
  return (operator<(u, v) || operator==(u, v));
}


bool
operator<=(long u, const CLASS_TYPE& v)
{
  return (operator<(u, v) || operator==(u, v));
}


bool
operator<=(const CLASS_TYPE& u, unsigned long v)
{
  return (operator<(u, v) || operator==(u, v));
}


bool
operator<=(unsigned long u, const CLASS_TYPE& v)
{
  return (operator<(u, v) || operator==(u, v));
}


// ----------------------------------------------------------------------------
//  SECTION: GREATER THAN operator: >
// ----------------------------------------------------------------------------

bool
operator>(const CLASS_TYPE& u, const CLASS_TYPE& v)
{
  return (! (operator<=(u, v)));
}


bool
operator>(const CLASS_TYPE& u, int64 v)
{
  return (! (operator<=(u, v)));
}


bool
operator>(int64 u, const CLASS_TYPE& v)
{
  return (! (operator<=(u, v)));
}


bool
operator>(const CLASS_TYPE& u, uint64 v)
{
  return (! (operator<=(u, v)));
}


bool
operator>(uint64 u, const CLASS_TYPE& v)
{
  return (! (operator<=(u, v)));
}


bool
operator>(const CLASS_TYPE& u, long v)
{
  return (! (operator<=(u, v)));
}


bool
operator>(long u, const CLASS_TYPE& v)
{
  return (! (operator<=(u, v)));
}


bool
operator>(const CLASS_TYPE& u, unsigned long v)
{
  return (! (operator<=(u, v)));
}


bool
operator>(unsigned long u, const CLASS_TYPE& v)
{
  return (! (operator<=(u, v)));
}


// ----------------------------------------------------------------------------
//  SECTION: GREATER THAN or EQUAL operator: >=
// ----------------------------------------------------------------------------

bool
operator>=(const CLASS_TYPE& u, const CLASS_TYPE& v)
{
  return (! (operator<(u, v)));
}


bool
operator>=(const CLASS_TYPE& u, int64 v)
{
  return (! (operator<(u, v)));
}


bool
operator>=(int64 u, const CLASS_TYPE& v)
{
  return (! (operator<(u, v)));
}


bool
operator>=(const CLASS_TYPE& u, uint64 v)
{
  return (! (operator<(u, v)));
}


bool
operator>=(uint64 u, const CLASS_TYPE& v)
{
  return (! (operator<(u, v)));
}


bool
operator>=(const CLASS_TYPE& u, long v)
{
  return (! (operator<(u, v)));
}


bool
operator>=(long u, const CLASS_TYPE& v)
{
  return (! (operator<(u, v)));
}


bool
operator>=(const CLASS_TYPE& u, unsigned long v)
{
  return (! (operator<(u, v)));
}


bool
operator>=(unsigned long u, const CLASS_TYPE& v)
{
  return (! (operator<(u, v)));
}


// ----------------------------------------------------------------------------
//  SECTION: Public members - Other utils.
// ----------------------------------------------------------------------------

// Convert to int64, long, or int.
#define TO_INTX(RET_TYPE, UP_RET_TYPE)   \
                                                             \
if (sgn == SC_ZERO)                                          \
return 0;                                                    \
                                                             \
int vnd = sc_min((int)DIGITS_PER_ ## UP_RET_TYPE, ndigits); \
                                                             \
RET_TYPE v = 0;                                              \
while (--vnd >= 0)                                           \
v = (v << BITS_PER_DIGIT) + digit[vnd];                      \
                                                             \
if (sgn == SC_NEG)                                           \
return -v;                                                   \
else                                                         \
return v;                                                    


int64
CLASS_TYPE::to_int64() const
{
  TO_INTX(int64, INT64);
}


long
CLASS_TYPE::to_long() const
{
  TO_INTX(long, LONG);
}


int
CLASS_TYPE::to_int() const
{
  TO_INTX(int, INT);
}


// Convert to unsigned int64, unsigned long or unsigned
// int. to_uint64, to_ulong, and to_uint have the same body except for
// the type of v defined inside.
uint64
CLASS_TYPE::to_uint64() const
{
  if (sgn == SC_ZERO)
    return 0;

  int vnd = sc_min((int)DIGITS_PER_INT64, ndigits);

  uint64 v = 0;

  if (sgn == SC_NEG) {

#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS];
#else
    sc_digit *d = new sc_digit[ndigits];
#endif

    vec_copy(ndigits, d, digit);

    convert_SM_to_2C_trimmed(IF_SC_SIGNED, sgn, nbits, ndigits, d);

    while (--vnd >= 0)
      v = (v << BITS_PER_DIGIT) + d[vnd];

#ifndef SC_MAX_NBITS
    delete [] d;
#endif

  }
  else {

    while (--vnd >= 0)
      v = (v << BITS_PER_DIGIT) + digit[vnd];

  }

  return v;
}


unsigned long
CLASS_TYPE::to_ulong() const
{
  if (sgn == SC_ZERO)
    return 0;

  int vnd = sc_min((int)DIGITS_PER_LONG, ndigits);

  unsigned long v = 0;

  if (sgn == SC_NEG) {

#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS];
#else
    sc_digit *d = new sc_digit[ndigits];
#endif

    vec_copy(ndigits, d, digit);

    convert_SM_to_2C_trimmed(IF_SC_SIGNED, sgn, nbits, ndigits, d);

    while (--vnd >= 0)
      v = (v << BITS_PER_DIGIT) + d[vnd];

#ifndef SC_MAX_NBITS
    delete [] d;
#endif

  }
  else {

    while (--vnd >= 0)
      v = (v << BITS_PER_DIGIT) + digit[vnd];

  }

  return v;
}


unsigned int
CLASS_TYPE::to_uint() const
{
  if (sgn == SC_ZERO)
    return 0;

  int vnd = sc_min((int)DIGITS_PER_INT, ndigits);

  unsigned int v = 0;

  if (sgn == SC_NEG) {

#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS];
#else
    sc_digit *d = new sc_digit[ndigits];
#endif

    vec_copy(ndigits, d, digit);

    convert_SM_to_2C_trimmed(IF_SC_SIGNED, sgn, nbits, ndigits, d);

    while (--vnd >= 0)
      v = (v << BITS_PER_DIGIT) + d[vnd];

#ifndef SC_MAX_NBITS
    delete [] d;
#endif

  }
  else {

    while (--vnd >= 0)
      v = (v << BITS_PER_DIGIT) + digit[vnd];

  }

  return v;
}


// Convert to double.
double
CLASS_TYPE::to_double() const
{
  if (sgn == SC_ZERO)
    return (double) 0.0;

  int vnd = ndigits;

  double v = 0.0;
  while (--vnd >= 0)
    v = v * DIGIT_RADIX + digit[vnd];

  if (sgn == SC_NEG)
    return -v;
  else
    return v;
}


// Return true if the bit i is 1, false otherwise. If i is outside the
// bounds, return 1/0 according to the sign of the number by assuming
// that the number has infinite length.

bool
CLASS_TYPE::test(int i) const
{
#ifdef SC_SIGNED
  if (check_if_outside(i)) {
    if (sgn == SC_NEG)
      return 1;
    else
      return 0;
  }
#else
  if (check_if_outside(i))
    return 0;
#endif  

  int bit_num = bit_ord(i);
  int digit_num = digit_ord(i);

  if (sgn == SC_NEG) {

#ifdef SC_MAX_NBITS
    sc_digit d[MAX_NDIGITS];
#else
    sc_digit *d = new sc_digit[ndigits];
#endif

    vec_copy(ndigits, d, digit);
    vec_complement(ndigits, d);
    bool val = ((d[digit_num] & one_and_zeros(bit_num)) != 0);

#ifndef SC_MAX_NBITS
    delete [] d;
#endif

    return val;

  }
  else 
    return ((digit[digit_num] & one_and_zeros(bit_num)) != 0);
}


// Set the ith bit with 1.
void 
CLASS_TYPE::set(int i) 
{
  if (check_if_outside(i))
    return;

  int bit_num = bit_ord(i);
  int digit_num = digit_ord(i);

  convert_SM_to_2C();
  digit[digit_num] |= one_and_zeros(bit_num);    
  digit[digit_num] &= DIGIT_MASK; // Needed to zero the overflow bits.
  convert_2C_to_SM();
}


// Set the ith bit with 0, i.e., clear the ith bit.
void 
CLASS_TYPE::clear(int i)
{
  if (check_if_outside(i))
    return;

  int bit_num = bit_ord(i);
  int digit_num = digit_ord(i);

  convert_SM_to_2C();
  digit[digit_num] &= ~(one_and_zeros(bit_num));
  digit[digit_num] &= DIGIT_MASK; // Needed to zero the overflow bits.
  convert_2C_to_SM();
}


// Create a mirror image of the number.
void
CLASS_TYPE::reverse()
{
  convert_SM_to_2C();
  vec_reverse(length(), ndigits, digit, length() - 1);
  convert_2C_to_SM();
}


// Get a packed bit representation of the number.
void 
CLASS_TYPE::get_packed_rep(sc_digit *buf) const
{
  int buf_ndigits = (length() - 1) / BITS_PER_DIGIT_TYPE + 1;

  // Initialize buf to zero.
  vec_zero(buf_ndigits, buf);

  if (sgn == SC_ZERO)
    return;

  const sc_digit *digit_or_d;
#ifdef SC_MAX_NBITS
  sc_digit d[MAX_NDIGITS];
#else
  sc_digit *d = new sc_digit[ndigits];
#endif

  if (sgn == SC_POS)
    digit_or_d = digit;

  else 
  {
    // If sgn is negative, we have to convert digit to its 2's
    // complement. Since this function is const, we can not do it on
    // digit. Since buf doesn't have overflow bits, we cannot also do
    // it on buf. Thus, we have to do the complementation on a copy of
    // digit, i.e., on d.

    vec_copy(ndigits, d, digit);
    vec_complement(ndigits, d);

    buf[buf_ndigits - 1] = ~((sc_digit) 0);

    digit_or_d = d;

  }

  // Copy the bits from digit to buf. The division and mod operations
  // below can be converted to addition/subtraction and comparison
  // operations at the expense of complicating the code. We can do it
  // if we see any performance problems.

  for (int i = length() - 1; i >= 0; --i) {

    if ((digit_or_d[digit_ord(i)] & one_and_zeros(bit_ord(i))) != 0) // Test.

      buf[i / BITS_PER_DIGIT_TYPE] |= 
        one_and_zeros(i % BITS_PER_DIGIT_TYPE); // Set.

    else  

      buf[i / BITS_PER_DIGIT_TYPE] &= 
        ~(one_and_zeros(i % BITS_PER_DIGIT_TYPE));  // Clear.

  }

#ifndef SC_MAX_NBITS
    delete[] d;
#endif
}


// Set a packed bit representation of the number.
void 
CLASS_TYPE::set_packed_rep(sc_digit *buf)
{
  // Initialize digit to zero.
  vec_zero(ndigits, digit);

  // Copy the bits from buf to digit.
  for (int i = length() - 1; i >= 0; --i) {

    if ((buf[i / BITS_PER_DIGIT_TYPE] & 
         one_and_zeros(i % BITS_PER_DIGIT_TYPE)) != 0) // Test.

      digit[digit_ord(i)] |= one_and_zeros(bit_ord(i));     // Set.

    else  

      digit[digit_ord(i)] &= ~(one_and_zeros(bit_ord(i)));  // Clear

  }

  convert_2C_to_SM();
}


// ----------------------------------------------------------------------------
//  SECTION: Private members.
// ----------------------------------------------------------------------------

// Create a copy of v with sgn s.
CLASS_TYPE::CLASS_TYPE(const CLASS_TYPE& v, small_type s) :
    sc_value_base(v), sgn(s), nbits(v.nbits), ndigits(v.ndigits), digit()
{
#ifndef SC_MAX_NBITS
  digit = new sc_digit[ndigits];
#endif

  vec_copy(ndigits, digit, v.digit);
}


// Create a copy of v where v is of the different type.
CLASS_TYPE::CLASS_TYPE(const OTHER_CLASS_TYPE& v, small_type s) :
    sc_value_base(v), sgn(s), nbits(num_bits(v.nbits)), ndigits(), digit()
{
#if (IF_SC_SIGNED == 1)
  ndigits = v.ndigits;
#else
  ndigits = DIV_CEIL(nbits);
#endif

#ifndef SC_MAX_NBITS
  digit = new sc_digit[ndigits];
#endif

  copy_digits(v.nbits, v.ndigits, v.digit);
}


// Create a signed number with (s, nb, nd, d) as its attributes (as
// defined in class CLASS_TYPE). If alloc is set, delete d.
CLASS_TYPE::CLASS_TYPE(small_type s, int nb, 
                       int nd, sc_digit *d, 
                       bool alloc) :
    sc_value_base(), sgn(s), nbits(num_bits(nb)), ndigits(), digit()
{
  ndigits = DIV_CEIL(nbits);

#ifndef SC_MAX_NBITS
  digit = new sc_digit[ndigits];
#endif

  if (ndigits <= nd)
    vec_copy(ndigits, digit, d);
  else
    vec_copy_and_zero(ndigits, digit, nd, d);

#ifndef SC_MAX_NBITS
  if (alloc)
    delete [] d;
#endif
}

// This constructor is mainly used in finding a "range" of bits from a
// number of type CLASS_TYPE. The function range(l, r) can have
// arbitrary precedence between l and r. If l is smaller than r, then
// the output is the reverse of range(r, l). 
CLASS_TYPE::CLASS_TYPE(const CLASS_TYPE* u, int l, int r) :
    sc_value_base(), sgn(), nbits(), ndigits(), digit()
{
  bool reversed = false;

  if( l < r ) {
    reversed = true;
    int tmp = l;
    l = r;
    r = tmp;
  }

  // at this point, l >= r

  // make sure that l and r point to the bits of u
  r = sc_max( r, 0 );
  l = sc_min( l, u->nbits - 1 );
    
  nbits = num_bits( l - r + 1 );

  // nbits can still be <= 0 because l and r have just been updated
  // with the bounds of u.

  // if u == 0 or the range is out of bounds, return 0
  if( u->sgn == SC_ZERO || nbits <= num_bits( 0 ) ) {
    sgn = SC_ZERO;
    if( nbits <= num_bits( 0 ) ) {
      nbits = 1;
    }
    ndigits = DIV_CEIL( nbits );
#ifndef SC_MAX_NBITS
    digit = new sc_digit[ndigits];
#endif
    vec_zero( ndigits, digit );
    return;
  }

  // The rest will be executed if u is not zero.

  ndigits = DIV_CEIL(nbits);
  
  // The number of bits up to and including l and r, respectively.
  int nl = l + 1; 
  int nr = r + 1; 
  
  // The indices of the digits that have lth and rth bits, respectively.
  int left_digit = DIV_CEIL(nl) - 1;
  int right_digit = DIV_CEIL(nr) - 1;
  
  int nd;

  // The range is performed on the 2's complement representation, so
  // first get the indices for that.
  if (u->sgn == SC_NEG)
    nd = left_digit + 1;
  else
    nd = left_digit - right_digit + 1;

  // Allocate memory for the range.
#ifdef SC_MAX_NBITS
  sc_digit d[MAX_NDIGITS];
#else
  digit = new sc_digit[ndigits];
  sc_digit *d = new sc_digit[nd];
#endif
  
  // Getting the range on the 2's complement representation.
  if (u->sgn == SC_NEG) {
    
    vec_copy(nd, d, u->digit);
    vec_complement(nd, d);  // d = -d;
    vec_shift_right(nd, d, r, DIGIT_MASK);
    
  }
  else {
    
    for (int i = right_digit; i <= left_digit; ++i)
      d[i - right_digit] = u->digit[i];
    
    vec_shift_right(nd, d, r - right_digit * BITS_PER_DIGIT, 0);
    
  }
  
  vec_zero(ndigits, digit);

  if (! reversed)
    vec_copy(sc_min(nd, ndigits), digit, d);
  
  else {

    // If l < r, i.e., reversed is set, reverse the bits of digit.  d
    // will be used as a temporary store. The following code tries to
    // minimize the use of bit_ord and digit_ord, which use mod and
    // div operators. Since these operators are function calls to
    // standard library routines, they are slow. The main idea in
    // reversing is "read bits out of d from left to right and push
    // them into digit using right shifting."

    // Take care of the last digit.
    int nd_less_1 = nd - 1;

    // Deletions will start from the left end and move one position
    // after each deletion.
    sc_digit del_mask = one_and_zeros(bit_ord(l - r));
      
    while (del_mask) {
      vec_shift_right(ndigits, digit, 1, ((d[nd_less_1] & del_mask) != 0));
      del_mask >>= 1;
    }

    // Take care of the other digits if any.

    // Insertion to digit will always occur at the left end.
    sc_digit ins_mask = one_and_zeros(BITS_PER_DIGIT - 1);

    for (int j = nd - 2; j >= 0; --j) { // j = nd - 2

      // Deletions will start from the left end and move one position
      // after each deletion.
      del_mask = ins_mask;

      while (del_mask) {
        vec_shift_right(ndigits, digit, 1, ((d[j] & del_mask) != 0));
        del_mask >>= 1;
      }
    }

    if (u->sgn == SC_NEG)
      vec_shift_right(ndigits, digit, 
                      ndigits * BITS_PER_DIGIT - length(), DIGIT_MASK);
    else
      vec_shift_right(ndigits, digit, 
                      ndigits * BITS_PER_DIGIT - length(), 0);


  }  // if reversed.

  convert_2C_to_SM();
  
#ifndef SC_MAX_NBITS
  delete [] d;
#endif
}

// This constructor is mainly used in finding a "range" of bits from a
// number of type OTHER_CLASS_TYPE. The function range(l, r) can have
// arbitrary precedence between l and r. If l is smaller than r, then
// the output is the reverse of range(r, l). 
CLASS_TYPE::CLASS_TYPE(const OTHER_CLASS_TYPE* u, int l, int r) :
    sc_value_base(), sgn(), nbits(), ndigits(), digit()
{
  bool reversed = false;

  if( l < r ) {
    reversed = true;
    int tmp = l;
    l = r;
    r = tmp;
  }

  // at this point, l >= r

  // make sure that l and r point to the bits of u
  r = sc_max( r, 0 );
  l = sc_min( l, u->nbits - 1 );
    
  nbits = num_bits( l - r + 1 );

  // nbits can still be <= 0 because l and r have just been updated
  // with the bounds of u.

  // if u == 0 or the range is out of bounds, return 0
  if( u->sgn == SC_ZERO || nbits <= num_bits( 0 ) ) {
    sgn = SC_ZERO;
    if( nbits <= num_bits( 0 ) ) {
      nbits = 1;
    }
    ndigits = DIV_CEIL( nbits );
#ifndef SC_MAX_NBITS
    digit = new sc_digit[ndigits];
#endif
    vec_zero( ndigits, digit );
    return;
  }

  // The rest will be executed if u is not zero.

  ndigits = DIV_CEIL(nbits);
  
  // The number of bits up to and including l and r, respectively.
  int nl = l + 1; 
  int nr = r + 1; 
  
  // The indices of the digits that have lth and rth bits, respectively.
  int left_digit = DIV_CEIL(nl) - 1;
  int right_digit = DIV_CEIL(nr) - 1;
  
  int nd;

  // The range is performed on the 2's complement representation, so
  // first get the indices for that.
  if (u->sgn == SC_NEG)
    nd = left_digit + 1;
  else
    nd = left_digit - right_digit + 1;

  // Allocate memory for the range.
#ifdef SC_MAX_NBITS
  sc_digit d[MAX_NDIGITS];
#else
  digit = new sc_digit[ndigits];
  sc_digit *d = new sc_digit[nd];
#endif
  
  // Getting the range on the 2's complement representation.
  if (u->sgn == SC_NEG) {
    
    vec_copy(nd, d, u->digit);
    vec_complement(nd, d);  // d = -d;
    vec_shift_right(nd, d, r, DIGIT_MASK);
    
  }
  else {
    
    for (int i = right_digit; i <= left_digit; ++i)
      d[i - right_digit] = u->digit[i];
    
    vec_shift_right(nd, d, r - right_digit * BITS_PER_DIGIT, 0);
    
  }
  
  vec_zero(ndigits, digit);

  if (! reversed)
    vec_copy(sc_min(nd, ndigits), digit, d);
  
  else {

    // If l < r, i.e., reversed is set, reverse the bits of digit.  d
    // will be used as a temporary store. The following code tries to
    // minimize the use of bit_ord and digit_ord, which use mod and
    // div operators. Since these operators are function calls to
    // standard library routines, they are slow. The main idea in
    // reversing is "read bits out of d from left to right and push
    // them into digit using right shifting."

    // Take care of the last digit.
    int nd_less_1 = nd - 1;

    // Deletions will start from the left end and move one position
    // after each deletion.
    sc_digit del_mask = one_and_zeros(bit_ord(l - r));
      
    while (del_mask) {
      vec_shift_right(ndigits, digit, 1, ((d[nd_less_1] & del_mask) != 0));
      del_mask >>= 1;
    }

    // Take care of the other digits if any.

    // Insertion to digit will always occur at the left end.
    sc_digit ins_mask = one_and_zeros(BITS_PER_DIGIT - 1);

    for (int j = nd - 2; j >= 0; --j) { // j = nd - 2

      // Deletions will start from the left end and move one position
      // after each deletion.
      del_mask = ins_mask;

      while (del_mask) {
        vec_shift_right(ndigits, digit, 1, ((d[j] & del_mask) != 0));
        del_mask >>= 1;
      }
    }

    if (u->sgn == SC_NEG)
      vec_shift_right(ndigits, digit, 
                      ndigits * BITS_PER_DIGIT - length(), DIGIT_MASK);
    else
      vec_shift_right(ndigits, digit, 
                      ndigits * BITS_PER_DIGIT - length(), 0);


  }  // if reversed.

  convert_2C_to_SM();
  
#ifndef SC_MAX_NBITS
  delete [] d;
#endif
}


// Print out all the physical attributes.
void
CLASS_TYPE::dump(::std::ostream& os) const
{
  // Save the current setting, and set the base to decimal.
#if defined(__MINGW32__)
  std::_Ios_Fmtflags old_flags = os.setf(::std::ios::dec,::std::ios::basefield);
#else
  fmtflags old_flags = os.setf(::std::ios::dec, ::std::ios::basefield);
#endif

  os << "width = " << length() << ::std::endl;
  os << "value = " << *this << ::std::endl;
  os << "bits  = ";

  int len = length();

  for (int i = len - 1; i >= 0; --i) {

    os << "01"[test(i)];
    if (--len % 4 == 0)
      os << " ";

  }

  os << ::std::endl;

  // Restore old_flags.
  os.setf(old_flags, ::std::ios::basefield);
}


// Checks to see if bit_num is out of bounds.
bool
CLASS_TYPE::check_if_outside(int bit_num) const
{
  if ((bit_num < 0) || (num_bits(bit_num) >= nbits)) {

#ifdef DEBUG_SYSTEMC
      if( bit_num < 0 || bit_num >= nbits ) {
	  char msg[BUFSIZ];
	  std::sprintf( msg, "%s::check_if_outside( int bit_num ) : "
		   "bit_num = %d is out of bounds",
		   CLASS_TYPE_STR, bit_num );
	  SC_REPORT_WARNING( sc_core::SC_ID_OUT_OF_BOUNDS_, msg );
      }
#endif

    return true;
  }

  return false;
}

// End of file.
