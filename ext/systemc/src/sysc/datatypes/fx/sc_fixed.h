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

  sc_fixed.h - 

  Original Author: Martin Janssen, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_fixed.h,v $
// Revision 1.2  2011/01/19 18:57:40  acg
//  Andy Goodrich: changes for IEEE_1666_2011.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:57  acg
// Andy Goodrich: added $Log command so that CVS comments are reproduced in
// the source.
//

#ifndef SC_FIXED_H
#define SC_FIXED_H


#include "sysc/datatypes/fx/sc_fix.h"


namespace sc_dt
{

// classes defined in this module
template <int W, int I, sc_q_mode Q, sc_o_mode O, int N> class sc_fixed;
template <int W, int I, sc_q_mode Q, sc_o_mode O, int N> class sc_fixed_fast;


// ----------------------------------------------------------------------------
//  TEMPLATE CLASS : sc_fixed
//
//  "Constrained" signed fixed-point class; arbitrary precision.
// ----------------------------------------------------------------------------

template <int W, int I,
	  sc_q_mode Q = SC_DEFAULT_Q_MODE_,
	  sc_o_mode O = SC_DEFAULT_O_MODE_, int N = SC_DEFAULT_N_BITS_>
class sc_fixed : public sc_fix
{

public:

    // constructors

    explicit sc_fixed( sc_fxnum_observer* = 0 );
    explicit sc_fixed( const sc_fxcast_switch&, sc_fxnum_observer* = 0 );

#define DECL_CTORS_T_A(tp)                                                    \
             sc_fixed( tp, sc_fxnum_observer* = 0 );                          \
             sc_fixed( tp, const sc_fxcast_switch&, sc_fxnum_observer* = 0 );

#define DECL_CTORS_T_B(tp)                                                    \
    explicit sc_fixed( tp, sc_fxnum_observer* = 0 );                          \
             sc_fixed( tp, const sc_fxcast_switch&, sc_fxnum_observer* = 0 );

    DECL_CTORS_T_A(int)
    DECL_CTORS_T_A(unsigned int)
    DECL_CTORS_T_A(long)
    DECL_CTORS_T_A(unsigned long)
    DECL_CTORS_T_A(float)
    DECL_CTORS_T_A(double)
    DECL_CTORS_T_A(const char*)
    DECL_CTORS_T_A(const sc_fxval&)
    DECL_CTORS_T_A(const sc_fxval_fast&)
    DECL_CTORS_T_A(const sc_fxnum&)
    DECL_CTORS_T_A(const sc_fxnum_fast&)
#ifndef SC_FX_EXCLUDE_OTHER
    DECL_CTORS_T_B(int64)
    DECL_CTORS_T_B(uint64)
    DECL_CTORS_T_B(const sc_int_base&)
    DECL_CTORS_T_B(const sc_uint_base&)
    DECL_CTORS_T_B(const sc_signed&)
    DECL_CTORS_T_B(const sc_unsigned&)
#endif

#undef DECL_CTORS_T_A
#undef DECL_CTORS_T_B

    // copy constructor

    sc_fixed( const sc_fixed<W,I,Q,O,N>& );


    // assignment operators

    sc_fixed& operator = ( const sc_fixed<W,I,Q,O,N>& );

#define DECL_ASN_OP_T(op,tp)                                                  \
    sc_fixed& operator op ( tp );

#ifndef SC_FX_EXCLUDE_OTHER
#define DECL_ASN_OP_OTHER(op)                                                 \
    DECL_ASN_OP_T(op,int64)                                                   \
    DECL_ASN_OP_T(op,uint64)                                                  \
    DECL_ASN_OP_T(op,const sc_int_base&)                                      \
    DECL_ASN_OP_T(op,const sc_uint_base&)                                     \
    DECL_ASN_OP_T(op,const sc_signed&)                                        \
    DECL_ASN_OP_T(op,const sc_unsigned&)
#else
#define DECL_ASN_OP_OTHER(op)
#endif

#define DECL_ASN_OP(op)                                                       \
    DECL_ASN_OP_T(op,int)                                                     \
    DECL_ASN_OP_T(op,unsigned int)                                            \
    DECL_ASN_OP_T(op,long)                                                    \
    DECL_ASN_OP_T(op,unsigned long)                                           \
    DECL_ASN_OP_T(op,float)                                                  \
    DECL_ASN_OP_T(op,double)                                                  \
    DECL_ASN_OP_T(op,const char*)                                             \
    DECL_ASN_OP_T(op,const sc_fxval&)                                         \
    DECL_ASN_OP_T(op,const sc_fxval_fast&)                                    \
    DECL_ASN_OP_T(op,const sc_fxnum&)                                         \
    DECL_ASN_OP_T(op,const sc_fxnum_fast&)                                    \
    DECL_ASN_OP_OTHER(op)

    DECL_ASN_OP(=)

    DECL_ASN_OP(*=)
    DECL_ASN_OP(/=)
    DECL_ASN_OP(+=)
    DECL_ASN_OP(-=)

    DECL_ASN_OP_T(<<=,int)
    DECL_ASN_OP_T(>>=,int)

    DECL_ASN_OP_T(&=,const sc_fix&)
    DECL_ASN_OP_T(&=,const sc_fix_fast&)
    DECL_ASN_OP_T(|=,const sc_fix&)
    DECL_ASN_OP_T(|=,const sc_fix_fast&)
    DECL_ASN_OP_T(^=,const sc_fix&)
    DECL_ASN_OP_T(^=,const sc_fix_fast&)

#undef DECL_ASN_OP_T
#undef DECL_ASN_OP_OTHER
#undef DECL_ASN_OP


    // auto-increment and auto-decrement

    const sc_fxval operator ++ ( int );
    const sc_fxval operator -- ( int );

    sc_fixed& operator ++ ();
    sc_fixed& operator -- ();

};


// ----------------------------------------------------------------------------
//  TEMPLATE CLASS : sc_fixed_fast
//
//  "Constrained" signed fixed-point class; limited precision.
// ----------------------------------------------------------------------------

template <int W, int I,
	  sc_q_mode Q = SC_DEFAULT_Q_MODE_,
	  sc_o_mode O = SC_DEFAULT_O_MODE_, int N = SC_DEFAULT_N_BITS_>
class sc_fixed_fast : public sc_fix_fast
{

public:

    // constructors

    explicit sc_fixed_fast( sc_fxnum_fast_observer* = 0 );
    explicit sc_fixed_fast( const sc_fxcast_switch&,
			    sc_fxnum_fast_observer* = 0 );

#define DECL_CTORS_T_A(tp)                                                    \
             sc_fixed_fast( tp, sc_fxnum_fast_observer* = 0 );                \
             sc_fixed_fast( tp, const sc_fxcast_switch&,                      \
                            sc_fxnum_fast_observer* = 0 );

#define DECL_CTORS_T_B(tp)                                                    \
    explicit sc_fixed_fast( tp, sc_fxnum_fast_observer* = 0 );                \
             sc_fixed_fast( tp, const sc_fxcast_switch&,                      \
                            sc_fxnum_fast_observer* = 0 );

    DECL_CTORS_T_A(int)
    DECL_CTORS_T_A(unsigned int)
    DECL_CTORS_T_A(long)
    DECL_CTORS_T_A(unsigned long)
    DECL_CTORS_T_A(float)
    DECL_CTORS_T_A(double)
    DECL_CTORS_T_A(const char*)
    DECL_CTORS_T_A(const sc_fxval&)
    DECL_CTORS_T_A(const sc_fxval_fast&)
    DECL_CTORS_T_A(const sc_fxnum&)
    DECL_CTORS_T_A(const sc_fxnum_fast&)
#ifndef SC_FX_EXCLUDE_OTHER
    DECL_CTORS_T_B(int64)
    DECL_CTORS_T_B(uint64)
    DECL_CTORS_T_B(const sc_int_base&)
    DECL_CTORS_T_B(const sc_uint_base&)
    DECL_CTORS_T_B(const sc_signed&)
    DECL_CTORS_T_B(const sc_unsigned&)
#endif

#undef DECL_CTORS_T_A
#undef DECL_CTORS_T_B

    // copy constructor

    sc_fixed_fast( const sc_fixed_fast<W,I,Q,O,N>& );


    // assignment operators

    sc_fixed_fast& operator = ( const sc_fixed_fast<W,I,Q,O,N>& );

#define DECL_ASN_OP_T(op,tp)                                                  \
    sc_fixed_fast& operator op ( tp );

#ifndef SC_FX_EXCLUDE_OTHER
#define DECL_ASN_OP_OTHER(op)                                                 \
    DECL_ASN_OP_T(op,int64)                                                   \
    DECL_ASN_OP_T(op,uint64)                                                  \
    DECL_ASN_OP_T(op,const sc_int_base&)                                      \
    DECL_ASN_OP_T(op,const sc_uint_base&)                                     \
    DECL_ASN_OP_T(op,const sc_signed&)                                        \
    DECL_ASN_OP_T(op,const sc_unsigned&)
#else
#define DECL_ASN_OP_OTHER(op)
#endif

#define DECL_ASN_OP(op)                                                       \
    DECL_ASN_OP_T(op,int)                                                     \
    DECL_ASN_OP_T(op,unsigned int)                                            \
    DECL_ASN_OP_T(op,long)                                                    \
    DECL_ASN_OP_T(op,unsigned long)                                           \
    DECL_ASN_OP_T(op,float)                                                  \
    DECL_ASN_OP_T(op,double)                                                  \
    DECL_ASN_OP_T(op,const char*)                                             \
    DECL_ASN_OP_T(op,const sc_fxval&)                                         \
    DECL_ASN_OP_T(op,const sc_fxval_fast&)                                    \
    DECL_ASN_OP_T(op,const sc_fxnum&)                                         \
    DECL_ASN_OP_T(op,const sc_fxnum_fast&)                                    \
    DECL_ASN_OP_OTHER(op)

    DECL_ASN_OP(=)

    DECL_ASN_OP(*=)
    DECL_ASN_OP(/=)
    DECL_ASN_OP(+=)
    DECL_ASN_OP(-=)

    DECL_ASN_OP_T(<<=,int)
    DECL_ASN_OP_T(>>=,int)

    DECL_ASN_OP_T(&=,const sc_fix&)
    DECL_ASN_OP_T(&=,const sc_fix_fast&)
    DECL_ASN_OP_T(|=,const sc_fix&)
    DECL_ASN_OP_T(|=,const sc_fix_fast&)
    DECL_ASN_OP_T(^=,const sc_fix&)
    DECL_ASN_OP_T(^=,const sc_fix_fast&)

#undef DECL_ASN_OP_T
#undef DECL_ASN_OP_OTHER
#undef DECL_ASN_OP


    // auto-increment and auto-decrement

    const sc_fxval_fast operator ++ ( int );
    const sc_fxval_fast operator -- ( int );

    sc_fixed_fast& operator ++ ();
    sc_fixed_fast& operator -- ();

};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// ----------------------------------------------------------------------------
//  TEMPLATE CLASS : sc_fixed
//
//  "Constrained" signed fixed-point class; arbitrary precision.
// ----------------------------------------------------------------------------

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed<W,I,Q,O,N>::sc_fixed( sc_fxnum_observer* observer_ )
: sc_fix( W, I, Q, O, N, observer_ )
{}

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed<W,I,Q,O,N>::sc_fixed( const sc_fxcast_switch& cast_sw,
			       sc_fxnum_observer* observer_ )
: sc_fix( W, I, Q, O, N, cast_sw, observer_ )
{}

#define DEFN_CTORS_T(tp)                                                      \
template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>                       \
inline                                                                        \
sc_fixed<W,I,Q,O,N>::sc_fixed( tp a,                                          \
			       sc_fxnum_observer* observer_ )                 \
: sc_fix( a, W, I, Q, O, N, observer_ )                                       \
{}                                                                            \
                                                                              \
template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>                       \
inline                                                                        \
sc_fixed<W,I,Q,O,N>::sc_fixed( tp a,                                          \
			       const sc_fxcast_switch& cast_sw,               \
			       sc_fxnum_observer* observer_ )                 \
: sc_fix( a, W, I, Q, O, N, cast_sw, observer_ )                              \
{}

DEFN_CTORS_T(int)
DEFN_CTORS_T(unsigned int)
DEFN_CTORS_T(long)
DEFN_CTORS_T(unsigned long)
DEFN_CTORS_T(float)
DEFN_CTORS_T(double)
DEFN_CTORS_T(const char*)
DEFN_CTORS_T(const sc_fxval&)
DEFN_CTORS_T(const sc_fxval_fast&)
DEFN_CTORS_T(const sc_fxnum&)
DEFN_CTORS_T(const sc_fxnum_fast&)
#ifndef SC_FX_EXCLUDE_OTHER
DEFN_CTORS_T(int64)
DEFN_CTORS_T(uint64)
DEFN_CTORS_T(const sc_int_base&)
DEFN_CTORS_T(const sc_uint_base&)
DEFN_CTORS_T(const sc_signed&)
DEFN_CTORS_T(const sc_unsigned&)
#endif

#undef DEFN_CTORS_T

// copy constructor

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed<W,I,Q,O,N>::sc_fixed( const sc_fixed<W,I,Q,O,N>& a )
: sc_fix( a, W, I, Q, O, N )
{}


// assignment operators

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed<W,I,Q,O,N>&
sc_fixed<W,I,Q,O,N>::operator = ( const sc_fixed<W,I,Q,O,N>& a )
{
    sc_fix::operator = ( a );
    return *this;
}

#define DEFN_ASN_OP_T(op,tp)                                                  \
template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>                       \
inline                                                                        \
sc_fixed<W,I,Q,O,N>&                                                          \
sc_fixed<W,I,Q,O,N>::operator op ( tp a )                                     \
{                                                                             \
    sc_fix::operator op ( a );                                                \
    return *this;                                                             \
}

#ifndef SC_FX_EXCLUDE_OTHER
#define DEFN_ASN_OP_OTHER(op)                                                 \
DEFN_ASN_OP_T(op,int64)                                                       \
DEFN_ASN_OP_T(op,uint64)                                                      \
DEFN_ASN_OP_T(op,const sc_int_base&)                                          \
DEFN_ASN_OP_T(op,const sc_uint_base&)                                         \
DEFN_ASN_OP_T(op,const sc_signed&)                                            \
DEFN_ASN_OP_T(op,const sc_unsigned&)
#else
#define DEFN_ASN_OP_OTHER(op)
#endif

#define DEFN_ASN_OP(op)                                                       \
DEFN_ASN_OP_T(op,int)                                                         \
DEFN_ASN_OP_T(op,unsigned int)                                                \
DEFN_ASN_OP_T(op,long)                                                        \
DEFN_ASN_OP_T(op,unsigned long)                                               \
DEFN_ASN_OP_T(op,float)                                                      \
DEFN_ASN_OP_T(op,double)                                                      \
DEFN_ASN_OP_T(op,const char*)                                                 \
DEFN_ASN_OP_T(op,const sc_fxval&)                                             \
DEFN_ASN_OP_T(op,const sc_fxval_fast&)                                        \
DEFN_ASN_OP_T(op,const sc_fxnum&)                                             \
DEFN_ASN_OP_T(op,const sc_fxnum_fast&)                                        \
DEFN_ASN_OP_OTHER(op)

DEFN_ASN_OP(=)

DEFN_ASN_OP(*=)
DEFN_ASN_OP(/=)
DEFN_ASN_OP(+=)
DEFN_ASN_OP(-=)

DEFN_ASN_OP_T(<<=,int)
DEFN_ASN_OP_T(>>=,int)

DEFN_ASN_OP_T(&=,const sc_fix&)
DEFN_ASN_OP_T(&=,const sc_fix_fast&)
DEFN_ASN_OP_T(|=,const sc_fix&)
DEFN_ASN_OP_T(|=,const sc_fix_fast&)
DEFN_ASN_OP_T(^=,const sc_fix&)
DEFN_ASN_OP_T(^=,const sc_fix_fast&)

#undef DEFN_ASN_OP_T
#undef DEFN_ASN_OP_OTHER
#undef DEFN_ASN_OP


// auto-increment and auto-decrement

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
const sc_fxval
sc_fixed<W,I,Q,O,N>::operator ++ ( int )
{
    return sc_fxval( sc_fix::operator ++ ( 0 ) );
}

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
const sc_fxval
sc_fixed<W,I,Q,O,N>::operator -- ( int )
{
    return sc_fxval( sc_fix::operator -- ( 0 ) );
}

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed<W,I,Q,O,N>&
sc_fixed<W,I,Q,O,N>::operator ++ ()
{
    sc_fix::operator ++ ();
    return *this;
}

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed<W,I,Q,O,N>&
sc_fixed<W,I,Q,O,N>::operator -- ()
{
    sc_fix::operator -- ();
    return *this;
}


// ----------------------------------------------------------------------------
//  TEMPLATE CLASS : sc_fixed_fast
//
//  "Constrained" signed fixed-point class; limited precision.
// ----------------------------------------------------------------------------

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed_fast<W,I,Q,O,N>::sc_fixed_fast( sc_fxnum_fast_observer* observer_ )
: sc_fix_fast( W, I, Q, O, N, observer_ )
{}

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed_fast<W,I,Q,O,N>::sc_fixed_fast( const sc_fxcast_switch& cast_sw,
					 sc_fxnum_fast_observer* observer_ )
: sc_fix_fast( W, I, Q, O, N, cast_sw, observer_ )
{}

#define DEFN_CTORS_T(tp)                                                      \
template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>                       \
inline                                                                        \
sc_fixed_fast<W,I,Q,O,N>::sc_fixed_fast( tp a,                                \
					 sc_fxnum_fast_observer* observer_ )  \
: sc_fix_fast( a, W, I, Q, O, N, observer_ )                                  \
{}                                                                            \
                                                                              \
template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>                       \
inline                                                                        \
sc_fixed_fast<W,I,Q,O,N>::sc_fixed_fast( tp a,                                \
					 const sc_fxcast_switch& cast_sw,     \
					 sc_fxnum_fast_observer* observer_ )  \
: sc_fix_fast( a, W, I, Q, O, N, cast_sw, observer_ )                         \
{}

DEFN_CTORS_T(int)
DEFN_CTORS_T(unsigned int)
DEFN_CTORS_T(long)
DEFN_CTORS_T(unsigned long)
DEFN_CTORS_T(float)
DEFN_CTORS_T(double)
DEFN_CTORS_T(const char*)
DEFN_CTORS_T(const sc_fxval&)
DEFN_CTORS_T(const sc_fxval_fast&)
DEFN_CTORS_T(const sc_fxnum&)
DEFN_CTORS_T(const sc_fxnum_fast&)
#ifndef SC_FX_EXCLUDE_OTHER
DEFN_CTORS_T(int64)
DEFN_CTORS_T(uint64)
DEFN_CTORS_T(const sc_int_base&)
DEFN_CTORS_T(const sc_uint_base&)
DEFN_CTORS_T(const sc_signed&)
DEFN_CTORS_T(const sc_unsigned&)
#endif

#undef DEFN_CTORS_T

// copy constructor

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed_fast<W,I,Q,O,N>::sc_fixed_fast( const sc_fixed_fast<W,I,Q,O,N>& a )
: sc_fix_fast( a, W, I, Q, O, N )
{}


// assignment operators

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed_fast<W,I,Q,O,N>&
sc_fixed_fast<W,I,Q,O,N>::operator = ( const sc_fixed_fast<W,I,Q,O,N>& a )
{
    sc_fix_fast::operator = ( a );
    return *this;
}

#define DEFN_ASN_OP_T(op,tp)                                                  \
template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>                       \
inline                                                                        \
sc_fixed_fast<W,I,Q,O,N>&                                                     \
sc_fixed_fast<W,I,Q,O,N>::operator op ( tp a )                                \
{                                                                             \
    sc_fix_fast::operator op ( a );                                           \
    return *this;                                                             \
}

#ifndef SC_FX_EXCLUDE_OTHER
#define DEFN_ASN_OP_OTHER(op)                                                 \
DEFN_ASN_OP_T(op,int64)                                                       \
DEFN_ASN_OP_T(op,uint64)                                                      \
DEFN_ASN_OP_T(op,const sc_int_base&)                                          \
DEFN_ASN_OP_T(op,const sc_uint_base&)                                         \
DEFN_ASN_OP_T(op,const sc_signed&)                                            \
DEFN_ASN_OP_T(op,const sc_unsigned&)
#else
#define DEFN_ASN_OP_OTHER(op)
#endif

#define DEFN_ASN_OP(op)                                                       \
DEFN_ASN_OP_T(op,int)                                                         \
DEFN_ASN_OP_T(op,unsigned int)                                                \
DEFN_ASN_OP_T(op,long)                                                        \
DEFN_ASN_OP_T(op,unsigned long)                                               \
DEFN_ASN_OP_T(op,float)                                                      \
DEFN_ASN_OP_T(op,double)                                                      \
DEFN_ASN_OP_T(op,const char*)                                                 \
DEFN_ASN_OP_T(op,const sc_fxval&)                                             \
DEFN_ASN_OP_T(op,const sc_fxval_fast&)                                        \
DEFN_ASN_OP_T(op,const sc_fxnum&)                                             \
DEFN_ASN_OP_T(op,const sc_fxnum_fast&)                                        \
DEFN_ASN_OP_OTHER(op)

DEFN_ASN_OP(=)

DEFN_ASN_OP(*=)
DEFN_ASN_OP(/=)
DEFN_ASN_OP(+=)
DEFN_ASN_OP(-=)

DEFN_ASN_OP_T(<<=,int)
DEFN_ASN_OP_T(>>=,int)

DEFN_ASN_OP_T(&=,const sc_fix&)
DEFN_ASN_OP_T(&=,const sc_fix_fast&)
DEFN_ASN_OP_T(|=,const sc_fix&)
DEFN_ASN_OP_T(|=,const sc_fix_fast&)
DEFN_ASN_OP_T(^=,const sc_fix&)
DEFN_ASN_OP_T(^=,const sc_fix_fast&)

#undef DEFN_ASN_OP_T
#undef DEFN_ASN_OP_OTHER
#undef DEFN_ASN_OP


// auto-increment and auto-decrement

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
const sc_fxval_fast
sc_fixed_fast<W,I,Q,O,N>::operator ++ ( int )
{
    return sc_fxval_fast( sc_fix_fast::operator ++ ( 0 ) );
}

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
const sc_fxval_fast
sc_fixed_fast<W,I,Q,O,N>::operator -- ( int )
{
    return sc_fxval_fast( sc_fix_fast::operator -- ( 0 ) );
}

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed_fast<W,I,Q,O,N>&
sc_fixed_fast<W,I,Q,O,N>::operator ++ ()
{
    sc_fix_fast::operator ++ ();
    return *this;
}

template<int W, int I, sc_q_mode Q, sc_o_mode O, int N>
inline
sc_fixed_fast<W,I,Q,O,N>&
sc_fixed_fast<W,I,Q,O,N>::operator -- ()
{
    sc_fix_fast::operator -- ();
    return *this;
}

} // namespace sc_dt


#endif

// Taf!
