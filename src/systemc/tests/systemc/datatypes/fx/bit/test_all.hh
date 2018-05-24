// This may look like C code, but it is really -*- C++ -*-
// 
// test_all.hh<2> -- 
// Copyright Synopsys 1998
// Author          : Ric Hilderink
// Created On      : Thu Jan 21 10:32:46 1999
// Status          : none
// 

#ifndef _test_all_hh_
#define _test_all_hh_



#define TEST_BIT_T(T_op)                                                      \
{                                                                             \
  out << #T_op << " " << T_WL << " " << T_IWL << "\n";                        \
  sc_fxtype_context fooc(sc_fxtype_params(T_WL, T_IWL));                      \
  T_op a = 0;                                                                 \
  int i;                                                                      \
  for (i = 0; i < T_WL; ++i)                                                  \
    {                                                                         \
      if (a[i])                                                               \
        out << "|";                                                           \
      else                                                                    \
        out << ".";                                                           \
    }                                                                         \
  out << "\t";                                                                \
  for (i = 0; i < T_WL; i += 3)                                               \
    a[i] = 1;                                                                 \
  for (i = 0; i < T_WL; ++i)                                                  \
    {                                                                         \
      if (a[i])                                                               \
        out << "|";                                                           \
      else                                                                    \
        out << ".";                                                           \
    }                                                                         \
  out << "\n";                                                                \
  a = 0;                                                                      \
  a = ~a;                                                                     \
  for (i = 0; i < T_WL; ++i)                                                  \
    {                                                                         \
      if (a[i])                                                               \
        out << "|";                                                           \
      else                                                                    \
        out << ".";                                                           \
    }                                                                         \
  out << "\t";                                                                \
  for (i = 0; i < T_WL; i += 3)                                               \
    a[i] = 0;                                                                 \
  for (i = 0; i < T_WL; ++i)                                                  \
    {                                                                         \
      if (a[i])                                                               \
        out << "|";                                                           \
      else                                                                    \
        out << ".";                                                           \
    }                                                                         \
  out << "\n";                                                                \
  out << "a = " << a.to_string(SC_BIN) << "\n";                               \
  a = "0b01010";                                                              \
  out << "a = " << a.to_string(SC_BIN) << "\n";                               \
}


#define TEST_BIT                                                              \
TEST_BIT_T(T_FX_FIX);                                                         \
TEST_BIT_T(T_FX_FIXED);                                                       \
TEST_BIT_T(T_FX_UFIX);                                                        \
TEST_BIT_T(T_FX_UFIXED);


#define T_FX_UFIX   sc_ufix
#define T_FX_FIX    sc_fix
#define T_FX_FIXED sc_fixed<T_WL, T_IWL>
#define T_FX_UFIXED sc_ufixed<T_WL, T_IWL>

#define T_WL 4
#define T_IWL 4

#endif
