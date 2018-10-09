/*
 * Copyright 2018 Google, Inc.
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
 *
 * Authors: Gabe Black
 */

#ifndef __SYSTEMC_EXT_DT_FX__USING_HH__
#define __SYSTEMC_EXT_DT_FX__USING_HH__

#include "_fx.hh"

using sc_dt::sc_fxnum;
using sc_dt::sc_fxnum_bitref;
using sc_dt::sc_fxnum_fast;
using sc_dt::sc_fix;
using sc_dt::sc_fix_fast;
using sc_dt::sc_ufix;
using sc_dt::sc_ufix_fast;
using sc_dt::sc_fixed;
using sc_dt::sc_fixed_fast;
using sc_dt::sc_ufixed;
using sc_dt::sc_ufixed_fast;
using sc_dt::sc_fxval;
using sc_dt::sc_fxval_fast;
using sc_dt::sc_fxcast_switch;
using sc_dt::sc_fxcast_context;
using sc_dt::sc_fxtype_params;
using sc_dt::sc_fxtype_context;
using sc_dt::sc_q_mode;
using sc_dt::SC_RND;
using sc_dt::SC_RND_ZERO;
using sc_dt::SC_RND_MIN_INF;
using sc_dt::SC_RND_INF;
using sc_dt::SC_RND_CONV;
using sc_dt::SC_TRN;
using sc_dt::SC_TRN_ZERO;
using sc_dt::sc_o_mode;
using sc_dt::SC_SAT;
using sc_dt::SC_SAT_ZERO;
using sc_dt::SC_SAT_SYM;
using sc_dt::SC_WRAP;
using sc_dt::SC_WRAP_SM;
using sc_dt::sc_switch;
using sc_dt::SC_OFF;
using sc_dt::SC_ON;
using sc_dt::sc_fmt;
using sc_dt::SC_F;
using sc_dt::SC_E;
using sc_dt::sc_context_begin;
using sc_dt::SC_NOW;
using sc_dt::SC_LATER;

using sc_core::SC_ID_INVALID_WL_;
using sc_core::SC_ID_INVALID_N_BITS_;
using sc_core::SC_ID_INVALID_DIV_WL_;
using sc_core::SC_ID_INVALID_CTE_WL_;
using sc_core::SC_ID_INVALID_MAX_WL_;
using sc_core::SC_ID_INVALID_FX_VALUE_;
using sc_core::SC_ID_INVALID_O_MODE_;
using sc_core::SC_ID_OUT_OF_RANGE_;
using sc_core::SC_ID_CONTEXT_BEGIN_FAILED_;
using sc_core::SC_ID_CONTEXT_END_FAILED_;
using sc_core::SC_ID_WRAP_SM_NOT_DEFINED_;

#endif  //__SYSTEMC_EXT_DT_FX__USING_HH__
