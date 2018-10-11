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

#ifndef __SYSTEMC_EXT_UTIL_SC_TRACE_FILE_HH__
#define __SYSTEMC_EXT_UTIL_SC_TRACE_FILE_HH__

#include <exception>

#include "../core/sc_time.hh"

namespace sc_dt
{

class sc_logic;
class sc_int_base;
class sc_uint_base;
class sc_signed;
class sc_unsigned;
class sc_bv_base;
class sc_lv_base;
class sc_fxval;
class sc_fxval_fast;
class sc_fxnum;
class sc_fxnum_fast;

} // namespace sc_dt

namespace sc_core
{

template <class T>
class sc_signal_in_if;

class sc_event;
class sc_time;

class sc_trace_file
{
  protected:
    sc_trace_file();

  public:
    virtual ~sc_trace_file();

    virtual void set_time_unit(double, sc_time_unit) = 0;
};

sc_trace_file *sc_create_vcd_trace_file(const char *name);
void sc_close_vcd_trace_file(sc_trace_file *tf);
void sc_write_comment(sc_trace_file *tf, const std::string &comment);

void sc_trace(sc_trace_file *, const bool &, const std::string &);
void sc_trace(sc_trace_file *, const bool *, const std::string &);
void sc_trace(sc_trace_file *, const float &, const std::string &);
void sc_trace(sc_trace_file *, const float *, const std::string &);
void sc_trace(sc_trace_file *, const double &, const std::string &);
void sc_trace(sc_trace_file *, const double *, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_logic &, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_logic *, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_int_base &,
              const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_int_base *,
              const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_uint_base &,
              const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_uint_base *,
              const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_signed &, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_signed *, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_unsigned &,
              const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_unsigned *,
              const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_bv_base &, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_bv_base *, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_lv_base &, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_lv_base *, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_fxval &, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_fxval *, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_fxval_fast &,
              const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_fxval_fast *,
              const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_fxnum &, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_fxnum *, const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_fxnum_fast &,
              const std::string &);
void sc_trace(sc_trace_file *, const sc_dt::sc_fxnum_fast *,
              const std::string &);


// Nonstandard
// sc_trace overloads for sc_event and sc_time.
void sc_trace(sc_trace_file *, const sc_event &, const std::string &);
void sc_trace(sc_trace_file *, const sc_event *, const std::string &);
void sc_trace(sc_trace_file *, const sc_time &, const std::string &);
void sc_trace(sc_trace_file *, const sc_time *, const std::string &);


// Nonstandard - unsigned versions necessary to avoid ambiguous overload
// resolution.
void sc_trace(sc_trace_file *, const unsigned char &,
              const std::string &, int width=(8 * sizeof(unsigned char)));
void sc_trace(sc_trace_file *, const unsigned char *,
              const std::string &, int width=(8 * sizeof(unsigned char)));
void sc_trace(sc_trace_file *, const unsigned short &,
              const std::string &, int width=(8 * sizeof(unsigned short)));
void sc_trace(sc_trace_file *, const unsigned short *,
              const std::string &, int width=(8 * sizeof(unsigned short)));
void sc_trace(sc_trace_file *, const unsigned int &,
              const std::string &, int width=(8 * sizeof(unsigned int)));
void sc_trace(sc_trace_file *, const unsigned int *,
              const std::string &, int width=(8 * sizeof(unsigned int)));
void sc_trace(sc_trace_file *, const unsigned long &,
              const std::string &, int width=(8 * sizeof(unsigned long)));
void sc_trace(sc_trace_file *, const unsigned long *,
              const std::string &, int width=(8 * sizeof(unsigned long)));

void sc_trace(sc_trace_file *, const char &,
              const std::string &, int width=(8 * sizeof(char)));
void sc_trace(sc_trace_file *, const char *,
              const std::string &, int width=(8 * sizeof(char)));
void sc_trace(sc_trace_file *, const short &,
              const std::string &, int width=(8 * sizeof(short)));
void sc_trace(sc_trace_file *, const short *,
              const std::string &, int width=(8 * sizeof(short)));
void sc_trace(sc_trace_file *, const int &,
              const std::string &, int width=(8 * sizeof(int)));
void sc_trace(sc_trace_file *, const int *,
              const std::string &, int width=(8 * sizeof(int)));
void sc_trace(sc_trace_file *, const long &,
              const std::string &, int width=(8 * sizeof(long)));
void sc_trace(sc_trace_file *, const long *,
              const std::string &, int width=(8 * sizeof(long)));
void sc_trace(sc_trace_file *, const sc_dt::int64 &,
              const std::string &, int width=(8 * sizeof(sc_dt::int64)));
void sc_trace(sc_trace_file *, const sc_dt::int64 *,
              const std::string &, int width=(8 * sizeof(sc_dt::int64)));
void sc_trace(sc_trace_file *, const sc_dt::uint64 &,
              const std::string &, int width=(8 * sizeof(sc_dt::uint64)));
void sc_trace(sc_trace_file *, const sc_dt::uint64 *,
              const std::string &, int width=(8 * sizeof(sc_dt::uint64)));

// Nonstandard function for enums
void sc_trace(sc_trace_file *, const unsigned int &,
              const std::string &, const char **enum_literals);

// Deprecated
void sc_trace_delta_cycles(sc_trace_file *, bool on=true);

template <class T>
void
sc_trace(sc_trace_file *tf, const sc_signal_in_if<T> &iface,
        const std::string &name)
{
    sc_trace(tf, iface.read(), name);
}

void sc_trace(sc_trace_file *, const sc_signal_in_if<char> &,
              const std::string &, int width);

void sc_trace(sc_trace_file *, const sc_signal_in_if<short> &,
              const std::string &, int width);

void sc_trace(sc_trace_file *, const sc_signal_in_if<int> &,
              const std::string &, int width);

void sc_trace(sc_trace_file *, const sc_signal_in_if<long> &,
              const std::string &, int width);

} // namespace sc_core

#endif  //__SYSTEMC_EXT_UTIL_SC_TRACE_FILE_HH__
