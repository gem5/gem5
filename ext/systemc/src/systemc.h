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

  systemc.h - Top-level include file for the SystemC library with usings.

  Original Author: Stan Y. Liao, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Andy Goodrich, Forte Design Systems, 31 Mar 2005
  Description of Modification: Changes for namespace support.

 *****************************************************************************/

#ifndef SYSTEMC_H
#define SYSTEMC_H

// INCLUDE SYSTEM (std) DEFINITIONS:

#include <cassert>
#include <climits>
#include <cmath> // math.h?
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#if defined(__sun) || defined(__sun__)
#   include <inttypes.h>
#elif !defined(WIN32) && !defined(_WIN32)
#   include <stdint.h>
#endif
#include <typeinfo>
#include <utility>
#include <vector>

// USINGS FOR I/O STREAM SUPPORT:

    using std::ios;
    using std::streambuf;
    using std::streampos;
    using std::streamsize;
    using std::iostream;
    using std::istream;
    using std::ostream;
    using std::cin;
    using std::cout;
    using std::cerr;
    using std::endl;
    using std::flush;
    using std::dec;
    using std::hex;
    using std::oct;
    using std::noshowbase;
    using std::showbase;

    using std::fstream;
    using std::ifstream;
    using std::ofstream;

//  from <cstdio>:

    using std::size_t;
    using std::FILE;
    using std::fpos_t;
    using std::fclose;
    using std::clearerr;

    using std::remove;
    using std::rename;
    using std::tmpfile;
    using std::tmpnam;
    using std::fflush;
    using std::fopen;
    using std::freopen;
    using std::setbuf;
    using std::setvbuf;
    using std::fprintf;
    using std::fscanf;
    using std::printf;
    using std::scanf;
    using std::sprintf;
    using std::sscanf;
    using std::vfprintf;
    using std::vprintf;
    using std::vsprintf;
    using std::fgetc;
    using std::fgets;
    using std::fputc;
    using std::fputs;
    using std::getc;
    using std::getchar;
    //using std::gets;
    using std::putc;
    using std::putchar;
    using std::puts;
    using std::ungetc;
    using std::fread;
    using std::fwrite;
    using std::fgetpos;
    using std::fseek;
    using std::fsetpos;
    using std::ftell;
    using std::rewind;
    using std::feof;
    using std::ferror;
    using std::perror;

//  from <cstdlib>:

    using std::div_t;
    using std::ldiv_t;

    using std::atof;
    using std::atoi;
    using std::atol;
    using std::strtod;
    using std::strtol;
    using std::strtoul;
    using std::rand;
    using std::srand;
    using std::calloc;
    using std::free;
    using std::malloc;
    using std::realloc;
    using std::abort;
    using std::atexit;
    using std::exit;
    using std::getenv;
    using std::system;
    using std::bsearch;
    using std::qsort;
    using std::abs;
    using std::div;
    using std::labs;
    using std::ldiv;
    using std::mblen;
    using std::mbtowc;
    using std::mbstowcs;
#if !defined(__CYGWIN__) && !defined(__CYGWIN32)
    using std::wctomb;
    using std::wcstombs;
#endif

//  from <cstring>:

    using std::memcpy;
    using std::memmove;
    using std::strcpy;
    using std::strncpy;
    using std::strcat;
    using std::strncat;
    using std::memcmp;
    using std::strcmp;
    using std::strcoll;
    using std::strncmp;
    using std::strxfrm;
    using std::memchr;
    using std::strchr;
    using std::strcspn;
    using std::strpbrk;
    using std::strrchr;
    using std::strspn;
    using std::strstr;
    using std::strtok;
    using std::memset;
    using std::strerror;
    using std::strlen;

// deprecated strstream support
#if defined( SC_INCLUDE_STRSTREAM )
#include <strstream>

    using std::strstream;
    using std::strstreambuf;
    using std::istrstream;
    using std::ostrstream;

#endif // SC_INCLUDE_STRSTREAM

// INCLUDE SYSTEMC DEFINITIONS for sc_dt AND sc_core NAMESPACES:

#include "systemc"

// USINGS FOR THE sc_dt NAMESPACE:

using sc_dt::SC_BIN;
using sc_dt::SC_BIN_SM;
using sc_dt::SC_BIN_US;
using sc_dt::SC_CSD;
using sc_dt::SC_DEC;
using sc_dt::SC_HEX;
using sc_dt::SC_HEX_SM;
using sc_dt::SC_HEX_US;
using sc_dt::SC_LOGIC_0;
using sc_dt::SC_LOGIC_1;
using sc_dt::SC_LOGIC_X;
using sc_dt::SC_LOGIC_Z;
using sc_dt::SC_NOBASE;
using sc_dt::SC_OCT;
using sc_dt::SC_OCT_SM;
using sc_dt::SC_OCT_US;
using sc_dt::int64;
using sc_dt::sc_abs;
using sc_dt::sc_bigint;
using sc_dt::sc_biguint;
using sc_dt::sc_bit;
using sc_dt::sc_bv;
using sc_dt::sc_bv_base;
using sc_dt::sc_digit;
using sc_dt::sc_int;
using sc_dt::sc_int_base;
using sc_dt::sc_io_show_base;
using sc_dt::sc_length_context;
using sc_dt::sc_length_param;
using sc_dt::sc_logic;
using sc_dt::sc_lv;
using sc_dt::sc_lv_base;
using sc_dt::sc_max;
using sc_dt::sc_min;
using sc_dt::sc_numrep;
using sc_dt::sc_signed;
using sc_dt::sc_uint;
using sc_dt::sc_uint_base;
using sc_dt::sc_unsigned;
using sc_dt::uint64;
// #ifdef SC_DT_DEPRECATED
using sc_dt::sc_logic_0;
using sc_dt::sc_logic_1;
using sc_dt::sc_logic_Z;
using sc_dt::sc_logic_X;
// #endif

#ifdef SC_INCLUDE_FX
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
#endif // SC_INCLUDE_FX

#if 0 // defined( _MSC_VER ) // supported versions of MSVC should support ADL

    using sc_dt::equal;
    using sc_dt::not_equal;
    using sc_dt::b_not;
    using sc_dt::b_and;
    using sc_dt::b_or;
    using sc_dt::b_xor;
    using sc_dt::lrotate;
    using sc_dt::rrotate;
    using sc_dt::reverse;
    using sc_dt::concat;
    using sc_dt::and_reduce;
    using sc_dt::or_reduce;
    using sc_dt::xor_reduce;
    using sc_dt::nand_reduce;
    using sc_dt::nor_reduce;
    using sc_dt::xnor_reduce;

#endif // defined( _MSC_VER )


// USINGS FOR sc_core:
//
// The explicit using for ::sc_core::wait is to remove an ambiguity with
// the constructor for the system's union wait on Unix and Linux. This
// causes problems with aCC, so users of aCC should explicitly select
// the SystemC wait functions using ::sc_core::wait(...). This is actually
// a good idea for SystemC programmers in general.

using namespace sc_core;

#if !defined( __HP_aCC )
    using ::sc_core::wait;
#endif // !defined( __HP_aCC )

#ifdef SC_USE_SC_STRING_OLD
	using   sc_dt::sc_string_old;
	typedef sc_dt::sc_string_old sc_string;
#endif
#ifdef SC_USE_STD_STRING
	typedef ::std::string sc_string;
#endif

#endif
