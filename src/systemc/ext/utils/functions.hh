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
 */

#ifndef __SYSTEMC_EXT_UTIL_FUNCTIONS_HH__
#define __SYSTEMC_EXT_UTIL_FUNCTIONS_HH__

#include <string>

namespace sc_dt
{

template <class T>
const T
sc_abs(const T &a)
{
    // return ( a >= 0 ? a : -a );
    // the code below is functionaly the same as the code above; the
    // difference is that the code below works for all arithmetic
    // SystemC datatypes.
    T z(a);
    z = 0;
    if (a >= z) {
        return a;
    } else {
        T c(a);
        c = -a;
        return c;
    }
}

template <class T>
const T sc_max(const T &a, const T &b) { return ((a >= b) ? a : b); }

template <class T>
const T sc_min(const T &a, const T &b) { return ((a <= b) ? a : b); }

} // namespace sc_dt

namespace sc_core
{

#define IEEE_1666_SYSTEMC 201101L

#define SC_VERSION_MAJOR 0
#define SC_VERSION_MINOR 1
#define SC_VERSION_PATCH 0
#define SC_VERSION_ORIGINATOR "gem5"
#define SC_VERSION_RELEASE_DATE "NA"
#define SC_VERSION_PRERELEASE "beta"
#define SC_IS_PRERELEASE true
#define SC_VERSION "0.1.0_beta-gem5"
#define SC_COPYRIGHT "Copyright 2018 Google, Inc."

extern const unsigned int sc_version_major;
extern const unsigned int sc_version_minor;
extern const unsigned int sc_version_patch;
extern const std::string sc_version_originator;
extern const std::string sc_version_release_date;
extern const std::string sc_version_prerelease;
extern const bool sc_is_prerelease;
extern const std::string sc_version_string;
extern const std::string sc_copyright_string;

static inline const char *
sc_release()
{
    return sc_version_string.c_str();
}
static inline const char *
sc_copyright()
{
    return sc_copyright_string.c_str();
}
const char *sc_version();

} // namespace sc_core

#endif  //__SYSTEMC_EXT_UTIL_FUNCTIONS_HH__
