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

#include "systemc/sc_time.hh"

#include "base/logging.hh"

namespace sc_core
{

sc_time::sc_time()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_time::sc_time(double, sc_time_unit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_time::sc_time(const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_time &
sc_time::operator = (const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

sc_dt::uint64
sc_time::value() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

double
sc_time::to_double() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0.0;
}
double
sc_time::to_seconds() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0.0;
}

const std::string
sc_time::to_string() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

bool
sc_time::operator == (const sc_time &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return true;
}

bool
sc_time::operator != (const sc_time &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

bool
sc_time::operator < (const sc_time &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

bool
sc_time::operator <= (const sc_time &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return true;
}

bool
sc_time::operator > (const sc_time &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

bool
sc_time::operator >= (const sc_time &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return true;
}

sc_time &
sc_time::operator += (const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

sc_time &
sc_time::operator -= (const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

sc_time &
sc_time::operator *= (double)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

sc_time &
sc_time::operator /= (double)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *this;
}

void
sc_time::print(std::ostream &) const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const sc_time
operator + (const sc_time &, const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_time();
}

const sc_time
operator - (const sc_time &, const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_time();
}

const sc_time
operator * (const sc_time &, double)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_time();
}

const sc_time
operator * (double, const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_time();
}

const sc_time
operator / (const sc_time &, double)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_time();
}

double
operator / (const sc_time &, const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0.0;
}

std::ostream &
operator << (std::ostream &os, const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return os;
}

const sc_time SC_ZERO_TIME;

void
sc_set_time_resolution(double, sc_time_unit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_time
sc_get_time_resolution()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_time();
}

const sc_time &
sc_max_time()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(const sc_time *)nullptr;
}

} // namespace sc_core
