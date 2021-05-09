/*
 * Copyright (c) 2021 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __BASE_TEMPERATURE_HH__
#define __BASE_TEMPERATURE_HH__

#include <ostream>

namespace gem5
{

/**
 * The class stores temperatures in Kelvin and provides helper methods
 * to convert to/from Celsius.
 */
class Temperature
{

  private:
    /** Temperature in Kelvin */
    double value;

  public:
    /** Explicit constructor assigning a value. */
    explicit constexpr Temperature(double _value=0.0)
        : value(_value)
    {
    }

    static Temperature fromKelvin(double _value);
    static Temperature fromCelsius(double _value);
    static Temperature fromFahrenheit(double _value);

    constexpr double toKelvin() const { return value; }
    constexpr double toCelsius() const { return value - 273.15; }
    double toFahrenheit() const;

    constexpr bool
    operator>(const Temperature &rhs) const
    {
        return value > rhs.value;
    }

    constexpr bool
    operator>=(const Temperature &rhs) const
    {
        return value >= rhs.value;
    }

    constexpr bool
    operator<(const Temperature &rhs) const
    {
        return value < rhs.value;
    }

    constexpr bool
    operator<=(const Temperature &rhs) const
    {
        return value <= rhs.value;
    }

    constexpr bool
    operator==(const Temperature &rhs) const
    {
        return value == rhs.value;
    }

    constexpr bool
    operator!=(const Temperature &rhs) const
    {
        return value != rhs.value;
    }

    constexpr Temperature
    operator+(const Temperature &rhs) const
    {
        return Temperature(value + rhs.value);
    }

    constexpr Temperature
    operator-(const Temperature &rhs) const
    {
        return Temperature(value - rhs.value);
    }

    friend constexpr Temperature operator*(
        const Temperature &lhs, const double &rhs);

    friend constexpr Temperature operator*(
        const double &lhs, const Temperature &rhs);

    friend constexpr Temperature operator/(
        const Temperature &lhs, const double &rhs);

    Temperature &
    operator+=(const Temperature &rhs)
    {
        value += rhs.value;
        return *this;
    }

    Temperature &
    operator-=(const Temperature &rhs)
    {
        value -= rhs.value;
        return *this;
    }

    Temperature &
    operator*=(const double &rhs)
    {
        value *= rhs;
        return *this;
    }

    Temperature &
    operator/=(const double &rhs)
    {
        value /= rhs;
        return *this;
    }

    friend std::ostream &operator<<(std::ostream &out, const Temperature &t);
};

constexpr Temperature
operator*(const Temperature &lhs, const double &rhs)
{
    return Temperature(lhs.value * rhs);
}

constexpr Temperature
operator*(const double &lhs, const Temperature &rhs)
{
    return Temperature(lhs * rhs.value);
}

constexpr Temperature
operator/(const Temperature &lhs, const double &rhs)
{
    return Temperature(lhs.value / rhs);
}

} // namespace gem5

#endif // __BASE_TEMPERATURE_HH__
