/*
 * Copyright (c) 2021 The Regents of the University of California.
 * All rights reserved.
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

#ifndef __BASE_STATS_UNITS_HH__
#define __BASE_STATS_UNITS_HH__

#include <type_traits>

#include "base/compiler.hh"
#include "base/cprintf.hh"

namespace gem5
{

/**
 * Convenience macros to declare the unit of a stat.
 */
#define UNIT_CYCLE                                                            \
    GEM5_DEPRECATED_MACRO(UNIT_CYCLE, statistics::units::Cycle::get(),        \
                          "Use statistics::units::Cycle::get()")
#define UNIT_TICK                                                             \
    GEM5_DEPRECATED_MACRO(UNIT_TICK, statistics::units::Tick::get(),          \
                          "Use statistics::units::Tick::get()")
#define UNIT_SECOND                                                           \
    GEM5_DEPRECATED_MACRO(UNIT_SECOND, statistics::units::Second::get(),      \
                          "Use statistics::units::Second::get()")
#define UNIT_BIT                                                              \
    GEM5_DEPRECATED_MACRO(UNIT_BIT, statistics::units::Bit::get(),            \
                          "Use statistics::units::Bit::get()")
#define UNIT_BYTE                                                             \
    GEM5_DEPRECATED_MACRO(UNIT_BYTE, statistics::units::Byte::get(),          \
                          "Use statistics::units::Byte::get()")
#define UNIT_JOULE                                                            \
    GEM5_DEPRECATED_MACRO(UNIT_JOULE, statistics::units::Joule::get(),        \
                          "Use statistics::units::Joule::get()")
#define UNIT_VOLT                                                             \
    GEM5_DEPRECATED_MACRO(UNIT_VOLD, statistics::units::Volt::get(),          \
                          "Use statistics::units::Volt::get()")
#define UNIT_CELSIUS                                                          \
    GEM5_DEPRECATED_MACRO(UNIT_CELSIUS,                                       \
                          statistics::units::DegreeCelsius::get(),            \
                          "Use statistics::units::DegreeCelsius::get()")
#define UNIT_RATE(T1, T2)                                                     \
    GEM5_DEPRECATED_MACRO(UNIT_RATE,                                          \
                          (statistics::units::Rate<T1, T2>::get()),           \
                          "Use statistics::units::Rate<T1, T2>::get()")
#define UNIT_RATIO                                                            \
    GEM5_DEPRECATED_MACRO(UNIT_RATIO, statistics::units::Ratio::get(),        \
                          "Use statistics::units::Ratio::get()")
#define UNIT_COUNT                                                            \
    GEM5_DEPRECATED_MACRO(UNIT_COUNT, statistics::units::Count::get(),        \
                          "Use statistics::units::Count::get()")
#define UNIT_WATT                                                             \
    GEM5_DEPRECATED_MACRO(UNIT_WATT, statistics::units::Watt::get(),          \
                          "Use statistics::units::Watt::get()")
#define UNIT_UNSPECIFIED                                                      \
    GEM5_DEPRECATED_MACRO(UNIT_UNSPECIFIED,                                   \
                          statistics::units::Unspecified::get(),              \
                          "Use statistics::units::Unspecified::get()")

namespace statistics
{

/**
 * Units for Stats.
 *
 * This header file provides an ability to associate a stat object with a
 * specific unit.
 *
 * The supported units are:
 *   - Cycle: represents clock cycles.
 *   - Tick: represents the count of gem5's Tick.
 *   - Second: represents the base unit of time defined by SI.
 *   - Bit: represents the number of computer bits.
 *   - Byte: represents 8 bits.
 *   - Volt: a SI derived unit measuring potential difference.
 *   - Joule: represents joule, a unit of energy, as defined by SI.
 *   - Watt: represents 1 watt, where 1 watt = 1 joule / second.
 *   - Celsius: represents 1 Celsius degree as defined by SI.
 *   - Rate(T1, T2): represents the unit of a quantity of T1 divided by
 *                   a quantity of T2.
 *   - Ratio: represents the unit of a quantity of unit T divided by a quantity
 *            of T.
 *   - Count: represents the count of a quantity that is not defined above.
 *   - Unspecified: the unit of the stat is unspecified.
 *
 * Each unit class is intended to be a singleton, which means only each unit
 * class has at most one object of that class exist throughout the program.
 * Therefore, copy constructors and assignment operators are deleted functions.
 *
 * When any of the following criteria is met, a new unit should be added,
 *   - The new unit is significant enough to be not included in Count unit.
 *     (e.g. Cycle unit, Tick unit)
 */
namespace units
{

/**
 * The Base class is the parent class of all unit classes.
 * This class is intended to an abstract class specifying common behaviors of
 * all unit classes.
 */
class Base
{
  public:
    virtual std::string getUnitString() const = 0;
};

class Cycle : public Base
{
  private:
    Cycle() {}

  public:
    Cycle(Cycle const &) = delete;
    void operator=(Cycle const &) = delete;

    static Cycle *
    get()
    {
        static Cycle instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Cycle";
    }

    std::string
    getUnitString() const override
    {
        return Cycle::toString();
    }
};

class Tick : public Base
{
  private:
    Tick() {}

  public:
    Tick(Tick const &) = delete;
    void operator=(Tick const &) = delete;

    static Tick *
    get()
    {
        static Tick instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Tick";
    }

    std::string
    getUnitString() const override
    {
        return Tick::toString();
    }
};

class Second : public Base
{
  private:
    Second() {}

  public:
    Second(Second const &) = delete;
    void operator=(Second const &) = delete;

    static Second *
    get()
    {
        static Second instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Second";
    }

    std::string
    getUnitString() const override
    {
        return Second::toString();
    }
};

class Bit : public Base
{
  private:
    Bit() {}

  public:
    Bit(Bit const &) = delete;
    void operator=(Bit const &) = delete;

    static Bit *
    get()
    {
        static Bit instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Bit";
    }

    std::string
    getUnitString() const override
    {
        return Bit::toString();
    }
};

class Byte : public Base
{
  private:
    Byte() {}

  public:
    Byte(Byte const &) = delete;
    void operator=(Byte const &) = delete;

    static Byte *
    get()
    {
        static Byte instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Byte";
    }

    std::string
    getUnitString() const override
    {
        return Byte::toString();
    }
};

class Watt : public Base
{
  private:
    Watt() {}

  public:
    Watt(Watt const &) = delete;
    void operator=(Watt const &) = delete;

    static Watt *
    get()
    {
        static Watt instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Watt";
    }

    std::string
    getUnitString() const override
    {
        return Watt::toString();
    }
};

class Joule : public Base
{
  private:
    Joule() {}

  public:
    Joule(Joule const &) = delete;
    void operator=(Joule const &) = delete;

    static Joule *
    get()
    {
        static Joule instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Joule";
    }

    std::string
    getUnitString() const override
    {
        return Joule::toString();
    }
};

class Volt : public Base
{
  private:
    Volt() {}

  public:
    Volt(Volt const &) = delete;
    void operator=(Volt const &) = delete;

    static Volt *
    get()
    {
        static Volt instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Volt";
    }

    std::string
    getUnitString() const override
    {
        return Volt::toString();
    }
};

class DegreeCelsius : public Base
{
  private:
    DegreeCelsius() {}

  public:
    DegreeCelsius(DegreeCelsius const &) = delete;
    void operator=(DegreeCelsius const &) = delete;

    static DegreeCelsius *
    get()
    {
        static DegreeCelsius instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Celsius";
    }

    std::string
    getUnitString() const override
    {
        return DegreeCelsius::toString();
    }
};

class Count : public Base
{
  private:
    Count() {}

  public:
    Count(Count const &) = delete;
    void operator=(Count const &) = delete;

    static Count *
    get()
    {
        static Count instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Count";
    }

    std::string
    getUnitString() const override
    {
        return Count::toString();
    }
};

class Ratio : public Base
{
  private:
    Ratio() {}

  public:
    Ratio(Ratio const &) = delete;
    void operator=(Ratio const &) = delete;

    static Ratio *
    get()
    {
        static Ratio instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Ratio";
    }

    std::string
    getUnitString() const override
    {
        return Ratio::toString();
    }
};

class Unspecified : public Base
{
  private:
    Unspecified() {}

  public:
    Unspecified(Unspecified const &) = delete;
    void operator=(Unspecified const &) = delete;

    static Unspecified *
    get()
    {
        static Unspecified instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return "Unspecified";
    }

    std::string
    getUnitString() const override
    {
        return Unspecified::toString();
    }
};

template <typename T1, typename T2>
class Rate : public Base
{
    static_assert(std::is_base_of_v<Base, T1>,
                  "Rate(T1,T2) must have "
                  "T1 and T2 derived from statistics::units::Base");
    static_assert(std::is_base_of_v<Base, T2>,
                  "Rate(T1,T2) must have "
                  "T1 and T2 derived from statistics::units::Base");
    static_assert(!std::is_same_v<T1, T2> || std::is_same_v<T1, Count> ||
                      std::is_same_v<T1, Unspecified>,
                  "Rate(T1,T2) must have T1 and T2 of different types; "
                  "otherwise, it would be a Ratio");

  private:
    Rate<T1, T2>() {}

  public:
    Rate<T1, T2>(Rate<T1, T2> const &) = delete;
    void operator=(Rate<T1, T2> const &) = delete;

    static Rate<T1, T2> *
    get()
    {
        static Rate<T1, T2> instance;
        return &instance;
    }

    static std::string
    toString()
    {
        return csprintf("(%s/%s)", T1::toString(), T2::toString());
    }

    std::string
    getUnitString() const override
    {
        return Rate<T1, T2>::toString();
    }
};

} // namespace units
} // namespace statistics
} // namespace gem5

#endif // __BASE_STATS_UNITS_HH__
