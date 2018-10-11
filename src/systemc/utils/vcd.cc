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

#include "systemc/utils/vcd.hh"

#include <ctime>
#include <iomanip>

#include "base/bitfield.hh"
#include "base/cprintf.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_time.hh"
#include "systemc/ext/dt/bit/sc_bv_base.hh"
#include "systemc/ext/dt/bit/sc_logic.hh"
#include "systemc/ext/dt/bit/sc_lv_base.hh"
#include "systemc/ext/dt/fx/sc_fxnum.hh"
#include "systemc/ext/dt/fx/sc_fxval.hh"
#include "systemc/ext/dt/int/sc_int_base.hh"
#include "systemc/ext/dt/int/sc_signed.hh"
#include "systemc/ext/dt/int/sc_uint_base.hh"
#include "systemc/ext/dt/int/sc_unsigned.hh"
#include "systemc/ext/utils/functions.hh"

namespace sc_gem5
{

namespace
{

std::string
cleanName(std::string name)
{
    for (int i = 0; i < name.length(); i++) {
        if (name[i] == '[')
            name[i] = '(';
        else if (name[i] == ']')
            name[i] = ')';
    }
    return name;
}

} // anonymous namespace

class VcdTraceValBase : public TraceValBase
{
  protected:
    std::string _vcdName;

    const char *
    stripLeadingBits(const char *orig)
    {
        const char first = orig[0];

        if (first != 'z' && first != 'x' && first != '0')
            return orig;

        const char *res = orig;
        while (*++res == first) {}

        if (first != '0' || *res != '1')
            res--;

        return res;
    }

    char
    scLogicToVcdState(char in)
    {
        switch (in) {
          case 'U':
          case 'X':
          case 'W':
          case 'D':
            return 'x';
          case '0':
          case 'L':
            return '0';
          case '1':
          case 'H':
            return '1';
          case 'Z':
            return 'z';
          default:
            return '?';
        }
    }

    void
    printVal(std::ostream &os, const std::string &rep)
    {
        switch (width()) {
          case 0:
            return;
          case 1:
            os << rep << vcdName() << std::endl;;
            return;
          default:
            os << "b" << stripLeadingBits(rep.c_str()) << " " <<
                vcdName() << std::endl;
            return;
        }
    }

  public:
    VcdTraceValBase(int width) : TraceValBase(width) {}
    ~VcdTraceValBase() {}

    void vcdName(const std::string &vcd_name) { _vcdName = vcd_name; }
    const std::string &vcdName() { return _vcdName; }
    virtual std::string vcdType() { return "wire"; }

    virtual void output(std::ostream &os) = 0;
};

void
VcdTraceScope::addValue(const std::string &name, VcdTraceValBase *value)
{
    size_t pos = name.find_first_of('.');
    if (pos == std::string::npos) {
        values.emplace_back(name, value);
    } else {
        std::string sname = name.substr(0, pos);
        auto it = scopes.find(sname);
        if (it == scopes.end())
            it = scopes.emplace(sname, new VcdTraceScope).first;
        it->second->addValue(name.substr(pos + 1), value);
    }
}

void
VcdTraceScope::output(const std::string &name, std::ostream &os)
{
    os << "$scope module " << name << " $end" << std::endl;

    for (auto &p: values) {
        const std::string &name = p.first;
        VcdTraceValBase *value = p.second;

        int w = value->width();
        if (w <= 0) {
            std::string msg = csprintf("'%s' has 0 bits", name);
            // The typo in this error message is intentional to match the
            // Accellera output.
            SC_REPORT_ERROR("(E710) object cannot not be traced", msg.c_str());
            return;
        }

        std::string clean_name = cleanName(name);
        if (w == 1) {
            ccprintf(os, "$var %s  % 3d  %s  %s       $end\n",
                     value->vcdType(), w, value->vcdName(), clean_name);
        } else {
            ccprintf(os, "$var %s  % 3d  %s  %s [%d:0]  $end\n",
                     value->vcdType(), w, value->vcdName(), clean_name, w - 1);
        }
    }

    for (auto &p: scopes)
        p.second->output(p.first, os);

    os << "$upscope $end" << std::endl;
}

template <typename T>
class VcdTraceVal : public TraceVal<T, VcdTraceValBase>
{
  public:
    typedef T TracedType;

    VcdTraceVal(const T* t, const std::string &vcd_name, int width) :
        TraceVal<T, VcdTraceValBase>(t, width)
    {
        this->vcdName(vcd_name);
    }
};

std::string
VcdTraceFile::nextSignalName()
{
    std::string name(_nextName);

    bool carry = false;
    int pos = NextNameChars - 1;
    do {
        carry = (_nextName[pos] == 'z');
        if (carry)
            _nextName[pos--] = 'a';
        else
            _nextName[pos--]++;
    } while (carry && pos >= 0);

    return name;
}

void
VcdTraceFile::initialize()
{
    finalizeTime();

    // Date.
    stream() << "$date" << std::endl;
    time_t long_time;
    time(&long_time);
    struct tm *p_tm = localtime(&long_time);
    stream() << std::put_time(p_tm, "     %b %d, %Y       %H:%M:%S\n");
    stream() << "$end" << std::endl << std::endl;

    // Version.
    stream() << "$version" << std::endl;
    stream() << " " << ::sc_core::sc_version() << std::endl;
    stream() << "$end" << std::endl << std::endl;

    // Timescale.
    stream() << "$timescale" << std::endl;
    stream() << "     " << ::sc_core::sc_time::from_value(timeUnitTicks) <<
        std::endl;
    stream() << "$end" << std::endl << std::endl;

    for (auto tv: traceVals)
        tv->finalize();

    topScope.output("SystemC", stream());

    stream() << "$enddefinitions  $end" << std::endl << std::endl;

    Tick now = scheduler.getCurTick();

    std::string timedump_comment =
        csprintf("All initial values are dumped below at time "
                 "%g sec = %g timescale units.",
                 static_cast<double>(now) / SimClock::Float::s,
                 static_cast<double>(now / timeUnitTicks));
    writeComment(timedump_comment);

    lastPrintedTime = now / timeUnitTicks;

    stream() << "$dumpvars" << std::endl;
    for (auto tv: traceVals)
        tv->output(stream());
    stream() << "$end" << std::endl << std::endl;

    initialized = true;
}

VcdTraceFile::~VcdTraceFile()
{
    for (auto tv: traceVals)
        delete tv;
    traceVals.clear();

    if (timeUnitTicks)
        ccprintf(stream(), "#%u\n", scheduler.getCurTick() / timeUnitTicks);
}

void
VcdTraceFile::trace(bool delta)
{
    if (!delta)
        deltasAtNow = 0;

    uint64_t deltaOffset = deltasAtNow;

    if (delta)
        deltaOffset = deltasAtNow++;

    if (_traceDeltas != delta)
        return;

    if (!initialized) {
        initialize();
        return;
    }

    Tick now = scheduler.getCurTick() / timeUnitTicks + deltaOffset;

    if (now <= lastPrintedTime) {
        // TODO warn about reversed time?
        return;
    }

    bool time_printed = false;
    for (auto tv: traceVals) {
        if (tv->check()) {
            if (!time_printed) {
                lastPrintedTime = now;
                ccprintf(stream(), "#%u\n", now);
                time_printed = true;
            }

            tv->output(stream());
        }
    }
    if (time_printed)
        stream() << std::endl;
}

class VcdTraceValBool : public VcdTraceVal<bool>
{
  public:
    using VcdTraceVal<bool>::VcdTraceVal;

    void
    output(std::ostream &os) override
    {
        printVal(os, this->value() ? "1" : "0");
    }
};

void
VcdTraceFile::addTraceVal(const bool *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValBool>(v, name);
}

template <typename T>
class VcdTraceValFloat : public VcdTraceVal<T>
{
  public:
    using VcdTraceVal<T>::VcdTraceVal;

    std::string vcdType() override { return "real"; }

    void
    output(std::ostream &os) override
    {
        ccprintf(os, "r%.16g %s\n", this->value(), this->vcdName());
    }
};

void
VcdTraceFile::addTraceVal(const float *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValFloat<float>>(v, name);
}
void
VcdTraceFile::addTraceVal(const double *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValFloat<double>>(v, name);
}

class VcdTraceValScLogic : public VcdTraceVal<sc_dt::sc_logic>
{
  public:
    using VcdTraceVal<sc_dt::sc_logic>::VcdTraceVal;

    void
    output(std::ostream &os) override
    {
        char str[2] = {
            scLogicToVcdState(value().to_char()),
            '\0'
        };
        printVal(os, str);
    }
};

void
VcdTraceFile::addTraceVal(const sc_dt::sc_logic *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValScLogic>(v, name);
}

template <typename T>
class VcdTraceValFinite : public VcdTraceVal<T>
{
  public:
    using VcdTraceVal<T>::VcdTraceVal;

    void
    finalize() override
    {
        VcdTraceVal<T>::finalize();
        this->_width = this->value().length();
    }

    void
    output(std::ostream &os) override
    {
        std::string str;
        const int w = this->width();

        str.reserve(w);
        for (int i = w - 1; i >= 0; i--)
            str += this->value()[i].to_bool() ? '1' : '0';

        this->printVal(os, str);
    }
};

void
VcdTraceFile::addTraceVal(const sc_dt::sc_int_base *v,
                          const std::string &name)
{
    addNewTraceVal<VcdTraceValFinite<sc_dt::sc_int_base>>(v, name);
}
void
VcdTraceFile::addTraceVal(const sc_dt::sc_uint_base *v,
                          const std::string &name)
{
    addNewTraceVal<VcdTraceValFinite<sc_dt::sc_uint_base>>(v, name);
}

void
VcdTraceFile::addTraceVal(const sc_dt::sc_signed *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValFinite<sc_dt::sc_signed>>(v, name);
}
void
VcdTraceFile::addTraceVal(const sc_dt::sc_unsigned *v,
                          const std::string &name)
{
    addNewTraceVal<VcdTraceValFinite<sc_dt::sc_unsigned>>(v, name);
}

template <typename T>
class VcdTraceValLogic : public VcdTraceVal<T>
{
  public:
    using VcdTraceVal<T>::VcdTraceVal;

    void
    finalize() override
    {
        VcdTraceVal<T>::finalize();
        this->_width = this->value().length();
    }

    void
    output(std::ostream &os) override
    {
        this->printVal(os, this->value().to_string());
    }
};

void
VcdTraceFile::addTraceVal(const sc_dt::sc_bv_base *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValLogic<::sc_dt::sc_bv_base>>(v, name);
}
void
VcdTraceFile::addTraceVal(const sc_dt::sc_lv_base *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValLogic<::sc_dt::sc_lv_base>>(v, name);
}

template <typename T>
class VcdTraceValFxval : public VcdTraceVal<T>
{
  public:
    using VcdTraceVal<T>::VcdTraceVal;

    std::string vcdType() override { return "real"; }

    void
    output(std::ostream &os) override
    {
        ccprintf(os, "r%.16g %s\n",
                this->value().to_double(), this->vcdName());
    }
};

void
VcdTraceFile::addTraceVal(const sc_dt::sc_fxval *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValFxval<sc_dt::sc_fxval>>(v, name);
}
void
VcdTraceFile::addTraceVal(const sc_dt::sc_fxval_fast *v,
                          const std::string &name)
{
    addNewTraceVal<VcdTraceValFxval<sc_dt::sc_fxval_fast>>(v, name);
}

template <typename T>
class VcdTraceValFxnum : public VcdTraceVal<T>
{
  public:
    using VcdTraceVal<T>::VcdTraceVal;

    void
    output(std::ostream &os) override
    {
        std::string str;
        const int w = this->width();

        str.reserve(w);
        for (int i = w - 1; i >= 0; i--)
            str += this->value()[i] ? '1' : '0';

        this->printVal(os, str);
    }
};

void
VcdTraceFile::addTraceVal(const sc_dt::sc_fxnum *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValFxnum<::sc_dt::sc_fxnum>>(v, name);
}
void
VcdTraceFile::addTraceVal(const sc_dt::sc_fxnum_fast *v,
                          const std::string &name)
{
    addNewTraceVal<VcdTraceValFxnum<::sc_dt::sc_fxnum_fast>>(v, name);
}

class VcdTraceValEvent : public VcdTraceVal<::sc_core::sc_event>
{
  public:
    using VcdTraceVal<::sc_core::sc_event>::VcdTraceVal;

    std::string vcdType() override { return "event"; }

    void
    output(std::ostream &os) override
    {
        if (value())
            printVal(os, "1");
        else
            os << std::endl;
    }
};

void
VcdTraceFile::addTraceVal(const sc_core::sc_event *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValEvent>(v, name);
}

class VcdTraceValTime : public VcdTraceVal<::sc_core::sc_time>
{
  private:
    static const int TimeWidth = 64;

  public:
    using VcdTraceVal<::sc_core::sc_time>::VcdTraceVal;

    std::string vcdType() override { return "time"; }

    void
    finalize() override
    {
        VcdTraceVal<::sc_core::sc_time>::finalize();
        _width = TimeWidth;
    }

    void
    output(std::ostream &os) override
    {
        char str[TimeWidth + 1];
        str[TimeWidth] = '\0';

        const uint64_t val = value().value();
        for (int i = 0; i < TimeWidth; i++)
            str[i] = ::bits(val, TimeWidth - i - 1) ? '1' : '0';

        printVal(os, str);
    }
};
void
VcdTraceFile::addTraceVal(const sc_core::sc_time *v, const std::string &name)
{
    addNewTraceVal<VcdTraceValTime>(v, name);
}

template <typename T>
class VcdTraceValInt : public VcdTraceVal<T>
{
  public:
    using VcdTraceVal<T>::VcdTraceVal;

    void
    output(std::ostream &os) override
    {
        const int w = this->width();
        char str[w + 1];
        str[w] = '\0';

        const uint64_t val =
            static_cast<uint64_t>(this->value()) & ::mask(sizeof(T) * 8);

        if (::mask(w) < val) {
            for (int i = 0; i < w; i++)
                str[i] = 'x';
        } else {
            for (int i = 0; i < w; i++)
                str[i] = ::bits(val, w - i - 1) ? '1' : '0';
        }

        this->printVal(os, str);
    }
};

void
VcdTraceFile::addTraceVal(const unsigned char *v, const std::string &name,
                          int width)
{
    addNewTraceVal<VcdTraceValInt<unsigned char>>(v, name, width);
}
void
VcdTraceFile::addTraceVal(const char *v, const std::string &name, int width)
{
    addNewTraceVal<VcdTraceValInt<char>>(v, name, width);
}
void
VcdTraceFile::addTraceVal(const unsigned short *v, const std::string &name,
                          int width)
{
    addNewTraceVal<VcdTraceValInt<unsigned short>>(v, name, width);
}
void
VcdTraceFile::addTraceVal(const short *v, const std::string &name, int width)
{
    addNewTraceVal<VcdTraceValInt<short>>(v, name, width);
}
void
VcdTraceFile::addTraceVal(const unsigned int *v, const std::string &name,
                          int width)
{
    addNewTraceVal<VcdTraceValInt<unsigned int>>(v, name, width);
}
void
VcdTraceFile::addTraceVal(const int *v, const std::string &name, int width)
{
    addNewTraceVal<VcdTraceValInt<int>>(v, name, width);
}
void
VcdTraceFile::addTraceVal(const unsigned long *v, const std::string &name,
                          int width)
{
    addNewTraceVal<VcdTraceValInt<unsigned long>>(v, name, width);
}
void
VcdTraceFile::addTraceVal(const long *v, const std::string &name, int width)
{
    addNewTraceVal<VcdTraceValInt<long>>(v, name, width);
}

void
VcdTraceFile::addTraceVal(const sc_dt::int64 *v, const std::string &name,
                          int width)
{
    addNewTraceVal<VcdTraceValInt<sc_dt::int64>>(v, name, width);
}
void
VcdTraceFile::addTraceVal(const sc_dt::uint64 *v, const std::string &name,
                          int width)
{
    addNewTraceVal<VcdTraceValInt<sc_dt::uint64>>(v, name, width);
}

void
VcdTraceFile::addTraceVal(const unsigned int *v, const std::string &name,
                          const char **literals)
{
    uint64_t count = 0;
    while (*literals++)
        count++;

    int bits = 0;
    while (count >> bits)
        bits++;

    addNewTraceVal<VcdTraceValInt<unsigned int>>(v, name, bits);
}

void
VcdTraceFile::writeComment(const std::string &comment)
{
    stream() << "$comment" << std::endl;
    stream() << comment << std::endl;
    stream() << "$end" << std::endl << std::endl;
}

} // namespace sc_gem5
