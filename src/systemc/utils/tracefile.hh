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

#ifndef __SYSTEMC_UTILS_TRACEFILE_HH__
#define __SYSTEMC_UTILS_TRACEFILE_HH__

#include <iostream>
#include <string>
#include <vector>

#include "systemc/core/event.hh"
#include "systemc/ext/channel/sc_signal_in_if.hh"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/dt/fx/sc_fxnum.hh"
#include "systemc/ext/utils/sc_trace_file.hh"

class OutputStream;

namespace sc_gem5
{

class TraceValBase
{
  protected:
    int _width;

  public:
    TraceValBase(int _width) : _width(_width) {}
    virtual ~TraceValBase() {}

    int width() { return _width; }

    virtual void finalize() {};
    virtual bool check() = 0;
};

template <typename T, typename Base=TraceValBase>
class TraceVal : public Base
{
  private:
    const T *t;
    T oldVal;

  public:
    TraceVal(const T *_t, int _width) : Base(_width), t(_t), oldVal(*t)
    {}
    ~TraceVal() {}

    void finalize() override { oldVal = *t; }
    const T &value() { return oldVal; }

    bool
    check() override
    {
        bool changed = (*t != oldVal);
        oldVal = *t;
        return changed;
    }
};

template <typename T, typename Base>
class TraceVal<::sc_core::sc_signal_in_if<T>, Base> : public Base
{
  private:
    const ::sc_core::sc_signal_in_if<T> *iface;
    T oldVal;

  public:
    TraceVal(const ::sc_core::sc_signal_in_if<T> *_iface, int _width) :
        Base(_width), iface(_iface), oldVal(iface->read())
    {}
    ~TraceVal() {}

    void finalize() override { oldVal = iface->read(); }
    const T &value() { return oldVal; }

    bool
    check() override
    {
        T newVal = iface->read();
        bool changed = (newVal != oldVal);
        oldVal = newVal;
        return changed;
    }
};

template <typename Base>
class TraceVal<::sc_core::sc_event, Base> : public Base
{
  private:
    bool triggered;
    uint64_t oldStamp;
    const Event *event;

  public:
    TraceVal(const ::sc_core::sc_event *_event, int _width) :
        Base(_width), triggered(false), oldStamp(0),
        event(Event::getFromScEvent(_event))
    {}
    ~TraceVal() {}

    bool value() { return triggered; }
    void finalize() override { oldStamp = event->triggeredStamp(); }

    bool
    check() override
    {
        uint64_t newStamp = event->triggeredStamp();
        triggered = (oldStamp != newStamp);
        oldStamp = newStamp;
        return triggered;
    }
};

template <typename T, typename Base>
class TraceValFxnumBase : public Base
{
  private:
    const T *t;
    T oldVal;

  public:
    TraceValFxnumBase(const T *_t, int _width) :
        Base(_width), t(_t),
        oldVal(_t->m_params.type_params(), _t->m_params.enc(),
                _t->m_params.cast_switch(), 0)
    {}
    ~TraceValFxnumBase() {}

    void
    finalize() override
    {
        oldVal = *t;
        this->_width = t->wl();
    }

    const T &value() { return oldVal; }

    bool
    check() override
    {
        bool changed = (*t != oldVal);
        oldVal = *t;
        return changed;
    }
};

template <typename Base>
class TraceVal<::sc_dt::sc_fxnum, Base> :
    public TraceValFxnumBase<::sc_dt::sc_fxnum, Base>
{
  public:
    using TraceValFxnumBase<::sc_dt::sc_fxnum, Base>::TraceValFxnumBase;
    ~TraceVal() {}
};

template <typename Base>
class TraceVal<::sc_dt::sc_fxnum_fast, Base> :
    public TraceValFxnumBase<::sc_dt::sc_fxnum_fast, Base>
{
  public:
    using TraceValFxnumBase<::sc_dt::sc_fxnum_fast, Base>::TraceValFxnumBase;
    ~TraceVal() {}
};

class TraceFile : public sc_core::sc_trace_file
{
  protected:
    OutputStream *_os;
    uint64_t timeUnitTicks;
    double timeUnitValue;
    ::sc_core::sc_time_unit timeUnitUnit;

    bool _traceDeltas;

    TraceFile(const std::string &name);

    std::ostream &stream();

  public:
    ~TraceFile();

    void traceDeltas(bool on) { _traceDeltas = on; }

    void set_time_unit(double, ::sc_core::sc_time_unit) override;
    void finalizeTime();

    virtual void trace(bool delta) = 0;

    virtual void addTraceVal(const bool *v, const std::string &name) = 0;
    virtual void addTraceVal(const float *v, const std::string &name) = 0;
    virtual void addTraceVal(const double *v, const std::string &name) = 0;

    virtual void addTraceVal(const sc_dt::sc_logic *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_dt::sc_int_base *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_dt::sc_uint_base *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_dt::sc_signed *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_dt::sc_unsigned *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_dt::sc_bv_base *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_dt::sc_lv_base *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_dt::sc_fxval *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_dt::sc_fxval_fast *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_dt::sc_fxnum *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_dt::sc_fxnum_fast *v,
                             const std::string &name) = 0;

    virtual void addTraceVal(const sc_core::sc_event *v,
                             const std::string &name) = 0;
    virtual void addTraceVal(const sc_core::sc_time *v,
                             const std::string &name) = 0;

    virtual void addTraceVal(const unsigned char *v,
                             const std::string &name, int width) = 0;
    virtual void addTraceVal(const char *v, const std::string &name,
                             int width) = 0;
    virtual void addTraceVal(const unsigned short *v,
                             const std::string &name, int width) = 0;
    virtual void addTraceVal(const short *v, const std::string &name,
                             int width) = 0;
    virtual void addTraceVal(const unsigned int *v,
                             const std::string &name, int width) = 0;
    virtual void addTraceVal(const int *v, const std::string &name,
                             int width) = 0;
    virtual void addTraceVal(const unsigned long *v,
                             const std::string &name, int width) = 0;
    virtual void addTraceVal(const long *v, const std::string &name,
                             int width) = 0;

    virtual void addTraceVal(const sc_dt::int64 *v,
                             const std::string &name, int width) = 0;
    virtual void addTraceVal(const sc_dt::uint64 *v,
                             const std::string &name, int width) = 0;

    virtual void addTraceVal(const unsigned int *,
                             const std::string &name,
                             const char **literals) = 0;

    virtual void writeComment(const std::string &comment) = 0;
};

} // namespace sc_gem5

#endif // __SYSTEMC_UTILS_TRACEFILE_HH__
