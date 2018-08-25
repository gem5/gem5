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

#ifndef __SYSTEMC_CORE_PROCESS_HH__
#define __SYSTEMC_CORE_PROCESS_HH__

#include <functional>
#include <memory>
#include <vector>

#include "base/fiber.hh"
#include "sim/eventq.hh"
#include "systemc/core/bindinfo.hh"
#include "systemc/core/list.hh"
#include "systemc/core/object.hh"
#include "systemc/core/sched_event.hh"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/core/sc_export.hh"
#include "systemc/ext/core/sc_interface.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/ext/core/sc_port.hh"
#include "systemc/ext/core/sc_process_handle.hh"
#include "systemc/ext/utils/sc_report.hh"

namespace sc_gem5
{

class Sensitivity
{
  protected:
    Process *process;

  public:
    Sensitivity(Process *p) : process(p) {}
    virtual ~Sensitivity() {}

    virtual void notifyWork(Event *e);
    void notify(Event *e);
    void notify() { notify(nullptr); }

    const std::string name();
};

class SensitivityTimeout : virtual public Sensitivity
{
  private:
    void timeout();
    ScEvent timeoutEvent;

  public:
    SensitivityTimeout(Process *p, ::sc_core::sc_time t);
    ~SensitivityTimeout();
};

class SensitivityEvent : virtual public Sensitivity
{
  private:
    const ::sc_core::sc_event *event;

  public:
    SensitivityEvent(Process *p, const ::sc_core::sc_event *e);
    ~SensitivityEvent();
};

//XXX This sensitivity can't be reused. To reset it, it has to be deleted and
//recreated. That works for dynamic sensitivities, but not for static.
//Fortunately processes can't be statically sensitive to sc_event_and_lists.
class SensitivityEventAndList : virtual public Sensitivity
{
  private:
    const ::sc_core::sc_event_and_list *list;
    int count;

  public:
    SensitivityEventAndList(
            Process *p, const ::sc_core::sc_event_and_list *list);
    ~SensitivityEventAndList();

    void notifyWork(Event *e) override;
};

class SensitivityEventOrList : virtual public Sensitivity
{
  private:
    const ::sc_core::sc_event_or_list *list;

  public:
    SensitivityEventOrList(
            Process *p, const ::sc_core::sc_event_or_list *list);
    ~SensitivityEventOrList();
};

// Combined sensitivities. These trigger when any of their parts do.

class SensitivityTimeoutAndEvent :
    public SensitivityTimeout, public SensitivityEvent
{
  public:
    SensitivityTimeoutAndEvent(
            Process *p, ::sc_core::sc_time t, const ::sc_core::sc_event *e) :
        Sensitivity(p), SensitivityTimeout(p, t), SensitivityEvent(p, e)
    {}
};

class SensitivityTimeoutAndEventAndList :
    public SensitivityTimeout, public SensitivityEventAndList
{
  public:
    SensitivityTimeoutAndEventAndList(
            Process *p, ::sc_core::sc_time t,
            const ::sc_core::sc_event_and_list *eal) :
        Sensitivity(p), SensitivityTimeout(p, t),
        SensitivityEventAndList(p, eal)
    {}

    void notifyWork(Event *e) override;
};

class SensitivityTimeoutAndEventOrList :
    public SensitivityTimeout, public SensitivityEventOrList
{
  public:
    SensitivityTimeoutAndEventOrList(
            Process *p, ::sc_core::sc_time t,
            const ::sc_core::sc_event_or_list *eol) :
        Sensitivity(p), SensitivityTimeout(p, t),
        SensitivityEventOrList(p, eol)
    {}
};

typedef std::vector<Sensitivity *> Sensitivities;


/*
 * Pending sensitivities. These are records of sensitivities to install later,
 * once all the information to configure them is available.
 */

class PendingSensitivity
{
  protected:
    Process *process;

  public:
    virtual void finalize(Sensitivities &s) = 0;
    PendingSensitivity(Process *p) : process(p) {}
    virtual ~PendingSensitivity() {}
};

class PendingSensitivityEvent : public PendingSensitivity
{
  private:
    const sc_core::sc_event *event;

  public:
    PendingSensitivityEvent(Process *p, const sc_core::sc_event *e) :
        PendingSensitivity(p), event(e) {}

    void
    finalize(Sensitivities &s) override
    {
        s.push_back(new SensitivityEvent(process, event));
    }
};

class PendingSensitivityInterface : public PendingSensitivity
{
  private:
    const sc_core::sc_interface *interface;

  public:
    PendingSensitivityInterface(Process *p, const sc_core::sc_interface *i) :
        PendingSensitivity(p), interface(i)
    {}

    void
    finalize(Sensitivities &s) override
    {
        s.push_back(new SensitivityEvent(process,
                                         &interface->default_event()));
    }
};

class PendingSensitivityPort : public PendingSensitivity
{
  private:
    const sc_core::sc_port_base *port;

  public:
    PendingSensitivityPort(Process *p, const sc_core::sc_port_base *pb) :
        PendingSensitivity(p), port(pb)
    {}

    void
    finalize(Sensitivities &s) override
    {
        for (int i = 0; i < port->size(); i++) {
            const ::sc_core::sc_event *e =
                &port->_gem5Interface(i)->default_event();
            s.push_back(new SensitivityEvent(process, e));
        }
    }
};

class PendingSensitivityExport : public PendingSensitivity
{
  private:
    const sc_core::sc_export_base *exp;

  public:
    PendingSensitivityExport(Process *p, const sc_core::sc_export_base *exp) :
        PendingSensitivity(p), exp(exp)
    {}

    void
    finalize(Sensitivities &s) override
    {
        s.push_back(new SensitivityEvent(process,
                    &exp->get_interface()->default_event()));
    }
};

class PendingSensitivityFinder : public PendingSensitivity
{
  private:
    const sc_core::sc_event_finder *finder;

  public:
    PendingSensitivityFinder(Process *p, const sc_core::sc_event_finder *f) :
        PendingSensitivity(p), finder(f)
    {}

    void
    finalize(Sensitivities &s) override
    {
        s.push_back(new SensitivityEvent(process, &finder->find_event()));
    }
};

typedef std::vector<PendingSensitivity *> PendingSensitivities;


class Process : public ::sc_core::sc_process_b, public ListNode
{
  public:
    virtual ::sc_core::sc_curr_proc_kind procKind() const = 0;
    bool needsStart() const { return _needsStart; }
    void needsStart(bool ns) { _needsStart = ns; }
    bool dynamic() const { return _dynamic; }
    bool isUnwinding() const { return _isUnwinding; }
    void isUnwinding(bool v) { _isUnwinding = v; }
    bool terminated() const { return _terminated; }

    void forEachKid(const std::function<void(Process *)> &work);

    bool suspended() const { return _suspended; }
    bool disabled() const { return _disabled; }

    void suspend(bool inc_kids);
    void resume(bool inc_kids);
    void disable(bool inc_kids);
    void enable(bool inc_kids);

    void kill(bool inc_kids);
    void reset(bool inc_kids);
    virtual void throw_it(ExceptionWrapperBase &exc, bool inc_kids);

    void injectException(ExceptionWrapperBase &exc);
    ExceptionWrapperBase *excWrapper;

    void syncResetOn(bool inc_kids);
    void syncResetOff(bool inc_kids);

    void incref() { refCount++; }
    void decref() { refCount--; }

    const ::sc_core::sc_event &resetEvent() { return _resetEvent; }
    const ::sc_core::sc_event &terminatedEvent() { return _terminatedEvent; }

    // This should only be called before initialization.
    void dontInitialize();

    void setStackSize(size_t size) { stackSize = size; }

    void finalize();

    void run();

    void addStatic(PendingSensitivity *);
    void setDynamic(Sensitivity *);

    void satisfySensitivity(Sensitivity *);

    void ready();

    virtual Fiber *fiber() { return Fiber::primaryFiber(); }

    static Process *newest() { return _newest; }

    void lastReport(::sc_core::sc_report *report);
    ::sc_core::sc_report *lastReport() const;

  protected:
    Process(const char *name, ProcessFuncWrapper *func, bool _dynamic);

    static Process *_newest;

    virtual ~Process()
    {
        popListNode();
        delete func;
        for (auto s: staticSensitivities)
            delete s;
    }

    ::sc_core::sc_event _resetEvent;
    ::sc_core::sc_event _terminatedEvent;

    ProcessFuncWrapper *func;
    sc_core::sc_curr_proc_kind _procKind;
    bool _needsStart;
    bool _dynamic;
    bool _isUnwinding;
    bool _terminated;

    void terminate();

    bool _suspended;
    bool _suspendedReady;
    bool _disabled;

    bool _syncReset;

    int refCount;

    size_t stackSize;

    Sensitivities staticSensitivities;
    PendingSensitivities pendingStaticSensitivities;

    Sensitivity *dynamicSensitivity;

    std::unique_ptr<::sc_core::sc_report> _lastReport;
};

inline void
Sensitivity::notifyWork(Event *e)
{
    process->satisfySensitivity(this);
}

inline void
Sensitivity::notify(Event *e)
{
    if (!process->disabled())
        notifyWork(e);
}

inline const std::string
Sensitivity::name()
{
    return std::string(process->name()) + ".timeout";
}

} // namespace sc_gem5

#endif  //__SYSTEMC_CORE_PROCESS_HH__
