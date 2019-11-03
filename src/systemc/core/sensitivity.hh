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

#ifndef __SYSTEMC_CORE_SENSITIVITY_HH__
#define __SYSTEMC_CORE_SENSITIVITY_HH__

#include <set>
#include <vector>

#include "sim/eventq.hh"
#include "systemc/core/sched_event.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/ext/core/sc_port.hh"

namespace sc_core
{

class sc_event;
class sc_event_and_list;
class sc_event_or_list;
class sc_event_finder;
class sc_export_base;
class sc_interface;
class sc_port_base;

} // namespace sc_core

namespace sc_gem5
{

class Process;
class Event;

/*
 * Common sensitivity interface.
 */

class Sensitivity
{
  protected:
    Process *process;

    Sensitivity(Process *p) : process(p) {}
    virtual ~Sensitivity() {}

    virtual void addToEvent(const ::sc_core::sc_event *e) = 0;
    virtual void delFromEvent(const ::sc_core::sc_event *e) = 0;

  public:
    virtual void clear() = 0;

    void satisfy();
    virtual bool notifyWork(Event *e);
    bool notify(Event *e);

    enum Category
    {
        Static,
        Dynamic
    };

    virtual Category category() = 0;

    bool ofMethod();
};


/*
 * Dynamic vs. static sensitivity.
 */

class DynamicSensitivity : virtual public Sensitivity
{
  protected:
    DynamicSensitivity(Process *p) : Sensitivity(p) {}

    void addToEvent(const ::sc_core::sc_event *e) override;
    void delFromEvent(const ::sc_core::sc_event *e) override;

  public:
    Category category() override { return Dynamic; }
};

typedef std::vector<DynamicSensitivity *> DynamicSensitivities;


class StaticSensitivity : virtual public Sensitivity
{
  protected:
    StaticSensitivity(Process *p) : Sensitivity(p) {}

    void addToEvent(const ::sc_core::sc_event *e) override;
    void delFromEvent(const ::sc_core::sc_event *e) override;

  public:
    Category category() override { return Static; }
};

typedef std::vector<StaticSensitivity *> StaticSensitivities;


/*
 * Sensitivity to an event or events, which can be static or dynamic.
 */

class SensitivityEvent : virtual public Sensitivity
{
  protected:
    const ::sc_core::sc_event *event;

    SensitivityEvent(Process *p, const ::sc_core::sc_event *e=nullptr) :
        Sensitivity(p), event(e)
    {}

  public:
    void clear() override { delFromEvent(event); }
};

class SensitivityEvents : virtual public Sensitivity
{
  protected:
    std::set<const ::sc_core::sc_event *> events;

    SensitivityEvents(Process *p) : Sensitivity(p) {}
    SensitivityEvents(
            Process *p, const std::set<const ::sc_core::sc_event *> &s) :
        Sensitivity(p), events(s)
    {}

  public:
    void
    clear() override
    {
        for (auto event: events)
            delFromEvent(event);
    }

    void
    addEvent(const ::sc_core::sc_event *event)
    {
        events.insert(event);
        addToEvent(event);
    }
};


/*
 * Static sensitivities.
 */

void newStaticSensitivityEvent(Process *p, const sc_core::sc_event *e);
void newStaticSensitivityInterface(Process *p, const sc_core::sc_interface *i);
void newStaticSensitivityPort(Process *p, const sc_core::sc_port_base *pb);
void newStaticSensitivityExport(
        Process *p, const sc_core::sc_export_base *exp);
void newStaticSensitivityFinder(
        Process *p, const sc_core::sc_event_finder *f);


class StaticSensitivityEvent :
    public StaticSensitivity, public SensitivityEvent
{
    friend void newStaticSensitivityEvent(
            Process *p, const sc_core::sc_event *e);

  protected:
    StaticSensitivityEvent(Process *p, const sc_core::sc_event *e) :
        Sensitivity(p), StaticSensitivity(p), SensitivityEvent(p, e)
    {}
};

class StaticSensitivityInterface :
    public StaticSensitivity, public SensitivityEvent
{
    friend void newStaticSensitivityInterface(
            Process *p, const sc_core::sc_interface *i);
  protected:
    StaticSensitivityInterface(Process *p, const sc_core::sc_interface *i);
};

class StaticSensitivityPort :
    public StaticSensitivity, public SensitivityEvents
{
    friend void newStaticSensitivityPort(
            Process *p, const sc_core::sc_port_base *pb);

  protected:
    StaticSensitivityPort(Process *p) :
        Sensitivity(p), StaticSensitivity(p), SensitivityEvents(p)
    {}
};

class StaticSensitivityExport :
    public StaticSensitivity, public SensitivityEvent
{
  private:
    friend void newStaticSensitivityExport(
            Process *p, const sc_core::sc_export_base *exp);

    StaticSensitivityExport(Process *p, const sc_core::sc_export_base *exp);
};


class StaticSensitivityFinder :
    public StaticSensitivity, public SensitivityEvents
{
  private:
    const sc_core::sc_event_finder *finder;

    friend void newStaticSensitivityFinder(
            Process *p, const sc_core::sc_event_finder *f);

    StaticSensitivityFinder(Process *p, const sc_core::sc_event_finder *f) :
        Sensitivity(p), StaticSensitivity(p), SensitivityEvents(p), finder(f)
    {}

  public:
    const ::sc_core::sc_event &find(::sc_core::sc_interface *i);
};


/*
 * Dynamic sensitivities.
 */

void newDynamicSensitivityEvent(Process *p, const sc_core::sc_event *e);
void newDynamicSensitivityEventOrList(
        Process *p, const sc_core::sc_event_or_list *eol);
void newDynamicSensitivityEventAndList(
        Process *p, const sc_core::sc_event_and_list *eal);

class DynamicSensitivityEvent :
    public DynamicSensitivity, public SensitivityEvent
{
  private:
    friend void newDynamicSensitivityEvent(
            Process *p, const sc_core::sc_event *e);

    DynamicSensitivityEvent(Process *p, const sc_core::sc_event *e) :
        Sensitivity(p), DynamicSensitivity(p), SensitivityEvent(p, e)
    {}
};

class DynamicSensitivityEventOrList :
    public DynamicSensitivity, public SensitivityEvents
{
  private:
    friend void newDynamicSensitivityEventOrList(
            Process *p, const sc_core::sc_event_or_list *eol);

    DynamicSensitivityEventOrList(
            Process *p, const sc_core::sc_event_or_list *eol);

    bool notifyWork(Event *e) override;
};

//XXX This sensitivity can't be reused. To reset it, it has to be deleted and
//recreated. That works for dynamic sensitivities, but not for static.
//Fortunately processes can't be statically sensitive to sc_event_and_lists.
class DynamicSensitivityEventAndList :
    public DynamicSensitivity, public SensitivityEvents
{
  private:
    friend void newDynamicSensitivityEventAndList(
            Process *p, const sc_core::sc_event_and_list *eal);

    DynamicSensitivityEventAndList(
            Process *p, const sc_core::sc_event_and_list *eal);

    bool notifyWork(Event *e) override;
};

} // namespace sc_gem5

#endif  //__SYSTEMC_CORE_SENSITIVITY_HH__
