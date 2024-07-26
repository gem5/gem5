/*
 * Copyright (c) 2013 ARM Limited
 * Copyright (c) 2022-2023 The University of Edinburgh
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
 * Copyright (c) 2020 Inria
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

/**
 * @file This file describes the base components used for the probe system.
 * There are currently 3 components:
 *
 * ProbePoint:          an event probe point i.e. sends a notify from the
 *                      point at which an instruction was committed.
 *
 * ProbeListener:       a listener provides a notify method that is called when
 *                      a probe point event occurs. Multiple ProbeListeners
 *                      can be added to each ProbePoint.
 *
 * ProbeListenerObject: a wrapper around a SimObject that can connect to
 *                      another SimObject on which it will add ProbeListeners.
 *
 * ProbeManager:        used to match up ProbeListeners and ProbePoints.
 *                      At <b>simulation init</b> this is handled by
 *                      regProbePoints followed by regProbeListeners being
 *                      called on each SimObject in hierarchical ordering.
 *                      ProbeListeners can be added/removed dynamically at
 *                      runtime.
 */

#ifndef __SIM_PROBE_PROBE_HH__
#define __SIM_PROBE_PROBE_HH__

#include <string>
#include <vector>

#include "base/compiler.hh"
#include "base/trace.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/** Forward declarations. */
class ProbeConnection;
class ProbeManager;
class ProbeListener;
struct ProbeListenerObjectParams;

/**
 * Name space containing shared probe point declarations.
 *
 * Probe types that are shared between multiple types of SimObjects
 * should live in this name space. This makes it possible to use a
 * common instrumentation interface for devices such as PMUs that have
 * different implementations in different ISAs.
 */
namespace probing
{
/* Note: This is only here for documentation purposes, new probe
 * points should normally be declared in their own header files. See
 * for example pmu.hh.
 */
}

class ProbeConnection
{
  public:
    ProbeConnection(ProbeManager* _manager,
               std::unique_ptr<ProbeListener> _listener);
    ~ProbeConnection();
    ProbeConnection(const ProbeConnection& other) = delete;

 protected:
    std::unique_ptr<ProbeListener> listener;
    ProbeManager* manager;
};
using ProbeConnectionPtr = std::unique_ptr<ProbeConnection>;

/**
 * ProbeListener base class; here to simplify things like containers
 * containing multiple types of ProbeListener.
 *
 * Note a ProbeListener is added to the ProbePoint in constructor by
 * using the ProbeManager passed in.
 */
class ProbeListener
{
  public:
    ProbeListener(std::string _name) : name(std::move(_name)) {}
    virtual ~ProbeListener();
    ProbeListener(const ProbeListener& other) = delete;
    ProbeListener& operator=(const ProbeListener& other) = delete;
    ProbeListener(ProbeListener&& other) noexcept = delete;
    ProbeListener& operator=(ProbeListener&& other) noexcept = delete;
    const std::string& getName() const { return name; }

  protected:
    const std::string name;
};

/**
 * ProbeListener base class; again used to simplify use of ProbePoints
 * in containers and used as to define interface for adding removing
 * listeners to the ProbePoint.
 */
class ProbePoint
{
  protected:
    const std::string name;
  public:
    ProbePoint(ProbeManager *manager, const std::string &name);
    virtual ~ProbePoint() {}

    virtual void addListener(ProbeListener *listener) = 0;
    virtual void removeListener(ProbeListener *listener) = 0;
    const std::string& getName() const { return name; }
};

/**
 * ProbeManager is a conduit class that lives on each SimObject,
 *  and is used to match up probe listeners with probe points.
 */
class ProbeManager
{
  private:
    /** Required for sensible debug messages.*/
    GEM5_CLASS_VAR_USED const SimObject *object;
    /** Vector for name look-up. */
    std::vector<ProbePoint *> points;

  public:
    ProbeManager(SimObject *obj)
        : object(obj)
    {}
    virtual ~ProbeManager() {}

    /**
     * @brief Add a ProbeListener to the ProbePoint named by pointName.
     *        If the name doesn't resolve a ProbePoint return false.
     * @param point_name name of the ProbePoint to add the ProbeListener to.
     * @param listener the ProbeListener to add.
     * @return true if added, false otherwise.
     */
    bool addListener(std::string_view point_name, ProbeListener &listener);

    /**
     * @brief Remove a ProbeListener from the ProbePoint named by pointName.
     *        If the name doesn't resolve a ProbePoint return false.
     * @param point_name the name of the ProbePoint to remove the ProbeListener
     *        from.
     * @param listener the ProbeListener to remove.
     * @return true if removed, false otherwise.
     */
    bool removeListener(std::string_view point_name, ProbeListener &listener);

    /**
     * @brief Add a ProbePoint to this SimObject ProbeManager.
     * @param point the ProbePoint to add.
     */
    void addPoint(ProbePoint &point);
    ProbePoint *getProbePoint(std::string_view point_name) const;

    template<typename Listener, typename... Args>
    ProbeConnectionPtr connect(Args&&... args)
    {
        return std::make_unique<ProbeConnection>(
            this, std::make_unique<Listener>(std::forward<Args>(args)...));
    }
};

/**
 * This class is a minimal wrapper around SimObject. It is used to declare
 * a python derived object that can be added as a ProbeListener to any other
 * SimObject.
 *
 * It instantiates manager from a call to Parent.any.
 * The vector of listeners is used simply to hold onto listeners until the
 * ProbeListenerObject is destroyed.
 */
class ProbeListenerObject : public SimObject
{
  protected:
    ProbeManager *manager;
    std::vector<std::unique_ptr<ProbeConnection>> listeners;

  public:
    explicit ProbeListenerObject(const ProbeListenerObjectParams &params);
    ~ProbeListenerObject() override = default;
    ProbeManager* getProbeManager() { return manager; }

    template<typename T, typename... Args>
    void connectListener(Args&&... args) {
        listeners.push_back(getProbeManager()->connect<T>(
            std::forward<Args>(args)...));

    }
};

/**
 * ProbeListenerArgBase is used to define the base interface to a
 * ProbeListenerArg (i.e the notify method on specific type).
 *
 * It is necessary to split this out from ProbeListenerArg, as that
 * templates off the class containing the function that notify calls.
 */
template <class Arg>
class ProbeListenerArgBase : public ProbeListener
{
  public:
    ProbeListenerArgBase(std::string name)
        : ProbeListener(std::move(name))
    {}
    virtual void notify(const Arg &val) = 0;
};

/**
 * ProbeListenerArg generates a listener for the class of Arg and the
 * class type T which is the class containing the function that notify will
 * call.
 *
 * Note that the function is passed as a pointer on construction.
 */
template <class T, class Arg>
class ProbeListenerArg : public ProbeListenerArgBase<Arg>
{
  private:
    T *object;
    void (T::* function)(const Arg &);

  public:
    /**
     * @param obj the class of type Tcontaining the method to call on notify.
     * @param name the name of the ProbePoint to add this listener to.
     * @param func a pointer to the function on obj (called on notify).
     */
    ProbeListenerArg(T *obj, std::string name,
        void (T::* func)(const Arg &))
        : ProbeListenerArgBase<Arg>(std::move(name)),
          object(obj),
          function(func)
    {}

    /**
     * @brief called when the ProbePoint calls notify. This is a shim through
     *        to the function passed during construction.
     * @param val the argument value to pass.
     */
    void notify(const Arg &val) override { (object->*function)(val); }
};

/**
 * ProbePointArg generates a point for the class of Arg. As ProbePointArgs talk
 * directly to ProbeListenerArgs of the same type, we can store the vector of
 * ProbeListeners as their Arg type (and not as base type).
 *
 * Methods are provided to addListener, removeListener and notify.
 */
template <typename Arg>
class ProbePointArg : public ProbePoint
{
    /** The attached listeners. */
    std::vector<ProbeListenerArgBase<Arg> *> listeners;

  public:
    ProbePointArg(ProbeManager *manager, std::string name)
        : ProbePoint(manager, name)
    {
    }

    /**
     * Informs whether any listeners are attached to this probe. This can
     * be used to avoid performing costly tasks needed by the probe when
     * nobody is listening.
     *
     * @return Whether this probe has any listener.
     */
    bool hasListeners() const { return listeners.size() > 0; }

    /**
     * @brief adds a ProbeListener to this ProbePoints notify list.
     * @param l the ProbeListener to add to the notify list.
     */
    void
    addListener(ProbeListener *l_base) override
    {
        auto *l = dynamic_cast<ProbeListenerArgBase<Arg> *>(l_base);
        panic_if(!l, "Wrong type of listener: expected %s got %s",
                 typeid(ProbeListenerArgBase<Arg>).name(),
                 typeid(*l_base).name());
        // check listener not already added
        if (std::find(listeners.begin(), listeners.end(), l) ==
            listeners.end()) {
            listeners.push_back(l);
        }
    }

    /**
     * @brief remove a ProbeListener from this ProbePoints notify list.
     * @param l the ProbeListener to remove from the notify list.
     */
    void
    removeListener(ProbeListener *l_base) override
    {
        auto *l = dynamic_cast<ProbeListenerArgBase<Arg> *>(l_base);
        panic_if(!l, "Wrong type of listener expected %s got %s",
                 typeid(ProbeListenerArgBase<Arg>).name(),
                 typeid(*l_base).name());
        listeners.erase(std::remove(listeners.begin(), listeners.end(), l),
                        listeners.end());
    }

    /**
     * @brief called at the ProbePoint call site, passes arg to each listener.
     * @param arg the argument to pass to each listener.
     */
    void
    notify(const Arg &arg)
    {
        for (auto *l : listeners) {
            l->notify(arg);
        }
    }
};


/**
 * ProbeListenerArgFunc generates a listener for the class of Arg and
 * a lambda callback function that is called by the notify.
 *
 * Note that the function is passed as lambda function on construction
 * Example:
 * ProbeListenerArgFunc<MyArg> (myobj->getProbeManager(),
 *                "MyProbePointName", [this](const MyArg &arg)
 *                { my_own_func(arg, xyz...); // do something with arg
 *  }));
 */
template <class Arg>
class ProbeListenerArgFunc : public ProbeListenerArgBase<Arg>
{
  typedef std::function<void(const Arg &)> NotifyFunction;
  private:
    NotifyFunction function;

  public:
    /**
     * @param obj the class of type Tcontaining the method to call on notify.
     * @param pm A probe manager that is not part of the obj
     * @param name the name of the ProbePoint to add this listener to.
     * @param func a pointer to the function on obj (called on notify).
     */
    ProbeListenerArgFunc(const std::string &name, const NotifyFunction &func)
      : ProbeListenerArgBase<Arg>(name),
        function(func)
    {}

    /**
     * @brief called when the ProbePoint calls notify. This is a shim through
     *        to the function passed during construction.
     * @param val the argument value to pass.
     */
    void notify(const Arg &val) override { function(val); }
};


} // namespace gem5

#endif//__SIM_PROBE_PROBE_HH__
