/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __CALLBACK_HH__
#define __CALLBACK_HH__

#include <list>

/**
 * Generic callback class.  This base class provides a virutal process
 * function that gets called when the callback queue is processed.
 */
class Callback
{
  public:
    /**
     * virtualize the destructor to make sure that the correct one
     * gets called.
     */
    virtual ~Callback() {}

    /**
     * virtual process function that is invoked when the callback
     * queue is executed.
     */
    virtual void process() = 0;
};

class CallbackQueue
{
  protected:
    /**
     * Simple typedef for the data structure that stores all of the
     * callbacks.
     */
    typedef std::list<Callback *> queue;

    /**
     * List of all callbacks.  To be called in fifo order.
     */
    queue callbacks;

  public:
    /**
     * Add a callback to the end of the queue
     * @param callback the callback to be added to the queue
     */
    void add(Callback *callback)
    {
        callbacks.push_back(callback);
    }

    /**
     * Find out if there are any callbacks in the queue
     */
    bool empty() const { return callbacks.empty(); }

    /**
     * process all callbacks
     */
    void process()
    {
        queue::iterator i = callbacks.begin();
        queue::iterator end = callbacks.end();

        while (i != end) {
            (*i)->process();
            ++i;
        }
    }

    /**
     * clear the callback queue
     */
    void clear()
    {
        callbacks.clear();
    }
};

template <class T, void (T::* F)()>
class MakeCallback : public Callback
{
  private:
    T *object;

  public:
    MakeCallback(T *o)
    : object(o)
    { }
    void process() { (object->*F)(); }
};

#endif // __CALLBACK_HH__
