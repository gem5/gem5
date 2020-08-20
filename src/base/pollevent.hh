/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#ifndef __POLLEVENT_H__
#define __POLLEVENT_H__

#include <poll.h>

#include <vector>

#include "sim/core.hh"

class Checkpoint;
class PollQueue;

class PollEvent : public Serializable
{
  private:
    friend class PollQueue;

  protected:
    pollfd pfd;
    PollQueue *queue;
    bool enabled;

  public:
    /**
     * @ingroup api_poll_event
     */
    PollEvent(int fd, int event);
    virtual ~PollEvent();

    /**
     * @ingroup api_poll_event
     * @{
     */
    void disable();
    void enable();
    virtual void process(int revent) = 0;
    /** @} */ // end of api_poll_event

    /**
     * @ingroup api_poll_event
     */
    bool queued() { return queue != 0; }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

class PollQueue
{
  private:
    typedef std::vector<PollEvent *> eventvec_t;
    eventvec_t events;

    pollfd *poll_fds;
    int max_size;
    int num_fds;

  public:
    /**
     * @ingroup api_poll_queue
     */
    PollQueue();
    ~PollQueue();

    /**
     * @ingroup api_poll_queue
     * @{
     */
    void copy();
    void remove(PollEvent *event);
    void schedule(PollEvent *event);
    void service();
    /** @} */ // end of api_poll_queue


  public:
    static void setupAsyncIO(int fd, bool set);
};

/**
 * @ingroup api_poll_queue
 */
extern PollQueue pollQueue;

#endif // __POLLEVENT_H__
