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

#include <sys/ioctl.h>
#include <sys/types.h>

#if defined(__sun__) || defined(__SUNPRO_CC)
#include <sys/file.h>

#endif

#include "base/pollevent.hh"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstring>

#include "base/logging.hh"
#include "base/types.hh"
#include "sim/async.hh"
#include "sim/eventq.hh"
#include "sim/serialize.hh"

namespace gem5
{

PollQueue pollQueue;

/////////////////////////////////////////////////////
//
PollEvent::PollEvent(int _fd, int _events)
    : queue(NULL), enabled(true)
{
    pfd.fd = _fd;
    pfd.events = _events;
    pfd.revents = 0;
}

PollEvent::~PollEvent()
{
    if (queue)
        queue->remove(this);
}

void
PollEvent::disable()
{
    if (!enabled) return;
    enabled = false;

    if (queue)
        queue->copy();
}

void
PollEvent::enable()
{
    if (enabled) return;
    enabled = true;

    if (queue)
        queue->copy();
}

void
PollEvent::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(pfd.fd);
    SERIALIZE_SCALAR(pfd.events);
    SERIALIZE_SCALAR(enabled);
}

void
PollEvent::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(pfd.fd);
    UNSERIALIZE_SCALAR(pfd.events);
    UNSERIALIZE_SCALAR(enabled);
}

/////////////////////////////////////////////////////
//
PollQueue::PollQueue()
    : poll_fds(NULL), max_size(0), num_fds(0)
{ }

PollQueue::~PollQueue()
{
    for (int i = 0; i < num_fds; i++)
        setupAsyncIO(poll_fds[0].fd, false);

    delete [] poll_fds;
}

void
PollQueue::copy()
{
    eventvec_t::iterator i = events.begin();
    eventvec_t::iterator end = events.end();

    num_fds = 0;

    while (i < end) {
        if ((*i)->enabled)
            poll_fds[num_fds++] = (*i)->pfd;
        ++i;
    }
}

void
PollQueue::remove(PollEvent *event)
{
    eventvec_t::iterator i = events.begin();
    eventvec_t::iterator end = events.end();

    while (i < end) {
        if (*i == event) {
           events.erase(i);
           copy();
           event->queue = NULL;
           return;
        }

        ++i;
    }

    panic("Event does not exist.  Cannot remove.");
}

void
PollQueue::schedule(PollEvent *event)
{
    if (event->queue)
        panic("Event already scheduled!");

    event->queue = this;
    events.push_back(event);
    setupAsyncIO(event->pfd.fd, true);

    // if we ran out of space in the fd array, double the capacity
    // if this is the first time that we've scheduled an event, create
    // the array with an initial size of 16
    if (++num_fds > max_size) {
        if (max_size > 0) {
            delete [] poll_fds;
            max_size *= 2;
        } else {
            max_size = 16;
        }

        poll_fds = new pollfd[max_size];
    }

    copy();
}

void
PollQueue::service()
{
    int ret = poll(poll_fds, num_fds, 0);

    if (ret <= 0)
        return;

    for (int i = 0; i < num_fds; i++) {
        int revents = poll_fds[i].revents;
        if (revents) {
            events[i]->process(revents);
            if (--ret <= 0)
                break;
        }
    }
}

template <class ArgT>
static int fcntlHelper(int fd, int cmd, ArgT arg)
{
    int retval = fcntl(fd, cmd, arg);
    if (retval == -1) {
        char *errstr = strerror(errno);
        panic("fcntl(%d, %d, %s): \"%s\" when setting up async IO.\n",
              errstr, fd, cmd, arg);
    }
    return retval;
}

static int fcntlHelper(int fd, int cmd)
{
    int retval = fcntl(fd, cmd);
    if (retval == -1) {
        char *errstr = strerror(errno);
        panic("fcntl(%d, %d): \"%s\" when setting up async IO.\n",
              errstr, fd, cmd);
    }
    return retval;
}

void
PollQueue::setupAsyncIO(int fd, bool set)
{
    int flags = fcntlHelper(fd, F_GETFL);

    if (set)
        flags |= FASYNC;
    else
        flags &= ~(FASYNC);

    if (set)
        fcntlHelper(fd, F_SETOWN, getpid());

    fcntlHelper(fd, F_SETFL, flags);

    // The file descriptor might already have events pending. We won't
    // see them if they occurred before we set the FASYNC
    // flag. Simulate a SIGIO to ensure that the FD will be polled in
    // next iteration of the simulation loop. We could just poll it,
    // but this is much simpler.
    if (set) {
        async_event = true;
        async_io = true;
        /* Wake up some event queue to handle event */
        getEventQueue(0)->wakeup();
    }
}

} // namespace gem5
