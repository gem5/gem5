/*
 * Copyright 2019 Google, Inc.
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

#ifndef __MEM_BACKDOOR_HH__
#define __MEM_BACKDOOR_HH__

#include <cstdint>
#include <functional>
#include <memory>

#include "base/addr_range.hh"
#include "base/callback.hh"

class MemBackdoor
{
  public:
    // Callbacks from this back door are set up using a callable which accepts
    // a const reference to this back door as their only parameter.
    typedef std::function<void(const MemBackdoor &backdoor)> CbFunction;

  private:
    // This wrapper class holds the callables described above so that they
    // can be stored in a generic CallbackQueue.
    class Callback : public ::Callback
    {
      public:
        Callback(MemBackdoor &bd, CbFunction cb) :
            _backdoor(bd), cbFunction(cb)
        {}

        void process() override { cbFunction(_backdoor); }
        // It looks like this is only called when the CallbackQueue is
        // destroyed and this Callback is currently in the queue.
        void autoDestruct() override { delete this; }

        MemBackdoor &backdoor() { return _backdoor; }

      private:
        MemBackdoor &_backdoor;
        CbFunction cbFunction;
    };

  public:
    enum Flags{
        // How data is allowed to be accessed through this backdoor.
        NoAccess = 0x0,
        Readable = 0x1,
        Writeable = 0x2
    };

    // The range in the guest address space covered by this back door.
    const AddrRange &range() const { return _range; }
    void range(const AddrRange &r) { _range = r; }

    // A pointer to the data accessible through this back door.
    uint8_t *ptr() const { return _ptr; }
    void ptr(uint8_t *p) { _ptr = p; }

    /*
     * Helper functions to make it easier to set/check particular flags.
     */

    bool readable() const { return _flags & Readable; }
    void
    readable(bool r)
    {
        if (r)
            _flags = (Flags)(_flags | Readable);
        else
            _flags = (Flags)(_flags & ~Readable);
    }

    bool writeable() const { return _flags & Writeable; }
    void
    writeable(bool w)
    {
        if (w)
            _flags = (Flags)(_flags | Writeable);
        else
            _flags = (Flags)(_flags & ~Writeable);
    }

    Flags flags() const { return _flags; }
    void flags(Flags f) { _flags = f; }

    MemBackdoor(AddrRange r, uint8_t *p, Flags flags) :
        invalidationCallbacks(new CallbackQueue),
        _range(r), _ptr(p), _flags(flags)
    {}

    MemBackdoor() : MemBackdoor(AddrRange(), nullptr, NoAccess)
    {}

    // Set up a callable to be called when this back door is invalidated. This
    // lets holders update their bookkeeping to remove any references to it,
    // and/or to propogate that invalidation to other interested parties.
    void
    addInvalidationCallback(CbFunction func)
    {
        auto *cb = new MemBackdoor::Callback(*this, func);
        assert(cb);
        invalidationCallbacks->add(cb);
    }

    // Notify and clear invalidation callbacks when the data in the backdoor
    // structure is no longer valid/current. The backdoor might then be
    // updated or even deleted without having to worry about stale data being
    // used.
    void
    invalidate()
    {
        invalidationCallbacks->process();
        // Delete and recreate the callback queue to ensure the callback
        // objects are deleted.
        invalidationCallbacks.reset(new CallbackQueue());
    }

  private:
    std::unique_ptr<CallbackQueue> invalidationCallbacks;

    AddrRange _range;
    uint8_t *_ptr;
    Flags _flags;
};

typedef MemBackdoor *MemBackdoorPtr;

#endif  //__MEM_BACKDOOR_HH__
