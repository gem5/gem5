/*
 * Copyright (c) 2015-2018 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Anthony Gutierrez
 *          Eric van Tassell
 */

/**
 * @file
 * An HSADriver is an emulated driver that controls an HSA agent,
 * or more simply put, an HSA device. An HSA device is a device
 * that has an associated HSA packet processor.
 *
 * In the base HSADriver class the open() method is implemented, as
 * well as the mmap() call, which maps the HSA packet processor's
 * doorbells. Drivers for other HSA devices should derive from this
 * class and implement the necessary methods; typically this is an
 * ioctl() method that satisfies the ioctl requests needed to manage
 * and control the device.
 */

#ifndef __DEV_HSA_HSA_DRIVER_HH__
#define __DEV_HSA_HSA_DRIVER_HH__

#include "base/types.hh"
#include "sim/emul_driver.hh"

struct HSADriverParams;
class HSADevice;
class PortProxy;
class ThreadContext;

class HSADriver : public EmulatedDriver
{
  public:
    HSADriver(HSADriverParams *p);

    int open(ThreadContext *tc, int mode, int flags);
    Addr mmap(ThreadContext *tc, Addr start, uint64_t length,
              int prot, int tgtFlags, int tgtFd, int offset);
  protected:
    /**
     * HSA agent (device) that is controled by this driver.
     */
    HSADevice *device;
    uint32_t queueId;

    void allocateQueue(ThreadContext *tc, Addr ioc_buf);
};

#endif // __DEV_HSA_HSA_DRIVER_HH__
