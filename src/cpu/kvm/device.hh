/*
 * Copyright (c) 2015 ARM Limited
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

#ifndef __CPU_KVM_DEVICE_HH__
#define __CPU_KVM_DEVICE_HH__

#include <cstdint>

/**
 * KVM device wrapper
 *
 * This is a wrapper around a device emulated by KVM. Such devices can
 * be created using KvmVM::createDevice() API. They are typically used
 * for in-kernel interrupt controllers and similar devices.
 *
 * Each device has a device-specific set of attributes which are
 * mapped into groups. The available attributes are described in the
 * device's documentation in the kernel source tree
 * (Documentation/virtual/kvm/devices/). Each attribute can be
 * accessed with the getAttr() and setAttr() access methods. The
 * presence of an attribute can be queried using the hasAttr() method.
 */
class KvmDevice
{
  public:
    KvmDevice(int fd);
    virtual ~KvmDevice();

  public:
    /**
     * Get the value of an attribute
     *
     * See KVM's documentation, and specifically the device-specific
     * documentation, for available attributes and attribute groups.
     *
     * @param group Attribute group
     * @param attr Attribute ID within group
     * @return Attribute value
     */
    template<typename T>
    T getAttr(uint32_t group, uint64_t attr) const {
        T data;
        getAttrPtr(group, attr, &data);
        return data;
    }

    /**
     * Get the value of an attribute
     *
     * See KVM's documentation, and specifically the device-specific
     * documentation, for available attributes and attribute groups.
     *
     * @param group Attribute group
     * @param attr Attribute ID within group
     * @return Attribute value
     */
    template<typename T>
    void setAttr(uint32_t group, uint64_t attr, const T &data) const {
        setAttrPtr(group, attr, &data);
    }

    void getAttrPtr(uint32_t group, uint64_t attr, void *data) const;
    void setAttrPtr(uint32_t group, uint64_t attr, const void *data) const;

    /**
     * Check if a device attribute is valid
     *
     * See KVM's documentation, and specifically the device-specific
     * documentation, for available attributes and attribute groups.
     *
     * @param group Attribute group
     * @param attr Attribute ID within group
     * @return true if attribute is valid, false otherwise.
     */
    bool hasAttr(uint32_t group, uint64_t attr) const;

  protected:
    int ioctl(int request, long p1) const;
    int ioctl(int request, void *p1) const {
        return ioctl(request, (long)p1);
    }
    int ioctl(int request) const {
        return ioctl(request, 0L);
    }

  private:
    int fd;
};

#endif // __CPU_KVM_DEVICE_HH__
