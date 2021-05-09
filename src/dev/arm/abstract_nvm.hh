/*
 * Copyright (c) 2013-2015 ARM Limited
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

#ifndef __DEV_ARM_ABSTRACT_NVM_HH__
#define __DEV_ARM_ABSTRACT_NVM_HH__

#include "base/callback.hh"
#include "params/AbstractNVM.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/**
 * This is an interface between the disk interface (which will handle the disk
 * data transactions) and the timing model. The timing model only takes care
 * of calculating the appropriate delay to the disk, and calling back a
 * function when the action has completed. All the other associated actions
 * (such as getting data from A to B) should be taken care of by the disk
 * interface.
 */
class AbstractNVM : public SimObject
{

  public:
    AbstractNVM(const AbstractNVMParams &p): SimObject(p) {};
    virtual ~AbstractNVM() {};

    /**
     * Initialize Memory.
     * This function is used to set the memory device dimensions to the
     * dimensions that it controls. For instance, One can imagine that the
     * memory is one disk, e.g. the /data partition of Android, which means
     * that the data handling part will have an image of /data. On the other
     * hand one might want to set up a Raid like configuration, without
     * wanting to create multiple disk images. In that case one can choose to
     * devide the image over multiple memory devices in any way he wants
     * (i.e. higher layers can implement some division based on logical
     * addresses, or intelligent file system interpretation analysis; to
     * effectively devide the disk over the devices; enabling object oriented
     * storage devices).
     * Moving this function outside of the constructor allows you the
     * flexibility to make this decision once the image is loaded.
     *
     * @param disk_size disksize in sectors; value can be obtained from the
     * disk image
     * @param sector_size size of one sector in bytes; value is defined in
     * disk_image.hh
     */
    virtual void initializeMemory(uint64_t disk_size, uint32_t sector_size) =
        0;

    /**
     * Access functions
     * Access function to simulate a read/write access to the memory. Once
     * the action has completed, the Callback event should be called. Putting
     * a NULL pointer as callback is valid syntax, and should result in the
     * simulation of the access, but with no callback to the higher layer.
     * This may be used to occupy the device, such that next actions will be
     * delayed. The read/write function will schedule the incoming requests
     * on a first come first serve basis.
     *
     * @param address The logical address to a location in the Non-volatile
     * memory.
     * @param amount The amount of data transfered from the NVM in bytes
     * @param event A pointer to a callback function that will perform the
     * actions taken by the disk controller on successfull completion of the
     * data transfer between the disk and the disk controller.
     */
    virtual void readMemory(uint64_t address, uint32_t amount,
                            const std::function<void()> &event) = 0;
    virtual void writeMemory(uint64_t address, uint32_t amount,
                             const std::function<void()> &event) = 0;
};

} // namespace gem5

#endif //__DEV_ARM_ABSTRACT_NVM_HH__
