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

/** @file
 * Device model for an IDE disk
 */

#ifndef __IDE_DISK_HH__
#define __IDE_DISK_HH__

#include "dev/io_device.hh"

class DiskImage;
class IdeController;

/**
 * SCSI Disk device model
 */
class IdeDisk : public SimObject
{
  protected:
    /** The IDE controller for this disk. */
    IdeController *ctrl;
    /** The image that contains the data of this disk. */
    DiskImage *image;

  protected:
    /** The disk delay in milliseconds. */
    int diskDelay;

  public:
    /**
     * Create and initialize this Disk.
     * @param name The name of this disk.
     * @param img The disk image of this disk.
     * @param disk_delay The disk delay in milliseconds
     */
    IdeDisk(const std::string &name, DiskImage *img, int disk_delay);

    /**
     * Delete the data buffer.
     */
    ~IdeDisk();

    /**
     * Set the controller for this device
     * @param c The IDE controller
     */
    void setController(IdeController *c) {
        if (ctrl) panic("Cannot change the controller once set!\n");
        ctrl = c;
    }

    void startIO(uint8_t *cmdBlk, uint8_t *prdPtr) {};

    void dmaStart() {};
    void dmaStop() {};

    /**
     * Serialize this object to the given output stream.
     * @param os The stream to serialize to.
     */
    void serialize(std::ostream &os);

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param cp The checkpoint to use.
     * @param section The section name describing this object.
     */
    void unserialize(Checkpoint *cp, const std::string &section);
};


#endif // __IDE_DISK_HH__
