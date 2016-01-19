/*
 * Copyright (c) 2013-2015 Advanced Micro Devices, Inc.
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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 * Author: Anthony Gutierrez
 */

#ifndef __HSA_OBJECT_HH__
#define __HSA_OBJECT_HH__

#include <functional>
#include <string>
#include <vector>

class HsaCode;

/* @class HsaObject
 * base loader object for HSA kernels. this class provides
 * the base method definitions for loading, storing, and
 * accessing HSA kernel objects into the simulator.
 */

class HsaObject
{
  public:
    HsaObject(const std::string &fileName);

    static HsaObject* createHsaObject(const std::string &fname);
    static std::vector<std::function<HsaObject*(const std::string&, int,
                                                uint8_t*)>> tryFileFuncs;

    virtual HsaCode* getKernel(const std::string &name) const = 0;
    virtual HsaCode* getKernel(int i) const = 0;
    virtual HsaCode* getFunction(const std::string &name) const = 0;
    virtual int numKernels() const = 0;

    const std::string& name() const { return filename; }

    uint8_t *readonlyData;


  protected:
    const std::string filename;
};

#endif // __HSA_OBJECT_HH__
