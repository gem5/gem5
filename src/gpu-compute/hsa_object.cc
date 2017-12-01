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

#include "gpu-compute/hsa_object.hh"

#include <cassert>
#include <fstream>

#include "base/logging.hh"

HsaObject::HsaObject(const std::string &fname)
    : readonlyData(nullptr), filename(fname)
{
}

HsaObject*
HsaObject::createHsaObject(const std::string &fname)
{
    HsaObject *hsaObj = nullptr;
    uint8_t *file_data = nullptr;
    int file_length = 0;

    std::ifstream code_file(fname, std::ifstream::ate | std::ifstream::in |
                            std::ifstream::binary);

    assert(code_file.is_open());
    assert(code_file.good());

    file_length = code_file.tellg();
    code_file.seekg(0, code_file.beg);
    file_data = new uint8_t[file_length];
    code_file.read((char*)file_data, file_length);
    code_file.close();

    for (const auto &tryFile : tryFileFuncs) {
        if ((hsaObj = tryFile(fname, file_length, file_data))) {
            return hsaObj;
        }
    }

    delete[] file_data;
    fatal("Unknown HSA object type for file: %s.\n", fname);

    return nullptr;
}
