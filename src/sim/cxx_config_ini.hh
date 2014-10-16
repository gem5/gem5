/*
 * Copyright (c) 2014 ARM Limited
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
 *
 * Authors: Andrew Bardsley
 */

/**
 * @file
 *
 *  .ini file reading wrapper for use with CxxConfigManager
 */

#ifndef __SIM_CXX_CONFIG_INI_HH__
#define __SIM_CXX_CONFIG_INI_HH__

#include "base/inifile.hh"
#include "base/str.hh"
#include "sim/cxx_config.hh"

/** CxxConfigManager interface for using .ini files */
class CxxIniFile : public CxxConfigFileBase
{
  protected:
    IniFile iniFile;

  public:
    CxxIniFile() { }

    /* Most of these functions work by mapping 'object' onto 'section' */

    bool getParam(const std::string &object_name,
        const std::string &param_name,
        std::string &value) const;

    bool getParamVector(const std::string &object_name,
        const std::string &param_name,
        std::vector<std::string> &values) const;

    bool getPortPeers(const std::string &object_name,
        const std::string &port_name,
        std::vector<std::string> &peers) const;

    bool objectExists(const std::string &object_name) const;

    void getAllObjectNames(std::vector<std::string> &list) const;

    void getObjectChildren(const std::string &object_name,
        std::vector<std::string> &children,
        bool return_paths = false) const;

    bool load(const std::string &filename);
};

#endif // __SIM_CXX_CONFIG_INI_HH__
