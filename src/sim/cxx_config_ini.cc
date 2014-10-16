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

#include "sim/cxx_config_ini.hh"

bool
CxxIniFile::getParam(const std::string &object_name,
    const std::string &param_name,
    std::string &value) const
{
    return iniFile.find(object_name, param_name, value);
}

bool
CxxIniFile::getParamVector(const std::string &object_name,
    const std::string &param_name,
    std::vector<std::string> &values) const
{
    std::string value;
    bool ret = iniFile.find(object_name, param_name, value);

    if (ret) {
        std::vector<std::string> sub_object_names;

        tokenize(values, value, ' ', true);
    }

    return ret;
}

bool
CxxIniFile::getPortPeers(const std::string &object_name,
    const std::string &port_name,
    std::vector<std::string> &peers) const
{
    return getParamVector(object_name, port_name, peers);
}

bool
CxxIniFile::objectExists(const std::string &object) const
{
    return iniFile.sectionExists(object);
}

void
CxxIniFile::getAllObjectNames(std::vector<std::string> &list) const
{
    iniFile.getSectionNames(list);
}

void
CxxIniFile::getObjectChildren(const std::string &object_name,
    std::vector<std::string> &children, bool return_paths) const
{
    if (!getParamVector(object_name, "children", children))
        return;

    if (return_paths && object_name != "root") {
        for (auto i = children.begin(); i != children.end(); ++i)
            *i = object_name + "." + *i;
    }
}

bool
CxxIniFile::load(const std::string &filename)
{
    return iniFile.load(filename);
}
