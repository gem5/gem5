# Copyright 2004-2006 The Regents of The University of Michigan
# Copyright 2010-20013 Advanced Micro Devices, Inc.
# Copyright 2013 Mark D. Hill and David A. Wood
# Copyright 2017-2020 ARM Limited
# Copyright 2021 Google, Inc.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import argparse
import importlib
import os.path
import sys

import importer
from code_formatter import code_formatter

parser = argparse.ArgumentParser()
parser.add_argument("modpath", help="module the simobject belongs to")
parser.add_argument("cxx_config_hh", help="cxx config header file to generate")

args = parser.parse_args()

basename = os.path.basename(args.cxx_config_hh)
sim_object_name = os.path.splitext(basename)[0]

importer.install()
module = importlib.import_module(args.modpath)
sim_object = getattr(module, sim_object_name)

code = code_formatter()

entry_class = "CxxConfigDirectoryEntry_%s" % sim_object_name
param_class = "%sCxxConfigParams" % sim_object_name

code(
    """#include "params/${sim_object_name}.hh"

#include "sim/cxx_config.hh"

namespace gem5
{

class ${param_class} : public CxxConfigParams, public ${sim_object_name}Params
{
  private:
    class DirectoryEntry : public CxxConfigDirectoryEntry
    {
      public:
        DirectoryEntry();

        CxxConfigParams *
        makeParamsObject() const
        {
            return new ${param_class};
        }
    };

    static inline AddToConfigDir dirEntry
        {"${sim_object_name}", new DirectoryEntry};

  public:
    bool setSimObject(const std::string &name, SimObject *simObject);

    bool setSimObjectVector(const std::string &name,
        const std::vector<SimObject *> &simObjects);

    void setName(const std::string &name_);

    const std::string &getName() { return this->name; }

    bool setParam(const std::string &name, const std::string &value,
        const Flags flags);

    bool setParamVector(const std::string &name,
        const std::vector<std::string> &values, const Flags flags);

    bool setPortConnectionCount(const std::string &name, unsigned int count);

    SimObject *simObjectCreate();
};

} // namespace gem5
"""
)

code.write(args.cxx_config_hh)
