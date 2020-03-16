/*
 * Copyright (c) 2019-2020 Arm Limited
 * All rights reserved.
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __BASE_STATS_TEXT_HH__
#define __BASE_STATS_TEXT_HH__

#include <iosfwd>
#include <stack>
#include <string>

#include "base/stats/output.hh"
#include "base/stats/types.hh"
#include "base/output.hh"

namespace Stats {

class Text : public Output
{
  protected:
    bool mystream;
    std::ostream *stream;

    // Object/group path
    std::stack<std::string> path;

  protected:
    bool noOutput(const Info &info);

  public:
    bool descriptions;
    bool spaces;

  public:
    Text();
    Text(std::ostream &stream);
    Text(const std::string &file);
    ~Text();

    void open(std::ostream &stream);
    void open(const std::string &file);
    std::string statName(const std::string &name) const;

    // Implement Visit
    void visit(const ScalarInfo &info) override;
    void visit(const VectorInfo &info) override;
    void visit(const DistInfo &info) override;
    void visit(const VectorDistInfo &info) override;
    void visit(const Vector2dInfo &info) override;
    void visit(const FormulaInfo &info) override;
    void visit(const SparseHistInfo &info) override;

    // Group handling
    void beginGroup(const char *name) override;
    void endGroup() override;

    // Implement Output
    bool valid() const override;
    void begin() override;
    void end() override;
};

std::string ValueToString(Result value, int precision);

Output *initText(const std::string &filename, bool desc, bool spaces);

} // namespace Stats

#endif // __BASE_STATS_TEXT_HH__
