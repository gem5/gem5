/*
 * Copyright (c) 2016-2019 Arm Limited
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

#ifndef __BASE_STATS_HDF5_HH__
#define __BASE_STATS_HDF5_HH__

#include <H5Cpp.h>

#include <memory>
#include <stack>
#include <string>
#include <vector>

#include "base/compiler.hh"
#include "base/output.hh"
#include "base/stats/output.hh"
#include "base/stats/types.hh"

namespace gem5
{

namespace statistics
{

class Hdf5 : public Output
{
  public:
    Hdf5(const std::string &file, unsigned chunking, bool desc, bool formulas);

    ~Hdf5();

    Hdf5() = delete;
    Hdf5(const Hdf5 &other) = delete;

  public: // Output interface
    void begin() override;
    void end() override;
    bool valid() const override;

    void beginGroup(const char *name) override;
    void endGroup() override;

    void visit(const ScalarInfo &info) override;
    void visit(const VectorInfo &info) override;
    void visit(const DistInfo &info) override;
    void visit(const VectorDistInfo &info) override;
    void visit(const Vector2dInfo &info) override;
    void visit(const FormulaInfo &info) override;
    void visit(const SparseHistInfo &info) override;

  protected:
    /**
     * Helper function to append vector stats and set their metadata.
     */
    H5::DataSet appendVectorInfo(const VectorInfo &info);

    /**
     * Helper function to append an n-dimensional double stat to the
     * file.
     *
     * This helper function assumes that all stats include a time
     * component. I.e., a Stat::Scalar is a 1-dimensional stat.
     *
     * @param info Stat info structure.
     * @param rank Stat dimensionality (including time).
     * @param dims Size of each of the dimensions.
     */
    H5::DataSet appendStat(const Info &info, int rank, hsize_t *dims,
                           const double *data);

    /**
     * Helper function to add a string vector attribute to a stat.
     *
     * @param loc Parent location in the file.
     * @param name Attribute name.
     * @param values Attribute value.
     */
    void addMetaData(H5::DataSet &loc, const char *name,
                     const std::vector<const char *> &values);

    /**
     * Helper function to add a string vector attribute to a stat.
     *
     * @param loc Parent location in the file.
     * @param name Attribute name.
     * @param values Attribute value.
     */
    void addMetaData(H5::DataSet &loc, const char *name,
                     const std::vector<std::string> &values);

    /**
     * Helper function to add a string attribute to a stat.
     *
     * @param loc Parent location in the file.
     * @param name Attribute name.
     * @param value Attribute value.
     */
    void addMetaData(H5::DataSet &loc, const char *name,
                     const std::string &value);

    /**
     * Helper function to add a double attribute to a stat.
     *
     * @param loc Parent location in the file.
     * @param name Attribute name.
     * @param value Attribute value.
     */
    void addMetaData(H5::DataSet &loc, const char *name, double value);

  protected:
    const std::string fname;
    const hsize_t timeChunk;
    const bool enableDescriptions;
    const bool enableFormula;

    std::stack<H5::Group> path;

    unsigned dumpCount;
    H5::H5File h5File;
};

std::unique_ptr<Output> initHDF5(
    const std::string &filename,unsigned chunking = 10,
    bool desc = true, bool formulas = true);

} // namespace statistics
} // namespace gem5

#endif // __BASE_STATS_HDF5_HH__
