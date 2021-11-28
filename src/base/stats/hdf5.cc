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

#include "base/stats/hdf5.hh"

#include "base/logging.hh"
#include "base/stats/info.hh"
#include "base/trace.hh"
#include "debug/Stats.hh"

namespace gem5
{

/**
 * Check if all strings in a container are empty.
 */
template<typename T>
bool emptyStrings(const T &labels)
{
    for (const auto &s : labels) {
        if (!s.empty())
            return false;
    }
    return true;
}


GEM5_DEPRECATED_NAMESPACE(Stats, statistics);
namespace statistics
{

Hdf5::Hdf5(const std::string &file, unsigned chunking,
           bool desc, bool formulas)
    : fname(file), timeChunk(chunking),
      enableDescriptions(desc), enableFormula(formulas),
      dumpCount(0)
{
    // Tell the library not to print exceptions by default. There are
    // cases where we rely on exceptions to determine if we need to
    // create a node or if we can just open it.
    H5::Exception::dontPrint();
}

Hdf5::~Hdf5()
{
}


void
Hdf5::begin()
{
    h5File = H5::H5File(fname,
                        // Truncate the file if this is the first dump
                        dumpCount > 0 ? H5F_ACC_RDWR : H5F_ACC_TRUNC);
    path.push(h5File.openGroup("/"));
}

void
Hdf5::end()
{
    assert(valid());

    dumpCount++;
}

bool
Hdf5::valid() const
{
    return true;
}


void
Hdf5::beginGroup(const char *name)
{
    auto base = path.top();

    // Try to open an existing stat group corresponding to the
    // name. Create it if it doesn't exist.
    H5::Group group;
    try {
        group = base.openGroup(name);
    } catch (const H5::FileIException& e) {
        group = base.createGroup(name);
    } catch (const H5::GroupIException& e) {
        group = base.createGroup(name);
    }

    path.push(group);
}

void
Hdf5::endGroup()
{
    assert(!path.empty());
    path.pop();
}

void
Hdf5::visit(const ScalarInfo &info)
{
    // Since this stat is a scalar, we need 1-dimensional value in the
    // stat file. The Hdf5::appendStat helper will populate the size
    // of the first dimension (time).
    hsize_t fdims[1] = { 0, };
    double data[1] = { info.result(), };

    appendStat(info, 1, fdims, data);
}

void
Hdf5::visit(const VectorInfo &info)
{
    appendVectorInfo(info);
}

void
Hdf5::visit(const DistInfo &info)
{
    warn_once("HDF5 stat files don't support distributions.\n");
}

void
Hdf5::visit(const VectorDistInfo &info)
{
    warn_once("HDF5 stat files don't support vector distributions.\n");
}

void
Hdf5::visit(const Vector2dInfo &info)
{
    // Request a 3-dimensional stat, the first dimension will be
    // populated by the Hdf5::appendStat() helper. The remaining two
    // dimensions correspond to the stat instance.
    hsize_t fdims[3] = { 0, info.x, info.y };
    H5::DataSet data_set = appendStat(info, 3, fdims, info.cvec.data());

    if (dumpCount == 0) {
        if (!info.subnames.empty() && !emptyStrings(info.subnames))
            addMetaData(data_set, "subnames", info.subnames);

        if (!info.y_subnames.empty() && !emptyStrings(info.y_subnames))
            addMetaData(data_set, "y_subnames", info.y_subnames);

        if (!info.subdescs.empty() && !emptyStrings(info.subdescs))
            addMetaData(data_set, "subdescs", info.subdescs);
    }
}

void
Hdf5::visit(const FormulaInfo &info)
{
    if (!enableFormula)
        return;

    H5::DataSet data_set = appendVectorInfo(info);

    if (dumpCount == 0)
        addMetaData(data_set, "equation", info.str());
}

void
Hdf5::visit(const SparseHistInfo &info)
{
    warn_once("HDF5 stat files don't support sparse histograms.\n");
}

H5::DataSet
Hdf5::appendVectorInfo(const VectorInfo &info)
{
    const VResult &vr(info.result());
    // Request a 2-dimensional stat, the first dimension will be
    // populated by the Hdf5::appendStat() helper. The remaining
    // dimension correspond to the stat instance.
    hsize_t fdims[2] = { 0, vr.size() };
    H5::DataSet data_set = appendStat(info, 2, fdims, vr.data());

    if (dumpCount == 0) {
        if (!info.subnames.empty() && !emptyStrings(info.subnames))
            addMetaData(data_set, "subnames", info.subnames);

        if (!info.subdescs.empty() && !emptyStrings(info.subdescs))
            addMetaData(data_set, "subdescs", info.subdescs);
    }

    return data_set;
}

H5::DataSet
Hdf5::appendStat(const Info &info, int rank, hsize_t *dims, const double *data)
{
    H5::Group group = path.top();
    H5::DataSet data_set;
    H5::DataSpace fspace;

    dims[0] = dumpCount + 1;

    if (dumpCount > 0) {
        // Get the existing stat if we have already dumped this stat
        // before.
        data_set = group.openDataSet(info.name);
        data_set.extend(dims);
        fspace = data_set.getSpace();
    } else {
        // We don't have the stat already, create it.

        H5::DSetCreatPropList props;

        // Setup max dimensions based on the requested file dimensions
        std::vector<hsize_t> max_dims(rank);
        std::copy(dims, dims + rank, max_dims.begin());
        max_dims[0] = H5S_UNLIMITED;

        // Setup chunking
        std::vector<hsize_t> chunk_dims(rank);
        std::copy(dims, dims + rank, chunk_dims.begin());
        chunk_dims[0] = timeChunk;
        props.setChunk(rank, chunk_dims.data());

        // Enable compression
        props.setDeflate(1);

        fspace = H5::DataSpace(rank, dims, max_dims.data());
        try {
            DPRINTF(Stats, "Creating dataset %s in group %s\n",
                info.name, group.getObjnameByIdx(group.getId()));
            data_set = group.createDataSet(info.name,
                H5::PredType::NATIVE_DOUBLE, fspace, props);
        } catch (const H5::Exception &e) {
          std::string err = "Failed creating H5::DataSet " +  info.name + "; ";
          err += e.getDetailMsg() + " in " + e.getFuncName();
          // Rethrow std exception so that it's passed on to the Python world
          throw std::runtime_error(err);
        }

        if (enableDescriptions && !info.desc.empty()) {
            addMetaData(data_set, "description", info.desc);
        }
    }

    // The first dimension is time which isn't included in data.
    dims[0] = 1;
    H5::DataSpace mspace(rank, dims);
    std::vector<hsize_t> foffset(rank, 0);
    foffset[0] = dumpCount;

    fspace.selectHyperslab(H5S_SELECT_SET, dims, foffset.data());
    data_set.write(data, H5::PredType::NATIVE_DOUBLE, mspace, fspace);

    return data_set;
}

void
Hdf5::addMetaData(H5::DataSet &loc, const char *name,
                  const std::vector<const char *> &values)
{
    H5::StrType type(H5::PredType::C_S1, H5T_VARIABLE);
    hsize_t dims[1] = { values.size(), };
    H5::DataSpace space(1, dims);
    H5::Attribute attribute = loc.createAttribute(name, type, space);
    attribute.write(type, values.data());
}

void
Hdf5::addMetaData(H5::DataSet &loc, const char *name,
                  const std::vector<std::string> &values)
{
    std::vector<const char *> cstrs(values.size());
    for (int i = 0; i < values.size(); ++i)
        cstrs[i] = values[i].c_str();

    addMetaData(loc, name, cstrs);
}

void
Hdf5::addMetaData(H5::DataSet &loc, const char *name,
                  const std::string &value)
{
    H5::StrType type(H5::PredType::C_S1, value.length() + 1);
    hsize_t dims[1] = { 1, };
    H5::DataSpace space(1, dims);
    H5::Attribute attribute = loc.createAttribute(name, type, space);
    attribute.write(type, value.c_str());
}

void
Hdf5::addMetaData(H5::DataSet &loc, const char *name, double value)
{
    hsize_t dims[1] = { 1, };
    H5::DataSpace space(1, dims);
    H5::Attribute attribute = loc.createAttribute(
        name, H5::PredType::NATIVE_DOUBLE, space);
    attribute.write(H5::PredType::NATIVE_DOUBLE, &value);
}


std::unique_ptr<Output>
initHDF5(const std::string &filename, unsigned chunking,
         bool desc, bool formulas)
{
    return  std::unique_ptr<Output>(
        new Hdf5(simout.resolve(filename), chunking, desc, formulas));
}

}; // namespace statistics
} // namespace gem5
