/*
 * Copyright 2018 Google, Inc.
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

#ifndef __SYSTEMC_UTILS_VCD_HH__
#define __SYSTEMC_UTILS_VCD_HH__

#include <map>

#include "base/types.hh"
#include "systemc/utils/tracefile.hh"

namespace sc_gem5
{

class VcdTraceValBase;

class VcdTraceScope
{
  private:
    std::vector<std::pair<std::string, VcdTraceValBase *>> values;
    std::map<std::string, VcdTraceScope *> scopes;

  public:
    void addValue(const std::string &name, VcdTraceValBase *value);
    void output(const std::string &name, std::ostream &os);
};

class VcdTraceFile : public TraceFile
{
  private:
    gem5::Tick lastPrintedTime;
    uint64_t deltasAtNow;

    static const int NextNameChars = 5;
    char _nextName[NextNameChars + 1];
    std::string nextSignalName();

    bool initialized;
    void initialize();

    std::vector<VcdTraceValBase *> traceVals;
    VcdTraceScope topScope;

  public:
    VcdTraceFile(const std::string &name) :
        TraceFile(name + ".vcd"), lastPrintedTime(0), deltasAtNow(0),
        initialized(false)
    {
        _nextName[NextNameChars] = '\0';
        for (int i = 0; i < NextNameChars; i++)
            _nextName[i] = 'a';
    }
    ~VcdTraceFile();

    void trace(bool delta) override;

    template<typename TV>
    void
    addNewTraceVal(const typename TV::TracedType *v, const std::string &name,
                   int width=1)
    {
        VcdTraceValBase *tv = new TV(v, nextSignalName(), width);
        traceVals.push_back(tv);
        topScope.addValue(name, tv);
    }

    void addTraceVal(const bool *v, const std::string &name) override;
    void addTraceVal(const float *v, const std::string &name) override;
    void addTraceVal(const double *v, const std::string &name) override;

    void addTraceVal(const sc_dt::sc_logic *v,
                     const std::string &name) override;
    void addTraceVal(const sc_dt::sc_int_base *v,
                     const std::string &name) override;
    void addTraceVal(const sc_dt::sc_uint_base *v,
                     const std::string &name) override;
    void addTraceVal(const sc_dt::sc_signed *v,
                     const std::string &name) override;
    void addTraceVal(const sc_dt::sc_unsigned *v,
                     const std::string &name) override;
    void addTraceVal(const sc_dt::sc_bv_base *v,
                     const std::string &name) override;
    void addTraceVal(const sc_dt::sc_lv_base *v,
                     const std::string &name) override;
    void addTraceVal(const sc_dt::sc_fxval *v,
                     const std::string &name) override;
    void addTraceVal(const sc_dt::sc_fxval_fast *v,
                     const std::string &name) override;
    void addTraceVal(const sc_dt::sc_fxnum *v,
                     const std::string &name) override;
    void addTraceVal(const sc_dt::sc_fxnum_fast *v,
                     const std::string &name) override;

    void addTraceVal(const sc_core::sc_event *v,
                     const std::string &name) override;
    void addTraceVal(const sc_core::sc_time *v,
                     const std::string &name) override;

    void addTraceVal(const unsigned char *v,
                     const std::string &name, int width) override;
    void addTraceVal(const char *v, const std::string &name,
                     int width) override;
    void addTraceVal(const unsigned short *v,
                     const std::string &name, int width) override;
    void addTraceVal(const short *v, const std::string &name,
                     int width) override;
    void addTraceVal(const unsigned int *v,
                     const std::string &name, int width) override;
    void addTraceVal(const int *v, const std::string &name,
                     int width) override;
    void addTraceVal(const unsigned long *v,
                     const std::string &name, int width) override;
    void addTraceVal(const long *v, const std::string &name,
                     int width) override;

    void addTraceVal(const sc_dt::int64 *v,
                     const std::string &name, int width) override;
    void addTraceVal(const sc_dt::uint64 *v,
                     const std::string &name, int width) override;

    void addTraceVal(const unsigned int *, const std::string &name,
                     const char **literals) override;

    void writeComment(const std::string &comment) override;
};

} // namespace sc_gem5

#endif // __SYSTEMC_UTILS_VCD_HH__
