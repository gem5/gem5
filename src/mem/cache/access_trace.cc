#include "mem/cache/access_trace.hh"

#include "base/cprintf.hh"
#include "base/statistics.hh"
#include "sim/core.hh"

namespace gem5
{

AccessTrace::AccessTrace(const std::string &cache_name)
    : name(cache_name)
{
    statistics::registerResetCallback([this] { start(); });
    registerExitCallback([this] { close(); });
}

void
AccessTrace::start()
{
    if (out)
        return;
    out = simout.create(name + ".access_trace.txt.gz", false, false);
    ccprintf(*out->stream(), "# tick set addr type hit (H/M) | "
             "tick set addr FILL/WBFILL/EVICT\n");
}

void
AccessTrace::close()
{
    if (out) {
        out->stream()->flush();
        simout.close(out);
        out = nullptr;
    }
}

void
AccessTrace::access(Tick t, unsigned set, Addr addr, const char *type,
                    bool hit)
{
    ccprintf(*out->stream(), "%d %d %#x %s %s\n", t, set, addr, type,
             hit ? "H" : "M");
}

void
AccessTrace::event(Tick t, unsigned set, Addr addr, const char *type)
{
    ccprintf(*out->stream(), "%d %d %#x %s\n", t, set, addr, type);
}

} // namespace gem5
