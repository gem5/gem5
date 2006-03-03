#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/remote_gdb.hh"
#include "cpu/exec_context.hh"
#include "kern/kernel_stats.hh"
#include "mem/functional/memory_control.hh"
#include "mem/functional/physical.hh"
#include "targetarch/vtophys.hh"
#include "sim/builder.hh"
#include "arch/isa_traits.hh"
#include "sim/byteswap.hh"
#include "sim/system.hh"
#include "base/trace.hh"

using namespace std;
using namespace TheISA;

vector<System *> System::systemList;

int System::numSystemsRunning = 0;

System::System(Params *p)
    : SimObject(p->name), memctrl(p->memctrl), physmem(p->physmem),
      init_param(p->init_param), numcpus(0), _params(p)
{
    // add self to global system list
    systemList.push_back(this);

    kernelSymtab = new SymbolTable;
    debugSymbolTable = new SymbolTable;

    /**
     * Load the kernel code into memory
     */
    // Load kernel code
    kernel = createObjectFile(params()->kernel_path);
    if (kernel == NULL)
        fatal("Could not load kernel file %s", params()->kernel_path);

    // Load program sections into memory
    kernel->loadSections(physmem, true);

    // setup entry points
    kernelStart = kernel->textBase();
    kernelEnd = kernel->bssBase() + kernel->bssSize();
    kernelEntry = kernel->entryPoint();

    // load symbols
    if (!kernel->loadGlobalSymbols(kernelSymtab))
        panic("could not load kernel symbols\n");

    if (!kernel->loadLocalSymbols(kernelSymtab))
        panic("could not load kernel local symbols\n");

    if (!kernel->loadGlobalSymbols(debugSymbolTable))
        panic("could not load kernel symbols\n");

    if (!kernel->loadLocalSymbols(debugSymbolTable))
        panic("could not load kernel local symbols\n");

    DPRINTF(Loader, "Kernel start = %#x\n", kernelStart);
    DPRINTF(Loader, "Kernel end   = %#x\n", kernelEnd);
    DPRINTF(Loader, "Kernel entry = %#x\n", kernelEntry);
    DPRINTF(Loader, "Kernel loaded...\n");

    // increment the number of running systms
    numSystemsRunning++;

    kernelBinning = new Kernel::Binning(this);
}

System::~System()
{
    delete kernelSymtab;
    delete kernel;

    delete kernelBinning;
}




int rgdb_wait = -1;

int
System::registerExecContext(ExecContext *xc, int id)
{
    if (id == -1) {
        for (id = 0; id < execContexts.size(); id++) {
            if (!execContexts[id])
                break;
        }
    }

    if (execContexts.size() <= id)
        execContexts.resize(id + 1);

    if (execContexts[id])
        panic("Cannot have two CPUs with the same id (%d)\n", id);

    execContexts[id] = xc;
    numcpus++;

    RemoteGDB *rgdb = new RemoteGDB(this, xc);
    GDBListener *gdbl = new GDBListener(rgdb, 7000 + id);
    gdbl->listen();
    /**
     * Uncommenting this line waits for a remote debugger to connect
     * to the simulator before continuing.
     */
    if (rgdb_wait != -1 && rgdb_wait == id)
        gdbl->accept();

    if (remoteGDB.size() <= id) {
        remoteGDB.resize(id + 1);
    }

    remoteGDB[id] = rgdb;

    return id;
}

void
System::startup()
{
    int i;
    for (i = 0; i < execContexts.size(); i++)
        execContexts[i]->activate(0);
}

void
System::replaceExecContext(ExecContext *xc, int id)
{
    if (id >= execContexts.size()) {
        panic("replaceExecContext: bad id, %d >= %d\n",
              id, execContexts.size());
    }

    execContexts[id] = xc;
    remoteGDB[id]->replaceExecContext(xc);
}

void
System::regStats()
{
    kernelBinning->regStats(name() + ".kern");
}

void
System::serialize(ostream &os)
{
    kernelBinning->serialize(os);

    kernelSymtab->serialize("kernel_symtab", os);
}


void
System::unserialize(Checkpoint *cp, const string &section)
{
    kernelBinning->unserialize(cp, section);

    kernelSymtab->unserialize("kernel_symtab", cp, section);
}

void
System::printSystems()
{
    vector<System *>::iterator i = systemList.begin();
    vector<System *>::iterator end = systemList.end();
    for (; i != end; ++i) {
        System *sys = *i;
        cerr << "System " << sys->name() << ": " << hex << sys << endl;
    }
}

extern "C"
void
printSystems()
{
    System::printSystems();
}

DEFINE_SIM_OBJECT_CLASS_NAME("System", System)

