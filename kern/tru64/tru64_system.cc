/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include "base/loader/aout_object.hh"
#include "base/loader/ecoff_object.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/remote_gdb.hh"
#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "kern/tru64/tru64_events.hh"
#include "kern/tru64/tru64_system.hh"
#include "mem/functional_mem/memory_control.hh"
#include "mem/functional_mem/physical_memory.hh"
#include "sim/builder.hh"
#include "targetarch/isa_traits.hh"
#include "targetarch/vtophys.hh"

//un-comment this to see the state of call stack when it changes.
//#define SW_DEBUG

using namespace std;

Tru64System::Tru64System(const string _name, const uint64_t _init_param,
                         MemoryController *_memCtrl, PhysicalMemory *_physmem,
                         const string &kernel_path, const string &console_path,
                         const string &palcode, const string &boot_osflags,
                         const bool _bin)
     : System(_name, _init_param, _memCtrl, _physmem, _bin), bin(_bin)
{
    kernelSymtab = new SymbolTable;
    consoleSymtab = new SymbolTable;

    ObjectFile *kernel = createObjectFile(kernel_path);
    if (kernel == NULL)
        fatal("Could not load kernel file %s", kernel_path);

    ObjectFile *console = createObjectFile(console_path);
    if (console == NULL)
        fatal("Could not load console file %s", console_path);

    if (!kernel->loadGlobalSymbols(kernelSymtab))
        panic("could not load kernel symbols\n");

    if (!console->loadGlobalSymbols(consoleSymtab))
        panic("could not load console symbols\n");

    // Load pal file
    ObjectFile *pal = createObjectFile(palcode);
    if (pal == NULL)
        fatal("Could not load PALcode file %s", palcode);
    pal->loadSections(physmem, true);

    // Load console file
    console->loadSections(physmem, true);

    // Load kernel file
    kernel->loadSections(physmem, true);
    kernelStart = kernel->textBase();
    kernelEnd = kernel->bssBase() + kernel->bssSize();
    kernelEntry = kernel->entryPoint();

    DPRINTF(Loader, "Kernel start = %#x\n"
            "Kernel end   = %#x\n"
            "Kernel entry = %#x\n",
            kernelStart, kernelEnd, kernelEntry);

    DPRINTF(Loader, "Kernel loaded...\n");

#ifdef FS_MEASURE
    //INSTRUMENTATION CODEGEN BEGIN ONE
    if (bin == true) {
        esIntrBin = new Statistics::MainBin(name() + " es_intr");
        fnBins.insert(make_pair("es_intr", esIntrBin));

        esRxeofBin = new Statistics::MainBin(name() + " es_rxeof");
        fnBins.insert(make_pair("es_rxeof", esRxeofBin));

        esNewbufBin = new Statistics::MainBin(name() + " es_newbuf");
        fnBins.insert(make_pair("es_newbuf", esNewbufBin));

        esDmaLoadBin = new Statistics::MainBin(name() + " es_dma_load");
        fnBins.insert(make_pair("es_dma_load", esDmaLoadBin));

        dmaMapLoadBin = new Statistics::MainBin(name() + " dma_map_load");
        fnBins.insert(make_pair("dma_map_load", dmaMapLoadBin));

        etherInputBin = new Statistics::MainBin(name() + " ether_input");
        fnBins.insert(make_pair("ether_input", etherInputBin));

        netisrInputBin = new Statistics::MainBin(name() + " netisr_input");
        fnBins.insert(make_pair("netisr_input", netisrInputBin));

        schednetisrIsrBin = new Statistics::MainBin(name() + " schednetisr_isr");
        fnBins.insert(make_pair("schednetisr_isr", schednetisrIsrBin));

        ipintrBin = new Statistics::MainBin(name() + " ipintr");
        fnBins.insert(make_pair("ipintr", ipintrBin));

        ipDooptionsBin = new Statistics::MainBin(name() + " ip_dooptions");
        fnBins.insert(make_pair("ip_dooptions", ipDooptionsBin));

        ipReassBin = new Statistics::MainBin(name() + " ip_reass");
        fnBins.insert(make_pair("ip_reass", ipReassBin));

        tcpInputBin = new Statistics::MainBin(name() + " tcp_input");
        fnBins.insert(make_pair("tcp_input", tcpInputBin));

        sbappendBin = new Statistics::MainBin(name() + " sbappend");
        fnBins.insert(make_pair("sbappend", sbappendBin));

        readBin = new Statistics::MainBin(name() + " read");
        fnBins.insert(make_pair("read", readBin));

        sooReadBin = new Statistics::MainBin(name() + " soo_read");
        fnBins.insert(make_pair("soo_read", sooReadBin));

        orecvBin = new Statistics::MainBin(name() + " orecv");
        fnBins.insert(make_pair("orecv", orecvBin));

        recvitBin = new Statistics::MainBin(name() + " recvit");
        fnBins.insert(make_pair("recvit", recvitBin));

        soreceiveBin = new Statistics::MainBin(name() + " soreceive");
        fnBins.insert(make_pair("soreceive", soreceiveBin));

        osendBin = new Statistics::MainBin(name() + " osend");
        fnBins.insert(make_pair("osend", osendBin));

        writeBin = new Statistics::MainBin(name() + " write");
        fnBins.insert(make_pair("write", writeBin));

        sooWriteBin = new Statistics::MainBin(name() + " soo_write");
        fnBins.insert(make_pair("soo_write", sooWriteBin));

        senditBin = new Statistics::MainBin(name() + " sendit");
        fnBins.insert(make_pair("sendit", senditBin));

        sosendBin = new Statistics::MainBin(name() + " sosend");
        fnBins.insert(make_pair("sosend", sosendBin));

        tcpSosendBin = new Statistics::MainBin(name() + " tcp_sosend");
        fnBins.insert(make_pair("tcp_sosend", tcpSosendBin));

        tcpOutputBin = new Statistics::MainBin(name() + " tcp_output");
        fnBins.insert(make_pair("tcp_output", tcpOutputBin));

        ipOutputBin = new Statistics::MainBin(name() + " ip_output");
        fnBins.insert(make_pair("ip_output", ipOutputBin));

        etherOutputBin = new Statistics::MainBin(name() + " ether_output");
        fnBins.insert(make_pair("ether_output", etherOutputBin));

        esStartBin = new Statistics::MainBin(name() + " es_start");
        fnBins.insert(make_pair("es_start", esStartBin));

        esTransmitBin = new Statistics::MainBin(name() + " es_transmit");
        fnBins.insert(make_pair("es_transmit", esTransmitBin));

        esTxeofBin = new Statistics::MainBin(name() + " es_txeof");
        fnBins.insert(make_pair("es_txeof", esTxeofBin));

        idleThreadBin = new Statistics::MainBin(name() + " idle_thread");
        fnBins.insert(make_pair("idle_thread", idleThreadBin));

    }
    //INSTRUMENTATION CODEGEN END
#endif //FS_MEASURE

    kernelPanicEvent = new BreakPCEvent(&pcEventQueue, "kernel panic");
    consolePanicEvent = new BreakPCEvent(&pcEventQueue, "console panic");
    badaddrEvent = new BadAddrEvent(&pcEventQueue, "badaddr");
    skipPowerStateEvent = new SkipFuncEvent(&pcEventQueue,
                                            "tl_v48_capture_power_state");
    skipScavengeBootEvent = new SkipFuncEvent(&pcEventQueue,
                                              "pmap_scavenge_boot");
    printfEvent = new PrintfEvent(&pcEventQueue, "printf");
    debugPrintfEvent = new DebugPrintfEvent(&pcEventQueue,
                                            "debug_printf", false);
    debugPrintfrEvent = new DebugPrintfEvent(&pcEventQueue,
                                             "debug_printfr", true);
    dumpMbufEvent = new DumpMbufEvent(&pcEventQueue, "dump_mbuf");

#ifdef FS_MEASURE
    //INSTRUMENTATION CODEGEN BEGIN TWO
    if (bin == true) {
          esIntrEvent = new FnEvent(&pcEventQueue, "es_intr", this);
          esRxeofEvent = new FnEvent(&pcEventQueue, "es_rxeof", this);
          esNewbufEvent = new FnEvent(&pcEventQueue, "es_newbuf", this);
          esDmaLoadEvent = new FnEvent(&pcEventQueue, "es_dma_load", this);
          dmaMapLoadEvent = new FnEvent(&pcEventQueue, "dma_map_load", this);
          etherInputEvent = new FnEvent(&pcEventQueue, "ether_input", this);
          netisrInputEvent = new FnEvent(&pcEventQueue, "netisr_input", this);
          schednetisrIsrEvent = new FnEvent(&pcEventQueue, "schednetisr_isr", this);
          ipintrEvent = new FnEvent(&pcEventQueue, "ipintr", this);
          ipDooptionsEvent = new FnEvent(&pcEventQueue, "ip_dooptions", this);
          ipReassEvent = new FnEvent(&pcEventQueue, "ip_reass", this);
          tcpInputEvent = new FnEvent(&pcEventQueue, "tcp_input", this);
          sbappendEvent = new FnEvent(&pcEventQueue, "sbappend", this);
          readEvent = new FnEvent(&pcEventQueue, "read", this);
          sooReadEvent = new FnEvent(&pcEventQueue, "soo_read", this);
          orecvEvent = new FnEvent(&pcEventQueue, "orecv", this);
          recvitEvent = new FnEvent(&pcEventQueue, "recvit", this);
          soreceiveEvent = new FnEvent(&pcEventQueue, "soreceive", this);
          osendEvent = new FnEvent(&pcEventQueue, "osend", this);
          writeEvent = new FnEvent(&pcEventQueue, "write", this);
          sooWriteEvent = new FnEvent(&pcEventQueue, "soo_write", this);
          senditEvent = new FnEvent(&pcEventQueue, "sendit", this);
          sosendEvent = new FnEvent(&pcEventQueue, "sosend", this);
          tcpSosendEvent = new FnEvent(&pcEventQueue, "tcp_sosend", this);
          tcpOutputEvent = new FnEvent(&pcEventQueue, "tcp_output", this);
          ipOutputEvent = new FnEvent(&pcEventQueue, "ip_output", this);
          etherOutputEvent = new FnEvent(&pcEventQueue, "ether_output", this);
          esStartEvent = new FnEvent(&pcEventQueue, "es_start", this);
          esTransmitEvent = new FnEvent(&pcEventQueue, "es_transmit", this);
          esTxeofEvent = new FnEvent(&pcEventQueue, "es_txeof", this);
          idleThreadEvent = new FnEvent(&pcEventQueue, "idle_thread", this);
    }
    //INSTRUMENTATION CODEGEN END
#endif //FS_MEASURE

    Addr addr = 0;
    if (kernelSymtab->findAddress("enable_async_printf", addr)) {
        Addr paddr = vtophys(physmem, addr);
        uint8_t *enable_async_printf =
            physmem->dma_addr(paddr, sizeof(uint32_t));

        if (enable_async_printf)
            *(uint32_t *)enable_async_printf = 0;
    }

    if (consoleSymtab->findAddress("env_booted_osflags", addr)) {
        Addr paddr = vtophys(physmem, addr);
        char *osflags = (char *)physmem->dma_addr(paddr, sizeof(uint32_t));

        if (osflags)
            strcpy(osflags, boot_osflags.c_str());
    }

    if (kernelSymtab->findAddress("panic", addr))
        kernelPanicEvent->schedule(addr);
    else
        panic("could not find kernel symbol \'panic\'");

    if (consoleSymtab->findAddress("panic", addr))
        consolePanicEvent->schedule(addr);

    if (kernelSymtab->findAddress("badaddr", addr))
        badaddrEvent->schedule(addr);
    else
        panic("could not find kernel symbol \'badaddr\'");

    if (kernelSymtab->findAddress("tl_v48_capture_power_state", addr))
        skipPowerStateEvent->schedule(addr);

    if (kernelSymtab->findAddress("pmap_scavenge_boot", addr))
        skipScavengeBootEvent->schedule(addr);

#if TRACING_ON
    if (kernelSymtab->findAddress("printf", addr))
        printfEvent->schedule(addr);

    if (kernelSymtab->findAddress("m5printf", addr))
        debugPrintfEvent->schedule(addr);

    if (kernelSymtab->findAddress("m5printfr", addr))
        debugPrintfrEvent->schedule(addr);

    if (kernelSymtab->findAddress("m5_dump_mbuf", addr))
        dumpMbufEvent->schedule(addr);
#endif

#ifdef FS_MEASURE
    //INSTRUMENTATION CODEGEN BEGIN THREE
    if (bin == true) {
        if (kernelSymtab->findAddress("es_intr", addr))
            esIntrEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'es_intr\'");

        if (kernelSymtab->findAddress("es_rxeof", addr))
            esRxeofEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'es_rxeof\'");

        if (kernelSymtab->findAddress("es_newbuf", addr))
            esNewbufEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'es_newbuf\'");

        if (kernelSymtab->findAddress("es_dma_load", addr))
            esDmaLoadEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'es_dma_load\'");

        if (kernelSymtab->findAddress("dma_map_load", addr))
            dmaMapLoadEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'dma_map_load\'");

        if (kernelSymtab->findAddress("ether_input", addr))
            etherInputEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'ether_input\'");

        if (kernelSymtab->findAddress("netisr_input", addr))
            netisrInputEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'netisr_input\'");

        if (kernelSymtab->findAddress("schednetisr_isr", addr))
            schednetisrIsrEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'schednetisr_isr\'");

        if (kernelSymtab->findAddress("ipintr", addr))
            ipintrEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'ipintr\'");

        if (kernelSymtab->findAddress("ip_dooptions", addr))
            ipDooptionsEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'ip_dooptions\'");

        if (kernelSymtab->findAddress("ip_reass", addr))
            ipReassEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'ip_reass\'");

        if (kernelSymtab->findAddress("tcp_input", addr))
            tcpInputEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'tcp_input\'");

        if (kernelSymtab->findAddress("sbappend", addr))
            sbappendEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'sbappend\'");

        if (kernelSymtab->findAddress("read", addr))
            readEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'read\'");

        if (kernelSymtab->findAddress("soo_read", addr))
            sooReadEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'soo_read\'");

        if (kernelSymtab->findAddress("orecv", addr))
            orecvEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'orecv\'");

        if (kernelSymtab->findAddress("recvit", addr))
            recvitEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'recvit\'");

        if (kernelSymtab->findAddress("soreceive", addr))
            soreceiveEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'soreceive\'");

        if (kernelSymtab->findAddress("osend", addr))
            osendEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'osend\'");

        if (kernelSymtab->findAddress("write", addr))
            writeEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'write\'");

        if (kernelSymtab->findAddress("soo_write", addr))
            sooWriteEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'soo_write\'");

        if (kernelSymtab->findAddress("sendit", addr))
            senditEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'sendit\'");

        if (kernelSymtab->findAddress("sosend", addr))
            sosendEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'sosend\'");

        if (kernelSymtab->findAddress("tcp_sosend", addr))
            tcpSosendEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'tcp_sosend\'");

        if (kernelSymtab->findAddress("tcp_output", addr))
            tcpOutputEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'tcp_output\'");

        if (kernelSymtab->findAddress("ip_output", addr))
            ipOutputEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'ip_output\'");

        if (kernelSymtab->findAddress("ether_output", addr))
            etherOutputEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'ether_output\'");

        if (kernelSymtab->findAddress("es_start", addr))
            esStartEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'es_start\'");

        if (kernelSymtab->findAddress("es_transmit", addr))
            esTransmitEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'es_transmit\'");

        if (kernelSymtab->findAddress("es_txeof", addr))
            esTxeofEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'es_txeof\'");

        if (kernelSymtab->findAddress("idle_thread", addr))
            idleThreadEvent->schedule(addr);
        else
            panic("could not find kernel symbol \'idle_thread\'");

    }
    //INSTRUMENTATION CODEGEN END
     if (bin == true) {
        fnCalls
            .name(name() + ":fnCalls")
            .desc("all fn calls being tracked")
            ;

        populateMap("es_intr", "");
        populateMap("es_rxeof", "es_intr");
        populateMap("es_newbuf", "es_rxeof");
        populateMap("es_dma_load", "es_newbuf");
        populateMap("dma_map_load", "es_dma_load");
        populateMap("ether_input", "es_rxeof");
        populateMap("netisr_input", "ether_input");
        populateMap("schednetisr_isr", "netisr_input");

        populateMap("ipintr", "");
        populateMap("ip_dooptions", "ipintr");
        populateMap("ip_reass", "ipintr");
        populateMap("tcp_input", "ipintr");
        populateMap("sbappend", "tcp_input");

        populateMap("read", "");
        populateMap("orecv", "");
        populateMap("soo_read", "read");
        populateMap("recvit", "orecv");
        populateMap("soreceive", "recvit");
        populateMap("soreceive", "soo_read");

        populateMap("write", "");
        populateMap("osend", "");
        populateMap("soo_write", "write");
        populateMap("sendit", "osend");
        populateMap("sosend", "sendit");
        populateMap("sosend", "soo_write");
        populateMap("tcp_sosend", "sosend");
        populateMap("tcp_output", "tcp_sosend");
        populateMap("ip_output", "tcp_output");
        populateMap("ether_output", "ip_output");
        populateMap("es_start", "ether_output");
        populateMap("es_transmit", "es_start");

        populateMap("es_txeof", "es_intr");

        populateMap("idle_thread", "");
    }
#endif //FS_MEASURE
}

Tru64System::~Tru64System()
{
    delete kernel;
    delete console;

    delete kernelSymtab;
    delete consoleSymtab;

    delete kernelPanicEvent;
    delete consolePanicEvent;
    delete badaddrEvent;
    delete skipPowerStateEvent;
    delete skipScavengeBootEvent;
    delete printfEvent;
    delete debugPrintfEvent;
    delete debugPrintfrEvent;
    delete dumpMbufEvent;

#ifdef FS_MEASURE
    //INSTRUMENTATION CODEGEN BEGIN FOUR
    if (bin == true) {
        delete esIntrEvent;
        delete esRxeofEvent;
        delete esNewbufEvent;
        delete esDmaLoadEvent;
        delete dmaMapLoadEvent;
        delete etherInputEvent;
        delete netisrInputEvent;
        delete schednetisrIsrEvent;
        delete ipintrEvent;
        delete ipDooptionsEvent;
        delete ipReassEvent;
        delete tcpInputEvent;
        delete sbappendEvent;
        delete readEvent;
        delete sooReadEvent;
        delete orecvEvent;
        delete recvitEvent;
        delete soreceiveEvent;
        delete osendEvent;
        delete writeEvent;
        delete sooWriteEvent;
        delete senditEvent;
        delete sosendEvent;
        delete tcpSosendEvent;
        delete tcpOutputEvent;
        delete ipOutputEvent;
        delete etherOutputEvent;
        delete esStartEvent;
        delete esTransmitEvent;
        delete esTxeofEvent;
        delete idleThreadEvent;
    }
    //INSTRUMENTATION CODEGEN END
#endif //FS_MEASURE
}

int
Tru64System::registerExecContext(ExecContext *xc)
{
    int xcIndex = System::registerExecContext(xc);

    if (xcIndex == 0) {
        // activate with zero delay so that we start ticking right
        // away on cycle 0
        xc->activate(0);
    }

    RemoteGDB *rgdb = new RemoteGDB(this, xc);
    GDBListener *gdbl = new GDBListener(rgdb, 7000 + xcIndex);
    gdbl->listen();

    if (remoteGDB.size() <= xcIndex) {
        remoteGDB.resize(xcIndex+1);
    }

    remoteGDB[xcIndex] = rgdb;

    return xcIndex;
}


void
Tru64System::replaceExecContext(ExecContext *xc, int xcIndex)
{
    System::replaceExecContext(xcIndex, xc);
    remoteGDB[xcIndex]->replaceExecContext(xc);
}

bool
Tru64System::breakpoint()
{
    return remoteGDB[0]->trap(ALPHA_KENTRY_IF);
}

#ifdef FS_MEASURE
void
Tru64System::populateMap(std::string callee, std::string caller)
{
    multimap<const string, string>::const_iterator i;
    i = callerMap.insert(make_pair(callee, caller));
    assert(i != callerMap.end() && "should not fail populating callerMap");
}

bool
Tru64System::findCaller(std::string callee, std::string caller) const
{
    typedef multimap<const std::string, std::string>::const_iterator iter;
    pair<iter, iter> range;

    range = callerMap.equal_range(callee);
    for (iter i = range.first; i != range.second; ++i) {
        if ((*i).second == caller)
            return true;
    }
    return false;
}

void
Tru64System::dumpState(ExecContext *xc) const
{
#ifndef SW_DEBUG
    return;
#endif
    if (xc->swCtx) {
        stack<fnCall *> copy(xc->swCtx->callStack);
        if (copy.empty())
            return;
        cprintf("xc->swCtx:\n");
        fnCall *top;
        cprintf("||   call: %d\n",xc->swCtx->calls);
        for (top = copy.top(); !copy.empty(); copy.pop() ) {
            top = copy.top();
            cprintf("||  %13s : %s \n", top->name, top->myBin->name());
        }
    }
}
#endif //FS_MEASURE

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Tru64System)

    Param<bool> bin;
    SimObjectParam<MemoryController *> mem_ctl;
    SimObjectParam<PhysicalMemory *> physmem;
    Param<uint64_t> init_param;

    Param<string> kernel_code;
    Param<string> console_code;
    Param<string> pal_code;
    Param<string> boot_osflags;

END_DECLARE_SIM_OBJECT_PARAMS(Tru64System)

BEGIN_INIT_SIM_OBJECT_PARAMS(Tru64System)

    INIT_PARAM_DFLT(bin, "is this system to be binned", false),
    INIT_PARAM(mem_ctl, "memory controller"),
    INIT_PARAM(physmem, "phsyical memory"),
    INIT_PARAM_DFLT(init_param, "numerical value to pass into simulator", 0),
    INIT_PARAM(kernel_code, "file that contains the kernel code"),
    INIT_PARAM(console_code, "file that contains the console code"),
    INIT_PARAM(pal_code, "file that contains palcode"),
    INIT_PARAM_DFLT(boot_osflags, "flags to pass to the kernel during boot",
                                   "a")


END_INIT_SIM_OBJECT_PARAMS(Tru64System)

CREATE_SIM_OBJECT(Tru64System)
{
    Tru64System *sys = new Tru64System(getInstanceName(), init_param, mem_ctl,
                                       physmem, kernel_code, console_code,
                                       pal_code, boot_osflags, bin);

    return sys;
}

REGISTER_SIM_OBJECT("Tru64System", Tru64System)
