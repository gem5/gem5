# Copyright (c) 2019-2021 ARM Limited
# All rights reserved.
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
# Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
# Copyright (c) 2009 The Hewlett-Packard Development Company
# Copyright (c) 2013 Advanced Micro Devices, Inc.
# All rights reserved.
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

from collections import OrderedDict

from slicc.symbols.Symbol import Symbol
from slicc.symbols.Var import Var
import slicc.generate.html as html
import re

python_class_map = {
                    "int": "Int",
                    "NodeID": "Int",
                    "uint32_t" : "UInt32",
                    "std::string": "String",
                    "bool": "Bool",
                    "CacheMemory": "RubyCache",
                    "WireBuffer": "RubyWireBuffer",
                    "Sequencer": "RubySequencer",
                    "HTMSequencer": "RubyHTMSequencer",
                    "GPUCoalescer" : "RubyGPUCoalescer",
                    "VIPERCoalescer" : "VIPERCoalescer",
                    "DirectoryMemory": "RubyDirectoryMemory",
                    "PerfectCacheMemory": "RubyPerfectCacheMemory",
                    "MemoryControl": "MemoryControl",
                    "MessageBuffer": "MessageBuffer",
                    "DMASequencer": "DMASequencer",
                    "RubyPrefetcher":"RubyPrefetcher",
                    "Cycles":"Cycles",
                   }

class StateMachine(Symbol):
    def __init__(self, symtab, ident, location, pairs, config_parameters):
        super().__init__(symtab, ident, location, pairs)
        self.table = None

        # Data members in the State Machine that have been declared before
        # the opening brace '{'  of the machine.  Note that these along with
        # the members in self.objects form the entire set of data members.
        self.config_parameters = config_parameters

        self.prefetchers = []

        for param in config_parameters:
            if param.pointer:
                var = Var(symtab, param.ident, location, param.type_ast.type,
                          "(*m_%s_ptr)" % param.ident, {}, self)
            else:
                var = Var(symtab, param.ident, location, param.type_ast.type,
                          "m_%s" % param.ident, {}, self)

            self.symtab.registerSym(param.ident, var)

            if str(param.type_ast.type) == "RubyPrefetcher":
                self.prefetchers.append(var)

        self.states = OrderedDict()
        self.events = OrderedDict()
        self.actions = OrderedDict()
        self.request_types = OrderedDict()
        self.transitions = []
        self.in_ports = []
        self.functions = []

        # Data members in the State Machine that have been declared inside
        # the {} machine.  Note that these along with the config params
        # form the entire set of data members of the machine.
        self.objects = []
        self.TBEType   = None
        self.EntryType = None
        # Python's sets are not sorted so we have to be careful when using
        # this to generate deterministic output.
        self.debug_flags = set()
        self.debug_flags.add('RubyGenerated')
        self.debug_flags.add('RubySlicc')

    def __repr__(self):
        return "[StateMachine: %s]" % self.ident

    def addState(self, state):
        assert self.table is None
        self.states[state.ident] = state

    def addEvent(self, event):
        assert self.table is None
        self.events[event.ident] = event

    def addAction(self, action):
        assert self.table is None

        # Check for duplicate action
        for other in self.actions.values():
            if action.ident == other.ident:
                action.warning("Duplicate action definition: %s" % action.ident)
                action.error("Duplicate action definition: %s" % action.ident)
            if action.short == other.short:
                other.warning("Duplicate action shorthand: %s" % other.ident)
                other.warning("    shorthand = %s" % other.short)
                action.warning("Duplicate action shorthand: %s" % action.ident)
                action.error("    shorthand = %s" % action.short)

        self.actions[action.ident] = action

    def addDebugFlag(self, flag):
        self.debug_flags.add(flag)

    def addRequestType(self, request_type):
        assert self.table is None
        self.request_types[request_type.ident] = request_type

    def addTransition(self, trans):
        assert self.table is None
        self.transitions.append(trans)

    def addInPort(self, var):
        self.in_ports.append(var)

    def addFunc(self, func):
        # register func in the symbol table
        self.symtab.registerSym(str(func), func)
        self.functions.append(func)

    def addObject(self, obj):
        self.symtab.registerSym(str(obj), obj)
        self.objects.append(obj)

    def addType(self, type):
        type_ident = '%s' % type.c_ident

        if type_ident == "%s_TBE" %self.ident:
            if self.TBEType != None:
                self.error("Multiple Transaction Buffer types in a " \
                           "single machine.");
            self.TBEType = type

        elif "interface" in type and "AbstractCacheEntry" == type["interface"]:
            if "main" in type and "false" == type["main"].lower():
                pass # this isn't the EntryType
            else:
                if self.EntryType != None:
                    self.error("Multiple AbstractCacheEntry types in a " \
                               "single machine.");
                self.EntryType = type

    # Needs to be called before accessing the table
    def buildTable(self):
        assert self.table is None

        table = {}

        for trans in self.transitions:
            # Track which actions we touch so we know if we use them
            # all -- really this should be done for all symbols as
            # part of the symbol table, then only trigger it for
            # Actions, States, Events, etc.

            for action in trans.actions:
                action.used = True

            index = (trans.state, trans.event)
            if index in table:
                table[index].warning("Duplicate transition: %s" % table[index])
                trans.error("Duplicate transition: %s" % trans)
            table[index] = trans

        # Look at all actions to make sure we used them all
        for action in self.actions.values():
            if not action.used:
                error_msg = "Unused action: %s" % action.ident
                if "desc" in action:
                    error_msg += ", "  + action.desc
                action.warning(error_msg)
        self.table = table

    # determine the port->msg buffer mappings
    def getBufferMaps(self, ident):
        msg_bufs = []
        port_to_buf_map = {}
        in_msg_bufs = {}
        for port in self.in_ports:
            buf_name = "m_%s_ptr" % port.pairs["buffer_expr"].name
            msg_bufs.append(buf_name)
            port_to_buf_map[port] = msg_bufs.index(buf_name)
            if buf_name not in in_msg_bufs:
                in_msg_bufs[buf_name] = [port]
            else:
                in_msg_bufs[buf_name].append(port)
        return port_to_buf_map, in_msg_bufs, msg_bufs

    def writeCodeFiles(self, path, includes):
        self.printControllerPython(path)
        self.printControllerHH(path)
        self.printControllerCC(path, includes)
        self.printCSwitch(path)
        self.printCWakeup(path, includes)

    def printControllerPython(self, path):
        code = self.symtab.codeFormatter()
        ident = self.ident

        py_ident = "%s_Controller" % ident
        c_ident = "%s_Controller" % self.ident

        code('''
from m5.params import *
from m5.SimObject import SimObject
from m5.objects.Controller import RubyController

class $py_ident(RubyController):
    type = '$py_ident'
    cxx_header = 'mem/ruby/protocol/${c_ident}.hh'
    cxx_class = 'gem5::ruby::$py_ident'
''')
        code.indent()
        for param in self.config_parameters:
            dflt_str = ''

            if param.rvalue is not None:
                dflt_str = str(param.rvalue.inline()) + ', '

            if param.type_ast.type.c_ident in python_class_map:
                python_type = python_class_map[param.type_ast.type.c_ident]
                code('${{param.ident}} = Param.${{python_type}}(${dflt_str}"")')

            else:
                self.error("Unknown c++ to python class conversion for c++ " \
                           "type: '%s'. Please update the python_class_map " \
                           "in StateMachine.py", param.type_ast.type.c_ident)

        code.dedent()
        code.write(path, '%s.py' % py_ident)


    def printControllerHH(self, path):
        '''Output the method declarations for the class declaration'''
        code = self.symtab.codeFormatter()
        ident = self.ident
        c_ident = "%s_Controller" % self.ident

        code('''
// Created by slicc definition of Module "${{self.short}}"

#ifndef __${ident}_CONTROLLER_HH__
#define __${ident}_CONTROLLER_HH__

#include <iostream>
#include <sstream>
#include <string>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/protocol/TransitionResult.hh"
#include "mem/ruby/protocol/Types.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "params/$c_ident.hh"

''')

        seen_types = set()
        for var in self.objects:
            if var.type.ident not in seen_types and not var.type.isPrimitive:
                code('#include "mem/ruby/protocol/${{var.type.c_ident}}.hh"')
                seen_types.add(var.type.ident)

        # for adding information to the protocol debug trace
        code('''
namespace gem5
{

namespace ruby
{

extern std::stringstream ${ident}_transitionComment;

class $c_ident : public AbstractController
{
  public:
    typedef ${c_ident}Params Params;
    $c_ident(const Params &p);
    static int getNumControllers();
    void init();

    MessageBuffer *getMandatoryQueue() const;
    MessageBuffer *getMemReqQueue() const;
    MessageBuffer *getMemRespQueue() const;
    void initNetQueues();

    void print(std::ostream& out) const;
    void wakeup();
    void resetStats();
    void regStats();
    void collateStats();

    void recordCacheTrace(int cntrl, CacheRecorder* tr);
    Sequencer* getCPUSequencer() const;
    DMASequencer* getDMASequencer() const;
    GPUCoalescer* getGPUCoalescer() const;

    bool functionalReadBuffers(PacketPtr&);
    bool functionalReadBuffers(PacketPtr&, WriteMask&);
    int functionalWriteBuffers(PacketPtr&);

    void countTransition(${ident}_State state, ${ident}_Event event);
    void possibleTransition(${ident}_State state, ${ident}_Event event);
    uint64_t getEventCount(${ident}_Event event);
    bool isPossible(${ident}_State state, ${ident}_Event event);
    uint64_t getTransitionCount(${ident}_State state, ${ident}_Event event);

private:
''')

        code.indent()
        # added by SS
        for param in self.config_parameters:
            if param.pointer:
                code('${{param.type_ast.type}}* m_${{param.ident}}_ptr;')
            else:
                code('${{param.type_ast.type}} m_${{param.ident}};')

        code('''
TransitionResult doTransition(${ident}_Event event,
''')

        if self.EntryType != None:
            code('''
                              ${{self.EntryType.c_ident}}* m_cache_entry_ptr,
''')
        if self.TBEType != None:
            code('''
                              ${{self.TBEType.c_ident}}* m_tbe_ptr,
''')

        code('''
                              Addr addr);

TransitionResult doTransitionWorker(${ident}_Event event,
                                    ${ident}_State state,
                                    ${ident}_State& next_state,
''')

        if self.TBEType != None:
            code('''
                                    ${{self.TBEType.c_ident}}*& m_tbe_ptr,
''')
        if self.EntryType != None:
            code('''
                                    ${{self.EntryType.c_ident}}*& m_cache_entry_ptr,
''')

        code('''
                                    Addr addr);

${ident}_Event m_curTransitionEvent;
${ident}_State m_curTransitionNextState;

${ident}_Event curTransitionEvent() { return m_curTransitionEvent; }
${ident}_State curTransitionNextState() { return m_curTransitionNextState; }

int m_counters[${ident}_State_NUM][${ident}_Event_NUM];
int m_event_counters[${ident}_Event_NUM];
bool m_possible[${ident}_State_NUM][${ident}_Event_NUM];

static std::vector<statistics::Vector *> eventVec;
static std::vector<std::vector<statistics::Vector *> > transVec;
static int m_num_controllers;

// Internal functions
''')

        for func in self.functions:
            proto = func.prototype
            if proto:
                code('$proto')

        if self.EntryType != None:
            code('''

// Set and Reset for cache_entry variable
void set_cache_entry(${{self.EntryType.c_ident}}*& m_cache_entry_ptr, AbstractCacheEntry* m_new_cache_entry);
void unset_cache_entry(${{self.EntryType.c_ident}}*& m_cache_entry_ptr);
''')

        if self.TBEType != None:
            code('''

// Set and Reset for tbe variable
void set_tbe(${{self.TBEType.c_ident}}*& m_tbe_ptr, ${ident}_TBE* m_new_tbe);
void unset_tbe(${{self.TBEType.c_ident}}*& m_tbe_ptr);
''')

        # Prototype the actions that the controller can take
        code('''

// Actions
''')
        if self.TBEType != None and self.EntryType != None:
            for action in self.actions.values():
                code('/** \\brief ${{action.desc}} */')
                code('void ${{action.ident}}(${{self.TBEType.c_ident}}*& '
                     'm_tbe_ptr, ${{self.EntryType.c_ident}}*& '
                     'm_cache_entry_ptr, Addr addr);')
        elif self.TBEType != None:
            for action in self.actions.values():
                code('/** \\brief ${{action.desc}} */')
                code('void ${{action.ident}}(${{self.TBEType.c_ident}}*& '
                     'm_tbe_ptr, Addr addr);')
        elif self.EntryType != None:
            for action in self.actions.values():
                code('/** \\brief ${{action.desc}} */')
                code('void ${{action.ident}}(${{self.EntryType.c_ident}}*& '
                     'm_cache_entry_ptr, Addr addr);')
        else:
            for action in self.actions.values():
                code('/** \\brief ${{action.desc}} */')
                code('void ${{action.ident}}(Addr addr);')

        # the controller internal variables
        code('''

// Objects
''')
        for var in self.objects:
            th = var.get("template", "")
            code('${{var.type.c_ident}}$th* m_${{var.ident}}_ptr;')

        code.dedent()
        code('''
};

} // namespace ruby
} // namespace gem5

#endif // __${ident}_CONTROLLER_H__
''')

        code.write(path, '%s.hh' % c_ident)

    def printControllerCC(self, path, includes):
        '''Output the actions for performing the actions'''

        code = self.symtab.codeFormatter()
        ident = self.ident
        c_ident = "%s_Controller" % self.ident

        # Unfortunately, clang compilers will throw a "call to function ...
        # that is neither visible in the template definition nor found by
        # argument-dependent lookup" error if "mem/ruby/common/BoolVec.hh" is
        # included after "base/cprintf.hh". This is because "base/cprintf.hh"
        # utilizes a "<<" operator in "base/cprintf_formats.hh" that is
        # defined in "mem/ruby/common/BoolVec.hh". While GCC compilers permit
        # the operator definition after usage in this case, clang compilers do
        # not.
        #
        # The reason for this verbose solution below is due to the gem5
        # style-checker, which will complain if "mem/ruby/common/BoolVec.hh"
        # is included above "base/cprintf.hh" in this file, despite it being
        # necessary in this case. This is therefore a bit of a hack to keep
        # both clang and our style-checker happy.
        base_include = '''
#include "base/compiler.hh"
#include "base/cprintf.hh"

'''

        boolvec_include = '''
#include "mem/ruby/common/BoolVec.hh"

'''

        code('''
// Created by slicc definition of Module "${{self.short}}"

#include <sys/types.h>
#include <unistd.h>

#include <cassert>
#include <sstream>
#include <string>
#include <typeinfo>

''')

        code(boolvec_include)
        code(base_include)
        # We have to sort self.debug_flags in order to produce deterministic
        # output and avoid unnecessary rebuilds of the generated files.
        for f in sorted(self.debug_flags):
            code('#include "debug/${{f}}.hh"')
        code('''
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/protocol/${ident}_Controller.hh"
#include "mem/ruby/protocol/${ident}_Event.hh"
#include "mem/ruby/protocol/${ident}_State.hh"
#include "mem/ruby/protocol/Types.hh"
#include "mem/ruby/system/RubySystem.hh"

''')
        for include_path in includes:
            code('#include "${{include_path}}"')

        # include object classes
        seen_types = set()
        for var in self.objects:
            if var.type.ident not in seen_types and not var.type.isPrimitive:
                code('#include "mem/ruby/protocol/${{var.type.c_ident}}.hh"')
            seen_types.add(var.type.ident)

        num_in_ports = len(self.in_ports)

        code('''
namespace gem5
{

namespace ruby
{

int $c_ident::m_num_controllers = 0;
std::vector<statistics::Vector *>  $c_ident::eventVec;
std::vector<std::vector<statistics::Vector *> >  $c_ident::transVec;

// for adding information to the protocol debug trace
std::stringstream ${ident}_transitionComment;

#ifndef NDEBUG
#define APPEND_TRANSITION_COMMENT(str) (${ident}_transitionComment << str)
#else
#define APPEND_TRANSITION_COMMENT(str) do {} while (0)
#endif

/** \\brief constructor */
$c_ident::$c_ident(const Params &p)
    : AbstractController(p)
{
    m_machineID.type = MachineType_${ident};
    m_machineID.num = m_version;
    m_num_controllers++;
    p.ruby_system->registerAbstractController(this);

    m_in_ports = $num_in_ports;
''')
        code.indent()

        #
        # After initializing the universal machine parameters, initialize the
        # this machines config parameters.  Also if these configuration params
        # include a sequencer, connect the it to the controller.
        #
        for param in self.config_parameters:
            if param.pointer:
                code('m_${{param.ident}}_ptr = p.${{param.ident}};')
            else:
                code('m_${{param.ident}} = p.${{param.ident}};')

            if re.compile("sequencer").search(param.ident) or \
                   param.type_ast.type.c_ident == "GPUCoalescer" or \
                   param.type_ast.type.c_ident == "VIPERCoalescer":
                code('''
if (m_${{param.ident}}_ptr != NULL) {
    m_${{param.ident}}_ptr->setController(this);
}
''')

        code('''

for (int state = 0; state < ${ident}_State_NUM; state++) {
    for (int event = 0; event < ${ident}_Event_NUM; event++) {
        m_possible[state][event] = false;
        m_counters[state][event] = 0;
    }
}
for (int event = 0; event < ${ident}_Event_NUM; event++) {
    m_event_counters[event] = 0;
}
''')
        code.dedent()
        code('''
}

void
$c_ident::initNetQueues()
{
    MachineType machine_type = string_to_MachineType("${{self.ident}}");
    [[maybe_unused]] int base = MachineType_base_number(machine_type);

''')
        code.indent()

        # set for maintaining the vnet, direction pairs already seen for this
        # machine.  This map helps in implementing the check for avoiding
        # multiple message buffers being mapped to the same vnet.
        vnet_dir_set = set()

        for var in self.config_parameters:
            vid = "m_%s_ptr" % var.ident
            if "network" in var:
                vtype = var.type_ast.type
                code('assert($vid != NULL);')

                # Network port object
                network = var["network"]

                if "virtual_network" in var:
                    vnet = var["virtual_network"]
                    vnet_type = var["vnet_type"]

                    assert (vnet, network) not in vnet_dir_set
                    vnet_dir_set.add((vnet,network))

                    code('''
m_net_ptr->set${network}NetQueue(m_version + base, $vid->getOrdered(), $vnet,
                                 "$vnet_type", $vid);
''')
                # Set Priority
                if "rank" in var:
                    code('$vid->setPriority(${{var["rank"]}})')

        code.dedent()
        code('''
}

void
$c_ident::init()
{
    // initialize objects
''')

        code.indent()

        for var in self.objects:
            vtype = var.type
            vid = "m_%s_ptr" % var.ident
            if "network" not in var:
                # Not a network port object
                if "primitive" in vtype:
                    code('$vid = new ${{vtype.c_ident}};')
                    if "default" in var:
                        code('(*$vid) = ${{var["default"]}};')
                else:
                    # Normal Object
                    th = var.get("template", "")
                    expr = "%s  = new %s%s" % (vid, vtype.c_ident, th)
                    args = ""
                    if "non_obj" not in vtype and not vtype.isEnumeration:
                        args = var.get("constructor", "")

                    code('$expr($args);')
                    code('assert($vid != NULL);')

                    if "default" in var:
                        code('*$vid = ${{var["default"]}}; // Object default')
                    elif "default" in vtype:
                        comment = "Type %s default" % vtype.ident
                        code('*$vid = ${{vtype["default"]}}; // $comment')

        # Set the prefetchers
        code()
        for prefetcher in self.prefetchers:
            code('${{prefetcher.code}}.setController(this);')

        code()
        for port in self.in_ports:
            # Set the queue consumers
            code('${{port.code}}.setConsumer(this);')

        # Initialize the transition profiling
        code()
        for trans in self.transitions:
            # Figure out if we stall
            stall = False
            for action in trans.actions:
                if action.ident == "z_stall":
                    stall = True

            # Only possible if it is not a 'z' case
            if not stall:
                state = "%s_State_%s" % (self.ident, trans.state.ident)
                event = "%s_Event_%s" % (self.ident, trans.event.ident)
                code('possibleTransition($state, $event);')

        code.dedent()
        code('''
    AbstractController::init();
    resetStats();
}
''')

        mq_ident = "NULL"
        for port in self.in_ports:
            if port.code.find("mandatoryQueue_ptr") >= 0:
                mq_ident = "m_mandatoryQueue_ptr"

        memoutq_ident = "NULL"
        for param in self.config_parameters:
            if param.ident.find("requestToMemory") >= 0:
                memoutq_ident = "m_requestToMemory_ptr"

        memq_ident = "NULL"
        for port in self.in_ports:
            if port.code.find("responseFromMemory_ptr") >= 0:
                memq_ident = "m_responseFromMemory_ptr"

        seq_ident = "NULL"
        for param in self.config_parameters:
            if param.ident == "sequencer":
                assert(param.pointer)
                seq_ident = "m_%s_ptr" % param.ident

        dma_seq_ident = "NULL"
        for param in self.config_parameters:
            if param.ident == "dma_sequencer":
                assert(param.pointer)
                dma_seq_ident = "m_%s_ptr" % param.ident

        coal_ident = "NULL"
        for param in self.config_parameters:
            if param.ident == "coalescer":
                assert(param.pointer)
                coal_ident = "m_%s_ptr" % param.ident

        if seq_ident != "NULL":
            code('''
Sequencer*
$c_ident::getCPUSequencer() const
{
    if (NULL != $seq_ident && $seq_ident->isCPUSequencer()) {
        return $seq_ident;
    } else {
        return NULL;
    }
}
''')
        else:
            code('''

Sequencer*
$c_ident::getCPUSequencer() const
{
    return NULL;
}
''')

        if dma_seq_ident != "NULL":
            code('''
DMASequencer*
$c_ident::getDMASequencer() const
{
    if (NULL != $dma_seq_ident) {
        return $dma_seq_ident;
    } else {
        return NULL;
    }
}
''')
        else:
            code('''

DMASequencer*
$c_ident::getDMASequencer() const
{
    return NULL;
}
''')

        if coal_ident != "NULL":
            code('''
GPUCoalescer*
$c_ident::getGPUCoalescer() const
{
    if (NULL != $coal_ident && !$coal_ident->isCPUSequencer()) {
        return $coal_ident;
    } else {
        return NULL;
    }
}
''')
        else:
            code('''

GPUCoalescer*
$c_ident::getGPUCoalescer() const
{
    return NULL;
}
''')

        code('''

void
$c_ident::regStats()
{
    AbstractController::regStats();

    // For each type of controllers, one controller of that type is picked
    // to aggregate stats of all controllers of that type. 
    if (m_version == 0) {

        Profiler *profiler = params().ruby_system->getProfiler();
        statistics::Group *profilerStatsPtr = &profiler->rubyProfilerStats;

        for (${ident}_Event event = ${ident}_Event_FIRST;
             event < ${ident}_Event_NUM; ++event) {
            std::string stat_name =
                "${c_ident}." + ${ident}_Event_to_string(event);
            statistics::Vector *t =
                new statistics::Vector(profilerStatsPtr, stat_name.c_str());
            t->init(m_num_controllers);
            t->flags(statistics::pdf | statistics::total |
                statistics::oneline | statistics::nozero);

            eventVec.push_back(t);
        }

        for (${ident}_State state = ${ident}_State_FIRST;
             state < ${ident}_State_NUM; ++state) {

            transVec.push_back(std::vector<statistics::Vector *>());

            for (${ident}_Event event = ${ident}_Event_FIRST;
                 event < ${ident}_Event_NUM; ++event) {
                std::string stat_name = "${c_ident}." +
                    ${ident}_State_to_string(state) +
                    "." + ${ident}_Event_to_string(event);
                statistics::Vector *t = new statistics::Vector(
                    profilerStatsPtr, stat_name.c_str());
                t->init(m_num_controllers);
                t->flags(statistics::pdf | statistics::total |
                    statistics::oneline | statistics::nozero);
                transVec[state].push_back(t);
            }
        }
    }

    for (${ident}_Event event = ${ident}_Event_FIRST;
                 event < ${ident}_Event_NUM; ++event) {
        std::string stat_name =
            "outTransLatHist." + ${ident}_Event_to_string(event);
        statistics::Histogram* t =
            new statistics::Histogram(&stats, stat_name.c_str());
        stats.outTransLatHist.push_back(t);
        t->init(5);
        t->flags(statistics::pdf | statistics::total |
                 statistics::oneline | statistics::nozero);

        statistics::Scalar* r = new statistics::Scalar(&stats,
                                             (stat_name + ".retries").c_str());
        stats.outTransLatHistRetries.push_back(r);
        r->flags(statistics::nozero);
    }

    for (${ident}_Event event = ${ident}_Event_FIRST;
                 event < ${ident}_Event_NUM; ++event) {
        std::string stat_name = "inTransLatHist." +
                                ${ident}_Event_to_string(event);
        statistics::Scalar* r = new statistics::Scalar(&stats,
                                             (stat_name + ".total").c_str());
        stats.inTransLatTotal.push_back(r);
        r->flags(statistics::nozero);

        r = new statistics::Scalar(&stats,
                              (stat_name + ".retries").c_str());
        stats.inTransLatRetries.push_back(r);
        r->flags(statistics::nozero);

        stats.inTransLatHist.emplace_back();
        for (${ident}_State initial_state = ${ident}_State_FIRST;
             initial_state < ${ident}_State_NUM; ++initial_state) {
            stats.inTransLatHist.back().emplace_back();
            for (${ident}_State final_state = ${ident}_State_FIRST;
                 final_state < ${ident}_State_NUM; ++final_state) {
                std::string stat_name = "inTransLatHist." +
                    ${ident}_Event_to_string(event) + "." +
                    ${ident}_State_to_string(initial_state) + "." +
                    ${ident}_State_to_string(final_state);
                statistics::Histogram* t =
                    new statistics::Histogram(&stats, stat_name.c_str());
                stats.inTransLatHist.back().back().push_back(t);
                t->init(5);
                t->flags(statistics::pdf | statistics::total |
                         statistics::oneline | statistics::nozero);
            }
        }
    }
}

void
$c_ident::collateStats()
{
    for (${ident}_Event event = ${ident}_Event_FIRST;
         event < ${ident}_Event_NUM; ++event) {
        for (unsigned int i = 0; i < m_num_controllers; ++i) {
            RubySystem *rs = params().ruby_system;
            std::map<uint32_t, AbstractController *>::iterator it =
                     rs->m_abstract_controls[MachineType_${ident}].find(i);
            assert(it != rs->m_abstract_controls[MachineType_${ident}].end());
            (*eventVec[event])[i] =
                (($c_ident *)(*it).second)->getEventCount(event);
        }
    }

    for (${ident}_State state = ${ident}_State_FIRST;
         state < ${ident}_State_NUM; ++state) {

        for (${ident}_Event event = ${ident}_Event_FIRST;
             event < ${ident}_Event_NUM; ++event) {

            for (unsigned int i = 0; i < m_num_controllers; ++i) {
                RubySystem *rs = params().ruby_system;
                std::map<uint32_t, AbstractController *>::iterator it =
                         rs->m_abstract_controls[MachineType_${ident}].find(i);
                assert(it != rs->m_abstract_controls[MachineType_${ident}].end());
                (*transVec[state][event])[i] =
                    (($c_ident *)(*it).second)->getTransitionCount(state, event);
            }
        }
    }
}

void
$c_ident::countTransition(${ident}_State state, ${ident}_Event event)
{
    assert(m_possible[state][event]);
    m_counters[state][event]++;
    m_event_counters[event]++;
}
void
$c_ident::possibleTransition(${ident}_State state,
                             ${ident}_Event event)
{
    m_possible[state][event] = true;
}

uint64_t
$c_ident::getEventCount(${ident}_Event event)
{
    return m_event_counters[event];
}

bool
$c_ident::isPossible(${ident}_State state, ${ident}_Event event)
{
    return m_possible[state][event];
}

uint64_t
$c_ident::getTransitionCount(${ident}_State state,
                             ${ident}_Event event)
{
    return m_counters[state][event];
}

int
$c_ident::getNumControllers()
{
    return m_num_controllers;
}

MessageBuffer*
$c_ident::getMandatoryQueue() const
{
    return $mq_ident;
}

MessageBuffer*
$c_ident::getMemReqQueue() const
{
    return $memoutq_ident;
}

MessageBuffer*
$c_ident::getMemRespQueue() const
{
    return $memq_ident;
}

void
$c_ident::print(std::ostream& out) const
{
    out << "[$c_ident " << m_version << "]";
}

void $c_ident::resetStats()
{
    for (int state = 0; state < ${ident}_State_NUM; state++) {
        for (int event = 0; event < ${ident}_Event_NUM; event++) {
            m_counters[state][event] = 0;
        }
    }

    for (int event = 0; event < ${ident}_Event_NUM; event++) {
        m_event_counters[event] = 0;
    }

    AbstractController::resetStats();
}
''')

        if self.EntryType != None:
            code('''

// Set and Reset for cache_entry variable
void
$c_ident::set_cache_entry(${{self.EntryType.c_ident}}*& m_cache_entry_ptr, AbstractCacheEntry* m_new_cache_entry)
{
  m_cache_entry_ptr = (${{self.EntryType.c_ident}}*)m_new_cache_entry;
}

void
$c_ident::unset_cache_entry(${{self.EntryType.c_ident}}*& m_cache_entry_ptr)
{
  m_cache_entry_ptr = 0;
}
''')

        if self.TBEType != None:
            code('''

// Set and Reset for tbe variable
void
$c_ident::set_tbe(${{self.TBEType.c_ident}}*& m_tbe_ptr, ${{self.TBEType.c_ident}}* m_new_tbe)
{
  m_tbe_ptr = m_new_tbe;
}

void
$c_ident::unset_tbe(${{self.TBEType.c_ident}}*& m_tbe_ptr)
{
  m_tbe_ptr = NULL;
}
''')

        code('''

void
$c_ident::recordCacheTrace(int cntrl, CacheRecorder* tr)
{
''')
        #
        # Record cache contents for all associated caches.
        #
        code.indent()
        for param in self.config_parameters:
            if param.type_ast.type.ident == "CacheMemory":
                assert(param.pointer)
                code('m_${{param.ident}}_ptr->recordCacheContents(cntrl, tr);')

        code.dedent()
        code('''
}

// Actions
''')
        if self.TBEType != None and self.EntryType != None:
            for action in self.actions.values():
                if "c_code" not in action:
                 continue

                code('''
/** \\brief ${{action.desc}} */
void
$c_ident::${{action.ident}}(${{self.TBEType.c_ident}}*& m_tbe_ptr, ${{self.EntryType.c_ident}}*& m_cache_entry_ptr, Addr addr)
{
    DPRINTF(RubyGenerated, "executing ${{action.ident}}\\n");
    try {
       ${{action["c_code"]}}
    } catch (const RejectException & e) {
       fatal("Error in action ${{ident}}:${{action.ident}}: "
             "executed a peek statement with the wrong message "
             "type specified. ");
    }
}

''')
        elif self.TBEType != None:
            for action in self.actions.values():
                if "c_code" not in action:
                 continue

                code('''
/** \\brief ${{action.desc}} */
void
$c_ident::${{action.ident}}(${{self.TBEType.c_ident}}*& m_tbe_ptr, Addr addr)
{
    DPRINTF(RubyGenerated, "executing ${{action.ident}}\\n");
    ${{action["c_code"]}}
}

''')
        elif self.EntryType != None:
            for action in self.actions.values():
                if "c_code" not in action:
                 continue

                code('''
/** \\brief ${{action.desc}} */
void
$c_ident::${{action.ident}}(${{self.EntryType.c_ident}}*& m_cache_entry_ptr, Addr addr)
{
    DPRINTF(RubyGenerated, "executing ${{action.ident}}\\n");
    ${{action["c_code"]}}
}

''')
        else:
            for action in self.actions.values():
                if "c_code" not in action:
                 continue

                code('''
/** \\brief ${{action.desc}} */
void
$c_ident::${{action.ident}}(Addr addr)
{
    DPRINTF(RubyGenerated, "executing ${{action.ident}}\\n");
    ${{action["c_code"]}}
}

''')
        for func in self.functions:
            code(func.generateCode())

        # Function for functional writes to messages buffered in the controller
        code('''
int
$c_ident::functionalWriteBuffers(PacketPtr& pkt)
{
    int num_functional_writes = 0;
''')
        for var in self.objects:
            vtype = var.type
            if vtype.isBuffer:
                vid = "m_%s_ptr" % var.ident
                code('num_functional_writes += $vid->functionalWrite(pkt);')

        for var in self.config_parameters:
            vtype = var.type_ast.type
            if vtype.isBuffer:
                vid = "m_%s_ptr" % var.ident
                code('num_functional_writes += $vid->functionalWrite(pkt);')

        code('''
    return num_functional_writes;
}
''')

        # Function for functional reads to messages buffered in the controller
        code('''
bool
$c_ident::functionalReadBuffers(PacketPtr& pkt)
{
''')
        for var in self.objects:
            vtype = var.type
            if vtype.isBuffer:
                vid = "m_%s_ptr" % var.ident
                code('if ($vid->functionalRead(pkt)) return true;')

        for var in self.config_parameters:
            vtype = var.type_ast.type
            if vtype.isBuffer:
                vid = "m_%s_ptr" % var.ident
                code('if ($vid->functionalRead(pkt)) return true;')

        code('''
    return false;
}

bool
$c_ident::functionalReadBuffers(PacketPtr& pkt, WriteMask &mask)
{
    bool read = false;
''')
        for var in self.objects:
            vtype = var.type
            if vtype.isBuffer:
                vid = "m_%s_ptr" % var.ident
                code('if ($vid->functionalRead(pkt, mask)) read = true;')

        for var in self.config_parameters:
            vtype = var.type_ast.type
            if vtype.isBuffer:
                vid = "m_%s_ptr" % var.ident
                code('if ($vid->functionalRead(pkt, mask)) read = true;')

        code('''
    return read;
}

} // namespace ruby
} // namespace gem5
''')

        code.write(path, "%s.cc" % c_ident)

    def printCWakeup(self, path, includes):
        '''Output the wakeup loop for the events'''

        code = self.symtab.codeFormatter()
        ident = self.ident

        outputRequest_types = True
        if len(self.request_types) == 0:
            outputRequest_types = False

        code('''
// ${ident}: ${{self.short}}

#include <sys/types.h>
#include <unistd.h>

#include <cassert>
#include <typeinfo>

#include "base/logging.hh"

''')
        # We have to sort self.debug_flags in order to produce deterministic
        # output and avoid unnecessary rebuilds of the generated files.
        for f in sorted(self.debug_flags):
            code('#include "debug/${{f}}.hh"')
        code('''
#include "mem/ruby/protocol/${ident}_Controller.hh"
#include "mem/ruby/protocol/${ident}_Event.hh"
#include "mem/ruby/protocol/${ident}_State.hh"

''')

        if outputRequest_types:
            code('''#include "mem/ruby/protocol/${ident}_RequestType.hh"''')

        code('''
#include "mem/ruby/protocol/Types.hh"
#include "mem/ruby/system/RubySystem.hh"

''')


        for include_path in includes:
            code('#include "${{include_path}}"')

        port_to_buf_map, in_msg_bufs, msg_bufs = self.getBufferMaps(ident)

        code('''
namespace gem5
{

namespace ruby
{

void
${ident}_Controller::wakeup()
{
    if (getMemReqQueue() && getMemReqQueue()->isReady(clockEdge())) {
        serviceMemoryQueue();
    }

    int counter = 0;
    while (true) {
        unsigned char rejected[${{len(msg_bufs)}}];
        memset(rejected, 0, sizeof(unsigned char)*${{len(msg_bufs)}});
        // Some cases will put us into an infinite loop without this limit
        assert(counter <= m_transitions_per_cycle);
        if (counter == m_transitions_per_cycle) {
            // Count how often we are fully utilized
            stats.fullyBusyCycles++;

            // Wakeup in another cycle and try again
            scheduleEvent(Cycles(1));
            break;
        }
''')

        code.indent()
        code.indent()

        # InPorts
        #
        for port in self.in_ports:
            code.indent()
            code('// ${ident}InPort $port')
            if "rank" in port.pairs:
                code('m_cur_in_port = ${{port.pairs["rank"]}};')
            else:
                code('m_cur_in_port = 0;')
            if port in port_to_buf_map:
                code('try {')
                code.indent()
            code('${{port["c_code_in_port"]}}')

            if port in port_to_buf_map:
                code.dedent()
                code('''
            } catch (const RejectException & e) {
                rejected[${{port_to_buf_map[port]}}]++;
            }
''')
            code.dedent()
            code('')

        code.dedent()
        code.dedent()
        code('''
        // If we got this far, we have nothing left todo or something went
        // wrong''')
        for buf_name, ports in in_msg_bufs.items():
            if len(ports) > 1:
                # only produce checks when a buffer is shared by multiple ports
                code('''
        if (${{buf_name}}->isReady(clockEdge()) && rejected[${{port_to_buf_map[ports[0]]}}] == ${{len(ports)}})
        {
            // no port claimed the message on the top of this buffer
            panic("Runtime Error at Ruby Time: %d. "
                  "All ports rejected a message. "
                  "You are probably sending a message type to this controller "
                  "over a virtual network that do not define an in_port for "
                  "the incoming message type.\\n",
                  Cycles(1));
        }
''')
        code('''
        break;
    }
}

} // namespace ruby
} // namespace gem5
''')

        code.write(path, "%s_Wakeup.cc" % self.ident)

    def printCSwitch(self, path):
        '''Output switch statement for transition table'''

        code = self.symtab.codeFormatter()
        ident = self.ident

        code('''
// ${ident}: ${{self.short}}

#include <cassert>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/ProtocolTrace.hh"
#include "debug/RubyGenerated.hh"
#include "mem/ruby/protocol/${ident}_Controller.hh"
#include "mem/ruby/protocol/${ident}_Event.hh"
#include "mem/ruby/protocol/${ident}_State.hh"
#include "mem/ruby/protocol/Types.hh"
#include "mem/ruby/system/RubySystem.hh"

#define HASH_FUN(state, event)  ((int(state)*${ident}_Event_NUM)+int(event))

#define GET_TRANSITION_COMMENT() (${ident}_transitionComment.str())
#define CLEAR_TRANSITION_COMMENT() (${ident}_transitionComment.str(""))

namespace gem5
{

namespace ruby
{

TransitionResult
${ident}_Controller::doTransition(${ident}_Event event,
''')
        if self.EntryType != None:
            code('''
                                  ${{self.EntryType.c_ident}}* m_cache_entry_ptr,
''')
        if self.TBEType != None:
            code('''
                                  ${{self.TBEType.c_ident}}* m_tbe_ptr,
''')
        code('''
                                  Addr addr)
{
''')
        code.indent()

        if self.TBEType != None and self.EntryType != None:
            code('${ident}_State state = getState(m_tbe_ptr, m_cache_entry_ptr, addr);')
        elif self.TBEType != None:
            code('${ident}_State state = getState(m_tbe_ptr, addr);')
        elif self.EntryType != None:
            code('${ident}_State state = getState(m_cache_entry_ptr, addr);')
        else:
            code('${ident}_State state = getState(addr);')

        code('''
${ident}_State next_state = state;

DPRINTF(RubyGenerated, "%s, Time: %lld, state: %s, event: %s, addr: %#x\\n",
        *this, curCycle(), ${ident}_State_to_string(state),
        ${ident}_Event_to_string(event), addr);

TransitionResult result =
''')
        if self.TBEType != None and self.EntryType != None:
            code('doTransitionWorker(event, state, next_state, m_tbe_ptr, m_cache_entry_ptr, addr);')
        elif self.TBEType != None:
            code('doTransitionWorker(event, state, next_state, m_tbe_ptr, addr);')
        elif self.EntryType != None:
            code('doTransitionWorker(event, state, next_state, m_cache_entry_ptr, addr);')
        else:
            code('doTransitionWorker(event, state, next_state, addr);')

        port_to_buf_map, in_msg_bufs, msg_bufs = self.getBufferMaps(ident)

        code('''

if (result == TransitionResult_Valid) {
    DPRINTF(RubyGenerated, "next_state: %s\\n",
            ${ident}_State_to_string(next_state));
    countTransition(state, event);

    DPRINTFR(ProtocolTrace, "%15d %3s %10s%20s %6s>%-6s %#x %s\\n",
             curTick(), m_version, "${ident}",
             ${ident}_Event_to_string(event),
             ${ident}_State_to_string(state),
             ${ident}_State_to_string(next_state),
             printAddress(addr), GET_TRANSITION_COMMENT());

    CLEAR_TRANSITION_COMMENT();
''')
        if self.TBEType != None and self.EntryType != None:
            code('setState(m_tbe_ptr, m_cache_entry_ptr, addr, next_state);')
            code('setAccessPermission(m_cache_entry_ptr, addr, next_state);')
        elif self.TBEType != None:
            code('setState(m_tbe_ptr, addr, next_state);')
            code('setAccessPermission(addr, next_state);')
        elif self.EntryType != None:
            code('setState(m_cache_entry_ptr, addr, next_state);')
            code('setAccessPermission(m_cache_entry_ptr, addr, next_state);')
        else:
            code('setState(addr, next_state);')
            code('setAccessPermission(addr, next_state);')

        code('''
} else if (result == TransitionResult_ResourceStall) {
    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %#x %s\\n",
             curTick(), m_version, "${ident}",
             ${ident}_Event_to_string(event),
             ${ident}_State_to_string(state),
             ${ident}_State_to_string(next_state),
             printAddress(addr), "Resource Stall");
} else if (result == TransitionResult_ProtocolStall) {
    DPRINTF(RubyGenerated, "stalling\\n");
    DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %#x %s\\n",
             curTick(), m_version, "${ident}",
             ${ident}_Event_to_string(event),
             ${ident}_State_to_string(state),
             ${ident}_State_to_string(next_state),
             printAddress(addr), "Protocol Stall");
}

return result;
''')
        code.dedent()
        code('''
}

TransitionResult
${ident}_Controller::doTransitionWorker(${ident}_Event event,
                                        ${ident}_State state,
                                        ${ident}_State& next_state,
''')

        if self.TBEType != None:
            code('''
                                        ${{self.TBEType.c_ident}}*& m_tbe_ptr,
''')
        if self.EntryType != None:
                  code('''
                                        ${{self.EntryType.c_ident}}*& m_cache_entry_ptr,
''')
        code('''
                                        Addr addr)
{
    m_curTransitionEvent = event;
    m_curTransitionNextState = next_state;
    switch(HASH_FUN(state, event)) {
''')

        # This map will allow suppress generating duplicate code
        cases = OrderedDict()

        for trans in self.transitions:
            case_string = "%s_State_%s, %s_Event_%s" % \
                (self.ident, trans.state.ident, self.ident, trans.event.ident)

            case = self.symtab.codeFormatter()
            # Only set next_state if it changes
            if trans.state != trans.nextState:
                if trans.nextState.isWildcard():
                    # When * is encountered as an end state of a transition,
                    # the next state is determined by calling the
                    # machine-specific getNextState function. The next state
                    # is determined before any actions of the transition
                    # execute, and therefore the next state calculation cannot
                    # depend on any of the transitionactions.
                    case('next_state = getNextState(addr); '
                         'm_curTransitionNextState = next_state;')
                else:
                    ns_ident = trans.nextState.ident
                    case('next_state = ${ident}_State_${ns_ident}; '
                         'm_curTransitionNextState = next_state;')

            actions = trans.actions
            request_types = trans.request_types

            # Check for resources
            case_sorter = []
            res = trans.resources
            for key,val in res.items():
                val = '''
if (!%s.areNSlotsAvailable(%s, clockEdge()))
    return TransitionResult_ResourceStall;
''' % (key.code, val)
                case_sorter.append(val)

            # Check all of the request_types for resource constraints
            for request_type in request_types:
                val = '''
if (!checkResourceAvailable(%s_RequestType_%s, addr)) {
    return TransitionResult_ResourceStall;
}
''' % (self.ident, request_type.ident)
                case_sorter.append(val)

            # Emit the code sequences in a sorted order.  This makes the
            # output deterministic (without this the output order can vary
            # since Map's keys() on a vector of pointers is not deterministic
            for c in sorted(case_sorter):
                case("$c")

            # Record access types for this transition
            for request_type in request_types:
                case('recordRequestType(${ident}_RequestType_${{request_type.ident}}, addr);')

            # Figure out if we stall
            stall = False
            for action in actions:
                if action.ident == "z_stall":
                    stall = True
                    break

            if stall:
                case('return TransitionResult_ProtocolStall;')
            else:
                if self.TBEType != None and self.EntryType != None:
                    for action in actions:
                        case('${{action.ident}}(m_tbe_ptr, m_cache_entry_ptr, addr);')
                elif self.TBEType != None:
                    for action in actions:
                        case('${{action.ident}}(m_tbe_ptr, addr);')
                elif self.EntryType != None:
                    for action in actions:
                        case('${{action.ident}}(m_cache_entry_ptr, addr);')
                else:
                    for action in actions:
                        case('${{action.ident}}(addr);')
                case('return TransitionResult_Valid;')

            case = str(case)

            # Look to see if this transition code is unique.
            if case not in cases:
                cases[case] = []

            cases[case].append(case_string)

        # Walk through all of the unique code blocks and spit out the
        # corresponding case statement elements
        for case,transitions in cases.items():
            # Iterative over all the multiple transitions that share
            # the same code
            for trans in transitions:
                code('  case HASH_FUN($trans):')
            code('    $case\n')

        code('''
      default:
        panic("Invalid transition\\n"
              "%s time: %d addr: %#x event: %s state: %s\\n",
              name(), curCycle(), addr, event, state);
    }

    return TransitionResult_Valid;
}

} // namespace ruby
} // namespace gem5
''')
        code.write(path, "%s_Transitions.cc" % self.ident)


    # **************************
    # ******* HTML Files *******
    # **************************
    def frameRef(self, click_href, click_target, over_href, over_num, text):
        code = self.symtab.codeFormatter(fix_newlines=False)
        code("""<A href=\"$click_href\" target=\"$click_target\" onmouseover=\"
    if (parent.frames[$over_num].location != parent.location + '$over_href') {
        parent.frames[$over_num].location='$over_href'
    }\">
    ${{html.formatShorthand(text)}}
    </A>""")
        return str(code)

    def writeHTMLFiles(self, path):
        # Create table with no row hilighted
        self.printHTMLTransitions(path, None)

        # Generate transition tables
        for state in self.states.values():
            self.printHTMLTransitions(path, state)

        # Generate action descriptions
        for action in self.actions.values():
            name = "%s_action_%s.html" % (self.ident, action.ident)
            code = html.createSymbol(action, "Action")
            code.write(path, name)

        # Generate state descriptions
        for state in self.states.values():
            name = "%s_State_%s.html" % (self.ident, state.ident)
            code = html.createSymbol(state, "State")
            code.write(path, name)

        # Generate event descriptions
        for event in self.events.values():
            name = "%s_Event_%s.html" % (self.ident, event.ident)
            code = html.createSymbol(event, "Event")
            code.write(path, name)

    def printHTMLTransitions(self, path, active_state):
        code = self.symtab.codeFormatter()

        code('''
<HTML>
<BODY link="blue" vlink="blue">

<H1 align="center">${{html.formatShorthand(self.short)}}:
''')
        code.indent()
        for i,machine in enumerate(self.symtab.getAllType(StateMachine)):
            mid = machine.ident
            if i != 0:
                extra = " - "
            else:
                extra = ""
            if machine == self:
                code('$extra$mid')
            else:
                code('$extra<A target="Table" href="${mid}_table.html">$mid</A>')
        code.dedent()

        code("""
</H1>

<TABLE border=1>
<TR>
  <TH> </TH>
""")

        for event in self.events.values():
            href = "%s_Event_%s.html" % (self.ident, event.ident)
            ref = self.frameRef(href, "Status", href, "1", event.short)
            code('<TH bgcolor=white>$ref</TH>')

        code('</TR>')
        # -- Body of table
        for state in self.states.values():
            # -- Each row
            if state == active_state:
                color = "yellow"
            else:
                color = "white"

            click = "%s_table_%s.html" % (self.ident, state.ident)
            over = "%s_State_%s.html" % (self.ident, state.ident)
            text = html.formatShorthand(state.short)
            ref = self.frameRef(click, "Table", over, "1", state.short)
            code('''
<TR>
  <TH bgcolor=$color>$ref</TH>
''')

            # -- One column for each event
            for event in self.events.values():
                trans = self.table.get((state,event), None)
                if trans is None:
                    # This is the no transition case
                    if state == active_state:
                        color = "#C0C000"
                    else:
                        color = "lightgrey"

                    code('<TD bgcolor=$color>&nbsp;</TD>')
                    continue

                next = trans.nextState
                stall_action = False

                # -- Get the actions
                for action in trans.actions:
                    if action.ident == "z_stall" or \
                       action.ident == "zz_recycleMandatoryQueue":
                        stall_action = True

                # -- Print out "actions/next-state"
                if stall_action:
                    if state == active_state:
                        color = "#C0C000"
                    else:
                        color = "lightgrey"

                elif active_state and next.ident == active_state.ident:
                    color = "aqua"
                elif state == active_state:
                    color = "yellow"
                else:
                    color = "white"

                code('<TD bgcolor=$color>')
                for action in trans.actions:
                    href = "%s_action_%s.html" % (self.ident, action.ident)
                    ref = self.frameRef(href, "Status", href, "1",
                                        action.short)
                    code('  $ref')
                if next != state:
                    if trans.actions:
                        code('/')
                    click = "%s_table_%s.html" % (self.ident, next.ident)
                    over = "%s_State_%s.html" % (self.ident, next.ident)
                    ref = self.frameRef(click, "Table", over, "1", next.short)
                    code("$ref")
                code("</TD>")

            # -- Each row
            if state == active_state:
                color = "yellow"
            else:
                color = "white"

            click = "%s_table_%s.html" % (self.ident, state.ident)
            over = "%s_State_%s.html" % (self.ident, state.ident)
            ref = self.frameRef(click, "Table", over, "1", state.short)
            code('''
  <TH bgcolor=$color>$ref</TH>
</TR>
''')
        code('''
<!- Column footer->
<TR>
  <TH> </TH>
''')

        for event in self.events.values():
            href = "%s_Event_%s.html" % (self.ident, event.ident)
            ref = self.frameRef(href, "Status", href, "1", event.short)
            code('<TH bgcolor=white>$ref</TH>')
        code('''
</TR>
</TABLE>
</BODY></HTML>
''')


        if active_state:
            name = "%s_table_%s.html" % (self.ident, active_state.ident)
        else:
            name = "%s_table.html" % self.ident
        code.write(path, name)

__all__ = [ "StateMachine" ]
