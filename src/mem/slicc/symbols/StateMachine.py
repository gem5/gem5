# Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
# Copyright (c) 2009 The Hewlett-Packard Development Company
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

from m5.util import orderdict

from slicc.symbols.Symbol import Symbol
from slicc.symbols.Var import Var
import slicc.generate.html as html
import re

python_class_map = {"int": "Int",
                    "std::string": "String",
                    "bool": "Bool",
                    "CacheMemory": "RubyCache",
                    "WireBuffer": "RubyWireBuffer",
                    "Sequencer": "RubySequencer",
                    "DirectoryMemory": "RubyDirectoryMemory",
                    "MemoryControl": "RubyMemoryControl",
                    "DMASequencer": "DMASequencer"
                    }

class StateMachine(Symbol):
    def __init__(self, symtab, ident, location, pairs, config_parameters):
        super(StateMachine, self).__init__(symtab, ident, location, pairs)
        self.table = None
        self.config_parameters = config_parameters

        for param in config_parameters:
            if param.pointer:
                var = Var(symtab, param.name, location, param.type_ast.type,
                          "(*m_%s_ptr)" % param.name, {}, self)
            else:
                var = Var(symtab, param.name, location, param.type_ast.type,
                          "m_%s" % param.name, {}, self)
            self.symtab.registerSym(param.name, var)

        self.states = orderdict()
        self.events = orderdict()
        self.actions = orderdict()
        self.transitions = []
        self.in_ports = []
        self.functions = []
        self.objects = []
        self.TBEType   = None
        self.EntryType = None

        self.message_buffer_names = []

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
        for other in self.actions.itervalues():
            if action.ident == other.ident:
                action.warning("Duplicate action definition: %s" % action.ident)
                action.error("Duplicate action definition: %s" % action.ident)
            if action.short == other.short:
                other.warning("Duplicate action shorthand: %s" % other.ident)
                other.warning("    shorthand = %s" % other.short)
                action.warning("Duplicate action shorthand: %s" % action.ident)
                action.error("    shorthand = %s" % action.short)

        self.actions[action.ident] = action

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
        self.objects.append(obj)

    def addType(self, type):
        type_ident = '%s' % type.c_ident

        if type_ident == "%s_TBE" %self.ident:
            if self.TBEType != None:
                self.error("Multiple Transaction Buffer types in a " \
                           "single machine.");
            self.TBEType = type

        elif "interface" in type and "AbstractCacheEntry" == type["interface"]:
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
        for action in self.actions.itervalues():
            if not action.used:
                error_msg = "Unused action: %s" % action.ident
                if "desc" in action:
                    error_msg += ", "  + action.desc
                action.warning(error_msg)
        self.table = table

    def writeCodeFiles(self, path):
        self.printControllerPython(path)
        self.printControllerHH(path)
        self.printControllerCC(path)
        self.printCSwitch(path)
        self.printCWakeup(path)
        self.printProfilerCC(path)
        self.printProfilerHH(path)
        self.printProfileDumperCC(path)
        self.printProfileDumperHH(path)

    def printControllerPython(self, path):
        code = self.symtab.codeFormatter()
        ident = self.ident
        py_ident = "%s_Controller" % ident
        c_ident = "%s_Controller" % self.ident
        code('''
from m5.params import *
from m5.SimObject import SimObject
from Controller import RubyController

class $py_ident(RubyController):
    type = '$py_ident'
''')
        code.indent()
        for param in self.config_parameters:
            dflt_str = ''
            if param.default is not None:
                dflt_str = str(param.default) + ', '
            if python_class_map.has_key(param.type_ast.type.c_ident):
                python_type = python_class_map[param.type_ast.type.c_ident]
                code('${{param.name}} = Param.${{python_type}}(${dflt_str}"")')
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

        self.message_buffer_names = []

        code('''
/** \\file $c_ident.hh
 *
 * Auto generated C++ code started by $__file__:$__line__
 * Created by slicc definition of Module "${{self.short}}"
 */

#ifndef __${ident}_CONTROLLER_HH__
#define __${ident}_CONTROLLER_HH__

#include <iostream>
#include <sstream>
#include <string>

#include "mem/protocol/${ident}_ProfileDumper.hh"
#include "mem/protocol/${ident}_Profiler.hh"
#include "mem/protocol/TransitionResult.hh"
#include "mem/protocol/Types.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "params/$c_ident.hh"
''')

        seen_types = set()
        for var in self.objects:
            if var.type.ident not in seen_types and not var.type.isPrimitive:
                code('#include "mem/protocol/${{var.type.c_ident}}.hh"')
            seen_types.add(var.type.ident)

        # for adding information to the protocol debug trace
        code('''
extern std::stringstream ${ident}_transitionComment;

class $c_ident : public AbstractController
{
// the coherence checker needs to call isBlockExclusive() and isBlockShared()
// making the Chip a friend class is an easy way to do this for now

public:
    typedef ${c_ident}Params Params;
    $c_ident(const Params *p);
    static int getNumControllers();
    void init();
    MessageBuffer* getMandatoryQueue() const;
    const int & getVersion() const;
    const std::string toString() const;
    const std::string getName() const;
    void stallBuffer(MessageBuffer* buf, Address addr);
    void wakeUpBuffers(Address addr);
    void wakeUpAllBuffers();
    void initNetworkPtr(Network* net_ptr) { m_net_ptr = net_ptr; }
    void print(std::ostream& out) const;
    void printConfig(std::ostream& out) const;
    void wakeup();
    void printStats(std::ostream& out) const;
    void clearStats();
    void blockOnQueue(Address addr, MessageBuffer* port);
    void unblock(Address addr);
    void recordCacheTrace(int cntrl, CacheRecorder* tr);
    Sequencer* getSequencer() const;

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
int m_number_of_TBEs;

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
                              const Address& addr);

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
                                    const Address& addr);

std::string m_name;
int m_transitions_per_cycle;
int m_buffer_size;
int m_recycle_latency;
std::map<std::string, std::string> m_cfg;
NodeID m_version;
Network* m_net_ptr;
MachineID m_machineID;
bool m_is_blocking;
std::map<Address, MessageBuffer*> m_block_map;
typedef std::vector<MessageBuffer*> MsgVecType;
typedef std::map< Address, MsgVecType* > WaitingBufType;
WaitingBufType m_waiting_buffers;
int m_max_in_port_rank;
int m_cur_in_port_rank;
static ${ident}_ProfileDumper s_profileDumper;
${ident}_Profiler m_profiler;
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

        code('''

// Actions
''')
        if self.TBEType != None and self.EntryType != None:
            for action in self.actions.itervalues():
                code('/** \\brief ${{action.desc}} */')
                code('void ${{action.ident}}(${{self.TBEType.c_ident}}*& m_tbe_ptr, ${{self.EntryType.c_ident}}*& m_cache_entry_ptr, const Address& addr);')
        elif self.TBEType != None:
            for action in self.actions.itervalues():
                code('/** \\brief ${{action.desc}} */')
                code('void ${{action.ident}}(${{self.TBEType.c_ident}}*& m_tbe_ptr, const Address& addr);')
        elif self.EntryType != None:
            for action in self.actions.itervalues():
                code('/** \\brief ${{action.desc}} */')
                code('void ${{action.ident}}(${{self.EntryType.c_ident}}*& m_cache_entry_ptr, const Address& addr);')
        else:
            for action in self.actions.itervalues():
                code('/** \\brief ${{action.desc}} */')
                code('void ${{action.ident}}(const Address& addr);')

        # the controller internal variables
        code('''

// Objects
''')
        for var in self.objects:
            th = var.get("template_hack", "")
            code('${{var.type.c_ident}}$th* m_${{var.c_ident}}_ptr;')

            if var.type.ident == "MessageBuffer":
                self.message_buffer_names.append("m_%s_ptr" % var.c_ident)

        code.dedent()
        code('};')
        code('#endif // __${ident}_CONTROLLER_H__')
        code.write(path, '%s.hh' % c_ident)

    def printControllerCC(self, path):
        '''Output the actions for performing the actions'''

        code = self.symtab.codeFormatter()
        ident = self.ident
        c_ident = "%s_Controller" % self.ident

        code('''
/** \\file $c_ident.cc
 *
 * Auto generated C++ code started by $__file__:$__line__
 * Created by slicc definition of Module "${{self.short}}"
 */

#include <sys/types.h>
#include <unistd.h>

#include <cassert>
#include <sstream>
#include <string>

#include "base/compiler.hh"
#include "base/cprintf.hh"
#include "debug/RubyGenerated.hh"
#include "debug/RubySlicc.hh"
#include "mem/protocol/${ident}_Controller.hh"
#include "mem/protocol/${ident}_Event.hh"
#include "mem/protocol/${ident}_State.hh"
#include "mem/protocol/Types.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/slicc_interface/RubySlicc_includes.hh"
#include "mem/ruby/system/System.hh"

using namespace std;
''')

        # include object classes
        seen_types = set()
        for var in self.objects:
            if var.type.ident not in seen_types and not var.type.isPrimitive:
                code('#include "mem/protocol/${{var.type.c_ident}}.hh"')
            seen_types.add(var.type.ident)

        code('''
$c_ident *
${c_ident}Params::create()
{
    return new $c_ident(this);
}

int $c_ident::m_num_controllers = 0;
${ident}_ProfileDumper $c_ident::s_profileDumper;

// for adding information to the protocol debug trace
stringstream ${ident}_transitionComment;
#define APPEND_TRANSITION_COMMENT(str) (${ident}_transitionComment << str)

/** \\brief constructor */
$c_ident::$c_ident(const Params *p)
    : AbstractController(p)
{
    m_version = p->version;
    m_transitions_per_cycle = p->transitions_per_cycle;
    m_buffer_size = p->buffer_size;
    m_recycle_latency = p->recycle_latency;
    m_number_of_TBEs = p->number_of_TBEs;
    m_is_blocking = false;
    m_name = "${ident}";
''')
        #
        # max_port_rank is used to size vectors and thus should be one plus the
        # largest port rank
        #
        max_port_rank = self.in_ports[0].pairs["max_port_rank"] + 1
        code('    m_max_in_port_rank = $max_port_rank;')
        code.indent()

        #
        # After initializing the universal machine parameters, initialize the
        # this machines config parameters.  Also detemine if these configuration
        # params include a sequencer.  This information will be used later for
        # contecting the sequencer back to the L1 cache controller.
        #
        contains_dma_sequencer = False
        sequencers = []
        for param in self.config_parameters:
            if param.name == "dma_sequencer":
                contains_dma_sequencer = True
            elif re.compile("sequencer").search(param.name):
                sequencers.append(param.name)
            if param.pointer:
                code('m_${{param.name}}_ptr = p->${{param.name}};')
            else:
                code('m_${{param.name}} = p->${{param.name}};')

        #
        # For the l1 cache controller, add the special atomic support which 
        # includes passing the sequencer a pointer to the controller.
        #
        if self.ident == "L1Cache":
            if not sequencers:
                self.error("The L1Cache controller must include the sequencer " \
                           "configuration parameter")

            for seq in sequencers:
                code('''
m_${{seq}}_ptr->setController(this);
    ''')

        else:
            for seq in sequencers:
                code('''
m_${{seq}}_ptr->setController(this);
    ''')

        #
        # For the DMA controller, pass the sequencer a pointer to the
        # controller.
        #
        if self.ident == "DMA":
            if not contains_dma_sequencer:
                self.error("The DMA controller must include the sequencer " \
                           "configuration parameter")

            code('''
m_dma_sequencer_ptr->setController(this);
''')
            
        code('m_num_controllers++;')
        for var in self.objects:
            if var.ident.find("mandatoryQueue") >= 0:
                code('m_${{var.c_ident}}_ptr = new ${{var.type.c_ident}}();')

        code.dedent()
        code('''
}

void
$c_ident::init()
{
    MachineType machine_type;
    int base;

    m_machineID.type = MachineType_${ident};
    m_machineID.num = m_version;

    // initialize objects
    m_profiler.setVersion(m_version);
    s_profileDumper.registerProfiler(&m_profiler);

''')

        code.indent()
        for var in self.objects:
            vtype = var.type
            vid = "m_%s_ptr" % var.c_ident
            if "network" not in var:
                # Not a network port object
                if "primitive" in vtype:
                    code('$vid = new ${{vtype.c_ident}};')
                    if "default" in var:
                        code('(*$vid) = ${{var["default"]}};')
                else:
                    # Normal Object
                    # added by SS
                    if "factory" in var:
                        code('$vid = ${{var["factory"]}};')
                    elif var.ident.find("mandatoryQueue") < 0:
                        th = var.get("template_hack", "")
                        expr = "%s  = new %s%s" % (vid, vtype.c_ident, th)

                        args = ""
                        if "non_obj" not in vtype and not vtype.isEnumeration:
                            if expr.find("TBETable") >= 0:
                                args = "m_number_of_TBEs"
                            else:
                                args = var.get("constructor_hack", "")

                        code('$expr($args);')

                    code('assert($vid != NULL);')

                    if "default" in var:
                        code('*$vid = ${{var["default"]}}; // Object default')
                    elif "default" in vtype:
                        comment = "Type %s default" % vtype.ident
                        code('*$vid = ${{vtype["default"]}}; // $comment')

                    # Set ordering
                    if "ordered" in var and "trigger_queue" not in var:
                        # A buffer
                        code('$vid->setOrdering(${{var["ordered"]}});')

                    # Set randomization
                    if "random" in var:
                        # A buffer
                        code('$vid->setRandomization(${{var["random"]}});')

                    # Set Priority
                    if vtype.isBuffer and \
                           "rank" in var and "trigger_queue" not in var:
                        code('$vid->setPriority(${{var["rank"]}});')

            else:
                # Network port object
                network = var["network"]
                ordered =  var["ordered"]
                vnet = var["virtual_network"]
                vnet_type = var["vnet_type"]

                assert var.machine is not None
                code('''
machine_type = string_to_MachineType("${{var.machine.ident}}");
base = MachineType_base_number(machine_type);
$vid = m_net_ptr->get${network}NetQueue(m_version + base, $ordered, $vnet, "$vnet_type");
''')

                code('assert($vid != NULL);')

                # Set ordering
                if "ordered" in var:
                    # A buffer
                    code('$vid->setOrdering(${{var["ordered"]}});')

                # Set randomization
                if "random" in var:
                    # A buffer
                    code('$vid->setRandomization(${{var["random"]}});')

                # Set Priority
                if "rank" in var:
                    code('$vid->setPriority(${{var["rank"]}})')

                # Set buffer size
                if vtype.isBuffer:
                    code('''
if (m_buffer_size > 0) {
    $vid->resize(m_buffer_size);
}
''')

                # set description (may be overriden later by port def)
                code('''
$vid->setDescription("[Version " + to_string(m_version) + ", ${ident}, name=${{var.c_ident}}]");

''')

            if vtype.isBuffer:
                if "recycle_latency" in var:
                    code('$vid->setRecycleLatency(${{var["recycle_latency"]}});')
                else:
                    code('$vid->setRecycleLatency(m_recycle_latency);')


        # Set the queue consumers
        code()
        for port in self.in_ports:
            code('${{port.code}}.setConsumer(this);')

        # Set the queue descriptions
        code()
        for port in self.in_ports:
            code('${{port.code}}.setDescription("[Version " + to_string(m_version) + ", $ident, $port]");')

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
                code('m_profiler.possibleTransition($state, $event);')

        code.dedent()
        code('}')

        has_mandatory_q = False
        for port in self.in_ports:
            if port.code.find("mandatoryQueue_ptr") >= 0:
                has_mandatory_q = True

        if has_mandatory_q:
            mq_ident = "m_%s_mandatoryQueue_ptr" % self.ident
        else:
            mq_ident = "NULL"

        seq_ident = "NULL"
        for param in self.config_parameters:
            if param.name == "sequencer":
                assert(param.pointer)
                seq_ident = "m_%s_ptr" % param.name

        code('''
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

Sequencer*
$c_ident::getSequencer() const
{
    return $seq_ident;
}

const int &
$c_ident::getVersion() const
{
    return m_version;
}

const string
$c_ident::toString() const
{
    return "$c_ident";
}

const string
$c_ident::getName() const
{
    return m_name;
}

void
$c_ident::stallBuffer(MessageBuffer* buf, Address addr)
{
    if (m_waiting_buffers.count(addr) == 0) {
        MsgVecType* msgVec = new MsgVecType;
        msgVec->resize(m_max_in_port_rank, NULL);
        m_waiting_buffers[addr] = msgVec;
    }
    (*(m_waiting_buffers[addr]))[m_cur_in_port_rank] = buf;
}

void
$c_ident::wakeUpBuffers(Address addr)
{
    if (m_waiting_buffers.count(addr) > 0) {
        //
        // Wake up all possible lower rank (i.e. lower priority) buffers that could
        // be waiting on this message.
        //
        for (int in_port_rank = m_cur_in_port_rank - 1;
             in_port_rank >= 0;
             in_port_rank--) {
            if ((*(m_waiting_buffers[addr]))[in_port_rank] != NULL) {
                (*(m_waiting_buffers[addr]))[in_port_rank]->reanalyzeMessages(addr);
            }
        }
        delete m_waiting_buffers[addr];
        m_waiting_buffers.erase(addr);
    }
}

void
$c_ident::wakeUpAllBuffers()
{
    //
    // Wake up all possible buffers that could be waiting on any message.
    //

    std::vector<MsgVecType*> wokeUpMsgVecs;
    
    if(m_waiting_buffers.size() > 0) {
        for (WaitingBufType::iterator buf_iter = m_waiting_buffers.begin();
             buf_iter != m_waiting_buffers.end();
             ++buf_iter) {
             for (MsgVecType::iterator vec_iter = buf_iter->second->begin();
                  vec_iter != buf_iter->second->end();
                  ++vec_iter) {
                  if (*vec_iter != NULL) {
                      (*vec_iter)->reanalyzeAllMessages();
                  }
             }
             wokeUpMsgVecs.push_back(buf_iter->second);
        }

        for (std::vector<MsgVecType*>::iterator wb_iter = wokeUpMsgVecs.begin();
             wb_iter != wokeUpMsgVecs.end();
             ++wb_iter) {
             delete (*wb_iter);
        }

        m_waiting_buffers.clear();
    }
}

void
$c_ident::blockOnQueue(Address addr, MessageBuffer* port)
{
    m_is_blocking = true;
    m_block_map[addr] = port;
}

void
$c_ident::unblock(Address addr)
{
    m_block_map.erase(addr);
    if (m_block_map.size() == 0) {
       m_is_blocking = false;
    }
}

void
$c_ident::print(ostream& out) const
{
    out << "[$c_ident " << m_version << "]";
}

void
$c_ident::printConfig(ostream& out) const
{
    out << "$c_ident config: " << m_name << endl;
    out << "  version: " << m_version << endl;
    map<string, string>::const_iterator it;
    for (it = m_cfg.begin(); it != m_cfg.end(); it++)
        out << "  " << it->first << ": " << it->second << endl;
}

void
$c_ident::printStats(ostream& out) const
{
''')
        #
        # Cache and Memory Controllers have specific profilers associated with
        # them.  Print out these stats before dumping state transition stats.
        #
        for param in self.config_parameters:
            if param.type_ast.type.ident == "CacheMemory" or \
               param.type_ast.type.ident == "DirectoryMemory" or \
                   param.type_ast.type.ident == "MemoryControl":
                assert(param.pointer)
                code('    m_${{param.ident}}_ptr->printStats(out);')

        code('''
    if (m_version == 0) {
        s_profileDumper.dumpStats(out);
    }
}

void $c_ident::clearStats() {
''')
        #
        # Cache and Memory Controllers have specific profilers associated with
        # them.  These stats must be cleared too.
        #
        for param in self.config_parameters:
            if param.type_ast.type.ident == "CacheMemory" or \
                   param.type_ast.type.ident == "MemoryControl":
                assert(param.pointer)
                code('    m_${{param.ident}}_ptr->clearStats();')

        code('''
    m_profiler.clearStats();
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
            for action in self.actions.itervalues():
                if "c_code" not in action:
                 continue

                code('''
/** \\brief ${{action.desc}} */
void
$c_ident::${{action.ident}}(${{self.TBEType.c_ident}}*& m_tbe_ptr, ${{self.EntryType.c_ident}}*& m_cache_entry_ptr, const Address& addr)
{
    DPRINTF(RubyGenerated, "executing ${{action.ident}}\\n");
    ${{action["c_code"]}}
}

''')
        elif self.TBEType != None:
            for action in self.actions.itervalues():
                if "c_code" not in action:
                 continue

                code('''
/** \\brief ${{action.desc}} */
void
$c_ident::${{action.ident}}(${{self.TBEType.c_ident}}*& m_tbe_ptr, const Address& addr)
{
    DPRINTF(RubyGenerated, "executing ${{action.ident}}\\n");
    ${{action["c_code"]}}
}

''')
        elif self.EntryType != None:
            for action in self.actions.itervalues():
                if "c_code" not in action:
                 continue

                code('''
/** \\brief ${{action.desc}} */
void
$c_ident::${{action.ident}}(${{self.EntryType.c_ident}}*& m_cache_entry_ptr, const Address& addr)
{
    DPRINTF(RubyGenerated, "executing ${{action.ident}}\\n");
    ${{action["c_code"]}}
}

''')
        else:
            for action in self.actions.itervalues():
                if "c_code" not in action:
                 continue

                code('''
/** \\brief ${{action.desc}} */
void
$c_ident::${{action.ident}}(const Address& addr)
{
    DPRINTF(RubyGenerated, "executing ${{action.ident}}\\n");
    ${{action["c_code"]}}
}

''')
        for func in self.functions:
            code(func.generateCode())

        code.write(path, "%s.cc" % c_ident)

    def printCWakeup(self, path):
        '''Output the wakeup loop for the events'''

        code = self.symtab.codeFormatter()
        ident = self.ident

        code('''
// Auto generated C++ code started by $__file__:$__line__
// ${ident}: ${{self.short}}

#include <sys/types.h>
#include <unistd.h>

#include <cassert>

#include "base/misc.hh"
#include "debug/RubySlicc.hh"
#include "mem/protocol/${ident}_Controller.hh"
#include "mem/protocol/${ident}_Event.hh"
#include "mem/protocol/${ident}_State.hh"
#include "mem/protocol/Types.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/slicc_interface/RubySlicc_includes.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

void
${ident}_Controller::wakeup()
{
    int counter = 0;
    while (true) {
        // Some cases will put us into an infinite loop without this limit
        assert(counter <= m_transitions_per_cycle);
        if (counter == m_transitions_per_cycle) {
            // Count how often we are fully utilized
            g_system_ptr->getProfiler()->controllerBusy(m_machineID);

            // Wakeup in another cycle and try again
            g_eventQueue_ptr->scheduleEvent(this, 1);
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
            if port.pairs.has_key("rank"):
                code('m_cur_in_port_rank = ${{port.pairs["rank"]}};')
            else:
                code('m_cur_in_port_rank = 0;')
            code('${{port["c_code_in_port"]}}')
            code.dedent()

            code('')

        code.dedent()
        code.dedent()
        code('''
        break;  // If we got this far, we have nothing left todo
    }
    // g_eventQueue_ptr->scheduleEvent(this, 1);
}
''')

        code.write(path, "%s_Wakeup.cc" % self.ident)

    def printCSwitch(self, path):
        '''Output switch statement for transition table'''

        code = self.symtab.codeFormatter()
        ident = self.ident

        code('''
// Auto generated C++ code started by $__file__:$__line__
// ${ident}: ${{self.short}}

#include <cassert>

#include "base/misc.hh"
#include "base/trace.hh"
#include "debug/ProtocolTrace.hh"
#include "debug/RubyGenerated.hh"
#include "mem/protocol/${ident}_Controller.hh"
#include "mem/protocol/${ident}_Event.hh"
#include "mem/protocol/${ident}_State.hh"
#include "mem/protocol/Types.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/System.hh"

#define HASH_FUN(state, event)  ((int(state)*${ident}_Event_NUM)+int(event))

#define GET_TRANSITION_COMMENT() (${ident}_transitionComment.str())
#define CLEAR_TRANSITION_COMMENT() (${ident}_transitionComment.str(""))

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
                                  const Address &addr)
{
''')
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

    DPRINTF(RubyGenerated, "%s, Time: %lld, state: %s, event: %s, addr: %s\\n",
            *this,
            g_eventQueue_ptr->getTime(),
            ${ident}_State_to_string(state),
            ${ident}_Event_to_string(event),
            addr);

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

        code('''
    if (result == TransitionResult_Valid) {
        DPRINTF(RubyGenerated, "next_state: %s\\n",
                ${ident}_State_to_string(next_state));
        m_profiler.countTransition(state, event);
        DPRINTFR(ProtocolTrace, "%15d %3s %10s%20s %6s>%-6s %s %s\\n",
                 curTick(), m_version, "${ident}",
                 ${ident}_Event_to_string(event),
                 ${ident}_State_to_string(state),
                 ${ident}_State_to_string(next_state),
                 addr, GET_TRANSITION_COMMENT());

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
        DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %s %s\\n",
                 curTick(), m_version, "${ident}",
                 ${ident}_Event_to_string(event),
                 ${ident}_State_to_string(state),
                 ${ident}_State_to_string(next_state),
                 addr, "Resource Stall");
    } else if (result == TransitionResult_ProtocolStall) {
        DPRINTF(RubyGenerated, "stalling\\n");
        DPRINTFR(ProtocolTrace, "%15s %3s %10s%20s %6s>%-6s %s %s\\n",
                 curTick(), m_version, "${ident}",
                 ${ident}_Event_to_string(event),
                 ${ident}_State_to_string(state),
                 ${ident}_State_to_string(next_state),
                 addr, "Protocol Stall");
    }

    return result;
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
                                        const Address& addr)
{
    switch(HASH_FUN(state, event)) {
''')

        # This map will allow suppress generating duplicate code
        cases = orderdict()

        for trans in self.transitions:
            case_string = "%s_State_%s, %s_Event_%s" % \
                (self.ident, trans.state.ident, self.ident, trans.event.ident)

            case = self.symtab.codeFormatter()
            # Only set next_state if it changes
            if trans.state != trans.nextState:
                ns_ident = trans.nextState.ident
                case('next_state = ${ident}_State_${ns_ident};')

            actions = trans.actions

            # Check for resources
            case_sorter = []
            res = trans.resources
            for key,val in res.iteritems():
                if key.type.ident != "DNUCAStopTable":
                    val = '''
if (!%s.areNSlotsAvailable(%s))
    return TransitionResult_ResourceStall;
''' % (key.code, val)
                case_sorter.append(val)


            # Emit the code sequences in a sorted order.  This makes the
            # output deterministic (without this the output order can vary
            # since Map's keys() on a vector of pointers is not deterministic
            for c in sorted(case_sorter):
                case("$c")

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
        for case,transitions in cases.iteritems():
            # Iterative over all the multiple transitions that share
            # the same code
            for trans in transitions:
                code('  case HASH_FUN($trans):')
            code('    $case')

        code('''
      default:
        fatal("Invalid transition\\n"
              "%s time: %d addr: %s event: %s state: %s\\n",
              name(), g_eventQueue_ptr->getTime(), addr, event, state);
    }
    return TransitionResult_Valid;
}
''')
        code.write(path, "%s_Transitions.cc" % self.ident)

    def printProfileDumperHH(self, path):
        code = self.symtab.codeFormatter()
        ident = self.ident

        code('''
// Auto generated C++ code started by $__file__:$__line__
// ${ident}: ${{self.short}}

#ifndef __${ident}_PROFILE_DUMPER_HH__
#define __${ident}_PROFILE_DUMPER_HH__

#include <cassert>
#include <iostream>
#include <vector>

#include "${ident}_Event.hh"
#include "${ident}_Profiler.hh"

typedef std::vector<${ident}_Profiler *> ${ident}_profilers;

class ${ident}_ProfileDumper
{
  public:
    ${ident}_ProfileDumper();
    void registerProfiler(${ident}_Profiler* profiler);
    void dumpStats(std::ostream& out) const;

  private:
    ${ident}_profilers m_profilers;
};

#endif // __${ident}_PROFILE_DUMPER_HH__
''')
        code.write(path, "%s_ProfileDumper.hh" % self.ident)

    def printProfileDumperCC(self, path):
        code = self.symtab.codeFormatter()
        ident = self.ident

        code('''
// Auto generated C++ code started by $__file__:$__line__
// ${ident}: ${{self.short}}

#include "mem/protocol/${ident}_ProfileDumper.hh"

${ident}_ProfileDumper::${ident}_ProfileDumper()
{
}

void
${ident}_ProfileDumper::registerProfiler(${ident}_Profiler* profiler)
{
    m_profilers.push_back(profiler);
}

void
${ident}_ProfileDumper::dumpStats(std::ostream& out) const
{
    out << " --- ${ident} ---\\n";
    out << " - Event Counts -\\n";
    for (${ident}_Event event = ${ident}_Event_FIRST;
         event < ${ident}_Event_NUM;
         ++event) {
        out << (${ident}_Event) event << " [";
        uint64 total = 0;
        for (int i = 0; i < m_profilers.size(); i++) {
             out << m_profilers[i]->getEventCount(event) << " ";
             total += m_profilers[i]->getEventCount(event);
        }
        out << "] " << total << "\\n";
    }
    out << "\\n";
    out << " - Transitions -\\n";
    for (${ident}_State state = ${ident}_State_FIRST;
         state < ${ident}_State_NUM;
         ++state) {
        for (${ident}_Event event = ${ident}_Event_FIRST;
             event < ${ident}_Event_NUM;
             ++event) {
            if (m_profilers[0]->isPossible(state, event)) {
                out << (${ident}_State) state << "  "
                    << (${ident}_Event) event << " [";
                uint64 total = 0;
                for (int i = 0; i < m_profilers.size(); i++) {
                     out << m_profilers[i]->getTransitionCount(state, event) << " ";
                     total += m_profilers[i]->getTransitionCount(state, event);
                }
                out << "] " << total << "\\n";
            }
        }
        out << "\\n";
    }
}
''')
        code.write(path, "%s_ProfileDumper.cc" % self.ident)

    def printProfilerHH(self, path):
        code = self.symtab.codeFormatter()
        ident = self.ident

        code('''
// Auto generated C++ code started by $__file__:$__line__
// ${ident}: ${{self.short}}

#ifndef __${ident}_PROFILER_HH__
#define __${ident}_PROFILER_HH__

#include <cassert>
#include <iostream>

#include "mem/protocol/${ident}_Event.hh"
#include "mem/protocol/${ident}_State.hh"
#include "mem/ruby/common/TypeDefines.hh"

class ${ident}_Profiler
{
  public:
    ${ident}_Profiler();
    void setVersion(int version);
    void countTransition(${ident}_State state, ${ident}_Event event);
    void possibleTransition(${ident}_State state, ${ident}_Event event);
    uint64 getEventCount(${ident}_Event event);
    bool isPossible(${ident}_State state, ${ident}_Event event);
    uint64 getTransitionCount(${ident}_State state, ${ident}_Event event);
    void clearStats();

  private:
    int m_counters[${ident}_State_NUM][${ident}_Event_NUM];
    int m_event_counters[${ident}_Event_NUM];
    bool m_possible[${ident}_State_NUM][${ident}_Event_NUM];
    int m_version;
};

#endif // __${ident}_PROFILER_HH__
''')
        code.write(path, "%s_Profiler.hh" % self.ident)

    def printProfilerCC(self, path):
        code = self.symtab.codeFormatter()
        ident = self.ident

        code('''
// Auto generated C++ code started by $__file__:$__line__
// ${ident}: ${{self.short}}

#include <cassert>

#include "mem/protocol/${ident}_Profiler.hh"

${ident}_Profiler::${ident}_Profiler()
{
    for (int state = 0; state < ${ident}_State_NUM; state++) {
        for (int event = 0; event < ${ident}_Event_NUM; event++) {
            m_possible[state][event] = false;
            m_counters[state][event] = 0;
        }
    }
    for (int event = 0; event < ${ident}_Event_NUM; event++) {
        m_event_counters[event] = 0;
    }
}

void
${ident}_Profiler::setVersion(int version)
{
    m_version = version;
}

void
${ident}_Profiler::clearStats()
{
    for (int state = 0; state < ${ident}_State_NUM; state++) {
        for (int event = 0; event < ${ident}_Event_NUM; event++) {
            m_counters[state][event] = 0;
        }
    }

    for (int event = 0; event < ${ident}_Event_NUM; event++) {
        m_event_counters[event] = 0;
    }
}
void
${ident}_Profiler::countTransition(${ident}_State state, ${ident}_Event event)
{
    assert(m_possible[state][event]);
    m_counters[state][event]++;
    m_event_counters[event]++;
}
void
${ident}_Profiler::possibleTransition(${ident}_State state,
                                      ${ident}_Event event)
{
    m_possible[state][event] = true;
}

uint64
${ident}_Profiler::getEventCount(${ident}_Event event)
{
    return m_event_counters[event];
}

bool
${ident}_Profiler::isPossible(${ident}_State state, ${ident}_Event event)
{
    return m_possible[state][event];
}

uint64
${ident}_Profiler::getTransitionCount(${ident}_State state,
                                      ${ident}_Event event)
{
    return m_counters[state][event];
}

''')
        code.write(path, "%s_Profiler.cc" % self.ident)

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
        for state in self.states.itervalues():
            self.printHTMLTransitions(path, state)

        # Generate action descriptions
        for action in self.actions.itervalues():
            name = "%s_action_%s.html" % (self.ident, action.ident)
            code = html.createSymbol(action, "Action")
            code.write(path, name)

        # Generate state descriptions
        for state in self.states.itervalues():
            name = "%s_State_%s.html" % (self.ident, state.ident)
            code = html.createSymbol(state, "State")
            code.write(path, name)

        # Generate event descriptions
        for event in self.events.itervalues():
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

        for event in self.events.itervalues():
            href = "%s_Event_%s.html" % (self.ident, event.ident)
            ref = self.frameRef(href, "Status", href, "1", event.short)
            code('<TH bgcolor=white>$ref</TH>')

        code('</TR>')
        # -- Body of table
        for state in self.states.itervalues():
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
            for event in self.events.itervalues():
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

        for event in self.events.itervalues():
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
