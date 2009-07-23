
/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

/*
 * $Id$
 *
 * */

#include "mem/slicc/symbols/StateMachine.hh"
#include "mem/slicc/generator/fileio.hh"
#include "mem/slicc/generator/html_gen.hh"
#include "mem/slicc/symbols/Action.hh"
#include "mem/slicc/symbols/Event.hh"
#include "mem/slicc/symbols/State.hh"
#include "mem/slicc/symbols/Transition.hh"
#include "mem/slicc/symbols/Var.hh"
#include "mem/slicc/symbols/SymbolTable.hh"
#include "mem/gems_common/util.hh"
#include "mem/gems_common/Vector.hh"

#include <set>

StateMachine::StateMachine(string ident, const Location& location, const Map<string, string>& pairs,  std::vector<std::string*>* latency_vector)
  : Symbol(ident, location, pairs)
{
  m_table_built = false;
  m_latency_vector = *latency_vector;
}

StateMachine::~StateMachine()
{
  // FIXME
  // assert(0);
}

void StateMachine::addState(State* state_ptr)
{
  assert(m_table_built == false);
  m_state_map.add(state_ptr, m_states.size());
  m_states.insertAtBottom(state_ptr);
}

void StateMachine::addEvent(Event* event_ptr)
{
  assert(m_table_built == false);
  m_event_map.add(event_ptr, m_events.size());
  m_events.insertAtBottom(event_ptr);
}

void StateMachine::addAction(Action* action_ptr)
{
  assert(m_table_built == false);

  // Check for duplicate action
  int size = m_actions.size();
  for(int i=0; i<size; i++) {
    if (m_actions[i]->getIdent() == action_ptr->getIdent()) {
      m_actions[i]->warning("Duplicate action definition: " + m_actions[i]->getIdent());
      action_ptr->error("Duplicate action definition: " + action_ptr->getIdent());
    }
    if (m_actions[i]->getShorthand() == action_ptr->getShorthand()) {
      m_actions[i]->warning("Duplicate action shorthand: " + m_actions[i]->getIdent());
      m_actions[i]->warning("    shorthand = " + m_actions[i]->getShorthand());
      action_ptr->warning("Duplicate action shorthand: " + action_ptr->getIdent());
      action_ptr->error("    shorthand = " + action_ptr->getShorthand());
    }
  }

  m_actions.insertAtBottom(action_ptr);
}

void StateMachine::addTransition(Transition* trans_ptr)
{
  assert(m_table_built == false);
  trans_ptr->checkIdents(m_states, m_events, m_actions);
  m_transitions.insertAtBottom(trans_ptr);
}

void StateMachine::addFunc(Func* func_ptr)
{
  // register func in the symbol table
  g_sym_table.registerSym(func_ptr->toString(), func_ptr);
  m_internal_func_vec.insertAtBottom(func_ptr);
}

void StateMachine::buildTable()
{
  assert(m_table_built == false);
  int numStates = m_states.size();
  int numEvents = m_events.size();
  int numTransitions = m_transitions.size();
  int stateIndex, eventIndex;

  for(stateIndex=0; stateIndex < numStates; stateIndex++) {
    m_table.insertAtBottom(Vector<Transition*>());
    for(eventIndex=0; eventIndex < numEvents; eventIndex++) {
      m_table[stateIndex].insertAtBottom(NULL);
    }
  }

  for(int i=0; i<numTransitions; i++) {
    Transition* trans_ptr = m_transitions[i];

    // Track which actions we touch so we know if we use them all --
    // really this should be done for all symbols as part of the
    // symbol table, then only trigger it for Actions, States, Events,
    // etc.

    Vector<Action*> actions = trans_ptr->getActions();
    for(int actionIndex=0; actionIndex < actions.size(); actionIndex++) {
      actions[actionIndex]->markUsed();
    }

    stateIndex = getStateIndex(trans_ptr->getStatePtr());
    eventIndex = getEventIndex(trans_ptr->getEventPtr());
    if (m_table[stateIndex][eventIndex] != NULL) {
      m_table[stateIndex][eventIndex]->warning("Duplicate transition: " + m_table[stateIndex][eventIndex]->toString());
      trans_ptr->error("Duplicate transition: " + trans_ptr->toString());
    }
    m_table[stateIndex][eventIndex] = trans_ptr;
  }

  // Look at all actions to make sure we used them all
  for(int actionIndex=0; actionIndex < m_actions.size(); actionIndex++) {
    Action* action_ptr = m_actions[actionIndex];
    if (!action_ptr->wasUsed()) {
      string error_msg = "Unused action: " +  action_ptr->getIdent();
      if (action_ptr->existPair("desc")) {
        error_msg += ", "  + action_ptr->getDescription();
      }
      action_ptr->warning(error_msg);
    }
  }

  m_table_built = true;
}

const Transition* StateMachine::getTransPtr(int stateIndex, int eventIndex) const
{
  return m_table[stateIndex][eventIndex];
}

// *********************** //
// ******* C Files ******* //
// *********************** //

void StateMachine::writeCFiles(string path)
{
  string comp = getIdent();
  string filename;

  // Output the method declarations for the class declaration
  {
    ostringstream sstr;
    printControllerH(sstr, comp);
    conditionally_write_file(path + comp + "_Controller.hh", sstr);
  }

  // Output switch statement for transition table
  {
    ostringstream sstr;
    printCSwitch(sstr, comp);
    conditionally_write_file(path + comp + "_Transitions.cc", sstr);
  }

  // Output the actions for performing the actions
  {
    ostringstream sstr;
    printControllerC(sstr, comp);
    conditionally_write_file(path + comp + "_Controller.cc", sstr);
  }

  // Output the wakeup loop for the events
  {
    ostringstream sstr;
    printCWakeup(sstr, comp);
    conditionally_write_file(path + comp + "_Wakeup.cc", sstr);
  }

  // Profiling
  {
    ostringstream sstr;
    printProfilerC(sstr, comp);
    conditionally_write_file(path + comp + "_Profiler.cc", sstr);
  }
  {
    ostringstream sstr;
    printProfilerH(sstr, comp);
    conditionally_write_file(path + comp + "_Profiler.hh", sstr);
  }

  // Write internal func files
  for(int i=0; i<m_internal_func_vec.size(); i++) {
    m_internal_func_vec[i]->writeCFiles(path);
  }

}

void StateMachine::printControllerH(ostream& out, string component)
{

  m_message_buffer_names.clear();

  out << "/** \\file " << getIdent() << ".hh" << endl;
  out << "  * " << endl;
  out << "  * Auto generated C++ code started by "<<__FILE__<<":"<<__LINE__<< endl;
  out << "  * Created by slicc definition of Module \"" << getShorthand() << "\"" << endl;
  out << "  */" << endl;
  out << endl;
  out << "#ifndef " << component << "_CONTROLLER_H" << endl;
  out << "#define " << component << "_CONTROLLER_H" << endl;
  out << endl;
  out << "#include \"mem/ruby/common/Global.hh\"" << endl;
  out << "#include \"mem/ruby/common/Consumer.hh\"" << endl;
  out << "#include \"mem/ruby/slicc_interface/AbstractController.hh\"" << endl;
  out << "#include \"mem/protocol/TransitionResult.hh\"" << endl;
  out << "#include \"mem/protocol/Types.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_Profiler.hh\"" << endl;

  // include object classes
  std::set<string> seen_types;
  for(int i=0; i<numObjects(); i++) {
    Var* var = m_objs[i];
    if (seen_types.count(var->getType()->cIdent()) == 0) {
      out << "#include \"mem/protocol/" << var->getType()->cIdent() << ".hh\"" << endl;
      //      out << "class " << var->getType()->cIdent() << ";" << endl;
      seen_types.insert(var->getType()->cIdent());
    }
  }

  out << endl;

  // for adding information to the protocol debug trace
  out << "extern stringstream " << component << "_" << "transitionComment;" << endl;

  out << "class " << component << "_Controller : public AbstractController {" << endl;

  /* the coherence checker needs to call isBlockExclusive() and isBlockShared()
     making the Chip a friend class is an easy way to do this for now */
  out << "#ifdef CHECK_COHERENCE" << endl;
  out << "#endif /* CHECK_COHERENCE */" << endl;

  out << "public:" << endl;
  //  out << "  " << component << "_Controller(int version, Network* net_ptr);" << endl;
  out << "  " << component << "_Controller(const string & name);" << endl;
  out << "  static int getNumControllers();" << endl;
  out << "  void init(Network* net_ptr, const vector<string> & argv);" << endl;
  out << "  MessageBuffer* getMandatoryQueue() const;" << endl;
  out << "  const int & getVersion() const;" << endl;
  out << "  const string toString() const;" << endl;
  out << "  const string getName() const;" << endl;
  out << "  const MachineType getMachineType() const;" << endl;
  out << "  void print(ostream& out) const;" << endl;
  out << "  void printConfig(ostream& out) const;" << endl;
  out << "  void wakeup();" << endl;
  out << "  void printStats(ostream& out) const { s_profiler.dumpStats(out); }" << endl;
  out << "  void clearStats() { s_profiler.clearStats(); }" << endl;
  out << "private:" << endl;
//added by SS
//  found_to_mem = 0;
  std::vector<std::string*>::const_iterator it;
  for(it=m_latency_vector.begin();it!=m_latency_vector.end();it++){
        out << "  int m_" << (*it)->c_str() << ";" << endl;
  }
  if (strncmp(component.c_str(), "L1Cache", 7) == 0) {
    out << "  bool servicing_atomic;" << endl;
    out << "  Address locked_read_request;" << endl;
  }
  out << "  int m_number_of_TBEs;" << endl;

  out << "  TransitionResult doTransition(" << component << "_Event event, " << component
      << "_State state, const Address& addr";
  if(CHECK_INVALID_RESOURCE_STALLS) {
    out << ", int priority";
  }
  out << ");  // in " << component << "_Transitions.cc" << endl;
  out << "  TransitionResult doTransitionWorker(" << component << "_Event event, " << component
      << "_State state, " <<  component << "_State& next_state, const Address& addr";
  if(CHECK_INVALID_RESOURCE_STALLS) {
    out << ", int priority";
  }
  out << ");  // in " << component << "_Transitions.cc" << endl;
  out << "  string m_name;" << endl;
  out << "  int m_transitions_per_cycle;" << endl;
  out << "  int m_buffer_size;" << endl;
  out << "  int m_recycle_latency;" << endl;
  out << "  map< string, string > m_cfg;" << endl;
  out << "  NodeID m_version;" << endl;
  out << "  Network* m_net_ptr;" << endl;
  out << "  MachineID m_machineID;" << endl;
  out << "  " << component << "_Profiler s_profiler;" << endl;
  out << "  static int m_num_controllers;" << endl;

  // internal function protypes
  out << "  // Internal functions" << endl;
  for(int i=0; i<m_internal_func_vec.size(); i++) {
    Func* func = m_internal_func_vec[i];
    string proto;
    func->funcPrototype(proto);
    if (proto != "") {
      out << "  " << proto;
    }
  }

  out << "  // Actions" << endl;
  for(int i=0; i < numActions(); i++) {
    const Action& action = getAction(i);
    out << "/** \\brief " << action.getDescription() << "*/" << endl;
    out << "  void " << action.getIdent() << "(const Address& addr);" << endl;
  }

  // the controller internal variables
  out << "  // Object" << endl;
  for(int i=0; i < numObjects(); i++) {
    const Var* var = m_objs[i];
    string template_hack = "";
    if (var->existPair("template_hack")) {
      template_hack = var->lookupPair("template_hack");
    }
    out << "  " << var->getType()->cIdent() << template_hack << "* m_"
        << var->cIdent() << "_ptr;" << endl;

    string str = "m_"+ var->cIdent() + "_ptr";
    if (var->getType()->cIdent() == "MessageBuffer")
        m_message_buffer_names.push_back(str);

  }


  out << "};" << endl;
  out << "#endif // " << component << "_CONTROLLER_H" << endl;
}

void StateMachine::printControllerC(ostream& out, string component)
{
  out << "/** \\file " << getIdent() << ".cc" << endl;
  out << "  * " << endl;
  out << "  * Auto generated C++ code started by "<<__FILE__<<":"<<__LINE__<< endl;
  out << "  * Created by slicc definition of Module \"" << getShorthand() << "\"" << endl;
  out << "  */" << endl;
  out << endl;
  out << "#include \"mem/ruby/common/Global.hh\"" << endl;
  out << "#include \"mem/ruby/slicc_interface/RubySlicc_includes.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_Controller.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_State.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_Event.hh\"" << endl;
  out << "#include \"mem/protocol/Types.hh\"" << endl;
  out << "#include \"mem/ruby/system/System.hh\"" << endl;

  // include object classes
  std::set<string> seen_types;
  for(int i=0; i<numObjects(); i++) {
    Var* var = m_objs[i];
    if (seen_types.count(var->getType()->cIdent()) == 0) {
      out << "#include \"mem/protocol/" << var->getType()->cIdent() << ".hh\"" << endl;
      seen_types.insert(var->getType()->cIdent());
    }

  }

  out << endl;

  out << "int " << component << "_Controller::m_num_controllers = 0;" << endl;

  // for adding information to the protocol debug trace
  out << "stringstream " << component << "_" << "transitionComment;" << endl;
  out << "#define APPEND_TRANSITION_COMMENT(str) (" << component << "_" << "transitionComment << str)" << endl;

  out << "/** \\brief constructor */" << endl;
  out << component << "_Controller::" << component
    //      << "_Controller(int version, Network* net_ptr)" << endl;
      << "_Controller(const string & name)" << endl;
  out << " : m_name(name)" << endl;
  out << "{ " << endl;
  if (strncmp(component.c_str(), "L1Cache", 7) == 0) {
    out << "  servicing_atomic = false;" << endl;
    out << "  locked_read_request = Address(-1);" << endl;
  }
  out << "  m_num_controllers++; " << endl;
  for(int i=0; i < numObjects(); i++) {
    const Var* var = m_objs[i];
    if ( var->cIdent().find("mandatoryQueue") != string::npos)
      out << "  m_" << var->cIdent() << "_ptr = new " << var->getType()->cIdent() << "();" << endl;
  }
  out << "}" << endl << endl;

  out << "void " << component << "_Controller::init(Network * net_ptr, const vector<string> & argv)" << endl;
  out << "{" << endl;
  out << "  for (size_t i=0; i < argv.size(); i+=2) {" << endl;
//  out << "    printf (\"ARG: %s = %s \\n \", argv[i].c_str(), argv[i+1].c_str());"<< endl;

  out << "    if (argv[i] == \"version\") " << endl;
  out << "      m_version = atoi(argv[i+1].c_str());" << endl;
  out << "    else if (argv[i] == \"transitions_per_cycle\") " << endl;
  out << "      m_transitions_per_cycle = atoi(argv[i+1].c_str());" << endl;
  out << "    else if (argv[i] == \"buffer_size\") " << endl;
  out << "      m_buffer_size = atoi(argv[i+1].c_str());" << endl;
//added by SS
  out << "    else if (argv[i] == \"recycle_latency\") " << endl;
  out << "      m_recycle_latency = atoi(argv[i+1].c_str());" << endl;
//added by SS --> for latency
//for loop on latency_vector to check with argv[i] and assign the value to the related m_latency ...
  out << "    else if (argv[i] == \"number_of_TBEs\") " << endl;
  out << "      m_number_of_TBEs = atoi(argv[i+1].c_str());" << endl;

  if (m_latency_vector.size()) {
  out << "    else { " << endl;
    std::vector<std::string*>::const_iterator it;
    for(it=m_latency_vector.begin();it!=m_latency_vector.end();it++) {
      string str = (*it)->c_str();
      str.erase(0,8);
//convert to lowercase
      size_t i;
      char* strc = (char*) malloc (str.length()+1);
      strc[str.length()]=0;
      for(i=0; i < str.length(); i++) {
        strc[i] = str.at(i);
        strc[i] = tolower(strc[i]);
      }
      str = strc;
      delete strc;
      out << "      if (argv[i] == \"" << str << "\"){" << endl;
      if (str == "to_mem_ctrl_latency")
        out << "        m_" << (*it)->c_str() << "=" << "atoi(argv[i+1].c_str())+(random() % 5);" << endl;
      else
        out << "        m_" << (*it)->c_str() << "=" << "atoi(argv[i+1].c_str());" << endl;
//      out << "        printf (\"SET m_" << it->c_str() << "= %i \\n \", m_" << it->c_str() << ");" << endl;
      out << "      }" << endl;
    }
  out << "    }" << endl;
  }
  out << "  }" << endl;

  out << "  m_net_ptr = net_ptr;" << endl;
  out << "  m_machineID.type = MachineType_" << component << ";" << endl;
  out << "  m_machineID.num = m_version;" << endl;

//  out << "  printf (\"I set m_LATENCY_ISSUE_LATENCY to %i \\n \", m_LATENCY_ISSUE_LATENCY);" << endl;
//  out << "  printf (\"I set m_LATENCY_CACHE_RESPONSE_LATENCY to %i \\n \", m_LATENCY_CACHE_RESPONSE_LATENCY);" << endl;

  // make configuration array
  out << "  for (size_t i=0; i < argv.size(); i+=2) {" << endl;
  out << "    if (argv[i] != \"version\") " << endl;
  out << "      m_cfg[argv[i]] = argv[i+1];" << endl;
  out << "  }" << endl;

  out << endl;

  // initialize objects
  out << "  // Objects" << endl;
  out << "  s_profiler.setVersion(m_version);" << endl;
  for(int i=0; i < numObjects(); i++) {
    const Var* var = m_objs[i];
    if (!var->existPair("network")) {
      // Not a network port object
      if (var->getType()->existPair("primitive")) {
        out << "    m_" << var->cIdent() << "_ptr = new " << var->getType()->cIdent() << ";\n";
        if (var->existPair("default")) {
          out << "    (*m_" << var->cIdent() << "_ptr) = " << var->lookupPair("default") << ";\n";
        }
        out << "  }\n";

      } else {
        // Normal Object
        string template_hack = "";
        if (var->existPair("template_hack")) {
          template_hack = var->lookupPair("template_hack");
        }
//added by SS
        string str = "";
        int found = 0;
        if (var->existPair("factory")) {
          out << "  m_" << var->cIdent() << "_ptr = " << var->lookupPair("factory");
        } else {
          if ( var->cIdent().find("mandatoryQueue") == string::npos) {

            str = "  m_" + var->cIdent() + "_ptr = new " + var->getType()->cIdent() + template_hack;
            out << str;
            if (str.find("TBETable")!=string::npos){
              found = 1;
            }

            if (!var->getType()->existPair("non_obj") && (!var->getType()->isEnumeration())) {
              str = "";
              if (var->existPair("constructor_hack")) {
                string constructor_hack = var->lookupPair("constructor_hack");
                str = "(" + constructor_hack + ")";
              } else {
                str = "()";
              }
              if (found)
                str = "(m_number_of_TBEs)";
              out << str;
            }
          }
        }

        out << ";\n";
        out << "  assert(m_" << var->cIdent() << "_ptr != NULL);" << endl;

        if (var->existPair("default")) {
          out << "  (*m_" << var->cIdent() << "_ptr) = " << var->lookupPair("default")
              << "; // Object default" << endl;
        } else if (var->getType()->hasDefault()) {
            out << "  (*m_" << var->cIdent() << "_ptr) = " << var->getType()->getDefault()
                << "; // Type " << var->getType()->getIdent() << " default" << endl;
        }

        // Set ordering
        if (var->existPair("ordered") && !var->existPair("trigger_queue")) {
          // A buffer
          string ordered =  var->lookupPair("ordered");
          out << "  m_" << var->cIdent() << "_ptr->setOrdering(" << ordered << ");\n";
        }

        // Set randomization
        if (var->existPair("random")) {
          // A buffer
          string value =  var->lookupPair("random");
          out << "  m_" << var->cIdent() << "_ptr->setRandomization(" << value << ");\n";
        }

        // Set Priority
        if (var->getType()->isBuffer() && var->existPair("rank") && !var->existPair("trigger_queue")) {
          string rank =  var->lookupPair("rank");
          out << "  m_" << var->cIdent() << "_ptr->setPriority(" << rank << ");\n";
        }
      }
    } else {
      // Network port object
      string network = var->lookupPair("network");
      string ordered =  var->lookupPair("ordered");
      string vnet =  var->lookupPair("virtual_network");

      assert (var->getMachine() != NULL);
      out << "  m_" << var->cIdent() << "_ptr = m_net_ptr->get"
          << network << "NetQueue(m_version+MachineType_base_number(string_to_MachineType(\""
          << var->getMachine()->getIdent() << "\")), "
          << ordered << ", " << vnet << ");\n";
      out << "  assert(m_" << var->cIdent() << "_ptr != NULL);" << endl;

      // Set ordering
      if (var->existPair("ordered")) {
        // A buffer
        string ordered =  var->lookupPair("ordered");
        out << "  m_" << var->cIdent() << "_ptr->setOrdering(" << ordered << ");\n";
      }

      // Set randomization
      if (var->existPair("random")) {
        // A buffer
        string value =  var->lookupPair("random");
        out << "  m_" << var->cIdent() << "_ptr->setRandomization(" << value << ");\n";
      }

      // Set Priority
      if (var->existPair("rank")) {
        string rank =  var->lookupPair("rank");
        out << "  m_" << var->cIdent() << "_ptr->setPriority(" << rank << ");\n";
      }

      // Set buffer size
      if (var->getType()->isBuffer()) {
        out << "  if (m_buffer_size > 0) {\n";
        out << "    m_" << var->cIdent() << "_ptr->setSize(m_buffer_size);\n";
        out << "  }\n";
      }

      // set description (may be overriden later by port def)
      out << "  m_" << var->cIdent()
          << "_ptr->setDescription(\"[Version \" + int_to_string(m_version) + \", "
          << component << ", name=" << var->cIdent() << "]\");" << endl;
      out << endl;
    }
  }

  // Set the queue consumers
  out << endl;
  for(int i=0; i < m_in_ports.size(); i++) {
    const Var* port = m_in_ports[i];
    out << "  " << port->getCode() << ".setConsumer(this);" << endl;
  }

  // Set the queue descriptions
  out << endl;
  for(int i=0; i < m_in_ports.size(); i++) {
    const Var* port = m_in_ports[i];
    out << "  " << port->getCode()
        << ".setDescription(\"[Version \" + int_to_string(m_version) + \", "
        << component << ", " << port->toString() << "]\");" << endl;
  }

  // Initialize the transition profiling
  out << endl;
  for(int i=0; i<numTransitions(); i++) {
    const Transition& t = getTransition(i);
    const Vector<Action*>& action_vec = t.getActions();
    int numActions = action_vec.size();

    // Figure out if we stall
    bool stall = false;
    for (int i=0; i<numActions; i++) {
      if(action_vec[i]->getIdent() == "z_stall") {
        stall = true;
      }
    }

    // Only possible if it is not a 'z' case
    if (!stall) {
      out << "  s_profiler.possibleTransition(" << component << "_State_"
          << t.getStatePtr()->getIdent() << ", " << component << "_Event_"
          << t.getEventPtr()->getIdent() << ");" << endl;
    }
  }

  //added by SS to initialize recycle_latency of message buffers
  std::vector<std::string>::const_iterator it;
  for ( it=m_message_buffer_names.begin() ; it != m_message_buffer_names.end(); it++ ){
    out << "  "<< (*it).c_str() << "->setRecycleLatency(m_recycle_latency);" << endl;
  }


  out << "}" << endl;

  out << endl;

  bool has_mandatory_q = false;
  for(int i=0; i < m_in_ports.size(); i++) {
    if (m_in_ports[i]->getCode().find("mandatoryQueue_ptr")!= string::npos)
      has_mandatory_q = true;
  }

  out << "int " << component << "_Controller::getNumControllers() {" << endl;
  out << "  return m_num_controllers;" << endl;
  out << "}" << endl;

  out << endl;

  out << "MessageBuffer* " << component << "_Controller::getMandatoryQueue() const {" << endl;
  if (has_mandatory_q)
    out << "  return m_" << component << "_mandatoryQueue_ptr;" << endl;
  else
    out << "  return NULL;" << endl;
  out << "}" << endl;

  out << endl;

  out << "const int & "<<component<<"_Controller::getVersion() const{" << endl;
  out << "  return m_version;" << endl;
  out << "}";

  out << endl;

  out << "const string "<<component<<"_Controller::toString() const{" << endl;
  out << "  return \"" << component<< "_Controller\";" << endl;
  out << "}";

  out << endl;

  out << "const string "<<component<<"_Controller::getName() const{" << endl;
  out << "  return m_name;" << endl;
  out << "}";

  out << endl;

  out << "const MachineType "<<component<<"_Controller::getMachineType() const{" << endl;
  out << "  return MachineType_" << component<< ";" << endl;
  out << "}";

  out << endl;

  out << "void " << component << "_Controller::print(ostream& out) const { out << \"[" << component
      << "_Controller \" << m_version << \"]\"; }" << endl;

  out << "void " << component << "_Controller::printConfig(ostream& out) const {" << endl;
  out << "  out << \"" << component << "_Controller config: \" << m_name << endl;" << endl;
  out << "  out << \"  version: \" << m_version << endl;" << endl;
  out << "  for(map< string, string >::const_iterator it = m_cfg.begin(); it != m_cfg.end(); it++) {" << endl;
  out << "    out << \"  \" << (*it).first << \": \" << (*it).second << endl;" << endl;
  out << "  }" << endl;
  out << "}" << endl;

  out << endl;
  out << "// Actions" << endl;
  out << endl;

  for(int i=0; i < numActions(); i++) {
    const Action& action = getAction(i);
    if (action.existPair("c_code")) {
      out << "/** \\brief " << action.getDescription() << "*/" << endl;
      out << "void " << component << "_Controller::"
          << action.getIdent() << "(const Address& addr)" << endl;
      out << "{" << endl;
      out << "  DEBUG_MSG(GENERATED_COMP, HighPrio,\"executing\");" << endl;
//added by SS
//it should point to m_latency...
//so I should change the string output of this lookup


      string c_code_string = action.lookupPair("c_code");
/*
      size_t found = c_code_string.find("RubyConfig::get");

      if (found!=string::npos){ //found --> replace it with local access
        //if it is related to latency --> replace it
        std::vector<std::string*>::const_iterator it;
        for(it=m_latency_vector.begin();it!=m_latency_vector.end();it++){
          string str = (*it)->c_str();
          str.erase(0,8);
          size_t fd = c_code_string.find(str, found);
          if (fd!=string::npos && (fd == found+15)){
            string rstr = "m_";
            rstr += (*it)->c_str();
            c_code_string.replace(found,15+str.size()+2,rstr);
            break;
          }
        }
      }
*/
      // add here:
      if (strncmp(component.c_str(), "L1Cache", 7) == 0) {
        if (c_code_string.find("writeCallback") != string::npos) {
          string::size_type pos = c_code_string.find("(((*m_L1Cache_sequencer_ptr)).writeCallback");
          assert(pos != string::npos);
          string atomics_string = "\n  if (servicing_atomic) { \n \
   servicing_atomic = false; \n \
   locked_read_request = Address(-1); \n \
 } \n \
 else if (!servicing_atomic) { \n \
   if (addr == locked_read_request) { \n \
     servicing_atomic = true; \n \
   } \n \
 } \n  ";
          c_code_string.insert(pos, atomics_string);
        }
      }
      out << c_code_string;

      out << "}" << endl;
    }
    out << endl;
  }
}

void StateMachine::printCWakeup(ostream& out, string component)
{
  out << "// Auto generated C++ code started by "<<__FILE__<<":"<<__LINE__<< endl;
  out << "// " << getIdent() << ": " << getShorthand() << endl;
  out << endl;
  out << "#include \"mem/ruby/common/Global.hh\"" << endl;
  out << "#include \"mem/ruby/slicc_interface/RubySlicc_includes.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_Controller.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_State.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_Event.hh\"" << endl;
  out << "#include \"mem/protocol/Types.hh\"" << endl;
  out << "#include \"mem/ruby/system/System.hh\"" << endl;
  out << endl;
  out << "void " << component << "_Controller::wakeup()" << endl;
  out << "{" << endl;
  //  out << "  DEBUG_EXPR(GENERATED_COMP, MedPrio,*this);" << endl;
  //  out << "  DEBUG_EXPR(GENERATED_COMP, MedPrio,g_eventQueue_ptr->getTime());" << endl;
  out << endl;
  out << "int counter = 0;" << endl;
  out << "  while (true) {" << endl;
  out << "    // Some cases will put us into an infinite loop without this limit" << endl;
  out << "    assert(counter <= m_transitions_per_cycle);" << endl;
  out << "    if (counter == m_transitions_per_cycle) {" << endl;
  out << "      g_system_ptr->getProfiler()->controllerBusy(m_machineID); // Count how often we're fully utilized" << endl;
  out << "      g_eventQueue_ptr->scheduleEvent(this, 1); // Wakeup in another cycle and try again" << endl;
  out << "      break;" << endl;
  out << "    }" << endl;

  // InPorts
  //
  // Find the position of the mandatory queue in the vector so that we can print it out first
  int j = -1;
  if (strncmp(component.c_str(), "L1Cache", 7) == 0) {
    for(int i=0; i < m_in_ports.size(); i++) {
        const Var* port = m_in_ports[i];
        assert(port->existPair("c_code_in_port"));
        if (port->toString().find("mandatoryQueue_in") != string::npos) {
          assert (j == -1);
          j = i;
        }
        else {
          cout << port->toString() << endl << flush;
        }
    }
    
    assert(j != -1);

    // print out the mandatory queue here
    const Var* port = m_in_ports[j];
    assert(port->existPair("c_code_in_port"));
    out << "    // " 
        << component << "InPort " << port->toString() 
        << endl;
    string output = port->lookupPair("c_code_in_port");
    string::size_type pos = output.find("TransitionResult result = doTransition((L1Cache_mandatory_request_type_to_event(((*in_msg_ptr)).m_Type)), L1Cache_getState(addr), addr);");
    assert(pos != string::npos);
    string atomics_string = "\n \
             bool postpone = false; \n \
             if ((((*in_msg_ptr)).m_Type) == CacheRequestType_ATOMIC) { \n \
               if (!servicing_atomic) { \n \
                  if (locked_read_request == Address(-1)) { \n \
                    locked_read_request = addr;  \n \
                  } \n \
                  else if (addr == locked_read_request) { \n \
                    ; // do nothing \n\ 
                  } \n \
                  else { \n \
                    assert(0); // should never be here if servicing one request at a time \n\ 
                  } \n \
               } \n \
               else if (addr != locked_read_request) { \n \
                 // this is probably caused by shift optimizations \n \
                 locked_read_request = addr; \n\
               } \n \
             } \n \
             else { \n \
               if (locked_read_request != Address(-1)) { \n \
                 locked_read_request = Address(-1); \n \
                 servicing_atomic = false; \n \
               } \n \
             } \n \
           if (!postpone) { \n \
               ";



    output.insert(pos, atomics_string);
    string foo = "// Cannot do anything with this transition, go check next doable transition (mostly likely of next port)\n";
    string::size_type next_pos = output.find(foo, pos);
    next_pos = next_pos + foo.length();

    assert(next_pos != string::npos);
    string complete = "              }\n";
    output.insert(next_pos, complete);
    //out << port->lookupPair("c_code_in_port");
    out << output;
    out << endl;
  }
  for(int i=0; i < m_in_ports.size(); i++) {
    const Var* port = m_in_ports[i];
    // don't print out mandatory queue twice
    if (i != j) {
      if (strncmp(component.c_str(), "L1Cache", 7) == 0) {
        if (port->toString().find("forwardRequestNetwork_in") != string::npos) {
          out << "    bool postpone = false;" << endl;
          out << "    if ((((*m_L1Cache_forwardToCache_ptr)).isReady())) {" << endl;
          out << "      const RequestMsg* in_msg_ptr;" << endl;
          out << "      in_msg_ptr = dynamic_cast<const RequestMsg*>(((*m_L1Cache_forwardToCache_ptr)).peek());" << endl;
          out << "      if ((servicing_atomic && locked_read_request == ((*in_msg_ptr)).m_Address)) {" << endl;
          out << "        postpone = true;" << endl;
          out << "      }" << endl;
          out << "    }" << endl;
          out << "    if (!postpone) {" << endl;
        }
      }
      assert(port->existPair("c_code_in_port"));
      out << "    // " 
          << component << "InPort " << port->toString() 
          << endl;
      out << port->lookupPair("c_code_in_port");
      if (strncmp(component.c_str(), "L1Cache", 7) == 0) {
        if (port->toString().find("forwardRequestNetwork_in") != string::npos) {
          out << "}" << endl;
        }
      }
      out << endl;
    }
  }

  out << "    break;  // If we got this far, we have nothing left todo" << endl;
  out << "  }" << endl;
  //  out << "  g_eventQueue_ptr->scheduleEvent(this, 1);" << endl;
  //  out << "  DEBUG_NEWLINE(GENERATED_COMP, MedPrio);" << endl;
  out << "}" << endl;
  out << endl;
}

void StateMachine::printCSwitch(ostream& out, string component)
{
  out << "// Auto generated C++ code started by "<<__FILE__<<":"<<__LINE__<< endl;
  out << "// " << getIdent() << ": " << getShorthand() << endl;
  out << endl;
  out << "#include \"mem/ruby/common/Global.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_Controller.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_State.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_Event.hh\"" << endl;
  out << "#include \"mem/protocol/Types.hh\"" << endl;
  out << "#include \"mem/ruby/system/System.hh\"" << endl;
  out << endl;
  out << "#define HASH_FUN(state, event)  ((int(state)*" << component
      << "_Event_NUM)+int(event))" << endl;
  out << endl;
  out << "#define GET_TRANSITION_COMMENT() (" << component << "_" << "transitionComment.str())" << endl;
  out << "#define CLEAR_TRANSITION_COMMENT() (" << component << "_" << "transitionComment.str(\"\"))" << endl;
  out << endl;
  out << "TransitionResult " << component << "_Controller::doTransition("
      << component << "_Event event, "
      << component << "_State state, "
      << "const Address& addr" << endl;
  if(CHECK_INVALID_RESOURCE_STALLS) {
    out << ", int priority";
  }
  out << ")" << endl;

  out << "{" << endl;
  out << "  " << component << "_State next_state = state;" << endl;
  out << endl;
  out << "  DEBUG_NEWLINE(GENERATED_COMP, MedPrio);" << endl;
  out << "  DEBUG_MSG(GENERATED_COMP, MedPrio,*this);" << endl;
  out << "  DEBUG_EXPR(GENERATED_COMP, MedPrio,g_eventQueue_ptr->getTime());" << endl;
  out << "  DEBUG_EXPR(GENERATED_COMP, MedPrio,state);" << endl;
  out << "  DEBUG_EXPR(GENERATED_COMP, MedPrio,event);" << endl;
  out << "  DEBUG_EXPR(GENERATED_COMP, MedPrio,addr);" << endl;
  out << endl;
  out << "  TransitionResult result = doTransitionWorker(event, state, next_state, addr";
  if(CHECK_INVALID_RESOURCE_STALLS) {
    out << ", priority";
  }
  out << ");" << endl;
  out << endl;
  out << "  if (result == TransitionResult_Valid) {" << endl;
  out << "    DEBUG_EXPR(GENERATED_COMP, MedPrio, next_state);" << endl;
  out << "    DEBUG_NEWLINE(GENERATED_COMP, MedPrio);" << endl;
  out << "    s_profiler.countTransition(state, event);" << endl;
  out << "    if (Debug::getProtocolTrace()) {" << endl
      << "      g_system_ptr->getProfiler()->profileTransition(\"" << component
      << "\", m_version, addr, " << endl
      << "        " << component << "_State_to_string(state), " << endl
      << "        " << component << "_Event_to_string(event), " << endl
      << "        " << component << "_State_to_string(next_state), GET_TRANSITION_COMMENT());" << endl
      << "    }" << endl;
  out << "    CLEAR_TRANSITION_COMMENT();" << endl;
  out << "    " << component << "_setState(addr, next_state);" << endl;
  out << "    " << endl;
  out << "  } else if (result == TransitionResult_ResourceStall) {" << endl;
  out << "    if (Debug::getProtocolTrace()) {" << endl
      << "      g_system_ptr->getProfiler()->profileTransition(\"" << component
      << "\", m_version, addr, " << endl
      << "        " << component << "_State_to_string(state), " << endl
      << "        " << component << "_Event_to_string(event), " << endl
      << "        " << component << "_State_to_string(next_state), " << endl
      << "        \"Resource Stall\");" << endl
      << "    }" << endl;
  out << "  } else if (result == TransitionResult_ProtocolStall) {" << endl;
  out << "    DEBUG_MSG(GENERATED_COMP,HighPrio,\"stalling\");" << endl
      << "    DEBUG_NEWLINE(GENERATED_COMP, MedPrio);" << endl;
  out << "    if (Debug::getProtocolTrace()) {" << endl
      << "      g_system_ptr->getProfiler()->profileTransition(\"" << component
      << "\", m_version, addr, " << endl
      << "        " << component << "_State_to_string(state), " << endl
      << "        " << component << "_Event_to_string(event), " << endl
      << "        " << component << "_State_to_string(next_state), " << endl
      << "        \"Protocol Stall\");" << endl
      << "    }" << endl
      << "  }" << endl;
  out << "  return result;" << endl;
  out << "}" << endl;
  out << endl;
  out << "TransitionResult " << component << "_Controller::doTransitionWorker("
      << component << "_Event event, "
      << component << "_State state, "
      << component << "_State& next_state, "
      << "const Address& addr" << endl;
  if(CHECK_INVALID_RESOURCE_STALLS) {
    out << ", int priority" << endl;
  }
  out << ")" << endl;

  out << "{" << endl;
  out << "" << endl;

  out << "  switch(HASH_FUN(state, event)) {" << endl;

  Map<string, Vector<string> > code_map; // This map will allow suppress generating duplicate code
  Vector<string> code_vec;

  for(int i=0; i<numTransitions(); i++) {
    const Transition& t = getTransition(i);
    string case_string = component + "_State_" + t.getStatePtr()->getIdent()
      + ", " + component + "_Event_" + t.getEventPtr()->getIdent();

    string code;

    code += "  {\n";
    // Only set next_state if it changes
    if (t.getStatePtr() != t.getNextStatePtr()) {
      code += "    next_state = " + component + "_State_" + t.getNextStatePtr()->getIdent() + ";\n";
    }

    const Vector<Action*>& action_vec = t.getActions();
    int numActions = action_vec.size();

    // Check for resources
    Vector<string> code_sorter;
    const Map<Var*, string>& res = t.getResources();
    Vector<Var*> res_keys = res.keys();
    for (int i=0; i<res_keys.size(); i++) {
      string temp_code;
      if (res_keys[i]->getType()->cIdent() == "DNUCAStopTable") {
        temp_code += res.lookup(res_keys[i]);
      } else {
        temp_code += "    if (!" + (res_keys[i]->getCode()) + ".areNSlotsAvailable(" + res.lookup(res_keys[i]) + ")) {\n";
        if(CHECK_INVALID_RESOURCE_STALLS) {
          // assert that the resource stall is for a resource of equal or greater priority
          temp_code += "      assert(priority >= "+ (res_keys[i]->getCode()) + ".getPriority());\n";
        }
        temp_code += "      return TransitionResult_ResourceStall;\n";
        temp_code += "    }\n";
      }
      code_sorter.insertAtBottom(temp_code);
    }

    // Emit the code sequences in a sorted order.  This makes the
    // output deterministic (without this the output order can vary
    // since Map's keys() on a vector of pointers is not deterministic
    code_sorter.sortVector();
    for (int i=0; i<code_sorter.size(); i++) {
      code += code_sorter[i];
    }

    // Figure out if we stall
    bool stall = false;
    for (int i=0; i<numActions; i++) {
      if(action_vec[i]->getIdent() == "z_stall") {
        stall = true;
      }
    }

    if (stall) {
      code += "    return TransitionResult_ProtocolStall;\n";
    } else {
      for (int i=0; i<numActions; i++) {
        code += "    " + action_vec[i]->getIdent() + "(addr);\n";
      }
      code += "    return TransitionResult_Valid;\n";
    }
    code += "  }\n";


    // Look to see if this transition code is unique.
    if (code_map.exist(code)) {
      code_map.lookup(code).insertAtBottom(case_string);
    } else {
      Vector<string> vec;
      vec.insertAtBottom(case_string);
      code_map.add(code, vec);
      code_vec.insertAtBottom(code);
    }
  }

  // Walk through all of the unique code blocks and spit out the
  // corresponding case statement elements
  for (int i=0; i<code_vec.size(); i++) {
    string code = code_vec[i];

    // Iterative over all the multiple transitions that share the same code
    for (int case_num=0; case_num<code_map.lookup(code).size(); case_num++) {
      string case_string = code_map.lookup(code)[case_num];
      out << "  case HASH_FUN(" << case_string << "):" << endl;
    }
    out << code;
  }

  out << "  default:" << endl;
  out << "    WARN_EXPR(m_version);" << endl;
  out << "    WARN_EXPR(g_eventQueue_ptr->getTime());" << endl;
  out << "    WARN_EXPR(addr);" << endl;
  out << "    WARN_EXPR(event);" << endl;
  out << "    WARN_EXPR(state);" << endl;
  out << "    ERROR_MSG(\"Invalid transition\");" << endl;
  out << "  }" << endl;
  out << "  return TransitionResult_Valid;" << endl;
  out << "}" << endl;
}

void StateMachine::printProfilerH(ostream& out, string component)
{
  out << "// Auto generated C++ code started by "<<__FILE__<<":"<<__LINE__<< endl;
  out << "// " << getIdent() << ": " << getShorthand() << endl;
  out << endl;
  out << "#ifndef " << component << "_PROFILER_H" << endl;
  out << "#define " << component << "_PROFILER_H" << endl;
  out << endl;
  out << "#include \"mem/ruby/common/Global.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_State.hh\"" << endl;
  out << "#include \"mem/protocol/" << component << "_Event.hh\"" << endl;
  out << endl;
  out << "class " << component << "_Profiler {" << endl;
  out << "public:" << endl;
  out << "  " << component << "_Profiler();" << endl;
  out << "  void setVersion(int version);" << endl;
  out << "  void countTransition(" << component << "_State state, " << component << "_Event event);" << endl;
  out << "  void possibleTransition(" << component << "_State state, " << component << "_Event event);" << endl;
  out << "  void dumpStats(ostream& out) const;" << endl;
  out << "  void clearStats();" << endl;
  out << "private:" << endl;
  out << "  int m_counters[" << component << "_State_NUM][" << component << "_Event_NUM];" << endl;
  out << "  int m_event_counters[" << component << "_Event_NUM];" << endl;
  out << "  bool m_possible[" << component << "_State_NUM][" << component << "_Event_NUM];" << endl;
  out << "  int m_version;" << endl;
  out << "};" << endl;
  out << "#endif // " << component << "_PROFILER_H" << endl;
}

void StateMachine::printProfilerC(ostream& out, string component)
{
  out << "// Auto generated C++ code started by "<<__FILE__<<":"<<__LINE__<< endl;
  out << "// " << getIdent() << ": " << getShorthand() << endl;
  out << endl;
  out << "#include \"mem/protocol/" << component << "_Profiler.hh\"" << endl;
  out << endl;

  // Constructor
  out << component << "_Profiler::" << component << "_Profiler()" << endl;
  out << "{" << endl;
  out << "  for (int state = 0; state < " << component << "_State_NUM; state++) {" << endl;
  out << "    for (int event = 0; event < " << component << "_Event_NUM; event++) {" << endl;
  out << "      m_possible[state][event] = false;" << endl;
  out << "      m_counters[state][event] = 0;" << endl;
  out << "    }" << endl;
  out << "  }" << endl;
  out << "  for (int event = 0; event < " << component << "_Event_NUM; event++) {" << endl;
  out << "    m_event_counters[event] = 0;" << endl;
  out << "  }" << endl;
  out << "}" << endl;

  // setVersion
  out << "void " << component << "_Profiler::setVersion(int version)" << endl;
  out << "{" << endl;
  out << "  m_version = version;" << endl;
  out << "}" << endl;

  // Clearstats
  out << "void " << component << "_Profiler::clearStats()" << endl;
  out << "{" << endl;
  out << "  for (int state = 0; state < " << component << "_State_NUM; state++) {" << endl;
  out << "    for (int event = 0; event < " << component << "_Event_NUM; event++) {" << endl;
  out << "      m_counters[state][event] = 0;" << endl;
  out << "    }" << endl;
  out << "  }" << endl;
  out << "  for (int event = 0; event < " << component << "_Event_NUM; event++) {" << endl;
  out << "    m_event_counters[event] = 0;" << endl;
  out << "  }" << endl;
  out << "}" << endl;

  // Count Transition
  out << "void " << component << "_Profiler::countTransition(" << component << "_State state, " << component << "_Event event)" << endl;
  out << "{" << endl;
  out << "  assert(m_possible[state][event]);" << endl;
  out << "  m_counters[state][event]++;" << endl;
  out << "  m_event_counters[event]++;" << endl;
  out << "}" << endl;

  // Possible Transition
  out << "void " << component << "_Profiler::possibleTransition(" << component << "_State state, " << component << "_Event event)" << endl;
  out << "{" << endl;
  out << "  m_possible[state][event] = true;" << endl;
  out << "}" << endl;

  // dumpStats
  out << "void " << component << "_Profiler::dumpStats(ostream& out) const" << endl;
  out << "{" << endl;
  out << "  out << \" --- " << component << " \" << m_version << \" ---\" << endl;" << endl;
  out << "  out << \" - Event Counts -\" << endl;" << endl;
  out << "  for (int event = 0; event < " << component << "_Event_NUM; event++) {" << endl;
  out << "    int count = m_event_counters[event];" << endl;
  out << "    out << (" << component << "_Event) event << \"  \" << count << endl;" << endl;
  out << "  }" << endl;
  out << "  out << endl;" << endl;
  out << "  out << \" - Transitions -\" << endl;" << endl;
  out << "  for (int state = 0; state < " << component << "_State_NUM; state++) {" << endl;
  out << "    for (int event = 0; event < " << component << "_Event_NUM; event++) {" << endl;
  out << "      if (m_possible[state][event]) {" << endl;
  out << "        int count = m_counters[state][event];" << endl;
  out << "        out << (" << component << "_State) state << \"  \" << (" << component << "_Event) event << \"  \" << count;" << endl;
  out << "        if (count == 0) {" << endl;
  out << "            out << \" <-- \";" << endl;
  out << "        }" << endl;
  out << "        out << endl;" << endl;
  out << "      }" << endl;
  out << "    }" << endl;
  out << "    out << endl;" << endl;
  out << "  }" << endl;
  out << "}" << endl;
}



// ************************** //
// ******* HTML Files ******* //
// ************************** //

string frameRef(string click_href, string click_target, string over_href, string over_target_num, string text)
{
  string temp;
  temp += "<A href=\"" + click_href + "\" ";
  temp += "target=\"" + click_target + "\" ";
  string javascript = "if (parent.frames[" + over_target_num + "].location != parent.location + '" + over_href + "') { parent.frames[" + over_target_num + "].location='" + over_href + "' }";
  //  string javascript = "parent." + target + ".location='" + href + "'";
  temp += "onMouseOver=\"" + javascript + "\" ";
  temp += ">" + text + "</A>";
  return temp;
}

string frameRef(string href, string target, string target_num, string text)
{
  return frameRef(href, target, href, target_num, text);
}


void StateMachine::writeHTMLFiles(string path)
{
  string filename;
  string component = getIdent();

  /*
  {
    ostringstream out;
    out << "<html>" << endl;
    out << "<head>" << endl;
    out << "<title>" << component << "</title>" << endl;
    out << "</head>" << endl;
    out << "<frameset rows=\"30,30,*\" frameborder=\"1\">" << endl;
    out << "  <frame name=\"Status\" src=\"empty.html\" marginheight=\"1\">" << endl;
    out << "  <frame name=\"Table\" src=\"" << component << "_table.html\" marginheight=\"1\">" << endl;
    out << "</frameset>" << endl;
    out << "</html>" << endl;
    conditionally_write_file(path + component + ".html", out);
  }
  */

  // Create table with no row hilighted
  {
    ostringstream out;
    printHTMLTransitions(out, numStates()+1);

    // -- Write file
    filename = component + "_table.html";
    conditionally_write_file(path + filename, out);
  }

  // Generate transition tables
  for(int i=0; i<numStates(); i++) {
    ostringstream out;
    printHTMLTransitions(out, i);

    // -- Write file
    filename = component + "_table_" + getState(i).getIdent() + ".html";
    conditionally_write_file(path + filename, out);
  }

  // Generate action descriptions
  for(int i=0; i<numActions(); i++) {
    ostringstream out;
    createHTMLSymbol(getAction(i), "Action", out);

    // -- Write file
    filename = component + "_action_" + getAction(i).getIdent() + ".html";
    conditionally_write_file(path + filename, out);
  }

  // Generate state descriptions
  for(int i=0; i<numStates(); i++) {
    ostringstream out;
    createHTMLSymbol(getState(i), "State", out);

    // -- Write file
    filename = component + "_State_" + getState(i).getIdent() + ".html";
    conditionally_write_file(path + filename, out);
  }

  // Generate event descriptions
  for(int i=0; i<numEvents(); i++) {
    ostringstream out;
    createHTMLSymbol(getEvent(i), "Event", out);

    // -- Write file
    filename = component + "_Event_" + getEvent(i).getIdent() + ".html";
    conditionally_write_file(path + filename, out);
  }
}

void StateMachine::printHTMLTransitions(ostream& out, int active_state)
{
  // -- Prolog
  out << "<HTML><BODY link=\"blue\" vlink=\"blue\">" << endl;

  // -- Header
  out << "<H1 align=\"center\">" << formatHTMLShorthand(getShorthand()) << ": " << endl;
  Vector<StateMachine*> machine_vec = g_sym_table.getStateMachines();
  for (int i=0; i<machine_vec.size(); i++) {
    StateMachine* type = machine_vec[i];
    if (i != 0) {
      out << " - ";
    }
    if (type == this) {
      out << type->getIdent() << endl;
    } else {
      out << "<A target=\"Table\"href=\"" + type->getIdent() + "_table.html\">" + type->getIdent() + "</A>  " << endl;
    }
  }
  out << "</H1>" << endl;

  // -- Table header
  out << "<TABLE border=1>" << endl;

  // -- Column headers
  out << "<TR>" << endl;

  // -- First column header
  out << "  <TH> </TH>" << endl;

  for(int event = 0; event < numEvents(); event++ ) {
    out << "  <TH bgcolor=white>";
    out << frameRef(getIdent() + "_Event_" + getEvent(event).getIdent() + ".html", "Status", "1", formatHTMLShorthand(getEvent(event).getShorthand()));
    out << "</TH>" << endl;
  }

  out << "</TR>" << endl;

  // -- Body of table
  for(int state = 0; state < numStates(); state++ ) {
    out << "<TR>" << endl;

    // -- Each row
    if (state == active_state) {
      out << "  <TH bgcolor=yellow>";
    } else {
      out << "  <TH bgcolor=white>";
    }

    string click_href = getIdent() + "_table_" + getState(state).getIdent() + ".html";
    string text = formatHTMLShorthand(getState(state).getShorthand());

    out << frameRef(click_href, "Table", getIdent() + "_State_" + getState(state).getIdent() + ".html", "1", formatHTMLShorthand(getState(state).getShorthand()));
    out << "</TH>" << endl;

    // -- One column for each event
    for(int event = 0; event < numEvents(); event++ ) {
      const Transition* trans_ptr = getTransPtr(state, event);

      if( trans_ptr != NULL ) {
        bool stall_action = false;
        string nextState;
        string actions_str;

        // -- Get the actions
        //        actions = trans_ptr->getActionShorthands();
        const Vector<Action*> actions = trans_ptr->getActions();
        for (int action=0; action < actions.size(); action++) {
          if ((actions[action]->getIdent() == "z_stall") ||
              (actions[action]->getIdent() == "zz_recycleMandatoryQueue")) {
            stall_action = true;
          }
          actions_str += "  ";
          actions_str += frameRef(getIdent() + "_action_" + actions[action]->getIdent() + ".html", "Status", "1",
                                  formatHTMLShorthand(actions[action]->getShorthand()));
          actions_str += "\n";
        }

        // -- Get the next state
        if (trans_ptr->getNextStatePtr()->getIdent() != getState(state).getIdent()) {
          string click_href = getIdent() + "_table_" + trans_ptr->getNextStatePtr()->getIdent() + ".html";
          nextState = frameRef(click_href, "Table", getIdent() + "_State_" + trans_ptr->getNextStatePtr()->getIdent() + ".html", "1",
                               formatHTMLShorthand(trans_ptr->getNextStateShorthand()));
        } else {
          nextState = "";
        }

        // -- Print out "actions/next-state"
        if (stall_action) {
          if (state == active_state) {
            out << "  <TD bgcolor=#C0C000>";
          } else {
            out << "  <TD bgcolor=lightgrey>";
          }
        } else if (active_state < numStates() && (trans_ptr->getNextStatePtr()->getIdent() == getState(active_state).getIdent())) {
          out << "  <TD bgcolor=aqua>";
        } else if (state == active_state) {
           out << "  <TD bgcolor=yellow>";
        } else {
          out << "  <TD bgcolor=white>";
        }

        out << actions_str;
        if ((nextState.length() != 0) && (actions_str.length() != 0)) {
          out << "/";
        }
        out << nextState;
        out << "</TD>" << endl;
      } else {
        // This is the no transition case
        if (state == active_state) {
          out << "  <TD bgcolor=#C0C000>&nbsp;</TD>" << endl;
        } else {
          out << "  <TD bgcolor=lightgrey>&nbsp;</TD>" << endl;
        }
      }
    }
    // -- Each row
    if (state == active_state) {
      out << "  <TH bgcolor=yellow>";
    } else {
      out << "  <TH bgcolor=white>";
    }

    click_href = getIdent() + "_table_" + getState(state).getIdent() + ".html";
    text = formatHTMLShorthand(getState(state).getShorthand());

    out << frameRef(click_href, "Table", getIdent() + "_State_" + getState(state).getIdent() + ".html", "1", formatHTMLShorthand(getState(state).getShorthand()));
    out << "</TH>" << endl;

    out << "</TR>" << endl;
  }

  // -- Column footer
  out << "<TR>" << endl;
  out << "  <TH> </TH>" << endl;

  for(int i = 0; i < numEvents(); i++ ) {
    out << "  <TH bgcolor=white>";
    out << frameRef(getIdent() + "_Event_" + getEvent(i).getIdent() + ".html", "Status", "1", formatHTMLShorthand(getEvent(i).getShorthand()));
    out << "</TH>" << endl;
  }
  out << "</TR>" << endl;

  // -- Epilog
  out << "</TABLE>" << endl;
  out << "</BODY></HTML>" << endl;
}


