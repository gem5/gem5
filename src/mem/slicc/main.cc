
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

#include "main.hh"
#include "StateMachine.hh"
#include "mif_gen.hh"
#include "html_gen.hh"
#include "fileio.hh"
#include "DeclListAST.hh"
#include "Type.hh"
#include "SymbolTable.hh"
#include "Event.hh"
#include "State.hh"
#include "Action.hh"
#include "Transition.hh"

// -- Main conversion functions

void printDotty(const StateMachine& sm, ostream& out);
void printTexTable(const StateMachine& sm, ostream& out);

DeclListAST* g_decl_list_ptr;
DeclListAST* parse(string filename);

int main(int argc, char *argv[])
{
  cerr << "SLICC v0.3" << endl;

  if (argc < 5) {
    cerr << "  Usage: generator.exec <code path> <html path> <ident> <html direction> files ... " << endl;
    exit(1);
  }

  // The path we should place the generated code
  string code_path(argv[1]);
  code_path += "/";

  // The path we should place the generated html
  string html_path(argv[2]);
  html_path += "/";

  string ident(argv[3]);

  string html_generate(argv[4]);

  Vector<DeclListAST*> decl_list_vec;

  // Parse
  cerr << "Parsing..." << endl;
  for(int i=5; i<argc; i++) {
    cerr << "  " << argv[i] << endl;
    DeclListAST* decl_list_ptr = parse(argv[i]);
    decl_list_vec.insertAtBottom(decl_list_ptr);
  }

  // Find machines
  cerr << "Generator pass 1..." << endl;
  int size = decl_list_vec.size();
  for(int i=0; i<size; i++) {
    DeclListAST* decl_list_ptr = decl_list_vec[i];
    decl_list_ptr->findMachines();
  }

  // Generate Code
  cerr << "Generator pass 2..." << endl;
  for(int i=0; i<size; i++) {
    DeclListAST* decl_list_ptr = decl_list_vec[i];
    decl_list_ptr->generate();
    delete decl_list_ptr;
  }

  // Generate C/C++ files
  cerr << "Writing C files..." << endl;

  {
    // Generate the name of the protocol
    ostringstream sstr;
    sstr << "// Auto generated C++ code started by "<<__FILE__<<":"<<__LINE__<<endl;
    sstr << endl;
    sstr << "#ifndef PROTOCOL_NAME_H" << endl;
    sstr << "#define PROTOCOL_NAME_H" << endl;
    sstr << endl;
    sstr << "const char CURRENT_PROTOCOL[] = \"";
    sstr << ident << "\";\n";
    sstr << "#endif // PROTOCOL_NAME_H" << endl;
    conditionally_write_file(code_path + "/protocol_name.hh", sstr);
  }

  g_sym_table.writeCFiles(code_path);

  // Generate HTML files
  if (html_generate == "html") {
    cerr << "Writing HTML files..." << endl;
    g_sym_table.writeHTMLFiles(html_path);
  } else if (html_generate == "no_html") {
    cerr << "No HTML files generated" << endl;
  } else {
    cerr << "ERROR, unidentified html direction" << endl;
  }

  cerr << "Done..." << endl;

  // Generate MIF files
  cerr << "Writing MIF files..." << endl;
  g_sym_table.writeMIFFiles(html_path);

  cerr << "Done..." << endl;

}
  /*
  if(!strcmp(argv[2], "parse")) {
    // Parse only
  } else if(!strcmp(argv[2], "state")) {
    printStateTableMIF(s, cout);
  } else if(!strcmp( argv[2], "event")) {
    printEventTableMIF(s, cout);
  } else if(!strcmp( argv[2], "action")) {
    printActionTableMIF(s, cout);
  } else if(!strcmp( argv[2], "transition")) {
    printTransitionTableMIF(s, cout);
  } else if(!strcmp( argv[2], "tbe")) {
    for(int i=0; i<s.numTypes(); i++) {
      if (s.getType(i).getIdent() == "TBE") {
        printTBETableMIF(s, s.getTypeFields(i), cout);
      }
    }
  } else if(!strcmp( argv[2], "dot")) {
    printDotty(s, cout);
  } else if(!strcmp( argv[2], "latex")) {
    printTexTable(s, cout);
  } else if (!strcmp( argv[2], "murphi")) {
    printMurphi(s, cout);
  } else if (!strcmp( argv[2], "html")) {
    printHTML(s);
  } else if(!strcmp( argv[2], "code")) {
    if (argc < 4) {
      cerr << "Error: Wrong number of command line parameters!" << endl;
      exit(1);
    }
  */


void printDotty(const StateMachine& sm, ostream& out)
{
  out << "digraph " << sm.getIdent() << " {" << endl;
  for(int i=0; i<sm.numTransitions(); i++) {
    const Transition& t = sm.getTransition(i);
    // Don't print ignored transitions
    if ((t.getActionShorthands() != "--") && (t.getActionShorthands() != "z")) {
    //    if (t.getStateShorthand() != t.getNextStateShorthand()) {
      out << "  " << t.getStateShorthand() << " -> ";
      out << t.getNextStateShorthand() << "[label=\"";
      out << t.getEventShorthand() << "/"
          << t.getActionShorthands() << "\"]" << endl;
    }
  }
  out << "}" << endl;
}

void printTexTable(const StateMachine& sm, ostream& out)
{
  const Transition* trans_ptr;
  int stateIndex, eventIndex;
  string actions;
  string nextState;

  out << "%& latex" << endl;
  out << "\\documentclass[12pt]{article}" << endl;
  out << "\\usepackage{graphics}" << endl;
  out << "\\begin{document}" << endl;
  //  out << "{\\large" << endl;
  out << "\\begin{tabular}{|l||";
  for(eventIndex=0; eventIndex < sm.numEvents(); eventIndex++) {
    out << "l";
  }
  out << "|} \\hline" << endl;

  for(eventIndex=0; eventIndex < sm.numEvents(); eventIndex++) {
    out << " & \\rotatebox{90}{";
    out << sm.getEvent(eventIndex).getShorthand();
    out << "}";
  }
  out << "\\\\ \\hline  \\hline" << endl;

  for(stateIndex=0; stateIndex < sm.numStates(); stateIndex++) {
    out << sm.getState(stateIndex).getShorthand();
    for(eventIndex=0; eventIndex < sm.numEvents(); eventIndex++) {
      out << " & ";
      trans_ptr = sm.getTransPtr(stateIndex, eventIndex);
      if (trans_ptr == NULL) {
      } else {
        actions = trans_ptr->getActionShorthands();
        // FIXME: should compare index, not the string
        if (trans_ptr->getNextStateShorthand() !=
            sm.getState(stateIndex).getShorthand() ) {
          nextState = trans_ptr->getNextStateShorthand();
        } else {
          nextState = "";
        }

        out << actions;
        if ((nextState.length() != 0) && (actions.length() != 0)) {
          out << "/";
        }
        out << nextState;
      }
    }
    out << "\\\\" << endl;
  }
  out << "\\hline" << endl;
  out << "\\end{tabular}" << endl;
  //  out << "}" << endl;
  out << "\\end{document}" << endl;
}

