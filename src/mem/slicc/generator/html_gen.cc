
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
 * html_gen.C
 *
 * Description: See html_gen.hh
 *
 * $Id: html_gen.C,v 3.4 2004/01/31 20:46:50 milo Exp $
 *
 * */

#include "mem/slicc/generator/html_gen.hh"
#include "mem/slicc/generator/fileio.hh"
#include "mem/slicc/symbols/SymbolTable.hh"

string formatHTMLShorthand(const string shorthand);


void createHTMLSymbol(const Symbol& sym, string title, ostream& out)
{
  out << "<HTML><BODY><BIG>" << endl;
  out << title << ": " << endl;
  out << formatHTMLShorthand(sym.getShorthand()) << " - ";
  out << sym.getDescription();
  out << "</BIG></BODY></HTML>" << endl;
}

void createHTMLindex(string title, ostream& out)
{
  out << "<html>" << endl;
  out << "<head>" << endl;
  out << "<title>" << title << "</title>" << endl;
  out << "</head>" << endl;
  out << "<frameset rows=\"*,30\">" << endl;
  Vector<StateMachine*> machine_vec = g_sym_table.getStateMachines();
  if (machine_vec.size() > 1) {
    string machine = machine_vec[0]->getIdent();
    out << "  <frame name=\"Table\" src=\"" << machine << "_table.html\">" << endl;
  } else {
    out << "  <frame name=\"Table\" src=\"empty.html\">" << endl;
  }

  out << "  <frame name=\"Status\" src=\"empty.html\">" << endl;
  out << "</frameset>" << endl;
  out << "</html>" << endl;
}

string formatHTMLShorthand(const string shorthand)
{
  string munged_shorthand = "";
  bool mode_is_normal = true;

  // -- Walk over the string, processing superscript directives
  for(unsigned int i = 0; i < shorthand.length(); i++) {
    if(shorthand[i] == '!') {
      // -- Reached logical end of shorthand name
      break;
    } else if( shorthand[i] == '_') {
      munged_shorthand += " ";
    } else if( shorthand[i] == '^') {
      // -- Process super/subscript formatting
      mode_is_normal = !mode_is_normal;
      if(mode_is_normal) {
        // -- Back to normal mode
        munged_shorthand += "</SUP>";
      } else {
        // -- Going to superscript mode
        munged_shorthand += "<SUP>";
      }
    } else if(shorthand[i] == '\\') {
      // -- Process Symbol character set
      if((i + 1) < shorthand.length()) {
        i++;  // -- Proceed to next char. Yes I know that changing the loop var is ugly!
        munged_shorthand += "<B><FONT size=+1>";
        munged_shorthand += shorthand[i];
        munged_shorthand += "</FONT></B>";
      } else {
        // -- FIXME: Add line number info later
        cerr << "Encountered a `\\` without anything following it!" << endl;
        exit( -1 );
      }
    } else {
      // -- Pass on un-munged
      munged_shorthand += shorthand[i];
    }
  } // -- end for all characters in shorthand

  // -- Do any other munging
  if(!mode_is_normal) {
    // -- Back to normal mode
    munged_shorthand += "</SUP>";
  }

  // -- Return the formatted shorthand name
  return munged_shorthand;
}


