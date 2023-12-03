// Copyright (c) 2006-2007 The Regents of The University of Michigan
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

////////////////////////////////////////////////////////////////////
//
// Nop instruction
//

// Per-cpu-model nop execute method.
def template NopExec{ {} };

output header{
    { /**
       * Nop class.
       */
      class Nop : public SparcStaticInst{
          public :
              // Constructor
              Nop(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
                  SparcStaticInst(mnem, _machInst,
                                  __opClass){ flags[IsNop] = true;
}

Fault
execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    return NoFault;
}

std::string
generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const override;
}
;
}
}
;

output decoder{ { std::string Nop::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab)
                      const { std::stringstream response;
printMnemonic(response, mnemonic);
return response.str();
}
}
}
;

def template NopExecute{
    { Fault %
      (class_name)s::execute(ExecContext * xc, trace::InstRecord *traceData)
          const { // Nothing to see here, move along
                  return NoFault;
}
}
}
;

// Primary format for integer operate instructions:
def format Nop(code, *opt_flags){
    { iop = InstObjParams(name, Name, 'Nop', code, opt_flags)
          header_output = BasicDeclare.subst(iop) decoder_output =
              BasicConstructor.subst(iop)
                  decode_block = BasicDecode.subst(iop)
                                     exec_output = NopExecute.subst(iop) }
};
