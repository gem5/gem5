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

#include <iostream.h>
#include <string>
#include <vector>

#include "base/str.hh"

int
main(int argc, char *argv[])
{
  if (argc != 3) {
    cout << "Usage: " << argv[0] << " <string> <token>\n";
    exit(1);
  }

  int i;
  string test = argv[1];
  vector<string> tokens1;
  vector<string> tokens2;
  char token = argv[2][0];

  cout << "string = \"" << test << "\", token = \'" << token << "\'\n";
  cout << "testing without ignore\n";
  tokenize(tokens1, test, token, false);

  if (tokens1.size()) {
      for (i = 0; i < tokens1.size() - 1; i++)
          cout << tokens1[i] << "(" << tokens1[i].size() << "), ";
      cout << tokens1[i]  << "(" << tokens1[i].size() << ")\n";
  }

  cout << "testing with ignore\n";
  tokenize(tokens2, test, token, true);

  if (tokens2.size()) {
      for (i = 0; i < tokens2.size() - 1; i++)
          cout << tokens2[i] << "(" << tokens2[i].size() << "), ";
      cout << tokens2[i] << "(" << tokens2[i].size() << ")\n";
  }

  return 0;
}
