/*****************************************************************************
 *                                McPAT/CACTI
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *                          All Rights Reserved
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/

#include <iostream>

#include "io.h"

using namespace std;


int main(int argc,char *argv[])
{

  uca_org_t result;
  if (argc != 53 && argc != 55)
  {
    bool infile_specified = false;
    string infile_name("");

    for (int32_t i = 0; i < argc; i++)
    {
      if (argv[i] == string("-infile"))
      {
        infile_specified = true;
        i++;
        infile_name = argv[i];
      }
    }

    if (infile_specified == false)
    {
      cerr << " Invalid arguments -- how to use CACTI:" << endl;
      cerr << "  1) cacti -infile <input file name>" << endl;
      cerr << "  2) cacti arg1 ... arg52 -- please refer to the README file" << endl;
      cerr << " No. of arguments input - " << argc << endl;
      exit(1);
    }
    else
    {
      result = cacti_interface(infile_name);
    }
  }
  else if (argc == 53)
  {
          result = cacti_interface(atoi(argv[ 1]),
                          atoi(argv[ 2]),
                          atoi(argv[ 3]),
                          atoi(argv[ 4]),
                          atoi(argv[ 5]),
                          atoi(argv[ 6]),
                          atoi(argv[ 7]),
                          atoi(argv[ 8]),
                          atoi(argv[ 9]),
                          atof(argv[10]),
                          atoi(argv[11]),
                          atoi(argv[12]),
                          atoi(argv[13]),
                          atoi(argv[14]),
                          atoi(argv[15]),
                          atoi(argv[16]),
                          atoi(argv[17]),
                          atoi(argv[18]),
                          atoi(argv[19]),
                          atoi(argv[20]),
                          atoi(argv[21]),
                          atoi(argv[22]),
                          atoi(argv[23]),
                          atoi(argv[24]),
                          atoi(argv[25]),
                          atoi(argv[26]),
                          atoi(argv[27]),
                          atoi(argv[28]),
                          atoi(argv[29]),
                          atoi(argv[30]),
                          atoi(argv[31]),
                          atoi(argv[32]),
                          atoi(argv[33]),
                          atoi(argv[34]),
                          atoi(argv[35]),
                          atoi(argv[36]),
                          atoi(argv[37]),
                          atoi(argv[38]),
                          atoi(argv[39]),
                          atoi(argv[40]),
                          atoi(argv[41]),
                          atoi(argv[42]),
                          atoi(argv[43]),
                          atoi(argv[44]),
                          atoi(argv[45]),
                          atoi(argv[46]),
                          atoi(argv[47]),
                          atoi(argv[48]),
                          atoi(argv[49]),
                          atoi(argv[50]),
                          atoi(argv[51]),
                          atoi(argv[52]));
  }
  else
  {
          result = cacti_interface(atoi(argv[ 1]),
                          atoi(argv[ 2]),
                          atoi(argv[ 3]),
                          atoi(argv[ 4]),
                          atoi(argv[ 5]),
                          atoi(argv[ 6]),
                          atoi(argv[ 7]),
                          atoi(argv[ 8]),
                          atof(argv[ 9]),
                          atoi(argv[10]),
                          atoi(argv[11]),
                          atoi(argv[12]),
                          atoi(argv[13]),
                          atoi(argv[14]),
                          atoi(argv[15]),
                          atoi(argv[16]),
                          atoi(argv[17]),
                          atoi(argv[18]),
                          atoi(argv[19]),
                          atoi(argv[20]),
                          atoi(argv[21]),
                          atoi(argv[22]),
                          atoi(argv[23]),
                          atoi(argv[24]),
                          atoi(argv[25]),
                          atoi(argv[26]),
                          atoi(argv[27]),
                          atoi(argv[28]),
                          atoi(argv[29]),
                          atoi(argv[30]),
                          atoi(argv[31]),
                          atoi(argv[32]),
                          atoi(argv[33]),
                          atoi(argv[34]),
                          atoi(argv[35]),
                          atoi(argv[36]),
                          atoi(argv[37]),
                          atoi(argv[38]),
                          atoi(argv[39]),
                          atoi(argv[40]),
                          atoi(argv[41]),
                          atoi(argv[42]),
                          atoi(argv[43]),
                          atoi(argv[44]),
                          atoi(argv[45]),
                          atoi(argv[46]),
                          atoi(argv[47]),
                          atoi(argv[48]),
                          atoi(argv[49]),
                          atoi(argv[50]),
                          atoi(argv[51]),
                          atoi(argv[52]),
                          atoi(argv[53]),
                          atoi(argv[54]));
  }

  result.cleanup();
//  delete result.data_array2;
//  if (result.tag_array2!=NULL)
//	  delete result.tag_array2;

  return 0;
}

