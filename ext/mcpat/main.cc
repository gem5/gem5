/*****************************************************************************
 *                                McPAT
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

#include "XML_Parse.h"
#include "globalvar.h"
#include "io.h"
#include "processor.h"
#include "version.h"
#include "xmlParser.h"

using namespace std;

void print_usage(char * argv0);

int main(int argc,char *argv[])
{
        char * fb ;
        bool infile_specified     = false;
        int  plevel               = 2;
        opt_for_clk	=true;
        //cout.precision(10);
        if (argc <= 1 || argv[1] == string("-h") || argv[1] == string("--help"))
        {
                print_usage(argv[0]);
        }

        for (int32_t i = 0; i < argc; i++)
        {
                if (argv[i] == string("-infile"))
                {
                        infile_specified = true;
                        i++;
                        fb = argv[ i];
                }

                if (argv[i] == string("-print_level"))
                {
                        i++;
                        plevel = atoi(argv[i]);
                }

                if (argv[i] == string("-opt_for_clk"))
                {
                        i++;
                        opt_for_clk = (bool)atoi(argv[i]);
                }
        }
        if (infile_specified == false)
        {
                print_usage(argv[0]);
        }


        cout<<"McPAT (version "<< VER_MAJOR <<"."<< VER_MINOR
                << " of " << VER_UPDATE << ") is computing the target processor...\n "<<endl;

        //parse XML-based interface
        ParseXML *p1= new ParseXML();
        p1->parse(fb);
        Processor proc(p1);
        proc.displayEnergy(2, plevel);
        delete p1;
        return 0;
}

void print_usage(char * argv0)
{
    cerr << "How to use McPAT:" << endl;
    cerr << "  mcpat -infile <input file name>  -print_level < level of details 0~5 >  -opt_for_clk < 0 (optimize for ED^2P only)/1 (optimzed for target clock rate)>"<< endl;
    //cerr << "    Note:default print level is at processor level, please increase it to see the details" << endl;
    exit(1);
}
