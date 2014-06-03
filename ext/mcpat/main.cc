/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *            Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/
#include <sys/stat.h>

#include <cassert>
#include <iostream>

#include "basic_components.h"
#include "io.h"
#include "system.h"
#include "version.h"
#include "xmlParser.h"

using namespace std;

void print_usage(char * argv0);

int main(int argc, char *argv[]) {
    char* xml_file = NULL;
    int plevel = 2;

    for (int32_t i = 0; i < argc; i++) {
        if (argv[i] == string("-infile")) {
            xml_file = argv[++i];

        } else if (argv[i] == string("-print_level")) {
            plevel = atoi(argv[++i]);

        } else if (argv[i] == string("-opt_for_clk")) {
            McPATComponent::opt_for_clk = (bool)atoi(argv[++i]);
        }
    }

    // Ensure that the XML file was specified
    if (xml_file == NULL) {
        cerr << "ERROR: Please specify infile\n\n";
        print_usage(argv[0]);
    }

    // Ensure that the XML file exists
    struct stat file_info;
    if (stat(xml_file, &file_info)) {
        cerr << "ERROR: File not found: " << xml_file << endl << endl;
        print_usage(argv[0]);
    }

    cout << "McPAT (version " << VER_MAJOR << "." << VER_MINOR
         << " of " << VER_UPDATE << ") is computing the target processor...\n "
         << endl;

    // Parse the XML input file
    XMLNode xml_data = XMLNode::openFileHelper(xml_file, "component");
    unsigned int num_children = xml_data.nChildNode("component");
    assert(num_children == 1);
    XMLNode system_xml = xml_data.getChildNode("component");
    assert(strcmp(system_xml.getAttribute("type"), "System") == 0);

    // Recursively instantiate the system hierarchy
    System* system = new System(&system_xml);

    // Recursively compute chip area
    system->computeArea();

    // Recursively compute the power consumed
    system->computeEnergy();

    // Recursively output the computed values
    system->displayData(2, plevel);

    // Clean up
    delete system;
    return 0;

}

void print_usage(char * argv0) {
    cerr << "How to use McPAT:" << endl;
    cerr << "  mcpat -infile <input file name>  -print_level < "
         << "level of details 0~5 >  -opt_for_clk < 0 (optimize for ED^2P "
         << "only)/1 (optimzed for target clock rate)>" << endl;
    exit(1);
}
