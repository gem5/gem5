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
 * NetworkConfig.hh
 *
 * Description: This header file is used to define all configuration parameters required by the interconnection network.
 *
 * Niket Agarwal, Princeton University
 *
 * */

#ifndef NETWORKCONFIG_H
#define NETWORKCONFIG_H

#include "mem/ruby/network/garnet-fixed-pipeline/NetworkHeader.hh"
#include "mem/gems_common/util.hh"
#include "mem/ruby/config/RubyConfig.hh"

class NetworkConfig {
        public:
                static bool isGarnetNetwork() {return g_GARNET_NETWORK; }
                static bool isDetailNetwork() {return g_DETAIL_NETWORK; }
                static int isNetworkTesting() {return g_NETWORK_TESTING; }
                static int getFlitSize() {return g_FLIT_SIZE; }
                static int getNumPipeStages() {return g_NUM_PIPE_STAGES; }
                static int getVCsPerClass() {return g_VCS_PER_CLASS; }
                static int getBufferSize() {return g_BUFFER_SIZE; }
  // This is no longer used. See config/rubyconfig.defaults to set Garnet parameters.
                static void readNetConfig()
                {
      /*
                        string filename = "network/garnet-flexible-pipeline/";
                        filename += NETCONFIG_DEFAULTS;

                        ifstream NetconfigFile( filename.c_str(), ios::in);
                        if(!NetconfigFile.is_open())
                        {
                                cout << filename << endl;
                                cerr << "Network Configuration file cannot be opened\n";
                                exit(1);
                        }

                        string line = "";

                        while(!NetconfigFile.eof())
                        {
                                getline(NetconfigFile, line, '\n');
                                string var = string_split(line, ':');

                                if(!var.compare("g_GARNET_NETWORK"))
                                {
                                        if(!line.compare("true"))
                                                g_GARNET_NETWORK = true;
                                        else
                                                g_GARNET_NETWORK = false;
                                }
                                if(!var.compare("g_DETAIL_NETWORK"))
                                {
                                        if(!line.compare("true"))
                                                g_DETAIL_NETWORK = true;
                                        else
                                                g_DETAIL_NETWORK = false;
                                }
                                if(!var.compare("g_NETWORK_TESTING"))
                                {
                                        if(!line.compare("true"))
                                                g_NETWORK_TESTING = true;
                                        else
                                                g_NETWORK_TESTING = false;
                                }
                                if(!var.compare("g_FLIT_SIZE"))
                                        g_FLIT_SIZE = atoi(line.c_str());
                                if(!var.compare("g_NUM_PIPE_STAGES"))
                                        g_NUM_PIPE_STAGES = atoi(line.c_str());
                                if(!var.compare("g_VCS_PER_CLASS"))
                                        g_VCS_PER_CLASS = atoi(line.c_str());
                                if(!var.compare("g_BUFFER_SIZE"))
                                        g_BUFFER_SIZE = atoi(line.c_str());
                        }
                        NetconfigFile.close();
      */
      /*
      cout << "g_GARNET_NETWORK = " << g_GARNET_NETWORK << endl;
      cout << "g_DETAIL_NETWORK = " << g_DETAIL_NETWORK << endl;
      cout << "g_NETWORK_TESTING = " << g_NETWORK_TESTING << endl;
      cout << "g_FLIT_SIZE = " << g_FLIT_SIZE << endl;
      cout << "g_NUM_PIPE_STAGES = " << g_NUM_PIPE_STAGES << endl;
      cout << "g_VCS_PER_CLASS= " << g_VCS_PER_CLASS << endl;
      cout << "g_BUFFER_SIZE = " << g_BUFFER_SIZE << endl;
      */
                }
};

#endif
