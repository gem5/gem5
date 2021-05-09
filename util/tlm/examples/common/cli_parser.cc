/*
 * Copyright (c) 2016, Dresden University of Technology (TU Dresden)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <sstream>

#include "cli_parser.hh"
#include "sim/cxx_manager.hh"

void
CliParser::usage(const std::string& prog_name)
{
    std::cerr
      << "Usage: " << prog_name
      << (" <config_file.ini> [ <option> ]\n\n"
          "OPTIONS:\n"

          "    -o <offset>                  -- set memory offset\n"
          "    -d <flag>                    -- set a gem5 debug flag\n"
          "                                    (-<flag> clears a flag)\n"
          "    -v                           -- verbose output\n"
          "    -e <ticks>                   -- end of simulation after a \n"
          "                                    given number of ticks\n"
          "\n");
    std::exit(EXIT_FAILURE);
}

void
CliParser::parse(int argc, char** argv)
{
    std::string prog_name(argv[0]);

    unsigned int arg_ptr = 1;

    if (argc == 1) {
        usage(prog_name);
    }

    configFile = std::string(argv[arg_ptr]);
    arg_ptr++;

    // default values
    verboseFlag = false;
    simulationEnd = 0;
    memoryOffset = 0;

    try {
        while (arg_ptr < argc) {
            std::string option(argv[arg_ptr]);
            arg_ptr++;
            unsigned num_args = argc - arg_ptr;

            if (option == "-d") {
                if (num_args < 1) {
                    usage(prog_name);
                }
                std::string flag(argv[arg_ptr]);
                arg_ptr++;
                debugFlags.push_back(flag);
            } else if (option == "-e") {
                if (num_args < 1) {
                    usage(prog_name);
                }
                std::istringstream(argv[arg_ptr]) >> simulationEnd;
                arg_ptr++;
            } else if (option == "-v") {
                verboseFlag = true;
            } else if (option == "-o") {
                if (num_args < 1) {
                    usage(prog_name);
                }
                std::istringstream(argv[arg_ptr]) >> memoryOffset;
                arg_ptr++;
                /* code */
            } else {
                usage(prog_name);
            }
        }
    } catch (gem5::CxxConfigManager::Exception &e) {
        std::cerr << e.name << ": " << e.message << "\n";
        std::exit(EXIT_FAILURE);
    }

    parsed = true;
}

std::string
CliParser::getDebugFlags()
{
    std::stringstream ss;
    for (auto& flag : debugFlags) {
        ss << flag << ' ';
    }
    return ss.str();
}
