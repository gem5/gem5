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
 *
 * Authors: Christian Menard
 */

#include <systemc>
#include <tlm>

#include "sc_master_port.hh"
#include "sim_control.hh"
#include "stats.hh"
#include "traffic_generator.hh"

// Defining global string variable decalred in stats.hh
std::string filename;

void
reportHandler(const sc_core::sc_report& report,
              const sc_core::sc_actions& actions)
{
    uint64_t systemc_time = report.get_time().value();
    uint64_t gem5_time = curTick();

    std::cerr << report.get_time();

    if (gem5_time < systemc_time) {
        std::cerr << " (<) ";
    } else if (gem5_time > systemc_time) {
        std::cerr << " (!) ";
    } else {
        std::cerr << " (=) ";
    }

    std::cerr << ": " << report.get_msg_type() << ' ' << report.get_msg()
              << '\n';
}

int
sc_main(int argc, char** argv)
{
    sc_core::sc_report_handler::set_handler(reportHandler);

    SimControl simControl("gem5", argc, argv);
    TrafficGenerator trafficGenerator("traffic_generator");

    filename = "m5out/stats-systemc.txt";

    tlm::tlm_target_socket<>* mem_port =
      dynamic_cast<tlm::tlm_target_socket<>*>(
        sc_core::sc_find_object("gem5.memory"));

    if (mem_port) {
        SC_REPORT_INFO("sc_main", "Port Found");
        trafficGenerator.socket.bind(*mem_port);
    } else {
        SC_REPORT_FATAL("sc_main", "Port Not Found");
        std::exit(EXIT_FAILURE);
    }

    std::cout << "Starting sc_main" << std::endl;

    sc_core::sc_start(); // Run to end of simulation

    SC_REPORT_INFO("sc_main", "End of Simulation");

    CxxConfig::statsDump();

    return EXIT_SUCCESS;
}
