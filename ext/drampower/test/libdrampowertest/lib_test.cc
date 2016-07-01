/*
 * Copyright (c) 2014, TU Delft, TU Eindhoven and TU Kaiserslautern
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Matthias Jung, Omar Naji
 *
 */

#include <string>
#include "libdrampower/LibDRAMPower.h"

#if USE_XERCES
    #include "xmlparser/MemSpecParser.h"
#endif

using namespace std;
using namespace Data;

int main(int argc, char* argv[])
{
        assert(argc == 2);
        //Setup of DRAMPower for your simulation
        string filename;
        //type path to memspec file
        filename = argv[1];
        //Parsing the Memspec specification of found in memspec folder
        #if USE_XERCES
            MemorySpecification memSpec(MemSpecParser::getMemSpecFromXML(filename));
        #else
            MemorySpecification memSpec;
        #endif
        libDRAMPower test = libDRAMPower(memSpec, 0);
        // During the simulation you can report activity
        // to DRAMPower with the doCommand(...) function:
        test.doCommand(MemCommand::ACT,0,35);
        test.doCommand(MemCommand::RDA,0,50);
        test.doCommand(MemCommand::ACT,4,51);
        test.doCommand(MemCommand::RDA,4,66);
        test.doCommand(MemCommand::ACT,0,86);
        test.doCommand(MemCommand::RDA,0,101);
        test.doCommand(MemCommand::ACT,2,102);
        //This functionality is still not implemented.
        test.updateCounters(false);
        test.doCommand(MemCommand::RDA,2,117);
        test.doCommand(MemCommand::ACT,5,119);
        test.doCommand(MemCommand::RDA,5,134);
        test.doCommand(MemCommand::ACT,0,137);
        test.doCommand(MemCommand::RDA,0,152);
        test.doCommand(MemCommand::ACT,3,159);
        test.doCommand(MemCommand::RDA,3,174);
        test.doCommand(MemCommand::ACT,0,195);
        test.doCommand(MemCommand::RDA,0,210);
        test.doCommand(MemCommand::ACT,4,232);
        test.doCommand(MemCommand::WRA,4,247);
        // Need at least tWRAPDEN = AL + CWL + BL/2 + WR + 1 cycles between WR and PDN_F_PRE
        test.doCommand(MemCommand::PDN_F_PRE,3,265);

        //set bool to true when this is the last update of the counters
        test.updateCounters(true);

        // At the end of your simulation call the getEnergy(...)
        // function to print the power report
        test.calcEnergy();

        // Accesing the results:

        // Number of issued Commands
        std::cout << "# of acts" << "\t" <<test.counters.numberofacts << endl;
        std::cout << "# of reads" << "\t" <<test.counters.numberofreads << endl;
        std::cout << "# of precharges" << "\t" <<test.counters.numberofpres << endl;
        // many other timing parameters in test.mpm.timings

        //ENERGIES per Rank
        std::cout << "ACT Cmd Energy" << "\t" << test.getEnergy().act_energy << endl;
        std::cout << "PRE Cmd Energy" << "\t" << test.getEnergy().pre_energy << endl;
        std::cout << "Read Cmd Energy" << "\t" << test.getEnergy().read_energy << endl;
        std::cout << "Write Cmd Energy" << "\t" << test.getEnergy().write_energy << endl;
        //Standby Energy for 1 rank
        //In total energy calculated for both ranks= test.memSpec.memArchSpec *
        //test.getEnergy().act_stdby_energy
        std::cout << "ACT Std Energy" << "\t" << test.getEnergy().act_stdby_energy << endl;
        //total active standby energy for both ranks
        std::cout << "ACT Std Energy total ranks" << "\t" << static_cast<double>(memSpec.memArchSpec.nbrOfRanks) *
        test.getEnergy().act_stdby_energy << "\n" ;
        std::cout << "PRE Std Energy" << "\t" << test.getEnergy().pre_stdby_energy << endl;
        std::cout << "Total Energy" << "\t" << test.getEnergy().total_energy << endl;
        //many other energies in test.mpm.energy

        //Powers per Rank
        std::cout << "Average Power" << "\t" << test.getPower().average_power << endl;
        //many other powers in test.getPower()

        // Test clearState function.
        test.clearState();
        return 0;
}
