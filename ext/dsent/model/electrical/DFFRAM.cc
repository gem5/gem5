/* Copyright (c) 2012 Massachusetts Institute of Technology
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "model/electrical/DFFRAM.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/timing_graph/ElectricalDriverMultiplier.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"
#include "model/electrical/Decoder.h"
#include "model/electrical/Multiplexer.h"

namespace DSENT
{
    using std::ceil;

    DFFRAM::DFFRAM(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    DFFRAM::~DFFRAM()
    {}

    void DFFRAM::initParameters()
    {
        addParameterName("NumberEntries");
        addParameterName("NumberBits");
        return;
    }

    void DFFRAM::initProperties()
    {
        return;
    }

    DFFRAM* DFFRAM::clone() const
    {
        // TODO
        return NULL;
    }

    void DFFRAM::constructModel()
    {
        // Get parameters
        unsigned int number_bits = getParameter("NumberBits").toUInt();
        unsigned int number_entries = getParameter("NumberEntries").toUInt();

        ASSERT(number_bits > 0, "[Error] " + getInstanceName() + 
            " -> Number of bits must be > 0!");
        ASSERT(number_entries > 0, "[Error] " + getInstanceName() + 
            " -> Number of entries must be > 0!");

        unsigned int number_addr_bits = (unsigned int)ceil(log2(number_entries));

        // Create ports
        createInputPort("In", makeNetIndex(0, number_bits-1));
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            createInputPort("WRAddr" + (String)i);
            createInputPort("RDAddr" + (String)i);
        }
        createInputPort("WE");
        createInputPort("CK");
        createOutputPort("Out", makeNetIndex(0, number_bits-1));

        // Create energy, power, and area results
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        getEventInfo("Idle")->setTransitionInfo("WE", TransitionInfo(1.0, 0.0, 0.0));

        createElectricalEventResult("Read");
        getEventInfo("Read")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        getEventInfo("Read")->setTransitionInfo("WE", TransitionInfo(1.0, 0.0, 0.0));
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            getEventInfo("Read")->setTransitionInfo("WRAddr" + (String)i, TransitionInfo(0.5, 0.0, 0.5));
        }
        createElectricalEventResult("Write");
        getEventInfo("Write")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        getEventInfo("Write")->setTransitionInfo("WE", TransitionInfo(0.0, 0.0, 1.0));
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            getEventInfo("Write")->setTransitionInfo("RDAddr" + (String)i, TransitionInfo(0.5, 0.0, 0.5));
        }

        // Init components - DFF array, Dec, Mux
        vector<String> dff_names(number_entries, "");
        vector<StdCell*> dffs(number_entries, NULL);
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            dff_names[i] = "DFF_" + (String)i;
            dffs[i] = getTechModel()->getStdCellLib()->createStdCell("DFFQ", dff_names[i]);
            dffs[i]->construct();
        }
                    
        const String& dec_name = "Dec";
        Decoder* dec = new Decoder(dec_name, getTechModel());
        dec->setParameter("NumberOutputs", number_entries);
        dec->construct();

        const String& mux_name = "Mux";
        Multiplexer* mux = new Multiplexer(mux_name, getTechModel());
        mux->setParameter("NumberInputs", number_entries);
        mux->setParameter("NumberBits", 1);
        mux->setParameter("BitDuplicate", "TRUE");
        mux->construct();

        // Init components - CK & WE
        const String& nand2cg0_name = "NAND2_CKGate0";
        StdCell* nand2cg0 = getTechModel()->getStdCellLib()->createStdCell("NAND2", nand2cg0_name);
        nand2cg0->construct();
        const String& invcg0_name = "INV_CKGate0";
        StdCell* invcg0 = getTechModel()->getStdCellLib()->createStdCell("INV", invcg0_name);
        invcg0->construct();

        // Init components - (CK & WE) & DecOut[i]
        vector<String> nand2cg1_names(number_entries, "");
        vector<StdCell*> nand2cg1s(number_entries, NULL);
        vector<String> invcg1_names(number_entries, "");
        vector<StdCell*> invcg1s(number_entries, NULL);
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            nand2cg1_names[i] = "NAND2_CKGate1_" + (String)i;
            nand2cg1s[i] = getTechModel()->getStdCellLib()->createStdCell("NAND2", nand2cg1_names[i]);
            nand2cg1s[i]->construct();

            invcg1_names[i] = "INV_CKGate1_" + (String)i;
            invcg1s[i] = getTechModel()->getStdCellLib()->createStdCell("INV", invcg1_names[i]);
            invcg1s[i]->construct();
        }

        // Connect Decoder
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            portConnect(dec, "Addr" + (String)i, "WRAddr" + (String)i);
        }
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            createNet("Dec_Out" + (String)i);
            portConnect(dec, "Out" + (String)i, "Dec_Out" + (String)i);
        }

        // Connect CKGate0 - CK, WE
        createNet("NAND2_CKGate0_Out");
        createNet("CKGate0_Out");
        portConnect(nand2cg0, "A", "CK");
        portConnect(nand2cg0, "B", "WE");
        portConnect(nand2cg0, "Y", "NAND2_CKGate0_Out");
        portConnect(invcg0, "A", "NAND2_CKGate0_Out");
        portConnect(invcg0, "Y", "CKGate0_Out");

        // Connect CKGate1 - CKGate0, Dec_Out
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            createNet("NAND2_CKGate1_Outs" + (String)i);
            createNet("CKGate1_Outs" + (String)i);
            portConnect(nand2cg1s[i], "A", "CKGate0_Out");
            portConnect(nand2cg1s[i], "B", "Dec_Out" + (String)i);
            portConnect(nand2cg1s[i], "Y", "NAND2_CKGate1_Outs" + (String)i);
            portConnect(invcg1s[i], "A", "NAND2_CKGate1_Outs" + (String)i);
            portConnect(invcg1s[i], "Y", "CKGate1_Outs" + (String)i);
        }

        // Connect DFF array
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            createNet("DFF_Out" + (String)i);
            for(unsigned int n = 0; n < number_bits; ++n)
            {
                portConnect(dffs[i], "D", "In", makeNetIndex(n));
                portConnect(dffs[i], "CK", "CKGate1_Outs" + (String)i);
            }
            portConnect(dffs[i], "Q", "DFF_Out" + (String)i);
        }

        // Connect Multiplexer
        createNet("Mux_Out");
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            portConnect(mux, "In" + (String)i, "DFF_Out" + (String)i);
        }
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            portConnect(mux, "Sel" + (String)i, "RDAddr" + (String)i);
        }
        portConnect(mux, "Out", "Mux_Out");

        // Use driver multiplier to connect Mux_Out to Out
        createDriverMultiplier("OutMult");
        ElectricalDriverMultiplier* drive_mult = getDriverMultiplier("OutMult");
        getNet("Mux_Out")->addDownstreamNode(drive_mult);
        for(unsigned int n = 0; n < number_bits; ++n)
        {
            drive_mult->addDownstreamNode(getNet("Out", makeNetIndex(n)));
        }

        // Add area and power results
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            addSubInstances(dffs[i], number_bits);
            addElectricalSubResults(dffs[i], number_bits);
        }

        addSubInstances(dec, 1.0);
        addElectricalSubResults(dec, 1.0);

        addSubInstances(mux, number_bits);
        addElectricalSubResults(mux, number_bits);

        addSubInstances(nand2cg0, 1.0);
        addElectricalSubResults(nand2cg0, 1.0);

        addSubInstances(invcg0, 1);
        addElectricalSubResults(invcg0, 1.0);

        for(unsigned int i = 0; i < number_entries; ++i)
        {
            addSubInstances(nand2cg1s[i], 1);
            addElectricalSubResults(nand2cg1s[i], 1.0);

            addSubInstances(invcg1s[i], 1);
            addElectricalSubResults(invcg1s[i], 1.0);
        }

        // Add write event
        Result* write_event = getEventResult("Write");
        write_event->addSubResult(nand2cg0->getEventResult("NAND2"), nand2cg0_name, 1.0);
        write_event->addSubResult(invcg0->getEventResult("INV"), invcg0_name, 1.0);
        write_event->addSubResult(dec->getEventResult("Decode"), dec_name, 1.0);
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            write_event->addSubResult(nand2cg1s[i]->getEventResult("NAND2"), nand2cg1_names[i], 1.0);
            write_event->addSubResult(invcg1s[i]->getEventResult("INV"), invcg1_names[i], 1.0);
            write_event->addSubResult(dffs[i]->getEventResult("DFFD"), dff_names[i], number_bits);
            write_event->addSubResult(dffs[i]->getEventResult("DFFQ"), dff_names[i], number_bits);
            write_event->addSubResult(dffs[i]->getEventResult("CK"), dff_names[i], number_bits);
        }

        // Add read event
        Result* read_event = getEventResult("Read");
        //for(unsigned int i = 0; i < number_entries; ++i)
        //{
        //    read_event->addSubResult(dffs[i]->getEventResult("DFFQ"), dff_names[i], number_bits);
        //}
        read_event->addSubResult(mux->getEventResult("Mux"), mux_name, number_bits);

        return;
    }

    void DFFRAM::propagateTransitionInfo()
    {
        // Update probability
        unsigned int number_entries = (unsigned int)getParameter("NumberEntries");
        unsigned int number_addr_bits = (unsigned int)ceil(log2(number_entries));

        // Update decoder
        ElectricalModel* dec = (ElectricalModel*)getSubInstance("Dec");
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            propagatePortTransitionInfo(dec, "Addr" + (String)i, "WRAddr" + (String)i);
        }
        dec->use();

        // Update CKGate0 nands + invs
        ElectricalModel* nand2cg0 = (ElectricalModel*)getSubInstance("NAND2_CKGate0");
        propagatePortTransitionInfo(nand2cg0, "A", "CK");
        propagatePortTransitionInfo(nand2cg0, "B", "WE");
        nand2cg0->use();
        ElectricalModel* invcg0 = (ElectricalModel*)getSubInstance("INV_CKGate0");
        propagatePortTransitionInfo(invcg0, "A", nand2cg0, "Y");
        invcg0->use();

        // Update CKGate1 nands + invs
        vector<ElectricalModel*> nand2cg1s(number_entries, NULL);
        vector<ElectricalModel*> invcg1s(number_entries, NULL);
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            nand2cg1s[i] = (ElectricalModel*)getSubInstance("NAND2_CKGate1_" + (String)i);
            propagatePortTransitionInfo(nand2cg1s[i], "A", invcg0, "Y");
            propagatePortTransitionInfo(nand2cg1s[i], "B", dec, "Out" + (String)i);
            nand2cg1s[i]->use();

            invcg1s[i] = (ElectricalModel*)getSubInstance("INV_CKGate1_" + (String)i);
            propagatePortTransitionInfo(invcg1s[i], "A", nand2cg1s[i], "Y");
            invcg1s[i]->use();
        }

        // Update DFF
        vector<ElectricalModel*> dffs(number_entries, NULL);
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            dffs[i] = (ElectricalModel*)getSubInstance("DFF_" + (String)i);
            propagatePortTransitionInfo(dffs[i], "D", "In");
            propagatePortTransitionInfo(dffs[i], "CK", invcg1s[i], "Y");
            dffs[i]->use();
        }

        // Update Mux
        ElectricalModel* mux = (ElectricalModel*)getSubInstance("Mux");
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            propagatePortTransitionInfo(mux, "In" + (String)i, dffs[i], "Q");
        }
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            propagatePortTransitionInfo(mux, "Sel" + (String)i, "RDAddr" + (String)i);
        }
        mux->use();

        // Set output probability
        getOutputPort("Out")->setTransitionInfo(mux->getOutputPort("Out")->getTransitionInfo());
        return;
    }
} // namespace DSENT

