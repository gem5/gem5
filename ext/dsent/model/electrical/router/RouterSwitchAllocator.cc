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

#include "model/electrical/router/RouterSwitchAllocator.h"

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/ModelGen.h"
#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"

namespace DSENT
{
    RouterSwitchAllocator::RouterSwitchAllocator(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    RouterSwitchAllocator::~RouterSwitchAllocator()
    {}

    void RouterSwitchAllocator::initParameters()
    {
        addParameterName("NumberInputPorts");
        addParameterName("NumberOutputPorts");
        addParameterName("TotalNumberVirtualChannels");
        addParameterName("ArbiterModel");
        return;
    }

    void RouterSwitchAllocator::initProperties()
    {}

    RouterSwitchAllocator* RouterSwitchAllocator::clone() const
    {
        // TODO
        return NULL;
    }

    void RouterSwitchAllocator::constructModel()
    {
        // Get parameters
        unsigned int number_input_ports = getParameter("NumberInputPorts").toUInt();
        unsigned int number_output_ports = getParameter("NumberOutputPorts").toUInt();
        unsigned int total_number_vcs = getParameter("TotalNumberVirtualChannels").toUInt();
        const String& arb_model = getParameter("ArbiterModel");

        ASSERT(number_input_ports > 0, "[Error] " + getInstanceName() +
                " -> Number of input ports must be > 0!");
        ASSERT(number_output_ports > 0, "[Error] " + getInstanceName() +
                " -> Number of output ports must be > 0!");
        ASSERT(total_number_vcs > 0, "[Error] " + getInstanceName() +
                " -> Total number of virtual channels must be > 0!");

        unsigned int stage1_number_requests = total_number_vcs;
        unsigned int number_stage1_arbiters = number_input_ports;
        unsigned int stage2_number_requests = number_input_ports;
        unsigned int number_stage2_arbiters = number_output_ports;

        getGenProperties()->set("NumberStage1Arbiters", number_stage1_arbiters);
        getGenProperties()->set("Stage1->NumberRequests", stage1_number_requests);
        getGenProperties()->set("NumberStage2Arbiters", number_stage2_arbiters);
        getGenProperties()->set("Stage2->NumberRequests", stage2_number_requests);

        // Create ports
        createInputPort("CK");
        for(unsigned int i = 0; i < number_stage1_arbiters; ++i)
        {
            for(unsigned int j = 0; j < stage1_number_requests; ++j)
            {
                createInputPort(String::format("Stage1Arb%d->Request%d", i, j));
                createInputPort(String::format("Stage1Arb%d->Grant%d", i, j));
            }
        }
        for(unsigned int i = 0; i < number_stage2_arbiters; ++i)
        {
            for(unsigned int j = 0; j < stage2_number_requests; ++j)
            {
                createInputPort(String::format("Stage2Arb%d->Request%d", i, j));
                createInputPort(String::format("Stage2Arb%d->Grant%d", i, j));
            }
        }

        // Create area, power, and event results
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));

        createElectricalEventResult("ArbitrateStage1");
        getEventInfo("ArbitrateStage1")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        createElectricalEventResult("ArbitrateStage2");
        getEventInfo("ArbitrateStage2")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));

        // Init Stage1 arbiter
        vector<String> stage1_arb_dff_names(stage1_number_requests, "");
        vector<StdCell*> stage1_arb_dffs(stage1_number_requests, NULL);
        for(unsigned int i = 0; i < stage1_number_requests; ++i)
        {
            stage1_arb_dff_names[i] = "Stage1ArbDFF" + (String)i;
            stage1_arb_dffs[i] = getTechModel()->getStdCellLib()->createStdCell("DFFQ", stage1_arb_dff_names[i]);
            stage1_arb_dffs[i]->construct();
        }
        const String& stage1_arb_name = "Stage1Arb";
        ElectricalModel* stage1_arb = (ElectricalModel*)ModelGen::createModel(arb_model, stage1_arb_name, getTechModel());
        stage1_arb->setParameter("NumberRequests", stage1_number_requests);
        stage1_arb->construct();

        // Init stage2 arbiter
        vector<String> stage2_arb_dff_names(stage2_number_requests, "");
        vector<StdCell*> stage2_arb_dffs(stage2_number_requests, NULL);
        for(unsigned int i = 0; i < stage2_number_requests; ++i)
        {
            stage2_arb_dff_names[i] = "Stage2ArbDFF" + (String)i;
            stage2_arb_dffs[i] = getTechModel()->getStdCellLib()->createStdCell("DFFQ", stage2_arb_dff_names[i]);
            stage2_arb_dffs[i]->construct();
        }
        const String& stage2_arb_name = "Stage2Arb";
        ElectricalModel* stage2_arb = (ElectricalModel*)ModelGen::createModel(arb_model, stage2_arb_name, getTechModel());
        stage2_arb->setParameter("NumberRequests", stage2_number_requests);
        stage2_arb->construct();

        // Connect ports
        for(unsigned int i = 0; i < stage1_number_requests; ++i)
        {
            const String& dff_in_name = "Stage1Arb_DFF_In" + (String)i;
            const String& req_name = "Stage1Arb->Request" + (String)i;
            const String& grant_name = "Stage1Arb->Grant" + (String)i;
            createNet(dff_in_name);
            createNet(req_name);
            createNet(grant_name);
            portConnect(stage1_arb_dffs[i], "D", dff_in_name);
            portConnect(stage1_arb_dffs[i], "CK", "CK");
            portConnect(stage1_arb_dffs[i], "Q", req_name);
            portConnect(stage1_arb, "Request" + (String)i, req_name);
            portConnect(stage1_arb, "Grant" + (String)i, grant_name);
            for(unsigned int j = 0; j < number_stage1_arbiters; ++j)
            {
                assignVirtualFanin(dff_in_name, String::format("Stage1Arb%d->Request%d", j, i));
                assignVirtualFanout(String::format("Stage1Arb%d->Grant%d", j, i), grant_name);
            }
        }
        for(unsigned int i = 0; i < stage2_number_requests; ++i)
        {
            const String& dff_in_name = "Stage2Arb_DFF_In" + (String)i;
            const String& req_name = "Stage2Arb->Request" + (String)i;
            const String& grant_name = "Stage2Arb->Grant" + (String)i;
            createNet(dff_in_name);
            createNet(req_name);
            createNet(grant_name);
            portConnect(stage2_arb_dffs[i], "D", dff_in_name);
            portConnect(stage2_arb_dffs[i], "CK", "CK");
            portConnect(stage2_arb_dffs[i], "Q", req_name);
            portConnect(stage2_arb, "Request" + (String)i, req_name);
            portConnect(stage2_arb, "Grant" + (String)i, grant_name);
            for(unsigned int j = 0; j < number_stage2_arbiters; ++j)
            {
                assignVirtualFanin(dff_in_name, String::format("Stage2Arb%d->Request%d", j, i));
                assignVirtualFanout(String::format("Stage2Arb%d->Grant%d", j, i), grant_name);
            }
        }

        // Add sub components
        for(unsigned int i = 0; i < stage1_number_requests; ++i)
        {
            addSubInstances(stage1_arb_dffs[i], 1.0);
            addElectricalSubResults(stage1_arb_dffs[i], 1.0);
        }
        addSubInstances(stage1_arb, number_stage1_arbiters);
        addElectricalSubResults(stage1_arb, number_stage1_arbiters);
        for(unsigned int i = 0; i < stage2_number_requests; ++i)
        {
            addSubInstances(stage2_arb_dffs[i], 1.0);
            addElectricalSubResults(stage2_arb_dffs[i], 1.0);
        }
        addSubInstances(stage2_arb, number_stage2_arbiters);
        addElectricalSubResults(stage2_arb, number_stage2_arbiters);

        // Update stage1 arb arbitrate
        getEventResult("ArbitrateStage1")->addSubResult(stage1_arb->getEventResult("Arbitrate"), stage1_arb_name, 1.0);

        // Update stage2 arb arbitrate
        getEventResult("ArbitrateStage2")->addSubResult(stage2_arb->getEventResult("Arbitrate"), stage2_arb_name, 1.0);
        return;
    }

    void RouterSwitchAllocator::propagateTransitionInfo()
    {
        ElectricalModel* stage1_arb = (ElectricalModel*)getSubInstance("Stage1Arb");
        stage1_arb->applyTransitionInfo("Arbitrate");
        stage1_arb->use();

        ElectricalModel* stage2_arb = (ElectricalModel*)getSubInstance("Stage2Arb");
        stage2_arb->applyTransitionInfo("Arbitrate");
        stage2_arb->use();

        return;
    }
} // namespace DSENT

