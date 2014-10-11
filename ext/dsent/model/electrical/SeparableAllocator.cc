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

#include "model/electrical/SeparableAllocator.h"

#include "model/ModelGen.h"
#include "model/timing_graph/ElectricalNet.h"

namespace DSENT
{
    SeparableAllocator::SeparableAllocator(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    SeparableAllocator::~SeparableAllocator()
    {}

    void SeparableAllocator::initParameters()
    {
        addParameterName("NumberRequesters");
        addParameterName("NumberResources");
        addParameterName("IsRequesterFirst", true);
        addParameterName("Stage1->ArbiterModel");
        addParameterName("Stage2->ArbiterModel");
        return;
    }

    void SeparableAllocator::initProperties()
    {
        addPropertyName("P(Request)");
        addPropertyName("Act(Request)");
        addPropertyName("P(CK)");
        addPropertyName("Act(CK)");
        return;
    }

    SeparableAllocator* SeparableAllocator::clone() const
    {
        // TODO
        return NULL;
    }

    void SeparableAllocator::constructModel()
    {
        // Get parameters
        unsigned int number_requesters = getParameter("NumberRequesters").toUInt();
        unsigned int number_resources = getParameter("NumberResources").toUInt();
        bool is_requester_first = getParameter("IsRequesterFirst").toBool();
        const String& stage1_arbiter_model = getParameter("Stage1->ArbiterModel");
        const String& stage2_arbiter_model = getParameter("Stage2->ArbiterModel");

        ASSERT(number_requesters > 0, "[Error] " + getInstanceName() + 
                " -> Number of requesters must be > 0!");
        ASSERT(number_resources > 0, "[Error] " + getInstanceName() +
                " -> Number of resources must be > 0!");

        // Create area, power, and event results
        createElectricalResults();
        addEventResult(new Result("Allocate"));

        // Create ports
        createInputPort("CK");
        for(unsigned int i = 0; i < number_requesters; ++i)
        {
            createInputPort("Request" + (String)i, makeNetIndex(0, number_resources-1));
            createOutputPort("Grant" + (String)i, makeNetIndex(0, number_resources-1));
        }

        // If is_requester_first is set, requests from the same requester will be arbitrate 
        // on stage 1
        if(is_requester_first)
        {
            // Init stage 1 arbiters
            for(unsigned int i = 0; i < number_requesters; ++i)
            {
                ElectricalModel* arb = (ElectricalModel*)ModelGen::createModel(stage1_arbiter_model, "Stage1Arb" + (String)i, getTechModel());
                arb->setParameter("NumberRequests", number_resources);
                arb->construct();

                addSubInstances(arb, 1.0);
                addElectricalSubResults(arb, 1.0);

                getEventResult("Allocate")->addSubResult(arb->getEventResult("Arbitrate"), arb->getInstanceName(), 1.0);

                createNet("Stage1Arb_In" + (String)i, makeNetIndex(0, number_resources-1));
                createNet("Stage1Arb_Out" + (String)i, makeNetIndex(0, number_resources-1));

                portConnect(arb, "CK", "CK");
                assign("Stage1Arb_In" + (String)i, "Request" + (String)i);
                for(unsigned int j = 0; j < number_resources; ++j)
                {
                    portConnect(arb, "Request" + (String)j, "Stage1Arb_In" + (String)i, makeNetIndex(j));
                    portConnect(arb, "Grant" + (String)j, "Stage1Arb_Out" + (String)i, makeNetIndex(j));
                }
            }

            // Init stage 2 arbiters
            for(unsigned int i = 0; i < number_resources; ++i)
            {
                ElectricalModel* arb = (ElectricalModel*)ModelGen::createModel(stage2_arbiter_model, "Stage2Arb" + (String)i, getTechModel());
                arb->setParameter("NumberRequests", number_requesters);
                arb->construct();

                addSubInstances(arb, 1.0);
                addElectricalSubResults(arb, 1.0);

                getEventResult("Allocate")->addSubResult(arb->getEventResult("Arbitrate"), arb->getInstanceName(), 1.0);

                createNet("Stage2Arb_In" + (String)i, makeNetIndex(0, number_requesters-1));
                createNet("Stage2Arb_Out" + (String)i, makeNetIndex(0, number_requesters-1));

                portConnect(arb, "CK", "CK");
                for(unsigned int j = 0; j < number_requesters; ++j)
                {
                    assign("Stage2Arb_In" + (String)i, makeNetIndex(j), "Stage1Arb_Out" + (String)j, makeNetIndex(i));
                    portConnect(arb, "Request" + (String)j, "Stage2Arb_In" + (String)i, makeNetIndex(j));
                    portConnect(arb, "Grant" + (String)j, "Stage2Arb_Out" + (String)i, makeNetIndex(j));
                    assign("Grant" + (String)j, makeNetIndex(i), "Stage2Arb_Out" + (String)i, makeNetIndex(j));
                }
            }
        }
        else
        {
            // Init stage 1 arbiters
            for(unsigned int i = 0; i < number_resources; ++i)
            {
                ElectricalModel* arb = (ElectricalModel*)ModelGen::createModel(stage1_arbiter_model, "Stage1Arb" + (String)i, getTechModel());
                arb->setParameter("NumberRequests", number_requesters);
                arb->construct();

                addSubInstances(arb, 1.0);
                addElectricalSubResults(arb, 1.0);

                getEventResult("Allocate")->addSubResult(arb->getEventResult("Arbitrate"), arb->getInstanceName(), 1.0);

                createNet("Stage1Arb_In" + (String)i, makeNetIndex(0, number_requesters-1));
                createNet("Stage1Arb_Out" + (String)i, makeNetIndex(0, number_requesters-1));

                portConnect(arb, "CK", "CK");
                for(unsigned int j = 0; j < number_requesters; ++j)
                {
                    assign("Stage1Arb_In" + (String)i, makeNetIndex(j), "Request" + (String)j, makeNetIndex(i));
                    portConnect(arb, "Request" + (String)j, "Stage1Arb_In" + (String)i, makeNetIndex(j));
                    portConnect(arb, "Grant" + (String)j, "Stage1Arb_Out" + (String)i, makeNetIndex(j));
                }
            }

            // Init stage 2 arbiters
            for(unsigned int i = 0; i < number_requesters; ++i)
            {
                ElectricalModel* arb = (ElectricalModel*)ModelGen::createModel(stage2_arbiter_model, "Stage2Arb" + (String)i, getTechModel());
                arb->setParameter("NumberRequests", number_requesters);
                arb->construct();

                addSubInstances(arb, 1.0);
                addElectricalSubResults(arb, 1.0);

                getEventResult("Allocate")->addSubResult(arb->getEventResult("Arbitrate"), arb->getInstanceName(), 1.0);

                createNet("Stage2Arb_In" + (String)i, makeNetIndex(0, number_resources-1));
                createNet("Stage2Arb_Out" + (String)i, makeNetIndex(0, number_resources-1));

                portConnect(arb, "CK", "CK");
                for(unsigned int j = 0; j < number_resources; ++j)
                {
                    assign("Stage2Arb_In" + (String)i, makeNetIndex(j), "Stage1Arb_Out" + (String)j, makeNetIndex(i));
                    portConnect(arb, "Request" + (String)j, "Stage2Arb_In", makeNetIndex(j));
                    portConnect(arb, "Grant" + (String)j, "Stage2Arb_Out", makeNetIndex(j));
                }
                assign("Grant" + (String)i, "Stage2Arb_Out" + (String)i);
            }
        }
        return;
    }

    void SeparableAllocator::updateModel()
    {
        // Get parameters
        unsigned int number_requesters = getParameter("NumberRequesters").toUInt();
        unsigned int number_resources = getParameter("NumberResources").toUInt();
        bool is_requester_first = getParameter("IsRequesterFirst").toBool();

        // Get probabilities from inputs
        const String& P_request = getProperty("P(Request)");
        const String& act_request = getProperty("Act(Request)");
        const String& P_CK = getProperty("P(CK)");
        const String& act_CK = getProperty("Act(CK)");

        const vector<double>& P_request_vector = LibUtil::castStringVector<double>(P_request.split("[,]"));
        const vector<double>& act_request_vector = LibUtil::castStringVector<double>(act_request.split("[,]"));

        ASSERT(P_request_vector.size() == (number_requesters * number_resources), "[Error] " + getInstanceName() +
                " -> Expecting " + (String)(number_requesters * number_resources) + 
                " request probabilities, but got " + P_request);
        ASSERT(act_request_vector.size() == (number_requesters * number_resources), "[Error] " + getInstanceName() +
                " -> Expecting " + (String)(number_requesters * number_resources) + 
                " request actvities multiplier, but got " + act_request);

        vector<double> P_int_request_vector(number_requesters * number_resources, 0.0);
        vector<double> act_int_request_vector(number_requesters * number_resources, 0.0);
        vector<double> P_out_request_vector(number_requesters * number_resources, 0.0);
        vector<double> act_out_request_vector(number_requesters * number_resources, 0.0);
        if(is_requester_first)
        {
            // Update stage1 arbiter 
            for(unsigned int i = 0; i < number_requesters; ++i)
            {
                vector<double> P_arb_request_vector(number_resources, 0.0);
                vector<double> act_arb_request_vector(number_resources, 0.0);
                for(unsigned int j = 0; j < number_resources; ++j)
                {
                    P_arb_request_vector[j] = P_request_vector[i * number_resources + j];
                    act_arb_request_vector[j] = act_request_vector[i * number_resources + j];
                }

                Model* arb = getSubInstance("Stage1Arb" + (String)i);
                arb->setProperty("P(Request)", LibUtil::vectorToString(P_arb_request_vector));
                arb->setProperty("Act(Request)", LibUtil::vectorToString(act_arb_request_vector));
                arb->setProperty("P(CK)", P_CK);
                arb->setProperty("Act(CK)", act_CK);
                arb->update();

                const vector<double>& P_arb_out_request_vector = LibUtil::castStringVector<double>(arb->getGenProperties()->get("P(Grant)").split("[,]"));
                const vector<double>& act_arb_out_request_vector = LibUtil::castStringVector<double>(arb->getGenProperties()->get("Act(Grant)").split("[,]"));
                for(unsigned int j = 0; j < number_resources; ++j)
                {
                    P_int_request_vector[i * number_resources + j] = P_arb_out_request_vector[j];
                    act_int_request_vector[i * number_resources + j] = act_arb_out_request_vector[j];
                }
            }
            // Update stage2 arbiter
            for(unsigned int i = 0; i < number_resources; ++i)
            {
                vector<double> P_arb_request_vector(number_requesters, 0.0);
                vector<double> act_arb_request_vector(number_requesters, 0.0);
                for(unsigned int j = 0; j < number_requesters; ++j)
                {
                    P_arb_request_vector[j] = P_int_request_vector[j * number_resources + i];
                    act_arb_request_vector[j] = act_int_request_vector[j * number_resources + i];
                }

                Model* arb = getSubInstance("Stage2Arb" + (String)i);
                arb->setProperty("P(Request)", LibUtil::vectorToString(P_arb_request_vector));
                arb->setProperty("Act(Request)", LibUtil::vectorToString(act_arb_request_vector));
                arb->setProperty("P(CK)", P_CK);
                arb->setProperty("Act(CK)", act_CK);
                arb->update();

                const vector<double>& P_arb_out_request_vector = LibUtil::castStringVector<double>(arb->getGenProperties()->get("P(Grant)").split("[,]"));
                const vector<double>& act_arb_out_request_vector = LibUtil::castStringVector<double>(arb->getGenProperties()->get("Act(Grant)").split("[,]"));
                for(unsigned int j = 0; j < number_requesters; ++j)
                {
                    P_out_request_vector[j * number_resources + i] = P_arb_out_request_vector[j];
                    act_out_request_vector[j * number_resources + i] = act_arb_out_request_vector[j];
                }
            }
        }
        else
        {

        }

        // Update output probabilities
        getGenProperties()->set("P(Grant)", LibUtil::vectorToString(P_out_request_vector));
        getGenProperties()->set("Act(Grant)", LibUtil::vectorToString(act_out_request_vector));

        return;
    }
} // namespace DSENT

