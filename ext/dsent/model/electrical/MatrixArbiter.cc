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

#include "model/electrical/MatrixArbiter.h"

#include <cmath>
#include <vector>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/ModelGen.h"
#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"

namespace DSENT
{
    using std::abs;
    using std::vector;

    MatrixArbiter::MatrixArbiter(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    MatrixArbiter::~MatrixArbiter()
    {}

    void MatrixArbiter::initParameters()
    {
        addParameterName("NumberRequests");
        return;
    }

    void MatrixArbiter::initProperties()
    {
        return;
    }

    MatrixArbiter* MatrixArbiter::clone() const
    {
        // TODO
        return NULL;
    }

    void MatrixArbiter::constructModel()
    {
        // Get parameters
        unsigned int number_requests = getParameter("NumberRequests").toUInt();

        ASSERT(number_requests > 0, "[Error] " + getInstanceName() +
                " -> Number of requests must be > 0!");

        // Connect ports
        createInputPort("CK");
        for(unsigned int i = 0; i < number_requests; ++i)
        {
            createInputPort("Request" + (String)i);
            createOutputPort("Grant" + (String)i);
        }

        // Create area, power, event results
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
//        for(unsigned int i = 0; i <= number_requests; ++i)
//        {
//            // Create arbitrate event with i requests
//            createElectricalEventResult("Arbitrate" + (String)i);
//            EventInfo* event_info = getEventInfo("Arbitrate" + (String)i);
//            event_info->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
//
//            for(unsigned int j = 0; j < i; ++j)
//            {
//                event_info->setTransitionInfo("Request" + (String)j, TransitionInfo(0.0, 0.0, 1.0));
//            }
//            for(unsigned int j = i; j < number_requests; ++j)
//            {
//                event_info->setTransitionInfo("Request" + (String)j, TransitionInfo(1.0, 0.0, 0.0));
//            
//            }
//            //double P_0 = (double)(number_requests - i) / (double)(number_requests);
//            //double P_1 = (double)(i) / (double)(number_requests);
//            //TransitionInfo trans(P_0 * P_0, P_0 * P_1, P_1 * P_1);
//
//            //for(unsigned int j = 0; j < number_requests; ++j)
//            //{
//            //    event_info->setTransitionInfo("Request" + (String)j, trans);
//            //}
//        }
        createElectricalEventResult("Arbitrate");
        getEventInfo("Arbitrate")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        for(unsigned int i = 0; i < number_requests; ++i)
        {
            getEventInfo("Arbitrate")->setTransitionInfo("Request" + (String)i, TransitionInfo(0.25, 0.25, 0.25));
        }

        if(number_requests == 1)
        {
            assign("Grant0", "Request0");
        }
        else
        {
            // Init components
            vector<String> g_inv_names(number_requests, "");
            vector<StdCell*> g_invs(number_requests, NULL);
            vector<String> g_and2_names(number_requests, "");
            vector<StdCell*> g_and2s(number_requests, NULL);
            for(unsigned int i = 0; i < number_requests; ++i)
            {
                g_inv_names[i] = "G_INV" + (String)i;
                g_and2_names[i] = "G_AND2" + (String)i;
                g_invs[i] = getTechModel()->getStdCellLib()->createStdCell("INV", g_inv_names[i]);
                g_invs[i]->construct();
                g_and2s[i] = getTechModel()->getStdCellLib()->createStdCell("AND2", g_and2_names[i]);
                g_and2s[i]->construct();
            }

            unsigned int number_states = (number_requests - 1) * number_requests / 2;

            vector<String> w_or2_names(number_states, "");
            vector<StdCell*> w_or2s(number_states, NULL);
            vector<String> w_and2_names(number_states, "");
            vector<StdCell*> w_and2s(number_states, NULL);
            vector<String> w_inv_names(number_states, "");
            vector<StdCell*> w_invs(number_states, NULL);
            vector<String> w_dff_names(number_states, "");
            vector<StdCell*> w_dffs(number_states, NULL);
            vector<String> dis_and2_names(number_states * 2, "");
            vector<StdCell*> dis_and2s(number_states * 2, NULL);
            vector<String> dis_inv_names(number_states, "");
            vector<StdCell*> dis_invs(number_states, NULL);
            unsigned int state_count = 0;
            for(unsigned int i = 0; i < number_requests; ++i)
            {
                for(unsigned int j = i + 1; j < number_requests; ++j)
                {
                    w_or2_names[state_count] = String::format("W_OR2_%d_%d", i, j);
                    w_and2_names[state_count] = String::format("W_AND2_%d_%d", i, j);
                    w_inv_names[state_count] = String::format("W_INV_%d_%d", i, j);
                    w_dff_names[state_count] = String::format("W_DFF_%d_%d", i, j);
                    w_or2s[state_count] = getTechModel()->getStdCellLib()->createStdCell("OR2", w_or2_names[state_count]);
                    w_or2s[state_count]->construct();
                    w_and2s[state_count] = getTechModel()->getStdCellLib()->createStdCell("AND2", w_and2_names[state_count]);
                    w_and2s[state_count]->construct();
                    w_invs[state_count] = getTechModel()->getStdCellLib()->createStdCell("INV", w_inv_names[state_count]);
                    w_invs[state_count]->construct();
                    w_dffs[state_count] = getTechModel()->getStdCellLib()->createStdCell("DFFQ", w_dff_names[state_count]);
                    w_dffs[state_count]->construct();

                    dis_inv_names[state_count] = String::format("Dis_INV_%d_%d", i, j);
                    dis_and2_names[state_count] = String::format("Dis_AND2_%d_%d", i, j);
                    dis_and2_names[state_count + number_states] = String::format("Dis_AND2_%d_%d", j, i);
                    dis_invs[state_count] = getTechModel()->getStdCellLib()->createStdCell("INV", dis_inv_names[state_count]);
                    dis_invs[state_count]->construct();
                    dis_and2s[state_count] = getTechModel()->getStdCellLib()->createStdCell("AND2", dis_and2_names[state_count]);
                    dis_and2s[state_count]->construct();
                    dis_and2s[state_count + number_states] = getTechModel()->getStdCellLib()->createStdCell("AND2", dis_and2_names[state_count + number_states]);
                    dis_and2s[state_count + number_states]->construct();
                    state_count++;
                }
            }

            vector<String> dis_or_names(number_requests, "");
            vector<ElectricalModel*> dis_ors(number_requests, NULL);
            for(unsigned int i = 0; i < number_requests; ++i)
            {
                dis_or_names[i] = "Dis_OR" + (String)i;
                dis_ors[i] = (ElectricalModel*)ModelGen::createModel("OR", dis_or_names[i], getTechModel());
                dis_ors[i]->setParameter("NumberInputs", number_requests-1);
                dis_ors[i]->setParameter("NumberBits", 1);
                dis_ors[i]->construct();
            }

            state_count = 0;
            for(unsigned int i = 0; i < number_requests; ++i)
            {
                createNet("Dis_OR_Out" + (String)i);
                createNet("G_INV_Out" + (String)i);
                portConnect(g_invs[i], "A", "Dis_OR_Out" + (String)i);
                portConnect(g_invs[i], "Y", "G_INV_Out" + (String)i);
                portConnect(g_and2s[i], "A", "Request" + (String)i);
                portConnect(g_and2s[i], "B", "G_INV_Out" + (String)i);
                portConnect(g_and2s[i], "Y", "Grant" + (String)i);

                for(unsigned int j = i + 1; j < number_requests; ++j)
                {
                    createNet(String::format("W_INV_Out_%d_%d", i, j));
                    createNet(String::format("W_OR2_Out_%d_%d", i, j));
                    createNet(String::format("W_AND2_Out_%d_%d", i, j));
                    createNet(String::format("W_DFF_Out_%d_%d", i, j));
                    portConnect(w_invs[state_count], "A", "Grant" + (String)i);
                    portConnect(w_invs[state_count], "Y", String::format("W_INV_Out_%d_%d", i, j));
                    portConnect(w_or2s[state_count], "A", String::format("W_DFF_Out_%d_%d", i, j));
                    portConnect(w_or2s[state_count], "B", "Grant" + (String)j);
                    portConnect(w_or2s[state_count], "Y", String::format("W_OR2_Out_%d_%d", i, j));
                    portConnect(w_and2s[state_count], "A", String::format("W_OR2_Out_%d_%d", i, j));
                    portConnect(w_and2s[state_count], "B", String::format("W_INV_Out_%d_%d", i, j));
                    portConnect(w_and2s[state_count], "Y", String::format("W_AND2_Out_%d_%d", i, j));
                    portConnect(w_dffs[state_count], "D", String::format("W_AND2_Out_%d_%d", i, j));
                    portConnect(w_dffs[state_count], "CK", "CK");
                    portConnect(w_dffs[state_count], "Q", String::format("W_DFF_Out_%d_%d", i, j));

                    createNet(String::format("Dis_AND2_Out_%d_%d", i, j));
                    createNet(String::format("Dis_AND2_Out_%d_%d", j, i));
                    createNet(String::format("Dis_INV_Out_%d_%d", j, i));
                    portConnect(dis_and2s[state_count], "A", "Request" + (String)i);
                    portConnect(dis_and2s[state_count], "B", String::format("W_DFF_Out_%d_%d", i, j));
                    portConnect(dis_and2s[state_count], "Y", String::format("Dis_AND2_Out_%d_%d", i, j));

                    portConnect(dis_invs[state_count], "A", String::format("W_DFF_Out_%d_%d", i, j));
                    portConnect(dis_invs[state_count], "Y", String::format("Dis_INV_Out_%d_%d", j, i));
                    portConnect(dis_and2s[state_count + number_states], "A", "Request" + (String)j);
                    portConnect(dis_and2s[state_count + number_states], "B", String::format("Dis_INV_Out_%d_%d", j, i));
                    portConnect(dis_and2s[state_count + number_states], "Y", String::format("Dis_AND2_Out_%d_%d", j, i));

                    state_count++;
                }
            }
            for(unsigned int i = 0; i < number_requests; ++i)
            {
                unsigned int k = 0;
                for(unsigned int j = 0; j < number_requests; ++j)
                {
                    if(i != j)
                    {
                        portConnect(dis_ors[i], "In" + (String)k, String::format("Dis_AND2_Out_%d_%d", j, i));
                        k++;
                    }
                }
                portConnect(dis_ors[i], "Out", "Dis_OR_Out" + (String)i);
            }

            // Add instances
            for(unsigned int i = 0; i < number_requests; ++i)
            {
                addSubInstances(g_invs[i], 1.0);
                addElectricalSubResults(g_invs[i], 1.0);
                addSubInstances(g_and2s[i], 1.0);
                addElectricalSubResults(g_and2s[i], 1.0);
                addSubInstances(dis_ors[i], 1.0);
                addElectricalSubResults(dis_ors[i], 1.0);
            }
            for(unsigned int i = 0; i < number_states; ++i)
            {
                addSubInstances(w_or2s[i], 1.0);
                addElectricalSubResults(w_or2s[i], 1.0);
                addSubInstances(w_and2s[i], 1.0);
                addElectricalSubResults(w_and2s[i], 1.0);
                addSubInstances(w_invs[i], 1.0);
                addElectricalSubResults(w_invs[i], 1.0);
                addSubInstances(w_dffs[i], 1.0);
                addElectricalSubResults(w_dffs[i], 1.0);
                addSubInstances(dis_and2s[i], 1.0);
                addElectricalSubResults(dis_and2s[i], 1.0);
                addSubInstances(dis_and2s[i + number_states], 1.0);
                addElectricalSubResults(dis_and2s[i + number_states], 1.0);
                addSubInstances(dis_invs[i], 1.0);
                addElectricalSubResults(dis_invs[i], 1.0);
            }

            // Update event
            //for(unsigned int i = 0; i <= number_requests; ++i)
            //{
            //Result* arb_event = getEventResult("Arbitrate" + (String)i);
            Result* arb_event = getEventResult("Arbitrate");
            for(unsigned int j = 0; j < number_requests; ++j)
            {
                arb_event->addSubResult(g_invs[j]->getEventResult("INV"), g_inv_names[j], 1.0);
                arb_event->addSubResult(g_and2s[j]->getEventResult("AND2"), g_and2_names[j], 1.0);
                arb_event->addSubResult(dis_ors[j]->getEventResult("OR"), dis_or_names[j], 1.0);
            }
            for(unsigned int j = 0; j < number_states; ++j)
            {
                arb_event->addSubResult(w_or2s[j]->getEventResult("OR2"), w_or2_names[j], 1.0);
                arb_event->addSubResult(w_and2s[j]->getEventResult("AND2"), w_and2_names[j], 1.0);
                arb_event->addSubResult(w_invs[j]->getEventResult("INV"), w_inv_names[j], 1.0);
                arb_event->addSubResult(w_dffs[j]->getEventResult("DFFD"), w_dff_names[j], 1.0);
                arb_event->addSubResult(w_dffs[j]->getEventResult("DFFQ"), w_dff_names[j], 1.0);
                arb_event->addSubResult(w_dffs[j]->getEventResult("CK"), w_dff_names[j], 1.0);
                arb_event->addSubResult(dis_and2s[j]->getEventResult("AND2"), dis_and2_names[j], 1.0);
                arb_event->addSubResult(dis_and2s[j + number_states]->getEventResult("AND2"), dis_and2_names[j + number_states], 1.0);
                arb_event->addSubResult(dis_invs[j]->getEventResult("INV"), dis_inv_names[j], 1.0);
            }
            //}
        }
        return;
    }

    void MatrixArbiter::propagateTransitionInfo()
    {
        // Get parameters
        unsigned int number_requests = getParameter("NumberRequests").toUInt();

        if(number_requests == 1)
        {
            propagatePortTransitionInfo("Grant0", "Request0");
        }
        else
        {
            unsigned int number_states = (number_requests - 1) * number_requests / 2;

            vector<ElectricalModel*> g_and2s(number_requests, NULL);
            vector<ElectricalModel*> g_invs(number_requests, NULL);
            vector<ElectricalModel*> w_invs(number_states, NULL);
            vector<ElectricalModel*> w_or2s(number_states, NULL);
            vector<ElectricalModel*> w_and2s(number_states, NULL);
            vector<ElectricalModel*> w_dffs(number_states, NULL);
            vector<ElectricalModel*> dis_invs(number_states, NULL);
            vector<ElectricalModel*> dis_and2s(number_requests * number_requests, NULL);
            vector<ElectricalModel*> dis_ors(number_requests, NULL);
            for(unsigned int i = 0; i < number_requests; ++i)
            {
                g_and2s[i] = (ElectricalModel*)getSubInstance("G_AND2" + (String)i);
                g_invs[i] = (ElectricalModel*)getSubInstance("G_INV" + (String)i);
                dis_ors[i] = (ElectricalModel*)getSubInstance("Dis_OR" + (String)i);
            }
            unsigned int state_count = 0;
            for(unsigned int i = 0; i < number_requests; ++i)
            {
                for(unsigned int j = i + 1; j < number_requests; ++j)
                {
                    w_invs[state_count] = (ElectricalModel*)getSubInstance(String::format("W_INV_%d_%d", i, j));
                    w_or2s[state_count] = (ElectricalModel*)getSubInstance(String::format("W_OR2_%d_%d", i, j));
                    w_and2s[state_count] = (ElectricalModel*)getSubInstance(String::format("W_AND2_%d_%d", i, j));
                    w_dffs[state_count] = (ElectricalModel*)getSubInstance(String::format("W_DFF_%d_%d", i, j));
                    dis_invs[state_count] = (ElectricalModel*)getSubInstance(String::format("Dis_INV_%d_%d", i, j));
                    dis_and2s[i * number_requests + j] = (ElectricalModel*)getSubInstance(String::format("Dis_AND2_%d_%d", i, j));
                    dis_and2s[j * number_requests + i] = (ElectricalModel*)getSubInstance(String::format("Dis_AND2_%d_%d", j, i));

                    w_dffs[state_count]->getInputPort("D")->setTransitionInfo(TransitionInfo(0.5, 0.0, 0.5));
                    propagatePortTransitionInfo(w_dffs[state_count], "CK", "CK");
                    w_dffs[state_count]->use();

                    state_count++;
                }
            }

            unsigned int iteration = 1;
            unsigned int max_number_iterations = 10;
            //vector<TransitionInfo> trans_vector(number_states, TransitionInfo(0.0, 0.0, 1.0));
            //vector<double> total_P_vector(number_states, 0.0);
            while(iteration < max_number_iterations)
            {
//                for(unsigned int i = 0; i < number_states; ++i)
//                {
//                    w_dffs[i]->getInputPort("D")->setTransitionInfo(trans_vector[i]);
//                    propagatePortTransitionInfo(w_dffs[i], "CK", "CK");
//                    w_dffs[i]->use();
//                }
                state_count = 0;
                for(unsigned int i = 0; i < number_requests; ++i)
                {
                    for(unsigned int j = i + 1; j < number_requests; ++j)
                    {
                        propagatePortTransitionInfo(dis_and2s[i * number_requests + j], "A", "Request" + (String)i);
                        propagatePortTransitionInfo(dis_and2s[i * number_requests + j], "B", w_dffs[state_count], "Q");
                        dis_and2s[i * number_requests + j]->use();
                        propagatePortTransitionInfo(dis_invs[state_count], "A", w_dffs[state_count], "Q");
                        dis_invs[state_count]->use();
                        propagatePortTransitionInfo(dis_and2s[j * number_requests + i], "A", "Request" + (String)j);
                        propagatePortTransitionInfo(dis_and2s[j * number_requests + i], "B", dis_invs[state_count], "Y");
                        dis_and2s[j * number_requests + i]->use();

                        state_count++;
                    }
                }
                for(unsigned int i = 0; i < number_requests; ++i)
                {
                    unsigned int k = 0;
                    for(unsigned int j = 0; j < number_requests; ++j)
                    {
                        if(i != j)
                        {
                            propagatePortTransitionInfo(dis_ors[i], "In" + (String)k, dis_and2s[j * number_requests + i], "Y");
                            k++;
                        }
                    }
                    dis_ors[i]->use();
                }
                for(unsigned int i = 0; i < number_requests; ++i)
                {
                    propagatePortTransitionInfo(g_invs[i], "A", dis_ors[i], "Out");
                    g_invs[i]->use();
                    propagatePortTransitionInfo(g_and2s[i], "A", "Request" + (String)i);
                    propagatePortTransitionInfo(g_and2s[i], "B", g_invs[i], "Y");
                    g_and2s[i]->use();
                }
                state_count = 0;
                for(unsigned int i = 0; i < number_requests; ++i)
                {
                    for(unsigned int j = i + 1; j < number_requests; ++j)
                    {
                        propagatePortTransitionInfo(w_invs[state_count], "A", g_and2s[i], "Y");
                        w_invs[state_count]->use();
                        propagatePortTransitionInfo(w_or2s[state_count], "A", w_dffs[state_count], "Q");
                        propagatePortTransitionInfo(w_or2s[state_count], "B", g_and2s[j], "Y");
                        w_or2s[state_count]->use();
                        propagatePortTransitionInfo(w_and2s[state_count], "A", w_or2s[state_count], "Y");
                        propagatePortTransitionInfo(w_and2s[state_count], "B", w_invs[state_count], "Y");
                        w_and2s[state_count]->use();
                        propagatePortTransitionInfo(w_dffs[state_count], "D", w_and2s[state_count], "Y");
                        propagatePortTransitionInfo(w_dffs[state_count], "CK", "CK");
                        w_dffs[state_count]->use();
                        state_count++;
                    }
                }

//                for(unsigned int i = 0; i < number_states; ++i)
//                {
//                    const TransitionInfo& new_trans = w_dffs[i]->getOutputPort("Q")->getTransitionInfo();
//                    total_P_vector[i] += new_trans.getProbability1();
//                    trans_vector[i] = TransitionInfo((1.0 - total_P_vector[i] / iteration) * (1.0 - total_P_vector[i] / iteration), 
//                            (1.0 - total_P_vector[i] / iteration) * (total_P_vector[i] / iteration), 
//                            (total_P_vector[i] / iteration) * (total_P_vector[i] / iteration));
//                }
//
//                for(unsigned int i = 0; i < number_requests; ++i)
//                {
//                    g_and2s[i]->getOutputPort("Y")->getTransitionInfo().print(cout);
//                }
//                cout << endl;
                iteration++;
            }

            for(unsigned int i = 0; i < number_requests; ++i)
            {
                propagatePortTransitionInfo("Grant" + (String)i, g_and2s[i], "Y");
            }
        }

        return;
    }
} // namespace DSENT

