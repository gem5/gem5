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

#include "model/electrical/Multiplexer.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/timing_graph/ElectricalDriverMultiplier.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"

namespace DSENT
{
    Multiplexer::Multiplexer(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    Multiplexer::~Multiplexer()
    {}

    void Multiplexer::initParameters()
    {
        addParameterName("NumberInputs");
        addParameterName("NumberBits");
        addParameterName("BitDuplicate", "TRUE");
        addParameterName("IsTopLevel", "TRUE");
        return;
    }

    void Multiplexer::initProperties()
    {
        return;
    }

    Multiplexer* Multiplexer::clone() const
    {
        return NULL;
    }

    void Multiplexer::constructModel()
    {
        // Get parameters
        unsigned int number_bits = (unsigned int) getParameter("NumberBits");
        unsigned int number_inputs = (unsigned int) getParameter("NumberInputs");
        unsigned int number_selects = (unsigned int) ceil(log2((double) number_inputs));
        bool bit_duplicate = (bool) getParameter("BitDuplicate");
        bool is_top_level = getParameter("IsTopLevel").toBool();
        
        ASSERT(number_inputs > 0, "[Error] " + getInstanceName() + " -> Number of inputs must be > 0!");
        ASSERT(number_bits > 0, "[Error] " + getInstanceName() + " -> Number of bits must be > 0!");
    
        //Construct electrical ports and nets
        //Create each input port
        for(unsigned int i = 0; i < number_inputs; ++i)
            createInputPort(    "In" + (String) i, makeNetIndex(0, number_bits-1));
        //Create select signals
        for(unsigned int i = 0; i < number_selects; ++i)
        {
            createInputPort(    "Sel" + (String)i);
        }
        //Create output
        createOutputPort(   "Out", makeNetIndex(0, number_bits-1));
        
        //Create energy, power, and area results
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        createElectricalEventResult("Mux");
                    
        //Number of inputs on the 0 side
        unsigned int inputs_0 = (unsigned int) ceil((double) number_inputs / 2.0);
        unsigned int selects_0 = (unsigned int) ceil(log2((double) inputs_0));
        //Number of inputs on the 1 side
        unsigned int inputs_1 = (unsigned int) floor((double) number_inputs / 2.0);
        unsigned int selects_1 = (unsigned int) ceil(log2((double) inputs_1));
        
        //Depending on whether we want to create a 1-bit instance and have it multiplied
        //up by number of bits or actually instantiate number_bits of 1-bit instances.
        //Recursively instantiates smaller multiplexers
        if (bit_duplicate || number_bits == 1)
        {
            //If it is just a 1-input multiplexer, just connect output to input and be done
            if (number_inputs == 1)
            {
                assign("Out", "In0");
            }
            else
            {
                //If it is more than 1 input, instantiate two sub multiplexers (Mux_way0 and Mux_way1)
                //and create a final 2:1 mux (muxf) to select between them
                String mux0_name = "Mux_way0";
                String mux1_name = "Mux_way1";
                String muxf_name = "Mux2_i" + (String)number_inputs;

                Multiplexer* mux0 = new Multiplexer(mux0_name, getTechModel());
                mux0->setParameter("NumberInputs", inputs_0);
                mux0->setParameter("NumberBits", 1);
                mux0->setParameter("BitDuplicate", "TRUE");
                mux0->setParameter("IsTopLevel", "FALSE");
                mux0->construct();
            
                Multiplexer* mux1 = new Multiplexer(mux1_name, getTechModel());
                mux1->setParameter("NumberInputs", inputs_1);
                mux1->setParameter("NumberBits", 1);
                mux1->setParameter("BitDuplicate", "TRUE");                                    
                mux1->setParameter("IsTopLevel", "FALSE");
                mux1->construct();
                
                StdCell* muxf = getTechModel()->getStdCellLib()->createStdCell("MUX2", muxf_name);
                muxf->construct();

                // TODO hack 
                // create selector driver at the top level
                if(is_top_level)
                {
                    for(unsigned int i = 0; i < number_selects; ++i)
                    {
                        StdCell* selinv0 = getTechModel()->getStdCellLib()->createStdCell("INV", String::format("Sel%dInv0", i));
                        StdCell* selinv1 = getTechModel()->getStdCellLib()->createStdCell("INV", String::format("Sel%dInv1", i));
                        selinv0->construct();
                        selinv1->construct();

                        addSubInstances(selinv0, 1.0);
                        addElectricalSubResults(selinv0, 1.0);
                        addSubInstances(selinv1, 1.0);
                        addElectricalSubResults(selinv1, 1.0);
                        getEventResult("Mux")->addSubResult(selinv0->getEventResult("INV"), String::format("Sel%dInv0", i), 1.0);                
                        getEventResult("Mux")->addSubResult(selinv1->getEventResult("INV"), String::format("Sel%dInv1", i), 1.0);                
                    }
                }

                //Create outputs of way0 and way1 multiplexers with final mux
                createNet("way0Out");
                createNet("way1Out");
                portConnect(mux0, "Out", "way0Out");
                portConnect(mux1, "Out", "way1Out");
                portConnect(muxf, "A", "way0Out");
                portConnect(muxf, "B", "way1Out");

                // TODO hack 
                // Connect selector bits
                if(is_top_level)
                {
                    for(unsigned int i = 0; i < number_selects; ++i)
                    {
                        ElectricalModel* selinv0 = (ElectricalModel*)getSubInstance(String::format("Sel%dInv0", i));
                        ElectricalModel* selinv1 = (ElectricalModel*)getSubInstance(String::format("Sel%dInv1", i));
                        createNet("SelInv" + (String)i);
                        createNet("SelBuf" + (String)i);
                        portConnect(selinv0, "A", "Sel" + (String)i);
                        portConnect(selinv0, "Y", "SelInv" + (String)i);
                        portConnect(selinv1, "A", "SelInv" + (String)i);
                        portConnect(selinv1, "Y", "SelBuf" + (String)i);
                    }
                }
                //Connect inputs to the sub multiplexers.
                //Note that multiple inputs are connected to the mux0 and mux1 input and the
                //selector signals are connected multiple times. This is just so that everything
                //is loaded appropriately since bit duplication is applied
                for (unsigned int n = 0; n < number_bits; ++n)
                {
                    //Connect inputs
                    for (unsigned int i = 0; i < inputs_0; ++i)
                        portConnect(mux0, "In" + (String) i, "In" + (String) i, makeNetIndex(n));
                    for (unsigned int i = 0; i < inputs_1; ++i)
                        portConnect(mux1, "In" + (String) i, "In" + (String) (i + inputs_0), makeNetIndex(n));                    
                    // TODO hack 
                    if(is_top_level)
                    {
                        //Connect selector bits
                        for (unsigned int i = 0; i < selects_0; ++i)
                            portConnect(mux0, "Sel" + (String)i, "SelBuf" + (String)i);
                        for (unsigned int i = 0; i < selects_1; ++i)
                            portConnect(mux1, "Sel" + (String)i, "SelBuf" + (String)i);
                        portConnect(muxf, "S0", "SelBuf" + (String)(number_selects - 1));
                    }
                    else
                    {
                        //Connect selector bits
                        for (unsigned int i = 0; i < selects_0; ++i)
                            portConnect(mux0, "Sel" + (String)i, "Sel" + (String)i);
                        for (unsigned int i = 0; i < selects_1; ++i)
                            portConnect(mux1, "Sel" + (String)i, "Sel" + (String)i);
                        portConnect(muxf, "S0", "Sel" + (String)(number_selects - 1));
                    }
                }

                //Connect final mux to outputs
                //Because we use bit duplication and so there is only only one multiplexer
                //instance, we must use driver multiplier to drive each output appropriately
                if (number_bits == 1)
                    portConnect(muxf, "Y", "Out");
                else
                {
                    createNet("OutTemp");
                    createDriverMultiplier("OutMult");
                    ElectricalDriverMultiplier* drive_mult = getDriverMultiplier("OutMult");
                    portConnect(muxf, "Y", "OutTemp");
                    getNet("OutTemp")->addDownstreamNode(drive_mult);
                    for (unsigned int n = 0; n < number_bits; ++n)
                        drive_mult->addDownstreamNode(getNet("Out", makeNetIndex(n)));
                }

                //Add area, power, and event results for each mux
                addSubInstances(mux0, number_bits);
                addElectricalSubResults(mux0, number_bits);
                addSubInstances(mux1, number_bits);
                addElectricalSubResults(mux1, number_bits);
                addSubInstances(muxf, number_bits);
                addElectricalSubResults(muxf, number_bits);
                getEventResult("Mux")->addSubResult(mux0->getEventResult("Mux"), mux0_name, number_bits);                
                getEventResult("Mux")->addSubResult(mux1->getEventResult("Mux"), mux1_name, number_bits);                
                getEventResult("Mux")->addSubResult(muxf->getEventResult("MUX2"), muxf_name, number_bits);                

            }

        }
        else
        {
            //Instantiate a bunch of 1-bit multiplexers
            for (unsigned int n = 0; n < number_bits; ++n)
            {
                String mux_name = "Mux_bit" + (String) n;

                Multiplexer* mux = new Multiplexer(mux_name, getTechModel());
                mux->setParameter("NumberInputs", number_inputs);
                mux->setParameter("NumberBits", 1);
                mux->setParameter("BitDuplicate", "TRUE");            
                mux->construct();

                // Connect inputs
                for (unsigned int i = 0; i < number_inputs; ++i)
                    portConnect(mux, "In" + (String) i, "In" + (String) i, makeNetIndex(n));
                for(unsigned int i = 0; i < number_selects; ++i)
                    portConnect(mux, "Sel" + (String)i, "Sel" + (String)i);
                portConnect(mux, "Out", "Out", makeNetIndex(n));

                //Add area, power, and event results for each mux
                addSubInstances(mux, 1.0);
                addElectricalSubResults(mux, 1.0);
                getEventResult("Mux")->addSubResult(mux->getEventResult("Mux"), mux_name, 1.0);
            }
        }

        return;
    }

    void Multiplexer::propagateTransitionInfo()
    {
        // The only thing can be updated are the input probabilities...so we will update them
        unsigned int number_bits = (unsigned int) getParameter("NumberBits");
        unsigned int number_inputs = (unsigned int) getParameter("NumberInputs");
        unsigned int number_selects = (unsigned int) ceil(log2((double) number_inputs));
        bool bit_duplicate = (bool) getParameter("BitDuplicate");
        bool is_top_level = getParameter("IsTopLevel").toBool();

        //Number of inputs on the 0 side
        unsigned int inputs_0 = (unsigned int) ceil((double) number_inputs / 2.0);
        unsigned int selects_0 = (unsigned int) ceil(log2((double) inputs_0));

        //Number of inputs on the 1 side
        unsigned int inputs_1 = (unsigned int) floor((double) number_inputs / 2.0);
        unsigned int selects_1 = (unsigned int) ceil(log2((double) inputs_1));

        if (bit_duplicate || number_bits == 1)
        {
            if (number_inputs == 1)
            {
                //If theres only 1 input, output transition = input transition
                propagatePortTransitionInfo("Out", "In0");
            }
            else
            { 
                // Update sub muxes with appropriate probabilities
                ElectricalModel* mux0 = (ElectricalModel*)getSubInstance("Mux_way0");
                for(unsigned int i = 0; i < inputs_0; ++i)
                {
                    propagatePortTransitionInfo(mux0, "In" + (String)i, "In" + (String)i);
                }
                for(unsigned int i = 0; i < selects_0; ++i)
                {
                    propagatePortTransitionInfo(mux0, "Sel" + (String)i, "Sel" + (String)i);
                }
                mux0->use();                
                ElectricalModel* mux1 = (ElectricalModel*)getSubInstance("Mux_way1");
                for(unsigned int i = 0; i < inputs_1; ++i)
                {
                    propagatePortTransitionInfo(mux1, "In" + (String)i, "In" + (String)(i + inputs_0));
                }
                for(unsigned int i = 0; i < selects_1; ++i)
                {
                    propagatePortTransitionInfo(mux1, "Sel" + (String)i, "Sel" + (String)i);
                }
                mux1->use();                
                ElectricalModel* muxf = (ElectricalModel*)getSubInstance("Mux2_i" + (String)number_inputs);
                propagatePortTransitionInfo(muxf, "A", mux0, "Out");
                propagatePortTransitionInfo(muxf, "B", mux1, "Out");
                propagatePortTransitionInfo(muxf, "S0", "Sel" + (String)(number_selects-1));
                muxf->use();

                // TODO hack
                if(is_top_level)
                {
                    for(unsigned int i = 0; i < number_selects; ++i)
                    {
                        ElectricalModel* selinv0 = (ElectricalModel*)getSubInstance(String::format("Sel%dInv0", i));
                        ElectricalModel* selinv1 = (ElectricalModel*)getSubInstance(String::format("Sel%dInv1", i));
                        propagatePortTransitionInfo(selinv0, "A", "Sel" + (String)i);
                        selinv0->use();
                        propagatePortTransitionInfo(selinv1, "A", selinv0, "Y");
                        selinv1->use();
                    }
                }

                // Set output transition
                propagatePortTransitionInfo("Out", muxf, "Y");
            }
        }
        else
        {
            // Go through each bit and set the appropriate probability
            for (unsigned int n = 0; n < number_bits; ++n)
            {
                ElectricalModel* mux_bit = (ElectricalModel*)getSubInstance("Mux_bit" + (String) n);
                for(unsigned int i = 0; i < number_inputs; ++i)
                {
                    propagatePortTransitionInfo(mux_bit, "In" + (String)i, "In" + (String)i);
                }
                for(unsigned int i = 0; i < number_selects; ++i)
                {
                    propagatePortTransitionInfo(mux_bit, "Sel" + (String)i, "Sel" + (String)i);
                }
                mux_bit->use();
            }            

            // Set output probability to be average that of probabilties of each output bit
            ElectricalModel* mux_bit = (ElectricalModel*)getSubInstance("Mux_bit0");
            propagatePortTransitionInfo("Out", mux_bit, "Out");
        }        
        return;
    }

} // namespace DSENT

