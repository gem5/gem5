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

#include "model/electrical/OR.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/std_cells/StdCellLib.h"
#include "model/std_cells/StdCell.h"
#include "model/timing_graph/ElectricalNet.h"

namespace DSENT
{
    using std::ceil;
    using std::floor;

    OR::OR(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    OR::~OR()
    {}

    void OR::initParameters()
    {
        addParameterName("NumberInputs");
        addParameterName("NumberBits");
        addParameterName("BitDuplicate", "TRUE");
        return;
    }

    void OR::initProperties()
    {
        return;
    }

    OR* OR::clone() const
    {
        // TODO
        return NULL;
    }

    void OR::constructModel()
    {
        // Get parameter
        unsigned int number_inputs = getParameter("NumberInputs").toUInt();
        unsigned int number_bits = getParameter("NumberBits").toUInt();
        bool bit_duplicate = getParameter("BitDuplicate").toBool();

        ASSERT(number_inputs > 0, "[Error] " + getInstanceName() +
                " -> Number of inputs must be > 0!");
        ASSERT(number_bits > 0, "[Error] " + getInstanceName() + 
                " -> Number of bits must be > 0!");


        // Init ports
        for(unsigned int i = 0; i < number_inputs; ++i)
        {
            createInputPort("In" + (String)i, makeNetIndex(0, number_bits-1));
        }
        createOutputPort("Out", makeNetIndex(0, number_bits-1));

        // Number of inputs on the 0 side
        unsigned int or0_number_inputs = (unsigned int)ceil((double)number_inputs / 2.0);
        // Number of inputs on the 1 side
        unsigned int or1_number_inputs = (unsigned int)floor((double)number_inputs / 2.0);

        // Create area, power, and event results
        createElectricalResults();
        createElectricalEventResult("OR");

        getEventInfo("Idle")->setStaticTransitionInfos();

        //Depending on whether we want to create a 1-bit instance and have it multiplied
        //up by number of bits or actually instantiate number_bits of 1-bit instances.
        //Recursively instantiates smaller ors
        if(bit_duplicate || number_bits == 1)
        {
            // If it is just a 1-input or, just connect output to input
            if(number_inputs == 1)
            {
                assign("Out", "In0");
            }
            else
            {
                // If it is more than 1 input, instantiate two sub ors (OR_way0 and OR_way1)
                // and create a final OR2 to OR them
                const String& or0_name = "OR_way0";
                const String& or1_name = "OR_way1";
                const String& orf_name = "OR2_i" + (String)number_inputs;

                OR* or0 = new OR(or0_name, getTechModel());
                or0->setParameter("NumberInputs", or0_number_inputs);
                or0->setParameter("NumberBits", 1);
                or0->setParameter("BitDuplicate", "TRUE");
                or0->construct();

                OR* or1 = new OR(or1_name, getTechModel());
                or1->setParameter("NumberInputs", or1_number_inputs);
                or1->setParameter("NumberBits", 1);
                or1->setParameter("BitDuplicate", "TRUE");
                or1->construct();

                StdCell* orf = getTechModel()->getStdCellLib()->createStdCell("OR2", orf_name);
                orf->construct();

                // Create outputs of way0 and way1 ors with final or
                createNet("way0_Out");
                createNet("way1_Out");
                portConnect(or0, "Out", "way0_Out");
                portConnect(or1, "Out", "way1_Out");
                portConnect(orf, "A", "way0_Out");
                portConnect(orf, "B", "way1_Out");

                // Connect inputs to the sub ors.
                for(unsigned int i = 0; i < or0_number_inputs; ++i)
                {
                    createNet("way0_In" + (String)i);
                    portConnect(or0, "In" + (String)i, "way0_In" + (String)i);
                    assignVirtualFanin("way0_In" + (String)i, "In" + (String)i);
                }
                for(unsigned int i = 0; i < or1_number_inputs; ++i)
                {
                    createNet("way1_In" + (String)i);
                    portConnect(or1, "In" + (String)i, "way1_In" + (String)i);
                    assignVirtualFanin("way1_In" + (String)i, "In" + (String)(i + or0_number_inputs));
                }

                // Connect outputs
                createNet("OR2_Out");
                portConnect(orf, "Y", "OR2_Out");
                assignVirtualFanout("Out", "OR2_Out");

                addSubInstances(or0, number_bits);
                addElectricalSubResults(or0, number_bits);
                addSubInstances(or1, number_bits);
                addElectricalSubResults(or1, number_bits);
                addSubInstances(orf, number_bits);
                addElectricalSubResults(orf, number_bits);

                Result* or_event = getEventResult("OR");
                or_event->addSubResult(or0->getEventResult("OR"), or0_name, number_bits);
                or_event->addSubResult(or1->getEventResult("OR"), or1_name, number_bits);
                or_event->addSubResult(orf->getEventResult("OR2"), orf_name, number_bits);

            }
        }
        else
        {
            // Init a bunch of 1-bit ors
            Result* or_event = getEventResult("OR");
            for(unsigned int n = 0; n < number_bits; ++n)
            {
                const String& or_name = "OR_bit" + (String)n;

                OR* ors = new OR(or_name, getTechModel());
                ors->setParameter("NumberInputs", number_inputs);
                ors->setParameter("NumberBits", 1);
                ors->setParameter("BitDuplicate", "TRUE");
                ors->construct();

                for(unsigned int i = 0; i < number_inputs; ++i)
                {
                    portConnect(ors, "In" + (String)i, "In" + (String)i, makeNetIndex(n));
                }
                portConnect(ors, "Out", "Out", makeNetIndex(n));

                addSubInstances(ors, 1.0);
                addElectricalSubResults(ors, 1.0);
                or_event->addSubResult(ors->getEventResult("OR"), or_name, 1.0);
            }
        }
        return;
    }

    void OR::propagateTransitionInfo()
    {
        // Get parameters
        unsigned int number_inputs = getParameter("NumberInputs").toUInt();
        unsigned int number_bits = getParameter("NumberBits").toUInt();
        bool bit_duplicate = getParameter("BitDuplicate").toBool();

        // Number of inputs on 0 side
        unsigned int or0_number_inputs = (unsigned int)ceil((double)number_inputs / 2.0);
        unsigned int or1_number_inputs = (unsigned int)floor((double)number_inputs / 2.0);

        if(bit_duplicate || number_bits == 1)
        {
            if(number_inputs == 1)
            {
                propagatePortTransitionInfo("Out", "In0");
            }
            else
            {
                ElectricalModel* or0 = (ElectricalModel*)getSubInstance("OR_way0");
                for(unsigned int i = 0; i < or0_number_inputs; ++i)
                {
                    propagatePortTransitionInfo(or0, "In" + (String)i, "In" + (String)i);
                }
                or0->use();

                ElectricalModel* or1 = (ElectricalModel*)getSubInstance("OR_way1");
                for(unsigned int i = 0; i < or1_number_inputs; ++i)
                {
                    propagatePortTransitionInfo(or1, "In" + (String)i, "In" + (String)i);
                }
                or1->use();

                ElectricalModel* orf = (ElectricalModel*)getSubInstance("OR2_i" + (String)number_inputs);
                propagatePortTransitionInfo(orf, "A", or0, "Out");
                propagatePortTransitionInfo(orf, "B", or1, "Out");
                orf->use();

                // Set output probability
                propagatePortTransitionInfo("Out", orf, "Y");
            }
        }
        else
        {
            for(unsigned int n = 0; n < number_bits; ++n)
            {
                ElectricalModel* or_bit = (ElectricalModel*)getSubInstance("OR_bit" + (String)n);
                for(unsigned int i = 0; i < number_inputs; ++i)
                {
                    propagatePortTransitionInfo(or_bit, "In" + (String)i, "In" + (String)i);
                }
                or_bit->use();
            }

            ElectricalModel* or_bit = (ElectricalModel*)getSubInstance("OR_bit0");
            propagatePortTransitionInfo("Out", or_bit, "Out");
        }
        return;
    }
} // namespace DSENT

