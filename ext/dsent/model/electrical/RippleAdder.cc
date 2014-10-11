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

#include "model/electrical/RippleAdder.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"

namespace DSENT
{
    RippleAdder::RippleAdder(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    RippleAdder::~RippleAdder()
    {}

    void RippleAdder::initParameters()
    {
        addParameterName("NumberBits");
        return;
    }

    void RippleAdder::initProperties()
    {
        return;
    }

    void RippleAdder::constructModel()
    {
        // Get properties
        unsigned int number_bits = (unsigned int) getParameter("NumberBits");

        //Construct electrical ports and nets
        createInputPort("CI");
        createOutputPort("CO");
        for(unsigned int i = 0; i < number_bits; ++i)
        {
            createInputPort("A" + String(i));
            createInputPort("B" + String(i));
            createOutputPort("S" + String(i));
            createNet("C" + String(i));
        }
        createNet("C" + String(number_bits));

        //Create energy, power, and area results
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        createElectricalEventResult("Add");
        Result* add_event = getEventResult("Add");

        // Connect all nets 
        assign("C0", "CI");
        assign("CO", "C" + String(number_bits));
        for (unsigned int i = 0; i < number_bits; ++i)
        {
            String n = (String) i;            
            StdCell* adder = getTechModel()->getStdCellLib()->createStdCell("ADDF", "ADDF_" + n);
            adder->construct();

            //Build electrical connectivity
            portConnect(adder, "A", "A" + String(i));
            portConnect(adder, "B", "B" + String(i));
            portConnect(adder, "CI", "C" + String(i));
            portConnect(adder, "S", "S" + String(i));
            portConnect(adder, "CO", "C" + String(i + 1));

            //Add ADDF instance, leakage power, energy, and add event results
            addSubInstances(adder, 1.0);                        
            addElectricalSubResults(adder, 1.0);
            add_event->addSubResult(adder->getEventResult("ADDF"), "ADDF_" + n, 1.0);
        }

        return;
    }

    void RippleAdder::propagateTransitionInfo()
    {
        unsigned int number_bits = getParameter("NumberBits").toUInt();

        TransitionInfo current_trans_CI = getInputPort("CI")->getTransitionInfo();
        for(unsigned int i = 0; i < number_bits; ++i)
        {
            ElectricalModel* adder = (ElectricalModel*)getSubInstance("ADDF_" + String(i));

            // Propagate input transition info
            propagatePortTransitionInfo(adder, "A", "A" + String(i));
            propagatePortTransitionInfo(adder, "B", "B" + String(i));
            assignPortTransitionInfo(adder, "CI", current_trans_CI);
            adder->use();

            // Assign output transition info
            propagatePortTransitionInfo("S" + String(i), adder, "S");
            current_trans_CI = adder->getOutputPort("CO")->getTransitionInfo();
        }
        getOutputPort("CO")->setTransitionInfo(current_trans_CI);
        return;
    }

} // namespace DSENT

