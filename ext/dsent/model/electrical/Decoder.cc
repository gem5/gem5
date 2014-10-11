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

#include "model/electrical/Decoder.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/std_cells/StdCellLib.h"
#include "model/std_cells/StdCell.h"

namespace DSENT
{
    using std::ceil;

    Decoder::Decoder(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    Decoder::~Decoder()
    {}

    void Decoder::initParameters()
    {
        addParameterName("NumberOutputs");
    }

    void Decoder::initProperties()
    {
        return;
    }

    Decoder* Decoder::clone() const
    {
        // TODO
        return NULL;
    }

    void Decoder::constructModel()
    {
        // Get parameters
        unsigned int number_outputs = getParameter("NumberOutputs").toUInt();

        ASSERT(number_outputs > 0, "[Error] " + getInstanceName() + " -> Number of outputs must be > 0!");

        unsigned int number_addr_bits = (unsigned int)ceil(log2(number_outputs));

        // Create ports
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            createInputPort("Addr" + (String)i);
        }
        for(unsigned int i = 0; i < number_outputs; ++i)
        {
            createOutputPort("Out" + (String)i);
        }

        // Create energy, power, and area results
        createElectricalResults();
        createElectricalEventResult("Decode");
        Result* decode_event = getEventResult("Decode");

        getEventInfo("Idle")->setStaticTransitionInfos();

        if(number_addr_bits == 0)
        {
            // Do not need a decoder
        }
        else if(number_addr_bits == 1)
        {
            const String& inv0_name = "Inv0";

            StdCell* inv0 = getTechModel()->getStdCellLib()->createStdCell("INV", inv0_name);
            inv0->construct();

            // Connect inputs and outputs
            portConnect(inv0, "A", "Addr0");
            portConnect(inv0, "Y", "Out0");
            assign("Out1", "Addr0");

            // Add area, power, and event results
            addSubInstances(inv0, 1.0);
            addElectricalSubResults(inv0, 1.0);
            decode_event->addSubResult(inv0->getEventResult("INV"), inv0_name, 1.0);
        }
        else
        {
            unsigned int number_addr_bits_0 = (unsigned int)ceil((double)number_addr_bits / 2.0);
            unsigned int number_addr_bits_1 = (unsigned int)floor((double)number_addr_bits / 2.0);

            unsigned int number_outputs_0 = (unsigned int)pow(2.0, number_addr_bits_0);
            unsigned int number_outputs_1 = (unsigned int)ceil((double)number_outputs / (double)number_outputs_0);

            const String& dec0_name = "Dec_way0";
            const String& dec1_name = "Dec_way1";
            vector<String> nand2_names(number_outputs, "");
            vector<String> inv_names(number_outputs, "");
            for(unsigned int i = 0; i < number_outputs; ++i)
            {
                nand2_names[i] = "NAND2_" + (String)i;
                inv_names[i] = "INV_" + (String)i;
            }

            Decoder* dec0 = new Decoder(dec0_name, getTechModel());
            dec0->setParameter("NumberOutputs", number_outputs_0);
            dec0->construct();

            Decoder* dec1 = new Decoder(dec1_name, getTechModel());
            dec1->setParameter("NumberOutputs", number_outputs_1);
            dec1->construct();

            vector<StdCell*> nand2s(number_outputs, NULL);
            vector<StdCell*> invs(number_outputs, NULL);
            for(unsigned int i = 0; i < number_outputs; ++i)
            {
                nand2s[i] = getTechModel()->getStdCellLib()->createStdCell("NAND2", nand2_names[i]);
                nand2s[i]->construct();
                invs[i] = getTechModel()->getStdCellLib()->createStdCell("INV", inv_names[i]);
                invs[i]->construct();
            }

            // Connect inputs and outputs
            for(unsigned int i = 0; i < number_addr_bits_0; ++i)
            {
                portConnect(dec0, "Addr" + (String)i, "Addr" + (String)i);
            }
            for(unsigned int i = 0; i < number_addr_bits_1; ++i)
            {
                portConnect(dec1, "Addr" + (String)i, "Addr" + (String)(i + number_addr_bits_0));
            }
            for(unsigned int i = 0; i < number_outputs_0; ++i)
            {
                createNet("way0Out" + (String)i);
                portConnect(dec0, "Out" + (String)i, "way0Out" + (String)i);
            }
            for(unsigned int i = 0; i < number_outputs_1; ++i)
            {
                createNet("way1Out" + (String)i);
                portConnect(dec1, "Out" + (String)i, "way1Out" + (String)i);
            }

            for(unsigned int i = 0; i < number_outputs; ++i)
            {
                createNet("nand" + (String)i + "Out");
                portConnect(nand2s[i], "A", "way0Out" + (String)(i%number_outputs_0));
                portConnect(nand2s[i], "B", "way1Out" + (String)((unsigned int)floor(i/number_outputs_0)));
                portConnect(nand2s[i], "Y", "nand" + (String)i + "Out");
                portConnect(invs[i], "A", "nand" + (String)i + "Out");
                portConnect(invs[i], "Y", "Out" + (String)i);
            }

            // Add area, power, and event results
            addSubInstances(dec0, 1.0);
            addElectricalSubResults(dec0, 1.0);
            decode_event->addSubResult(dec0->getEventResult("Decode"), dec0_name, 1.0);
            addSubInstances(dec1, 1.0);
            addElectricalSubResults(dec1, 1.0);
            decode_event->addSubResult(dec1->getEventResult("Decode"), dec1_name, 1.0);
            for(unsigned int i = 0; i < number_outputs; ++i)
            {
                addSubInstances(nand2s[i], 1.0);
                addElectricalSubResults(nand2s[i], 1.0);
                decode_event->addSubResult(nand2s[i]->getEventResult("NAND2"), nand2_names[i], 1.0);

                addSubInstances(invs[i], 1.0);
                addElectricalSubResults(invs[i], 1.0);
                decode_event->addSubResult(invs[i]->getEventResult("INV"), inv_names[i], 1.0);
            }
        }
        return;
    }

    void Decoder::propagateTransitionInfo()
    {
        // The only thing can be updated are the input probabilities
        unsigned int number_outputs = getParameter("NumberOutputs").toUInt();

        unsigned int number_addr_bits = (unsigned int)ceil(log2(number_outputs));
        
        if(number_addr_bits == 0)
        {
            // Do not need a decoder
        }
        else if(number_addr_bits == 1)
        {
            ElectricalModel* inv0 = (ElectricalModel*)getSubInstance("Inv0");
            propagatePortTransitionInfo(inv0, "A", "Addr0");
            inv0->use();

            // Since # addr bits is 1, the output 0 is directly connected
            propagatePortTransitionInfo("Out0", inv0, "Y");
            propagatePortTransitionInfo("Out1", "Addr0");
        }
        else
        {
            unsigned int number_addr_bits_0 = (unsigned int)ceil((double)number_addr_bits / 2.0);
            unsigned int number_addr_bits_1 = (unsigned int)floor((double)number_addr_bits / 2.0);

            unsigned int number_outputs_0 = (unsigned int)pow(2.0, number_addr_bits_0);

            // Update decoders with probabilities
            ElectricalModel* dec0 = (ElectricalModel*)getSubInstance("Dec_way0");
            for(unsigned int i = 0; i < number_addr_bits_0; ++i)
            {
                propagatePortTransitionInfo(dec0, "Addr" + (String)i, "Addr" + (String)i);
            }
            dec0->use();
            ElectricalModel* dec1 = (ElectricalModel*)getSubInstance("Dec_way1");
            for(unsigned int i = 0; i < number_addr_bits_1; ++i)
            {
                propagatePortTransitionInfo(dec1, "Addr" + (String)i, "Addr" + (String)(i + number_addr_bits_0));
            }
            dec1->use();

            for(unsigned int i = 0; i < number_outputs; ++i)
            {
                ElectricalModel* nand2 = (ElectricalModel*)getSubInstance("NAND2_" + (String)i);
                propagatePortTransitionInfo(nand2, "A", dec0, "Out" + (String)(i%number_outputs_0));
                propagatePortTransitionInfo(nand2, "B", dec1, "Out" + (String)((unsigned int)floor(i/number_outputs_0)));
                nand2->use();

                ElectricalModel* inv = (ElectricalModel*)getSubInstance("INV_" + (String)i);
                propagatePortTransitionInfo(inv, "A", nand2, "Y");
                inv->use();

                propagatePortTransitionInfo("Out" + (String)i, inv, "Y");
            }
        }
        return;
    }

} // namespace DSENT

