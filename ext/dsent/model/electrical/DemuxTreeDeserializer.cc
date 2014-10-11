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

#include "model/electrical/DemuxTreeDeserializer.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/std_cells/StdCellLib.h"
#include "model/std_cells/StdCell.h"
#include "model/electrical/Multiplexer.h"
#include "model/timing_graph/ElectricalNet.h"

namespace DSENT
{
    using std::ceil;

    DemuxTreeDeserializer::DemuxTreeDeserializer(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    DemuxTreeDeserializer::~DemuxTreeDeserializer()
    {}

    void DemuxTreeDeserializer::initParameters()
    {
        addParameterName("InDataRate");
        addParameterName("OutDataRate");
        addParameterName("OutBits");              //Output width will just be output width / serialization ratio
        addParameterName("BitDuplicate", "TRUE");
        return;
    }

    void DemuxTreeDeserializer::initProperties()
    {
        return;
    }

    DemuxTreeDeserializer* DemuxTreeDeserializer::clone() const
    {
        // TODO
        return NULL;
    }

    void DemuxTreeDeserializer::constructModel()
    {

        // Get parameters
        double in_data_rate = getParameter("InDataRate");
        double out_data_rate = getParameter("OutDataRate");
        unsigned int out_bits = getParameter("OutBits");
        bool bit_duplicate = getParameter("BitDuplicate");                        
        
        // Calculate deserialization ratio
        unsigned int deserialization_ratio = (unsigned int) floor(in_data_rate / out_data_rate);    
        ASSERT(deserialization_ratio == in_data_rate / out_data_rate,
            "[Error] " + getInstanceName() + " -> Cannot have non-integer deserialization ratios!");
        ASSERT((deserialization_ratio & (deserialization_ratio - 1)) == 0,
            "[Error] " + getInstanceName() + " -> Deserialization ratio must be a power of 2");
        
        // Calculate output width
        unsigned int input_bits = out_bits / deserialization_ratio;
        ASSERT(out_bits >= deserialization_ratio, "[Error] " + getInstanceName() + 
            " -> Output width must be >= deserialization ratio!");
        ASSERT(floor((double) out_bits / deserialization_ratio) == input_bits,
            "[Error] " + getInstanceName() + " -> Output width must be a multiple of the serialization ratio!");
                
        // Store calculated numbers
        getGenProperties()->set("DeserializationRatio", deserialization_ratio);
        getGenProperties()->set("InputBits", input_bits);
        
        // Create ports
        createInputPort("In", makeNetIndex(0, input_bits-1));
        createInputPort("InCK");
        createOutputPort("Out", makeNetIndex(0, out_bits-1));        
        
        //Create energy, power, and area results
        createElectricalResults();
        createElectricalEventResult("Deserialize");
        getEventInfo("Deserialize")->setTransitionInfo("InCK", TransitionInfo(0.0, (double) deserialization_ratio / 2.0, 0.0));
        // Set conditions during idle state
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("InCK", TransitionInfo(0.0, (double) deserialization_ratio / 2.0, 0.0));

        // Mark InCK as a false path (since timing tool will do strange stuff due to all the clock divides and stuff)
        getNet("InCK")->setFalsePath(true);
        
        // Create deserializer
        if (deserialization_ratio == 1)
        {
            // No need to do anything, hohoho
            assign("Out", "In");
        }
        else if (input_bits == 1)
        {
            //-----------------------------------------------------------------
            // Create 2:1 demux deserializer
            //-----------------------------------------------------------------
            const String& des_dff_way0_name = "DesDFFWay0";
            const String& des_dff_way1_name = "DesDFFWay1";                
            const String& des_latch_name = "DesLatch";
            const String& ck_dff_name = "CKDFF";
            const String& ck_inv_name = "CKINV";            
            const String& out_way0_name = "OutWay0";
            const String& out_way1_name = "OutWay1";
            const String& mid_way0_name = "MidWay0";
            const String& ck_div2_name = "CK_div2";
            const String& ck_div2_b_name = "CK_div2_b";
            
            // Create nets
            createNet(out_way0_name);
            createNet(out_way1_name);
            createNet(mid_way0_name);
            createNet(ck_div2_name);
            createNet(ck_div2_b_name);
            
            // Create the dffs and latch needed on both ways
            StdCell* des_dff_way0 = getTechModel()->getStdCellLib()->createStdCell("DFFQ", des_dff_way0_name);
            des_dff_way0->construct();
            StdCell* des_dff_way1 = getTechModel()->getStdCellLib()->createStdCell("DFFQ", des_dff_way1_name);
            des_dff_way1->construct();
            StdCell* des_latch = getTechModel()->getStdCellLib()->createStdCell("LATQ", des_latch_name);
            des_latch->construct();
            
            // Create clk divide circuit
            StdCell* ck_dff = getTechModel()->getStdCellLib()->createStdCell("DFFQ", ck_dff_name);
            ck_dff->construct();            
            StdCell* ck_inv = getTechModel()->getStdCellLib()->createStdCell("INV", ck_inv_name);
            ck_inv->construct();
            
            // Connect ports
            portConnect(des_dff_way0, "CK", "InCK");
            portConnect(des_dff_way0, "D", mid_way0_name);
            portConnect(des_dff_way0, "Q", out_way0_name);
            portConnect(des_latch, "G", "InCK");
            portConnect(des_latch, "D", "In");
            portConnect(des_latch, "Q", mid_way0_name);
            portConnect(des_dff_way1, "CK", "InCK");
            portConnect(des_dff_way1, "D", "In");
            portConnect(des_dff_way1, "Q", out_way1_name);
            portConnect(ck_dff, "CK", "InCK");
            portConnect(ck_dff, "D", ck_div2_b_name);
            portConnect(ck_dff, "Q", ck_div2_name);
            portConnect(ck_inv, "A", ck_div2_name);
            portConnect(ck_inv, "Y", ck_div2_b_name);
            
            // Add sub instances
            addSubInstances(des_dff_way0, 1.0);
            addElectricalSubResults(des_dff_way0, 1.0);
            addSubInstances(des_dff_way1, 1.0);
            addElectricalSubResults(des_dff_way1, 1.0);
            addSubInstances(des_latch, 1.0);
            addElectricalSubResults(des_latch, 1.0);
            addSubInstances(ck_dff, 1.0);
            addElectricalSubResults(ck_dff, 1.0);
            addSubInstances(ck_inv, 1.0);
            addElectricalSubResults(ck_inv, 1.0);

            Result* deserialize = getEventResult("Deserialize");
            deserialize->addSubResult(des_dff_way0->getEventResult("CK"), des_dff_way0_name, 1.0);
            deserialize->addSubResult(des_dff_way0->getEventResult("DFFD"), des_dff_way0_name, 1.0);
            deserialize->addSubResult(des_dff_way0->getEventResult("DFFQ"), des_dff_way0_name, 1.0);
            deserialize->addSubResult(des_dff_way1->getEventResult("CK"), des_dff_way1_name, 1.0);
            deserialize->addSubResult(des_dff_way1->getEventResult("DFFD"), des_dff_way1_name, 1.0);
            deserialize->addSubResult(des_dff_way1->getEventResult("DFFQ"), des_dff_way1_name, 1.0);
            deserialize->addSubResult(des_latch->getEventResult("G"), des_latch_name, 1.0);
            deserialize->addSubResult(des_latch->getEventResult("LATD"), des_latch_name, 1.0);
            deserialize->addSubResult(des_latch->getEventResult("LATQ"), des_latch_name, 1.0);
            deserialize->addSubResult(ck_dff->getEventResult("CK"), ck_dff_name, 1.0);
            deserialize->addSubResult(ck_dff->getEventResult("DFFD"), ck_dff_name, 1.0);
            deserialize->addSubResult(ck_dff->getEventResult("DFFQ"), ck_dff_name, 1.0);
            deserialize->addSubResult(ck_inv->getEventResult("INV"), ck_inv_name, 1.0);
            //-----------------------------------------------------------------
            
            //-----------------------------------------------------------------
            // Create Sub-deserializers
            //-----------------------------------------------------------------
            // Create sub-deserializers
            const String& demux_way0_name = "DemuxTree_way0_" + (String) deserialization_ratio + "_to_1";
            const String& demux_way1_name = "DemuxTree_way1_" + (String) deserialization_ratio + "_to_1";
            
            DemuxTreeDeserializer* demux_way0 = new DemuxTreeDeserializer(demux_way0_name, getTechModel());
            demux_way0->setParameter("InDataRate", in_data_rate / 2.0);
            demux_way0->setParameter("OutDataRate", out_data_rate);
            demux_way0->setParameter("OutBits", out_bits / 2);
            demux_way0->setParameter("BitDuplicate", "TRUE");
            demux_way0->construct();            
            
            DemuxTreeDeserializer* demux_way1 = new DemuxTreeDeserializer(demux_way1_name, getTechModel());
            demux_way1->setParameter("InDataRate", in_data_rate / 2.0);
            demux_way1->setParameter("OutDataRate", out_data_rate);
            demux_way1->setParameter("OutBits", out_bits / 2);
            demux_way1->setParameter("BitDuplicate", "TRUE");
            demux_way1->construct();
            
            // Connect ports
            portConnect(demux_way0, "In", out_way0_name);
            portConnect(demux_way0, "InCK", ck_div2_name);
            portConnect(demux_way0, "Out", "Out", makeNetIndex(0, out_bits/2-1));
            
            portConnect(demux_way1, "In", out_way1_name);
            portConnect(demux_way1, "InCK", ck_div2_name);
            portConnect(demux_way1, "Out", "Out", makeNetIndex(out_bits/2, out_bits-1));
            
            // Add subinstances and area results
            addSubInstances(demux_way0, 1.0);
            addElectricalSubResults(demux_way0, 1.0);
            addSubInstances(demux_way1, 1.0);
            addElectricalSubResults(demux_way1, 1.0);

            deserialize->addSubResult(demux_way0->getEventResult("Deserialize"), demux_way0_name, 1.0);
            deserialize->addSubResult(demux_way1->getEventResult("Deserialize"), demux_way1_name, 1.0);
            //-----------------------------------------------------------------
            
        }
        else if (bit_duplicate)
        {
            const String& demux_name = "DemuxTree_" + (String) deserialization_ratio + "_to_1";

            DemuxTreeDeserializer* des_bit = new DemuxTreeDeserializer(demux_name, getTechModel());
            des_bit->setParameter("InDataRate", in_data_rate);
            des_bit->setParameter("OutDataRate", out_data_rate);
            des_bit->setParameter("OutBits", deserialization_ratio);
            des_bit->setParameter("BitDuplicate", "TRUE");
            des_bit->construct();
            
            // Create VFI and VFO nets
            createNet("InVFI");
            createNet("OutVFO", makeNetIndex(0, deserialization_ratio-1));
            
            // Connect ports
            portConnect(des_bit, "In", "InVFI");
            portConnect(des_bit, "Out", "OutVFO");
                
            // Do VFI and VFO
            assignVirtualFanin("InVFI", "In");
            for (unsigned int i = 0; i < input_bits; ++i)
            {
                portConnect(des_bit, "InCK", "InCK");                        
                for (unsigned int j = 0; j < deserialization_ratio; ++j)
                    assignVirtualFanout("Out", makeNetIndex(i*deserialization_ratio + j), "OutVFO", makeNetIndex(j));
            }
            // Add subinstances and area results
            addSubInstances(des_bit, input_bits);
            addElectricalSubResults(des_bit, input_bits);
            getEventResult("Deserialize")->addSubResult(des_bit->getEventResult("Deserialize"), demux_name, input_bits);            
        }
        else
        {
            //Instantiate a bunch of 1 input bit deserializers
            for (unsigned int i = 0; i < input_bits; ++i)
            {
                const String& demux_name = "DemuxTree_" + (String) deserialization_ratio + "_to_1_bit" + (String) i;
                
                DemuxTreeDeserializer* des_bit = new DemuxTreeDeserializer(demux_name, getTechModel());
                des_bit->setParameter("InDataRate", in_data_rate);
                des_bit->setParameter("OutDataRate", out_data_rate);
                des_bit->setParameter("OutBits", deserialization_ratio);
                des_bit->setParameter("BitDuplicate", "TRUE");
                des_bit->construct();
                
                portConnect(des_bit, "In", "In", makeNetIndex(i));
                portConnect(des_bit, "InCK", "InCK");
                portConnect(des_bit, "Out", "Out", makeNetIndex(i*deserialization_ratio, (i+1)*deserialization_ratio-1));
                
                addSubInstances(des_bit, 1.0);
                addElectricalSubResults(des_bit, 1.0);
                getEventResult("Deserialize")->addSubResult(des_bit->getEventResult("Deserialize"), demux_name, 1.0);
            }
        }

        return;
    }

    void DemuxTreeDeserializer::propagateTransitionInfo()
    {
        // Get parameters
        bool bit_duplicate = getParameter("BitDuplicate");                        
        // Get generated properties
        unsigned int deserialization_ratio = getGenProperties()->get("DeserializationRatio");
        unsigned int input_bits = getGenProperties()->get("InputBits");
                
        // Calculate output transitions and activities
        if (deserialization_ratio == 1)
        {
            // If no deserialization, then just propagate input transition info to output port
            propagatePortTransitionInfo("Out", "In");
        }
        else if (input_bits == 1)
        {            
            const String& des_dff_way0_name = "DesDFFWay0";
            const String& des_dff_way1_name = "DesDFFWay1";                
            const String& des_latch_name = "DesLatch";
            const String& ck_dff_name = "CKDFF";
            const String& ck_inv_name = "CKINV";            

            // Sub-deserializer names
            const String& demux_way0_name = "DemuxTree_way0_" + (String) deserialization_ratio + "_to_1";
            const String& demux_way1_name = "DemuxTree_way1_" + (String) deserialization_ratio + "_to_1";

            // Update transition info for deserialization registers/latches
            ElectricalModel* des_latch = (ElectricalModel*) getSubInstance(des_latch_name);
            propagatePortTransitionInfo(des_latch, "G", "InCK");
            propagatePortTransitionInfo(des_latch, "D", "In");
            des_latch->use();

            ElectricalModel* des_dff_way0 = (ElectricalModel*) getSubInstance(des_dff_way0_name);
            propagatePortTransitionInfo(des_dff_way0, "CK", "InCK");
            propagatePortTransitionInfo(des_dff_way0, "D", des_latch, "Q");
            des_dff_way0->use();

            ElectricalModel* des_dff_way1 = (ElectricalModel*) getSubInstance(des_dff_way1_name);
            propagatePortTransitionInfo(des_dff_way1, "CK", "InCK");
            propagatePortTransitionInfo(des_dff_way1, "D", "In");
            des_dff_way1->use();

            // Get input transitions of input clock
            double P01_CK = getInputPort("InCK")->getTransitionInfo().getNumberTransitions01();
            // Update transition info for clk division DFF
            ElectricalModel* ck_dff = (ElectricalModel*) getSubInstance(ck_dff_name);
            propagatePortTransitionInfo(ck_dff, "CK", "InCK");
            // Since it is a clock divider, P01 is D and Q are simply half the P01 of D and Q of
            // the input clock
            if (P01_CK != 0) ck_dff->getInputPort("D")->setTransitionInfo(TransitionInfo(0.0, P01_CK * 0.5, 0.0));
            else ck_dff->getInputPort("D")->setTransitionInfo(TransitionInfo(0.5, 0.0, 0.5));

            ck_dff->use();
            // Update transition info of clk divided inverter
            ElectricalModel* ck_inv = (ElectricalModel*) getSubInstance(ck_inv_name);
            propagatePortTransitionInfo(ck_inv, "A", ck_dff, "Q");
            ck_inv->use();
            
            // Update transition info for next demux stages
            ElectricalModel* demux_way0 = (ElectricalModel*) getSubInstance(demux_way0_name);
            propagatePortTransitionInfo(demux_way0, "In", des_dff_way0, "Q");
            propagatePortTransitionInfo(demux_way0, "InCK", ck_dff, "Q");
            demux_way0->use();
            ElectricalModel* demux_way1 = (ElectricalModel*) getSubInstance(demux_way1_name);
            propagatePortTransitionInfo(demux_way1, "In", des_dff_way1, "Q");
            propagatePortTransitionInfo(demux_way1, "InCK", ck_dff, "Q");
            demux_way1->use();
            
            propagatePortTransitionInfo("Out", demux_way0, "Out");            
        }
        else if (bit_duplicate)
        {
            // Propagate transition info
            const String& demux_name = "DemuxTree_" + (String) deserialization_ratio + "_to_1";
            ElectricalModel* demux = (ElectricalModel*) getSubInstance(demux_name);
            propagatePortTransitionInfo(demux, "In", "In");
            propagatePortTransitionInfo(demux, "InCK", "InCK");
            demux->use();

            propagatePortTransitionInfo("Out", demux, "Out");            
        }
        else
        {
            // Set output probability to be average that of probabilties of each output bit
            // Update all 1 bit deserializers
            for (unsigned int i = 0; i < input_bits; ++i)
            {
                const String& demux_name = "DemuxTree_" + (String) deserialization_ratio + "_to_1_bit" + (String) i;
                ElectricalModel* demux_bit = (ElectricalModel*) getSubInstance(demux_name);
                propagatePortTransitionInfo(demux_bit, "In", "In");
                propagatePortTransitionInfo(demux_bit, "InCK", "InCK");
                demux_bit->use();

                propagatePortTransitionInfo("Out", demux_bit, "Out");
            }
        }
        
        return;
    }

} // namespace DSENT

