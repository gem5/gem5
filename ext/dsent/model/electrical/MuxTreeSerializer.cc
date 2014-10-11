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

#include "model/electrical/MuxTreeSerializer.h"

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

    MuxTreeSerializer::MuxTreeSerializer(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    MuxTreeSerializer::~MuxTreeSerializer()
    {}

    void MuxTreeSerializer::initParameters()
    {
        addParameterName("InDataRate");
        addParameterName("OutDataRate");
        addParameterName("InBits");              //Output width will just be input width / serialization ratio
    }

    void MuxTreeSerializer::initProperties()
    {
        return;
    }

    MuxTreeSerializer* MuxTreeSerializer::clone() const
    {
        // TODO
        return NULL;
    }

    void MuxTreeSerializer::constructModel()
    {
        // Get parameters
        double in_data_rate = getParameter("InDataRate").toDouble();
        double out_data_rate = getParameter("OutDataRate").toDouble();
        unsigned int in_bits = getParameter("InBits").toUInt();
        
        // Calculate serialization ratio
        unsigned int serialization_ratio = (unsigned int) floor(out_data_rate / in_data_rate);    
        ASSERT(serialization_ratio == out_data_rate / in_data_rate,
            "[Error] " + getInstanceName() + " -> Cannot have non-integer serialization ratios " +
            "(" + (String) (in_data_rate / out_data_rate) + ")!");
                
        // Calculate output width
        ASSERT(floor((double) in_bits / serialization_ratio) == (double) in_bits / serialization_ratio,
            "[Error] " + getInstanceName() + " -> Input width (" + (String) in_bits + ") " +
            "must be a multiple of the serialization ratio (" + (String) serialization_ratio + ")!");
        unsigned int output_bits = in_bits / serialization_ratio;
        
        // Calculate the number of multiplexer stages
        unsigned int number_stages = (unsigned int)ceil(log2((double) serialization_ratio));                    
            
        // Store calculated values
        getGenProperties()->set("SerializationRatio", serialization_ratio);
        getGenProperties()->set("OutputBits", output_bits);
        getGenProperties()->set("NumberStages", number_stages);
                
        // Create ports
        createInputPort("In", makeNetIndex(0, in_bits-1));
        createInputPort("OutCK");
        createOutputPort("Out", makeNetIndex(0, output_bits-1));

        //Create energy, power, and area results
        createElectricalResults();
        createElectricalEventResult("Serialize");
        getEventInfo("Serialize")->setTransitionInfo("OutCK", TransitionInfo(0.0, (double) serialization_ratio / 2.0, 0.0));
        //Set conditions during idle state
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("OutCK", TransitionInfo(0.0, (double) serialization_ratio / 2.0, 0.0));
        
        // Mark OutCK as a false path (since timing tool will do strange stuff due to all the clock divides and stuff)
        getNet("OutCK")->setFalsePath(true);
        
        // Create mux-tree instance
        if (serialization_ratio == 1)
        {
            // No need to do anything, hohoho
            assign("Out", "In");
        }
        else
        {
            // Create multiplexer
            String mux_tree_name = "MuxTree";        
            ElectricalModel* mux_tree = new Multiplexer(mux_tree_name, getTechModel());
            mux_tree->setParameter("NumberInputs", serialization_ratio);
            mux_tree->setParameter("NumberBits", output_bits);
            mux_tree->setParameter("BitDuplicate", "TRUE");
            mux_tree->construct();
            // Create nets
            if (number_stages > 1)
                createNet("MuxSel_b", makeNetIndex(0, number_stages-2));
            createNet("MuxSel", makeNetIndex(0, number_stages-1));
            assign("MuxSel", makeNetIndex(number_stages-1), "OutCK");
            // Create reindexed net (to help out with indexing)
            createNet("InTmp", makeNetIndex(0, in_bits-1));            
            for (unsigned int i = 0; i < serialization_ratio; ++i)
                for (unsigned int j = 0; j < output_bits; ++j)
                    assign("InTmp", makeNetIndex(i*output_bits+j), "In", makeNetIndex(j*serialization_ratio+i));

            // Connect ports
            for (unsigned int i = 0; i < serialization_ratio; ++i)
                portConnect(mux_tree, "In" + (String) i, "InTmp", makeNetIndex(i*output_bits, (i+1)*output_bits-1));
            
            for (unsigned int i = 0; i < number_stages; ++i)
                portConnect(mux_tree, "Sel" + (String) i, "MuxSel", makeNetIndex(i));
            portConnect(mux_tree, "Out", "Out");

            // Add subinstance and events
            addSubInstances(mux_tree, 1.0);
            addElectricalSubResults(mux_tree, 1.0);
            // Add serialize event/power
            getEventResult("Serialize")->addSubResult(mux_tree->getEventResult("Mux"), mux_tree_name, 1.0);

            // Create clock dividers (assumes power of 2...), don't need divider for fastest output stage
            for (unsigned int i = 0; i < number_stages - 1; ++i)
            {
                // Clk dividing registers
                const String& clk_div_dff_name = "ClkDivDFF_" + (String) i;
                StdCell* clk_div_dff = getTechModel()->getStdCellLib()->createStdCell("DFFQ", clk_div_dff_name);                
                clk_div_dff->construct();
                portConnect(clk_div_dff, "D", "MuxSel_b", makeNetIndex(i));
                portConnect(clk_div_dff, "Q", "MuxSel", makeNetIndex(i));
                portConnect(clk_div_dff, "CK", "MuxSel", makeNetIndex(i+1));
                addSubInstances(clk_div_dff, 1.0);
                addElectricalSubResults(clk_div_dff, 1.0);
                
                // Inversions
                const String& clk_div_inv_name = "ClkDivINV_" + (String) i;
                StdCell* clk_div_inv = getTechModel()->getStdCellLib()->createStdCell("INV", clk_div_inv_name);
                clk_div_inv->construct();
                portConnect(clk_div_inv, "A", "MuxSel", makeNetIndex(i));
                portConnect(clk_div_inv, "Y", "MuxSel_b", makeNetIndex(i));
                addSubInstances(clk_div_inv, 1.0);
                addElectricalSubResults(clk_div_inv, 1.0);
                            
                getEventResult("Serialize")->addSubResult(clk_div_dff->getEventResult("CK"), clk_div_dff_name, 1.0); 
                getEventResult("Serialize")->addSubResult(clk_div_dff->getEventResult("DFFD"), clk_div_dff_name, 1.0); 
                getEventResult("Serialize")->addSubResult(clk_div_dff->getEventResult("DFFQ"), clk_div_dff_name, 1.0); 
                getEventResult("Serialize")->addSubResult(clk_div_inv->getEventResult("INV"), clk_div_inv_name, 1.0);
            }
        }

        return;
    }

    void MuxTreeSerializer::propagateTransitionInfo()
    {
        // Get some generated properties
        const unsigned int serialization_ratio = getGenProperties()->get("SerializationRatio");
        const unsigned int number_stages = getGenProperties()->get("NumberStages");
        
        // Set transition info of the mux tree and clock divide DFF
        if (serialization_ratio == 1)
        {
            // If no serialization, then just propagate input transition info to output port
            propagatePortTransitionInfo("Out", "In");
        }
        else
        {

            // Propagate transition probabilities to the mux tree
            ElectricalModel* mux_tree = (ElectricalModel*) getSubInstance("MuxTree");
            // All input ports of the mux have the same probability
            for (unsigned int i = 0; i < serialization_ratio; ++i)
                propagatePortTransitionInfo(mux_tree, "In" + (String) i, "In");
            // Connect last stage of the mux
            propagatePortTransitionInfo(mux_tree, "Sel" + (String) (number_stages - 1), "OutCK");                
            // Keep track of the last clock divider
            ElectricalModel* last_clk_div_dff = NULL;
            // Find P01 of OutCK
            double last_P01_CK = getInputPort("OutCK")->getTransitionInfo().getNumberTransitions01();
            // Start from the last stage (since it is the stage with no clock division)
            for (unsigned int i = 0; i < number_stages - 1; ++i)
            {
                const String& clk_div_dff_name = "ClkDivDFF_" + (String) (number_stages - i - 2);
                const String& clk_div_inv_name = "ClkDivINV_" + (String) (number_stages - i - 2);
                
                ElectricalModel* clk_div_dff = (ElectricalModel*) getSubInstance(clk_div_dff_name);
                if (last_clk_div_dff == NULL)
                    propagatePortTransitionInfo(clk_div_dff, "CK", "OutCK");
                else
                    propagatePortTransitionInfo(clk_div_dff, "CK", last_clk_div_dff, "Q");
                // Since it is a clock divider, P01 is D and Q are simply half the P01 of D and Q of
                // the input clock
                if (last_P01_CK != 0) clk_div_dff->getInputPort("D")->setTransitionInfo(TransitionInfo(0.0, last_P01_CK * 0.5, 0.0));
                else clk_div_dff->getInputPort("D")->setTransitionInfo(TransitionInfo(0.5, 0.0, 0.5));

                clk_div_dff->use();

                ElectricalModel* clk_div_inv = (ElectricalModel*) getSubInstance(clk_div_inv_name);
                propagatePortTransitionInfo(clk_div_inv, "A", clk_div_dff, "Q");
                clk_div_inv->use();
                
                // Connect select port of the mux
                propagatePortTransitionInfo(mux_tree, "Sel" + (String) (number_stages - i - 2), clk_div_dff, "Q");
                
                // Clk divide by 2;
                last_P01_CK = last_P01_CK * 0.5;
                // Remember the last clk div DFF
                last_clk_div_dff = clk_div_dff;
            }
            
            mux_tree->use();
            // Set output transition info to be the output transition info of the mux tree
            propagatePortTransitionInfo("Out", mux_tree, "Out");            
        }

        return;
    }

} // namespace DSENT

