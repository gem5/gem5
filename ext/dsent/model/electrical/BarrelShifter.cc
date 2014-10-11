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

#include "model/electrical/BarrelShifter.h"
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
    BarrelShifter::BarrelShifter(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    BarrelShifter::~BarrelShifter()
    {}

    void BarrelShifter::initParameters()
    {
        addParameterName("NumberBits");
        addParameterName("ShiftIndexMin");
        addParameterName("ShiftIndexMax");
        addParameterName("BitDuplicate", "TRUE");
        return;
    }

    void BarrelShifter::initProperties()
    {
        return;
    }

    BarrelShifter* BarrelShifter::clone() const
    {
        return NULL;
    }

    void BarrelShifter::constructModel()
    {
        // Get parameters
        unsigned int number_bits = getParameter("NumberBits");
        unsigned int number_shift_bits = (unsigned int)ceil(log2((double) number_bits));
        unsigned int shift_index_min = getParameter("ShiftIndexMin");
        unsigned int shift_index_max = getParameter("ShiftIndexMax");
        bool bit_duplicate = (bool) getParameter("BitDuplicate");
        
        ASSERT(number_bits > 0, "[Error] " + getInstanceName() + " -> Number of bits must be > 0!");
        // No need to check these if there arent any shifts
        if (number_shift_bits > 0)
        {
            ASSERT(shift_index_min <= number_shift_bits,
                "[Error] " + getInstanceName() + " -> Min shift index must be >= 0 and <= " +
                "the total number of shift bits!");
            ASSERT(shift_index_max >= shift_index_min && shift_index_max <= number_shift_bits,
                "[Error] " + getInstanceName() + " -> Max shift index must be >= minimum shift index and <= " +
                "the total number of shift bits!");
        }
    
        //Construct electrical ports
        //Create each input port
        createInputPort(    "In", makeNetIndex(0, number_bits-1));
        //Create output
        createOutputPort(   "Out", makeNetIndex(0, number_bits-1));
        
        //Create shift ports (which only exists if there is something to shift)
        if (shift_index_min != number_shift_bits)
            for (unsigned int i = shift_index_min; i <= shift_index_max; ++i)
                createInputPort(    "Shift" + (String) i);

        //Create energy, power, and area results
        createElectricalResults();
        createElectricalEventResult("BarrelShift");
        //Set conditions during idle event
        getEventInfo("Idle")->setStaticTransitionInfos();

        //If the input is only 1-bit, connect input to output and be done
        if (number_shift_bits == 0 || (shift_index_min == number_shift_bits))
            assign("Out", "In");
        else
        {
            for (unsigned int i = shift_index_min; i <= shift_index_max; ++i)
            {
                //Create internally buffered shift select signals
                createNet("Shift_b" + (String) i);
                createNet("Shift_i" + (String) i);
            }                                
        
            // Create shift and shifted signals
            for (unsigned int i = shift_index_min; i <= shift_index_max; ++i)
            {
                unsigned int current_shifts = (unsigned int)pow(2, i);
                const String& n = (String) current_shifts;
                //Instantiate and connect intermediate nets. In a barrel-shifter, the nets do
                //all the "shifting" and the muxes just select which to take
                createNet("R_" + n, makeNetIndex(0, number_bits-1));               //wire R_n[number_bits-1:0]
                createNet("RS_" + n, makeNetIndex(0, number_bits-1));              //wire RS_n[number_bits-1:0]

                //Implements the shifts
                //assign RS_n[number_bits-1:number_bits-current_shifts] = R_n[current_shifts-1:0];
                assign("RS_" + n, makeNetIndex(number_bits-current_shifts, number_bits-1),
                        "R_" + n, makeNetIndex(0, current_shifts-1));
                //assign RS_n[number_bits-current_shifts-1:0] = R_n[current_shifts:number_bits-1];
                assign("RS_" + n, makeNetIndex(0, number_bits-current_shifts-1),
                        "R_" + n, makeNetIndex(current_shifts, number_bits-1));                
            }
            
            const String& n_max = (String) pow(2, shift_index_max+1);            
            const String& n_min = (String) pow(2, shift_index_min);
            // Create the R_(max) net
            createNet("R_" + n_max, makeNetIndex(0, number_bits-1));            
            // Set R_1 to be the input
            assign("R_" + n_min, "In");
            // Set R_(max) to be the output
            assign("Out", "R_" + n_max, makeNetIndex(0, number_bits-1));

            for (unsigned int i = shift_index_min; i <= shift_index_max; ++i)
            {
                unsigned int current_shifts = (unsigned int)pow(2, i);
                const String& n = (String) current_shifts;
                const String& n_next = (String) (current_shifts * 2);
                
                const String& buf_inv_0_name = "ShiftBufInv0_" + (String) n;
                const String& buf_inv_1_name = "ShiftBufInv1_" + (String) n;
                // Create shift buffer inverters
                StdCell* buf_inv_0 = getTechModel()->getStdCellLib()->createStdCell("INV", buf_inv_0_name);
                buf_inv_0->construct();
                StdCell* buf_inv_1 = getTechModel()->getStdCellLib()->createStdCell("INV", buf_inv_1_name);
                buf_inv_1->construct();
                
                // Connect up shift buffer inverters
                portConnect(buf_inv_0, "A", "Shift" + (String) i);
                portConnect(buf_inv_0, "Y", "Shift_b" + (String) i);
                portConnect(buf_inv_1, "A", "Shift_b" + (String) i);
                portConnect(buf_inv_1, "Y", "Shift_i" + (String) i);
                
                // Add area, power, and event results for inverters
                addSubInstances(buf_inv_0, 1.0);
                addSubInstances(buf_inv_1, 1.0);
                addElectricalSubResults(buf_inv_0, 1.0);
                addElectricalSubResults(buf_inv_1, 1.0);
                getEventResult("BarrelShift")->addSubResult(buf_inv_0->getEventResult("INV"), buf_inv_0_name, 1.0);                
                getEventResult("BarrelShift")->addSubResult(buf_inv_1->getEventResult("INV"), buf_inv_1_name, 1.0);                
                
                //Instantiate 2:1 multiplexers, one for each shift bit.
                const String& mux_name = "SRL" + n;
                Multiplexer* mux = new Multiplexer(mux_name, getTechModel());
                mux->setParameter("NumberInputs", 2);
                mux->setParameter("NumberBits", number_bits);
                mux->setParameter("BitDuplicate", bit_duplicate);
                mux->construct();
                
                //Just have to connect the In0 and In1 inputs of the mux to the
                //non-shifted and shifted intermediate signals, respectively.                
                portConnect(mux, "In0", "R_" + n);
                portConnect(mux, "In1", "RS_" + n);
                //Selector connects to the shift signal for that index
                portConnect(mux, "Sel0", "Shift_i" + (String) i);
                //Connect mux output
                portConnect(mux, "Out", "R_" + n_next);
                
                //Add area, power, and event results for each mux
                addSubInstances(mux, 1.0);
                addElectricalSubResults(mux, 1.0);
                getEventResult("BarrelShift")->addSubResult(mux->getEventResult("Mux"), mux_name, 1.0);                
            }
        }
        return;
    }
    
    void BarrelShifter::propagateTransitionInfo()
    {
        // The only thing can be updated are the input probabilities...so we will update them
        unsigned int number_bits = (unsigned int) getParameter("NumberBits");
        unsigned int number_shift_bits = (unsigned int) ceil(log2((double) number_bits));
        unsigned int shift_index_min = getParameter("ShiftIndexMin");
        unsigned int shift_index_max = getParameter("ShiftIndexMax");        
        
        // Keep track of the multiplexer of the last stage
        ElectricalModel* last_mux = NULL;        
        // We only need to update stuff if we are not shifting by exact multiples
        // of number of input bits
        if (shift_index_min < number_shift_bits)
        {
            for (unsigned int i = shift_index_min; i <= shift_index_max; ++i)
            {        
                unsigned int current_shifts = (unsigned int)pow(2, i);
                String n = (String) current_shifts;

                // Set the 
                const String& buf_inv_0_name = "ShiftBufInv0_" + (String) n;
                const String& buf_inv_1_name = "ShiftBufInv1_" + (String) n;
                const String& mux_name = "SRL" + n;

                // Set the transition infos for the inverter buffers
                ElectricalModel* buf_inv_0 = (ElectricalModel*) getSubInstance(buf_inv_0_name);
                propagatePortTransitionInfo(buf_inv_0, "A", "Shift" + (String) i);
                buf_inv_0->use();

                ElectricalModel* buf_inv_1 = (ElectricalModel*) getSubInstance(buf_inv_1_name);
                propagatePortTransitionInfo(buf_inv_1, "A", buf_inv_0, "Y");
                buf_inv_1->use();
                
                // Set the transition infos for the shift multiplexers
                ElectricalModel* mux = (ElectricalModel*) getSubInstance(mux_name);
                propagatePortTransitionInfo(mux, "Sel0", buf_inv_1, "Y");
                if (last_mux == NULL)
                {
                    propagatePortTransitionInfo(mux, "In0", "In");
                    propagatePortTransitionInfo(mux, "In1", "In");
                }
                else
                {
                    propagatePortTransitionInfo(mux, "In0", last_mux, "Out");
                    propagatePortTransitionInfo(mux, "In1", last_mux, "Out");                    
                }
                mux->use();                
                
                // Set this to be the last mux visted
                last_mux = mux;
            }                        
        }
        
        // If there isn't anything to shift
        if (last_mux == NULL)
            propagatePortTransitionInfo("Out", "In");
        // Take the transition info of the last mux
        else
            propagatePortTransitionInfo("Out", last_mux, "Out");
            
        return;
    }
    
} // namespace DSENT

