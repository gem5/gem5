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

#include "model/electrical/TestModel.h"

#include <cmath>

#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"
#include "model/electrical/RippleAdder.h"
#include "model/electrical/Multiplexer.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalLoad.h"
#include "model/timing_graph/ElectricalTimingTree.h"

namespace DSENT
{
    TestModel::TestModel(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initProperties();
    }

    TestModel::~TestModel()
    {}

    void TestModel::initProperties()
    {
        return;
    }

    TestModel* TestModel::clone() const
    {
        return NULL;
    }

    void TestModel::constructModel()
    {
        unsigned int num_bits = 64;
        unsigned int mux_bits = 1;
    
        // Create the instance        
        createNet("CK");
        createNet("CI");
        getNet("CI")->setDistributedCap(100e-15);
        getNet("CI")->setDistributedRes(10);
        createNet("CO");
        createNet("A", makeNetIndex(0, num_bits - 1));
        createNet("B", makeNetIndex(0, num_bits - 1));
        createNet("S", makeNetIndex(0, num_bits - 1));        
        
        StdCell* ci_reg = getTechModel()->getStdCellLib()->createStdCell("DFFQ", "DFFQ-CI");
        ci_reg->setProperty("P(D)", 0.5);
        ci_reg->setProperty("P(CK)", 0.5);
        ci_reg->construct();
        portConnect(ci_reg, "Q", "CI");
        portConnect(ci_reg, "CK", "CK");
        //ci_reg->connect("Q", getNet("CI"));
        //ci_reg->connect("CK", getNet("CK"));
        addSubInstances(ci_reg, 1.0);

        StdCell* co_reg = getTechModel()->getStdCellLib()->createStdCell("DFFQ", "DFFQ-CO");
        co_reg->setProperty("P(D)", 0.5);
        co_reg->setProperty("P(CK)", 0.5);
        co_reg->construct();
        portConnect(co_reg, "D", "CO");
        portConnect(co_reg, "CK", "CK");        
        //co_reg->connect("D", getNet("CO"));
        //co_reg->connect("CK", getNet("CK"));
        addSubInstances(co_reg, 1.0);
        
        for (unsigned int i = 0; i < num_bits; i++)
        {
            StdCell* a_reg = getTechModel()->getStdCellLib()->createStdCell("DFFQ", "DFFQ-A[" + (String) i + "]");
            a_reg->setProperty("P(D)", 0.5);
            a_reg->setProperty("P(CK)", 0.5);
            a_reg->construct();
            portConnect(a_reg, "Q", "A", makeNetIndex(i));
            portConnect(a_reg, "CK", "CK");
            //a_reg->connect("Q", getNet("A[" + (String) i + "]"));
            //a_reg->connect("CK", getNet("CK"));
            addSubInstances(a_reg, 1.0);
            
            StdCell* b_reg = getTechModel()->getStdCellLib()->createStdCell("DFFQ", "DFFQ-B[" + (String) i + "]");
            b_reg->setProperty("P(D)", 0.5);
            b_reg->setProperty("P(CK)", 0.5);
            b_reg->construct();
            portConnect(b_reg, "Q", "B", makeNetIndex(i));
            portConnect(b_reg, "CK", "CK");
            //b_reg->connect("Q", getNet("B[" + (String) i + "]"));
            //b_reg->connect("CK", getNet("CK"));
            addSubInstances(b_reg, 1.0);

            StdCell* s_reg = getTechModel()->getStdCellLib()->createStdCell("DFFQ", "DFFQ-S[" + (String) i + "]");
            s_reg->setProperty("P(D)", 0.5);
            s_reg->setProperty("P(CK)", 0.5);
            s_reg->construct();
            portConnect(s_reg, "D", "S", makeNetIndex(i));
            portConnect(s_reg, "CK", "CK");
            //s_reg->connect("D", getNet("A[" + (String) i + "]"));
            //s_reg->connect("CK", getNet("CK"));
            addSubInstances(s_reg, 1.0);
        }
        

        //Create some adders!

        ElectricalModel* ripple_adder = new RippleAdder("Adder_1", getTechModel());
        ripple_adder->setParameter("NumberBits", num_bits);
        ripple_adder->setProperty("P(A)", 0.5);
        ripple_adder->setProperty("P(B)", 0.5);
        ripple_adder->setProperty("P(CI)", 0.5);        
        
        ripple_adder->construct();
        addSubInstances(ripple_adder, 1.0);
        portConnect(ripple_adder, "CI", "CI");
        portConnect(ripple_adder, "CO", "CO");
        portConnect(ripple_adder, "A", "A");
        portConnect(ripple_adder, "B", "B");
        portConnect(ripple_adder, "S", "S");

        ElectricalModel* multiplexer = new Multiplexer("Mux_1", getTechModel());
        multiplexer->setParameter("NumberInputs", 2);
        multiplexer->setParameter("NumberBits", mux_bits);
        multiplexer->setParameter("BitDuplicate", "FALSE");
        //multiplexer->setProperty("P(In)", "[0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]");
        //multiplexer->setProperty("P(Sel)", "[0.5, 0.5, 0.5]");
        //multiplexer->setProperty("Act(In)", "[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]");
        //multiplexer->setProperty("Act(Sel)", "[2.0, 4.0, 8.0]");
        multiplexer->setProperty("P(In)", "[0.5, 0.5]");
        multiplexer->setProperty("P(Sel)", "[0.5]");
        multiplexer->setProperty("Act(In)", "[1.0, 1.0]");
        multiplexer->setProperty("Act(Sel)", "[1.0]");
        multiplexer->construct();
        
        createNet("In0", makeNetIndex(0, mux_bits-1));
        createNet("In1", makeNetIndex(0, mux_bits-1));
        createNet("In2", makeNetIndex(0, mux_bits-1));
        createNet("In3", makeNetIndex(0, mux_bits-1));
        createNet("In4", makeNetIndex(0, mux_bits-1));
        createNet("Out", makeNetIndex(0, mux_bits-1));
        
        portConnect(multiplexer, "In0", "In0");
        portConnect(multiplexer, "In1", "In1");
        //portConnect(multiplexer, "In2", "In2");
        //portConnect(multiplexer, "In3", "In3");
        //portConnect(multiplexer, "In4", "In4");
        portConnect(multiplexer, "Out", "Out");
        
        for (unsigned int i = 0; i < mux_bits; ++i)
        {
            String n = (String) i;

            createLoad("OutLoad[" + n + "]");
            getLoad("OutLoad[" + n + "]")->setLoadCap(100e-15);

            getNet("Out", makeNetIndex(i))->addDownstreamNode(getLoad("OutLoad[" + n + "]"));
        }
        createNet("Sel", makeNetIndex(0, 2));
        assign("Sel", makeNetIndex(0), "CK");
        assign("Sel", makeNetIndex(1), "CK");
        assign("Sel", makeNetIndex(2), "CK");
        
        //portConnect(multiplexer, "Sel", "Sel");

        addSubInstances(multiplexer, 1.0);

        //ElectricalTimingAbstract* abstract = new ElectricalTimingAbstract("HAHAHA", getTechModel(), ripple_adder);
        //abstract->buildAbstract();                    
        
        return;
    }
    
    void TestModel::updateModel()
    {
        Model::updateModel();
        
        //ElectricalTimingTree* t = new ElectricalTimingTree("Add", this);
        //t->performTimingOpt(getNet("CK"), 4.21300e-8);
        //t->performTimingOpt(getNet("CK"), 1e-9);
        //delete t;

        ElectricalTimingTree* t2 = new ElectricalTimingTree("Mux", this);
        t2->performTimingOpt(getNet("In1", makeNetIndex(0)), 500e-12);
        delete t2;


    }
    
    void TestModel::evaluateModel()
    {
        Model::evaluateModel();
        
        //ripple_adder->getNddPowerResult("LeakagePower")->print("RippleAdder->Leakage", 10, cout);
        getSubInstance("Adder_1")->getNddPowerResult("Leakage")->print("RippleAdder->Leakage", 0, cout);
        //ripple_adder->getAreaResult("TotalArea")->print("RippleAdder->TotalArea", 10, cout);
        getSubInstance("Adder_1")->getAreaResult("Active")->print("RippleAdder->ActiveArea", 0, cout);
        //ripple_adder->getEventResult("AddEvent")->print("RippleAdder->AddEvent", 10, cout);
        getSubInstance("Adder_1")->getEventResult("Add")->print("RippleAdder->Add", 0, cout);

        getSubInstance("Mux_1")->getNddPowerResult("Leakage")->print("Multiplexer->Leakage", 0, cout);
        getSubInstance("Mux_1")->getAreaResult("Active")->print("Multiplexer->ActiveArea", 0, cout);
        getSubInstance("Mux_1")->getEventResult("Mux")->print("Multiplexer->MuxEvent", 0, cout);
        cout << "Multiplexer->P(Out) = " << getSubInstance("Mux_1")->getGenProperties()->get("P(Out)") << endl;

        getSubInstance("DFFQ-CI")->getNddPowerResult("Leakage")->print("DFFQ-CI->Leakage", 0, cout);
        getSubInstance("DFFQ-CI")->getAreaResult("Active")->print("DFFQ-CI->ActiveArea", 0, cout);
        getSubInstance("DFFQ-CI")->getEventResult("DFF")->print("DFFQ-CI->DFF", 0, cout);
        getSubInstance("DFFQ-CI")->getEventResult("CK")->print("DFFQ-CI->CK", 0, cout);
        
        //ripple_adder->getNddPowerResult("LeakagePower")->print("RippleAdder->Leakage", 10, cout);
        getSubInstance("Adder_1")->getNddPowerResult("Leakage")->print("RippleAdder->Leakage", 0, cout);
        //ripple_adder->getAreaResult("TotalArea")->print("RippleAdder->TotalArea", 10, cout);
        getSubInstance("Adder_1")->getAreaResult("Active")->print("RippleAdder->ActiveArea", 0, cout);
        //ripple_adder->getEventResult("AddEvent")->print("RippleAdder->AddEvent", 10, cout);
        getSubInstance("Adder_1")->getEventResult("Add")->print("RippleAdder->AddEvent", 0, cout);
    }
    
} // namespace DSENT

