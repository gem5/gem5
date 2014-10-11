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

#include "model/ModelGen.h"

#include <iostream>

#include "model/Model.h"
// Standard cells
#include "model/std_cells/StdCell.h"
#include "model/std_cells/INV.h"
#include "model/std_cells/NAND2.h"
#include "model/std_cells/NOR2.h"
#include "model/std_cells/MUX2.h"
#include "model/std_cells/XOR2.h"
#include "model/std_cells/DFFQ.h"
#include "model/std_cells/LATQ.h"
#include "model/std_cells/ADDF.h"
#include "model/std_cells/OR2.h"
#include "model/std_cells/AND2.h"
#include "model/std_cells/BUF.h"
// Electrical functional units
#include "model/electrical/TestModel.h"
#include "model/electrical/RippleAdder.h"
#include "model/electrical/Multiplexer.h"
#include "model/electrical/MultiplexerCrossbar.h"
#include "model/electrical/OR.h"
#include "model/electrical/Decoder.h"
#include "model/electrical/DFFRAM.h"
#include "model/electrical/MatrixArbiter.h"
#include "model/electrical/SeparableAllocator.h"
#include "model/electrical/router/Router.h"
#include "model/electrical/RepeatedLink.h"
#include "model/electrical/BroadcastHTree.h"
// Optical functional units
#include "model/optical/OpticalLinkBackendTx.h"
#include "model/optical/OpticalLinkBackendRx.h"
#include "model/optical/SWMRLink.h"
#include "model/optical/SWSRLink.h"
#include "model/optical/LaserSource.h"
#include "model/optical/ThrottledLaserSource.h"
#include "model/optical/RingModulator.h"
#include "model/optical/RingDetector.h"
// Networks
#include "model/network/ElectricalMesh.h"
#include "model/network/ElectricalClos.h"
#include "model/network/PhotonicClos.h"

namespace DSENT
{
    using std::cout;
    using std::endl;

    //TODO: Eventually automate the creation of this file

    Model* ModelGen::createModel(const String& model_name_, const String& instance_name_, const TechModel* tech_model_)
    {
        Log::printLine("ModelGen::createModel -> " + model_name_);
        
        if("INV" == model_name_)
        {
            return new INV(instance_name_, tech_model_);
        }
        else if("NAND2" == model_name_)
        {
            return new NAND2(instance_name_, tech_model_);
        }
        else if("NOR2" == model_name_)
        {
            return new NOR2(instance_name_, tech_model_);
        }
        else if("MUX2" == model_name_)
        {
            return new MUX2(instance_name_, tech_model_);
        }
        else if("XOR2" == model_name_)
        {
            return new XOR2(instance_name_, tech_model_);
        }
        else if("DFFQ" == model_name_)
        {
            return new DFFQ(instance_name_, tech_model_);
        }
        else if("LATQ" == model_name_)
        {
            return new LATQ(instance_name_, tech_model_);
        }
        else if("ADDF" == model_name_)
        {
            return new ADDF(instance_name_, tech_model_);
        }
        else if("OR2" == model_name_)
        {
            return new OR2(instance_name_, tech_model_);
        }
        else if("AND2" == model_name_)
        {
            return new AND2(instance_name_, tech_model_);
        }
        else if("BUF" == model_name_)
        {
            return new BUF(instance_name_, tech_model_);
        }
        else if("TestModel" == model_name_)
        {
            return new TestModel(instance_name_, tech_model_);
        }
        else if("RippleAdder" == model_name_)
        {
            return new RippleAdder(instance_name_, tech_model_);
        }
        else if("Multiplexer" == model_name_)
        {
            return new Multiplexer(instance_name_, tech_model_);
        }
        else if("OR" == model_name_)
        {
            return new OR(instance_name_, tech_model_);
        }
        else if("MultiplexerCrossbar" == model_name_)
        {
            return new MultiplexerCrossbar(instance_name_, tech_model_);
        }
        else if("Decoder" == model_name_)
        {
            return new Decoder(instance_name_, tech_model_);
        }
        else if("DFFRAM" == model_name_)
        {
            return new DFFRAM(instance_name_, tech_model_);
        }
        else if("MatrixArbiter" == model_name_)
        {
            return new MatrixArbiter(instance_name_, tech_model_);
        }
        else if("SeparableAllocator" == model_name_)
        {
            return new SeparableAllocator(instance_name_, tech_model_);
        }
        else if("Router" == model_name_)
        {
            return new Router(instance_name_, tech_model_);
        }
        else if("OpticalLinkBackendTx" == model_name_)
        {
            return new OpticalLinkBackendTx(instance_name_, tech_model_);
        }
        else if("OpticalLinkBackendRx" == model_name_)
        {
            return new OpticalLinkBackendRx(instance_name_, tech_model_);
        }
        else if("SWMRLink" == model_name_)
        {
            return new SWMRLink(instance_name_, tech_model_);
        }
        else if("SWSRLink" == model_name_)
        {
            return new SWSRLink(instance_name_, tech_model_);
        }
        else if("LaserSource" == model_name_)
        {
            return new LaserSource(instance_name_, tech_model_);
        }
        else if("ThrottledLaserSource" == model_name_)
        {
            return new ThrottledLaserSource(instance_name_, tech_model_);
        }
        else if("RingModulator" == model_name_)
        {
            return new RingModulator(instance_name_, tech_model_);
        }
        else if("RingDetector" == model_name_)
        {
            return new RingDetector(instance_name_, tech_model_);
        }
        else if("RepeatedLink" == model_name_)
        {
            return new RepeatedLink(instance_name_, tech_model_);
        }
        else if("BroadcastHTree" == model_name_)
        {
            return new BroadcastHTree(instance_name_, tech_model_);
        }
        else if("ElectricalMesh" == model_name_)
        {
            return new ElectricalMesh(instance_name_, tech_model_);
        }
        else if("ElectricalClos" == model_name_)
        {
            return new ElectricalClos(instance_name_, tech_model_);
        }
        else if("PhotonicClos" == model_name_)
        {
            return new PhotonicClos(instance_name_, tech_model_);
        }
        else
        {
            const String& error_msg = "[Error] Invalid model name (" + model_name_ + ")";
            throw Exception(error_msg);
            return NULL;
        }
        return NULL;
    }
    
    StdCell* ModelGen::createStdCell(const String& std_cell_name_, const String& instance_name_, const TechModel* tech_model_)
    {
        Log::printLine("ModelGen::createStdCell -> " + std_cell_name_);
        
        if("INV" == std_cell_name_)
        {
            return new INV(instance_name_, tech_model_);
        }
        else if("NAND2" == std_cell_name_)
        {
            return new NAND2(instance_name_, tech_model_);
        }
        else if("NOR2" == std_cell_name_)
        {
            return new NOR2(instance_name_, tech_model_);
        }
        else if("MUX2" == std_cell_name_)
        {
            return new MUX2(instance_name_, tech_model_);
        }
        else if("XOR2" == std_cell_name_)
        {
            return new XOR2(instance_name_, tech_model_);
        }
        else if("DFFQ" == std_cell_name_)
        {
            return new DFFQ(instance_name_, tech_model_);
        }
        else if("LATQ" == std_cell_name_)
        {
            return new LATQ(instance_name_, tech_model_);
        }
        else if("ADDF" == std_cell_name_)
        {
            return new ADDF(instance_name_, tech_model_);
        }
        else if("OR2" == std_cell_name_)
        {
            return new OR2(instance_name_, tech_model_);
        }
        else if("AND2" == std_cell_name_)
        {
            return new AND2(instance_name_, tech_model_);
        }
        else if("BUF" == std_cell_name_)
        {
            return new BUF(instance_name_, tech_model_);
        }
        else
        {
            const String& error_msg = "[Error] Invalid Standard Cell name (" + std_cell_name_ + ")";
            throw Exception(error_msg);
            return NULL;
        }
        return NULL;
    }

    ElectricalModel* ModelGen::createRAM(const String& ram_name_, const String& instance_name_, const TechModel* tech_model_)
    {
        Log::printLine("ModelGen::createRAM -> " + ram_name_);
        
        if("DFFRAM" == ram_name_)
        {
            return new DFFRAM(instance_name_, tech_model_);
        }
        else
        {
            const String& error_msg = "[Error] Invalid RAM name (" + ram_name_ + ")";
            throw Exception(error_msg);
            return NULL;
        }
        return NULL;
    }

    ElectricalModel* ModelGen::createCrossbar(const String& crossbar_name_, const String& instance_name_, const TechModel* tech_model_)
    {
        Log::printLine("ModelGen::createCrossbar -> " + crossbar_name_);
        
        if("MultiplexerCrossbar" == crossbar_name_)
        {
            return new MultiplexerCrossbar(instance_name_, tech_model_);
        }
        else
        {
            const String& error_msg = "[Error] Invalid Crossbar name (" + crossbar_name_ + ")";
            throw Exception(error_msg);
            return NULL;
        }
        return NULL;
    }
    //-----------------------------------------------------------------

    void ModelGen::printAvailableModels()
    {
        // TODO: Need more descriptions
        cout << "INV NAND2 NOR2 MUX2 XOR2 DFFQ LATQ ADDF OR2 AND2 BUF" << endl;
        cout << "RippleAdder Multiplexer OR RepeatedLink BroadcastHTree" << endl;
        cout << "MultiplexerCrossbar Decoder DFFRAM MatrixArbiter SeparableAllocator Router" << endl;
        cout << "OpticalLinkBackendTx OpticalLinkBackendRx SWMRLink SWSRLink" << endl;
        cout << "LaserSource ThrottledLaserSource RingModulator RingDetector" << endl;
        cout << "ElectricalMesh ElectricalClos PhotonicClos" << endl;
        return;
    }
} // namespace DSENT

