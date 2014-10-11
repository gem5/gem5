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

#include "model/electrical/router/RouterInputPort.h"

#include <cmath>
#include <vector>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/ModelGen.h"
#include "model/std_cells/StdCellLib.h"
#include "model/std_cells/StdCell.h"

namespace DSENT
{
    using std::ceil;
    using std::vector;
    using LibUtil::castStringVector;

    RouterInputPort::RouterInputPort(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    RouterInputPort::~RouterInputPort()
    {}

    void RouterInputPort::initParameters()
    {
        addParameterName("NumberVirtualNetworks");
        addParameterName("NumberVirtualChannelsPerVirtualNetwork");
        addParameterName("NumberBuffersPerVirtualChannel");
        addParameterName("NumberBitsPerFlit");
        addParameterName("BufferModel");
        return;
    }

    void RouterInputPort::initProperties()
    {
        return;
    }

    RouterInputPort* RouterInputPort::clone() const
    {
        // TODO 
        return NULL;
    }

    void RouterInputPort::constructModel()
    {
        // Get parameters
        unsigned int number_vns = getParameter("NumberVirtualNetworks").toUInt();
        const vector<unsigned int>& number_vcs_per_vn_vector = castStringVector<unsigned int>(getParameter("NumberVirtualChannelsPerVirtualNetwork").split("[,]"));
        const vector<unsigned int>& number_bufs_per_vc_vector = castStringVector<unsigned int>(getParameter("NumberBuffersPerVirtualChannel").split("[,]"));
        unsigned int number_bits_per_flit = getParameter("NumberBitsPerFlit").toUInt();
        const String& buffer_model = getParameter("BufferModel");

        ASSERT(number_vns > 0, "[Error] " + getInstanceName() + 
            " -> Number of virtual networks must be > 0!");
        ASSERT(number_vcs_per_vn_vector.size() == number_vns, "[Error] " + getInstanceName() + 
            " -> Expecting " + (String)number_vns + " number of vcs, got " + 
            getParameter("NumberVirtualChannelsPerVirtualNetwork"));
        for(unsigned int i = 0; i < number_vns; ++i)
        {
            ASSERT(number_vcs_per_vn_vector[i] > 0, "[Error] " + getInstanceName() + 
                " -> Number of virtual channels per virtual network must be > 0!");
        }
        ASSERT(number_bufs_per_vc_vector.size() == number_vns, "[Error] " + getInstanceName() + 
            " -> Expecting " + (String)number_vns + " number of bufs per vc, got " + 
            getParameter("NumberBuffersPerVirtualChannel"));
        for(unsigned int i = 0; i < number_vns; ++i)
        {
            ASSERT(number_bufs_per_vc_vector[i] > 0, "[Error] " + getInstanceName() + 
                " -> Number of buffers per virtual channel must be > 0!");
        }
        ASSERT(number_bits_per_flit > 0, "[Error] " + getInstanceName() + 
            " -> Number of bits per buffer must be > 0!");

        // Calculate total number of buffers needed in the RAM
        unsigned int total_number_vcs = 0;
        unsigned int total_number_bufs = 0;
        for(unsigned int i = 0; i < number_vns; ++i)
        {
            total_number_vcs += number_vcs_per_vn_vector[i];
            total_number_bufs += number_vcs_per_vn_vector[i] * number_bufs_per_vc_vector[i];
        }
        unsigned int number_addr_bits = (unsigned int)ceil(log2(total_number_bufs));

        getGenProperties()->set("TotalNumberVirtualChannels", total_number_vcs);
        getGenProperties()->set("TotalNumberBuffers", total_number_bufs);
        getGenProperties()->set("NumberAddressBits", number_addr_bits);
        getGenProperties()->set("NumberOutputs", 1);

        createInputPort("CK");
        createInputPort("FlitIn", makeNetIndex(0, number_bits_per_flit-1));
        createOutputPort("FlitOut", makeNetIndex(0, number_bits_per_flit-1));

        // Create energy, power, and area results
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));

        addEventResult(new Result("ReadBuffer"));
        addEventResult(new Result("WriteBuffer"));

        // Init RAM
        const String& ram_name = "RAM";
        ElectricalModel* ram = ModelGen::createRAM(buffer_model, ram_name, getTechModel());
        ram->setParameter("NumberEntries", total_number_bufs);
        ram->setParameter("NumberBits", number_bits_per_flit);
        ram->construct();

        // Init DFF for read address
        vector<String> rd_addr_dff_names(number_addr_bits, "");
        vector<StdCell*> rd_addr_dffs(number_addr_bits, NULL);
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            rd_addr_dff_names[i] = "RDAddr_DFF" + (String)i;
            rd_addr_dffs[i] = getTechModel()->getStdCellLib()->createStdCell("DFFQ", rd_addr_dff_names[i]);
            rd_addr_dffs[i]->construct();
        }

        // Connect RDAddr_DFFs
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            createNet("RDAddr_DFF_Out" + (String)i);

            portConnect(rd_addr_dffs[i], "CK", "CK");
            portConnect(rd_addr_dffs[i], "Q", "RDAddr_DFF_Out" + (String)i);
        }

        // Connect RAM
        portConnect(ram, "In", "FlitIn");
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            portConnect(ram, "WRAddr" + (String)i, "FlitIn", makeNetIndex(i));
            portConnect(ram, "RDAddr" + (String)i, "RDAddr_DFF_Out" + (String)i);
        }
        portConnect(ram, "WE", "FlitIn", makeNetIndex(number_bits_per_flit-1));
        portConnect(ram, "CK", "CK");
        portConnect(ram, "Out", "FlitOut");

        // Add area, power, event results 
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            addSubInstances(rd_addr_dffs[i], number_addr_bits);
            addElectricalSubResults(rd_addr_dffs[i], number_addr_bits);
        }
        addSubInstances(ram, 1.0);
        addElectricalSubResults(ram, 1.0);

        getEventResult("WriteBuffer")->addSubResult(ram->getEventResult("Write"), ram_name, 1.0);

        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            getEventResult("ReadBuffer")->addSubResult(rd_addr_dffs[i]->getEventResult("DFFD"), rd_addr_dff_names[i], number_addr_bits);
            getEventResult("ReadBuffer")->addSubResult(rd_addr_dffs[i]->getEventResult("DFFQ"), rd_addr_dff_names[i], number_addr_bits);
            getEventResult("ReadBuffer")->addSubResult(rd_addr_dffs[i]->getEventResult("CK"), rd_addr_dff_names[i], number_addr_bits);
        }
        getEventResult("ReadBuffer")->addSubResult(ram->getEventResult("Read"), ram_name, 1.0);

        return;
    }

    void RouterInputPort::propagateTransitionInfo()
    {
        // Update probability and activity
        unsigned int number_addr_bits = getGenProperties()->get("NumberAddressBits").toUInt();

        vector<ElectricalModel*> rd_addr_dffs(number_addr_bits, NULL);
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            rd_addr_dffs[i] = (ElectricalModel*)getSubInstance("RDAddr_DFF" + (String)i);
            assignPortTransitionInfo(rd_addr_dffs[i], "D", TransitionInfo());
            propagatePortTransitionInfo(rd_addr_dffs[i], "CK", "CK");
            rd_addr_dffs[i]->use();
        }

        ElectricalModel* ram = (ElectricalModel*)getSubInstance("RAM");

        // Setup default transition info
        const String& current_event = getGenProperties()->get("UseModelEvent");
        if(current_event != "Idle")
        {
            propagatePortTransitionInfo(ram, "In", "FlitIn");
            propagatePortTransitionInfo(ram, "CK", "CK");
            assignPortTransitionInfo(ram, "WE", TransitionInfo(0.0, 0.0, 1.0));
            for(unsigned int i = 0; i < number_addr_bits; ++i)
            {
                assignPortTransitionInfo(ram, "WRAddr" + (String)i, TransitionInfo(0.25, 0.25, 0.25));
                assignPortTransitionInfo(ram, "RDAddr" + (String)i, TransitionInfo(0.25, 0.25, 0.25));
            }
        }
        ram->use();
        // Set output probability
        propagatePortTransitionInfo("FlitOut", ram, "Out");
        return;
    }
} // namespace DSENT

