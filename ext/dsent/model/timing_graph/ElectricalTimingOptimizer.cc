#include "model/timing_graph/ElectricalTimingOptimizer.h"

#include "model/PortInfo.h"
#include "model/ModelGen.h"
#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalTimingTree.h"

namespace DSENT
{
    ElectricalTimingOptimizer::ElectricalTimingOptimizer(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_), m_model_(NULL)
    {}

    ElectricalTimingOptimizer::~ElectricalTimingOptimizer()
    {}

    void ElectricalTimingOptimizer::setModel(ElectricalModel* model_)
    {
        m_model_ = model_;
        return;
    }

    ElectricalModel* ElectricalTimingOptimizer::getModel()
    {
        return m_model_;
    }

    void ElectricalTimingOptimizer::constructModel()
    {
        if(getModel() == NULL)
        {
            return;
        }

        const Map<PortInfo*>* port_info = getModel()->getInputs();
        Map<PortInfo*>::ConstIterator it_begin = port_info->begin();
        Map<PortInfo*>::ConstIterator it_end = port_info->end();
        Map<PortInfo*>::ConstIterator it;

        for(it = it_begin; it != it_end; ++it)
        {
            const String& port_name = it->first;
            const PortInfo* port_info = it->second;
            StdCell* inv0 = getTechModel()->getStdCellLib()->createStdCell("INV", port_name + "Driver0");
            inv0->construct();
            StdCell* inv1 = getTechModel()->getStdCellLib()->createStdCell("INV", port_name + "Driver1");
            inv1->construct();

            addSubInstances(inv0, 1.0);
            addSubInstances(inv1, 1.0);

            createInputPort(port_name, port_info->getNetIndex());
            createNet(port_name + "Driver0In");
            createNet(port_name + "Driver0Out");
            createNet(port_name + "Driver1Out");
            assignVirtualFanin(port_name + "Driver0In", port_name);
            portConnect(inv0, "A", port_name + "Driver0In");
            portConnect(inv0, "Y", port_name + "Driver0Out");
            portConnect(inv1, "A", port_name + "Driver0Out");
            portConnect(inv1, "Y", port_name + "Driver1Out");

            createNet(port_name + "In", port_info->getNetIndex());
            assignVirtualFanout(port_name + "In", port_name + "Driver1Out");

            portConnect(getModel(), port_name, port_name + "In");
        }

        return;
    }
}// namespace DSENT

