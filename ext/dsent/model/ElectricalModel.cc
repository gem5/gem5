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

#include "model/ElectricalModel.h"

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalDriverMultiplier.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalLoad.h"
#include "model/timing_graph/ElectricalDelay.h"

namespace DSENT
{
    ElectricalModel::ElectricalModel(const String& instance_name_, const TechModel* tech_model_)
        : Model(instance_name_, tech_model_)
    {
        m_curr_driving_strengths_idx_ = -1;
        m_input_ports_ = new Map<PortInfo*>;
        m_output_ports_ = new Map<PortInfo*>;
        m_net_references_ = new Map<NetIndex>;
        m_drivers_ = new Map<ElectricalDriver*>;
        m_driver_multipliers_ = new Map<ElectricalDriverMultiplier*>;
        m_nets_ = new Map<ElectricalNet*>;
        m_loads_ = new Map<ElectricalLoad*>;
        m_delays_ = new Map<ElectricalDelay*>;
        m_event_infos_ = new Map<EventInfo*>;
    }

    ElectricalModel::~ElectricalModel()
    {
        deletePtrMap<PortInfo>(m_input_ports_);
        deletePtrMap<PortInfo>(m_output_ports_);
        delete m_net_references_;
        deletePtrMap<ElectricalDriver>(m_drivers_);
        deletePtrMap<ElectricalDriverMultiplier>(m_driver_multipliers_);
        deletePtrMap<ElectricalNet>(m_nets_);
        deletePtrMap<ElectricalLoad>(m_loads_);
        deletePtrMap<ElectricalDelay>(m_delays_);
        deletePtrMap<EventInfo>(m_event_infos_);
        m_input_ports_ = NULL;
        m_output_ports_ = NULL;
        m_net_references_ = NULL;
        m_drivers_ = NULL;
        m_driver_multipliers_ = NULL;
        m_nets_ = NULL;
        m_loads_ = NULL;
        m_net_references_ = NULL;
        m_event_infos_ = NULL;
    }

    void ElectricalModel::checkProperties() const
    {
        // Check if the specified driving strength exists in the available driving strengths
        if(getProperties()->keyExist("DrivingStrength"))
        {
            const double driving_strength = getProperty("DrivingStrength");
            bool is_found = false;
            for(int i = 0; i < (int)m_driving_strengths_.size(); ++i)
            {
                if(driving_strength == m_driving_strengths_[i])
                {
                    is_found = true;
                    break;
                }
            }
            ASSERT(is_found, "[Error] " + getInstanceName() + 
                " -> Driving strength (" + String(driving_strength) + ")"
                " not found in available driving strengths (" + 
                getParameter("AvailableDrivingStrengths"));
        }

        // Do normal check on the properties
        Model::checkProperties();
        return;
    }

    double ElectricalModel::getDrivingStrength() const
    {
        if(m_curr_driving_strengths_idx_ == -1)
        {
            return 0;
        }
        else
        {
            return m_driving_strengths_[m_curr_driving_strengths_idx_];
        }
    }

    int ElectricalModel::getDrivingStrengthIdx() const
    {
        return m_curr_driving_strengths_idx_;
    }

    void ElectricalModel::setDrivingStrengthIdx(int idx_)
    {
        ASSERT(((idx_ >= 0) && (idx_ < (int)m_driving_strengths_.size())), 
                "[Error] " + getInstanceName() + 
                " -> Driving strength index out of range (" + String(idx_) + ")");

        m_curr_driving_strengths_idx_ = idx_;
        setProperty("DrivingStrength", m_driving_strengths_[m_curr_driving_strengths_idx_]);
        
        Log::printLine(getInstanceName() + " -> Changing drive strength to " + (String) m_driving_strengths_[m_curr_driving_strengths_idx_]);
        update();
        return;
    }

    void ElectricalModel::setMinDrivingStrength()
    {
        setDrivingStrengthIdx(0);
        return;
    }

    bool ElectricalModel::hasMinDrivingStrength() const
    {
        return (m_curr_driving_strengths_idx_ == 0);
    }

    bool ElectricalModel::hasMaxDrivingStrength() const
    {
        return (m_curr_driving_strengths_idx_ == ((int)m_driving_strengths_.size() - 1));
    }

    void ElectricalModel::increaseDrivingStrength()
    {
        if(!hasMaxDrivingStrength())
        {
            setDrivingStrengthIdx(m_curr_driving_strengths_idx_ + 1);
        }
        return;
    }

    void ElectricalModel::decreaseDrivingStrength()
    {
        if(!hasMinDrivingStrength())
        {
            setDrivingStrengthIdx(m_curr_driving_strengths_idx_ - 1);
        }
        return;
    }

    void ElectricalModel::setAvailableDrivingStrengths(const String& driving_strengths_)
    {
        setParameter("AvailableDrivingStrengths", driving_strengths_);
        const vector<String>& split_str = driving_strengths_.split("[,");

        // Check if there is at least one driving strength specified
        ASSERT(!split_str.empty(), "[Error] " + getInstanceName() + 
            " -> Specified driving strength string does not contain any driving strengths (" +
            driving_strengths_ + ")");

        // TODO - check if the driving strengths is sorted

        // Overwrite the available driving strengths
        m_driving_strengths_.clear();
        for(int i = 0; i < (int)split_str.size(); ++i)
        {
            m_driving_strengths_.push_back(split_str[i].toDouble());
        }

        // Set the driving strength to minimum
        m_curr_driving_strengths_idx_ = 0;
        setProperty("DrivingStrength", m_driving_strengths_[m_curr_driving_strengths_idx_]);
        return;
    }
    
    // Connect a port (input or output) to some ElectricalNet
    void ElectricalModel::portConnect(ElectricalModel* connect_model_, const String& connect_port_name_, const String& connect_net_name_)
    {
        ASSERT(m_net_references_->keyExist(connect_net_name_), "[Error] " + getInstanceName() +
            " -> Net '" + connect_net_name_ + "' does not exist!");
            
        portConnect(connect_model_, connect_port_name_, connect_net_name_, m_net_references_->get(connect_net_name_));        
    }
        
    void ElectricalModel::portConnect(ElectricalModel* connect_model_, const String& connect_port_name_, const String& connect_net_name_, const NetIndex& connect_net_indices_)
    {
        ASSERT(m_net_references_->keyExist(connect_net_name_), "[Error] " + getInstanceName() +
            " -> Net '" + connect_net_name_ + "' does not exist!");
            
        // Check whether the port name is an input or output, ASSERTion error if neither
        bool is_input = connect_model_->getInputs()->keyExist(connect_port_name_);
        bool is_output = connect_model_->getOutputs()->keyExist(connect_port_name_);
        
        ASSERT(is_input || is_output, "[Error] " + getInstanceName() + " -> Model '" + connect_model_->getInstanceName() + 
            "' does not have a port named '" + connect_port_name_ + "'!");
        
        int connect_net_width = connect_net_indices_.second - connect_net_indices_.first + 1;
        const NetIndex& port_indices = connect_model_->getNetReference(connect_port_name_);
        int port_width = port_indices.second - port_indices.first + 1;
        
        ASSERT(connect_net_width == port_width, "[Error] " + getInstanceName() + " -> Port width mismatch for Model '" + 
            connect_model_->getInstanceName() + "." + connect_port_name_ + toString(port_indices) + 
            "' and net '" + connect_net_name_ + toString(connect_net_indices_) + "'!");

        int port_index = port_indices.first;
        int connect_net_index = connect_net_indices_.first;
        
        if(is_input)
        {
            while(port_index <= port_indices.second)
            {
                getNet(connect_net_name_, makeNetIndex(connect_net_index))->addDownstreamNode(
                    connect_model_->getNet(connect_port_name_, makeNetIndex(port_index)));
                ++port_index;
                ++connect_net_index;
            }
        }
        else if(is_output)
        {
            while (port_index <= port_indices.second)
            {
                connect_model_->getNet(connect_port_name_, makeNetIndex(port_index))->addDownstreamNode(
                    getNet(connect_net_name_, makeNetIndex(connect_net_index)));
                ++port_index;
                ++connect_net_index;
            }
        }
    }
    
    //Get Drivers
    const Map<ElectricalDriver*>* ElectricalModel::getDrivers() const
    {
        return m_drivers_;
    }
    
    ElectricalDriver* ElectricalModel::getDriver(const String& name_)
    {
        return m_drivers_->get(name_);
    }

    //Get Driver Multipliers
    const Map<ElectricalDriverMultiplier*>* ElectricalModel::getDriverMultipliers() const
    {
        return m_driver_multipliers_;
    }
    
    ElectricalDriverMultiplier* ElectricalModel::getDriverMultiplier(const String& name_)
    {
        return m_driver_multipliers_->get(name_);
    }

    //Get Nets
    const Map<ElectricalNet*>* ElectricalModel::getNets() const
    {
        return m_nets_;
    }
    
    ElectricalNet* ElectricalModel::getNet(const String& name_)
    {
        return getNet(name_, m_net_references_->get(name_));
    }

    ElectricalNet* ElectricalModel::getNet(const String& name_, const NetIndex& index_)
    {
        ASSERT(index_.first == index_.second, "[Error] " + getInstanceName() +
            " -> Ambiguous get net since (" + name_ + ") is a bus consisting of several nets!");
        return m_nets_->get(name_ + "[" + (String) index_.first + "]");
    }
    
    //Get Loads
    const Map<ElectricalLoad*>* ElectricalModel::getLoads() const
    {
        return m_loads_;
    }

    ElectricalLoad* ElectricalModel::getLoad(const String& name_)
    {
        return m_loads_->get(name_);
    }
    
    //Get Delays
    const Map<ElectricalDelay*>* ElectricalModel::getDelays() const
    {
        return m_delays_;
    }

    ElectricalDelay* ElectricalModel::getDelay(const String& name_)
    {
        return m_delays_->get(name_);
    }

    //Get Inputs
    const Map<PortInfo*>* ElectricalModel::getInputs() const
    {
        return m_input_ports_;
    }
    
    PortInfo* ElectricalModel::getInputPort(const String& name_)
    {
        ASSERT(m_input_ports_->keyExist(name_), "[Error] " + getInstanceName() +
                " -> Input port (" + name_ + ") does not exist");

        return m_input_ports_->get(name_);
    }

    const PortInfo* ElectricalModel::getInputPort(const String& name_) const
    {
        ASSERT(m_input_ports_->keyExist(name_), "[Error] " + getInstanceName() +
                " -> Input port (" + name_ + ") does not exist");

        return m_input_ports_->get(name_);
    }

    //Get Outputs
    const Map<PortInfo*>* ElectricalModel::getOutputs() const
    {
        return m_output_ports_;
    }

    PortInfo* ElectricalModel::getOutputPort(const String& name_)
    {
        ASSERT(m_output_ports_->keyExist(name_), "[Error] " + getInstanceName() +
                " -> Output port (" + name_ + ") does not exist");

        return m_output_ports_->get(name_);
    }
    
    const PortInfo* ElectricalModel::getOutputPort(const String& name_) const
    {
        ASSERT(m_output_ports_->keyExist(name_), "[Error] " + getInstanceName() +
                " -> Output port (" + name_ + ") does not exist");

        return m_output_ports_->get(name_);
    }
    
    const Map<NetIndex>* ElectricalModel::getNetReferences() const
    {
        return m_net_references_;
    }

    const NetIndex ElectricalModel::getNetReference(const String& name_) const
    {
        return m_net_references_->get(name_);
    }

    //-------------------------------------------------------------------------
    //  Electrical Connectivity and Timing Element Creation Functions
    //-------------------------------------------------------------------------

    // Input Port creation
    void ElectricalModel::createInputPort(const String& name_, const NetIndex& net_indices_)
    {
        // Create the new nets (including its net reference)
        // This should already check that it has not been previously declared
        createNet(name_, net_indices_);
        // Add the net name to list of input ports
        m_input_ports_->set(name_, new PortInfo(name_, net_indices_));
        return;
    }

    // Output Port creation
    void ElectricalModel::createOutputPort(const String& name_, const NetIndex& net_indices_)
    {
        // Create the new nets (including its net reference)
        // This should already check that it has not been previously declared
        createNet(name_, net_indices_);
        // Add the net name to list of output ports
        m_output_ports_->set(name_, new PortInfo(name_, net_indices_));
        return;
    }

    // Net creation
    void ElectricalModel::createNet(const String& name_)
    {
        // Creating a net with specifying an index range means that the net is just
        // a 1-bit wire indexed at [0]
        createNet(name_, makeNetIndex(0, 0));
        return;
    }

    void ElectricalModel::createNet(const String& name_, const NetIndex& net_indices_)
    {
        // Check that it hasn't been previously declared
        ASSERT( !m_nets_->keyExist(name_) && !m_net_references_->keyExist(name_),
                "[Error] " + getInstanceName() + " -> Redeclaration of net " + name_);

        int start = net_indices_.first;
        int end = net_indices_.second;
        
        for (int index = start; index <= end; ++index)
        {
            String indexed_name = name_ + "[" + (String) index + "]";
            // Create the new net
            ElectricalNet* net = new ElectricalNet(indexed_name, this);
            // Add the net to net map
            m_nets_->set(indexed_name, net);
        }
        // Add net to net references
        m_net_references_->set(name_, net_indices_);        
        return;
    }
    
    // Driver creation
    void ElectricalModel::createDriver(const String& name_, bool sizable_)
    {
        // Check that it hasn't been previously declared
        ASSERT( !m_drivers_->keyExist(name_),
                "[Error] " + getInstanceName() + " -> Redeclaration of driver " + name_);

        ElectricalDriver* driver = new ElectricalDriver(name_, this, sizable_);
        m_drivers_->set(name_, driver);
        return;
    }

    /*
    void ElectricalModel::createDriver(const String& name_, bool sizable_, int start_index_, int end_index_)
    {
        for (int index = start_index_; index <= end_index_; ++index)
        {
            createDriver(name_ + "[" + (String) index + "]", sizable_);
        }
        return;
    }
    */
    
    // Driver Multiplier creation
    void ElectricalModel::createDriverMultiplier(const String& name_)
    {
        // Check that it hasn't been previously declared
        ASSERT( !m_driver_multipliers_->keyExist(name_),
                "[Error] " + getInstanceName() + " -> Redeclaration of driver_multiplier " + name_);

        ElectricalDriverMultiplier* driver_multiplier = new ElectricalDriverMultiplier(name_, this);
        m_driver_multipliers_->set(name_, driver_multiplier);
        return;
    }

    // Load creation
    
    void ElectricalModel::createLoad(const String& name_)
    {
        // Check that it hasn't been previously declared
        ASSERT( !m_loads_->keyExist(name_),
                "[Error] " + getInstanceName() + " -> Redeclaration of load " + name_);

        ElectricalLoad* load = new ElectricalLoad(name_, this);
        m_loads_->set(name_, load);
        return;
    }
    
    /*
    void ElectricalModel::createLoad(const String& name_, int start_index_, int end_index_)
    {
        for (int index = start_index_; index <= end_index_; ++index)
        {
            createLoad(name_ + "[" + (String) index + "]");
        }
        return;
    }
    */

    // Delay creation
    void ElectricalModel::createDelay(const String& name_)
    {
        // Check that it hasn't been previously declared
        ASSERT( !m_delays_->keyExist(name_),
                "[Error] " + getInstanceName() + " -> Redeclaration of delay " + name_);

        ElectricalDelay* delay = new ElectricalDelay(name_, this);
        m_delays_->set(name_, delay);
        return;
    }

    /*
    void ElectricalModel::createDelay(const String& name_, int start_index_, int end_index_)
    {
        for (int index = start_index_; index <= end_index_; ++index)
        {
            createDelay(name_ + "[" + (String) index + "]");
        }
        return;
    }
    */
    //-------------------------------------------------------------------------
    
    // Assign a net to be downstream from another net
    // case 1: 'assign downstream_net_name_ = upstream_net_name_'
    void ElectricalModel::assign(const String& downstream_net_name_, const String& upstream_net_name_)
    {
        ASSERT(getNetReferences()->keyExist(downstream_net_name_), "[Error] " + getInstanceName() + " -> Net '" +
            downstream_net_name_ + "' does not exist!");

        ASSERT(getNetReferences()->keyExist(upstream_net_name_), "[Error] " + getInstanceName() + " -> Net '" +
            upstream_net_name_ + "' does not exist!");
        
        assign(downstream_net_name_, getNetReference(downstream_net_name_),
            upstream_net_name_, getNetReference(upstream_net_name_));
            
        return;
    }

    // case 2: 'assign downstream_net_name_[begin:end] = upstream_net_name_'
    void ElectricalModel::assign(const String& downstream_net_name_, const NetIndex& downstream_net_indices_, const String& upstream_net_name_)
    {
        ASSERT(getNetReferences()->keyExist(downstream_net_name_), "[Error] " + getInstanceName() + " -> Net '" +
            downstream_net_name_ + "' does not exist!");

        ASSERT(getNetReferences()->keyExist(upstream_net_name_), "[Error] " + getInstanceName() + " -> Net '" +
            upstream_net_name_ + "' does not exist!");
            
        assign(downstream_net_name_, downstream_net_indices_,
            upstream_net_name_, getNetReference(upstream_net_name_));

        return;
    }

    // case 3: 'assign downstream_net_name_ = upstream_net_name_[begin:end]'
    void ElectricalModel::assign(const String& downstream_net_name_, const String& upstream_net_name_, const NetIndex& upstream_net_indices_)
    {
        ASSERT(getNetReferences()->keyExist(downstream_net_name_), "[Error] " + getInstanceName() + " -> Net '" +
            downstream_net_name_ + "' does not exist!");

        ASSERT(getNetReferences()->keyExist(upstream_net_name_), "[Error] " + getInstanceName() + " -> Net '" +
            upstream_net_name_ + "' does not exist!");
            
        assign(downstream_net_name_, getNetReference(downstream_net_name_),
            upstream_net_name_, upstream_net_indices_);

        return;    
    }
    // case 4: 'assign downstream_net_name_[begin:end] = upstream_net_name_[begin:end]'
    void ElectricalModel::assign(const String& downstream_net_name_, const NetIndex& downstream_net_indices_, const String& upstream_net_name_, const NetIndex& upstream_net_indices_)
    {
        ASSERT(getNetReferences()->keyExist(downstream_net_name_), "[Error] " + getInstanceName() + " -> Net '" +
            downstream_net_name_ + "' does not exist!");

        ASSERT(getNetReferences()->keyExist(upstream_net_name_), "[Error] " + getInstanceName() + " -> Net '" +
            upstream_net_name_ + "' does not exist!");
            
        // Check that the assignment widths are the same
        int downstream_width = downstream_net_indices_.second - downstream_net_indices_.first + 1;
        int upstream_width = upstream_net_indices_.second - upstream_net_indices_.first + 1;

        ASSERT(downstream_width == upstream_width, "[Error] " + getInstanceName() + " -> Assignment width mismatch: " +
            downstream_net_name_ + " (" + (String) downstream_width + ") and " +
            upstream_net_name_ + " (" + (String) upstream_width + ")");

        // Loop through indices and connect them together
        int down_index = downstream_net_indices_.first;
        int up_index = upstream_net_indices_.first;        
        while (down_index <= downstream_net_indices_.second)
        {
            getNet(upstream_net_name_, makeNetIndex(up_index))->addDownstreamNode(
                getNet(downstream_net_name_, makeNetIndex(down_index)));
            
            ++up_index;
            ++down_index;
        }
        
        return;
    }

    // Assign a net to another net through a driver multiplier
    void ElectricalModel::assignVirtualFanout(const String& downstream_net_name_, const String& upstream_net_name_)
    {
        ASSERT(getNetReferences()->keyExist(upstream_net_name_), "[Error] " + getInstanceName() + 
                " -> Net '" + upstream_net_name_ + "' does not exist!");
        ASSERT(getNetReferences()->keyExist(downstream_net_name_), "[Error] " + getInstanceName() + 
                " -> Net '" + downstream_net_name_ + "' does not exist!");

        assignVirtualFanout(downstream_net_name_, getNetReference(downstream_net_name_), upstream_net_name_, getNetReference(upstream_net_name_));
        return;
    }

    // Assign a net to another net through a driver multiplier
    void ElectricalModel::assignVirtualFanout(const String& downstream_net_name_, const NetIndex& downstream_net_indices_, const String& upstream_net_name_, const NetIndex& upstream_net_indices_)
    {
        ASSERT(getNetReferences()->keyExist(upstream_net_name_), "[Error] " + getInstanceName() + 
                " -> Net '" + upstream_net_name_ + "' does not exist!");
        ASSERT(getNetReferences()->keyExist(downstream_net_name_), "[Error] " + getInstanceName() + 
                " -> Net '" + downstream_net_name_ + "' does not exist!");

        const String& drive_mult_name = upstream_net_name_ + "_" + (String) upstream_net_indices_.first + "_DriverMultiplier";
        bool is_drive_mult_exist = getDriverMultipliers()->keyExist(drive_mult_name);

        // Create a driver multiplier and assign it to upstream_net since it doesn't exist 
        if(!is_drive_mult_exist)
        {
            createDriverMultiplier(drive_mult_name);
            getNet(upstream_net_name_, upstream_net_indices_)->addDownstreamNode(getDriverMultiplier(drive_mult_name));
        }

        // Assign downstream_net_name_[end:begin] = driver_multiplier_name_
        ElectricalDriverMultiplier* drive_mult = getDriverMultiplier(drive_mult_name);
        int begin_index = downstream_net_indices_.first;
        int end_index = downstream_net_indices_.second;
        for(int i = begin_index; i <= end_index; ++i)
        {
            drive_mult->addDownstreamNode(getNet(downstream_net_name_, makeNetIndex(i)));
        }
        return;
    }

    void ElectricalModel::assignVirtualFanin(const String& downstream_net_name_, const String& upstream_net_name_)
    {
        ASSERT(getNetReferences()->keyExist(upstream_net_name_), "[Error] " + getInstanceName() + 
                " -> Net '" + upstream_net_name_ + "' does not exist!");
        ASSERT(getNetReferences()->keyExist(downstream_net_name_), "[Error] " + getInstanceName() + 
                " -> Net '" + downstream_net_name_ + "' does not exist!");

        assignVirtualFanin(downstream_net_name_, getNetReference(downstream_net_name_), upstream_net_name_, getNetReference(upstream_net_name_));
        return;
    }

    void ElectricalModel::assignVirtualFanin(const String& downstream_net_name_, const NetIndex& downstream_net_indices_, const String& upstream_net_name_, const NetIndex& upstream_net_indices_)
    {
        ASSERT(getNetReferences()->keyExist(upstream_net_name_), "[Error] " + getInstanceName() + 
                " -> Net '" + upstream_net_name_ + "' does not exist!");
        ASSERT(getNetReferences()->keyExist(downstream_net_name_), "[Error] " + getInstanceName() + 
                " -> Net '" + downstream_net_name_ + "' does not exist!");

        int begin_index = upstream_net_indices_.first;
        int end_index = upstream_net_indices_.second;

        for(int i = begin_index; i <= end_index; ++i)
        {
            getNet(upstream_net_name_, makeNetIndex(i))->addDownstreamNode(getNet(downstream_net_name_, downstream_net_indices_));
        }
        return;
    }

    void ElectricalModel::createElectricalResults()
    {
        // Add active area result
        addAreaResult(new Result("Active"));

        // Add wire area result
        TechModel::ConstWireLayerIterator it_begin = getTechModel()->getAvailableWireLayers()->begin();
        TechModel::ConstWireLayerIterator it_end = getTechModel()->getAvailableWireLayers()->end();
        TechModel::ConstWireLayerIterator it;
        for(it = it_begin; it != it_end; ++it)
        {
            const String& layer_name = (*it);
            addAreaResult(new Result(layer_name + "Wire"));
        }

        // Add leakage result
        addNddPowerResult(new Result("Leakage"));

        // Add idle event result
        createElectricalEventResult("Idle");
        return;
    }

    void ElectricalModel::addElectricalSubResults(const ElectricalModel* model_, double number_models_)
    {
        // Add active area sub result
        getAreaResult("Active")->addSubResult(model_->getAreaResult("Active"), model_->getInstanceName(), number_models_);

        // Add wire area sub result
        TechModel::ConstWireLayerIterator it_begin = getTechModel()->getAvailableWireLayers()->begin();
        TechModel::ConstWireLayerIterator it_end = getTechModel()->getAvailableWireLayers()->end();
        TechModel::ConstWireLayerIterator it;
        for(it = it_begin; it != it_end; ++it)
        {
            const String& layer_name = (*it);
            const String& result_name = layer_name + "Wire";
            getAreaResult(result_name)->addSubResult(model_->getAreaResult(result_name), model_->getInstanceName(), number_models_);
        }

        // Add leakage sub result
        getNddPowerResult("Leakage")->addSubResult(model_->getNddPowerResult("Leakage"), model_->getInstanceName(), number_models_);

        // Add idle event sub result
        getEventResult("Idle")->addSubResult(model_->getEventResult("Idle"), model_->getInstanceName(), number_models_);
        return;
    }

    void ElectricalModel::addElectricalWireSubResult(const String& wire_layer_, const Result* result_, const String& producer_, double number_results_)
    {
        getAreaResult(wire_layer_ + "Wire")->addSubResult(result_, producer_, number_results_);
        return;
    }

    void ElectricalModel::createElectricalAtomicResults()
    {
        // Add active area result
        addAreaResult(new AtomicResult("Active"));

        // Add wire area result
        TechModel::ConstWireLayerIterator it_begin = getTechModel()->getAvailableWireLayers()->begin();
        TechModel::ConstWireLayerIterator it_end = getTechModel()->getAvailableWireLayers()->end();
        TechModel::ConstWireLayerIterator it;
        for(it = it_begin; it != it_end; ++it)
        {
            const String& layer_name = (*it);
            addAreaResult(new AtomicResult(layer_name + "Wire"));
        }

        // Add leakage result
        addNddPowerResult(new AtomicResult("Leakage"));

        // Add idle event result
        createElectricalEventAtomicResult("Idle");
        return;
    }

    void ElectricalModel::addElecticalAtomicResultValues(const ElectricalModel* model_, double number_models_)
    {
        getAreaResult("Active")->addValue(model_->getAreaResult("Active")->calculateSum() * number_models_);

        // Add wire area sub result
        TechModel::ConstWireLayerIterator it_begin = getTechModel()->getAvailableWireLayers()->begin();
        TechModel::ConstWireLayerIterator it_end = getTechModel()->getAvailableWireLayers()->end();
        TechModel::ConstWireLayerIterator it;
        for(it = it_begin; it != it_end; ++it)
        {
            const String& layer_name = (*it);
            const String& result_name = layer_name + "Wire";
            getAreaResult(result_name)->addValue(model_->getAreaResult(result_name)->calculateSum() * number_models_);
        }

        // Add leakage sub result
        getNddPowerResult("Leakage")->addValue(model_->getNddPowerResult("Leakage")->calculateSum() * number_models_);

        // Add idle event sub result
        getEventResult("Idle")->addValue(model_->getEventResult("Idle")->calculateSum() * number_models_);
        return;
    }

    void ElectricalModel::addElecticalWireAtomicResultValue(const String& wire_layer_, double value_)
    {
        getAreaResult(wire_layer_ + "Wire")->addValue(value_);
        return;
    }

    void ElectricalModel::resetElectricalAtomicResults()
    {
        getAreaResult("Active")->setValue(0.0);

        // Reset wire area sub result
        TechModel::ConstWireLayerIterator it_begin = getTechModel()->getAvailableWireLayers()->begin();
        TechModel::ConstWireLayerIterator it_end = getTechModel()->getAvailableWireLayers()->end();
        TechModel::ConstWireLayerIterator it;
        for(it = it_begin; it != it_end; ++it)
        {
            const String& layer_name = (*it);
            const String& result_name = layer_name + "Wire";
            getAreaResult(result_name)->setValue(0.0);
        }

        // Reset leakage sub result
        getNddPowerResult("Leakage")->setValue(0.0);

        // Reset idle event sub result
        getEventResult("Idle")->setValue(0.0);

        return;
    }

    void ElectricalModel::createElectricalEventResult(const String& name_)
    {
        // Add the event result
        addEventResult(new Result(name_));
        // Add event info
        m_event_infos_->set(name_, new EventInfo(name_, getInputs()));
        return;
    }

    void ElectricalModel::createElectricalEventAtomicResult(const String& name_)
    {
        // Add the event result
        addEventResult(new AtomicResult(name_));
        // Add event info
        m_event_infos_->set(name_, new EventInfo(name_, getInputs()));
        return;
    }

    void ElectricalModel::assignPortTransitionInfo(ElectricalModel* downstream_model_, const String& downstream_port_name_, const TransitionInfo& trans_info_)
    {
        ASSERT(downstream_model_ != NULL, "[Error] " + getInstanceName() +
                " -> Downstream model does not exist");

        downstream_model_->getInputPort(downstream_port_name_)->setTransitionInfo(trans_info_);
        return;
    }

    void ElectricalModel::propagatePortTransitionInfo(const String& downstream_port_name_, const String& upstream_port_name_)
    {
        const TransitionInfo& trans_info = getInputPort(upstream_port_name_)->getTransitionInfo();
        getOutputPort(downstream_port_name_)->setTransitionInfo(trans_info);
        return;
    }

    void ElectricalModel::propagatePortTransitionInfo(ElectricalModel* downstream_model_, const String& downstream_port_name_, const String& upstream_port_name_)
    {
        ASSERT(downstream_model_ != NULL, "[Error] " + getInstanceName() +
                " -> Downstream model does not exist");

        const TransitionInfo& trans_info = getInputPort(upstream_port_name_)->getTransitionInfo();
        downstream_model_->getInputPort(downstream_port_name_)->setTransitionInfo(trans_info);
        return;
    }

    void ElectricalModel::propagatePortTransitionInfo(ElectricalModel* downstream_model_, const String& downstream_port_name_, const ElectricalModel* upstream_model_, const String& upstream_port_name_)
    {
        ASSERT(downstream_model_ != NULL, "[Error] " + getInstanceName() +
                " -> Downstream model does not exist");
        ASSERT(upstream_model_ != NULL, "[Error] " + getInstanceName() +
                " -> Upstream model does not exist");

        const TransitionInfo& trans_info = upstream_model_->getOutputPort(upstream_port_name_)->getTransitionInfo();

        downstream_model_->getInputPort(downstream_port_name_)->setTransitionInfo(trans_info);
        return;
    }

    void ElectricalModel::propagatePortTransitionInfo(const String& downstream_port_name_, const ElectricalModel* upstream_model_, const String& upstream_port_name_)
    {
        ASSERT(upstream_model_ != NULL, "[Error] " + getInstanceName() +
                " -> Upstream model does not exist");
    
        const TransitionInfo& trans_info = upstream_model_->getOutputPort(upstream_port_name_)->getTransitionInfo();
        getOutputPort(downstream_port_name_)->setTransitionInfo(trans_info);
        return;
    }

    void ElectricalModel::propagateTransitionInfo()
    {
        // by default do nothing.
    }

    void ElectricalModel::useModel(const String& event_name_)
    {
        getGenProperties()->set("UseModelEvent", event_name_);
        applyTransitionInfo(event_name_);
        useModel();
        return;
    }

    void ElectricalModel::useModel()
    {
        propagateTransitionInfo();
        return;
    }

    void ElectricalModel::applyTransitionInfo(const String& event_name_)
    {
        // Check if the event actually exists
        ASSERT(hasEventResult(event_name_), "[Error] " + getInstanceName() +
                " -> Event (" + event_name_ + ") does not exist in the result map");
        ASSERT(m_event_infos_->keyExist(event_name_), "[Error] " + getInstanceName() +
                " -> Event (" + event_name_ + ") does not exist in the event info map");

        const EventInfo* event_info = m_event_infos_->get(event_name_);

        // Set the input ports' transition information for the event
        Map<PortInfo*>::ConstIterator it_begin = m_input_ports_->begin();
        Map<PortInfo*>::ConstIterator it_end = m_input_ports_->end();
        Map<PortInfo*>::ConstIterator it;
        for(it = it_begin; it != it_end; ++it)
        {
            const String& port_name = it->first;
            PortInfo* port_info = it->second;
            const TransitionInfo& trans_info = event_info->getTransitionInfo(port_name);
            port_info->setTransitionInfo(trans_info);
        }

        return;
    }

    EventInfo* ElectricalModel::getEventInfo(const String& event_name_)
    {
        ASSERT(m_event_infos_->keyExist(event_name_), "[Error] " + getInstanceName() +
                " -> Event (" + event_name_ + ") does not exist");

        return m_event_infos_->get(event_name_);
    }

} // namespace DSENT

