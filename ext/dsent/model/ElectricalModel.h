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

#ifndef __DSENT_MODEL_ELECTRICALMODEL_H__
#define __DSENT_MODEL_ELECTRICALMODEL_H__

#include "util/CommonType.h"
#include "model/Model.h"
#include "model/TransitionInfo.h"

namespace DSENT
{
    class PortInfo;
    class EventInfo;
    class ElectricalDriver;
    class ElectricalDriverMultiplier;
    class ElectricalNet;
    class ElectricalLoad;
    class ElectricalDelay;

    // A Net index consisting of a start and end index
    typedef std::pair<int, int> NetIndex;

    // Helper function to make net index
    inline NetIndex makeNetIndex(int start_index_, int end_index_)
    {
        ASSERT((end_index_ >= start_index_), (String)"[Error] Invalid net index range " +

                "[" + (String)start_index_ + ":" + (String)end_index_ + "]");

        return NetIndex(start_index_, end_index_);
    }

    // Helper function to make net index
    inline NetIndex makeNetIndex(int index_)
    {
        return makeNetIndex(index_, index_);
    }

    // Helper function to trun NetIndex to String
    inline String toString(const NetIndex& net_index_)
    {
        return "[" + String(net_index_.second) + ":" + String(net_index_.first) + "]";
    }

    // ElectricalModel specifies physical connectivity to other models as well as the port
    // parameters for the current model
    class ElectricalModel : public Model
    {
        public:
            ElectricalModel(const String& instance_name_, const TechModel* tech_model_);
            virtual ~ElectricalModel();

        public:
            // Check if all properties needed exist in the m_properties_
            virtual void checkProperties() const;
            // Set available driving strength vector from string
            void setAvailableDrivingStrengths(const String& driving_strengths_);

            //-----------------------------------------------------------------
            // Connectivity specification
            //-----------------------------------------------------------------
            // Net Indices
            const Map<NetIndex>* getNetReferences() const;
            const NetIndex getNetReference(const String& name_) const;
            // Input Ports
            void createInputPort(const String& name_, const NetIndex& net_indices_ = NetIndex(0, 0));
            const Map<PortInfo*>* getInputs() const;
            PortInfo* getInputPort(const String& name_);
            const PortInfo* getInputPort(const String& name_) const;
            // Output Ports
            void createOutputPort(const String& name_, const NetIndex& net_indices_ = NetIndex(0, 0));
            const Map<PortInfo*>* getOutputs() const;
            PortInfo* getOutputPort(const String& name_);
            const PortInfo* getOutputPort(const String& name_) const;
            // Electrical Nets
            void createNet(const String& name_);
            void createNet(const String& name_, const NetIndex& net_indices_);
            const Map<ElectricalNet*>* getNets() const;
            ElectricalNet* getNet(const String& name_);
            ElectricalNet* getNet(const String& name_, const NetIndex& index_);

            // Assign a net to be downstream from another net
            // case 1: 'assign downstream_net_name_ = upstream_net_name_'
            void assign(const String& downstream_net_name_, const String& upstream_net_name_);
            // case 2: 'assign downstream_net_name_[end:begin] = upstream_net_name_'
            void assign(const String& downstream_net_name_, const NetIndex& downstream_net_indices_, const String& upstream_net_name_);
            // case 3: 'assign downstream_net_name_ = upstream_net_name_[end:begin]'
            void assign(const String& downstream_net_name_, const String& upstream_net_name_, const NetIndex& upstream_net_indices_);
            // case 4: 'assign downstream_net_name_[end:begin] = upstream_net_name_[end:begin]'
            void assign(const String& downstream_net_name_, const NetIndex& downstream_net_indices_, const String& upstream_net_name_, const NetIndex& upstream_net_indices_);

            // Connect a port (input or output) to some ElectricalNet
            // case 1: .connect_port_name_(connect_net_name_)
            void portConnect(ElectricalModel* connect_model_, const String& connect_port_name_, const String& connect_net_name_);
            // case 2: .connect_port_name_(connect_net_name[end:begin])
            void portConnect(ElectricalModel* connect_model_, const String& connect_port_name_, const String& connect_net_name_, const NetIndex& connect_net_indices_);

            // Assign a net to be downstream from another net through a driver multipliers
            void assignVirtualFanout(const String& downstream_net_name_, const String& upstream_net_name_);
            void assignVirtualFanout(const String& downstream_net_name_, const NetIndex& downstream_net_indices_, const String& upstream_net_name_, const NetIndex& upstream_net_indices_);
            // Assign a net to be downstream from another net
            // This is used to enable bit_duplication 
            void assignVirtualFanin(const String& downstream_net_name_, const String& upstream_net_name_);
            void assignVirtualFanin(const String& downstream_net_name_, const NetIndex& downstream_net_indices_, const String& upstream_net_name_, const NetIndex& upstream_net_indices_);
            //-----------------------------------------------------------------

            //-----------------------------------------------------------------
            // Timing Model Components
            //-----------------------------------------------------------------
            // Electrical Drivers
            void createDriver(const String& name_, bool sizable_);
            //void createDriver(const String& name_, bool sizable_, int start_index_, int end_index_);
            const Map<ElectricalDriver*>* getDrivers() const;
            ElectricalDriver* getDriver(const String& name_);
            // Electrical Driver Multipliers
            void createDriverMultiplier(const String& name_);
            const Map<ElectricalDriverMultiplier*>* getDriverMultipliers() const;
            ElectricalDriverMultiplier* getDriverMultiplier(const String& name_);


            // Electrical Loads
            void createLoad(const String& name_);                        
            //void createLoad(const String& name_, int start_index_, int end_index_);                        
            const Map<ElectricalLoad*>* getLoads() const;
            ElectricalLoad* getLoad(const String& name_);
            // Electrical Delay creation
            void createDelay(const String& name_);                        
            //void createDelay(const String& name_, int start_index_, int end_index_);
            const Map<ElectricalDelay*>* getDelays() const;
            ElectricalDelay* getDelay(const String& name_);
            //-----------------------------------------------------------------

            // Get current driving strength
            double getDrivingStrength() const;
            // Get current driving strength index
            int getDrivingStrengthIdx() const;
            // Set driving strength by index
            void setDrivingStrengthIdx(int idx_);
            // Set the instance to minimum driving strength
            void setMinDrivingStrength();
            // Return true if the instance has minimum driving strength
            bool hasMinDrivingStrength() const;
            // Return true if the instance has maximum driving strength
            bool hasMaxDrivingStrength() const;
            // Increase driving strength index by 1
            void increaseDrivingStrength();
            // Decrease driving strength index by 1
            void decreaseDrivingStrength();

            // Create the default sets of the electrical results
            void createElectricalResults();
            // Add the default sets of the electrical results from a model
            void addElectricalSubResults(const ElectricalModel* model_, double number_models_);
            // Add extra wire sub results
            void addElectricalWireSubResult(const String& wire_layer_, const Result* result_, const String& producer_, double number_results_);
            // Create the default sets of the electrical atomic results
            void createElectricalAtomicResults();
            // Accumulate the electrical atomic results' values
            void addElecticalAtomicResultValues(const ElectricalModel* model_, double number_models_);
            // Add extra wire sub results
            void addElecticalWireAtomicResultValue(const String& wire_layer_, double value_);
            // Reset the electrical atomic results' values
            void resetElectricalAtomicResults();
            // Create an electrical event result. This will add an event associate to all input/output ports
            void createElectricalEventResult(const String& name_);
            // Create an electrical event atomic result
            void createElectricalEventAtomicResult(const String& name_);

            //-----------------------------------------------------------------
            // Helper functions to propagate transition information
            //-----------------------------------------------------------------
            void assignPortTransitionInfo(ElectricalModel* downstream_model_, const String& downstream_port_name_, const TransitionInfo& trans_info_);
            void propagatePortTransitionInfo(const String& downstream_port_name_, const String& upstream_port_name_);
            void propagatePortTransitionInfo(ElectricalModel* downstream_model_, const String& downstream_port_name_, const String& upstream_port_name_);
            void propagatePortTransitionInfo(ElectricalModel* downstream_model_, const String& downstream_port_name_, const ElectricalModel* upstream_model_, const String& upstream_port_name_);
            void propagatePortTransitionInfo(const String& downstream_port_name_, const ElectricalModel* upstream_model_, const String& upstream_port_name_);
            virtual void propagateTransitionInfo();
            //-----------------------------------------------------------------

            //-----------------------------------------------------------------
            // Helper functions to insert and remove buffers
            //-----------------------------------------------------------------
            
            //-----------------------------------------------------------------

            virtual void useModel(const String& event_name_);
            virtual void useModel();
            // TODO - add comments
            void applyTransitionInfo(const String& event_name_);
            // TODO - add comments
            EventInfo* getEventInfo(const String& event_name_);

        protected:
            // In an ElectricalModel, the complete port-to-port connectivity
            // of all sub-instance must be specified. Addition/Removal ports or
            // port-related nets cannot happen after this step
            //virtual void constructModel() = 0;
            // In an ElectricalModel, updateModel MUST finish all necessary
            // calculations such that a timing model can be run
            //virtual void updateModel() = 0;
            // In an ElectricalModel, evaluateModel should calculate all
            // event energies, now that the connectivity and timing has been
            // completed
            //virtual void evaluateModel() = 0;

        private:
            // Private copy constructor. Use clone to perform copy operation.
            ElectricalModel(const ElectricalModel& model_);

        private:
            // Contains the driving strengths in increasing order
            vector<double> m_driving_strengths_;
            // Driving strength index in the driving strength vector
            int m_curr_driving_strengths_idx_;

            //Connectivity elements
            // Nets can come in various bus widths. A net reference is really
            // just a helper map mapping a referenced map name to a bunch of
            // net indices. A net index returns the starting and end indices of
            // a net if the net is a multi-bit bus of some sort
            Map<NetIndex>* m_net_references_;
            // Map of the input ports
            Map<PortInfo*>* m_input_ports_;
            // Map of the output ports
            Map<PortInfo*>* m_output_ports_;
            // Map of all our electrical nets
            Map<ElectricalNet*>* m_nets_;

            //Timing model elements
            // Map of all our electrical drivers
            Map<ElectricalDriver*>* m_drivers_;
            // Map of all our driver multipliers
            Map<ElectricalDriverMultiplier*>* m_driver_multipliers_;
            // Map of all our electrical loads
            Map<ElectricalLoad*>* m_loads_;
            // Map of all our idealized delays
            Map<ElectricalDelay*>* m_delays_;            

            // Map of the event infos
            Map<EventInfo*>* m_event_infos_;

    }; // class ElectricalModel
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICALMODEL_H__

