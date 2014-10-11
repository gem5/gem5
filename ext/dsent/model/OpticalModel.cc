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

#include "model/OpticalModel.h"

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/optical_graph/OpticalWaveguide.h"
#include "model/optical_graph/OpticalNode.h"
#include "model/optical_graph/OpticalLaser.h"
#include "model/optical_graph/OpticalModulator.h"
#include "model/optical_graph/OpticalFilter.h"
#include "model/optical_graph/OpticalDetector.h"
#include "model/optical_graph/OpticalWavelength.h"

namespace DSENT
{
    OpticalModel::OpticalModel(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        m_optical_input_ports_ = new Map<PortInfo*>;
        m_optical_output_ports_ = new Map<PortInfo*>;

        m_waveguides_ = new Map<OpticalWaveguide*>;
        m_lasers_ = new Map<OpticalLaser*>;
        m_modulators_ = new Map<OpticalModulator*>;
        m_filters_ = new Map<OpticalFilter*>;
        m_detectors_ = new Map<OpticalDetector*>;
    }

    OpticalModel::~OpticalModel()
    {
        delete m_optical_input_ports_;
        delete m_optical_output_ports_;
        deletePtrMap<OpticalWaveguide>(m_waveguides_);
        deletePtrMap<OpticalLaser>(m_lasers_);
        deletePtrMap<OpticalModulator>(m_modulators_);
        deletePtrMap<OpticalFilter>(m_filters_);
        deletePtrMap<OpticalDetector>(m_detectors_);
        m_optical_input_ports_ = NULL;
        m_optical_output_ports_ = NULL;
        m_waveguides_ = NULL;
        m_lasers_ = NULL;
        m_modulators_ = NULL;
        m_filters_ = NULL;
        m_detectors_ = NULL;
    }

    // Connect an optical port (input or output) to some OpticalWaveguide
    void OpticalModel::opticalPortConnect(OpticalModel* connect_model_, const String& port_name_, const String& waveguide_name_)
    {
        // Check that the connecting waveguide exists
        ASSERT(m_waveguides_->keyExist(waveguide_name_), "[Error] " + getInstanceName() +
            " -> Waveguide '" + waveguide_name_ + "' does not exist!");
            
        // Check whether the port name is an input or output, assertion error if neither
        bool is_input = connect_model_->getOpticalInputs()->keyExist(port_name_);
        bool is_output = connect_model_->getOpticalOutputs()->keyExist(port_name_);                        
        ASSERT(is_input || is_output, "[Error] " + getInstanceName() + " -> Model '" + connect_model_->getInstanceName() + 
            "' does not have a port named '" + port_name_ + "'!");
        
        // Get the two waveguides
        OpticalWaveguide* port_waveguide = connect_model_->getWaveguide(port_name_);
        OpticalWaveguide* connect_waveguide = getWaveguide(waveguide_name_);
        
        // Check that the two waveguides expect the same wavelengths
        ASSERT((port_waveguide->getWavelengths().first == connect_waveguide->getWavelengths().first) &&
            (port_waveguide->getWavelengths().second == connect_waveguide->getWavelengths().second),
            "[Error] " + getInstanceName() + " -> Optical port expects different wavelengths for Model '" + 
            connect_model_->getInstanceName() + "." + port_name_ + "' and waveguide '" + waveguide_name_ + "'!");
        
        if(is_input)
        {
            connect_waveguide->addDownstreamNode(port_waveguide);
        }
        else if(is_output)
        {
            port_waveguide->addDownstreamNode(connect_waveguide);
        }
    }
        
    //Get Waveguides
    const Map<OpticalWaveguide*>* OpticalModel::getWaveguides() const
    {
        return m_waveguides_;
    }
    
    OpticalWaveguide* OpticalModel::getWaveguide(const String& name_)
    {
        return m_waveguides_->get(name_);
    }

    //Get Lasers
    const Map<OpticalLaser*>* OpticalModel::getLasers() const
    {
        return m_lasers_;
    }

    OpticalLaser* OpticalModel::getLaser(const String& name_)
    {
        return m_lasers_->get(name_);
    }

    //Get Modulators
    const Map<OpticalModulator*>* OpticalModel::getModulators() const
    {
        return m_modulators_;
    }
    
    OpticalModulator* OpticalModel::getModulator(const String& name_)
    {
        return m_modulators_->get(name_);
    }

    //Get Filters
    const Map<OpticalFilter*>* OpticalModel::getFilters() const
    {
        return m_filters_;
    }
    
    OpticalFilter* OpticalModel::getFilter(const String& name_)
    {
        return m_filters_->get(name_);
    }

    //Get Detectors
    const Map<OpticalDetector*>* OpticalModel::getDetectors() const
    {
        return m_detectors_;
    }

    OpticalDetector* OpticalModel::getDetector(const String& name_)
    {
        return m_detectors_->get(name_);
    }
        
    //Get Inputs
    const Map<PortInfo*>* OpticalModel::getOpticalInputs() const
    {
        return m_optical_input_ports_;
    }
    
    PortInfo* OpticalModel::getOpticalInputPort(const String& name_)
    {
        ASSERT(m_optical_input_ports_->keyExist(name_), "[Error] " + getInstanceName() +
                " -> Input port (" + name_ + ") does not exist");

        return m_optical_input_ports_->get(name_);
    }

    const PortInfo* OpticalModel::getOpticalInputPort(const String& name_) const
    {
        ASSERT(m_optical_input_ports_->keyExist(name_), "[Error] " + getInstanceName() +
                " -> Input port (" + name_ + ") does not exist");

        return m_optical_input_ports_->get(name_);
    }
    
    //Get Outputs
    const Map<PortInfo*>* OpticalModel::getOpticalOutputs() const
    {
        return m_optical_output_ports_;
    }

    PortInfo* OpticalModel::getOpticalOutputPort(const String& name_)
    {
        ASSERT(m_optical_output_ports_->keyExist(name_), "[Error] " + getInstanceName() +
                " -> Input port (" + name_ + ") does not exist");

        return m_optical_output_ports_->get(name_);
    }

    const PortInfo* OpticalModel::getOpticalOutputPort(const String& name_) const
    {
        ASSERT(m_optical_output_ports_->keyExist(name_), "[Error] " + getInstanceName() +
                " -> Input port (" + name_ + ") does not exist");

        return m_optical_output_ports_->get(name_);
    }

    //-------------------------------------------------------------------------
    //  Optical Connectivity Creation Functions
    //-------------------------------------------------------------------------
    void OpticalModel::createOpticalInputPort(const String& name_, const WavelengthGroup& wavelength_group_)
    {
        // Create the new waveguides
        // This should already check that it has not been previously declared
        createWaveguide(name_, wavelength_group_);
        // Add the waveguide name to list of input ports
        m_optical_input_ports_->set(name_, new PortInfo(name_, wavelength_group_));
        return;
    }

    void OpticalModel::createOpticalOutputPort(const String& name_, const WavelengthGroup& wavelength_group_)
    {
        // Create the new waveguides (including its waveguide reference)
        // This should already check that it has not been previously declared
        createWaveguide(name_, wavelength_group_);
        // Add the waveguide name to list of output ports
        m_optical_output_ports_->set(name_, new PortInfo(name_, wavelength_group_));
        return;
    }

    // Waveguide creation
    void OpticalModel::createWaveguide(const String& name_, const WavelengthGroup& wavelengths_)
    {
        // Check that the waveguide hasn't been previously declared
        ASSERT( !m_waveguides_->keyExist(name_), "[Error] " + getInstanceName() +
            " -> Redeclaration of waveguide " + name_);
        m_waveguides_->set(name_, new OpticalWaveguide(name_, this, wavelengths_));
        return;
    }

    // Laser creation
    void OpticalModel::createLaser(const String& name_, const WavelengthGroup& wavelengths_)
    {
        // Check that the laser hasn't been previously declared
        ASSERT( !m_lasers_->keyExist(name_), "[Error] " + getInstanceName() +
            " -> Redeclaration of laser " + name_);
        m_lasers_->set(name_, new OpticalLaser(name_, this, wavelengths_));
        return;
    }

    // Modulator creation
    void OpticalModel::createModulator(const String& name_, const WavelengthGroup& wavelengths_, bool opt_loss_, OpticalTransmitter* transmitter_)
    {
        // Check that the modulator hasn't been previously declared
        ASSERT( !m_modulators_->keyExist(name_), "[Error] " + getInstanceName() +
            " -> Redeclaration of modulator " + name_);
        m_modulators_->set(name_, new OpticalModulator(name_, this, wavelengths_, opt_loss_, transmitter_));
        return;
    }

    // Modulator Multiplier creation
    void OpticalModel::createFilter(const String& name_, const WavelengthGroup& wavelengths_, bool drop_all_, const WavelengthGroup& drop_wavelengths_)
    {
        // Check that the filter hasn't been previously declared
        ASSERT( !m_filters_->keyExist(name_), "[Error] " + getInstanceName() +
            " -> Redeclaration of filter " + name_);
        m_filters_->set(name_, new OpticalFilter(name_, this, wavelengths_, drop_all_, drop_wavelengths_));
        return;
    }

    // Detector creation    
    void OpticalModel::createDetector(const String& name_, const WavelengthGroup& wavelengths_, OpticalReceiver* receiver_)
    {
        // Check that the detector hasn't been previously declared
        ASSERT( !m_detectors_->keyExist(name_), "[Error] " + getInstanceName() +
            " -> Redeclaration of detector " + name_);
        m_detectors_->set(name_, new OpticalDetector(name_, this, wavelengths_, receiver_));
        return;
    }
    
    //-------------------------------------------------------------------------    

    // Assign a waveguide to be downstream from another waveguide
    // assign downtream_waveguide_name_ = upstream_waveguide_name_
    void OpticalModel::opticalAssign(const String& downstream_waveguide_name_, const String& upstream_waveguide_name_)
    {
        ASSERT(getWaveguides()->keyExist(downstream_waveguide_name_), "[Error] " + getInstanceName() + " -> Waveguide '" +
            downstream_waveguide_name_ + "' does not exist!");

        ASSERT(getWaveguides()->keyExist(upstream_waveguide_name_), "[Error] " + getInstanceName() + " -> Waveguide '" +
            upstream_waveguide_name_ + "' does not exist!");
        
        // Get the two waveguides
        OpticalWaveguide* upstream_waveguide = getWaveguide(upstream_waveguide_name_);
        OpticalWaveguide* downstream_waveguide = getWaveguide(downstream_waveguide_name_);        
        
        // Check that the two waveguides expect the same wavelengths
        ASSERT((upstream_waveguide->getWavelengths().first == downstream_waveguide->getWavelengths().first) &&
            (upstream_waveguide->getWavelengths().second == downstream_waveguide->getWavelengths().second),
            "[Error] " + getInstanceName() + " -> Assignment expects different wavelengths for waveguide '" +
                upstream_waveguide_name_ + "' and waveguide '" + downstream_waveguide_name_ + "'!");

        // Connect the downstream waveguide and the upstream waveguide
        upstream_waveguide->addDownstreamNode(downstream_waveguide);
        return;
    }
} // namespace DSENT
