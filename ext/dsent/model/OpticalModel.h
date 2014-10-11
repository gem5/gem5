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

#ifndef __DSENT_MODEL_OPTICALMODEL_H__
#define __DSENT_MODEL_OPTICALMODEL_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class PortInfo;
    class EventInfo;
    class OpticalWaveguide;
    class OpticalLaser;
    class OpticalFilter;
    class OpticalModulator;
    class OpticalDetector;
    class OpticalReceiver;
    class OpticalTransmitter;
    
    // A Wavelength group consisting of start and end wavelength indices
    // Assuming it is the same as a net index so I can use the PortInfo class
    typedef NetIndex WavelengthGroup;

    // Helper function for making waveguide groups
    inline WavelengthGroup makeWavelengthGroup(int start_index_, int end_index_)
    {
        ASSERT(end_index_ >= start_index_, (String) "[Error] Invalid wavelength group range " +
            "[" + (String) start_index_ + ":" + (String) end_index_ + "]");
            
        return WavelengthGroup(start_index_, end_index_);
    }    
    
    // Helper function for making wavelength groups
    inline WavelengthGroup makeWavelengthGroup(int index_)
    {
        return makeWavelengthGroup(index_, index_);
    }

    // OpticalModel specifies optical connectivity to other optical models as well
    class OpticalModel : public ElectricalModel
    {        
        
        public:
            OpticalModel(const String& instance_name_, const TechModel* tech_model_);
            virtual ~OpticalModel();

        public:
            //-----------------------------------------------------------------
            // Connectivity specification
            //-----------------------------------------------------------------

        /*
        
            // Waveguide multiplier
            void setWaveguideMultiplier(unsigned int waveguide_multiplier_);
            unsigned int getWaveguideMultiplier();

        */
            // Input Ports
            void createOpticalInputPort(const String& name_, const WavelengthGroup& wavelengths_);
            const Map<PortInfo*>* getOpticalInputs() const;
            PortInfo* getOpticalInputPort(const String& name_);
            const PortInfo* getOpticalInputPort(const String& name_) const;

            // Output Ports
            void createOpticalOutputPort(const String& name_, const WavelengthGroup& wavelengths_);
            const Map<PortInfo*>* getOpticalOutputs() const;
            PortInfo* getOpticalOutputPort(const String& name_);
            const PortInfo* getOpticalOutputPort(const String& name_) const;

            // Optical Waveguides
            void createWaveguide(const String& name_, const WavelengthGroup& wavelengths_);
            const Map<OpticalWaveguide*>* getWaveguides() const;
            OpticalWaveguide* getWaveguide(const String& name_);

            // Assign a waveguide to be downstream from another waveguide
            void opticalAssign(const String& downstream_waveguide_name_, const String& upstream_waveguide_name_);
            
            // Connect a port (input or output) to some waveguide
            void opticalPortConnect(OpticalModel* connect_model_, const String& connect_port_name_, const String& connect_waveguide_name_);
            //-----------------------------------------------------------------

            //-----------------------------------------------------------------
            // Optical Graph Model Components
            //-----------------------------------------------------------------
            // Optical Laser Sources

            void createLaser(const String& name_, const WavelengthGroup& wavelengths_);
            const Map<OpticalLaser*>* getLasers() const;
            OpticalLaser* getLaser(const String& name_);
            // Optical Laser Sources
            void createFilter(const String& name_, const WavelengthGroup& wavelengths_, bool drop_all_, const WavelengthGroup& drop_wavelengths_);
            const Map<OpticalFilter*>* getFilters() const;
            OpticalFilter* getFilter(const String& name_);            
            // Optical Modulators
            void createModulator(const String& name_, const WavelengthGroup& wavelengths_, bool opt_loss_, OpticalTransmitter* transmitter_);
            const Map<OpticalModulator*>* getModulators() const;
            OpticalModulator* getModulator(const String& name_);
            // Optical Detectors
            void createDetector(const String& name_, const WavelengthGroup& wavelengths_, OpticalReceiver* receiver_);
            const Map<OpticalDetector*>* getDetectors() const;
            OpticalDetector* getDetector(const String& name_);

            //-----------------------------------------------------------------
           
        protected:
            // In an OpticalModel, the complete optical port-to-port connectivity
            // of all sub-instances must be specified. Addition/Removal optical
            // ports or port-related nets cannot happen after this step
            //virtual void constructModel() = 0;
            // In an OpticalModel, updateModel MUST finish all necessary
            // calculations such that loss and wavelength power can be calculated
            //virtual void updateModel() = 0;
            // In an OpticalModel, evaluateModel should calculate all wavelength
            // power, updating power and energy events as necessary
            //virtual void evaluateModel() = 0;
            
        private:
            // Private copy constructor. Use clone to perform copy operation.
            OpticalModel(const OpticalModel& model_);
                    
        private:
            // Map of all input ports
            Map<PortInfo*>* m_optical_input_ports_;
            // Map of all output ports
            Map<PortInfo*>* m_optical_output_ports_;
            
            // Optical graph model elements
            // Map of all waveguides
            Map<OpticalWaveguide*>* m_waveguides_;
            // Map of all laser source elements
            Map<OpticalLaser*>* m_lasers_;
            // Map of all filter elements
            Map<OpticalFilter*>* m_filters_;
            // Map of all modulator elements
            Map<OpticalModulator*>* m_modulators_;
            // Map of all photodetector elements
            Map<OpticalDetector*>* m_detectors_;

    }; // class OpticalModel
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALMODEL_H__

