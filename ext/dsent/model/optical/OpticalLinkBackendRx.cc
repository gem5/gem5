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

#include "model/optical/OpticalLinkBackendRx.h"

#include "util/Constants.h"
#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/electrical/DemuxTreeDeserializer.h"
#include "model/electrical/BarrelShifter.h"
#include "model/electrical/Multiplexer.h"
#include <cmath>

namespace DSENT
{
    // TODO: Kind of don't like the way thermal tuning is written here. Maybe will switch
    // to curve fitting the CICC paper, which uses results from a monte-carlo sim. Also, there is
    // redundant code between this one and the tx one...

    OpticalLinkBackendRx::OpticalLinkBackendRx(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    OpticalLinkBackendRx::~OpticalLinkBackendRx()
    {}

    void OpticalLinkBackendRx::initParameters()
    {
        addParameterName("OutBits");
        addParameterName("CoreDataRate");
        addParameterName("LinkDataRate");
        addParameterName("RingTuningMethod");
        addParameterName("BitDuplicate");
        return;
    }

    void OpticalLinkBackendRx::initProperties()
    {
        return;
    }

    void OpticalLinkBackendRx::constructModel()
    {
        unsigned int out_bits = getParameter("OutBits");
        double core_data_rate = getParameter("CoreDataRate");
        double link_data_rate = getParameter("LinkDataRate");
        const String& tuning_method = getParameter("RingTuningMethod");
        bool bit_duplicate = getParameter("BitDuplicate");

        // Calculate deserialization ratio
        unsigned int deserialization_ratio = (unsigned int) floor(link_data_rate / core_data_rate);    
        ASSERT(deserialization_ratio == link_data_rate / core_data_rate,
            "[Error] " + getInstanceName() + " -> Cannot have non-integer deserialization ratios!");
        ASSERT((deserialization_ratio & (deserialization_ratio - 1)) == 0,
            "[Error] " + getInstanceName() + " -> Deserialization ratio must be a power of 2");
        
        // Calculate output width
        unsigned int in_bits = out_bits / deserialization_ratio;
        ASSERT(out_bits >= deserialization_ratio, "[Error] " + getInstanceName() + 
            " -> Output width must be >= deserialization ratio!");
        ASSERT(floor((double) out_bits / deserialization_ratio) == in_bits,
            "[Error] " + getInstanceName() + " -> Output width must be a multiple of the serialization ratio!");            

        getGenProperties()->set("DeserializationRatio", deserialization_ratio);
        getGenProperties()->set("InBits", in_bits);
        
        // Create ports
        createInputPort("In", makeNetIndex(0, in_bits-1));
        createInputPort("LinkCK");
        createOutputPort("Out", makeNetIndex(0, out_bits-1));        
        
        //Create energy, power, and area results
        createElectricalResults();
        // Create ring heating power cost
        addNddPowerResult(new AtomicResult("RingTuning"));
        // Create process bits event
        createElectricalEventResult("ProcessBits");
        getEventInfo("ProcessBits")->setTransitionInfo("LinkCK", TransitionInfo(0.0, (double) deserialization_ratio / 2.0, 0.0));
        // Set conditions during idle state
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("LinkCK", TransitionInfo(0.0, (double) deserialization_ratio / 2.0, 0.0));

        // Create deserializer
        const String& deserializer_name = "Deserializer";
        DemuxTreeDeserializer* deserializer = new DemuxTreeDeserializer(deserializer_name, getTechModel());
        deserializer->setParameter("OutBits", out_bits);
        deserializer->setParameter("InDataRate", link_data_rate);
        deserializer->setParameter("OutDataRate", core_data_rate);
        deserializer->setParameter("BitDuplicate", bit_duplicate);
        deserializer->construct();
        
        addSubInstances(deserializer, 1.0);
        addElectricalSubResults(deserializer, 1.0);
        getEventResult("ProcessBits")->addSubResult(deserializer->getEventResult("Deserialize"), deserializer_name, 1.0);
        
        if ((tuning_method == "ThermalWithBitReshuffle") || (tuning_method == "ElectricalAssistWithBitReshuffle"))
        {            
            // If a bit reshuffling backend is present, create the reshuffling backend
            unsigned int reorder_degree = getBitReorderDegree();

            // Create intermediate nets
            createNet("ReorderIn", makeNetIndex(0, in_bits+reorder_degree-1));
            assign("ReorderIn", makeNetIndex(0, in_bits-1), "In");
            assign("ReorderIn", makeNetIndex(in_bits, in_bits+reorder_degree-1), "ReorderIn", makeNetIndex(0, reorder_degree-1));
            createNet("DeserializerIn", makeNetIndex(0, in_bits-1));
            createNet("BarrelShiftIn", makeNetIndex(0, out_bits-1));
                        
            // Create bit reorder muxes
            const String& reorder_mux_name = "ReorderMux";
            Multiplexer* reorder_mux = new Multiplexer(reorder_mux_name, getTechModel());
            reorder_mux->setParameter("NumberBits", in_bits);
            reorder_mux->setParameter("NumberInputs", reorder_degree);
            reorder_mux->setParameter("BitDuplicate", bit_duplicate);
            reorder_mux->construct();

            // Create barrelshifter
            unsigned int shift_index_min = (unsigned int)ceil(log2(deserialization_ratio));
            unsigned int shift_index_max = std::max(shift_index_min, (unsigned int) ceil(log2(out_bits)) - 1);

            // Remember some things
            getGenProperties()->set("ReorderDegree", reorder_degree);
            getGenProperties()->set("ShiftIndexMin", shift_index_min);
            getGenProperties()->set("ShiftIndexMax", shift_index_max);
            
            const String& barrel_shift_name = "BarrelShifter";
            BarrelShifter* barrel_shift = new BarrelShifter(barrel_shift_name, getTechModel());
            barrel_shift->setParameter("NumberBits", out_bits);
            barrel_shift->setParameter("ShiftIndexMax", shift_index_max);
            barrel_shift->setParameter("ShiftIndexMin", shift_index_min);
            barrel_shift->setParameter("BitDuplicate", bit_duplicate);
            barrel_shift->construct();
            
            // Connect serializer
            portConnect(deserializer, "In", "DeserializerIn");
            portConnect(deserializer, "Out", "BarrelShiftIn");
            portConnect(deserializer, "InCK", "LinkCK");

            // Connect barrelshifter
            // TODO: Connect barrelshift shifts!
            portConnect(barrel_shift, "In", "BarrelShiftIn");
            portConnect(barrel_shift, "Out", "Out");
            
            // Connect bit reorder muxes
            // TODO: Connect re-order multiplex select signals!
            for (unsigned int i = 0; i < reorder_degree; i++)
                portConnect(reorder_mux, "In" + (String) i, "ReorderIn", makeNetIndex(i, i+in_bits-1));            
            portConnect(reorder_mux, "Out", "DeserializerIn");
            
            addSubInstances(barrel_shift, 1.0);
            addSubInstances(reorder_mux, 1.0);
            addElectricalSubResults(barrel_shift, 1.0);
            addElectricalSubResults(reorder_mux, 1.0);
            getEventResult("ProcessBits")->addSubResult(barrel_shift->getEventResult("BarrelShift"), barrel_shift_name, 1.0);
            getEventResult("ProcessBits")->addSubResult(reorder_mux->getEventResult("Mux"), reorder_mux_name, 1.0);
        }
        else if ((tuning_method == "FullThermal") || (tuning_method == "AthermalWithTrim"))
        {
            // If no bit reshuffling backend is present, then just connect deserializer up            
            portConnect(deserializer, "In", "In");
            portConnect(deserializer, "Out", "Out");
            portConnect(deserializer, "InCK", "LinkCK");
        }
        else
        {
            ASSERT(false, "[Error] " + getInstanceName() + " -> Unknown ring tuning method '" + tuning_method + "'!");
        }
        
        return;
    }
    
    void OpticalLinkBackendRx::updateModel()
    {
        // Update everyone
        Model::updateModel();
        // Update ring tuning power
        getNddPowerResult("RingTuning")->setValue(getRingTuningPower());        
        return;
    }

    void OpticalLinkBackendRx::propagateTransitionInfo()
    {
        // Get parameters
        const String& tuning_method = getParameter("RingTuningMethod");;

        // Get properties
        
        // Update the deserializer        
        if ((tuning_method == "ThermalWithBitReshuffle") || (tuning_method == "ElectricalAssistWithBitReshuffle"))
        {
            // Get generated properties
            unsigned int reorder_degree = getGenProperties()->get("ReorderDegree");
            unsigned int shift_index_min = getGenProperties()->get("ShiftIndexMin");
            unsigned int shift_index_max = getGenProperties()->get("ShiftIndexMax"); 
            
            // Reorder mux shift select bits
            unsigned int reorder_sel_bits = (unsigned int)ceil(log2(reorder_degree));

            // Create bit reorder muxes
            const String& reorder_mux_name = "ReorderMux";
            ElectricalModel* reorder_mux = (ElectricalModel*) getSubInstance(reorder_mux_name);
            for (unsigned int i = 0; i < reorder_degree; ++i)
                propagatePortTransitionInfo(reorder_mux, "In" + (String) i, "In");
            // Set select transitions to be 0, since these are statically configured
            for (unsigned int i = 0; i < reorder_sel_bits; ++i)
                reorder_mux->getInputPort("Sel" + (String) i)->setTransitionInfo(TransitionInfo(0.5, 0.0, 0.5));                
            reorder_mux->use();

            // Update the deserializer
            ElectricalModel* deserializer = (ElectricalModel*) getSubInstance("Deserializer");
            propagatePortTransitionInfo(deserializer, "In", reorder_mux, "Out");
            propagatePortTransitionInfo(deserializer, "InCK", "LinkCK");
            deserializer->use();
            
            // Update barrel shifter
            const String& barrel_shift_name = "BarrelShifter";
            ElectricalModel* barrel_shift = (ElectricalModel*) getSubInstance(barrel_shift_name);
            propagatePortTransitionInfo(barrel_shift, "In", deserializer, "Out");
            // Set shift transitions to be very low (since it is affected by slow temperature time constants)
            for (unsigned int i = shift_index_min; i <= shift_index_max; ++i)
                barrel_shift->getInputPort("Shift" + (String) i)->setTransitionInfo(TransitionInfo(0.499, 0.001, 0.499));
            barrel_shift->use();
            
            // Set output transition info
            propagatePortTransitionInfo("Out", barrel_shift, "Out");
        }
        else if ((tuning_method == "FullThermal") || (tuning_method == "AthermalWithTrim"))
        {
            // Update the deserializer
            ElectricalModel* deserializer = (ElectricalModel*) getSubInstance("Deserializer");
            propagatePortTransitionInfo(deserializer, "In", "In");
            propagatePortTransitionInfo(deserializer, "InCK", "LinkCK");
            deserializer->use();

            // Set output transition info
            propagatePortTransitionInfo("Out", deserializer, "Out");
        }
        else
        {
            ASSERT(false, "[Error] " + getInstanceName() + " -> Unknown ring tuning method '" + tuning_method + "'!");
        }

        return;
    }
    
    double OpticalLinkBackendRx::getRingTuningPower()
    {
        // Get properties
        const String& tuning_method = getParameter("RingTuningMethod");;
        unsigned int number_rings = getGenProperties()->get("InBits");        

        // Get tech model parameters
        double R = getTechModel()->get("Ring->Radius");
        double n_g = getTechModel()->get("Ring->GroupIndex");
        double heating_efficiency = getTechModel()->get("Ring->HeatingEfficiency");
        // This can actually be derived if we know thermo-optic coefficient (delta n / delta T)
        double tuning_efficiency = getTechModel()->get("Ring->TuningEfficiency");        
        double sigma_r_local = getTechModel()->get("Ring->LocalVariationSigma");
        double sigma_r_systematic = getTechModel()->get("Ring->SystematicVariationSigma");
        double T_max = getTechModel()->get("Ring->TemperatureMax");
        double T_min = getTechModel()->get("Ring->TemperatureMin");
        double T = getTechModel()->get("Temperature");
        
        // Get constants
        double c = Constants::c;
        double pi = Constants::pi;
        
        double tuning_power = 0.0;
        
        if (tuning_method == "ThermalWithBitReshuffle")
        {
            // When an electrical backend is present, rings only have to tune to the nearest channel
            // This can be approximated as each ring tuning to something exactly 1 channel away

            // Setup calculations
            double L = 2 * pi * R;                  // Optical length
            double FSR = c / (n_g * L);             // Free spectral range
            double freq_sep = FSR / number_rings;   // Channel separation
            
            // Calculate tuning power
            tuning_power = number_rings * freq_sep / (tuning_efficiency * heating_efficiency);
        }
        else if (tuning_method == "ElectricalAssistWithBitReshuffle")
        {
            // Electrical assistance allows for a fraction of the tuning range to be
            // covered electrically. This is most pronounced when the tuning range is small,
            // such is the case when bit reshuffling is applied

            // Get electrically tunable range
            double max_assist = getTechModel()->get("Ring->MaxElectricallyTunableFreq");
            
            // Setup calculations
            double L = 2 * pi * R;                  // Optical length
            double FSR = c / (n_g * L);             // Free spectral range
            double freq_sep = FSR / number_rings;   // Channel separation
            double heating_range = std::max(0.0, freq_sep - max_assist);  // The distance needed to bridge using heaters            
            
            // Calculate tuning power, which is really only the power spent on heating since
            // distance tuned electrically is pretty much free
            tuning_power = number_rings * heating_range / (tuning_efficiency * heating_efficiency);            
        }        
        else if (tuning_method == "FullThermal")
        {
            // If there is no bit reshuffling backend, each ring must tune to an
            // absolute channel frequency. Since we can only heat rings (and not cool),
            // we can only red-shift (decrease frequency). Thus, a fabrication bias
            // must be applied such that under any process and temperature corner, the
            // ring resonance remains above channel resonance
            // I'll use 3 sigmas of sigma_r_local and sigma_r_systematic, and bias against
            // the full temperature range
            double fabrication_bias_freq = 3.0 * sqrt(pow(sigma_r_local, 2) + pow(sigma_r_systematic, 2)) +
                (T_max - T_min) * tuning_efficiency;
                
            // The local/systematic variations are 0 on average. Thus, the tuning distance can be calculated as
            double tuning_distance = fabrication_bias_freq - (T - T_min) * tuning_efficiency;
            
            // Tuning power needed is just the number of rings * tuning distance / (tuning and heating efficiencies)
            tuning_power = number_rings * tuning_distance / (tuning_efficiency * heating_efficiency);
        }
        else if (tuning_method == "AthermalWithTrim")
        {
            // Athermal!
            tuning_power = 0;
        }
        else
        {
            ASSERT(false, "[Error] " + getInstanceName() + " -> Unknown ring tuning method '" + tuning_method + "'!");
        }

        return tuning_power;
    }
    
    unsigned int OpticalLinkBackendRx::getBitReorderDegree()
    {
        // Get properties
        unsigned int number_rings = getGenProperties()->get("InBits");        

        // Get tech model parameters
        double R = getTechModel()->get("Ring->Radius");
        double n_g = getTechModel()->get("Ring->GroupIndex");
        // This can actually be derived if we know thermo-optic coefficient (delta n / delta T)
        double sigma_r_local = getTechModel()->get("Ring->LocalVariationSigma");
        
        // Get constants
        double c = Constants::c;
        double pi = Constants::pi;
        
        // Calculates the degree of bit re-order multiplexing needed for bit-reshuffling backend
        // Bit reshuffling tuning is largely unaffected by sigma_r_systematic. However, sigma_r_local
        // Can potentially throw each ring to a channel several channels away. This just calculates
        // the degree of bit reorder muxing needed to realign bits in the correct order

        // Setup calculations
        double L = 2 * pi * R;                  // Optical length
        double FSR = c / (n_g * L);             // Free spectral range
        double freq_sep = FSR / number_rings;   // Channel separation        
        // Using 4 sigmas as the worst re-ordering case (must double to get both sides)
        unsigned int worst_case_channels = (unsigned int)ceil(2.0 * 4.0 * sigma_r_local / freq_sep);
        
        return worst_case_channels;
    }
    
} // namespace DSENT

