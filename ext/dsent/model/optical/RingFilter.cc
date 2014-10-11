#include "model/optical/RingFilter.h"

#include "model/optical_graph/OpticalWaveguide.h"
#include "model/optical_graph/OpticalFilter.h"

namespace DSENT
{
    RingFilter::RingFilter(const String& instance_name_, const TechModel* tech_model_)
        : OpticalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    RingFilter::~RingFilter()
    {}

    void RingFilter::initParameters()
    {
        addParameterName("InStart");
        addParameterName("InEnd");
        addParameterName("DropStart");
        addParameterName("DropEnd");
        addParameterName("DropAll", "TRUE");
        return;
    }

    void RingFilter::initProperties()
    {
        return;
    }

    void RingFilter::constructModel()
    {
        //TODO: Add tuning energy/ndd-power costs?
    
        // Create Area result
        Result* area_result = new AtomicResult("Photonic");
        addAreaResult(area_result);
    
        // Get parameters
        WavelengthGroup in_wavelengths = makeWavelengthGroup(getParameter("InStart"), getParameter("InEnd"));
        WavelengthGroup drop_wavelengths = makeWavelengthGroup(getParameter("DropStart"), getParameter("DropEnd"));
        bool drop_all = getParameter("DropAll");
        
        // Create optical ports
        createOpticalInputPort(     "In", in_wavelengths);
        createOpticalOutputPort(    "Drop", drop_wavelengths);
        createOpticalOutputPort(    "Out", in_wavelengths);
        // Create the filter
        createFilter(       "RingFilter",   in_wavelengths, drop_all, drop_wavelengths);
        OpticalFilter* ring_filter = getFilter("RingFilter");
        // Connect the filter
        getWaveguide("In")->addDownstreamNode(ring_filter);
        ring_filter->addDownstreamNode(getWaveguide("Out"));
        ring_filter->setDropPort(getWaveguide("Drop"));                
    }
    
    void RingFilter::updateModel()
    {
        //TODO: Get numbers from tech model;
        double ring_area = 200e-12;
        double thru_loss = 1e-4;
        double drop_loss = 1.0;
        // Get parameters
        WavelengthGroup drop_wavelengths = makeWavelengthGroup(getParameter("DropStart"), getParameter("DropEnd"));
        int number_wavelengths = drop_wavelengths.second - drop_wavelengths.first + 1;
        // Update losses
        OpticalFilter* ring_filter = getFilter("RingFilter");
        ring_filter->setLoss(thru_loss * number_wavelengths);
        ring_filter->setDropLoss(drop_loss + thru_loss * number_wavelengths);
        // Update area
        getAreaResult("Photonic")->setValue(ring_area * (number_wavelengths));        
    }
    
} // namespace DSENT

