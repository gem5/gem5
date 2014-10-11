#include "model/optical/OpticalTestModel.h"
#include "model/optical_graph/OpticalGraph.h"
#include "model/optical_graph/OpticalWaveguide.h"
#include "model/optical/RingModulator.h"
#include "model/optical/RingFilter.h"
#include "model/optical/RingDetector.h"
#include "model/optical/LaserSource.h"

namespace DSENT
{
    OpticalTestModel::OpticalTestModel(const String& instance_name_, const TechModel* tech_model_)
        : OpticalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    OpticalTestModel::~OpticalTestModel()
    {}

    void OpticalTestModel::initParameters()
    {
        return;
    }

    void OpticalTestModel::initProperties()
    {
        return;
    }

    void OpticalTestModel::constructModel()
    {
        unsigned int wavelengths = 64;
        unsigned int number_readers = 1;
        
        createWaveguide("LaserToMod", makeWavelengthGroup(0, wavelengths-1));
                    
        // Create laser
        LaserSource* laser = new LaserSource("Laser", getTechModel());
        laser->setParameter("OutStart", 0);
        laser->setParameter("OutEnd", wavelengths-1);
        laser->construct();
        
        // Create modulator
        RingModulator* modulator = new RingModulator("Modulator", getTechModel());
        modulator->setParameter("InStart", 0);
        modulator->setParameter("InEnd", wavelengths-1);
        modulator->setParameter("ModStart", 0);
        modulator->setParameter("ModEnd", wavelengths-1);
        modulator->construct();
        
        for (unsigned int i = 0; i <= number_readers; ++i)
        {
            String n = (String) i;            
            createWaveguide("WaveguideDet-" + n, makeWavelengthGroup(0, wavelengths-1));        
        }

        // Create a SWMR Configuration
        for (unsigned int i = 0; i < number_readers; ++i)
        {
            String n = (String) i;
            
            // Create resonant ring detector
            RingDetector* detector = new RingDetector("Detector-" + n, getTechModel());
            detector->setParameter("InStart", 0);
            detector->setParameter("InEnd", wavelengths-1);
            detector->setParameter("DetStart", 0);
            detector->setParameter("DetEnd", wavelengths-1);
            detector->setParameter("DropAll", "FALSE");
            detector->setParameter("SenseAmp", "TRUE");
            detector->construct();
            
            opticalPortConnect(detector, "In", "WaveguideDet-" + n);
            opticalPortConnect(detector, "Out", "WaveguideDet-" + (String) (i + 1));
            
            addSubInstances(detector, 1.0);
        }

        opticalPortConnect(laser, "Out", "LaserToMod");
        opticalPortConnect(modulator, "In", "LaserToMod");
        opticalPortConnect(modulator, "Out", "WaveguideDet-0");
        
        addSubInstances(laser, 1.0);
        addSubInstances(modulator, 1.0);
    }
    
    void OpticalTestModel::updateModel()
    {
        double data_rate = 8e9;
        double extinction_ratio = 5;
        double insertion_loss = 3;

        Model* laser = getSubInstance("Laser");
        laser->update();
        
        getWaveguide("LaserToMod")->setLoss(10);
        
        Model* modulator = getSubInstance("Modulator");
        modulator->setProperty("ExtinctionRatio", extinction_ratio);
        modulator->setProperty("InsertionLoss", insertion_loss);
        modulator->setProperty("DataRate", data_rate);
        modulator->setProperty("P(In)", 0.5);
        modulator->setProperty("Act(In)", 1.0);
        modulator->update();        
        
        unsigned int number_readers = 1;
        for (unsigned int i = 0; i < number_readers; ++i)
        {
            Model* detector = getSubInstance("Detector-" + (String) i);
            detector->setProperty("ExtinctionRatio", extinction_ratio);
            detector->setProperty("DataRate", data_rate);
            detector->setProperty("P(In)", 0.5);
            detector->setProperty("Act(In)", 1.0);
            detector->update();
        }
        

    }
    
} // namespace DSENT

