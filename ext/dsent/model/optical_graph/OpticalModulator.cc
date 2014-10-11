
#include "model/optical_graph/OpticalModulator.h"
#include "model/optical_graph/OpticalTransmitter.h"

namespace DSENT
{
    OpticalModulator::OpticalModulator(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_, bool opt_loss_, OpticalTransmitter* transmitter_)
        : OpticalNode(OpticalNode::MODULATOR, instance_name_, model_, wavelengths_), m_transmitter_(transmitter_), m_insertion_loss_(0), m_extinction_ratio_(0), m_opt_loss_(opt_loss_)
    {

    }

    OpticalModulator::~OpticalModulator()
    {

    }

    bool OpticalModulator::canOptimizeLoss() const
    {
        return m_opt_loss_;
    }
    
    void OpticalModulator::setLosses(double IL_dB_, double ER_dB_)
    {
        m_insertion_loss_ = IL_dB_;
        m_extinction_ratio_ = ER_dB_;
        
        return;
    }

    bool OpticalModulator::setModulatorSpec(double IL_dB_, double ER_dB_)
    {
        // Ask the transmitter to design to those specs, returns success or fail
        return m_transmitter_->setTransmitterSpec(IL_dB_, ER_dB_);
    }
    
    double OpticalModulator::getPower(double util_) const
    {
        return m_transmitter_->getPower(util_);
    }
        
    double OpticalModulator::getInsertionLoss() const
    {
        return m_insertion_loss_;
    }

    double OpticalModulator::getExtinctionRatio() const
    {
        return m_extinction_ratio_;
    }
            
} // namespace DSENT


