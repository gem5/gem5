
#include "model/optical_graph/OpticalNode.h"

namespace DSENT
{
    // Set the optical node initial visited num
    const int OpticalNode::OPTICAL_NODE_INIT_VISITED_NUM = 0;

    OpticalNode::OpticalNode(Type type_, const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_)
        :m_type_(type_), m_instance_name_(instance_name_), m_model_(model_), m_wavelengths_(wavelengths_)
    {
        m_loss_ = 0.0;
        setVisitedNum(OpticalNode::OPTICAL_NODE_INIT_VISITED_NUM);
        m_downstream_nodes_ = new vector<OpticalNode*>;
    }

    OpticalNode::~OpticalNode()
    {

    }
    
    OpticalNode::Type OpticalNode::getType() const
    {
        return m_type_;
    }
    
    vector<OpticalNode*>* OpticalNode::getDownstreamNodes() const
    {
        return m_downstream_nodes_;
    }

    const String& OpticalNode::getInstanceName() const
    {
        return m_instance_name_;
    }
    
    OpticalModel* OpticalNode::getModel()
    {
        return m_model_;
    }
    
    const OpticalModel* OpticalNode::getModel() const
    {
        return (const OpticalModel*) m_model_;
    }

    void OpticalNode::addDownstreamNode(OpticalNode* node_)
    {
        ASSERT(node_->isExpected(getWavelengths()), "[Error] " + getInstanceName() +
            " -> Downstream node not expecting a superset of the current wavelengths");
        m_downstream_nodes_->push_back(node_);
    }    
    
    WavelengthGroup OpticalNode::getWavelengths() const
    {
        return m_wavelengths_;
    }

    bool OpticalNode::isExpected(const WavelengthGroup& wavelengths_) const
    {
        // Check that the lower limits are within bounds
        bool lower_match = (wavelengths_.first >= getWavelengths().first);
        // Check that the upper limits are within bounds
        bool upper_match = (wavelengths_.second <= getWavelengths().second);
        // Assert that there are no misalignments
        ASSERT(lower_match == upper_match, "[Error] " + getInstanceName() +
            " -> Wavelength group misalignment!");
        // Both upper and lower bounds must match
        return (upper_match && lower_match);
    }    

    //-------------------------------------------------------------------------
    void OpticalNode::setLoss(double loss_)
    {
        m_loss_ = loss_;
    }
    
    double OpticalNode::getLoss() const
    {
        return m_loss_;
    }

    void OpticalNode::setVisitedNum(int visited_num_)
    {
        m_visited_num_ = visited_num_;
    }
    
    int OpticalNode::getVisitedNum() const
    {
        return m_visited_num_;
    }
    //-------------------------------------------------------------------------
        
} // namespace DSENT


