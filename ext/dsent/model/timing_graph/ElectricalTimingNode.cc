
#include "model/timing_graph/ElectricalTimingNode.h"
#include "model/timing_graph/ElectricalLoad.h"

namespace DSENT
{
    // Set the optical node initial visited num
    const int ElectricalTimingNode::TIMING_NODE_INIT_VISITED_NUM = 0;

    ElectricalTimingNode::ElectricalTimingNode(const String& instance_name_, ElectricalModel* model_)
        : m_instance_name_(instance_name_), m_model_(model_), m_false_path_(false), m_crit_path_(-1),
        m_visited_num_(ElectricalTimingNode::TIMING_NODE_INIT_VISITED_NUM), m_delay_left_(0.0)
    {
        m_upstream_nodes_ = new vector<ElectricalTimingNode*>();
        m_downstream_nodes_ = new vector<ElectricalTimingNode*>();
    }

    ElectricalTimingNode::~ElectricalTimingNode()
    {
        delete m_upstream_nodes_;
        delete m_downstream_nodes_;
    }

    double ElectricalTimingNode::getMaxUpstreamRes() const
    {
        double max_res = 0.0;

        for(unsigned int i = 0; i < m_upstream_nodes_->size(); ++i)
        {
            double res = m_upstream_nodes_->at(i)->getMaxUpstreamRes();
            if(max_res < res)
            {
                max_res = res;
            }
        }
        return max_res;
    }
    
    double ElectricalTimingNode::getTotalDownstreamCap() const
    {
        double cap_sum = 0;
        
        for(unsigned int i = 0; i < m_downstream_nodes_->size(); ++i)
        {
            cap_sum += m_downstream_nodes_->at(i)->getTotalDownstreamCap();
        }
        
        return cap_sum;    
    }

    vector<ElectricalTimingNode*>* ElectricalTimingNode::getUpstreamNodes() const
    {
        return m_upstream_nodes_;
    }
    
    vector<ElectricalTimingNode*>* ElectricalTimingNode::getDownstreamNodes() const
    {
        return m_downstream_nodes_;
    }

    const String& ElectricalTimingNode::getInstanceName() const
    {
        return m_instance_name_;
    }
    
    ElectricalModel* ElectricalTimingNode::getModel()
    {
        return m_model_;
    }
    
    bool ElectricalTimingNode::isDriver() const
    {
        return false;
    }

    bool ElectricalTimingNode::isNet() const
    {
        return false;
    }

    bool ElectricalTimingNode::isLoad() const
    {
        return false;
    }
    

    const ElectricalModel* ElectricalTimingNode::getModel() const
    {
        return (const ElectricalModel*) m_model_;
    }

    void ElectricalTimingNode::addDownstreamNode(ElectricalTimingNode* node_)
    {
        m_downstream_nodes_->push_back(node_);
        node_->m_upstream_nodes_->push_back(this);
        return;
    }
    
    void ElectricalTimingNode::setFalsePath(bool false_path_)
    {
        m_false_path_ = false_path_;
        return;
    }
    
    bool ElectricalTimingNode::getFalsePath() const
    {
        return m_false_path_;
    }


    //-------------------------------------------------------------------------
    // Functions for delay optimization
    //-------------------------------------------------------------------------
    // By default, electrical timing nodes cannot be sized up/down
    bool ElectricalTimingNode::hasMaxDrivingStrength() const
    {
        return true;
    }

    bool ElectricalTimingNode::hasMinDrivingStrength() const
    {
        return true;
    }
    
    void ElectricalTimingNode::increaseDrivingStrength()
    {
        return;
    }

    void ElectricalTimingNode::decreaseDrivingStrength()
    {
        return;
    }
    //-------------------------------------------------------------------------
    
    //-------------------------------------------------------------------------
    // Node variables for critical path delay calculations
    //-------------------------------------------------------------------------
    void ElectricalTimingNode::setCritPath(int crit_path_)
    {
        m_crit_path_ = crit_path_;
        return;
    }
    
    int ElectricalTimingNode::getCritPath() const
    {
        return m_crit_path_;
    }

    void ElectricalTimingNode::setVisitedNum(int visited_num_)
    {
        m_visited_num_ = visited_num_;
        return;
    }
    
    int ElectricalTimingNode::getVisitedNum() const
    {
        return m_visited_num_;
    }
    
    void ElectricalTimingNode::setDelayLeft(double delay_left_)
    {
        m_delay_left_ = delay_left_;
    }
    
    double ElectricalTimingNode::getDelayLeft() const
    {
        return m_delay_left_;
    }    
    //-------------------------------------------------------------------------
        
} // namespace DSENT


