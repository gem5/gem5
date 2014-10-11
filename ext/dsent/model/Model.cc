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

#include "model/Model.h"

#include <vector>

#include "util/Result.h"

namespace DSENT
{
    using std::vector;
    using LibUtil::deletePtrMap;
    using LibUtil::clonePtrMap;

    Model::SubModel::SubModel(Model* model_, double num_models_)
        : m_model_(model_), m_num_models_(num_models_)
    {}

    Model::SubModel::~SubModel()
    {
        delete m_model_;
    }

    Model* Model::SubModel::getModel()
    {
        return m_model_;
    }

    const Model* Model::SubModel::getModel() const
    {
        return m_model_;
    }

    double Model::SubModel::getNumModels() const
    {
        return m_num_models_;
    }

    Model::SubModel* Model::SubModel::clone() const
    {
        return new SubModel(*this);
    }

    Model::SubModel::SubModel(const SubModel& sub_model_)
    {
        m_model_ = sub_model_.m_model_->clone();
        m_num_models_ = sub_model_.m_num_models_;
    }

    const char Model::TYPE_SEPARATOR[] = ">>";
    const char Model::HIERARCHY_SEPARATOR[] = "->";
    const char Model::SUBFIELD_SEPARATOR[] = ":";
    const char Model::DETAIL_SEPARATOR[] = "@";

    Model::Model(const String& instance_name_, const TechModel* tech_model_)
        : m_instance_name_(instance_name_), m_tech_model_(tech_model_),
            m_constructed_(false), m_updated_(false), m_evaluated_(false)
    {
        m_property_names_ = new vector<String>;
        m_parameter_names_ = new vector<String>;
        m_parameters_ = new ParameterMap();        
        m_properties_ = new PropertyMap();
        m_generated_properties_ = new PropertyMap();
        m_sub_instances_ = new Map<SubModel*>();
        m_event_map_ = new Map<Result*>();
        m_area_map_ = new Map<Result*>();
        m_ndd_power_map_ = new Map<Result*>();
    }

    Model::~Model()
    {
        // Clear parameter names
        delete m_parameter_names_;
        // Clear property name
        delete m_property_names_;

        // Clear parameters
        delete m_parameters_;
        m_parameters_ = NULL;
        // Clear input properties
        delete m_properties_;
        m_properties_ = NULL;

        // Clear generated properties
        delete m_generated_properties_;
        m_generated_properties_ = NULL;

        // Clear sub models
        deletePtrMap<SubModel>(m_sub_instances_);
        m_sub_instances_ = NULL;

        // Clear all results
        deletePtrMap<Result>(m_event_map_);
        m_event_map_ = NULL;
        deletePtrMap<Result>(m_area_map_);
        m_area_map_ = NULL;
        deletePtrMap<Result>(m_ndd_power_map_);
        m_ndd_power_map_ = NULL;
    }

    void Model::setInstanceName(const String& instance_name_)
    {
        m_instance_name_ = instance_name_;
        return;
    }

    const String& Model::getInstanceName() const
    {
        return m_instance_name_;
    }

    void Model::setIsTopModel(bool is_top_model_)
    {
        m_is_top_model_ = is_top_model_;
        return;
    }

    bool Model::getIsTopModel() const
    {
        return m_is_top_model_;
    }

    //-------------------------------------------------------------------------
    //  Parameters and properties checks
    //-------------------------------------------------------------------------
    void Model::addParameterName(const String& parameter_name_)
    {
        ASSERT(!m_constructed_, "[Error] " + getInstanceName() +
            " -> Cannot add additional parameters names after model is constructed!");
        m_parameter_names_->push_back(parameter_name_);

        return;
    }

    void Model::addParameterName(const String& parameter_name_, const String& parameter_default_)
    {
        ASSERT(!m_constructed_, "[Error] " + getInstanceName() +
            " -> Cannot add additional parameters names after model is constructed!");
        m_parameter_names_->push_back(parameter_name_);
        setParameter(parameter_name_, parameter_default_);
        return;
    }
    
    const vector<String>* Model::getParameterNames() const
    {
        return m_parameter_names_;
    }

    void Model::addPropertyName(const String& property_name_)
    {
        ASSERT(!m_constructed_, "[Error] " + getInstanceName() +
            " -> Cannot add additional property names after model is constructed!");
        m_property_names_->push_back(property_name_);
        return;
    }

    void Model::addPropertyName(const String& property_name_, const String& property_default_)
    {
        ASSERT(!m_constructed_, "[Error] " + getInstanceName() +
            " -> Cannot add additional property names after model is constructed!");
        m_property_names_->push_back(property_name_);
        setProperty(property_name_, property_default_);
        return;
    }

    const vector<String>* Model::getPropertyNames() const
    {
        return m_property_names_;
    }

    void Model::checkParameters() const
    {
        String missing_parameters = "";
        
        for(int i = 0; i < (int)m_parameter_names_->size(); ++i)
        {
            const String& parameter_name = m_parameter_names_->at(i);
            if (!m_parameters_->keyExist(parameter_name))
                missing_parameters += "    " + parameter_name + "\n";
        }
        
        ASSERT(missing_parameters.size() == 0, "[Error] " + m_instance_name_ + 
                " -> Missing parameters:\n" + missing_parameters);
        return;
    }

    void Model::checkProperties() const
    {
        String missing_properties = "";
        
        for(int i = 0; i < (int)m_property_names_->size(); ++i)
        {
            const String& property_name = m_property_names_->at(i);
            if (!m_properties_->keyExist(property_name))
                missing_properties += "    " + property_name + "\n";
        }
        
        ASSERT(missing_properties.size() == 0, "[Error] " + m_instance_name_ + 
                " -> Missing properties:\n" + missing_properties);
        return;
    }
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    //  Parameters Manipulation
    //-------------------------------------------------------------------------
    const ParameterMap* Model::getParameters() const
    {
        return m_parameters_;
    }

    const String Model::getParameter(const String& parameter_name_) const
    {
        return m_parameters_->get(parameter_name_);
    }
    
    void Model::setParameter(const String& parameter_name_, const String& parameter_value_)
    {
        ASSERT(!m_constructed_, "[Error] " + getInstanceName() +
            " -> Cannot set parameters after model is constructed!");
        m_parameters_->set(parameter_name_, parameter_value_);
    }
    //-------------------------------------------------------------------------
    
    //-------------------------------------------------------------------------
    // Properties Manipulation
    //-------------------------------------------------------------------------
    const PropertyMap* Model::getProperties() const
    {
        return m_properties_;
    }

    const String Model::getProperty(const String& property_name_) const
    {
        return m_properties_->get(property_name_);
    }
    
    void Model::setProperty(const String& property_name_, const String& property_value_)
    {
        // If any properties changed, reset updated and evaluated flags
        m_updated_ = false;
        m_evaluated_ = false;
        m_properties_->set(property_name_, property_value_);
    }
    //-------------------------------------------------------------------------
            
    PropertyMap* Model::getGenProperties()
    {
        return m_generated_properties_;
    }

    const PropertyMap* Model::getGenProperties() const
    {
        return m_generated_properties_;
    }

    void Model::addSubInstances(Model* sub_instance_, double num_sub_instances_)
    {
        // Get instance name
        const String& sub_instance_name = sub_instance_->getInstanceName();

        // Check if the instance exists
        if(m_sub_instances_->keyExist(sub_instance_name))
        {
            const String& error_msg = "[Error] " + m_instance_name_ + 
            " -> Instance exists (" + sub_instance_name + ")";
            throw Exception(error_msg);
        }

        // Check if the num_sub_instances_ is a positive number
        ASSERT((num_sub_instances_ >= 0), "[Error] " + m_instance_name_ + 
        " -> Invalid number of instance (" + String(num_sub_instances_) + ")");

        // Add the instance
        m_sub_instances_->set(sub_instance_name, new SubModel(sub_instance_, num_sub_instances_));
        return;
    }

    Model* Model::getSubInstance(const String& sub_instance_name_)
    {
        // Throw an Exception if the instance already exists
        if(!m_sub_instances_->keyExist(sub_instance_name_))
        {
            const String& error_msg = "[Error] " + m_instance_name_ + 
            " -> Instance not exists (" + sub_instance_name_ + ")";
            throw Exception(error_msg);
        }

        return m_sub_instances_->get(sub_instance_name_)->getModel();
    }

    const Model* Model::getSubInstance(const String& sub_instance_name_) const
    {
        // Throw an Exception if the instance does not exist
        if(!m_sub_instances_->keyExist(sub_instance_name_))
        {
            const String& error_msg = "[Error] " + m_instance_name_ + 
            " -> Instance not exists (" + sub_instance_name_ + ")";
            throw Exception(error_msg);
        }

        return m_sub_instances_->get(sub_instance_name_)->getModel();
    }

    bool Model::hasSubInstance(const String& sub_instance_name_) const
    {
        return m_sub_instances_->keyExist(sub_instance_name_);
    }

    void Model::addAreaResult(Result* area_)
    {
        const String& area_name = area_->getName();

        // Throw an Exception if the area already exists
        if(m_area_map_->keyExist(area_name))
        {
            const String& error_msg = "Internal error: area (" + area_name + 
                ") exists";
            throw Exception(error_msg);
        }

        // Add the area
        m_area_map_->set(area_name, area_);
        return;
    }

    Result* Model::getAreaResult(const String& area_name_)
    {
        return m_area_map_->get(area_name_);
    }

    const Result* Model::getAreaResult(const String& area_name_) const
    {
        return m_area_map_->get(area_name_);
    }

    bool Model::hasAreaResult(const String& area_name_) const
    {
        return m_area_map_->keyExist(area_name_);
    }

    void Model::addNddPowerResult(Result* ndd_power_)
    {
        const String& ndd_power_name = ndd_power_->getName();

        // Throw an Exception if the ndd_power already exists
        if(m_ndd_power_map_->keyExist(ndd_power_name))
        {
            const String& error_msg = "Internal error: ndd_power (" + ndd_power_name + 
                ") exists";
            throw Exception(error_msg);
        }

        // Add the ndd_power
        m_ndd_power_map_->set(ndd_power_name, ndd_power_);
        return;
    }

    Result* Model::getNddPowerResult(const String& ndd_power_name_)
    {
        return m_ndd_power_map_->get(ndd_power_name_);
    }

    const Result* Model::getNddPowerResult(const String& ndd_power_name_) const
    {
        return m_ndd_power_map_->get(ndd_power_name_);
    }

    bool Model::hasNddPowerResult(const String& ndd_power_name_) const
    {
        return m_ndd_power_map_->keyExist(ndd_power_name_);
    }

    void Model::addEventResult(Result* event_)
    {
        const String& event_name = event_->getName();

        // Throw an Exception if the event already exists
        if(m_event_map_->keyExist(event_name))
        {
            const String& error_msg = "Internal error: event (" + event_name + 
                ") exists";
            throw Exception(error_msg);
        }

        // Add the event
        m_event_map_->set(event_name, event_);
        return;
    }

    Result* Model::getEventResult(const String& event_name_)
    {
        return m_event_map_->get(event_name_);
    }

    const Result* Model::getEventResult(const String& event_name_) const
    {
        return m_event_map_->get(event_name_);
    }

    bool Model::hasEventResult(const String& event_name_) const
    {
        return m_event_map_->keyExist(event_name_);
    }

    const TechModel* Model::getTechModel() const
    {
        return m_tech_model_;
    }

    const void* Model::parseQuery(const String& query_type_, const String& query_hier_, const String& query_sub_field_)
    {
        // Break query by hierarchy separator
        vector<String> hier_split = query_hier_.splitByString(HIERARCHY_SEPARATOR);

        // Check if the query_hier matches the instance name
        ASSERT((hier_split[0] == m_instance_name_), "[Error] " + 
                m_instance_name_ + " -> Mismatch in instance name (" + 
                hier_split[0] + ")");

        // If there is no more hierarchy separator, this process the query
        if(hier_split.size() == 1)
        {
            // Query the model
            return processQuery(query_type_, query_sub_field_);
        }
        else
        {
            // Reconstruct the query
            String temp_query_hier = hier_split[1];
            for(int i = 2; i < (int)hier_split.size(); ++i)
            {
                temp_query_hier += HIERARCHY_SEPARATOR + hier_split[i];
            }

            // Get sub instance's name
            const String& temp_sub_instance_name = hier_split[1];
            ASSERT(m_sub_instances_->keyExist(temp_sub_instance_name), "[Error] " + 
                    m_instance_name_ + " -> No sub-instances queried (" + 
                    temp_sub_instance_name + ")");

            return m_sub_instances_->get(temp_sub_instance_name)->getModel()->parseQuery(query_type_, temp_query_hier, query_sub_field_);
        }
    }

    const void* Model::processQuery(const String& query_type_, const String& query_sub_field_)
    {
        if(query_type_ == "Property")
        {
            return getProperties();
        }
        else if(query_type_ == "Parameter")
        {
            return getParameters();
        }
        else if(query_type_.contain("Hier"))
        {
            return this;
        }
        else if(query_type_ == "Area")
        {
            return queryArea(query_sub_field_);
        }
        else if(query_type_ == "NddPower")
        {
            return queryNddPower(query_sub_field_);
        }
        else if(query_type_ == "Energy")
        {
            return queryEventEnergyCost(query_sub_field_);
        }
        else 
        {
            const String& error_msg = "[Error] " + m_instance_name_ + " -> Unknown query type (" + query_type_ + ")";
            throw Exception(error_msg);
            return NULL;
        }
    }

    const Result* Model::queryArea(const String& area_name_) const
    {
        ASSERT(m_area_map_->keyExist(area_name_), "[Error] " + m_instance_name_ + 
                " -> Unknown queried area name (" + area_name_ + ")");
        return m_area_map_->get(area_name_);
    }

    const Result* Model::queryNddPower(const String& ndd_power_name_)
    {
        ASSERT(m_ndd_power_map_->keyExist(ndd_power_name_), "[Error] " + m_instance_name_ + 
                " -> Unknown queried ndd power name (" + ndd_power_name_ + ")");

        use("Idle");
        return m_ndd_power_map_->get(ndd_power_name_);
    }

    const Result* Model::queryEventEnergyCost(const String& event_name_)
    {
        ASSERT(m_event_map_->keyExist(event_name_), "[Error] " + m_instance_name_ + 
                " -> Unknown queried event name (" + event_name_ + ")");

        use(event_name_);
        return m_event_map_->get(event_name_);
    }

    // Update checks whether the model needs updating, whether all properties have been specified,
    // and calls updateModel if update is necessary
    void Model::construct()
    {
        // Model should not be constructed yet
        ASSERT(!m_constructed_, "[Error] " + getInstanceName() + " -> Cannot construct an already contructed model!");
        // Check if whether all needed parameters are defined
        checkParameters();
        constructModel();
        m_constructed_ = true;
        m_updated_ = false;
        m_evaluated_ = false;        
        return;
    }
    
    // Update checks whether the model needs updating, whether all properties have been specified,
    // and calls updateModel if update is necessary
    void Model::update()
    {
        // Model should be constructed
        ASSERT(m_constructed_, "[Error] " + getInstanceName() + " -> Cannot update an unconstructed model!");
        // If the model needs updating (due to property change)
        // an update is necessary
        if (!m_updated_)
        {
            // Check if all properties needed exist
            checkProperties();
            updateModel();
            m_updated_ = true;
            m_evaluated_ = false;
        }        
        return;
    }

    // Evaluate checks whether the model needs to be evaluated. 
    void Model::evaluate()
    {
        // Model should be constructed
        ASSERT(m_constructed_, "[Error] " + getInstanceName() + " -> Cannot evaluate an unconstructed model!");
        // Model should be updated
        ASSERT(m_updated_, "[Error] " + getInstanceName() + " -> Cannot evaluate without first updating!");
        // If the model needs evaluating
        if (!m_evaluated_)
        {
            evaluateModel();
            m_evaluated_ = true;
        }

        return;
    }

    void Model::use(const String& event_name_)
    {
        useModel(event_name_);
        return;
    }
    
    void Model::use()
    {
        useModel();
        return;
    }

    // By default, update model will iterate through all sub-instances and do updateModel on them
    void Model::updateModel()
    {
        Map<SubModel*>::Iterator iter = m_sub_instances_->begin();
        Map<SubModel*>::Iterator end = m_sub_instances_->end();
        while (iter != end)
        {
            iter->second->getModel()->update();
            iter++;
        }        
        return;
    }

    // By default, update model will iterate through all sub-instances and do updateModel on them
    void Model::evaluateModel()
    {
        Map<SubModel*>::Iterator iter = m_sub_instances_->begin();
        Map<SubModel*>::Iterator end = m_sub_instances_->end();
        while (iter != end)
        {
            iter->second->getModel()->evaluate();
            iter++;
        }        
        return;
    }

    void Model::useModel(const String& /* event_name_ */)
    {}

    void Model::useModel()
    {}

    void Model::printHierarchy(const String& query_type_, const String& query_sub_field_, const String& prepend_str_, int detail_level_, ostream& ost_) const
    {
        if(query_type_ == "InstHier")
        {
            ost_ << prepend_str_ << getInstanceName() << endl;
            printInstHierarchy(prepend_str_, detail_level_, ost_);
            //if(detail_level_ > 0)
            //{
                //for(Map<SubModel*>::ConstIterator it = m_sub_instances_->begin(); it != m_sub_instances_->end(); ++it)
                //{
                    //const Model* sub_model = (it->second)->getModel();
                    //String temp_prepend_str = prepend_str_ + "    ";
                    //sub_model->printHierarchy(query_type_, query_sub_field_, temp_prepend_str, detail_level_ - 1, ost_);
                //}
            //}
        }
        else
        {
            const Map<Result*>* result_map;

            if(query_type_ == "AreaHier")
            {
                result_map = m_area_map_;
            }
            else if(query_type_ == "NddPowerHier")
            {
                result_map = m_ndd_power_map_;
            }
            else if(query_type_ == "EventHier")
            {
                result_map = m_event_map_;
            }
            else
            {
                const String& error_msg = "[Error] " + m_instance_name_ + " -> Unknown query type (" + query_type_ + ")";
                throw Exception(error_msg);
                return;
            }

            if(query_sub_field_ == "")
            {
                for(Map<Result*>::ConstIterator it = result_map->begin(); it != result_map->end(); ++it)
                {
                    const Result* result = it->second;
                    ost_ << prepend_str_ << getInstanceName() << "->" << result->getName() << endl;
                    result->printHierarchy(prepend_str_, detail_level_, ost_);
                }
            }
            else
            {
                const Result* result = result_map->get(query_sub_field_);
                ost_ << prepend_str_ << getInstanceName() << "->" << result->getName() << endl;
                result->printHierarchy(prepend_str_, detail_level_, ost_);
            }
        }
        return;
    }

    void Model::printInstHierarchy(const String& prepend_str_, int detail_level_, ostream& ost_) const
    {
        if(detail_level_ > 0)
        {
            for(Map<SubModel*>::ConstIterator it = m_sub_instances_->begin(); it != m_sub_instances_->end(); ++it)
            {
                const Model* sub_model = it->second->getModel();
                String temp_prepend_str = prepend_str_ + "    ";

                ost_ << prepend_str_ << " |--" << sub_model->getInstanceName() << endl;
                sub_model->printInstHierarchy(temp_prepend_str, detail_level_ - 1, ost_);
            }
        }
        return;
    }

    Model* Model::clone() const
    {
        throw Exception(getInstanceName() + " -> Cannot be cloned!");
    }
    
    Model::Model(const Model& model_)
    {
        // Copy instance's name
        m_instance_name_ = model_.m_instance_name_;

        // Clone properties
        m_properties_ = model_.m_properties_->clone();

        // Clone instances
        m_sub_instances_ = clonePtrMap(model_.m_sub_instances_);

        // Clone events, area, ndd_power
        m_event_map_ = clonePtrMap(model_.m_event_map_);
        m_area_map_ = clonePtrMap(model_.m_area_map_);
        m_ndd_power_map_ = clonePtrMap(model_.m_ndd_power_map_);

        // Copy tech model pointer
        m_tech_model_ = model_.m_tech_model_;
    }

} // namespace DSENT

