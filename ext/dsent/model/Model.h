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

#ifndef __DSENT_MODEL_MODEL_H__
#define __DSENT_MODEL_MODEL_H__

#include <vector>

#include "util/CommonType.h"

namespace DSENT
{
    using std::vector;

    class TechModel;
    class Result;

    // Base class for all the models
    class Model
    {
        protected:
            class SubModel
            {
                public:
                    SubModel(Model* model_, double num_models_);
                    ~SubModel();

                public:
                    Model* getModel();
                    const Model* getModel() const;
                    double getNumModels() const;

                    SubModel* clone() const;

                protected:
                    SubModel(const SubModel& sub_model_);

                private:
                    // Pointer to the actual model instance
                    Model* m_model_;
                    // Number of models are added
                    double m_num_models_;
            };

        public:
            // Model Constants
            static const char TYPE_SEPARATOR[];
            static const char HIERARCHY_SEPARATOR[];
            static const char SUBFIELD_SEPARATOR[];
            static const char DETAIL_SEPARATOR[];

        public:
            Model(const String& instance_name_, const TechModel* tech_model_);
            virtual ~Model();

        public:
            // Set the name of this instance
            void setInstanceName(const String& instance_name_);
            // Get the name of this instance
            const String& getInstanceName() const;

            // Set if the model is the top model
            void setIsTopModel(bool is_top_model_);
            bool getIsTopModel() const;

            // Add a parameter name (with and without default)
            void addParameterName(const String& parameter_name_);
            void addParameterName(const String& parameter_name_, const String& parameter_default_);
            const vector<String>* getParameterNames() const;
            
            // Add a property name
            void addPropertyName(const String& property_name_);
            void addPropertyName(const String& property_name_, const String& property_default_);
            const vector<String>* getPropertyNames() const;

            // Check if all parameters needed exist in the m_parameters_
            void checkParameters() const;
            // Check if all properties needed exist in the m_properties_
            void checkProperties() const;

            // Get the pointer to parameters
            const ParameterMap* getParameters() const;
            const String getParameter(const String& parameter_name_) const;
            void setParameter(const String& parameter_name_, const String& parameter_value_);
            
            // Get the pointer to properties
            const PropertyMap* getProperties() const;
            const String getProperty(const String& property_name_) const;
            void setProperty(const String& property_name_, const String& property_value_);

            // Get the pointer to generated properties
            PropertyMap* getGenProperties();
            const PropertyMap* getGenProperties() const;

            // Add an instance to this model. num_sub_instances_ specifies the 
            // number of the same instance is added.
            void addSubInstances(Model* sub_instance_, double num_sub_instances_);
            // Get an instance by name.            
            Model* getSubInstance(const String& sub_instance_name_);
            const Model* getSubInstance(const String& sub_instance_name_) const;
            // Check if an instance exists
            bool hasSubInstance(const String& sub_instance_name_) const;

            // Add a new area
            void addAreaResult(Result* area_);
            // Get the pointer to an area. The area is specified by name.
            Result* getAreaResult(const String& area_name_);
            const Result* getAreaResult(const String& area_name_) const;
            // Check if an area exists
            bool hasAreaResult(const String& area_name_) const;

            // Add a new ndd_power
            void addNddPowerResult(Result* ndd_power_);
            // Get the pointer to an ndd_power. The ndd_power is specified by name.
            Result* getNddPowerResult(const String& ndd_power_name_);
            const Result* getNddPowerResult(const String& ndd_power_name_) const;
            // Check if a ndd_power exists
            bool hasNddPowerResult(const String& ndd_power_name_) const;

            // Add a new event
            void addEventResult(Result* event_);
            // Get the pointer to an event. The event is specified by name.
            Result* getEventResult(const String& event_name_);
            const Result* getEventResult(const String& event_name_) const;
            // Check if an event exists
            bool hasEventResult(const String& event_name_) const;

            // Get the pointer to the TechModel.
            const TechModel* getTechModel() const;

            // Clone and return a new instance
            virtual Model* clone() const;

            // Checks to make sure all required parameters are present, makes sure that the model
            // has not been constructed yet, and calls constructModel. This function is not meant
            // to be overwritten by child classes; constructModel should be overwritten instead
            void construct();
            // Update checks whether the model needs updating, whether all properties have been specified,
            // and calls updateModel if update is necessary. This function is not meant
            // to be overwritten by child classes; updateModel should be overwritten instead
            void update();
            // Evaluate checks whether the model needs to be evaluated. Note that this function is
            // not meant to be overwritten by child classes; evaluateModel should be overwritten
            // instead
            void evaluate();
            void use(const String& event_name_);
            void use();

            // Resolve query hierarchy and process query
            const void* parseQuery(const String& query_type_, const String& query_hier_, const String& query_sub_field_);
            // Process the query
            virtual const void* processQuery(const String& query_type_, const String& query_sub_field_);

            // Print hierarchically
            void printHierarchy(const String& query_type_, const String& query_sub_field_, const String& prepend_str_, int detail_level_, ostream& ost_) const;
            void printInstHierarchy(const String& prepend_str_, int detail_level_, ostream& ost_) const;

        protected:
            // Query area
            const Result* queryArea(const String& area_name_) const;
            // Query non-data-dependent power
            const Result* queryNddPower(const String& ndd_power_name_);
            // Query event energy cost
            const Result* queryEventEnergyCost(const String& event_name_);
            
            // Constructs the model
            virtual void constructModel() = 0;
            // Updates timing related information of the model
            virtual void updateModel();
            // Evaluate non data dependent power of the model
            virtual void evaluateModel();
            virtual void useModel(const String& event_name_);
            virtual void useModel();

        private:
            // Private copy constructor. Use clone to perform copy operation.
            Model(const Model& model_);

        private:
            // Name of this instance
            String m_instance_name_;
            // Set if this model is the top model
            bool m_is_top_model_;
            
            // Contains the parameters of a model
            // Parameters are needed in order to constructModel() and CANNOT be
            // changed after constructModel() has been called
            ParameterMap* m_parameters_;
            // Contains the names of all model parameters
            vector<String>* m_parameter_names_;
            // Contains the properties of a model
            // Properties are required in order to updateModel() and CAN be
            // changed be changed after constructModel(). Call updateModel()
            // after properties have been changed to use the new properties
            PropertyMap* m_properties_;
            // Contains the property names needed to update the model
            vector<String>* m_property_names_;
            // Contains generated properties of the model
            // Generated properties are used mostly as a scratch space for
            // variables used in the model that must be passed from one
            // function to another
            PropertyMap* m_generated_properties_;
            
            // Contains the instances of this model
            Map<SubModel*>* m_sub_instances_;
            // Contains the area resulst of a model
            Map<Result*>* m_area_map_;
            // Contains the noo power results of a model
            Map<Result*>* m_ndd_power_map_;
            // Contains the event results of a model
            Map<Result*>* m_event_map_;
            // Pointer to a TechModel which contains the technology information
            const TechModel* m_tech_model_;
            
            // Set when a model is constructed
            bool m_constructed_;
            // Set when a model is updated
            bool m_updated_;
            // Set when a model is evaluated
            bool m_evaluated_;
            
    }; // class Model
} // namespace DSENT

#endif // __DSENT_MODEL_MODEL_H__

