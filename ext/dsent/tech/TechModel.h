#ifndef __DSENT_TECH_TECH_MODEL_H__
#define __DSENT_TECH_TECH_MODEL_H__

#include <vector>
#include <set>

#include "libutil/Config.h"
#include "libutil/String.h"

namespace DSENT
{
    class StdCellLib;

    using std::set;
    using std::vector;
    using LibUtil::String;

    class TechModel : public LibUtil::Config
    {
        public:
            typedef std::set<String>::const_iterator ConstWireLayerIterator;

        public:
            TechModel();
            virtual ~TechModel();

        public:
            // Set the pointer to a standard cell library
            void setStdCellLib(const StdCellLib* std_cell_lib_);
            // Get the pointer to the standard cell library
            const StdCellLib* getStdCellLib() const;

            // Return a cloned copy of this instance
            virtual TechModel* clone() const;
            // Override readFile function to include multiple technology files
            virtual void readFile(const String& filename_);

            // Transistor
            // Returns the leakage current of NMOS transistors, given the transistor stakcing, transistor widths, and input combination
            double calculateNmosLeakageCurrent(unsigned int num_stacks_, double uni_stacked_mos_width_, unsigned int input_vector_) const;
            double calculateNmosLeakageCurrent(unsigned int num_stacks_, const vector<double>& stacked_mos_widths_, unsigned int input_vector_) const;
            // Returns the leakage current of PMOS transistors, given the transistor stakcing, transistor widths, and input combination
            double calculatePmosLeakageCurrent(unsigned int num_stacks_, double uni_stacked_mos_width_, unsigned int input_vector_) const;
            double calculatePmosLeakageCurrent(unsigned int num_stacks_, const vector<double>& stacked_mos_widths_, unsigned int input_vector_) const;
            // Returns the leakage current, given the transistor stakcing, transistor widths, input combination,
            // and technology information (vdd, subthreshold swing, subthreshold dibl swing)
            double calculateLeakageCurrentFactor(unsigned int num_stacks_, const vector<double>& stacked_mos_widths_, unsigned int input_vector_, double vdd_, double subthreshold_swing_, double dibl_swing_) const;

            // Wire
            // Check if the wire layer exist
            bool isWireLayerExist(const String& layer_name_) const;
            const std::set<String>* getAvailableWireLayers() const;
            // Return wire capacitance for given wire layer, wire width, wire spacing, and wire length
            double calculateWireCapacitance(const String& layer_name_, double width_, double spacing_, double length_) const;
            // Return wire resistance for given wire layer, wire width, and wire length
            double calculateWireResistance(const String& layer_name_, double width_, double length_) const;

        private:
            // Private copy constructor. Use clone to perform copy operation
            TechModel(const TechModel& tech_model_);

        private:
            // A pointer to a standard cell library
            const StdCellLib* m_std_cell_lib_;
            // A set of available wire layers
            std::set<String>* m_available_wire_layers_;
    }; // class TechModel
} // namespace DSENT

#endif // __DSENT_TECH_TECH_MODEL_H__

