#include "util/Config.h"

#include "model/std_cells/StdCellLib.h"

namespace DSENT
{

    Config* Config::ms_singleton_ = NULL;

    void Config::allocate(const String& cfg_file_name_)
    {
        Log::printLine("Config::allocate");

        // Allocate static Config instance
        ASSERT(!ms_singleton_, "Config singleton is allocated");
        ms_singleton_ = new Config();
        ms_singleton_->readFile(cfg_file_name_);

        Log::printLine("Config::allocate - End");
        return;
    }

    void Config::release()
    {
        Log::printLine("Config::release");

        // Release static Config instance
        ASSERT(ms_singleton_, "Config singleton is not allocated");
        delete ms_singleton_;
        ms_singleton_ = NULL;

        Log::printLine("Config::release - End");
        return;
    }

    Config* Config::getSingleton()
    {
        ASSERT(ms_singleton_, "Config singleton is not allocated");
        return ms_singleton_;
    }

    Config::Config()
        : m_tech_model_(NULL)
    {}

    Config::~Config()
    {
        delete m_tech_model_;
    }

    void Config::setTechModel(const TechModel* tech_model_)
    {
        ASSERT((tech_model_ != NULL), "tech_model_ is null");

        m_tech_model_ = tech_model_;
        return;
    }

    const TechModel* Config::getTechModel() const
    {
        ASSERT((m_tech_model_ != NULL), "m_tech_model_ is null");

        return m_tech_model_;
    }

    void Config::readFile(const String& file_name_)
    {
        Log::printLine("Config::readFile");

        LibUtil::Config::readFile(file_name_);

        Log::printLine("Config::readFile - End");
        return;
    }

    void Config::constructTechModel(const String& overwrite_str_)
    {
        Log::printLine("Config::constructTechModel");

        // Allocate static TechModel instance
        const String& electrical_tech_model_filename = get("ElectricalTechModelFilename");

        TechModel* tech_model = new TechModel();
        tech_model->readFile(electrical_tech_model_filename);
        if(keyExist("PhotonicTechModelFilename"))
        {
            const String& photonic_tech_model_filename = get("PhotonicTechModelFilename");
            tech_model->readFile(photonic_tech_model_filename);
        }

        // Overwrite the settings at runtime
        tech_model->readString(overwrite_str_);

        // Allocate static StdCellLib instance
        StdCellLib* std_cell_lib = new StdCellLib(tech_model);

        // Set the StdCellLib pointer in static TechModel instance
        tech_model->setStdCellLib(std_cell_lib);

        m_tech_model_ = tech_model;
        Log::printLine("Config::constructTechModel - End");
        return;
    }
} // namespace DSENT

