#ifndef __DSENT_UTIL_CONFIG_H__
#define __DSENT_UTIL_CONFIG_H__

#include "util/CommonType.h"

namespace DSENT
{
    class TechModel;
    class StdCellLib;

    class Config : public LibUtil::Config
    {
        public:
            static void allocate(const String& cfg_file_name_);
            static void release();
            static Config* getSingleton();

        protected:
            static Config* ms_singleton_;

        public:
            Config();
            ~Config();

        public:
            void setTechModel(const TechModel* tech_model_);
            const TechModel* getTechModel() const;

            void constructTechModel(const String& overwrite_str_);

        protected:
            void readFile(const String& file_name_);

        protected:
            const TechModel* m_tech_model_;
    }; // class Config
} // namespace DSENT

#endif // __DSENT_UTIL_CONFIG_H__

