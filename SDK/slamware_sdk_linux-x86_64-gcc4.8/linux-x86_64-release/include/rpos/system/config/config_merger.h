#pragma once

#include <rpos/core/rpos_core_config.h>
#include <json/json.h>


namespace rpos { namespace system { namespace config {

    class RPOS_CORE_API ConfigMerger
    {
    public:
        /*merge reference config into ccustomized config according to white list policy
        @customized: config file from customers
        @ref: our reference config
        @whiteList: policy written in Json format that specify whether customer has the authority to config a field.
                    only fields that marked "accept" can be used from @customized
                    if a field ia json array type and marked in whiteList as "combine", then merged config will contain a union of the array
        */
        static bool mergeConfigInplace(Json::Value& customized, const Json::Value& ref, const Json::Value& whiteList, const std::vector<std::string>& ignoreList);
		static bool mergeConfigInplace(Json::Value& customized, const Json::Value& ref, const Json::Value& whiteList, const std::string& path, const std::vector<std::string>& ignoreList);
        static bool overwriteConfigInplace(const Json::Value& customized, Json::Value& ref);
        static Json::Value mergeConfig(const Json::Value& customized, const Json::Value& ref, const Json::Value& whiteList, const std::vector<std::string>& ignoreList);
        //overwrite all contents in ref with customized
        static Json::Value overwriteConfig(const Json::Value& customized, const Json::Value& ref);
    };

} } }