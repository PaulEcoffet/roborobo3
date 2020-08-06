#include <ext/Config/NegociateConfigurationLoader.h>
#include <ext/Config/MaxOneConfigurationLoader.h>
#include <ext/Config/LionConfigurationLoader.h>
#include "Config/ConfigurationLoader.h"
#include "Config/TemplateWanderConfigurationLoader.h"
#include "Config/TemplateBoidsConfigurationLoader.h"
#include "Config/TemplateRandomwalkConfigurationLoader.h"
#include "Config/TemplateEEConfigurationLoader.h"
#include "Config/TemplateVanillaEEConfigurationLoader.h"
#include "Config/TemplateMedeaConfigurationLoader.h"
#include "Config/ForagingRegionsConfigurationLoader.h"
#include "Config/TutorialConfigurationLoader.h"
#include "Config/PyNegotiateConfigurationLoader.h"
//###DO-NOT-DELETE-THIS-LINE###TAG:INCLUDE###//


ConfigurationLoader::ConfigurationLoader()
{
    //nothing to do
}

ConfigurationLoader::~ConfigurationLoader()
{
    //nothing to do
}

ConfigurationLoader* ConfigurationLoader::make_ConfigurationLoader (std::string configurationLoaderObjectName)
{
    if (0)
    {
        // >>> Never reached
    }
#if defined PRJ_NEGOCIATE || !defined MODULAR
    else if (configurationLoaderObjectName == "NegociateConfigurationLoader")
    {
        return new NegociateConfigurationLoader();
    }
#endif
#if defined PRJ_MAXONE || !defined MODULAR
    else if (configurationLoaderObjectName == "MaxOneConfigurationLoader")
    {
        return new MaxOneConfigurationLoader();
    }
#endif
#if defined PRJ_LION || !defined MODULAR
    else if (configurationLoaderObjectName == "LionConfigurationLoader")
    {
        return new LionConfigurationLoader();
    }
#endif
#if defined PRJ_PYNEGOTIATE || !defined MODULAR
    else if (configurationLoaderObjectName == "PyNegotiateConfigurationLoader")
    {
        return new PyNegotiateConfigurationLoader();
    }
#endif
        //###DO-NOT-DELETE-THIS-LINE###TAG:SWITCH###//
    else
    {
        std::cerr << "No configuration loader found. Exiting." << std::endl;
        exit(1);
        return NULL;
    }

}
