#include "Config/ConfigurationLoader.h"
#include "Config/TemplateWanderConfigurationLoader.h"
#include "Config/TemplateBoidsConfigurationLoader.h"
#include "Config/TemplateRandomwalkConfigurationLoader.h"
#include "Config/TemplateEEConfigurationLoader.h"
#include "Config/TemplateVanillaEEConfigurationLoader.h"
#include "Config/TemplateMedeaConfigurationLoader.h"
#include "Config/TemplateMovingConfigurationLoader.h"
#include "Config/MovingEEConfigurationLoader.h"
#include "Config/MovingNSConfigurationLoader.h"
#include "Config/MonoRobotConfigurationLoader.h"
#include "Config/SingleGenomeConfigurationLoader.h"
#include "Config/CoopOpportunity2MaxConfigurationLoader.h"
#include "Config/CoopFixed2ConfigurationLoader.h"
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
#if defined PRJ_TEMPLATEWANDER || !defined MODULAR
	else if (configurationLoaderObjectName == "TemplateWanderConfigurationLoader" )
	{
		return new TemplateWanderConfigurationLoader();
	}
#endif
#if defined PRJ_TEMPLATEBOIDS || !defined MODULAR
	else if (configurationLoaderObjectName == "TemplateBoidsConfigurationLoader" )
	{
		return new TemplateBoidsConfigurationLoader();
	}
#endif
#if defined PRJ_TEMPLATERANDOMWALK || !defined MODULAR
	else if (configurationLoaderObjectName == "TemplateRandomwalkConfigurationLoader" )
	{
		return new TemplateRandomwalkConfigurationLoader();
	}
#endif
#if defined PRJ_TEMPLATEEE || !defined MODULAR
    else if (configurationLoaderObjectName == "TemplateEEConfigurationLoader" )
    {
        return new TemplateEEConfigurationLoader();
    }
#endif
#if defined PRJ_TEMPLATEVANILLAEE || !defined MODULAR
	else if (configurationLoaderObjectName == "TemplateVanillaEEConfigurationLoader" )
	{
		return new TemplateVanillaEEConfigurationLoader();
	}
#endif
#if defined PRJ_TEMPLATEMEDEA || !defined MODULAR
	else if (configurationLoaderObjectName == "TemplateMedeaConfigurationLoader" )
	{
		return new TemplateMedeaConfigurationLoader();
	}
#endif
#if defined PRJ_TEMPLATEMOVING || !defined MODULAR
	else if (configurationLoaderObjectName == "TemplateMovingConfigurationLoader" )
	{
		return new TemplateMovingConfigurationLoader();
	}
#endif
#if defined PRJ_MOVINGEE || !defined MODULAR
	else if (configurationLoaderObjectName == "MovingEEConfigurationLoader" )
	{
		return new MovingEEConfigurationLoader();
	}
#endif
#if defined PRJ_MOVINGNS || !defined MODULAR
	else if (configurationLoaderObjectName == "MovingNSConfigurationLoader" )
	{
		return new MovingNSConfigurationLoader();
	}
#endif
#if defined PRJ_MONOROBOT || !defined MODULAR
	else if (configurationLoaderObjectName == "MonoRobotConfigurationLoader" )
	{
		return new MonoRobotConfigurationLoader();
	}
#endif
#if defined PRJ_SINGLEGENOME || !defined MODULAR
	else if (configurationLoaderObjectName == "SingleGenomeConfigurationLoader" )
	{
		return new SingleGenomeConfigurationLoader();
	}
#endif
#if defined PRJ_COOPOPPORTUNITY2MAX || !defined MODULAR
	else if (configurationLoaderObjectName == "CoopOpportunity2MaxConfigurationLoader" )
	{
		return new CoopOpportunity2MaxConfigurationLoader();
	}
#endif
#if defined PRJ_COOPFIXED2 || !defined MODULAR
	else if (configurationLoaderObjectName == "CoopFixed2ConfigurationLoader" )
	{
		return new CoopFixed2ConfigurationLoader();
	}
#endif
    //###DO-NOT-DELETE-THIS-LINE###TAG:SWITCH###//
	else
	{
		return NULL;
	}

}
