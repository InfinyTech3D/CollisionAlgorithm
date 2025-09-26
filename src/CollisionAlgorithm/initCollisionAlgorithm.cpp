#include <CollisionAlgorithm/initCollisionAlgorithm.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/helper/system/SetDirectory.h>
#include <stdio.h>
#include <string.h>

#include <string>

#define Q(x) #x
#define QUOTE(x) Q(x)

namespace sofa::collisionalgorithm
{

// CollisionPipeline
extern void registerCollisionLoop(sofa::core::ObjectFactory* factory);

// Algorithms
extern void registerFind2DClosestProximityAlgorithm(sofa::core::ObjectFactory* factory);
extern void registerInsertionAlgorithm(sofa::core::ObjectFactory* factory);

// BroadPhase
extern void registerAABBBroadPhase(sofa::core::ObjectFactory* factory);
extern void registerFullAABBBroadPhase(sofa::core::ObjectFactory* factory);

// Distance Filters
extern void registerDistanceFilter(sofa::core::ObjectFactory* factory);

// Geometry
extern void registerSubsetGeometry(sofa::core::ObjectFactory* factory);
extern void registerPointGeometry(sofa::core::ObjectFactory* factory);
extern void registerEdgeGeometry(sofa::core::ObjectFactory* factory);
extern void registerTriangleGeometry(sofa::core::ObjectFactory* factory);
extern void registerTetrahedronGeometry(sofa::core::ObjectFactory* factory);

extern "C"
{
    SOFA_COLLISIONALGORITHM_API void initExternalModule();
    SOFA_COLLISIONALGORITHM_API const char* getModuleName();
    SOFA_COLLISIONALGORITHM_API const char* getModuleVersion();
    SOFA_COLLISIONALGORITHM_API const char* getModuleLicense();
    SOFA_COLLISIONALGORITHM_API const char* getModuleDescription();
    SOFA_COLLISIONALGORITHM_API void registerObjects(sofa::core::ObjectFactory* factory);
}

void initCollisionAlgorithm() 
{
    static bool first = true;
    if (first)
    {
        // make sure that this plugin is registered into the PluginManager
        sofa::helper::system::PluginManager::getInstance().registerPlugin(MODULE_NAME);

        first = false;
#ifdef PLUGIN_DATA_DIR
        sofa::helper::system::DataRepository.addLastPath(std::string(QUOTE(PLUGIN_DATA_DIR)));
#endif
        sofa::helper::system::DataRepository.addLastPath(
            sofa::helper::system::SetDirectory::GetCurrentDir());
    }
}

void initExternalModule()
{
    initCollisionAlgorithm();
}

const char* getModuleName() { return MODULE_NAME; }

const char* getModuleVersion()
{
#ifdef PLUGIN_GIT_INFO
    return QUOTE(PLUGIN_GIT_INFO);
#else
    return "??? to get the last git hash you must active the setupGit macro in CMakeLists";
#endif
}

const char* getModuleLicense() { return "LGPL"; }

const char* getModuleDescription() { return "Plugin for collision detection"; }

void registerObjects(sofa::core::ObjectFactory* factory)
{
    // Register CollisionPipeline
    registerCollisionLoop(factory);
    // Register Algorithms
    registerFind2DClosestProximityAlgorithm(factory);
    registerInsertionAlgorithm(factory);
    // Register BroadPhase
    registerAABBBroadPhase(factory);
    registerFullAABBBroadPhase(factory);
    // Register Distance Filters
    registerDistanceFilter(factory);
    // Register Geometry
    registerSubsetGeometry(factory);
    registerPointGeometry(factory);
    registerEdgeGeometry(factory);
    registerTriangleGeometry(factory);
    registerTetrahedronGeometry(factory);
}

}  // namespace collisionalgorithm
