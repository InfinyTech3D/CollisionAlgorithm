#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/helper/system/SetDirectory.h>
#include <stdio.h>
#include <string.h>

#include <string>

#define Q(x) #x
#define QUOTE(x) Q(x)

namespace sofa::collisionAlgorithm
{

// CollisionPipeline
extern void registerCollisionLoop(sofa::core::ObjectFactory* factory);

// Algorithms
extern void registerFind2DClosestProximityAlgorithm(sofa::core::ObjectFactory* factory);
extern void registerFindClosestProximityAlgorithm(sofa::core::ObjectFactory* factory);
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
}  // namespace sofa::collisionAlgorithm

namespace sofa::component
{

// Here are just several convenient functions to help user to know what contains the plugin

extern "C"
{
    void initExternalModule();
    const char* getModuleName();
    const char* getModuleVersion();
    const char* getModuleLicense();
    const char* getModuleDescription();
    void registerObjects(sofa::core::ObjectFactory* factory);
}

void initCollisionAlgorithm() 
{
    static bool first = true;
    if (first)
    {
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

const char* getModuleName() { return "CollisionAlgorithm"; }

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
    using namespace sofa::collisionAlgorithm;
    // Register CollisionPipeline
    registerCollisionLoop(factory);
    // Register Algorithms
    registerFind2DClosestProximityAlgorithm(factory);
    registerFindClosestProximityAlgorithm(factory);
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

}  // namespace sofa::component
