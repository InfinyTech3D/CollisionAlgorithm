\defgroup -Sofa Sofa
\defgroup -Sofa-sofa sofa
\ingroup -Sofa
\defgroup -Sofa-sofa-collisionAlgorithm collisionAlgorithm
\ingroup -Sofa-sofa
\defgroup -Sofa-sofa-collisionAlgorithm-AABBBroadPhase AABBBroadPhase
\ingroup -Sofa-sofa-collisionAlgorithm
##Description

**AABBBroadPhase** is a real-time collision detection component that uses the Axis-Aligned Bounding Box (AABB) approach for broad-phase collision handling.

It organizes elements into a 3D spatial grid, efficiently indexing and retrieving them to identify potential collisions, optimizing the process of narrowing down collision checks for more precise detection in 3D simulations.


##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
+ **nbox** (<em>sofa::type::Vec<3u, int></em>): <br>number of bbox
+ **isStatic** (<em>bool</em>): <br>Optimization: object is not moving in the scene
+ **method** (<em>int</em>): <br>chosen method to determine the boxes containing the elements
+ **thread** (<em>int</em>): <br>Number of threads
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

\defgroup -Sofa-sofa-collisionAlgorithm-CollisionLoop CollisionLoop
\ingroup -Sofa-sofa-collisionAlgorithm
##Description
CollisionLoop

##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

\defgroup -Sofa-sofa-collisionAlgorithm-DistanceFilter DistanceFilter
\ingroup -Sofa-sofa-collisionAlgorithm
##Description
DistanceFilter

##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
+ **distance** (<em>double</em>): <br>Min distance
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

\defgroup -Sofa-sofa-collisionAlgorithm-EdgeGeometry EdgeGeometry
\ingroup -Sofa-sofa-collisionAlgorithm
##Description
EdgeGeometry

\defgroup -Sofa-sofa-collisionAlgorithm-EdgeGeometry-Vec3d EdgeGeometry<Vec3d>
\ingroup -Sofa-sofa-collisionAlgorithm-EdgeGeometry 
##Description
EdgeGeometry

##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
+ **color** (<em>sofa::type::RGBAColor</em>): <br>Color of the collision model
+ **drawScaleNormal** (<em>double</em>): <br>Color of the collision model
+ **draw** (<em>bool</em>): <br>draw
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

\defgroup -Sofa-sofa-collisionAlgorithm-Find2DClosestProximityAlgorithm Find2DClosestProximityAlgorithm
\ingroup -Sofa-sofa-collisionAlgorithm
##Description
Find2DClosestProximityAlgorithm

##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
+ **drawcollision** (<em>bool</em>): <br>draw collision
+ **projectionMatrix** (<em>sofa::type::Mat<3u, 4u, double></em>): <br>projection matrix
+ **output** (<em>sofa::collisionAlgorithm::DetectionOutput<sofa::collisionAlgorithm::BaseProximity, sofa::collisionAlgorithm::BaseProximity></em>): <br>output of the collision detection
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

\defgroup -Sofa-sofa-collisionAlgorithm-FindClosestProximityAlgorithm FindClosestProximityAlgorithm
\ingroup -Sofa-sofa-collisionAlgorithm
##Description
FindClosestProximityAlgorithm

##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
+ **drawcollision** (<em>bool</em>): <br>draw collision
+ **output** (<em>sofa::collisionAlgorithm::DetectionOutput<sofa::collisionAlgorithm::BaseProximity, sofa::collisionAlgorithm::BaseProximity></em>): <br>output of the collision detection
+ **projective** (<em>bool</em>): <br>projection of closest prox onto from element
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

\defgroup -Sofa-sofa-collisionAlgorithm-FullAABBBroadPhase FullAABBBroadPhase
\ingroup -Sofa-sofa-collisionAlgorithm
##Description
FullAABBBroadPhase

##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
+ **nbox** (<em>sofa::type::Vec<3u, int></em>): <br>number of bbox
+ **isStatic** (<em>bool</em>): <br>Optimization: object is not moving in the scene
+ **method** (<em>int</em>): <br>chosen method to determine the boxes containing the elements
+ **thread** (<em>int</em>): <br>Number of threads
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

\defgroup -Sofa-sofa-collisionAlgorithm-PointGeometry PointGeometry
\ingroup -Sofa-sofa-collisionAlgorithm
##Description
PointGeometry

\defgroup -Sofa-sofa-collisionAlgorithm-PointGeometry-Vec3d PointGeometry<Vec3d>
\ingroup -Sofa-sofa-collisionAlgorithm-PointGeometry 
##Description
PointGeometry

##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
+ **color** (<em>sofa::type::RGBAColor</em>): <br>Color of the collision model
+ **drawScaleNormal** (<em>double</em>): <br>Color of the collision model
+ **draw** (<em>bool</em>): <br>draw
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

\defgroup -Sofa-sofa-collisionAlgorithm-SubsetGeometry SubsetGeometry
\ingroup -Sofa-sofa-collisionAlgorithm
##Description
SubsetGeometry

\defgroup -Sofa-sofa-collisionAlgorithm-SubsetGeometry-Vec3d SubsetGeometry<Vec3d>
\ingroup -Sofa-sofa-collisionAlgorithm-SubsetGeometry 
##Description
SubsetGeometry

##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
+ **color** (<em>sofa::type::RGBAColor</em>): <br>Color of the collision model
+ **drawScaleNormal** (<em>double</em>): <br>Color of the collision model
+ **draw** (<em>bool</em>): <br>draw
+ **indices** (<em>sofa::type::vector<int, sofa::type::CPUMemoryManager<int> ></em>): <br>Indices to keep
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

\defgroup -Sofa-sofa-collisionAlgorithm-TetrahedronGeometry TetrahedronGeometry
\ingroup -Sofa-sofa-collisionAlgorithm
##Description
TetrahedronGeometry

\defgroup -Sofa-sofa-collisionAlgorithm-TetrahedronGeometry-Vec3d TetrahedronGeometry<Vec3d>
\ingroup -Sofa-sofa-collisionAlgorithm-TetrahedronGeometry 
##Description
TetrahedronGeometry

##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
+ **color** (<em>sofa::type::RGBAColor</em>): <br>Color of the collision model
+ **drawScaleNormal** (<em>double</em>): <br>Color of the collision model
+ **draw** (<em>bool</em>): <br>draw
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

\defgroup -Sofa-sofa-collisionAlgorithm-TriangleGeometry TriangleGeometry
\ingroup -Sofa-sofa-collisionAlgorithm
##Description
TriangleGeometry

\defgroup -Sofa-sofa-collisionAlgorithm-TriangleGeometry-Vec3d TriangleGeometry<Vec3d>
\ingroup -Sofa-sofa-collisionAlgorithm-TriangleGeometry 
##Description
TriangleGeometry

##Parameters: 
+ **name** (<em>std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> ></em>): <br>object name
+ **printLog** (<em>bool</em>): <br>if true, emits extra messages at runtime.
+ **tags** (<em>sofa::core::objectmodel::TagSet</em>): <br>list of the subsets the object belongs to
+ **bbox** (<em>sofa::type::BoundingBox</em>): <br>this object bounding box
+ **componentState** (<em>sofa::core::objectmodel::ComponentState</em>): <br>The state of the component among (Dirty, Valid, Undefined, Loading, Invalid).
+ **listening** (<em>bool</em>): <br>if true, handle the events, otherwise ignore the events
+ **color** (<em>sofa::type::RGBAColor</em>): <br>Color of the collision model
+ **drawScaleNormal** (<em>double</em>): <br>Color of the collision model
+ **draw** (<em>bool</em>): <br>draw
##Output Signals: 
+ __connection__ : Signal event called at connection
+ __disconnection__ : Signal event called at connection

