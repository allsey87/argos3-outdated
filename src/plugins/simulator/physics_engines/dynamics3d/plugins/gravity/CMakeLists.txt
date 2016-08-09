#
# Dynamics3D Gravity plugin
#

#
# Plugin headers
#
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D_PLUGINS_GRAVITY
    dynamics3d_gravity_plugin.h)

#
# Plugin sources
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D_PLUGINS_GRAVITY
    ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D_PLUGINS_GRAVITY}
    dynamics3d_gravity_plugin.cpp)

#
# Create the gravity plugin
#
include_directories(
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/BroadphaseCollision
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionDispatch
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionShapes
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/Gimpact
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/NarrowPhaseCollision
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Character
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/ConstraintSolver
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Dynamics
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Featherstone
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/MLCPSolvers
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Vehicle
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/LinearMath)
add_library(argos3plugin_${ARGOS_BUILD_FOR}_dyn3dplugin_gravity SHARED ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D_PLUGINS_GRAVITY})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_dyn3dplugin_gravity
                      argos3plugin_${ARGOS_BUILD_FOR}_dynamics3d
                      argos3plugin_${ARGOS_BUILD_FOR}_prototype)

install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D_PLUGINS_GRAVITY} DESTINATION include/argos3/plugins/physics_engines/dynamics3d/plugins/gravity)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_dyn3dplugin_gravity
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
