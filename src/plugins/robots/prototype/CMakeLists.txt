#
# Robot headers
#
# argos3/plugins/robots/prototype/control_interface

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_CONTROLINTERFACE
  control_interface/ci_prototype_joints_sensor.h
  control_interface/ci_prototype_joints_actuator.h
  control_interface/ci_prototype_proximity_sensor.h
  control_interface/ci_prototype_colored_blob_forwards_camera_sensor.h)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR
    simulator/dynamics3d_prototype_model.h
    simulator/prototype_joints_default_sensor.h
    simulator/prototype_joints_default_actuator.h
    simulator/body_entity.h
    simulator/body_equipped_entity.h
    simulator/frame_entity.h
    simulator/frame_equipped_entity.h
    simulator/joint_entity.h
    simulator/joint_equipped_entity.h
    simulator/prototype_entity.h
    simulator/prototype_proximity_default_sensor.h
    simulator/prototype_proximity_sensor_equipped_entity.h
    simulator/prototype_leds_default_actuator.h
    simulator/prototype_led_equipped_entity.h
    simulator/prototype_forwards_camera_equipped_entity.h
    simulator/prototype_colored_blob_forwards_camera_sensor.h
    simulator/geometry3.h
    simulator/box_geometry3.h
    simulator/cylinder_geometry3.h
    simulator/sphere_geometry3.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Robot sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_CONTROLINTERFACE}
    control_interface/ci_prototype_joints_sensor.cpp
    control_interface/ci_prototype_joints_actuator.cpp
    control_interface/ci_prototype_proximity_sensor.cpp
    control_interface/ci_prototype_colored_blob_forwards_camera_sensor.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR}

    simulator/dynamics3d_prototype_model.cpp
    simulator/prototype_joints_default_sensor.cpp
    simulator/prototype_joints_default_actuator.cpp
    simulator/body_entity.cpp
    simulator/body_equipped_entity.cpp
    simulator/frame_entity.cpp
    simulator/frame_equipped_entity.cpp
    simulator/joint_entity.cpp
    simulator/joint_equipped_entity.cpp
    simulator/prototype_entity.cpp
    simulator/prototype_proximity_default_sensor.cpp
    simulator/prototype_proximity_sensor_equipped_entity.cpp
    simulator/prototype_leds_default_actuator.cpp
    simulator/prototype_led_equipped_entity.cpp
    simulator/prototype_forwards_camera_equipped_entity.cpp
    simulator/prototype_colored_blob_forwards_camera_sensor.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  include(ARGoSCheckQTOpenGL)
  if(ARGOS_COMPILE_QTOPENGL)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR}
      simulator/qtopengl_prototype.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE}
      simulator/qtopengl_prototype.cpp)
  endif(ARGOS_COMPILE_QTOPENGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create the prototype plugin
#
include_directories(
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/BroadphaseCollision
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionDispatch
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionShapes
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/Gimpact
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/NarrowPhaseCollision
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/ConstraintSolver
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Dynamics
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Vehicle
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Character
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/LinearMath)
add_library(argos3plugin_${ARGOS_BUILD_FOR}_prototype SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_prototype
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_actuators
  argos3plugin_${ARGOS_BUILD_FOR}_sensors
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics3d)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_prototype argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/prototype/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR}      DESTINATION include/argos3/plugins/robots/prototype/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_prototype
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
