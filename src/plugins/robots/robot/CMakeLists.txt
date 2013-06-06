#
# Robot headers
#
# argos3/plugins/robots/robot/control_interface

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOT_CONTROLINTERFACE
  control_interface/ci_robot_joints_sensor.h
  control_interface/ci_robot_joints_actuator.h)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOT_SIMULATOR
    simulator/dynamics3d_robot_model.h
    simulator/robot_joints_default_sensor.h
    simulator/robot_joints_default_actuator.h
    simulator/body_entity.h
    simulator/body_equipped_entity.h
    simulator/frame_entity.h
    simulator/frame_equipped_entity.h
    simulator/joint_entity.h
    simulator/joint_equipped_entity.h
    simulator/robot_entity.h
    simulator/geometry3.h
    simulator/box_geometry3.h
    simulator/cylinder_geometry3.h
    simulator/sphere_geometry3.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Robot sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOT
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOT_CONTROLINTERFACE}
    control_interface/ci_robot_joints_sensor.cpp
    control_interface/ci_robot_joints_actuator.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOT
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOT}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOT_SIMULATOR}

    simulator/dynamics3d_robot_model.cpp
    simulator/robot_joints_default_sensor.cpp
    simulator/robot_joints_default_actuator.cpp
    simulator/body_entity.cpp
    simulator/body_equipped_entity.cpp
    simulator/frame_entity.cpp
    simulator/frame_equipped_entity.cpp
    simulator/joint_entity.cpp
    simulator/joint_equipped_entity.cpp
    simulator/robot_entity.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  include(ARGoSCheckQTOpenGL)
  if(ARGOS_COMPILE_QTOPENGL)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOT_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOT_SIMULATOR}
      simulator/qtopengl_robot.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOT
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOT}
      simulator/qtopengl_robot.cpp)
  endif(ARGOS_COMPILE_QTOPENGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create the robot plugin
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
add_library(argos3plugin_${ARGOS_BUILD_FOR}_robot SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOT})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_robot
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_actuators
  argos3plugin_${ARGOS_BUILD_FOR}_sensors
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics3d)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_robot argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOT_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/robot/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOT_SIMULATOR}      DESTINATION include/argos3/plugins/robots/robot/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_robot
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
