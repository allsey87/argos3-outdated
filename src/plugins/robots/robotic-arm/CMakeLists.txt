#
# Robotic-arm headers
#
# argos3/plugins/robots/robotic-arm/control_interface

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOTICARM_CONTROLINTERFACE
  control_interface/ci_roboticarm_rotational_joint_sensor.h
  control_interface/ci_roboticarm_rotational_joint_actuator.h)
# argos3/plugins/robots/-bot/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOTICARM_SIMULATOR
    simulator/dynamics2d_roboticarm_model.h
    simulator/dynamics3d_roboticarm_model.h
#   simulator/roboticarm_rotational_joint_sensor.h
#   simulator/roboticarm_rotational_joint_actuator.h
    simulator/link_entity.h
    simulator/link_equipped_entity.h
    simulator/roboticarm_entity.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Robotic arm sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOTICARM
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOTICARM_CONTROLINTERFACE}
    control_interface/ci_roboticarm_rotational_joint_sensor.cpp
    control_interface/ci_roboticarm_rotational_joint_actuator.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOTICARM
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOTICARM}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOTICARM_SIMULATOR}
    simulator/dynamics2d_roboticarm_model.cpp
    simulator/dynamics3d_roboticarm_model.cpp
#   simulator/roboticarm_rotational_joint_sensor.cpp
#   simulator/roboticarm_rotational_joint_actuator.cpp
    simulator/link_entity.cpp
    simulator/link_equipped_entity.cpp
    simulator/roboticarm_entity.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  include(ARGoSCheckQTOpenGL)
  if(ARGOS_COMPILE_QTOPENGL)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOTICARM_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOTICARM_SIMULATOR}
      simulator/qtopengl_roboticarm.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOTICARM
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOTICARM}
      simulator/qtopengl_roboticarm.h
      simulator/qtopengl_roboticarm.cpp)
  endif(ARGOS_COMPILE_QTOPENGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create the robotic-arm plugin
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
add_library(argos3plugin_${ARGOS_BUILD_FOR}_roboticarm SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_ROBOTICARM})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_roboticarm
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_actuators
  argos3plugin_${ARGOS_BUILD_FOR}_sensors
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics3d)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_roboticarm argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOTICARM_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/robotic-arm/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_ROBOTICARM_SIMULATOR}      DESTINATION include/argos3/plugins/robots/robotic-arm/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_roboticarm
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
