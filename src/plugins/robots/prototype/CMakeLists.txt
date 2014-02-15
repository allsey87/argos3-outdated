#
# Robot headers
#
# argos3/plugins/robots/prototype/control_interface

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_CONTROLINTERFACE
#  control_interface/ci_cameras_sensor.h
  control_interface/ci_prototype_joints_sensor.h
  control_interface/ci_prototype_joints_actuator.h
  control_interface/ci_prototype_proximity_sensor.h
  control_interface/ci_prototype_radios_actuator.h
  control_interface/ci_prototype_radios_sensor.h
  control_interface/ci_prototype_electromagnets_actuator.h)
  
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_UTILITY
  utility/geometry3.h
  utility/box_geometry3.h
  utility/cylinder_geometry3.h
  utility/sphere_geometry3.h)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_ACTUATORS
    simulator/actuators/prototype_joints_default_actuator.h
    simulator/actuators/prototype_leds_default_actuator.h
    simulator/actuators/prototype_electromagnets_default_actuator.h
    simulator/actuators/prototype_radios_default_actuator.h)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_ENTITIES
    simulator/entities/barcode2_entity.h
    simulator/entities/barcode2_equipped_entity.h
    simulator/entities/body_entity.h
    simulator/entities/body_equipped_entity.h
    simulator/entities/frame_entity.h
    simulator/entities/frame_equipped_entity.h
    simulator/entities/joint_entity.h
    simulator/entities/joint_equipped_entity.h
    simulator/entities/electromagnet_entity.h
    simulator/entities/electromagnet_equipped_entity.h
    simulator/entities/camera_entity.h
    simulator/entities/camera_equipped_entity.h
    simulator/entities/radio_entity.h
    simulator/entities/radio_equipped_entity.h
    simulator/entities/prototype_proximity_sensor_equipped_entity.h
    simulator/entities/prototype_entity.h
    simulator/entities/prototype_led_equipped_entity.h)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_MEDIA
    simulator/media/barcode2_medium.h
    simulator/media/radio_medium.h)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_PHYSICS_MODELS
    simulator/physics_models/dynamics3d_prototype_model.h)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS
#    simulator/sensors/cameras_default_sensor.h
    simulator/sensors/prototype_joints_default_sensor.h
    simulator/sensors/prototype_proximity_default_sensor.h
    simulator/sensors/prototype_radios_default_sensor.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Robot sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_UTILITY}
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_CONTROLINTERFACE}
#    control_interface/ci_cameras_sensor.cpp
    control_interface/ci_prototype_joints_sensor.cpp
    control_interface/ci_prototype_joints_actuator.cpp
    control_interface/ci_prototype_proximity_sensor.cpp
    control_interface/ci_prototype_radios_actuator.cpp
    control_interface/ci_prototype_radios_sensor.cpp
    control_interface/ci_prototype_electromagnets_actuator.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_ACTUATORS}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_ENTITIES}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_MEDIA}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_PHYSICS_MODELS}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS}
    simulator/actuators/prototype_joints_default_actuator.cpp
    simulator/actuators/prototype_leds_default_actuator.cpp
    simulator/actuators/prototype_electromagnets_default_actuator.cpp
    simulator/actuators/prototype_radios_default_actuator.cpp    
    simulator/entities/barcode2_entity.cpp
    simulator/entities/barcode2_equipped_entity.cpp
    simulator/entities/body_entity.cpp
    simulator/entities/body_equipped_entity.cpp
    simulator/entities/frame_entity.cpp
    simulator/entities/frame_equipped_entity.cpp
    simulator/entities/joint_entity.cpp
    simulator/entities/joint_equipped_entity.cpp
    simulator/entities/electromagnet_entity.cpp
    simulator/entities/electromagnet_equipped_entity.cpp
    simulator/entities/camera_entity.cpp
    simulator/entities/camera_equipped_entity.cpp
    simulator/entities/radio_entity.cpp
    simulator/entities/radio_equipped_entity.cpp
    simulator/entities/prototype_entity.cpp
    simulator/entities/prototype_proximity_sensor_equipped_entity.cpp    
    simulator/entities/prototype_led_equipped_entity.cpp
    simulator/media/barcode2_medium.cpp
    simulator/media/radio_medium.cpp
    simulator/physics_models/dynamics3d_prototype_model.cpp
#    simulator/sensors/cameras_default_sensor.cpp
    simulator/sensors/prototype_joints_default_sensor.cpp    
    simulator/sensors/prototype_proximity_default_sensor.cpp
    simulator/sensors/prototype_radios_default_sensor.cpp)

  # Compile the graphical visualization only if the necessary libraries have been found
  include(ARGoSCheckQTOpenGL)
  if(ARGOS_COMPILE_QTOPENGL)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR}
      simulator/visualizations/qtopengl_prototype.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE}
      simulator/visualizations/qtopengl_prototype.cpp)
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
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_ACTUATORS}      DESTINATION include/argos3/plugins/robots/prototype/simulator/actuators)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_ENTITIES}       DESTINATION include/argos3/plugins/robots/prototype/simulator/entities)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_MEDIA}          DESTINATION include/argos3/plugins/robots/prototype/simulator/media)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_PHYSICS_MODELS} DESTINATION include/argos3/plugins/robots/prototype/simulator/physics_models)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS}        DESTINATION include/argos3/plugins/robots/prototype/simulator/sensors)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_prototype
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
