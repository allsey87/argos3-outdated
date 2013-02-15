#
# Headers
#
# Headers of the 3d dynamics physics engine
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D
  dynamics3d_box_entity.h
  dynamics3d_entity.h
  dynamics3d_engine.h)

#
# Source files
#
# Dynamics 3D source files
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D
  dynamics3d_box_entity.cpp
  dynamics3d_engine.cpp)

#
# Create dynamics3d engine plugin library
#
# Add the bullet include paths
include_directories(${BULLET_INCLUDE_DIRS})
# Create library
add_library(argos3plugin_${ARGOS_BUILD_FOR}_dynamics3d SHARED
  ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_dynamics3d ${BULLET_LIBRARIES})
install(FILES
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D}
  DESTINATION
  include/argos3/plugins/simulator/physics_engines/dynamics3d)
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_dynamics3d
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
