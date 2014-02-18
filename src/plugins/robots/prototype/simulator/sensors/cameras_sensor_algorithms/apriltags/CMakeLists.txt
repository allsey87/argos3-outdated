#
# Cameras Sensor Apriltags Algorithm
#

#
# Algorithm headers
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_APRILTAGS
    cameras_sensor_apriltags_algorithm.h)

#
# Algorithm sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_APRILTAGS
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_APRILTAGS}
    cameras_sensor_apriltags_algorithm.cpp)

#
# Create the Cameras Sensor Apriltags Algorithm
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_camerassensoralgorithm_apriltags SHARED
            ${ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_APRILTAGS})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_camerassensoralgorithm_apriltags
                      argos3plugin_${ARGOS_BUILD_FOR}_prototype)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_APRILTAGS} 
        DESTINATION include/argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/apriltags)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_camerassensoralgorithm_apriltags
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
