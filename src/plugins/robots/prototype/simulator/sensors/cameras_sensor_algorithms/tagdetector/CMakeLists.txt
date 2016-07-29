#
# Cameras Sensor Tag Detector Algorithm
#

#
# Algorithm headers
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_TAGDETECTOR
    cameras_sensor_tagdetector_algorithm.h)

#
# Algorithm sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_TAGDETECTOR
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_TAGDETECTOR}
    cameras_sensor_tagdetector_algorithm.cpp)

#
# Create the Cameras Sensor Tag Detector Algorithm
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_camerassensoralgorithm_tagdetector SHARED
            ${ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_TAGDETECTOR})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_camerassensoralgorithm_tagdetector
                      argos3plugin_${ARGOS_BUILD_FOR}_prototype)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_TAGDETECTOR} 
        DESTINATION include/argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/tagdetector)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_camerassensoralgorithm_tagdetector
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
