#
# Cameras Sensor LED Detector Algorithm
#

#
# Algorithm headers
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_LEDDETECTOR
    cameras_sensor_leddetector_algorithm.h)

#
# Algorithm sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_LEDDETECTOR
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_LEDDETECTOR}
    cameras_sensor_leddetector_algorithm.cpp)

#
# Create the Cameras Sensor LED Detector Algorithm
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_camerassensoralgorithm_leddetector SHARED
            ${ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_LEDDETECTOR})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_camerassensoralgorithm_leddetector
                      argos3plugin_${ARGOS_BUILD_FOR}_prototype)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERASSENSORALGORITHMS_LEDDETECTOR} 
        DESTINATION include/argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/leddetector)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_camerassensoralgorithm_leddetector
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
