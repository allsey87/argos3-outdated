#
# Camera Default Sensor LED Detector Algorithm
#

#
# Algorithm headers
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERADEFAULTSENSORALGORITHMS_LEDDETECTOR
    cameras_default_sensor_leddetector_algorithm.h)

#
# Algorithm sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERADEFAULTSENSORALGORITHMS_LEDDETECTOR
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERADEFAULTSENSORALGORITHMS_LEDDETECTOR}
    cameras_default_sensor_leddetector_algorithm.cpp)

#
# Create the Camera Default Sensor LED Detector Algorithm
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_cameradefaultsensoralgorithm_leddetector SHARED
            ${ARGOS3_SOURCES_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERADEFAULTSENSORALGORITHMS_LEDDETECTOR})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_cameradefaultsensoralgorithm_leddetector
                      argos3plugin_${ARGOS_BUILD_FOR}_prototype)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_PROTOTYPE_SIMULATOR_SENSORS_CAMERADEFAULTSENSORALGORITHMS_LEDDETECTOR} 
        DESTINATION include/argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor_algorithms/leddetector)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_cameradefaultsensoralgorithm_leddetector
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
