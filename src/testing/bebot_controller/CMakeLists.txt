add_library(bebot_controller MODULE
   src/bebot_controller.cpp
   src/bebot_controller.h)

target_link_libraries(bebot_controller
   argos3core_${ARGOS_BUILD_FOR}
   argos3plugin_${ARGOS_BUILD_FOR}_prototype)

