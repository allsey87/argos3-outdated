add_library(radio_testing_loop_functions MODULE
   radio_testing_loop_functions.cpp
   radio_testing_loop_functions.h)

target_link_libraries(radio_testing_loop_functions
   argos3plugin_${ARGOS_BUILD_FOR}_prototype)

