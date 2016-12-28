add_library(srocs_loop_functions MODULE
   srocs_loop_functions.cpp
   srocs_loop_functions.h)

target_link_libraries(srocs_loop_functions
   argos3plugin_${ARGOS_BUILD_FOR}_prototype)

