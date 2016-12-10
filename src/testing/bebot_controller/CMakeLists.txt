# Find dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(bebot_controller MODULE
   src/bebot_controller.cpp
   src/bebot_controller.h
   src/block.cc
   src/block.h
   src/block_detector.cc
   src/block_detector.h
   src/block_tracker.cc
   src/block_tracker.h
   src/hungarian_solver.cc
   src/hungarian_solver.h
   src/pid_controller.h
   src/structure_analyser.cc
   src/structure_analyser.h
   src/structure.h
   src/tag.cc
   src/tag.h
   src/target.cc
   src/target.h)

target_link_libraries(bebot_controller
   argos3core_${ARGOS_BUILD_FOR}
   argos3plugin_${ARGOS_BUILD_FOR}_prototype
   ${OpenCV_LIBS})

