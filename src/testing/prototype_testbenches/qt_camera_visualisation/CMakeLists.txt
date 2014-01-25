if(ARGOS_BUILD_FOR_SIMULATOR)
  include(ARGoSCheckQTOpenGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

qt4_wrap_cpp(ARGOS3_MOC_SOURCES_PROTOTYPE_CAMERA_USER_FUNCTIONS
             prototype_camera_user_functions.h
             prototype_camera_window.h)

if(ARGOS_BUILD_FOR_SIMULATOR AND ARGOS_COMPILE_QTOPENGL)
  add_library(prototype_camera_loop_functions MODULE
      prototype_camera_user_functions.h
      prototype_camera_user_functions.cpp
      prototype_camera_window.h
      prototype_camera_window.cpp
      ${ARGOS3_MOC_SOURCES_PROTOTYPE_CAMERA_USER_FUNCTIONS})
  target_link_libraries(prototype_camera_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_prototype)
endif(ARGOS_BUILD_FOR_SIMULATOR AND ARGOS_COMPILE_QTOPENGL)

