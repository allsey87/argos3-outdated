if(ARGOS_BUILD_FOR_SIMULATOR)
  include(ARGoSCheckQTOpenGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

qt4_wrap_cpp(ARGOS3_MOC_SOURCES_CAMERA_USER_FUNCTIONS
             camera_user_functions.h
             camera_window.h)

if(ARGOS_BUILD_FOR_SIMULATOR AND ARGOS_COMPILE_QTOPENGL)
  add_library(camera_loop_functions MODULE
      camera_user_functions.h
      camera_user_functions.cpp
      camera_window.h
      camera_window.cpp
      ${ARGOS3_MOC_SOURCES_CAMERA_USER_FUNCTIONS})
  target_link_libraries(camera_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_prototype)
endif(ARGOS_BUILD_FOR_SIMULATOR AND ARGOS_COMPILE_QTOPENGL)

