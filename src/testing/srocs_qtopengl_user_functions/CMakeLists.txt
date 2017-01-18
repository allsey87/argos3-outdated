if(ARGOS_BUILD_FOR_SIMULATOR)
  include(ARGoSCheckQTOpenGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

qt4_wrap_cpp(ARGOS3_MOC_SOURCES_SROCS_QTOPENGL_USER_FUNCTIONS
             srocs_qtopengl_user_functions.h
             qt_robot_user_interface_widget.h)

if(ARGOS_BUILD_FOR_SIMULATOR AND ARGOS_COMPILE_QTOPENGL)
  add_library(srocs_qtopengl_user_functions MODULE
      srocs_qtopengl_user_functions.h
      srocs_qtopengl_user_functions.cpp
      qt_robot_user_interface_widget.h
      qt_robot_user_interface_widget.cpp
      ${ARGOS3_MOC_SOURCES_SROCS_QTOPENGL_USER_FUNCTIONS})
  target_link_libraries(srocs_qtopengl_user_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_prototype)
endif(ARGOS_BUILD_FOR_SIMULATOR AND ARGOS_COMPILE_QTOPENGL)

