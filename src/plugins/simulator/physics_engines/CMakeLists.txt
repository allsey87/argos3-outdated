add_subdirectory(dynamics2d)

find_package(Bullet 2.81)

if(BULLET_FOUND)
  add_subdirectory(dynamics3d)
  message(STATUS "The Dynamics3D plugin will use the Bullet Physics Engine")
else(BULLET_FOUND)
  message(STATUS "Couldn't find the Bullet Physics Engine")
endif(BULLET_FOUND)

